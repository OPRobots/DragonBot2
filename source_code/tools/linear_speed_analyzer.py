#!/usr/bin/env python3
"""Analizador del lazo de velocidad lineal del DragonBot2.

Lee el volcado del log de control (SRAM_TYPE_CONTROL; menu -> DEBUG) y estudia
la cadena encoder -> PI de velocidad -> PWM, con foco en:

  1. Regimen permanente: error, rizado de la velocidad medida y tension de
     comando (a partir de pwm_avg = (pwm_left + pwm_right)/2, que cancela la
     parte angular del reparto diferencial).
  2. Arranque/aceleracion: rampa de ideal_linear_speed, retardo hasta que el
     robot empieza a moverse (deadband de friccion) y error de seguimiento.
  3. Feedforward: ajuste de  V = k0 + kv*v + ka*a  desde los datos de pista.
  4. Ruido del encoder: reconstruye la velocidad cruda (pre-filtro EMA) y
     compara estimadores alternativos (EMA/MA) en ruido vs retardo.

Modelo de la medida (src/encoders.c):
    new_speed = diff_ticks * MM_PER_TICK * CONTROL_FREQUENCY_HZ        (mm/s)
    speed     = 0.2*new_speed + 0.8*speed_prev                          (EMA a=0.2)

Invertir la EMA da la serie cruda cuantizada:
    raw[i] = (speed[i] - (1-a)*speed[i-1]) / a

Hardware (BOM): encoder AS5145B (4096 cuentas/vuelta en cuadratura x4),
driver MP6615 (DC con escobillas, sensa corriente), bateria 3S.

Uso
---
  python3 linear_speed_analyzer.py report <log> [<log> ...]
  python3 linear_speed_analyzer.py ramp   <log> [<log> ...]
  python3 linear_speed_analyzer.py ff     <log> [<log> ...] [--min-v 200]
  python3 linear_speed_analyzer.py noise  <log> [--target 2000] [--alpha 0.2]
  python3 linear_speed_analyzer.py plot   <log> [-o salida.png]

Notas
-----
- Pensado para runs en el MISMO circuito (las metricas dependen de la pista).
- Acepta logs con o sin los campos angular_voltage/line_offset.
- Requiere numpy; scipy es opcional (mejora el analisis espectral y las graficas).
- Constantes del firmware replicadas aqui: si cambian en src/, actualizarlas.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

try:
    from scipy import signal as sps

    HAVE_SCIPY = True
except Exception:  # pragma: no cover
    sps = None
    HAVE_SCIPY = False


# ---------------------------------------------------------------------------
# Constantes del firmware (mantener sincronizadas)
# ---------------------------------------------------------------------------

MICROMETERS_PER_TICK = 70.43493572812115          # include/config.h
MICROMETERS_PER_MILLIMETER = 1000.0               # include/constants.h
MM_PER_TICK = MICROMETERS_PER_TICK / MICROMETERS_PER_MILLIMETER

CONTROL_FREQUENCY_HZ = 1000.0                     # include/constants.h
EMA_ALPHA = 0.2                                   # src/encoders.c

MOTORS_MAX_PWM = 1049                             # include/constants.h
MOTORES_SATURATION_PWM = 1000                     # include/constants.h

# Campos numericos del log de control (sram_print_control_log).
CONTROL_FIELDS = [
    "t", "target_linear_speed", "ideal_linear_speed", "measured_linear_speed",
    "left_measured_linear_speed", "right_measured_linear_speed",
    "ideal_angular_speed", "measured_angular_speed",
    "pwm_left", "pwm_right", "angular_voltage", "line_offset", "battery_voltage",
]

# Minimo necesario para considerar una linea un registro de control util.
REQUIRED = ("t", "ideal_linear_speed", "measured_linear_speed", "pwm_left", "pwm_right")

# Los saltos de tiempo > GAP_MS (o dt<=0) separan runs distintas / reinicios.
GAP_MS = 50.0

# Banda alta usada para medir el rizado de la velocidad (Hz).
NOISE_HF_HZ = 20.0


# ---------------------------------------------------------------------------
# Utilidades
# ---------------------------------------------------------------------------

def fmt(x, nd=3):
    """Formatea un numero (o 'n/d')."""
    if x is None:
        return "n/d"
    try:
        if isinstance(x, float) and math.isnan(x):
            return "n/d"
    except TypeError:
        pass
    return f"{x:.{nd}f}"


def voltage(pwm_avg, battery):
    """Tension de comando equivalente (V) a partir del PWM medio."""
    return pwm_avg / MOTORS_MAX_PWM * battery


# ---------------------------------------------------------------------------
# Lectura del log
# ---------------------------------------------------------------------------

def read_control_records(path):
    """Devuelve una lista de segmentos (dict campo -> np.array)."""
    recs = []
    for raw in open(path, errors="replace"):
        s = raw.strip()
        if "=" not in s or "ideal_linear_speed" not in s:
            continue
        rec = {}
        ok = True
        for tok in s.split(","):
            if "=" not in tok:
                continue
            k, _, v = tok.partition("=")
            try:
                rec[k.strip()] = float(v)
            except ValueError:
                ok = False
                break
        if ok and all(req in rec for req in REQUIRED):
            recs.append(rec)

    if not recs:
        raise SystemExit(f"[{os.path.basename(path)}] no contiene registros de control")

    segments = []
    cur = [recs[0]]
    for r in recs[1:]:
        dt = r["t"] - cur[-1]["t"]
        if dt > GAP_MS or dt <= 0:
            segments.append(cur)
            cur = [r]
        else:
            cur.append(r)
    segments.append(cur)

    out = []
    for seg in segments:
        d = {}
        for f in CONTROL_FIELDS:
            d[f] = np.asarray([r.get(f, np.nan) for r in seg], dtype=float)
        out.append(d)
    return out


def pwm_avg(d):
    """PWM medio (cancela la parte angular). nan si faltan campos."""
    if np.all(np.isnan(d.get("pwm_left", np.array([np.nan])))):
        return np.full(len(d["t"]), np.nan)
    return 0.5 * (d["pwm_left"] + d["pwm_right"])


def has(d, field):
    arr = d.get(field)
    return arr is not None and np.any(np.isfinite(arr))


# ---------------------------------------------------------------------------
# Metricas por segmento
# ---------------------------------------------------------------------------

def segment_metrics(d):
    """Metricas de regimen permanente + rampa de un segmento."""
    t = d["t"]
    n = len(t)
    il = d["ideal_linear_speed"]
    ml = d["measured_linear_speed"]
    tl = d["target_linear_speed"] if has(d, "target_linear_speed") else np.full(n, np.nan)
    pv = pwm_avg(d)
    batt = d["battery_voltage"] if has(d, "battery_voltage") else np.full(n, np.nan)

    m = {"n": n}
    m["dur"] = (t[-1] - t[0]) / 1000.0 if n > 1 else 0.0
    m["target"] = float(np.nanmedian(tl)) if has(d, "target_linear_speed") else float(np.nanmax(il))

    dt = np.diff(t)
    m["contiguous"] = bool(dt.size == 0 or np.all((dt > 0) & (dt <= GAP_MS)))
    m["dt_ms"] = float(np.median(dt)) if dt.size else 1.0

    # --- Rampa de arranque (ideal creciente) ---
    d_il = np.diff(il)
    ramp = np.where(np.isfinite(d_il) & (d_il > 0.5))[0]
    m["ramp"] = ramp.size > 10
    if m["ramp"]:
        a0, a1 = int(ramp[0]), int(ramp[-1])
        m["ramp_ms"] = (a1 - a0) * m["dt_ms"]
        e = il[a0:a1 + 1] - ml[a0:a1 + 1]
        m["ramp_err_mean"] = float(np.nanmean(e))
        m["ramp_err_p95"] = float(np.nanpercentile(np.abs(e), 95))
        m["ramp_err_max"] = float(np.nanmax(np.abs(e)))
        seg_ml = ml[a0:a1 + 1]
        tgt = m["target"] if np.isfinite(m["target"]) else float(np.nanmax(il))
        thr_motion = 2.0
        thr_5pct = 0.05 * tgt
        idx_motion = np.where(seg_ml > thr_motion)[0]
        idx_5pct = np.where(seg_ml >= thr_5pct)[0]
        m["t_motion_ms"] = float(idx_motion[0]) * m["dt_ms"] if idx_motion.size else None
        m["t_5pct_ms"] = float(idx_5pct[0]) * m["dt_ms"] if idx_5pct.size else None
        m["V_ramp_start"] = float(voltage(pv[a0], batt[a0])) if np.isfinite(pv[a0]) else None
        m["V_ramp_end"] = float(voltage(pv[a1], batt[a1])) if np.isfinite(pv[a1]) else None
    else:
        for k in ("ramp_ms", "ramp_err_mean", "ramp_err_p95", "ramp_err_max",
                  "t_motion_ms", "t_5pct_ms", "V_ramp_start", "V_ramp_end"):
            m[k] = None

    # --- Regimen permanente (ideal == target y target>umbral) ---
    tgt = m["target"]
    if np.isfinite(tgt) and tgt > 500:
        ss = (np.abs(il - tgt) < 1.0)
    else:
        ss = np.zeros(n, dtype=bool)
    m["ss_n"] = int(np.count_nonzero(ss))
    if m["ss_n"] > 30:
        err = il - ml
        m["ss_err_mean"] = float(np.nanmean(err[ss]))
        m["ss_err_std"] = float(np.nanstd(err[ss]))
        m["meas_mean"] = float(np.nanmean(ml[ss]))
        m["meas_std"] = float(np.nanstd(ml[ss]))
        m["pwm_avg_mean"] = float(np.nanmean(pv[ss]))
        m["pwm_avg_std"] = float(np.nanstd(pv[ss]))
        m["V_mean"] = float(np.nanmean(voltage(pv[ss], batt[ss]))) if np.any(np.isfinite(batt[ss])) else None
        m["overshoot_pct"] = (float(np.nanmax(ml[ss])) - tgt) / tgt * 100.0
    else:
        for k in ("ss_err_mean", "ss_err_std", "meas_mean", "meas_std",
                  "pwm_avg_mean", "pwm_avg_std", "V_mean", "overshoot_pct"):
            m[k] = None
    return m


# ---------------------------------------------------------------------------
# Reconstruccion de la velocidad cruda y estimadores
# ---------------------------------------------------------------------------

def recover_raw(meas, alpha=EMA_ALPHA):
    """Invierte la EMA para recuperar la velocidad cruda (cuantizada)."""
    raw = np.empty_like(meas)
    raw[0] = meas[0]
    raw[1:] = (meas[1:] - (1.0 - alpha) * meas[:-1]) / alpha
    return raw


def ema_causal(x, alpha):
    y = np.empty_like(x)
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = alpha * x[i] + (1.0 - alpha) * y[i - 1]
    return y


def ma_causal(x, n):
    """Media movil causal: y[i] = mean(x[i-n+1 .. i])."""
    c = np.cumsum(np.insert(x, 0, 0.0))
    y = np.empty_like(x)
    for i in range(len(x)):
        j = max(0, i - n + 1)
        y[i] = (c[i + 1] - c[j]) / (i - j + 1)
    return y


def moving_average(x, n):
    """Media movil centrada sin artefactos de borde (pad reflectante, n impar)."""
    if n <= 1 or len(x) < 3:
        return x.copy()
    n = min(n, len(x) - 1)
    if n % 2 == 0:
        n += 1
    k = np.ones(n) / n
    pad = n // 2
    xp = np.pad(x, pad, mode="reflect")
    y = np.convolve(xp, k, mode="valid")
    return y[:len(x)]


def hf_residual(x, n=101):
    """Contenido de alta frecuencia = x - su media movil, recortando bordes."""
    if len(x) < n + 4:
        return x - np.mean(x)
    y = x[n:] - moving_average(x, n)[n:]
    return y


def estimators(raw):
    """Lista de (nombre, group_delay_ms, salida) para comparar ruido vs retardo."""
    dt_ms = 1000.0 / CONTROL_FREQUENCY_HZ
    out = []
    for a in (0.30, 0.20, 0.15, 0.10, 0.05):
        out.append((f"EMA a={a:.2f}", (1.0 - a) / a * dt_ms, ema_causal(raw, a)))
    out.append(("EMA2 a=0.20", 2.0 * 0.8 / 0.2 * dt_ms, ema_causal(ema_causal(raw, 0.20), 0.20)))
    out.append(("EMA2 a=0.15", 2.0 * 0.85 / 0.15 * dt_ms, ema_causal(ema_causal(raw, 0.15), 0.15)))
    for n in (3, 5, 8, 11, 16):
        out.append((f"MA N={n}", (n - 1) / 2.0 * dt_ms, ma_causal(raw, n)))
    return out


# ---------------------------------------------------------------------------
# Feedforward: V = k0 + kv*v + ka*a
# ---------------------------------------------------------------------------

def ff_fit(segments, min_v=200.0):
    """Ajusta V = k0 + kv*v + ka*a sobre los datos disponibles.

    La aceleracion comandada se estima como d(ideal)/dt dentro de cada segmento.
    Devuelve (coef, rms, n, v_absmax).
    """
    V_all, v_all, a_all = [], [], []
    for d in segments:
        if len(d["t"]) < 3:
            continue
        il = d["ideal_linear_speed"]
        ml = d["measured_linear_speed"]
        pv = pwm_avg(d)
        batt = d["battery_voltage"] if has(d, "battery_voltage") else np.full_like(il, np.nan)
        if not np.all(np.isfinite(pv)):
            continue
        a = np.gradient(il) * CONTROL_FREQUENCY_HZ  # mm/s^2 (paso de 1 ms)
        mask = (il > min_v) & np.isfinite(ml) & np.isfinite(pv)
        V_all.append(voltage(pv[mask], batt[mask]))
        v_all.append(ml[mask])
        a_all.append(a[mask])
    if not V_all:
        return None
    V = np.concatenate(V_all)
    v = np.concatenate(v_all)
    a = np.concatenate(a_all)
    ok = np.isfinite(V) & np.isfinite(v) & np.isfinite(a)
    V, v, a = V[ok], v[ok], a[ok]
    if V.size < 20:
        return None
    A = np.column_stack([v, a, np.ones_like(v)])
    coef, *_ = np.linalg.lstsq(A, V, rcond=None)
    rms = float(np.sqrt(np.mean((A @ coef - V) ** 2)))
    return coef, rms, int(V.size), float(np.max(np.abs(v)))


# ---------------------------------------------------------------------------
# Comandos
# ---------------------------------------------------------------------------

def cmd_report(a):
    for path in a.inputs:
        segs = read_control_records(path)
        pairs = [(d, segment_metrics(d)) for d in segs]
        pairs = [(d, m) for (d, m) in pairs if m["n"] >= a.min_samples]
        print("=" * 92)
        print(f"{os.path.basename(path)}  ({len(segs)} segmento(s), {len(pairs)} analizado(s))")
        if a.plot:
            os.makedirs(a.plot, exist_ok=True)
        for i, (d, r) in enumerate(pairs):
            print("-" * 92)
            print(f" seg {i}: n={r['n']} dur={r['dur']:.2f}s target={fmt(r['target'],0)} mm/s")
            if r["ramp"]:
                print(f"   rampa {fmt(r['ramp_ms'],0)} ms | e_mean {fmt(r['ramp_err_mean'],1)} "
                      f"p95 {fmt(r['ramp_err_p95'],1)} max {fmt(r['ramp_err_max'],1)} mm/s")
                print(f"   arranque: t_motion {fmt(r['t_motion_ms'],0)} ms | "
                      f"t_5% {fmt(r['t_5pct_ms'],0)} ms | V {fmt(r['V_ramp_start'],2)} -> "
                      f"{fmt(r['V_ramp_end'],2)} V")
            if r["ss_n"] > 30:
                print(f"   SS    e_mean {fmt(r['ss_err_mean'],2)} std {fmt(r['ss_err_std'],1)} mm/s | "
                      f"v_med {fmt(r['meas_mean'],1)} (std {fmt(r['meas_std'],1)}) | "
                      f"pwm_avg {fmt(r['pwm_avg_mean'],1)} (std {fmt(r['pwm_avg_std'],1)}) | "
                      f"V {fmt(r['V_mean'],3)} | over {fmt(r['overshoot_pct'],2)}%")
            if a.plot:
                out = os.path.join(a.plot, f"{os.path.splitext(os.path.basename(path))[0]}_seg{i}.png")
                save_plot(path, d, out)


def cmd_ramp(a):
    for path in a.inputs:
        segs = read_control_records(path)
        print("=" * 92)
        print(f"{os.path.basename(path)}")
        any_ramp = False
        for i, d in enumerate(segs):
            m = segment_metrics(d)
            if not m["ramp"]:
                continue
            any_ramp = True
            print(f" rampa seg {i}: {fmt(m['ramp_ms'],0)} ms  target {fmt(m['target'],0)}  "
                  f"V {fmt(m['V_ramp_start'],2)}->{fmt(m['V_ramp_end'],2)} V")
            print(f"   t_motion {fmt(m['t_motion_ms'],0)} ms | t_5% {fmt(m['t_5pct_ms'],0)} ms | "
                  f"e_mean {fmt(m['ramp_err_mean'],1)} p95 {fmt(m['ramp_err_p95'],1)} "
                  f"max {fmt(m['ramp_err_max'],1)} mm/s")
            if a.verbose:
                _print_launch_trace(d)
        if not any_ramp:
            print("  (sin rampa de arranque en este log)")


def _print_launch_trace(d, step=4, n=60):
    """Traza los primeros n ms desde el inicio de la rampa."""
    il = d["ideal_linear_speed"]
    ml = d["measured_linear_speed"]
    pv = pwm_avg(d)
    d_il = np.diff(il)
    r = np.where(np.isfinite(d_il) & (d_il > 0.5))[0]
    if not r.size:
        return
    a0 = int(r[0])
    print("   t+ms  ideal  measured  err   pwm_avg")
    for k in range(0, n, step):
        i = a0 + k
        if i >= len(il):
            break
        print(f"   {k:4d} {il[i]:6.0f} {ml[i]:9.2f} {il[i]-ml[i]:6.1f} {pv[i]:7.1f}")


def cmd_ff(a):
    for path in a.inputs:
        segs = read_control_records(path)
        res = ff_fit(segs, a.min_v)
        print("=" * 92)
        print(f"{os.path.basename(path)}")
        if res is None:
            print("  (datos insuficientes para el ajuste)")
            continue
        (kv, ka, k0), rms, n, vmax = res
        print(f"  ajuste V = {kv:.6f}*v + {ka:.6f}*a + {k0:.4f}   "
              f"(rms {rms:.3f} V, n={n}, v_max {vmax:.0f} mm/s)")
        print(f"  -> kv ~ {kv:.6f} V/(mm/s)  |  k0 ~ {k0:.3f} V  |  ka ~ {ka:.6f} V/(mm/s^2)")
        print(f"  -> V(2000)={kv*2000+k0:.3f} V   V(3500)={kv*3500+k0:.3f} V")
        if kv <= 0:
            print("  AVISO: kv<=0 -> este log no aisla el lazo lineal (curvas/frenadas "
                  "hacen que v no dependa solo del comando). Usa una recta a target fijo.")
        elif vmax > 2500:
            print("  nota: a alta v el modelo lineal pierde validez (friccion/back-EMF); "
                  "compara con la curva V(v) de abajo.")
        # Curva V(v) empirica por bins (solo tramos casi sin aceleracion).
        _print_v_curve(segs, a.min_v)


def _print_v_curve(segments, min_v, bin_mm=400):
    vv, VV = [], []
    for d in segments:
        if len(d["t"]) < 3:
            continue
        il = d["ideal_linear_speed"]
        ml = d["measured_linear_speed"]
        pv = pwm_avg(d)
        batt = d["battery_voltage"] if has(d, "battery_voltage") else np.full_like(il, np.nan)
        a = np.abs(np.gradient(il)) * CONTROL_FREQUENCY_HZ
        mask = (il > min_v) & (a < 2000.0) & np.isfinite(ml) & np.isfinite(pv)
        vv.append(ml[mask]); VV.append(voltage(pv[mask], batt[mask]))
    if not vv:
        return
    v = np.concatenate(vv); V = np.concatenate(VV)
    print("  curva V(v) (|a|<2000 mm/s^2):")
    lo = 0
    while lo * bin_mm < np.nanmax(v):
        hi = lo * bin_mm + bin_mm
        m = (v >= lo * bin_mm) & (v < hi)
        if m.sum() > 5:
            print(f"    v~{lo*bin_mm+bin_mm//2:5d}: V={np.nanmean(V[m]):.3f} V  (n={m.sum()})")
        lo += 1


def cmd_noise(a):
    for path in a.inputs:
        segs = read_control_records(path)
        best, best_n = None, 0
        for d in segs:
            il = d["ideal_linear_speed"]
            tgt = np.nanmedian(d["target_linear_speed"]) if has(d, "target_linear_speed") else np.nan
            if np.isfinite(tgt) and abs(tgt - a.target) > 1:
                continue
            m = (il == tgt)
            if m.sum() > best_n:
                best, best_n = d, int(m.sum())
        print("=" * 92)
        if best is None or best_n < 50:
            print(f"{os.path.basename(path)}: sin tramo de SS a {a.target:.0f} mm/s")
            continue
        d = best
        il = d["ideal_linear_speed"]
        ml = d["measured_linear_speed"]
        tgt = np.nanmedian(d["target_linear_speed"]) if has(d, "target_linear_speed") else np.nan
        idx = np.where(il == tgt)[0]
        idx = idx[5:]  # margen de asentamiento
        meas = ml[idx]
        if a.alpha is not None:
            alpha = a.alpha
        else:
            alpha = EMA_ALPHA
        raw = recover_raw(meas, alpha)
        ticks = raw / (MM_PER_TICK * CONTROL_FREQUENCY_HZ)
        resid = ticks - np.round(ticks)
        print(f"{os.path.basename(path)}  target {tgt:.0f} mm/s  n={len(meas)}")
        print(f"  mm/tick={MM_PER_TICK*1000:.3f} um  ticks/ms medio={np.nanmean(ticks):.2f}  "
              f"paso cuantizacion={MM_PER_TICK*CONTROL_FREQUENCY_HZ:.1f} mm/s "
              f"({100*MM_PER_TICK*CONTROL_FREQUENCY_HZ/tgt:.2f}%/tick)")
        print(f"  reconstruccion: residual std={np.nanstd(resid):.4f} tick "
              f"(~0 -> entero; ~0.25 -> medio tick, coherente con (izq+der)/2)")
        print(f"  raw  std={np.nanstd(raw):.1f} mm/s   medido(EMA) std={np.nanstd(meas):.1f} mm/s")
        print(f"  {'estimador':<16}{'retardo(ms)':>12}{'std_out':>10}{'hf_rms':>10}")
        for name, lag, y in estimators(raw):
            hf = hf_residual(y, 101)
            print(f"  {name:<16}{lag:12.1f}{np.nanstd(y):10.1f}{np.nanstd(hf):10.1f}")
        print(f"  (hf = contenido >~{NOISE_HF_HZ:.0f} Hz, via resta de media movil 100 ms)")
        print("  nota: parte del 'ruido' es rizado MECANICO/INL del encoder (no removible "
              "sin retardo).")


def cmd_plot(a):
    segs = read_control_records(a.input)
    segs.sort(key=lambda d: len(d["t"]), reverse=True)
    save_plot(a.input, segs[0], a.output)


def save_plot(path, seg, out):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib no disponible: se omite el grafico.", file=sys.stderr)
        return

    t = (seg["t"] - seg["t"][0]) / 1000.0
    il = seg["ideal_linear_speed"]
    ml = seg["measured_linear_speed"]
    pv = pwm_avg(seg)
    err = il - ml
    m = segment_metrics(seg)

    fig, ax = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    ax[0].plot(t, il, label="ideal", lw=1.0)
    ax[0].plot(t, ml, label="measured", lw=0.7, alpha=0.8)
    ax[0].set_title(f"{os.path.basename(path)}  v ideal vs medida  "
                    f"(SS err {fmt(m['ss_err_mean'],1)}, std {fmt(m['meas_std'],1)} mm/s)")
    ax[0].set_ylabel("mm/s"); ax[0].legend(); ax[0].grid(True)

    ax[1].plot(t, err, lw=0.7, color="C3")
    ax[1].axhline(0, color="k", lw=0.5)
    ax[1].set_title("error ideal - measured (mm/s)"); ax[1].grid(True)

    ax[2].plot(t, pv, lw=0.7, color="C2")
    ax[2].set_title("pwm_avg (comando lineal)"); ax[2].set_xlabel("t (s)")
    ax[2].grid(True)

    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"  grafica guardada en {out}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_parser():
    p = argparse.ArgumentParser(
        description="Analiza el lazo de velocidad lineal del DragonBot2.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("report", help="metricas SS + rampa de cada segmento")
    r.add_argument("inputs", nargs="+", help="log(s) de control")
    r.add_argument("--min-samples", type=int, default=100,
                   help="minimo de muestras por segmento (def 100)")
    r.add_argument("--plot", metavar="DIR", default=None,
                   help="guarda una grafica por segmento en DIR")
    r.set_defaults(func=cmd_report)

    rp = sub.add_parser("ramp", help="analiza arranques/aceleraciones")
    rp.add_argument("inputs", nargs="+", help="log(s) de control")
    rp.add_argument("--verbose", action="store_true", help="imprime la traza del arranque")
    rp.set_defaults(func=cmd_ramp)

    f = sub.add_parser("ff", help="ajuste de feedforward V = k0 + kv*v + ka*a")
    f.add_argument("inputs", nargs="+", help="log(s) de control")
    f.add_argument("--min-v", type=float, default=200.0,
                   help="velocidad minima a incluir en el ajuste (def 200 mm/s)")
    f.set_defaults(func=cmd_ff)

    nz = sub.add_parser("noise", help="ruido del encoder y comparativa de estimadores")
    nz.add_argument("inputs", nargs="+", help="log(s) de control")
    nz.add_argument("--target", type=float, default=2000.0,
                    help="velocidad del tramo SS a analizar (def 2000 mm/s)")
    nz.add_argument("--alpha", type=float, default=None,
                    help=f"alpha de la EMA a invertir (def {EMA_ALPHA})")
    nz.set_defaults(func=cmd_noise)

    pl = sub.add_parser("plot", help="grafica una run")
    pl.add_argument("input", help="log de control")
    pl.add_argument("-o", "--output", default="linear_speed_run.png")
    pl.set_defaults(func=cmd_plot)

    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    if not HAVE_SCIPY and args.cmd in ("plot",):
        print("[aviso] scipy no disponible: graficos basicos", file=sys.stderr)
    args.func(args)


if __name__ == "__main__":
    main()
