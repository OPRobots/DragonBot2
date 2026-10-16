#!/usr/bin/env python3
"""Analizador y predictor de ganancias del lazo de linea del DragonBot2.

Lee el volcado del log de control (SRAM_TYPE_CONTROL; menu -> DEBUG) y:

  1. Recupera las ganancias kp/ki/kd que produjeron la run (regresion exacta,
     porque el firmware calcula u = kp*e + ki*sum(e) + kd*(e - e_{k-1})).
  2. Calcula metricas de calidad: error lateral (e_RMS, bias), seguimiento de
     velocidad, chatter del PWM y una medida espectral de "resonancia" que sirve
     de proxy del margen de estabilidad (independiente de cualquier modelo).
  3. Predice un ajuste fino: a partir de la resonancia medida recomienda el kp
     que la lleva al objetivo, sin necesidad de identificar la planta.

Por que la resonancia y no un modelo de planta
----------------------------------------------
El lazo se observa en lazo cerrado y sin excitacion externa, asi que la planta
NO es identificable de forma fiable a partir de una sola run (los estimadores
salen sesgados: al cambiar kp, la "ganancia del actuador" estimada cambia
entre -5 y -13 rad/s/V). En cambio, la FORMA del espectro de error si es
interpretable:

    e = S(s) * d          (S = sensibilidad = 1/(1+PC), d = perturbacion de pista)

El pico de |S| en la banda del lazo es un indicador clasico del margen de fase:
un lazo mal amortiguado muestra un pico prominente. Definimos

    resdB = 10*log10( max PSD(e) en [6,25] Hz  /  media PSD(e) en [0.3,4] Hz )

- resdB muy negativo  -> lazo sobreamortiguado / poco ganado (e_LF grande).
- resdB cerca de 0    -> lazo al borde de oscilar.

IMPORTANTE - sensor RECORTADO (linea en el borde del array): si el error
satura en +-1000 el lazo es no lineal, el error real es mayor que el medido y
- peor - al crecer el error de BAJA frecuencia el ratio resdB BAJA (el
denominador crece) => parece "mejor margen" justo cuando el seguimiento
empeora. En ese regimen resdB NO es fiable: la herramienta lo marca como
PIERDE LINEA y recomienda SUBIR kp (el problema es subviraje / velocidad alta,
no falta de margen). El recorte se mide como la fraccion de |e| >= 999.

En el barrido real (SPEED_TEST, 2000) se midio que resdB sube ~11 dB por cada
2x de kp (11 dB/octava). De ahi la prediccion:

    kp_nuevo = kp_actual * 2 ** ((resdB_objetivo - resdB_medido) / 11)

Objetivo por defecto: resdB <= -4 dB (margen comodo). El chatter (|dPWM|) se
usa como criterio secundario para el kd.

Uso
---
  python3 line_control_tuner.py report  <log> [<log> ...] [--plot]
  python3 line_control_tuner.py suggest <log> [<log> ...]
  python3 line_control_tuner.py sweep   <log> [<log> ...]
  python3 line_control_tuner.py scale   --ref-kp 0.005 --ref-kd 0.03 \
                                        --ref-v 2000 --speeds 2000,2500,3000 [--emit-c]
  python3 line_control_tuner.py plot    <log> [-o salida.png]

Notas
-----
- Pensado para runs en el MISMO circuito (la metrica depende de la pista).
- Los logs de odometria (sin angular_voltage) tambien se aceptan, pero sin
  recuperar ganancias.
- Requiere numpy; scipy es necesario para el analisis espectral y las graficas.
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
# Constantes del analisis
# ---------------------------------------------------------------------------

# Campos numericos del log de control (sram_print_control_log).
CONTROL_FIELDS = [
    "t", "target_linear_speed", "ideal_linear_speed", "measured_linear_speed",
    "left_measured_linear_speed", "right_measured_linear_speed",
    "ideal_angular_speed", "measured_angular_speed",
    "pwm_left", "pwm_right", "angular_voltage", "line_offset", "battery_voltage",
]

MOTORES_SATURATION_PWM = 1000  # include/constants.h

# Bandas del analisis espectral (Hz).
RES_LF_HZ = (0.3, 4.0)   # banda de seguimiento (perturbacion de pista)
RES_HF_HZ = (6.0, 25.0)  # banda donde asoma la resonancia del lazo

# Objetivo de margen: resdB <= TARGET -> lazo bien amortiguado.
DEFAULT_TARGET_RES_DB = -4.0
DEFAULT_RES_MAX_DB = -1.0

# Regla empirica (SPEED_TEST, 2000 mm/s): resdB sube ~11 dB por 2x de kp.
DEFAULT_SLOPE_DB_PER_OCT = 11.0

# Umbrales de chatter (rms de |dPWM| por rueda, PWM sobre 1049).
# Calibrados con el barrido real: kd=0.03 -> ~90-110 ; kd=0.05 -> ~170.
DEFAULT_CHATTER_HI = 140.0
DEFAULT_CHATTER_LO = 70.0

# Recorte del sensor de linea: |e| en el tope = la linea esta en el BORDE del
# array (error real > medido, lazo no lineal). Con el error recortado el ratio
# resdB deja de ser un proxy valido del margen (ver nota en suggest).
SENSOR_CLIP = 999.0
CLIP_MAX_FRAC = 0.02

# Cortes de segmento: saltos de tiempo > GAP_MS separan runs distintas.
GAP_MS = 100.0


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


def _sos_lp(fc, fs, order=4):
    return sps.butter(order, min(fc, fs / 2.0 * 0.999) / (fs / 2.0), btype="low", output="sos")


def _sos_hp(fc, fs, order=4):
    return sps.butter(order, min(fc, fs / 2.0 * 0.999) / (fs / 2.0), btype="high", output="sos")


def _sos_bp(lo, hi, fs, order=4):
    nyq = fs / 2.0
    lo = max(lo, 1e-6)
    hi = min(hi, nyq * 0.999)
    return sps.butter(order, [lo / nyq, hi / nyq], btype="band", output="sos")


def _filt(sos, x):
    try:
        return sps.sosfiltfilt(sos, x)
    except Exception:
        return x


def band_rms(x, fs, band, kind):
    """RMS de x filtrado en una banda (kind = 'low' | 'band' | 'high')."""
    if not HAVE_SCIPY or len(x) < 32:
        return float(np.sqrt(np.mean(x ** 2)))
    lo, hi = band
    if kind == "low":
        sos = _sos_lp(hi, fs)
    elif kind == "high":
        sos = _sos_hp(lo, fs)
    else:
        sos = _sos_bp(lo, hi, fs)
    return float(np.sqrt(np.mean(_filt(sos, x) ** 2)))


# ---------------------------------------------------------------------------
# Lectura del log
# ---------------------------------------------------------------------------

def read_control_records(path):
    """Devuelve una lista de segmentos (dict campo -> np.array)."""
    recs = []
    for raw in open(path, errors="replace"):
        s = raw.strip()
        if "=" not in s or "line_offset" not in s:
            continue
        rec = {}
        ok = True
        for tok in s.split(","):
            if "=" not in tok:
                continue
            k, _, v = tok.partition("=")
            k = k.strip()
            try:
                rec[k] = float(v)
            except ValueError:
                ok = False
                break
        if ok and "t" in rec and "line_offset" in rec:
            recs.append(rec)

    if not recs:
        raise SystemExit(f"[{os.path.basename(path)}] no contiene registros de control")

    # Segmenta por saltos de tiempo (offset, reinicio, concatenaciones).
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


# ---------------------------------------------------------------------------
# Recuperacion de ganancias y metricas
# ---------------------------------------------------------------------------

def recover_gains(e, u):
    """u = kp*e + ki*sum(e) + kd*(e - e_{k-1})  ->  (kp, ki, kd, rms_resid)."""
    n = len(e)
    de = np.empty(n)
    de[0] = 0.0
    de[1:] = np.diff(e)
    c = np.cumsum(e) / n  # normalizado (mejora el condicionamiento)
    A = np.column_stack([e, c, de])
    coef, *_ = np.linalg.lstsq(A, u, rcond=None)
    kp, ki_n, kd = coef
    resid = u - A @ coef
    return float(kp), float(ki_n / n), float(kd), float(np.sqrt(np.mean(resid ** 2)))


def resonance(el, fs):
    """(resdB, resHz, prom_db) de la senal centrada el. (None,None,None) si no aplica.

    resdB = pico HF / media LF  (proxy del margen, sube con el pico de |S|).
    prom_db = pico HF / mediana HF (cuanto SOBRESALE el pico: una resonancia
    real tiene prom alta; el ruido de cuantizacion es ancho y tiene prom baja).
    """
    if not HAVE_SCIPY or len(el) < 128 or fs <= 2 * RES_HF_HZ[1]:
        return None, None, None
    nps = int(min(2048, len(el)))
    f, P = sps.welch(el, fs=fs, nperseg=nps)
    lf = (f >= RES_LF_HZ[0]) & (f <= RES_LF_HZ[1])
    hf = (f >= RES_HF_HZ[0]) & (f <= RES_HF_HZ[1])
    if not lf.any() or not hf.any():
        return None, None, None
    plf = float(P[lf].mean())
    if plf <= 0:
        return None, None, None
    idx = int(np.argmax(P[hf]))
    pk = float(P[hf][idx])
    med = float(np.median(P[hf]))
    prom = 10 * math.log10(pk / med) if med > 0 else float("inf")
    return float(10 * math.log10(pk / plf)), float(f[hf][idx]), float(prom)


def compute_metrics(d, target_res_db=DEFAULT_TARGET_RES_DB):
    def has(field):
        arr = d.get(field)
        return arr is not None and np.any(np.isfinite(arr))

    e = d["line_offset"]
    n = len(e)
    t = d["t"]
    dt_ms = float(np.median(np.diff(t))) if n > 1 else 1.0
    fs = 1000.0 / dt_ms if dt_ms > 0 else 1000.0
    dur = (t[-1] - t[0]) / 1000.0 if n > 1 else 0.0

    m = {"n": n, "fs": fs, "dur": dur, "dt_ms": dt_ms}
    m["v_target"] = float(np.nanmedian(d["target_linear_speed"])) if has("target_linear_speed") else None
    m["v_ideal"] = float(np.nanmedian(d["ideal_linear_speed"])) if has("ideal_linear_speed") else None
    m["v_meas"] = float(np.nanmedian(d["measured_linear_speed"])) if has("measured_linear_speed") else (
        float(np.nanmedian(d["linear_speed"])) if has("linear_speed") else None)
    if has("ideal_linear_speed") and has("measured_linear_speed"):
        m["v_err"] = float(np.nanmedian(d["ideal_linear_speed"] - d["measured_linear_speed"]))
    else:
        m["v_err"] = None

    m["e_mean"] = float(np.mean(e))
    m["e_rms"] = float(np.sqrt(np.mean(e ** 2)))
    ae = np.abs(e)
    m["e_p95"] = float(np.percentile(ae, 95))
    m["e_max"] = float(ae.max())
    m["e_frac800"] = float(np.mean(ae > 800))
    m["e_clip_frac"] = float(np.mean(ae >= SENSOR_CLIP))

    el = e - np.mean(e)
    m["e_lf_rms"] = band_rms(el, fs, RES_LF_HZ, "low")
    m["e_hf_rms"] = band_rms(el, fs, RES_HF_HZ, "band")
    m["res_db"], m["res_hz"], m["res_prom_db"] = resonance(el, fs)
    # El margen solo es interpretable si (a) la pista excita de verdad al lazo
    # (en una recta bien seguida domina la cuantizacion y resdB enganya) y
    # (b) el error NO recorta el sensor. Si la linea toca el borde del array,
    # el error medido esta limitado, el lazo es no lineal y resdB se distorsiona
    # (ademas, al crecer el error LF el ratio BAJA -> parece "mejor margen").
    m["res_valid"] = (m["res_db"] is not None and m["e_lf_rms"] >= 40.0
                      and n >= 300 and m["e_clip_frac"] < CLIP_MAX_FRAC)

    # Chatter del comando y del PWM (según campos disponibles).
    if has("angular_voltage"):
        u = d["angular_voltage"]
        du = np.abs(np.diff(u))
        m["u_rms"] = float(np.sqrt(np.nanmean(u ** 2)))
        m["du_rms"] = float(np.sqrt(np.nanmean(du ** 2))) if du.size else 0.0
    else:
        m["u_rms"] = None
        m["du_rms"] = None

    if has("pwm_left") and has("pwm_right"):
        pl = d["pwm_left"]
        pr = d["pwm_right"]
        dpwm = (np.abs(np.diff(pl)) + np.abs(np.diff(pr))) / 2.0
        m["dpwm_rms"] = float(np.sqrt(np.nanmean(dpwm ** 2))) if dpwm.size else 0.0
        m["dpwm_p95"] = float(np.nanpercentile(dpwm, 95)) if dpwm.size else 0.0
        m["sat_frac"] = float(np.mean((np.abs(pl) >= MOTORES_SATURATION_PWM) |
                                      (np.abs(pr) >= MOTORES_SATURATION_PWM)))
    else:
        m["dpwm_rms"] = None
        m["dpwm_p95"] = None
        m["sat_frac"] = None

    # Ganancias usadas (si hay comando).
    if has("angular_voltage"):
        kp, ki, kd, rr = recover_gains(e, d["angular_voltage"])
        m.update(kp=kp, ki=ki, kd=kd, gain_resid=rr)
    else:
        m.update(kp=None, ki=None, kd=None, gain_resid=None)

    m["feasible"] = (m["res_db"] is not None and m["res_valid"]
                     and m["res_db"] <= target_res_db + 3.0
                     and (m["sat_frac"] is None or m["sat_frac"] < 0.05))
    return m


def predict_kp(kp, res_db, target_db, slope):
    """kp que llevaria resdB al objetivo, segun la regla dB/octava."""
    if kp is None or res_db is None or kp <= 0 or slope <= 0:
        return None
    return float(kp * (2.0 ** ((target_db - res_db) / slope)))


def predict_kd(kd, m, chatter_hi, chatter_lo, target_res_db):
    """Sugerencia de kd (secundaria): sobre todo evita demasiado chatter."""
    if kd is None or m["dpwm_rms"] is None:
        return None, "n/d"
    chat = m["dpwm_rms"]
    if chat >= chatter_hi:
        return kd * 0.7, f"chatter alto ({chat:.0f}): bajar kd"
    if (m["res_db"] is not None and m["res_db"] > target_res_db
            and chat <= chatter_lo):
        return kd * 1.15, f"margen justo y chatter bajo ({chat:.0f}): subir kd un poco"
    return kd, "mantener"


# ---------------------------------------------------------------------------
# Presentacion
# ---------------------------------------------------------------------------

def summarize(m, label):
    def num(x, spec=".0f", pad=""):
        return "n/d" if x is None else format(x, spec)

    lines = []
    lines.append(f"  {label}")
    lines.append(f"    muestras        : {m['n']}   dur {m['dur']:.2f} s   fs {m['fs']:.0f} Hz")
    lines.append(f"    velocidad       : ideal {num(m['v_ideal'])}  medida {num(m['v_meas'])}  "
                 f"err {num(m['v_err'], '+.1f')} mm/s")
    if m["kp"] is not None:
        lines.append(f"    ganancias usadas: kp={m['kp']:.5f}  ki={m['ki']:.2e}  kd={m['kd']:.5f}"
                     f"   (resid {m['gain_resid']:.3f})")
    lines.append(f"    error lateral   : mean {m['e_mean']:+.0f}  rms {m['e_rms']:.0f}  "
                 f"p95 {m['e_p95']:.0f}  max {m['e_max']:.0f}  >800 {100*m['e_frac800']:.1f}%")
    lines.append(f"    desglose e      : LF(0.3-4Hz) {m['e_lf_rms']:.0f}   HF(6-25Hz) {m['e_hf_rms']:.0f}")
    clip = 100 * m["e_clip_frac"]
    tag_clip = "  ⚠ PIERDE LINEA" if m["e_clip_frac"] >= CLIP_MAX_FRAC else ""
    lines.append(f"    recorte sensor  : |e|>=999 en {clip:.1f}% del tiempo{tag_clip}")
    if m["res_db"] is not None:
        if m["e_clip_frac"] >= CLIP_MAX_FRAC:
            tag = "no fiable (sensor recortado)"
        elif not m["res_valid"]:
            tag = "no fiable (poca excitacion de pista)"
        elif m["res_db"] > DEFAULT_RES_MAX_DB:
            tag = "OSCILA"
        else:
            tag = "ok"
        lines.append(f"    resonancia      : {m['res_db']:+.1f} dB @ {m['res_hz']:.1f} Hz  "
                     f"(prom {m['res_prom_db']:+.1f} dB)  ({tag})")
    if m["u_rms"] is not None:
        lines.append(f"    comando angular : u_rms {m['u_rms']:.2f} V   |du|_rms {m['du_rms']:.3f}")
    if m["dpwm_rms"] is not None:
        lines.append(f"    chatter PWM     : |dPWM| rms {m['dpwm_rms']:.0f}  p95 {m['dpwm_p95']:.0f}"
                     f"   sat {100*m['sat_frac']:.1f}%")
    return "\n".join(lines)


HEADER = ("fichero", "v", "kp", "kd", "e_rms", "e_LF", "resdB", "kHz", "clip%", "dPWM", "veredicto")


def row_for(name, m, res_max):
    clip = m.get("e_clip_frac", 0.0) or 0.0
    if clip >= CLIP_MAX_FRAC:
        verdict = "PIERDE"
    elif m["res_db"] is None or not m["res_valid"]:
        verdict = "?"
    elif m["res_db"] > res_max:
        verdict = "OSCILA"
    elif m["kp"] is not None and m["e_lf_rms"] > 0.9 * m["e_rms"]:
        verdict = "FLOJO"
    else:
        verdict = "ok"
    v = f"{m['v_ideal']:.0f}" if m["v_ideal"] is not None else "n/d"
    dpwm = f"{m['dpwm_rms']:.0f}" if m["dpwm_rms"] is not None else "n/d"
    return (os.path.basename(name), v,
            fmt(m["kp"], 5), fmt(m["kd"], 4), f"{m['e_rms']:.0f}", f"{m['e_lf_rms']:.0f}",
            fmt(m["res_db"], 1), fmt(m["res_hz"], 1), f"{100*clip:.0f}", dpwm, verdict)


def print_table(rows):
    widths = [max(len(str(r[i])) for r in rows) for i in range(len(rows[0]))]
    for i, r in enumerate(rows):
        cells = [str(c).ljust(widths[j]) for j, c in enumerate(r)]
        print("  " + "  ".join(cells))
        if i == 0:
            print("  " + "  ".join("-" * widths[j] for j in range(len(r))))


# ---------------------------------------------------------------------------
# Comandos
# ---------------------------------------------------------------------------

def load_all(paths):
    runs = []
    for p in paths:
        segs = read_control_records(p)
        # analiza el segmento mas largo; avisa si hay varios.
        segs.sort(key=len, reverse=True)
        if len(segs) > 1:
            print(f"  [aviso] {os.path.basename(p)}: {len(segs)} segmentos; "
                  f"se analiza el mayor ({len(segs[0])} de {sum(len(s) for s in segs)} muestras)",
                  file=sys.stderr)
        runs.append((p, segs[0]))
    return runs


def cmd_report(a):
    runs = load_all(a.inputs)
    rows = [HEADER]
    for p, seg in runs:
        m = compute_metrics(seg, a.target_res)
        print(summarize(m, os.path.basename(p)))
        print()
        rows.append(row_for(p, m, a.res_max))
    print("=== Comparativa ===")
    print_table(rows)
    if a.plot:
        os.makedirs(a.plot, exist_ok=True)
        for p, seg in runs:
            out = os.path.join(a.plot, os.path.splitext(os.path.basename(p))[0] + ".png")
            try:
                save_plot(p, seg, out)
            except Exception as ex:  # pragma: no cover
                print(f"  [aviso] no se pudo graficar {p}: {ex}", file=sys.stderr)


def cmd_suggest(a):
    runs = load_all(a.inputs)
    for p, seg in runs:
        m = compute_metrics(seg, a.target_res)
        print(summarize(m, os.path.basename(p)))
        if m["kp"] is None or m["res_db"] is None:
            print("    -> no se puede predecir (falta comando angular o datos suficientes)\n")
            continue
        if m["e_clip_frac"] >= CLIP_MAX_FRAC:
            print(f"    REGIMEN: PIERDE LINEA - el sensor esta en el tope {100*m['e_clip_frac']:.1f}% "
                  f"del tiempo (linea en el borde del array).")
            print("    -> resdB NO es fiable aqui: el error recortado y el LF enorme lo falsean "
                  "(al crecer el error LF baja el ratio y parece 'mejor margen').")
            print("    -> el problema dominante es SUBVIRAJE (o velocidad excesiva), no falta de margen.")
            print(f"    -> prueba SUBIR kp ~25%: {m['kp']:.5f} -> {1.25*m['kp']:.5f} "
                  f"(manteniendo kd) y valora bajar velocidad.\n")
            continue
        if not m["res_valid"]:
            print("    -> resonancia no fiable (pista poco exigente / recta): "
                  "se omite la prediccion. Usa una run con curvas.\n")
            continue
        kp_new = predict_kp(m["kp"], m["res_db"], a.target_res, a.slope)
        if kp_new is not None:
            # limita el salto a [0.5x, 2x] para ser prudente.
            kp_new = max(0.5 * m["kp"], min(2.0 * m["kp"], kp_new))
        kd_new, kd_why = predict_kd(m["kd"], m, a.chatter_hi, a.chatter_lo, a.target_res)
        if kd_new is None:
            kd_new, kd_why = m["kd"], "mantener"
        print(f"    metricas            : e_rms {m['e_rms']:.0f}  resdB {m['res_db']:+.1f}  "
              f"chatter {m['dpwm_rms']:.0f}")
        print(f"    objetivo            : resdB <= {a.target_res:+.1f} dB")
        if kp_new is not None:
            delta = 100 * (kp_new - m["kp"]) / m["kp"]
            print(f"    -> kp: {m['kp']:.5f}  ->  {kp_new:.5f}   ({delta:+.0f}%)")
        print(f"    -> kd: {m['kd']:.5f}  ->  {kd_new:.5f}   ({kd_why})")
        if kp_new is not None:
            kp_round = round(kp_new, 5)
            print(f"    copiar en kinematics.c: kp={kp_round:.5f}, kd={round(kd_new,5):.5f}")
        conf = "alta" if (m["res_valid"] and m["res_prom_db"] >= 8 and m["n"] >= 1000) else "media"
        print(f"    confianza: {conf}  (metodo espectral relativo; validar en pista)\n")


def cmd_sweep(a):
    runs = load_all(a.inputs)
    data = []
    for p, seg in runs:
        m = compute_metrics(seg, a.target_res)
        data.append((p, m))

    print("=== Barrido ===")
    rows = [HEADER]
    for p, m in data:
        rows.append(row_for(p, m, a.res_max))
    print_table(rows)
    print()

    # Coste: e_rms + w_chat * |dPWM|rms ; factible si resdB <= res_max y sat ok.
    def cost(m):
        chat = m["dpwm_rms"] if m["dpwm_rms"] is not None else 0.0
        return m["e_rms"] + a.w_chat * chat

    feas = [(p, m) for p, m in data if m["feasible"]]
    if feas:
        best_p, best = min(feas, key=lambda pm: cost(pm[1]))
        print(f"  Mejor medido (factible): {os.path.basename(best_p)}  "
              f"kp={fmt(best['kp'],5)} kd={fmt(best['kd'],4)}  "
              f"e_rms={best['e_rms']:.0f} resdB={best['res_db']:+.1f}")
        if best["res_db"] is not None:
            kp_next = predict_kp(best["kp"], best["res_db"], a.target_res, a.slope)
            if kp_next is not None and abs(kp_next - best["kp"]) / best["kp"] > 0.03:
                print(f"  Siguiente punto sugerido: kp={kp_next:.5f} kd={best['kd']:.4f}"
                      f"  (para llevar resdB de {best['res_db']:+.1f} a {a.target_res:+.1f} dB)")
            else:
                print("  Esta cerca del objetivo; no hace falta mover mas kp.")
    else:
        best_p, best = min(data, key=lambda pm: (pm[1]["res_db"] if pm[1]["res_db"] is not None else 0))
        print(f"  Ninguna run factible; la de mejor margen es {os.path.basename(best_p)} "
              f"(resdB={best['res_db']:+.1f}). Bajar kp.")


def cmd_scale(a):
    v_ref = a.ref_v
    speeds = [float(x) for x in a.speeds.split(",") if x.strip()]
    print(f"=== Escalado de ganancias  kp/kd  proporcional a 1/v  "
          f"(referencia: {a.ref_kp}/{a.ref_kd} a {v_ref:.0f} mm/s) ===")
    print(f"  {'v (mm/s)':>9}  {'kp':>10}  {'kd':>10}")
    for v in speeds:
        kp = a.ref_kp * v_ref / v
        kd = a.ref_kd * v_ref / v
        print(f"  {v:9.0f}  {kp:10.6f}  {kd:10.6f}")
    if a.emit_c:
        print("\n// Fragmento para include/kinematics.c")
        print("#define KPI_SET(line_kp, line_ki, line_kd)                                  \\")
        print("  {                                                                         \\")
        print("    [KPI_LINEAR] = {.kp = 0.003, .ki = 0.0002, .kd = 0.00},                 \\")
        print("    [KPI_ANGULAR] = {.kp = 0.0, .ki = 0.0, .kd = 0.0},                      \\")
        print("    [KPI_LINE_SENSORS] = {.kp = line_kp, .ki = line_ki, .kd = line_kd},     \\")
        print("  }\n")
        names = ["kpi_test", "kpi_normal", "kpi_medium", "kpi_fast", "kpi_super", "kpi_haki"]
        for i, v in enumerate(speeds):
            nm = names[i] if i < len(names) else f"kpi_v{int(v)}"
            kp = a.ref_kp * v_ref / v
            kd = a.ref_kd * v_ref / v
            print(f"static struct kpi_params {nm}[] = "
                  f"KPI_SET({kp:.6f}, 0.0, {kd:.6f});  /* v={v:.0f} */")


def cmd_plot(a):
    segs = read_control_records(a.input)
    segs.sort(key=len, reverse=True)
    save_plot(a.input, segs[0], a.output)


def save_plot(path, seg, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    m = compute_metrics(seg, DEFAULT_TARGET_RES_DB)
    t = (seg["t"] - seg["t"][0]) / 1000.0
    e = seg["line_offset"]
    u = seg["angular_voltage"]
    fs = m["fs"]

    fig, ax = plt.subplots(4, 1, figsize=(12, 11), sharex=False)

    ax[0].plot(t, e, lw=0.7)
    ax[0].axhline(0, color="k", lw=0.5)
    ax[0].set_title(f"{os.path.basename(path)}  e=line_offset  "
                    f"(kp={fmt(m['kp'],5)}, kd={fmt(m['kd'],4)})")
    ax[0].set_ylabel("line_offset"); ax[0].grid(True)

    ax[1].plot(t, u, lw=0.7, color="C1")
    ax[1].set_title("comando angular u (V)"); ax[1].set_ylabel("V"); ax[1].grid(True)

    ax[2].plot(t, seg["pwm_left"], lw=0.6, label="pwm_left")
    ax[2].plot(t, seg["pwm_right"], lw=0.6, label="pwm_right")
    ax[2].set_title(f"PWM  (|dPWM| rms {m['dpwm_rms']:.0f})")
    ax[2].set_xlabel("t (s)"); ax[2].legend(); ax[2].grid(True)

    if HAVE_SCIPY and len(e) >= 128:
        f, P = sps.welch(e - e.mean(), fs=fs, nperseg=int(min(2048, len(e))))
        ax[3].semilogy(f, P + 1e-12, lw=0.7)
        ax[3].axvspan(*RES_LF_HZ, color="C2", alpha=0.15, label="LF (seguimiento)")
        ax[3].axvspan(*RES_HF_HZ, color="C3", alpha=0.15, label="HF (resonancia)")
        if m["res_db"] is not None:
            ax[3].set_title(f"PSD del error  ->  resonancia {m['res_db']:+.1f} dB "
                            f"@ {m['res_hz']:.1f} Hz  (objetivo <= {DEFAULT_TARGET_RES_DB:+.1f})")
        ax[3].set_xlabel("Hz"); ax[3].set_ylabel("PSD"); ax[3].set_xlim(0, 200)
        ax[3].legend(); ax[3].grid(True, which="both")

    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"  grafica guardada en {out}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_parser():
    p = argparse.ArgumentParser(
        description="Analiza y predice ganancias del lazo de linea del DragonBot2.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    def add_common(sp):
        sp.add_argument("inputs", nargs="+", help="log(s) de control")
        sp.add_argument("--target-res", type=float, default=DEFAULT_TARGET_RES_DB,
                        help=f"resdB objetivo (def {DEFAULT_TARGET_RES_DB})")
        sp.add_argument("--res-max", type=float, default=DEFAULT_RES_MAX_DB,
                        help=f"resdB maximo aceptable (def {DEFAULT_RES_MAX_DB})")
        sp.add_argument("--slope", type=float, default=DEFAULT_SLOPE_DB_PER_OCT,
                        help=f"dB por octava de kp (def {DEFAULT_SLOPE_DB_PER_OCT})")

    r = sub.add_parser("report", help="metricas de una o varias runs")
    add_common(r)
    r.add_argument("--plot", metavar="DIR", default=None,
                   help="guarda una grafica por run en DIR")
    r.set_defaults(func=cmd_report)

    s = sub.add_parser("suggest", help="predice un ajuste fino de kp/kd")
    add_common(s)
    s.add_argument("--chatter-hi", type=float, default=DEFAULT_CHATTER_HI)
    s.add_argument("--chatter-lo", type=float, default=DEFAULT_CHATTER_LO)
    s.set_defaults(func=cmd_suggest)

    w = sub.add_parser("sweep", help="compara un barrido de runs y recomienda el siguiente punto")
    add_common(w)
    w.add_argument("--w-chat", type=float, default=0.0,
                   help="peso del chatter en el coste (def 0)")
    w.set_defaults(func=cmd_sweep)

    c = sub.add_parser("scale", help="tabla de ganancias escaladas 1/v (+ codigo C)")
    c.add_argument("--ref-kp", type=float, required=True)
    c.add_argument("--ref-kd", type=float, required=True)
    c.add_argument("--ref-v", type=float, default=2000.0)
    c.add_argument("--speeds", default="2000,2500,3000,3500,3750,4000")
    c.add_argument("--emit-c", action="store_true")
    c.set_defaults(func=cmd_scale)

    pl = sub.add_parser("plot", help="grafica una run")
    pl.add_argument("input")
    pl.add_argument("-o", "--output", default="line_control_run.png")
    pl.set_defaults(func=cmd_plot)

    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    if not HAVE_SCIPY and args.cmd != "scale":
        print("[aviso] scipy no disponible: analisis espectral limitado", file=sys.stderr)
    args.func(args)


if __name__ == "__main__":
    main()
