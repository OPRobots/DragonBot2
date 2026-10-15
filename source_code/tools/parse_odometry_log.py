#!/usr/bin/env python3
"""Parser y analizador del log de odometria del DragonBot2.

Lee la salida de texto del volcado SRAM (menu config -> DEBUG -> volcado de
odometria) o el modo debug en vivo, y:

  - Extrae los campos del log.
  - Imprime un resumen (duracion, distancia, rumbo, slip, bias, deriva).
  - Grafica la trayectoria x/y, el rumbo y las velocidades (si hay matplotlib).

Formato de cada linea del log SRAM (sram_print_odometry_log):
  t=<ms>,left_speed=..,right_speed=..,angular_speed=..,left_mm=..,right_mm=..,
  pos_x=..,pos_y=..,angle=..,linear_speed=..,encoder_omega=..,gyro_bias=..,
  slip=..,dt=..,line_offset=..,var_x=..,var_y=..,var_theta=..

Uso:
  python3 parse_odometry_log.py odom_log.txt
  python3 parse_odometry_log.py odom_log.txt --csv odom_clean.csv
  python3 parse_odometry_log.py odom_log.txt --no-plot
"""

import argparse
import math
import sys
from collections import defaultdict


# Campos numericos esperados del log de odometria.
FIELDS = [
    "t", "left_speed", "right_speed", "angular_speed", "left_mm", "right_mm",
    "pos_x", "pos_y", "angle", "linear_speed", "encoder_omega", "encoder_v",
    "gyro_rate", "gyro_bias", "slip", "dt", "line_offset", "var_x", "var_y",
    "var_theta",
]

# Campos que deben aparecer para aceptar la linea como registro de odometria.
REQUIRED = ("left_speed", "pos_x", "pos_y", "angle")


def parse(path):
    """Devuelve un dict {campo: [valores]} a partir del texto capturado."""
    data = defaultdict(list)
    skipped = 0
    with open(path, "r", errors="replace") as fh:
        for raw in fh:
            line = raw.strip()
            if not line or "=" not in line or "," not in line:
                continue
            if not all(req + "=" in line for req in REQUIRED):
                skipped += 1
                continue

            record = {}
            ok = True
            for token in line.split(","):
                if "=" not in token:
                    continue
                key, _, value = token.partition("=")
                key = key.strip()
                value = value.strip().rstrip(",")
                try:
                    record[key] = float(value)
                except ValueError:
                    ok = False
                    break
            if not ok:
                skipped += 1
                continue

            for field in FIELDS:
                if field in record:
                    data[field].append(record[field])

    if skipped:
        print(f"(ignoradas {skipped} lineas sin formato de odometria)", file=sys.stderr)
    return data


def summarize(data):
    n = len(data.get("t", []))
    if n == 0:
        print("No se han encontrado registros de odometria.")
        return

    t = data["t"]
    duration_s = (t[-1] - t[0]) / 1000.0 if n > 1 else 0.0

    # Distancia recorrida integrando la velocidad lineal fusionada.
    distance_mm = 0.0
    for i in range(1, n):
        dt = data["dt"][i] if data["dt"][i] > 0 else 0.001
        distance_mm += 0.5 * (data["linear_speed"][i] + data["linear_speed"][i - 1]) * dt

    # Distancia recta entre inicio y fin (para estimar error de cierre).
    dx = data["pos_x"][-1] - data["pos_x"][0]
    dy = data["pos_y"][-1] - data["pos_y"][0]
    closure_mm = math.hypot(dx, dy)

    # Deriva de rumbo y comparacion gyro vs encoders.
    angle_deg = data["angle"][-1] - data["angle"][0]
    gyro_int = 0.0
    enc_int = 0.0
    for i in range(1, n):
        dt = data["dt"][i] if data["dt"][i] > 0 else 0.001
        gyro_int += data["angular_speed"][i] * dt
        enc_int += data["encoder_omega"][i] * dt

    slip = data.get("slip", [])
    slip_frac = (sum(1 for s in slip if s > 0.5) / len(slip)) if slip else 0.0

    bias = data.get("gyro_bias", [0.0])

    print("=== Resumen odometria ===")
    print(f"  muestras         : {n}")
    print(f"  duracion         : {duration_s:.2f} s")
    print(f"  dt medio         : {1000.0 * (sum(data['dt']) / n):.3f} ms" if data.get("dt") else "  dt medio         : n/d")
    print(f"  distancia (int v): {distance_mm:.1f} mm")
    print(f"  cierre inicio-fin: {closure_mm:.1f} mm")
    print(f"  rumbo final      : {angle_deg:.2f} deg")
    print(f"  integral gyro    : {math.degrees(gyro_int):.2f} deg")
    print(f"  integral encoder : {math.degrees(enc_int):.2f} deg")
    print(f"  slip (fraccion)  : {slip_frac * 100:.1f} %")
    print(f"  gyro_bias final  : {bias[-1]:.4f} rad/s")

    if data.get("var_x"):
        print(f"  var final        : x={data['var_x'][-1]:.1f} "
              f"y={data['var_y'][-1]:.1f} th={data['var_theta'][-1]:.6f}")


def save_csv(data, path):
    n = len(data.get("t", []))
    with open(path, "w") as fh:
        fh.write(",".join(FIELDS) + "\n")
        for i in range(n):
            row = []
            for field in FIELDS:
                values = data.get(field, [])
                row.append(f"{values[i]:.6f}" if i < len(values) else "")
            fh.write(",".join(row) + "\n")
    print(f"CSV guardado en {path} ({n} filas)")


def plot(data):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib no disponible: se omite el grafico.", file=sys.stderr)
        return

    t = [x / 1000.0 for x in data["t"]]

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))

    # Trayectoria x-y.
    ax = axes[0][0]
    ax.plot(data["pos_x"], data["pos_y"], "-", lw=1)
    ax.plot(data["pos_x"][0], data["pos_y"][0], "go", label="inicio")
    ax.plot(data["pos_x"][-1], data["pos_y"][-1], "ro", label="fin")
    ax.set_title("Trayectoria (mm)  +y = izquierda")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.axis("equal")
    ax.grid(True)
    ax.legend()

    # Rumbo.
    ax = axes[0][1]
    ax.plot(t, data["angle"], label="angle (deg)")
    ax.set_title("Rumbo")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("grados")
    ax.grid(True)

    # Velocidades.
    ax = axes[1][0]
    ax.plot(t, data["linear_speed"], label="v fusion (mm/s)")
    ax.plot(t, data["left_speed"], "--", label="v izq (mm/s)", alpha=0.7)
    ax.plot(t, data["right_speed"], "--", label="v der (mm/s)", alpha=0.7)
    ax.set_title("Velocidad lineal")
    ax.set_xlabel("t (s)")
    ax.grid(True)
    ax.legend()

    # Velocidades angulares + slip.
    ax = axes[1][1]
    ax.plot(t, data["angular_speed"], label="w fusion (rad/s)")
    ax.plot(t, data["encoder_omega"], "--", label="w encoders (rad/s)", alpha=0.7)
    if data.get("slip"):
        slip_t = [t[i] for i in range(len(t)) if data["slip"][i] > 0.5]
        for st in slip_t:
            ax.axvline(st, color="red", alpha=0.05)
    ax.set_title("Velocidad angular (rojo = slip)")
    ax.set_xlabel("t (s)")
    ax.grid(True)
    ax.legend()

    fig.tight_layout()
    out = "odometry_analysis.png"
    fig.savefig(out, dpi=110)
    print(f"Grafica guardada en {out}")


def main():
    parser = argparse.ArgumentParser(description="Analiza el log de odometria del DragonBot2.")
    parser.add_argument("log", help="fichero de texto con el volcado del log")
    parser.add_argument("--csv", help="exportar los datos limpios a CSV")
    parser.add_argument("--no-plot", action="store_true", help="no generar grafica")
    args = parser.parse_args()

    data = parse(args.log)
    summarize(data)
    if args.csv:
        save_csv(data, args.csv)
    if not args.no_plot:
        plot(data)


if __name__ == "__main__":
    main()
