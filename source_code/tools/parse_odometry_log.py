#!/usr/bin/env python3
"""Parser y analizador del log de odometria del DragonBot2.

Lee la salida de texto del volcado SRAM (menu config -> DEBUG -> volcado) y:

  - Extrae los campos del log.
  - Imprime un resumen (duracion, distancia, paso entre muestras, rumbo, slip,
    bias, deriva).
  - Grafica la trayectoria x/y, el rumbo, las velocidades y el offset de linea
    (si esta disponible).

Formatos aceptados
------------------
Log ligero (SRAM_TYPE_ODOMETRY), una muestra cada LOG_DISTANCE_MM (10 mm en el
perfil normal):
  t,pos_x,pos_y,angle,linear_speed,angular_speed,line_offset,slip

Log completo (SRAM_TYPE_ODOMETRY_DEBUG), una muestra cada LOG_DISTANCE_MM
(5 mm en el perfil debug):
  t,left_speed,right_speed,angular_speed,left_mm,right_mm,pos_x,pos_y,angle,
  linear_speed,encoder_omega,encoder_v,gyro_rate,gyro_bias,slip,dt,line_offset,
  var_x,var_y,var_theta

El volcado del menu imprime tambien bloques de control (ligero y completo);
este parser los ignora y procesa solo los bloques de odometria. Si hay varios
bloques de odometria (p. ej. ligero y completo en el mismo fichero), se
analizan por separado. Tambien acepta el formato antiguo, que repetia
`clave=valor` en cada linea.

Uso:
  python3 parse_odometry_log.py odom_log.txt
  python3 parse_odometry_log.py odom_log.txt --csv odom_clean.csv
  python3 parse_odometry_log.py odom_log.txt --no-plot
"""

import argparse
import math
import os
import sys
from collections import defaultdict


# Campos numericos que puede contener un log de odometria.
FIELDS = [
    "t", "left_speed", "right_speed", "angular_speed", "left_mm", "right_mm",
    "pos_x", "pos_y", "angle", "linear_speed", "encoder_omega", "encoder_v",
    "gyro_rate", "gyro_bias", "slip", "dt", "line_offset", "var_x", "var_y",
    "var_theta",
]

# Campos minimos para aceptar un bloque como log de odometria.
REQUIRED = ("pos_x", "pos_y", "angle")


def _split(line):
    return [c.strip() for c in line.split(",")]


def _header_kind(line):
    """Clasifica una linea de cabecera CSV.

    Devuelve (tipo, columnas), donde tipo es "odometry", "control", "other" o
    None si la linea no es una cabecera.
    """
    cols = _split(line)
    if not cols or cols[0] != "t":
        return None, cols
    if all(req in cols for req in REQUIRED):
        return "odometry", cols
    if "ideal_linear_speed" in cols:
        return "control", cols
    return "other", cols


def _parse_key_value(line):
    """Parsea el formato antiguo (clave=valor,clave=valor,...). None si falla."""
    record = {}
    for token in line.split(","):
        if "=" not in token:
            continue
        key, _, value = token.partition("=")
        try:
            record[key.strip()] = float(value.strip())
        except ValueError:
            return None
    return record


def parse(path):
    """Devuelve una lista de bloques de odometria.

    Cada bloque es un dict con dos claves:
      - "header": lista de columnas (None para el formato clave=valor).
      - "data":   dict {campo: [valores]}.
    """
    datasets = []
    current = None
    block = None
    skipped = 0

    with open(path, "r", errors="replace") as fh:
        for raw in fh:
            line = raw.strip()
            if not line:
                continue

            kind, cols = _header_kind(line)
            if kind is not None:
                block = kind
                if kind == "odometry":
                    current = {"header": cols, "data": defaultdict(list)}
                    datasets.append(current)
                else:
                    # Bloques de control u otros: se ignoran hasta la proxima
                    # cabecera de odometria.
                    current = None
                continue

            if "=" in line:
                # Formato antiguo clave=valor: un unico bloque sin cabecera.
                if block != "kv":
                    current = {"header": None, "data": defaultdict(list)}
                    datasets.append(current)
                    block = "kv"
                record = _parse_key_value(line)
            elif block == "odometry" and current is not None:
                values = _split(line)
                if len(values) != len(current["header"]):
                    skipped += 1
                    continue
                try:
                    record = {k: float(v) for k, v in zip(current["header"], values)}
                except ValueError:
                    skipped += 1
                    continue
            else:
                # Linea de un bloque de control/otro: se descarta sin contar
                # como error.
                continue

            if record is None or not all(req in record for req in REQUIRED):
                skipped += 1
                continue

            for field in FIELDS:
                if field in record:
                    current["data"][field].append(record[field])

    if skipped:
        print(f"(ignoradas {skipped} lineas sin formato de odometria)", file=sys.stderr)

    return [ds for ds in datasets if ds["data"].get("t")]


def _kind_label(dataset):
    keys = dataset["data"].keys()
    if "left_mm" in keys or "encoder_omega" in keys or "gyro_bias" in keys:
        return "completo"
    return "ligero"


def _dt(data, i):
    """Intervalo (s) hasta la muestra i, usando dt o los timestamps."""
    if i <= 0:
        return 0.0
    dt = data.get("dt")
    if dt and i < len(dt) and dt[i] > 0:
        return dt[i]
    t = data["t"]
    return (t[i] - t[i - 1]) / 1000.0


def summarize(dataset):
    data = dataset["data"]
    n = len(data.get("t", []))
    if n == 0:
        print("No se han encontrado registros de odometria.")
        return

    t = data["t"]
    duration_s = (t[-1] - t[0]) / 1000.0 if n > 1 else 0.0

    # Distancia recorrida: longitud de la trayectoria si hay posiciones (mas
    # robusta ante huecos temporales, p. ej. el retardo de arranque); si no,
    # integrando la velocidad lineal.
    distance_mm = None
    distance_src = ""
    if "pos_x" in data and "pos_y" in data:
        distance_mm = 0.0
        for i in range(1, n):
            distance_mm += math.hypot(data["pos_x"][i] - data["pos_x"][i - 1],
                                      data["pos_y"][i] - data["pos_y"][i - 1])
        distance_src = "pos"
    elif "linear_speed" in data:
        distance_mm = 0.0
        for i in range(1, n):
            distance_mm += 0.5 * (data["linear_speed"][i] + data["linear_speed"][i - 1]) * _dt(data, i)
        distance_src = "int v"

    # Distancia recta entre inicio y fin (para estimar error de cierre).
    closure_mm = None
    if "pos_x" in data and "pos_y" in data:
        dx = data["pos_x"][-1] - data["pos_x"][0]
        dy = data["pos_y"][-1] - data["pos_y"][0]
        closure_mm = math.hypot(dx, dy)

    # Paso espacial entre muestras: sirve para comprobar el trigger por
    # distancia (10 mm en normal, 5 mm en debug).
    step_mm = None
    if "pos_x" in data and "pos_y" in data and n > 1:
        steps = sorted(
            math.hypot(data["pos_x"][i] - data["pos_x"][i - 1],
                       data["pos_y"][i] - data["pos_y"][i - 1])
            for i in range(1, n)
        )
        step_mm = steps[len(steps) // 2]
    elif distance_mm is not None and n > 1:
        step_mm = distance_mm / (n - 1)

    angle_deg = (data["angle"][-1] - data["angle"][0]) if "angle" in data else None

    # Integrales de velocidad angular: fusion/gyro y encoders.
    gyro_int = None
    if "angular_speed" in data:
        gyro_int = sum(data["angular_speed"][i] * _dt(data, i) for i in range(1, n))
    enc_int = None
    if "encoder_omega" in data:
        enc_int = sum(data["encoder_omega"][i] * _dt(data, i) for i in range(1, n))

    slip_frac = None
    if "slip" in data and data["slip"]:
        slip_frac = sum(1 for s in data["slip"] if s > 0.5) / len(data["slip"])

    bias = data["gyro_bias"][-1] if data.get("gyro_bias") else None

    dt_mean_ms = None
    if data.get("dt"):
        dt_mean_ms = 1000.0 * (sum(data["dt"]) / len(data["dt"]))
    elif n > 1:
        diffs = sorted(t[i] - t[i - 1] for i in range(1, n) if t[i] > t[i - 1])
        if diffs:
            dt_mean_ms = diffs[len(diffs) // 2]

    print(f"=== Resumen odometria ({_kind_label(dataset)}) ===")
    print(f"  muestras         : {n}")
    print(f"  duracion         : {duration_s:.2f} s")
    if dt_mean_ms is not None:
        print(f"  dt medio         : {dt_mean_ms:.3f} ms")
    if distance_mm is not None:
        print(f"  distancia ({distance_src}): {distance_mm:.1f} mm")
    if step_mm is not None:
        if step_mm > 0:
            print(f"  paso medio       : {step_mm:.2f} mm ({1000.0 / step_mm:.1f} muestras/m)")
        else:
            print(f"  paso medio       : {step_mm:.2f} mm")
    if closure_mm is not None:
        print(f"  cierre inicio-fin: {closure_mm:.1f} mm")
    if angle_deg is not None:
        print(f"  rumbo final      : {angle_deg:.2f} deg")
    if gyro_int is not None:
        label = "integral gyro" if enc_int is not None else "integral w fus"
        print(f"  {label:<17}: {math.degrees(gyro_int):.2f} deg")
    if enc_int is not None:
        print(f"  integral encoder : {math.degrees(enc_int):.2f} deg")
    if slip_frac is not None:
        print(f"  slip (fraccion)  : {slip_frac * 100:.1f} %")
    if bias is not None:
        print(f"  gyro_bias final  : {bias:.4f} rad/s")

    if data.get("var_x"):
        print(f"  var final        : x={data['var_x'][-1]:.1f} "
              f"y={data['var_y'][-1]:.1f} th={data['var_theta'][-1]:.6f}")


def save_csv(dataset, path):
    data = dataset["data"]
    header = dataset["header"]
    if header is None:
        header = [field for field in FIELDS if field in data]

    n = len(data.get("t", []))
    with open(path, "w") as fh:
        fh.write(",".join(header) + "\n")
        for i in range(n):
            row = []
            for field in header:
                values = data.get(field, [])
                row.append(f"{values[i]:.6f}" if i < len(values) else "")
            fh.write(",".join(row) + "\n")
    print(f"CSV guardado en {path} ({n} filas)")


def plot(dataset, suffix=""):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib no disponible: se omite el grafico.", file=sys.stderr)
        return

    data = dataset["data"]
    t = [x / 1000.0 for x in data["t"]]
    has_line = "line_offset" in data
    rows = 3 if has_line else 2

    fig, axes = plt.subplots(rows, 2, figsize=(13, 4.5 * rows))

    # Trayectoria x-y.
    ax = axes[0][0]
    if "pos_x" in data and "pos_y" in data:
        ax.plot(data["pos_x"], data["pos_y"], "-", lw=1)
        ax.plot(data["pos_x"][0], data["pos_y"][0], "go", label="inicio")
        ax.plot(data["pos_x"][-1], data["pos_y"][-1], "ro", label="fin")
        ax.axis("equal")
        ax.legend()
    ax.set_title(f"Trayectoria (mm) - log {_kind_label(dataset)}  +y = izquierda")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.grid(True)

    # Rumbo.
    ax = axes[0][1]
    if "angle" in data:
        ax.plot(t, data["angle"], label="angle (deg)")
        ax.legend()
    ax.set_title("Rumbo")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("grados")
    ax.grid(True)

    # Velocidades.
    ax = axes[1][0]
    if "linear_speed" in data:
        ax.plot(t, data["linear_speed"], label="v fusion (mm/s)")
    if "left_speed" in data:
        ax.plot(t, data["left_speed"], "--", label="v izq (mm/s)", alpha=0.7)
    if "right_speed" in data:
        ax.plot(t, data["right_speed"], "--", label="v der (mm/s)", alpha=0.7)
    if "encoder_v" in data and "linear_speed" not in data:
        ax.plot(t, data["encoder_v"], label="v encoders (mm/s)")
    if any(key in data for key in ("linear_speed", "left_speed", "right_speed", "encoder_v")):
        ax.legend()
    else:
        ax.text(0.5, 0.5, "sin datos de velocidad", ha="center", va="center",
                transform=ax.transAxes)
    ax.set_title("Velocidad lineal")
    ax.set_xlabel("t (s)")
    ax.grid(True)

    # Velocidades angulares + slip.
    ax = axes[1][1]
    if "angular_speed" in data:
        ax.plot(t, data["angular_speed"], label="w fusion (rad/s)")
    if "encoder_omega" in data:
        ax.plot(t, data["encoder_omega"], "--", label="w encoders (rad/s)", alpha=0.7)
    if "gyro_rate" in data and "angular_speed" not in data:
        ax.plot(t, data["gyro_rate"], label="gyro (rad/s)")
    if data.get("slip"):
        for i, value in enumerate(data["slip"]):
            if value > 0.5:
                ax.axvline(t[i], color="red", alpha=0.05)
    if any(key in data for key in ("angular_speed", "encoder_omega", "gyro_rate")):
        ax.legend()
    else:
        ax.text(0.5, 0.5, "sin datos angulares", ha="center", va="center",
                transform=ax.transAxes)
    ax.set_title("Velocidad angular (rojo = slip)")
    ax.set_xlabel("t (s)")
    ax.grid(True)

    if has_line:
        ax = axes[2][0]
        ax.plot(t, data["line_offset"], label="line_offset")
        ax.set_title("Sensor de linea")
        ax.set_xlabel("t (s)")
        ax.set_ylabel("offset")
        ax.grid(True)
        ax.legend()

        ax = axes[2][1]
        ax.set_title("Slip")
        ax.set_xlabel("t (s)")
        if data.get("slip"):
            ax.step(t, data["slip"], where="post", label="slip")
            ax.set_ylim(-0.2, 1.2)
            ax.grid(True)
            ax.legend()
        else:
            ax.axis("off")
    fig.tight_layout()
    out = f"odometry_analysis{suffix}.png"
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"Grafica guardada en {out}")


def main():
    parser = argparse.ArgumentParser(description="Analiza el log de odometria del DragonBot2.")
    parser.add_argument("log", help="fichero de texto con el volcado del log")
    parser.add_argument("--csv", help="exportar los datos limpios a CSV")
    parser.add_argument("--no-plot", action="store_true", help="no generar grafica")
    args = parser.parse_args()

    datasets = parse(args.log)
    if not datasets:
        print("No se han encontrado registros de odometria.")
        return 1

    multiple = len(datasets) > 1
    for index, dataset in enumerate(datasets):
        if multiple:
            print(f"\n--- bloque de odometria {index + 1}/{len(datasets)} ---")
        summarize(dataset)

        if args.csv:
            path = args.csv
            if multiple:
                root, ext = os.path.splitext(args.csv)
                path = f"{root}_{index + 1}{ext or '.csv'}"
            save_csv(dataset, path)

        if not args.no_plot:
            plot(dataset, suffix=f"_{index + 1}" if multiple else "")

    return 0


if __name__ == "__main__":
    sys.exit(main())
