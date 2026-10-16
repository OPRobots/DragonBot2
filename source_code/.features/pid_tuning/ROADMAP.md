# ROADMAP — Tuning del control (velocidad lineal + posición de línea) — DragonBot2

> Estado: **documento vivo**. Marcar `[x]` a medida que se completen las tareas.
> Objetivo: ajustar lo mejor posible las **ganancias** del control del robot, por estrategia de
> velocidad, con una metodología de test segura y reproducible.
>
> Se divide en **Parte A — control de velocidad lineal** (prerequisito) y **Parte B — control de
> posición sobre la línea**.
>
> Ver también: `.features/odometry/ROADMAP.md` (estimación de pose, usada aquí como sensor de
> diagnóstico).

> **Decisiones cerradas (D1–D5):**
> **D1** = velocidad base de pruebas = **`SPEED_TEST` (`linear_speed = 2000`)**. Las de carrera se derivan
> después por escalado `∝ 1/v`: `SPEED_NORMAL = 2500`, `SPEED_MEDIUM = 3000`, `SPEED_FAST = 3500`,
> `SPEED_SUPER = 3750`, `SPEED_HAKI = 4000`.
> **D2** = **sin amortiguación por gyro** (`KPI_ANGULAR` se mantiene a 0): en curvas opondría el giro
> necesario y en linefollower no conocemos la pista por anticipado.
> **D3** = `ki = 0` en el lazo de línea.
> **D4** = **sin tuning en runtime** (varias runs y recarga de código).
> **D5** = ganancias **por velocidad**: inferir desde una a las demás y finetunear después (las
> velocidades pueden cambiar).

---

## 1. Objetivos y alcance

- **Correcto**: entender la ley de control actual y qué términos están activos.
- **Seguro**: todos los test mantienen el **seguimiento de línea activo** (o sin traslación) para no
  chocar.
- **Reproducible**: test concretos + logs a 1 kHz + análisis offline.
- **Extensible**: empezar por `SPEED_TEST` (2000), luego replicar/escalar por velocidad.

Fuera de alcance por ahora: control de trayectoria basado en odometría, control de rumbo absoluto.

---

## 2. Estado actual del control

### 2.1 Arquitectura
- `control_loop()` a **1 kHz** (`sys_tick`, slot `CONTROL`).
- Dos lazos lineales que producen un reparto diferencial:
  - **Línea (posición)**: PID sobre `get_line_sensor_position()` (rango **±1000**) →
    `angular_voltage`. **Este es el lazo a tunear.**
  - **Guiñada (KPI_ANGULAR)**: PID sobre la velocidad angular medida por el gyro (`lsm6dsr_get_gyro_z_radps()`).
    **Deshabilitado** por defecto (`angular_correction_enabled = false`) → hoy no actúa.
- Reparto: `voltage_left = linear_voltage + angular_voltage`, `voltage_right = linear_voltage − angular_voltage`.
- Conversión a PWM: `pwm = voltage / battery_voltage * MOTORS_MAX_PWM` (compensación de batería).

### 2.2 Ley de control (código)
```c
line_sensors_error = get_line_sensor_position();          // ±1000
angular_voltage =
    kpi[KPI_ANGULAR].kp*angular_error + ... +
    kpi[KPI_LINE_SENSORS].kp*line_sensors_error +
    kpi[KPI_LINE_SENSORS].ki*sum_line_sensors_error +
    kpi[KPI_LINE_SENSORS].kd*(line_sensors_error - last_line_sensors_error);
```

### 2.3 Ganancias actuales (`kinematics.c`, por estrategia)
| Estrategia | v (mm/s) | `KPI_LINEAR` (velocidad) | `KPI_LINE_SENSORS` (línea) |
|---|---|---|---|
| `SPEED_TEST` (`kpi_test`) | 2000 | 0.003 / 0.0002 / 0 | **0.005 / 0 / 0.03** (referencia) |
| `SPEED_NORMAL` (`kpi_normal`) | 2500 | idem | 0.0040 / 0 / 0.024 |
| `SPEED_MEDIUM` (`kpi_medium`) | 3000 | idem | 0.003333 / 0 / 0.020 |
| `SPEED_FAST` (`kpi_fast`) | 3500 | idem | 0.002857 / 0 / 0.01714 |
| `SPEED_SUPER` (`kpi_super`) | 3750 | idem | 0.002667 / 0 / 0.0160 |
| `SPEED_HAKI` (`kpi_haki`) | 4000 | idem | 0.0025 / 0 / 0.0150 |

> `KPI_ANGULAR` = 0 en todas (D2). Las ganancias de carrera están **escaladas `∝ 1/v`** desde
> `SPEED_TEST` (F2.4 / D5) y quedan **pendientes de finetunear en pista** velocidad por velocidad (F4.1).

### 2.4 Sensor de línea
- **24 sensores** (`LINE_SENSOR_COUNT`), leídos por 3 multiplexores de 8 (`MUX_CHANNEL_COUNT = 8`).
- La posición (`get_line_sensor_position`) se recalcula en el slot `ENCODERS_LINE` (1 kHz), pero el
  **array completo se refresca al ritmo del ADC/DMA+mux** (cada 8 conversiones) → **confirmar el
  periodo real** (retardo del lazo).
- Filtro: `menu_run_get_filter_mode()` → `FILTER_BIGGER` (ventana de sensores según última posición).
- Rango de salida: −1000…+1000 (centro = 0). **El array va ~115 mm por delante del eje de giro** →
  actúa como un **derivador/lead** (amortiguación intrínseca).
- Los `#define KP_LINE_SENSORS` de `config.h` (0.08/0/0.12) **no se usan** (legacy).

### 2.5 Actuador
- **Identificado con T1** (giro en el sitio): **`K ≈ −20 rad/s/V`**, **`τ ≈ 55 ms`** (3 reps).
- Pendiente: **deadband** y comportamiento en **saturación** (`MOTORES_SATURATION_PWM = 1000`).
- ⚠️ En lazo cerrado la ganancia aparente `u→ω` **no** es identificable de forma fiable (sesgo: −5…−13);
  el valor de T1 (lazo abierto) es el válido.

---

## 3. Modelo de la planta y métricas

### 3.1 Modelo (pequeña señal)
```
e  ≈  − v · ∫∫ ω                     (e = error lateral, v = velocidad)
ω  =  G_motor(s) · u                 (u = comando angular; G_motor ≈ K/(τ s + 1))
⇒  P(s) = e/u ≈ − v · K / ( s² · (τ s + 1) )
```
más el **cero de adelanto** por el sensor a distancia `L`: `≈ (L/v)·s + 1` (aporta amortiguación).

Consecuencia: la **ganancia del lazo es proporcional a `v`** → a más velocidad, mismas ganancias dan
**menos margen**. Por eso tiene sentido tunear por estrategia.

### 3.2 Métricas de calidad
- **Error lateral RMS** (`line_offset`).
- **Sobreoscilación** y **tiempo de establecimiento** (test de escalón).
- **Margen de estabilidad** (oscilación sostenida / "limit cycle").
- **Pérdida de línea** (evento crítico).

---

## 4. Test seguros

> Todos mantienen **seguimiento de línea activo**, salvo el de giro en el sitio (no traslada).

| Test | Qué mide | Traslación | Riesgo | Resultado |
|---|---|---|---|---|
| **T1 — Giro en el sitio** | `G_motor` (`K`, `τ`) | No | Ninguno | ✅ Hecho: `K≈−20 rad/s/V`, `τ≈55 ms` |
| **T2 — ID en lazo cerrado** | Planta completa desde runs normales | Sí (seguro) | Ninguno | ❌ No fiable sin excitación (ver §12); sustituido por el método espectral |
| **T3 — Offset lateral al arrancar** | Respuesta del lazo (escalón) | Sí (seguro) | Ninguno | No hecho (innecesario con el barrido A–F) |
| **T4 — Dither en recta** (opcional) | Excitación **conocida** para ID | Sí | Bajo | ⬜ Pendiente → da modelo riguroso |

### T1 — Giro en el sitio (escalón de guiñada)
- Robot en suelo despejado, `v = 0`, con la corrección de línea **desactivada**.
- Aplicar un **escalón de comando angular** `ΔV` durante `T` ms; repetir `N` veces.
- Registrar `ω` (`gyro_rate`) y el comando → obtener `K` (ganancia) y `τ` (63 %).

### T2 — Identificación en lazo cerrado
- Run **normal** a `SPEED_TEST` (2000) siguiendo la línea (rectas + curvas).
- Registrar `line_error`, `u` (comando angular), `pwm_*`, `ω`, `v`.
- Ajustar `G_motor(s)` y validar `P(s)` con `e ≈ −v·∫∫ω`.

### T3 — Offset lateral al arrancar (test B)
- Colocar el robot con un **offset lateral conocido** (p. ej. 20 mm) y **arrancar** con el seguidor
  activo. Convergerá (con `v≠0` sí corrige lateralmente).
- Medir sobreoscilación, tiempo de establecimiento y, si hay, oscilación.

### T4 — Dither (solo si hace falta)
- En un tramo **recto**, inyectar una perturbación pequeña conocida (seno o PRBS) en el comando,
  con el seguidor activo como red de seguridad.

---

## 5. Parte A — Control de velocidad lineal (prerequisito)

**Por qué antes que la línea:** el lazo de línea asume que `v` sigue bien la consigna (la planta de
línea es `∝ v`). Un lazo de velocidad lento u oscilante contamina el de línea. Es un lazo **SISO más
simple** → conviene dejarlo afinado primero.

### 5.1 Estado actual
- `KPI_LINEAR` = `kp 0.003`, `ki 0.0002`, `kd 0` (igual en `kpi_test` y `kpi_run`).
- **Referencia**: `ideal_linear_speed` (rampa generada por `update_ideal_linear_speed()` según
  `accel_hard`/`accel_soft`/`break_accel`).
- **Medición**: `get_measured_linear_speed()` = media de velocidades de encoder (filtradas, LPF α=0.2).
- **Salida**: `linear_voltage` → `pwm` (con compensación de batería).
- Ya se registra en `control_log`: `target/ideal/measured`, izq/der, `pwm`, batería.

### 5.2 Qué caracterizar
- **Respuesta a la rampa de arranque** (0→target): retardo, error de seguimiento, sobreoscilación.
- **Error estacionario** a velocidad constante (`ideal − measured`): recoge fricción/ventilador → aquí
  **sí** tiene sentido `ki` (a diferencia de la línea).
- **Sensibilidad a la batería y a la carga** (ventilador, aceleraciones).

### 5.3 Test (seguros, sin maniobras especiales)
| Test | Qué mide | Traslación |
|---|---|---|
| **S1 — Arranque 0→target** en recta | respuesta de `v` (subida/sobreoscilación/error) | Sí (seguro) |
| **S2 — Run a velocidad constante** (recta/círculo) | error estacionario, rizado | Sí (seguro) |

Ambos forman parte de runs normales: no requieren modos de test nuevos.

### 5.4 Fases
- [x] S0.1 Log de control disponible: el volcado se cambió a `SRAM_TYPE_CONTROL`; incluye
      `target/ideal/measured`, izq/der, `pwm_left/right`, `battery_voltage` (el voltaje de comando se
      deriva de `pwm`). Suficiente para el análisis del lazo de velocidad. ✅
- [x] S1.1 Analizar arranques (S1) de 2–3 runs → estimar retardo y error de seguimiento. ✅
- [x] S2.1 Ajustar `kp` (respuesta), `ki` (error estacionario), `kd` (normalmente 0). ✅ Ya estaba bien
      afinado: **error estacionario ≈ 0.05 %**, **sobreoscilación ≈ 2 %** → se **mantiene** `KPI_LINEAR`
      (`0.003 / 0.0002 / 0`).
- [x] S3.1 Validar en pista y dejar `KPI_LINEAR` afinado (el resultado vale para todas las velocidades). ✅

**Estado:** ✅ (el lazo de velocidad se dejó como estaba; no requería cambios)

---

## 6. Parte B — Fases: posición sobre la línea

### Fase 0 — Preparación: logging + modos de test
- [x] F0.1 **Log para el lazo de línea**: ampliado `control_log_t` con **`angular_voltage`** (comando del
      PID de línea) y **`line_offset`** (error de línea). Con `measured_angular_speed` (ω) y
      `measured_linear_speed` (v) queda el **lazo completo** en una sola traza. ✅
- [x] F0.2 **Modo T1** (giro en el sitio): `CALIBRATE_YAW_STEP` (menú → CALIBRACIÓN). Con el control
      pausado aplica un escalón diferencial (`set_motors_pwm(+100, −100)`) y muestrea el gyro a 1 kHz
      **guardando en SRAM** (`SRAM_TYPE_YAW_STEP`) para poder volcar **después con el robot parado**
      (menú → DEBUG → valor 11). `YAW_STEP_PWM/BASELINE/SAMPLES/REPS` ajustables. ✅
- [ ] F0.3 **Modo T3**: arranque con offset lateral (o inyección de offset virtual en la línea).
- [~] F0.4 **Ganancias en runtime**: **descartado (D4)**. Se itera con varias runs + recarga de código.
- [ ] F0.5 **Anti-windup** del integrador y saturación del comando (relevante sobre todo en Parte A).
- [x] F0.6 **Script de análisis/predicción** `tools/line_control_tuner.py`: recupera las ganancias
      del log (kp/ki/kd, exacto), calcula métricas (e_RMS, LF/HF, chatter, **resonancia espectral**
      como proxy del margen) y **predice** el `kp` que lleva la resonancia al objetivo. Subcomandos
      `report` / `suggest` / `sweep` / `scale` / `plot`. ✅ (ver §12)

**Implicaciones:** `sram.h/.c`, `control.c`, `sensors.c`, `menu_configs.c`/`calibrations.c`, `tools/`.
**Estado:** ⬜

### Fase 1 — Identificación de la planta ✅ (parcial: actuador)
- [x] F1.1 **T1** (giro en el sitio) → **`K ≈ −20 rad/s/V`**, **`τ ≈ 55 ms`** (consistente en 3 reps). ✅
- [~] F1.2/F1.3/F1.4 El **modelo lineal continuo no reprodujo** la estabilidad observada (el `kd`
      sobre `e` cuantizado aporta amortiguación no lineal) → se pasó a **barrido empírico en pista**.

**Estado:** ✅ (identificación del actuador; modelo completo descartado)

### Fase 2 — Ajuste de ganancias (empírico en pista) ✅
- [x] Barrido de `kp`/`kd` con el log de control, comparando `e_RMS`, oscilación y chatter del PWM.
- [x] Resultado (mismo circuito, SPEED_TEST), ya automatizado con el script (§12):

| Caso | kp | kd | e_RMS | e_LF | resdB | |ΔPWM| med |
|---|---|---|---|---|---|---|
| A | 0.0025 | 0.03 | 275 | 254 | −14.5 | 52 |
| B | 0.0035 | 0.03 | 215 | 179 | −8.8 | 48 |
| E | **0.005** | **0.03** | **157** | 119 | **−4.0** | 53 |
| F | 0.007 | 0.03 | 149 | 85 | **+3.4** ⚠️ | 61 |
| C/D | — | 0.05 | — | — | −15.6 / −10.7 | 91–96 (❌ ruido, sin mejora) |

> `resdB` = pico de |S| en 6–25 Hz respecto a la banda de seguimiento (proxy del margen, ver §12).
> Sube **≈ +11 dB por cada 2× de `kp`** → es la base de la predicción automática.

- **Adoptado: `kp = 0.005`, `ki = 0`, `kd = 0.03`** (F oscila; `kd=0.05` duplica el chatter sin mejorar).
- [x] F2.4 Escalado `∝ 1/v` para las demás velocidades.

**Estado:** ✅

### Fase 3 — Validación en pista ✅
- [x] F3.1 Ganancias implementadas en `kinematics.c` (`kpi_test` = referencia).
- [x] F3.3 Runs en pista (A–F) → `e_RMS` bajó de 275 a 157, sin pérdida de línea.
- [x] F3.4 Iterado hasta el punto dulce.
- [~] F3.2 (T3 offset lateral) no necesario dados los resultados.

**Estado:** ✅

### Fase 4 — Extensión por velocidad ✅ (por escalado)
- [x] F4.1 Ganancias por estrategia implementadas en `kinematics.c` (`kpi_normal`…`kpi_haki`),
      **escaladas `∝ 1/v`** desde la referencia de `SPEED_TEST` (D5). Pendiente de finetunear en pista.
- [ ] F4.2 (Opcional) ganancias adaptativas según velocidad.

**Estado:** ✅ (escalado); finetuning por velocidad pendiente.


---

## 7. Qué registrar y cómo

- **Formato**: nuevo log `SRAM_TYPE_CONTROL_TUNE` (volcado igual que el de odometría).
- **Volcado**: menú config → DEBUG → volcado; capturar por serie a fichero
  (`cat /dev/ttyACM0 > log.txt`, o `--filter log2file`; ver notas de odometría).
- **Recordatorio**: cada **arranque de carrera formatea la SRAM** → una maniobra por arranque,
  volcar antes de la siguiente.

Campos mínimos por muestra: `line_error`, `u` (comando angular), `pwm_left/right`, `ω`, `v`,
`heading`/posición lateral.

---

## 8. Reparto de trabajo

### Solo el usuario (físico/criterio)
- Preparar la pista (rectas, curva amplia, curvas cerradas).
- Ejecutar T1/T2/T3 en la pista y volcar los logs.
- Decidir la métrica objetivo (tiempo vs estabilidad) y confirmar la velocidad base (`SPEED_TEST`, 2000).

### Agente (offline, desde logs)
- Identificar `G_motor` y la planta.
- Optimizar `kp/ki/kd` y proponer valores.
- Analizar sobreoscilación/oscilación y comparar runs.

### Firmware (pendiente, todo opcional)
- ✅ Ya hecho: log de control ampliado (`angular_voltage`, `line_offset`) y modo **T1** (`SRAM_TYPE_YAW_STEP`).
- [ ] **T3** (offset lateral al arrancar) — innecesario hasta ahora.
- [ ] **Anti-windup** del integrador + saturación del comando.
- [ ] **T4 (dither)** — excitación conocida en recta para identificación rigurosa del modelo (ver §4).

---

## 9. Decisiones

- [x] **D1** Velocidad base de pruebas: **`SPEED_TEST` (`linear_speed = 2000`)** (más prudente). Las de
      carrera (`2500 / 3000 / 3500 / 3750 / 4000`) se derivan por escalado `∝ 1/v`. ✅
- [x] **D2** **Sin amortiguación por gyro**: `KPI_ANGULAR` se queda a 0. En curvas opondría el giro
      necesario y en linefollower no hay información anticipada de la pista. ✅
- [x] **D3** `ki = 0` en el lazo de línea (sí se usará `ki` en el lazo de velocidad, Parte A). ✅
- [x] **D4** **Sin tuning en runtime**: varias runs + recarga de código (menú limpio). ✅
- [x] **D5** Ganancias **por velocidad**: inferir desde la referencia (`SPEED_TEST`) a las demás y finetunear después. En
      concreto: **`KPI_LINEAR` (velocidad) es ≈ independiente de `v`** → **una sola tabla para todas**;
      **`KPI_LINE_SENSORS` (línea) escala `∝ v`** → se derivan de una referencia por **escalado `∝ 1/v`**
      (gain scheduling), o tabla por velocidad. Así, cambiar las velocidades no obliga a re-tunear todo. ✅
- [x] **D6** **Orden**: primero **Parte A (velocidad lineal)** y luego **Parte B (línea)**. ✅

---

## 10. Matriz de implicaciones por fase

| Fase | Archivos tocados | Riesgo | Reversible | Depende de |
|------|------------------|--------|------------|------------|
| 0 | `sram.*`, `control.c`, `menu_configs.c`, `tools/*` | Medio | Sí | D4 |
| 1 | (solo logs) | Bajo | Sí | F0 |
| 2 | `tools/*` (offline) | Bajo | Sí | F1 |
| 3 | `kinematics.c` | Bajo | Sí | F2 |
| 4 | `kinematics.c` | Bajo | Sí | F3 |

---

## 11. Referencias de código

- `src/control.c` — `control_loop()`, ley de control, reparto diferencial, logging.
- `src/kinematics.c`, `include/kinematics.h` — `kpi_test`/`kpi_run`, ganancias por estrategia.
- `src/sensors.c`, `include/sensors.h` — `get_line_sensor_position()`, `line_sensors_update_position()`,
  filtro y geometría (24 sensores, mux).
- `src/menu_run.c`, `include/menu_run.h` — estrategias de velocidad y modo de filtro.
- `include/config.h` — defines legacy de KP/KI/KD de línea (no usados).
- `include/sram.h`, `src/sram.c` — esquema y volcado de logs.
- `src/main.c` — secuencia de arranque y activación de la corrección de línea.
- `tools/line_control_tuner.py` — análisis y predicción de ganancias del lazo de línea (§12).
- `tools/parse_odometry_log.py` — parser/analizador del log de odometría.

---

## 12. Herramienta de análisis/predicción (`tools/line_control_tuner.py`)

**Qué hace** (a partir del volcado del log de control, sin tocar el firmware):
- **Recupera las ganancias** de la run (kp/ki/kd) por regresión exacta (`u = kp·e + ki·Σe + kd·Δe`).
- **Métricas**: `e_RMS`, `e_LF` (0.3–4 Hz, seguimiento), `e_HF` (6–25 Hz), bias, p95, saturación de
  motores, **recorte del sensor** (fracción de `|e| ≥ 999`, línea en el borde del array), error de
  velocidad y **chatter** del PWM (`|ΔPWM|`).
- **Margen (proxy)**: `resdB = 10·log10( pico PSD(e) en 6–25 Hz / media PSD(e) en 0.3–4 Hz )`.
  Es el pico de `|S|` (sensibilidad) → indicador clásico de margen de fase.
- **Predicción**: `kp_nuevo = kp · 2^((resdB_obj − resdB)/SLOPE)`, con `SLOPE ≈ 11 dB/octava`
  (medido) y `resdB_obj ≈ −4 dB`. Limita el salto a ±2× por prudencia.

**Límite crítico — sensor recortado**: `resdB` es `pico HF / media LF`. Si el error satura el sensor
(línea en el borde, `|e| ≥ 999` en > 2 % del tiempo), el error real es mayor que el medido y, al crecer
el error de **baja** frecuencia, el **denominador crece → `resdB` BAJA**: *parece* que mejora el margen
justo cuando el seguimiento empeora → la predicción empujaría `kp` **hacia abajo**, arruinando la pista.
Por eso, con recorte > 2 %, la herramienta marca **PIERDE LINEA** (no fiable) y recomienda **subir
`kp`** (el problema es subviraje/velocidad, no margen). Caso SPEED_FAST: `kp 0.00286 → 0.00238` (lo que
sugirió la versión antigua) **subió** `e_RMS` de 469 a 527 y el recorte de 11 % a 16 %.

**Por qué NO se identifica la planta** (y por qué la herramienta no la necesita): el lazo sólo se
observa **en cerrado y sin excitación externa**, así que un modelo `u→e` sale **sesgado** (al cambiar
`kp`, la ganancia de actuador estimada varía entre −5 y −13 rad/s/V). El **espectro** sí es
interpretable sin modelo. Para un modelo riguroso haría falta **excitación** (dither, test T4).

**Uso**
```bash
python3 tools/line_control_tuner.py report  <log...> [--plot DIR]   # métricas + comparativa
python3 tools/line_control_tuner.py suggest <log...>               # ajuste fino de kp/kd
python3 tools/line_control_tuner.py sweep   <log...>               # barrido: mejor + siguiente punto
python3 tools/line_control_tuner.py scale --ref-kp 0.005 --ref-kd 0.03 --ref-v 2000 \
                                          --speeds 2000,2500,3000,3500,3750,4000 [--emit-c]
python3 tools/line_control_tuner.py plot <log> -o run.png
```

**Flujo recomendado por velocidad**: corre a la velocidad objetivo en el mismo circuito → `suggest`
`<log>` → aplica `kp`/`kd` sugeridos → repite; usa `scale` para la tabla inicial `∝ 1/v`. Si `suggest`
dice **PIERDE LINEA**, **sube `kp`** (o baja velocidad); no confíes en `resdB` en ese régimen.

**Validez**: la métrica es **relativa** → comparar siempre en el **mismo circuito** y con curvas (en
rectas con poco error domina la cuantización y el script lo marca como *no fiable*). Y solo es
interpretable **sin recorte del sensor**: un run que pierde la línea (> 2 % en el tope) no sirve para
predecir `kp` por esta vía.

---

## 13. Notas de teoría (recordatorio)

- La planta lleva **`s²`** (doble integrador) → un **P** puro no estabiliza bien; se necesita **amortiguación**.
- La amortiguación viene de: **`kd`** (derivada del error, sobre señal cuantizada → ruidosa) y del
  **lead natural del sensor** (`L/v`). Por eso `kd` suele ser **pequeño y delicado**.
- **`ki`** suele ser **0** en seguidores de línea (el error estacionario en la línea no es un problema;
  y en curvas sostenidas provoca **windup**).
- A misma ganancia, **subir `v` reduce el margen** en el lazo de **línea** (planta `∝ v`) → re-tunear por
  velocidad o **escalar `∝ 1/v`** desde una referencia.
- **El escalado `∝ 1/v` NO mantiene el error constante**: en curva, el error estacionario es
  `e = v/(R·K·kp)`; con `kp ∝ 1/v` queda `e ∝ v²`. Por eso a `SPEED_FAST` (3500) el `e_RMS` es
  `157·(3500/2000)² ≈ 480` (medido 469) y la línea se sale. Mantener el error constante exigiría
  `kp ∝ v` (lazo inestable). El compromiso real es **subir `kp` hasta justo antes de oscilar**.
- El lazo de **velocidad** es ≈ **independiente de `v`** (motor ~lineal) → **una sola tabla** basta; vigilar
  solo la **saturación** a alta velocidad.
