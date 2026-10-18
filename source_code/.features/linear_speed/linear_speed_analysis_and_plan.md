# Control de velocidad lineal — Análisis en profundidad y plan de mejora

> Proyecto: **DragonBot2** (siguelíneas tipo Robotracer, All Japan Robotracer 2027)
> Estado: **borrador de análisis** (a mover a `.features/linear_speed/` en modo build)
> Fecha: 2026-10-16
> Autor: análisis asistido a partir de código, logs de pista y datasheets.

---

## 0. Alcance y decisiones del usuario

**Objetivo:** mejorar al máximo el lazo de velocidad lineal, desde la lectura de encoders hasta el PI, incluida la gestión de aceleraciones; evaluar la conveniencia de un **feedforward**; y reducir el **ruido de la velocidad medida**.

**Decisiones tomadas:**
- **Prioridades:** (1) seguimiento en rampa/aceleración, (2) rechazo de perturbaciones, (3) suavidad/ruido de la velocidad medida.
- **Feedforward:** *primero analizar/estimar y decidir después* (no se compromete implementación aún).
- **Feedforward de par (corriente):** *analizar antes de comprometer* (hay sensado de corriente sin usar).
- **Identificación:** **solo en pista**, con runs normales (sin banco, sin maniobras de excitación nuevas de momento).
- **Consigna:** en Linefollower es **fija** durante la vuelta; en Robotracer se prevé **velocidad adaptativa** (acelerar en rectas, frenar en curvas según radio) → el diseño debe aceptar `v_ideal`/`a_ideal` **dinámicos** desde el principio.
- **Cambios estructurales autorizados (reversibles):** anti-windup + límites de salida; rampa con límite de jerk (S-curve); cambiar el estimador de velocidad; arreglar `accel_soft` y la división entera; **log nuevo** dedicado a tuneo/validación del FF (en vez de ampliar el log de control).

**Hardware confirmado (BOM + datasheets):**

| Elemento | Componente | Dato relevante |
|---|---|---|
| Encoder | **AS5145B** magnético 12-bit | ABI **1024 pulsos/vuelta**; modo cuadratura **x4** (`TIM_SMCR_SMS_EM3`) → **4096 cuentas/vuelta**. Sin pulsos perdidos hasta 15 000 rpm. INL ±0,5–1,4° |
| Driver motor | **MP6615** ×2 (H-bridge **DC escobillas**, 8 A) | Hasta 40 V; **sensado de corriente bidireccional integrado** (SOA/SOB, ratio ≈ 1/11000) |
| Corriente motor | `ADC2[4], ADC2[5]` → `motor_current_raw[]` | **Medida pero sin usar** hoy (solo debug comentado) |
| Batería | JST-XH 4 pines; logs 11–12,6 V | **3S LiPo** (11,1–12,6 V) |

**Datos hardware PENDIENTES** (para cerrar el modelo): relación de reducción encoder↔rueda y diámetro de rueda; modelo/kV del motor; capacidad de batería; `RREF`/`VREF` del sensado de corriente; confirmar que el PWM es control de tensión.

---

## 1. Cadena de medida y control (encoder → PI → PWM)

**Scheduler** (`src/sys_tick.c`): SysTick a **8 kHz**, 8 slots; cada tarea a **1 kHz** en su slot:

| Slot | Tarea | Frecuencia |
|---|---|---|
| 0 | `clock_tick()` + `check_buttons()` | 1 kHz |
| 1 | `update_encoder_readings()` + `line_sensors_update_position()` | 1 kHz |
| 2 | `lsm6dsr_update()` + `odometry_update()` | 1 kHz |
| 3 | `control_loop()` | 1 kHz |
| 4 | `update_battery_voltage()` | 1 kHz |
| 5–7 | `sram_store()` (volcado) | 1 kHz |

> El lazo de control lee la velocidad de encoder actualizada **2 slots antes** (~250 µs de retardo de slot). `odometry_update()` está condicionado a `!sram_log_is_paused()`.

**Medida de velocidad** (`src/encoders.c`, en slot 1):
1. `read_encoder_left/right()` → `timer_get_counter(TIM3/TIM4)` (ENC en modo cuadratura x4, periodo `0xFFFF`).
2. `max_likelihood_counter_diff()` → `left/right_diff_ticks` (mínima diferencia; maneja wrap).
3. `new_speed = diff_ticks × (µm/tick)/1000 × CONTROL_FREQUENCY_HZ` = `diff_ticks × 70.4349` mm/s.
4. **EMA α = 0,2**: `speed = 0.2·new + 0.8·speed_prev` (τ ≈ 4,5 ms; retardo de grupo ≈ 4,5 ms).
5. `get_measured_linear_speed() = (left_speed + right_speed)/2`.

**Consigna** (`src/control.c`):
- `update_ideal_linear_speed()`: rampa entera `ideal += accel/1000` cada ms (aceleración cuantizada a múltiplos de 1000 mm/s²); `accel_hard`/`accel_soft` al subir; `break_accel` al bajar.
- PI: `linear_voltage = kp·e + ki·Σe + kd·Δe`, `e = ideal − measured`; `kp=0.003`, `ki=0.0002`, `kd=0`.
- `pwm = V/batería × 1049` (compensación de batería) y reparto diferencial `V_left = lin + ang`, `V_right = lin − ang`.
- **`pwm_avg = (pwm_left + pwm_right)/2` cancela exactamente la parte angular** → sirve para aislar el lazo lineal (usado en este análisis).

---

## 2. Hallazgos cuantitativos (de los logs)

Fuentes: `linear_speed_control_fine_tuning_straight.txt` (3 arranques limpios), `..._racing.txt`, `line_control_fine_tunning_SPEED_FAST_*.txt`, `line_control_kp-*`.

| Métrica | Valor | Comentario |
|---|---|---|
| Error estacionario (2000 mm/s) | **−0,1 %…−0,2 %** (≈ −2,7 ± 19 mm/s) | Excelente; lo sostiene el integral |
| Tensión para 2000 mm/s | **1,70 V** (`pwm_avg`≈148; batt≈12 V) | |
| Tensión para 3500 mm/s | **≈3,0–3,15 V** (`pwm_avg`≈300–311) | |
| Error medio en rampa 0→2000 (398 ms) | **+23 mm/s** (p95 77; máx **95**) | 1,2 % medio, 4,7 % pico |
| Sobreoscilación tras rampa | **≈2–4 %** (máx ≈2079) | Parte es rizado de medida |
| Resolución de encoder | **70,43 mm/s por tick** | a 2000 = 28,4 ticks/ms → **3,5 %/tick** |
| Velocidad cruda (1 ms) std | **≈68 mm/s** | tras EMA α=0,2 → **≈26 mm/s** |
| Rizado en pista a 3500 (SPEED_FAST) | **≈74–90 mm/s** | Incluye dinámica real en curvas |

**Arranque (hallazgo clave):**

```
t+0ms  ideal=5    measured=0     pwm=3
t+6ms  ideal=35   measured=0     pwm=13
t+10ms ideal=55   measured=0     pwm=21   ← empieza a moverse
t+12ms ideal=65   measured=11    pwm=22
t+26ms ideal=135  measured=40    pwm=52   ← error pico ≈95 mm/s
t+46ms ideal=235  measured=180   pwm=69   ← el error ya baja
```

→ **≈11–12 ms de “deadband” sin movimiento** porque el PI debe **integrar desde cero** la tensión de fricción (≈0,25 V). Un feedforward eliminaría ese retardo y el pico de error.

**Modelo estático estimado** (rampa, `pwm_avg`, v>200):
```
V = 0,00071·v + 0,000050·a + 0,278           (rms residual 0,088 V)
```
Por 2 puntos (2000→1,70 V; 3500→3,05 V): pendiente ≈ **0,0009 V/(mm/s)**.
→ `u_ff ≈ k0 + kv·v + ka·a`, con `k0≈0,26 V` (fricción/deadband), `kv≈0,0008–0,0009 V/(mm/s)`, `ka≈5·10⁻⁵ V/(mm/s²)`.

> La reconstrucción del `µm/tick` desde el log coincide **exacto** con 70,4349 (residuo 0,006 ticks) → el modelo de la EMA es correcto y la cuantización es real.

---

## 3. Problemas / bugs detectados

1. **`accel_soft` muerto.** `speed_hard==0 || ideal<speed_hard` con `speed_hard ≥ target` en TODAS las estrategias (FAST 3500=target; SUPER 4000>3750; HAKI 4500>4000; TEST/NORMAL/MEDIUM `speed_hard=0`) ⇒ **siempre `accel_hard`**. La rampa de dos tramos nunca entra. Además, en TEST/NORMAL/MEDIUM `accel_soft=0`, así que “arreglarlo” a lo bruto dejaría el robot clavado.
2. **División entera** `accel / CONTROL_FREQUENCY_HZ` → aceleración cuantizada a múltiplos de 1000 mm/s².
3. **Rampa sin límite de jerk (S-curve).** Salto de a=0 a a=5000–30000 mm/s² y vuelta; a SPEED_HAKI (30 000 mm/s²) puede provocar patinaje/pérdida de línea al arrancar.
4. **Sin anti-windup:** `sum_linear_error` crece sin cota; única defensa es el clamp de PWM (+ safety-stop). Item F0.5 ya pendiente en `pid_tuning/ROADMAP.md`.
5. **`kd` sin `/dt`:** término `kd·(e−e_prev)` = derivada efectiva ×1000. Hoy `kd=0`, pero si se activa explota.
6. **`ADAPTATIVE_SPEED` y `RUN_ALGORITHM`** del menú existen pero **no se usan** (solo LEDs); `menu_run_get_adaptative_speed()`/`get_run_algorithm()` no se llaman en el control.
7. La **odometría ya tiene un EKF** que estima `v` (`X[S_V]`), pero **el lazo de velocidad no lo usa** (usa la EMA cruda).

---

## 4. Gestión de aceleración — propuestas

- **Rampa de dos tramos real:** redefinir `speed_hard` como *umbral de conmutación a `accel_soft` por debajo del target* (p. ej. `if (target - ideal < blend) accel = accel_soft;`), con valores coherentes (no 0) para todas las estrategias.
- **Eliminar división entera:** acumular con punto fijo (por ejemplo `ideal_fixed += accel·dt_ms`) o redondear.
- **S-curve (jerk limitado):** limitar la variación de la aceleración (`da/dt`) para suavizar arranque y frenada, sobre todo a alta velocidad.
- **Anti-windup:** clamp condicional de `sum_linear_error` (p. ej. integrar solo si el comando no está saturado) y limitar el comando de tensión al máximo alcanzable según batería **antes** de convertir a PWM.

---

## 5. Feedforward — diseño y estimación

**Forma propuesta:**
```
u_ff = k0·sat(v_ideal) + kv·v_ideal + ka·a_ideal
```
- `k0`: fricción seca/deadband (≈0,26 V).
- `kv`: parte estática velocidad↔tensión (≈0,0008–0,0009 V/(mm/s)).
- `ka`: término inercial (≈5·10⁻⁵ V/(mm/s²)).
- `a_ideal` es conocida (la rampa o el planificador del robotracer).

**Beneficios esperados:**
- Arranque: elimina los ~11 ms de deadband y el pico de error (~95 mm/s → mucho menor).
- Rampa: error de seguimiento de ~23 mm/s → ~5–10 mm/s.
- Rechazo de perturbaciones (fricción/carga/batería): el PI solo corrige el residuo.
- **Sinergia con el ruido:** al no tener que aportar la parte estática, se puede bajar `ki` y/o filtrar más la velocidad → menos ruido en PWM.

**Riesgos:** requiere re-tuning (bajar `ki`); modelo imperfecto (fricción no lineal, temperatura); si es configurable (`enabled=false`), riesgo bajo.

**Vía de par (corriente):** el MP6615 mide corriente. Un FF de par (`i_ff ∝ J·a + fricción`) da control de aceleración más limpio que el de tensión, útil para el robotracer adaptativo. **A analizar antes de comprometer** (necesita `RREF`/`VREF` y verificar la dinámica de la corriente).

**Log nuevo propuesto** (`SRAM_TYPE_LINEAR_TUNE`, aditivo, ≤80 B): `t, v_ideal, a_ideal, v_medida_cruda, v_medida_filtrada, V_lin_cmd, pwm_avg, corriente_L, corriente_R, batería` (+ opcionalmente los términos `P/I/FF` por separado). Permite comparar estimadores y validar el FF en la misma pasada.

---

## 6. Ruido de la velocidad por encoders

**Fuentes:**
- **Cuantización:** 1 tick = 70,43 mm/s → a 2000, 3,5 %/muestra (teórica uniforme ≈ 20 mm/s std).
- **Rizado real:** la velocidad cruda reconstruida varía ±2–3 ticks (std ≈ 68 mm/s) **> cuantización pura** ⇒ jitter mecánico (cogging/ripple de par) y **INL del AS5145B** (±0,5–1,4°); PSD con energía concentrada en **20–100 Hz** (coherente con la frecuencia de giro del encoder y armónicos).
- La EMA α=0,2 baja a ≈26 mm/s pero **parte del rizado es real** y no se elimina sin retardo.

**Opciones (coste: retardo):**
1. Bajar α (α=0,1 → ≈17; α=0,05 → ≈11).
2. Media móvil / **regresión LS de la posición** en ventana de N ms (≈1,3× mejor que la diff de extremos a igual retardo, N=10).
3. Usar la **`v` del EKF de odometría** (ya existe) y/o sintonizarlo.
4. **FF + bajar ancho de banda del PI** (sinergia): permite filtrar más sin perder seguimiento.
5. Instrumentar el log nuevo para **comparar candidatos** en la misma pasada y elegir con datos.

A 2000 mm/s el contaje por ms ya es razonable; el límite real es el rizado mecánico/INL, no la resolución.

---

## 7. Plan de fases (roadmap)

- [ ] **F1 — Documentar** (este documento): cadena, hallazgos, bugs.
- [ ] **F2 — Correcciones de aceleración** (reversibles): `accel_soft`, división entera, S-curve, anti-windup + límites de salida.
- [ ] **F3 — Log nuevo `SRAM_TYPE_LINEAR_TUNE`** y volcado/parseo.
- [ ] **F4 — Estimación FF** `k0/kv/ka` desde runs normales (regresión `V(v,a)`), + vía de par si procede.
- [ ] **F5 — Comparativa de estimadores de velocidad** (EMA / LS / EKF) con métricas ruido vs retardo.
- [ ] **F6 — A/B del FF** en los mismos circuitos (métricas: `e_rampa`, overshoot, `e_ss`, `|ΔPWM|`), decidir con datos.
- [ ] **F7 — Preparar velocidad adaptativa** (robotracer): el FF y la rampa aceptan `v_ideal`/`a_ideal` dinámicos.

---

## 8. Referencias de código

- `src/encoders.c`, `include/encoders.h` — lectura, cuantización, EMA, `max_likelihood_counter_diff`.
- `src/control.c`, `include/control.h` — `control_loop()`, rampa (`update_ideal_linear_speed`), PI lineal/angular, logging.
- `src/kinematics.c`, `include/kinematics.h` — estrategias y `linear_accel` (`accel_hard/soft`, `break_accel`, `speed_hard`).
- `src/motors.c` — `set_motors_pwm` (clamp, saturación), `motor_current_raw`.
- `src/setup.c` — config de TIM3/TIM4 (encoders x4), TIM1 (PWM motores), ADC/DMA (corriente, batería).
- `src/sys_tick.c` — scheduler de slots a 1 kHz.
- `src/odometry.c`, `include/odometry.h` — EKF 6 estados (incluye `v`).
- `include/sram.h`, `src/sram.c` — esquema y volcado de logs (añadir `SRAM_TYPE_LINEAR_TUNE`).
- `.features/pid_tuning/ROADMAP.md` — antecedentes del tuning de velocidad/línea.

---

## 9. Herramienta de análisis

`tools/linear_speed_analyzer.py` reproduce estas métricas desde el volcado del log
de control (`SRAM_TYPE_CONTROL`). No requiere scipy (es opcional, para las gráficas).

```
python3 tools/linear_speed_analyzer.py report <log...> [--plot DIR]   # SS + rampa por segmento
python3 tools/linear_speed_analyzer.py ramp   <log...> [--verbose]    # arranques/aceleraciones
python3 tools/linear_speed_analyzer.py ff     <log...> [--min-v 200]  # ajuste V=k0+kv*v+ka*a + curva V(v)
python3 tools/linear_speed_analyzer.py noise  <log> [--target 2000]   # ruido y estimadores
python3 tools/linear_speed_analyzer.py plot   <log> [-o out.png]
```

Notas de interpretación:

- `pwm_avg = (pwm_left+pwm_right)/2` cancela la parte angular y aísla el lazo lineal.
- El ajuste `ff` solo es válido en runs con `target` fijo (recta); en circuito da
  `kv<=0` (la herramienta lo avisa) porque `v` no depende solo del comando.
- El residual de reconstrucción de la velocidad cruda es ~0.25 tick (no 0) porque
  `measured = (v_izq + v_der)/2` es media de dos canales cuantizados → medio tick
  (≈35.2 mm/s) de resolución efectiva.
- La fracción de "ruido" medida en HF (>20 Hz, std ≈ 24 mm/s a α=0.2) mezcla
  ruido de cuantización con rizado mecánico/INL del AS5145B; no es todo removible
  con filtrado (habría que pagar retardo).
