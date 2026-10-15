# ROADMAP — Odometría fiable (DragonBot2)

> Estado: **documento vivo**. Marcar `[x]` a medida que se completen las tareas.
> Objetivo: obtener la odometría (posición `x/y` y rumbo `θ`) más fiable posible con el
> hardware disponible: **encoders de cuadratura** (TIM3/TIM4) + **IMU LSM6DSR** (gyro + accel)
> + **array de sensores de línea** (única referencia absoluta lateral). Sin magnetómetro.

> **Decisiones** (ver §9): D1 = convención §3 (CCW+, +y izquierda); D2 = `odometry_update()`
> en el mismo slot que MPU/BMP; D3 *(revisado)* = alcance hasta el **EKF** (Fase 3 pasa a objetivo
> principal, ya no aparcada); D4 = acelerómetro incluido *(revocado por D7)*; D5 = migrar todos los
> consumidores a odometría y eliminar getters innecesarios; D6 = sin compensación de inclinación;
> D7 = acelerómetro eliminado; **D8** = base de medida **sin filtros software** de cara al EKF;
> **D9** = el estimador de modelo (**α-β / EKF**) sustituye al filtro pasa-bajos; **D10** = ir
> directo al **EKF sin pasos intermedios ni *fallback*** (el robot está en desarrollo);
> **D11** = descartar la fusión absoluta con la línea (sin mapa).

---

## 1. Objetivos y alcance

- **Correcta**: unidades, signos y matemática verificados, con una única convención.
- **Encapsulada**: toda la fusión y el estado en el módulo `odometry.c`.
- **Robusta**: preparada para derrapes, vibración y deriva de bias.
- **Validable**: procedimientos de calibración y ensayos repetibles comparables con verdad de referencia.
- **Seguimiento de trayectoria** (objetivo final): base de estimación fiable (pose + incertidumbre) para
  recortar curvas / atajar zigzags en el futuro.

Fuera de alcance (por ahora): control de rumbo/posición cerrado en competición y magnetómetro.
La navegación basada en odometría **entra** en alcance vía el EKF (Fase 3).

---

## 2. Estado actual (análisis "as-is")

### 2.1 Flujo y temporización

- `sys_tick.h`: SysTick a **8 kHz**, **8 slots** → cada tarea corre nominalmente a **1 kHz**.
- `update_encoder_readings()` → slot `SYS_TICK_SLOT_ENCODERS_LINE` (idx 1).
- `lsm6dsr_update()` → slot `SYS_TICK_SLOT_MPU_BMP` (idx 2).
- `control_loop()` → slot `SYS_TICK_SLOT_CONTROL` (idx 3).
- `constants.h`: `UPDATE_FREQUENCY_HZ = CONTROL_FREQUENCY_HZ = 1000`.
- **Riesgo**: encoder y gyro asumen `dt = 1/1000` fijo. Si un slot se pasa de 125 µs
  (p. ej. los volcados a SRAM en los slots `FLUSH_*` o `sram_log` dentro de `control_loop`),
  el periodo real deja de ser 1 ms y **sesga velocidad e integral de ángulo**.
  Mitigación: medir `dt` real con el contador de ciclos **DWT** (ya habilitado en
  `setup.c` con `dwt_enable_cycle_counter()`).

### 2.2 Hallazgos (con referencia)

| # | Hallazgo | Ubicación | Impacto |
|---|----------|-----------|---------|
| H1 | **Signo de rumbo invertido**: `rad_integ` es horario-positivo (por el `-` en la integral) pero se usa con rotación CCW-positiva (`cos/sin`). Giro físico a izquierdas ⇒ gráfica a derechas. | `lsm6dsr.c:212-213`, `encoders.c:202-203` | **Alto** (bug observado) |
| H2 | **Distancia desde velocidad filtrada**, no desde ticks crudos; además `avg_millimeters` está mal nombrado (es Δdistancia/ciclo). | `encoders.c:200-203` | Alto (sesgo + lag) |
| H3 | **Heading de fin de ciclo** en vez de heading medio/arco. | `encoders.c:202-203` | Medio (error 2º orden en curvas) |
| H4 | **Doble filtrado del gyro**: LPF hardware (`gy_filter_lp1` + `LIGHT`) **y** LPF software (`alpha=0.27`). *(Revisado: la cascada es intencionada y beneficiosa —más limpieza + antialias—; el coste es solo retardo de fase, inocuo para el rumbo integrado. Ver F2.2.)* | `lsm6dsr.c:138-140`, `lsm6dsr.c:208-210` | Bajo (lag, tolerable) |
| H5 | **Sin data-ready**: se lee `OUTZ` sin comprobar `STATUS_REG (0x1E)`; ODR gyro 1666 Hz vs lectura 1 kHz. | `lsm6dsr.c:69-76`, `lsm6dsr.c:203-215` | Bajo/Medio |
| H6 | **Bias del gyro solo al arranque**: sin ZUPT ni estimación online (deriva por temperatura). | `lsm6dsr.c:161-201` | Medio (en carreras largas) |
| H7 | **Acelerómetro configurado pero nunca leído**. *(Resuelto: eliminado por D7.)* | `lsm6dsr.c:128-129` | Cerrado |
| H8 | **Sin detección de derrape**: no se comparan gyro y encoders. *(Resuelto en F2.5.)* | — | Cerrado |
| H9 | **Constantes de escala sin calibrar**: `MICROMETERS_PER_TICK` y `WHEELS_SEPARATION` son valores fijos sin test. | `config.h:22-24` | Alto (error proporcional global) |
| H10 | **Convenciones de signo dispares** entre encoder (horario+) y heading de posición (CCW) y `get_measured_angular_speed` (`-gyro`). | `encoders.c:198`, `control.c:94-96` | Medio (confusión/fusión) |
| H11 | `reset_encoders()` no resetea `left_speed/right_speed/angular_speed`. | `encoders.c:93-104` | Bajo (transitorio) |
| H12 | `PI = 3.1415` (baja precisión) y defines `GYRO_SENSITIVITY_*` sin usar. | `constants.h:5`, `lsm6dsr.c:17-18` | Muy bajo (limpieza) |

### 2.3 Lo que está correcto

- Escala gyro LSM6DSR: 35/70/140 mdps/LSB (`lsm6dsr.h:22-26`) y conversión `raw·sens/1000 = dps`,
  `·π/180 = rad/s` (`lsm6dsr.c:221-231`). ✔
- Offset de gyro restado **antes** del filtro y calibrado por full-scale en EEPROM. ✔
- `max_likelihood_counter_diff()` maneja overflow de 16 bits y dirección. ✔
- Velocidad lineal de encoders dimensionalmente correcta: `diff_ticks · 70.43 = mm/s` (µm/ms ≡ mm/s). ✔

---

## 3. Decisión de convención ✅ CONFIRMADA (D1)

**Convención única para todo `odometry.c`:**

- `+x` = avance del robot.
- `+y` = izquierda del robot.
- `θ` = rumbo, **positivo = giro a la izquierda (CCW)**, `θ = ∫ ω_z dt`.
- `ω_z` (gyro) positivo = CCW. La velocidad angular de encoders pasa a `ω_enc = (v_right − v_left)/L`.

Consecuencias:
- Se elimina el `-` de la integración del gyro en `lsm6dsr.c` (o se mueve la integración a odometría).
- `get_measured_angular_speed()` (`control.c`) y el PID angular deben alinearse a esta convención
  **si algún día se activa el control de rumbo** (hoy `ideal_angular_speed = 0` y corrección deshabilitada).
- `get_encoder_angular_speed()` se usa solo con `abs()` en saturación (`motors.c:127`): cambio de signo inocuo.

> ⚠️ Cualquier consumidor que compare encoder y gyro debe usar la **misma** convención. Centralizar.

---

## 4. Arquitectura objetivo

### 4.1 Reparto de responsabilidades

| Módulo | Responsabilidad |
|--------|-----------------|
| `encoders.c` | Ticks → **deltas crudos** (nuevos getters), distancias totales y velocidades filtradas (control). **Se le quita** `x/y` y el cálculo de rumbo. |
| `lsm6dsr.c` | Driver IMU: init, calibración/bias, lectura raw gyro, filtro, data-ready, escala. **Se le quita** la integración de ángulo. (Acelerómetro eliminado, D7.) |
| `odometry.c` (**nuevo**) | Estado y fusión: `x, y, θ, v, ω, bias_gyro, slip`. Única fuente de verdad de posición/rumbo. |
| `control.c` | Consume odometría para logging y (futuro) control de rumbo. |
| `sys_tick.c` | Llama a `odometry_update()`. |
| `sram.c` / `sram.h` | Esquema de `odometry_log_t` y su impresión. |

### 4.2 API propuesta — `include/odometry.h`

```c
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdbool.h>

void  odometry_init(void);
void  odometry_reset(void);            /* x, y, theta, bias, slip, histórico */
void  odometry_update(void);           /* 1 kHz, desde SysTick */

float odometry_get_x(void);            /* mm */
float odometry_get_y(void);            /* mm */
float odometry_get_theta_rad(void);    /* rad, CCW+ */
float odometry_get_theta_deg(void);    /* grados, CCW+ */
float odometry_get_linear_speed(void); /* mm/s */
float odometry_get_angular_speed(void);/* rad/s, CCW+ */
float odometry_get_encoder_omega(void);/* rad/s, CCW+ (diferencial encoders) */
float odometry_get_gyro_bias(void);    /* rad/s */
bool  odometry_get_slip(void);

#endif
```

### 4.3 Punto de integración en el scheduler

- **Decidido (D2)**: llamar `odometry_update()` al final del slot `SYS_TICK_SLOT_MPU_BMP`
  (tras `lsm6dsr_update()` y el futuro `bmp_update()`): gyro fresco y deltas de encoder del
  slot anterior (skew ≈ 125 µs, despreciable).
- **¿Es correcto?** Sí. El slot `MPU_BMP` corre a 1 kHz igual que `ENCODERS_LINE`, y los deltas
  `left_diff_ticks`/`right_diff_ticks` se actualizaron en el slot inmediatamente anterior, por lo
  que el dato de encoder usado es el más fresco disponible (el intervalo del delta coincide con el
  ciclo de 1 ms). La única consideración es el **presupuesto de tiempo del slot**: si BMP280 vuelve
  a activarse (usa SPI a baja velocidad + conmutación de baudrate) o si crece el trabajo, el slot
  podría superar 125 µs. Mitigación ya prevista en Fase 2 (F2.1): `dt` real por DWT, que hace la
  integración inmune a esa variación.
- Alternativa descartada por ahora: crear un slot propio (implicaría subir `SYSTICK_FREQUENCY_HZ`
  o recortar slots de flush). No necesario si F2.1 entra.

---

## 5. Estrategia de filtrado y transición al EKF

**Principio:** un pasa-bajos causal **siempre** añade retraso. Al integrar el ritmo medido,
`θ = ∫ω_filtrado dt`, filtrar equivale a filtrar el propio rumbo:
`θ_integrado ≈ θ_real − τ·ω`. Ese déficit crece con el giro y **deforma la trayectoria** (y en lazo
cerrado **come margen de fase** → inestabilidad). Por eso el objetivo no es "ver el ritmo limpio",
sino **estimar bien la pose**.

**Consecuencia (D8/D9):** de cara al EKF **no se pre-filtra** la medida; el EKF *es* el filtro.
Pre-filtrar rompe la independencia del ruido (el EKF asume `R` blanco) y añade lag que no se puede
deshacer. Se alimenta el estimador con medidas lo más **crudas** posible y un modelo de ruido `R`.

**Qué hace "mágico" al Kalman (y qué no):**
- No es un suavizador: es un estimador **modelo + medidas** ponderados por sus incertidumbres (`Q`, `R`).
- Su verdadero valor en un gyro es **estimar online el bias** `b_g` (la deriva es el error dominante).
- **El acelerómetro no observa el rumbo** (solo la gravedad): no corrige la deriva de `θ`. Las correcciones
  de rumbo deben venir de **encoders** (rumbo diferencial) y las de lateral de los **sensores de línea**
  (única referencia absoluta disponible). Sin magnetómetro no hay rumbo absoluto.
- Requiere **tuning de `Q/R`** y robustez (gating de innovaciones, clamps). Sin logs para ajustar, no vale.

**Pasos:**
1. **Quitar la EMA software** del gyro (D8). Decidir antialias:
   - **Elegido: ODR 833 Hz sin LPF1** (contenido < 416 Hz < Nyquist de la lectura a 1 kHz → sin aliasing;
     la irregularidad de muestras la absorbe el EKF). ~~ODR 1666 Hz + LPF1 `LIGHT`~~ descartado.
2. **Paso intermedio barato:** filtro **α-β** (Kalman estacionario) como único filtro del ritmo: no se
   retrasa ante un giro sostenido y apenas usa CPU.
3. **EKF de pose (Fase 3):** estado `[x, y, θ, v, ω, b_g]`; predicción cada ciclo con `dt` medido;
   medidas de encoders (`v`, `ω`), gyro (`ω + b_g`) y línea (offset lateral absoluto). El **ZUPT** deja de
   ser un filtro aparte y pasa a ser una **pseudo-medida** (`v=0, ω=0`) que observa el bias.

---

## 6. Reparto de trabajo y calibración automática

**Solo el usuario (físico/criterio):**
- Aportar una **distancia patrón** (regla/marcas) para la escala de rueda.
- Ejecutar maniobras: arranque, giro 360°, recta medida, círculos/"8", pista real, derrape.
- Flashear/resetear y **volcar los logs SRAM**.
- Decidir qué es "buena trayectoria" (métrica objetivo).

**Automatizable en firmware (sin intervención manual):** *(implementado en Fase 4)*
- **Bias del gyro** al arranque (existe) + refinado online por ZUPT (existe).
- **Ancho de vía `WHEELS_SEPARATION`**: girar en el sitio hasta acumular **2π con el gyro** →
  `sep = (Δticks_R − Δticks_L)·µm_tick / 2π`. Totalmente automático (el gyro es la referencia de ángulo).
  Añadible como calibración de menú, igual que la del gyro Z.
- **Umbrales de ruido** (slip/ZUPT) y `R` del EKF: medir el gyro en reposo N muestras → fijar a Nσ.
- **Escala de rueda** `µm_tick`: semi-automática (el usuario aporta la distancia patrón;
  `µm_tick = distancia_mm·1000 / Δticks_medio`). Automatizable del todo si hay un tramo entre marcas.

**Ajustable por el agente desde logs (offline, script en `tools/`):**
- Re-estimar `µm_tick` y `WHEELS_SEPARATION` desde los propios datos (giro: gyro vs ticks; recta: distancia).
- Estimar densidad de ruido y modelo de deriva → `R` y estado de bias.
- Ajustar `Q/R` del EKF (covariance matching / máxima verosimilitud).
- Comparar configuraciones por **error de cierre de trayectoria** y generar el fichero de constantes.

**Campos que debe registrar el log para el auto-tuning (implementar antes del EKF):**
`gyro_crudo` (rad/s sin filtrar), `dt`/ciclos DWT y, cuando exista, `line_offset`.

---

## 7. Fases

Cada fase incluye: **tareas**, **implicaciones** (archivos, riesgos) y **criterio de validación**.

### Fase 0 — Corrección y matemática básica ✅ COMPLETADA
**Objetivo:** eliminar el bug de signo (H1) y la distancia filtrada (H2/H3). Cambios pequeños y localizados.

- [x] F0.1 Fijar la convención de §3 y documentarla en cabecera de `encoders.c` (Fase 1 la replicará en `odometry.h`).
- [x] F0.2 Corregir el signo del rumbo (arreglar H1): `lsm6dsr.c` integra ahora **CCW-positivo**; `get_measured_angular_speed()` (`control.c`) y `get_encoder_angular_speed()` (`encoders.c`) alineados a CCW+.
- [x] F0.3 Distancia por ticks crudos: `d = 0.5·(dl + dr)` con `dl/dr` de `left_diff_ticks/right_diff_ticks` (mm).
- [x] F0.4 **Modelo de arco** en la integración de posición (usa el heading del gyro y `last_heading_rad`); equivale al heading medio para giros pequeños.
- [x] F0.5 Eliminado `avg_millimeters`; el avance por ciclo es ahora la variable local `delta_millimeters`.
- [x] F0.6 `reset_encoders()` resetea también `left_speed/right_speed/angular_speed` y `last_heading_rad` (H11).

**Implicaciones reales:** `src/encoders.c`, `src/lsm6dsr.c`, `src/control.c`. Riesgo bajo. API pública intacta.
**Estado:** ✅ **Compila sin errores** (`pio run` OK, 0.60 s, sin warnings). RAM 4.4 %, Flash 5.7 %.
**Validación en hardware (pendiente de banco, usuario):**
- Avanzar describiendo un "8" o un círculo: el trazo de `pos_x/pos_y` debe seguir el sentido **real** del giro.
- Comparar `get_encoder_avg_millimeters()` con avance medido con regla.
- Ver §8 (tabla de ensayos).

---

### Fase 1 — Encapsulación en `odometry.c` ✅ COMPLETADA
**Objetivo:** una sola fuente de verdad para `x/y/θ`.

- [x] F1.1 Creados `include/odometry.h` + `src/odometry.c` con la API de §4.2.
- [x] F1.2 Integración de rumbo movida a `odometry.c`; `lsm6dsr.c` queda como driver puro (se eliminan `rad_integ`/`deg_integ`, `lsm6dsr_get_gyro_z_rads/_degrees/_reset_gyro_z`).
- [x] F1.3 Añadidos `get_encoder_left_diff_ticks()` / `get_encoder_right_diff_ticks()` en `encoders.c`.
- [x] F1.4 `encoders.c` ya no calcula `position_x/position_y` (estado y fusión en `odometry.c`).
- [x] F1.5 **Migrados los consumidores** y **eliminados** `get_encoder_position_x/y` (D5). Sin wrappers.
- [x] F1.6 `control.c` usa `odometry_get_x/y/theta_deg`. `debug.*`/`sram.*` no requerían cambios (usan los campos del struct).
- [x] F1.7 `odometry_update()` registrado en `sys_tick.c`, **al final del slot `MPU_BMP`** tras `lsm6dsr_update()` (§4.3).
- [x] F1.8 `odometry_reset()` desde `main.c` (arranque de carrera) y `reset_control_all()` (`control.c`); `odometry_init()` en `setup()`.

**Implicaciones reales:** nuevo módulo `odometry.*` + `encoders.*`, `lsm6dsr.*`, `control.c`, `sys_tick.c`,
`setup.c`, `main.c`. `sram.*` sin cambios (el esquema `odometry_log_t` se amplía en Fase 2).
**Estado:** ✅ **Compila sin errores ni warnings nuevos** (compilación limpia; RAM 4.4 %, Flash 5.7 %).
Los 2 warnings existentes (`bmp280.c`, `debug.c`) son preexistentes y ajenos.
**Nota de alineación temporal:** al mover la integración al slot `MPU_BMP`, el rumbo y el
desplazamiento quedan mejor alineados que en Fase 0 (desfase ≈ 125 µs en vez de ≈ 875 µs).
**Validación en hardware (pendiente, usuario):** misma trayectoria que en Fase 0 → no debe haber regresión.

---

### Fase 2 — Robustez ✅ COMPLETADA
**Objetivo:** exactitud bajo condiciones reales (dt variable, deriva, derrape, vibración).

- [x] F2.1 **`dt` real por DWT** (`read_cycle_counter()`) en `odometry_update()`; `θ += ω·tilt·dt`.
      Con clamp a 5 ms si el contador se desborda o el intervalo es anómalo.
- [x] F2.2 **Filtrado en cascada** (revisado): se **reactiva** el LPF1 hardware del gyro en `LIGHT`
      (`lsm6dsr_gy_filter_lp1_set(ENABLE)` + `lsm6dsr_gy_lp1_bandwidth_set(LSM6DSR_LIGHT)`) junto con el
      LPF software (`low_pass_filter_alpha`). El LPF1 actúa antes de la decimación del sensor (antialias) y
      el software después; no son redundantes. Coste: algo de retardo de fase, asumible en un seguidor de
      línea (~inocuo para el rumbo integrado). Ajustar el nivel del LPF1 (`LIGHT`..`XTREME`) y/o `alpha` si
      se quiere más/menos limpieza. *(Nota: era la configuración original del firmware.)* **→ Superseded por
      F3.0 (D8/D9): se abandona la cascada y se pasa a medida cruda.**
- [x] F2.3 **Data-ready** del gyro (`STATUS_REG 0x1E`, bit `gda`); solo se actualiza el filtro con muestra
      nueva. Red de seguridad: si el flag no llega en 5 ciclos, se fuerza lectura (nunca congela el rumbo).
      **Nota de sincronización:** el gyro va a **1666 Hz** (muestra nueva cada ~600 µs) y el slot lee a
      **1 kHz**, por lo que no hay correspondencia 1:1 (se decima ~1666→1000). El LSM6DSR no tiene ODR de
      1000 Hz; con `dt` medido por DWT, la integral es correcta a primer orden. *(En F3.0 el ODR pasó a
      **833 Hz** sin LPF1; ver F3.0 y §5.)*
- [x] F2.4 **Bias online / ZUPT**: en reposo (ambas ruedas sin ticks > 200 ms y `|ω| < 0.5 rad/s`) se refina
      `gyro_bias` y se congela la integración de θ. `odometry_get_gyro_bias()` expuesto y registrado.
      El bias **se conserva** entre `odometry_reset()` (solo `odometry_init()` lo pone a 0).
- [x] F2.5 **Detección de derrape**: se compara el **ángulo integrado** (gyro vs encoders) en una ventana
      de 100 ms (cancela desfase de filtros y cuantización). Flag `slip` expuesto y registrado.
      Decisión conservadora: **no se altera la traslación** (sin medición alternativa hasta la fusión con línea).
- [x] F2.6 **Acelerómetro**: **eliminado** (D7). En pista nivelada la inclinación es despreciable y, de
      haberla, sería en rectas con `ω≈0`, por lo que no aporta al rumbo; se descarta también como
      diagnóstico para reducir complejidad. Toda la lectura/getters/log del acelerómetro fueron removidos.
      (El bloque de configuración de XL de `lsm6dsr_init()` se conserva sin cambios: el código original lo
      anota como "helper clock only" y desactivarlo es un riesgo aparte a validar.)
- [x] F2.7 Log `odometry_log_t` ampliado: `linear_speed`, `encoder_omega`, `gyro_bias`, `slip`
      (impresión y `sizeof` actualizados). *(El campo `accel_z_g` se añadió y luego se retiró con el acelerómetro, D7.)*

**Implicaciones reales:** `odometry.c` (núcleo), `lsm6dsr.c` (acelerómetro, data-ready, filtro), `sram.h/.c`,
`control.c`. Riesgo medio por tuning de umbrales (constantes al inicio de `odometry.c`).
**Estado:** ✅ **Compila limpio** (RAM 4.4 %, Flash 5.8 %); sin warnings nuevos.
**Compromisos y límites (documentados):**
- El giro (1666 Hz) y el slot (1 kHz) no están sincronizados: se decima ~1666→1000 muestras/s. No hay
  ODR de 1000 Hz en el LSM6DSR; con `dt` medido, el error es de aliasing/cuantización menor.
- No se aplica compensación por inclinación y el acelerómetro está eliminado (D6/D7): pista nivelada y,
  de inclinarse, sería en rectas.
- El detector de derrape detecta **derrape de guiñada** (pérdida de tracción que desalinea el giro medido);
  **no** detecta derrape longitudinal puro (no hay medición alternativa: el acelerómetro quedó descartado, D7).
- Cambiar el filtro (F2.2) es fácilmente reversible: reactivar LPF1 hardware o ajustar `low_pass_filter_alpha`.

**Validación en hardware (pendiente, conjunta):**
- Deriva estacionaria: robot parado N s ⇒ `|Δθ|` bajo, y `gyro_bias` convergiendo en el log.
- Derrape inducido (levantar una rueda / superficie deslizante): `slip` se activa.
- Inclinación: sobre rampa, θ no se contamina (y no hay sesgo en llano).
- Recta/círculo/"8": sin regresión respecto a Fase 0/1.

---

### Fase 3 — Fusión avanzada (EKF) ✅ CERRADA (D3 revisado)
**Objetivo:** fusión óptima de modelo + medidas, con confianza (covarianza), como base para el
seguimiento de trayectoria. Ver §5 para la estrategia de filtrado.

- [x] F3.0 **Base de medida sin filtros software** (D8) ✅. Decidido **ODR 833 Hz sin LPF1 y sin EMA**
      (evita aliasing al leer a 1 kHz; el EKF maneja la irregularidad de muestras con `gda`). Se conserva
      el offset calibrado; la medida queda cruda. Añadidos al log `dt` y `line_offset`
      (`measured_angular_speed` pasa a ser la rata cruda).
- [x] F3.1 **α-β intermedio**: **descartado** (D10). El robot está en desarrollo y no se requiere estabilidad
      incremental; se va directo al EKF.
- [x] F3.2 **EKF de pose** `[x, y, θ, v, ω, b_g]` ✅. Implementado en `odometry.c` (sustituye a la integración
      simple; sin *fallback*, D10):
      - Predicción cada ciclo (`dt` medido): `x += v·cosθ·dt`, `y += v·sinθ·dt`, `θ += ω·dt`.
      - Medidas: `z_gyro = ω + b_g` (**gated por `lsm6dsr_gyro_sample_ready()`**), `z_enc = {v, ω}` desde
        deltas crudos de ticks; **ZUPT** (`v=0, ω=0`) cuando está parado.
      - `Q`/`R`/`P0` en defines al inicio de `odometry.c` (valores iniciales a tunear con logs).
      - Simetrización de `P` para estabilidad numérica.
- [~] F3.3 **Fusión absoluta con sensores de línea**: **CANCELADA (D11)**. Sin mapa no es posible: la
      línea es una feature **relativa** ("estoy a `e` de la línea") co-creada con la trayectoria; sin revisitas
      ni modelo de pista no hay información de posición absoluta. La posición de la línea es, sobre todo,
      **entrada de control**. (Con vueltas múltiples sería *loop closure / SLAM-lite*, otra cosa — ver §12.)
- [x] F3.4 Robustez ✅: **gating de innovaciones** (límites físicos por medida, descarta glitches/divergencia)
      + **clamps de estado** (`v`, `ω`, `b_g`) y **suelo de varianza** (`P` diagonal).
- [x] F3.5 Covarianza publicada y registrada: `odometry_get_var_x/y/theta()` y campos `var_x/var_y/var_theta`
      en `odometry_log_t` (para el futuro planificador y para el tuning offline).

**Implicaciones reales:** `odometry.c` (reescrito con el EKF), `lsm6dsr.c/.h` (flag de muestra nueva),
`sram.h/.c` (log ampliado, `SRAM_LOG_MAX_PAYLOAD` 64→80), `control.c`.
**Estado:** ✅ **Compila limpio** (RAM 5.2 %, Flash 5.9 %; sin warnings nuevos).
**Pendiente de validación en banco:** que `x/y/θ` sigan la trayectoria real y que `gyro_bias` y las
`var_*` se comporten (ver §8). El tuning de `Q/R` se hará con los logs (§6).

> Nota clave: el **acelerómetro no observa el rumbo** (solo la gravedad). La única corrección absoluta de
> lateral disponible es el **array de línea**; para rumbo absoluto haría falta magnetómetro (no disponible).

---

### Fase 4 — Calibración y validación (prioridad alta: habilita el tuning del EKF) ✅ COMPLETADA
**Objetivo:** cerrar la escala y crear un banco de pruebas repetible y **auto-calibrable** (ver §6).

- [x] F4.1/F4.2 **Calibración de geometría en runtime + EEPROM**: `get_micrometers_per_tick()` y
      `get_wheels_separation()` sustituyen a los `#define` en `encoders.c`/`odometry.c`; se cargan de EEPROM
      (`config_load_calibration()`), con los valores de compilación como defecto.
- [x] F4.5 **Auto-calibración en firmware** (menú → CALIBRACIÓN):
      - `CALIBRATE_WHEEL_SEPARATION`: giro motorizado hasta `2π` con el gyro → `sep = (Δder−Δizq)/θ`.
        Pausa el control (`set_control_paused`) para poder accionar los motores.
      - `CALIBRATE_WHEEL_SCALE`: recorrer `WHEEL_SCALE_CALIBRATION_MM` (patrón) y confirmar con MENU →
        `µm/tick = ref_mm·1000 / Δticks_medio`.
      - Ambas escriben a `eeprom_data`; se persisten con `CALIBRATE_STORE_EEPROM`.
- [x] F4.4 **Script de análisis** `tools/parse_odometry_log.py`: parsea el volcado del log SRAM, imprime
      resumen (duración, distancia, cierre, integral gyro vs encoder, slip, bias, varianzas) y grafica
      trayectoria/rumbo/velocidades. Exporta CSV opcional.
- [x] **Modo debug en vivo** de odometría (`DEBUG_TYPE_ODOMETRY`, menú → DEBUG, solo banco con cable):
      imprime `x,y,θ,v,ω,gyro,enc_w,bias,slip,dt,var_*` cada 50 ms.
- [ ] F4.3 Rutina de ensayos: recta, cuadrado, círculo, "8", deriva estacionaria, derrape ← **pendiente (usuario)**.
- [ ] F4.6 Documentar resultado de cada ensayo (tabla de errores) en este ROADMAP ← **pendiente**.

**Estado:** ✅ **Compila limpio** (RAM 5.2 %, Flash 6.1 %).
**Pendiente de usuario:** correr la auto-calibración, validar con el modo debug/script, y pasar logs para
tunear `Q/R` y umbrales.

---

## 8. Procedimientos de validación (detalle)

| Ensayo | Procedimiento | Métrica |
|--------|----------------|---------|
| Recta | Avance `D` conocido sin girar | `|Δx − D|`, `|Δy|` |
| Giro puro | Giro `360°` sobre eje | `|θ − 2π|`, desplazamiento residual |
| Cuadrado | 4 rectas + 4 giros de 90° | error de cierre (distancia inicio–fin) |
| Deriva ZUPT | Parado `T` segundos | `Δθ`, deriva de `x/y` |
| Derrape | Forzar pérdida de tracción | activación de `slip`, error de θ |
| Inclinación | *(N/A: sin compensación de tilt ni acelerómetro, D6/D7)* | — |

---

## 9. Registro de decisiones

- [x] **D1** Convención §3 (CCW+, +y izquierda). ✅ Aceptada.
- [x] **D2** `odometry_update()` en el **mismo slot** que `MPU_BMP` (gyro fresco, deltas de encoder
      del ciclo). ✅ Aceptada. Ver §4.3.
- [x] **D3** *(revisado)* Alcance hasta el **EKF**; Fase 3 pasa a **objetivo principal** (ya no aparcada). ✅
- [x] **D4** Acelerómetro **incluido ya** (robot en desarrollo). ⚠️ *Revocado por D7.*
- [x] **D5** **Migrar todo** lo no necesario a odometría y eliminar getters sobrantes. ✅
- [x] **D6** **Descartar** la compensación de inclinación (pista nivelada; inclinaciones solo en rectas con
      `ω≈0`). ✅
- [x] **D7** **Eliminar el acelerómetro** por completo (sustituye a D4): no aporta al rumbo en pista
      nivelada y añade complejidad. ✅
- [x] **D8** **Base de medida sin filtros software** de cara al EKF: el EKF *es* el filtro; se conserva el
      LPF1 solo como antialias (o se baja el ODR a 833 Hz sin LPF1). ✅ (planificado en F3.0)
- [x] **D9** **El estimador de modelo sustituye al filtro**: α-β → EKF. Un pasa-bajos causal siempre añade
      retraso; el EKF cambia ruido por modelo, no por lag. ✅
- [x] **D10** Ir **directo al EKF** sin α-β ni *fallback* de estimación simple (el robot está en desarrollo;
      no se necesita estabilidad incremental). ✅
- [x] **D11** **Descartar la fusión absoluta con la línea** (F3.3): sin mapa, la línea no aporta posición
      absoluta. Con vueltas múltiples sería *loop closure* (SLAM-lite), que se trata como idea futura (§12). ✅

---

## 10. Matriz de implicaciones por fase

| Fase | Archivos tocados | Riesgo | Reversible | Depende de | Estado |
|------|------------------|--------|------------|------------|--------|
| 0 | `encoders.c`, `lsm6dsr.c`, `control.c` | Bajo | Sí (git) | D1 | ✅ Compila; validación HW pendiente |
| 1 | + `odometry.*`, `sys_tick.c`, `sram.*`, `main.c` | Medio | Sí | F0, D2, D5 | ✅ Compila; validación HW pendiente |
| 2 | `odometry.c`, `lsm6dsr.c`, `sram.*` | Medio | Sí | F1, D4 | ✅ Compila; validación HW pendiente |
| 3 | `odometry.c`, `lsm6dsr.c` (filtros), `sram.*` | Alto | Sí | F2, D8, D9 | ✅ Cerrada: F3.0/F3.2/F3.4/F3.5; F3.3 cancelada |
| 4 | `config.h`, `lsm6dsr.c` (auto-cal), `tools/*`, este ROADMAP | Bajo | Sí | F0–F2 | ⬜ Prioridad alta |

---

## 11. Referencias de código

- `src/encoders.c` — deltas de ticks, distancias y velocidades.
- `src/odometry.c`, `include/odometry.h` — **núcleo**: pose, fusión, estado (x/y/θ/v/ω/bias/slip).
- `src/lsm6dsr.c` — driver gyro: calibración/bias, lectura raw, filtro, data-ready, escala.
- `src/control.c` — control, y logging de odometría.
- `src/sys_tick.c`, `include/sys_tick.h` — scheduler de slots.
- `include/constants.h`, `include/config.h` — frecuencias y constantes físicas.
- `include/sram.h`, `src/sram.c` — esquema y volcado de logs.
- `src/config.c` — calibración de geometría en runtime (µm/tick, separación).
- `src/calibrations.c` — rutinas de calibración (gyro, sensores, separación, escala).
- `tools/parse_odometry_log.py` — parseo/análisis del log de odometría.
- `lib/lsm6dsr_reg/lsm6dsr_reg.h` — registros LSM6DSR.

---

## 12. Ideas futuras (aparcadas — no forma parte de Fases 1–4)

Estas capacidades irían **más allá del EKF** y no se implementan ahora. Se anotan para no perderlas.

### 12.1 Cierre de bucle / mapa incremental (SLAM-lite)
- **Cuándo aplica:** solo con **vueltas múltiples** (linefollower con 2–3 vueltas o persecución;
  robotracer es 1 vuelta de reconocimiento + vueltas de run). Sin revisitas no hay información de posición
  absoluta.
- **Idea:** grabar geometría en la 1ª pasada (distancias entre landmarks) y, al reobservar, **corregir la
  deriva acumulada** (grafo de poses). A partir de ahí, línea y marcas vuelven a ser referencia **absoluta**
  relativa al mapa construido.
- **Coste:** alto (asociación de datos, optimización). Empezar por lo más simple: **reset puntual** al
  re-detectar el inicio/marca conocida.

### 12.2 Landmarks con sensores de marcas (robotracer)
- Las marcas laterales (hoy **sin conectar**) son **anclas discretas**: cada cruce es un reseteo puntual de
  posición/along-track y una frontera de sección. Es "posicionamiento por landmarks", no fusión continua.
- Requiere: conectar los sensores, definir la convención de marcas (lado, orden) y resolver la asociación.

### 12.3 Consistencia línea ↔ (ω/v) para derrape
- Los sensores de línea ven la **curvatura de la pista**; con `κ ≈ ω/v` se obtiene una señal de **subviraje**
  independiente de la comparación gyro↔encoders. No corrige posición; mejora la detección de derrape.
- Opcional y ligero; se puede añadir al detector de `slip` existente.

### 12.4 Referencia de rumbo absoluta
- Para rumbo absoluto haría falta **magnetómetro** (no disponible). Sin él, el rumbo se corrige solo por
  encoders/landmarks, no de forma continua.
