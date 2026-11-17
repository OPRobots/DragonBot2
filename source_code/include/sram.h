#ifndef SRAM_H
#define SRAM_H

#include <stdbool.h>
#include <stdint.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#include "constants.h"
#include "delay.h"
#include "sys_tick.h"

/* Tamaño del ring en RAM, en bytes. DEBE ser potencia de 2 y <= 32768. */
#define SRAM_LOG_RING_SIZE 4096u

/* Payload máximo aceptado por registro. Debe cubrir el mayor struct de log. */
#define SRAM_LOG_MAX_PAYLOAD 80u

/* Número máximo de registros que se vuelcan en una pasada del flusher. */
#define SRAM_LOG_FLUSH_BATCH_MAX 8u

/* Presupuesto de tiempo por slot de volcado (us). Debe dejar margen en el
 * slot del SysTick (125 us) para no desbordar al siguiente. */
#define SRAM_LOG_FLUSH_BUDGET_US 90u
#define SRAM_LOG_FLUSH_BUDGET_CYCLES ((SYSCLK_FREQUENCY_HZ / 1000000u) * SRAM_LOG_FLUSH_BUDGET_US)

/* Retardo (us) en los flancos de CS de la SRAM. 0 = sin retardo (más rápido).
 * Si la SRAM diera errores, subir a 1. */
#ifndef SRAM_CS_DELAY_US
#define SRAM_CS_DELAY_US 0u
#endif

/* ─────────────────────────────────────────────────────────────────────────
 * Tipos de registro y estructuras de log
 * ───────────────────────────────────────────────────────────────────────── */

typedef uint8_t sram_record_type_t;
enum {
  SRAM_TYPE_ANY = 0x00,
  SRAM_TYPE_CONTROL = 0x01,
  SRAM_TYPE_ODOMETRY = 0x02,
  SRAM_TYPE_YAW_STEP_DEBUG = 0x03,
  SRAM_TYPE_ODOMETRY_DEBUG = 0x04,
  SRAM_TYPE_CONTROL_DEBUG = 0x05,
  SRAM_TYPE_LOG_MACROARRAY = 0x06,
  /* ── USER TYPES (0x07 – 0xFE) ── */
  SRAM_TYPE_INVALID = 0xFE,
};

/* Todos los structs de log empiezan por timestamp_ms; sram_log lo rellena. */

/* Log de control ligero: setpoints, medida y estado básico. */
typedef struct __attribute__((packed)) {
  uint32_t timestamp_ms;
  int16_t target_linear_speed;
  int16_t ideal_linear_speed;
  float measured_linear_speed;
  float ideal_angular_speed;
  float measured_angular_speed;
  int32_t line_offset;
  float battery_voltage;
} control_log_t;

/* Log de control completo: añade ruedas, PWM y tensión angular. */
typedef struct __attribute__((packed)) {
  uint32_t timestamp_ms;
  int16_t target_linear_speed;
  int16_t ideal_linear_speed;
  float measured_linear_speed;
  float left_measured_linear_speed;
  float right_measured_linear_speed;
  float ideal_angular_speed;
  float measured_angular_speed;
  int16_t pwm_left;
  int16_t pwm_right;
  float angular_voltage;
  int32_t line_offset;
  float battery_voltage;
} control_debug_log_t;

/* Log de odometría ligero: posición, rumbo y estado básico. */
typedef struct __attribute__((packed)) {
  uint32_t timestamp_ms;
  float position_x;
  float position_y;
  float angle;
  float linear_speed;
  float angular_speed;
  int32_t line_offset;
  uint8_t slip;
} odometry_log_t;

/* Log de odometría completo: fusión, encoders, gyro, bias, dt y varianzas. */
typedef struct __attribute__((packed)) {
  uint32_t timestamp_ms;
  float left_measured_linear_speed;
  float right_measured_linear_speed;
  float measured_angular_speed;
  int32_t left_millimeters;
  int32_t right_millimeters;
  float position_x;
  float position_y;
  float angle;
  float linear_speed;
  float encoder_omega;
  float encoder_v;
  float gyro_rate;
  float gyro_bias;
  uint8_t slip;
  float dt;
  int32_t line_offset;
  float var_x;
  float var_y;
  float var_theta;
} odometry_debug_log_t;

/* Test T1 (escalón de guiñada en el sitio): una muestra por ms. */
typedef struct __attribute__((packed)) {
  uint32_t timestamp_ms;
  uint16_t rep;
  int16_t pwm; /* 0 en la base, YAW_STEP_DEBUG_PWM durante el escalón */
  float gyro_rate;
} yaw_step_debug_log_t;

/**
 * Log comodín. Renombrar field1..field4 y LOG_MACROARRAY_LABELS al usarlo.
 * Si se añaden o quitan campos, actualizar LOG_MACROARRAY_FIELDS, el struct
 * anónimo y las etiquetas; los static_assert de sram.c validan el tamaño.
 * Evitar nombres reservados por otros logs: pos_x, pos_y, angle,
 * ideal_linear_speed, pwm_left, pwm_right, etc.
 *
 * Ejemplo:
 *   log_macroarray_t log = {
 *       .field1 = 123,
 *       .field2 = -45,
 *       .field3 = 500,
 *       .field4 = 480,
 *   };
 *   sram_log(SRAM_TYPE_LOG_MACROARRAY, &log, sizeof(log));
 */
#define LOG_MACROARRAY_FIELDS 4
#define LOG_MACROARRAY_LABELS {"field1", "field2", "field3", "field4"}
typedef struct __attribute__((packed)) {
  uint32_t timestamp_ms;
  union {
    struct {
      int32_t field1;
      int32_t field2;
      int32_t field3;
      int32_t field4;
    };
    int32_t field[LOG_MACROARRAY_FIELDS];
  };
} log_macroarray_t;

/*
 * Buffer de interpretación de registros SRAM.
 * Agregar aquí todo struct de log nuevo para que el buffer crezca solo;
 * sram_print_logs detecta y reporta registros más grandes que este union.
 */
typedef union {
  control_log_t control;
  odometry_log_t odometry;
  control_debug_log_t control_debug;
  odometry_debug_log_t odometry_debug;
  yaw_step_debug_log_t yaw_step_debug;
  log_macroarray_t log_macroarray;
} sram_log_t;

typedef enum {
  SRAM_OK = 0,
  SRAM_ERR_FULL = -1,
  SRAM_ERR_NOT_INIT = -2,
  SRAM_ERR_PARAM = -3,
} sram_result_t;

typedef struct {
  sram_record_type_t filter;
  uint32_t _pos;
  uint16_t _remaining;
} sram_iterator_t;

/* ─────────────────────────────────────────────────────────────────────────
 * API de bajo nivel (acceso directo a la SRAM)
 * ───────────────────────────────────────────────────────────────────────── */

void sram_init(void);
void sram_format(void);

void sram_iterator_init(sram_iterator_t *iterator, sram_record_type_t type);
bool sram_iter_next(sram_iterator_t *iterator, void *buf, uint16_t buf_size, uint16_t *out_size, sram_record_type_t *out_type);

void sram_print_logs(sram_record_type_t type);

uint16_t sram_get_total_records(void);
uint16_t sram_count_records(sram_record_type_t type);
char *sram_free_bytes(void);

/**
 * @brief Encola un registro de log.
 *
 * El payload debe empezar por un `uint32_t timestamp_ms`. sram_log lo rellena
 * automáticamente con get_log_timestamp(); el valor que traiga @p data en esos
 * primeros 4 bytes se ignora.
 *
 * @return true si se ha encolado el registro.
 */
bool sram_log(sram_record_type_t type, const void *data, uint16_t size);

uint16_t sram_log_pending(void);
uint16_t sram_log_dropped(void);

/**
 * @brief Vuelca a la SRAM los registros pendientes del ring.
 * Se ejecuta desde los slots de volcado del SysTick. Escribe registros hasta
 * vaciar el ring, agotar @p budget_cycles o alcanzar SRAM_LOG_FLUSH_BATCH_MAX.
 */
void sram_store(uint32_t budget_cycles);

void sram_log_set_paused(bool paused);
bool sram_log_is_paused(void);

/* ─────────────────────────────────────────────────────────────────────────
 * Diagnóstico / test
 * ───────────────────────────────────────────────────────────────────────── */

/** @brief Recorre los registros validando tipos/tamaños y el header. */
bool sram_debug_validate(void);

/**
 * @brief Test de overflow: formatea, escribe @p records registros sinteticos
 * (mas que la capacidad, forzando wrap/overwrite) y valida el resultado.
 * Ejercita el camino real (SPI incluido) sin poner el robot en pista.
 */
bool sram_debug_overflow_test(uint32_t records);

#endif
