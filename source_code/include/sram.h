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
#define SRAM_LOG_MAX_PAYLOAD 64u

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
  /* ── USER TYPES (0x03 – 0xFE) ── */
  SRAM_TYPE_INVALID = 0xFE,
};

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
  float battery_voltage;
} control_log_t;

/*
 * Buffer de interpretación de registros SRAM.
 * Agregar aquí todo struct de log nuevo para que el buffer crezca solo;
 * sram_print_logs detecta y reporta registros más grandes que este union.
 */
typedef union {
  control_log_t control;
  /* sensor_log_t sensor; */
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

#endif
