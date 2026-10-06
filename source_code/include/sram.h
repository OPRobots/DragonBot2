#ifndef SRAM_H
#define SRAM_H

#include <stdbool.h>
#include <stdint.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#include "delay.h"

typedef uint8_t sram_record_type_t;
enum {
  SRAM_TYPE_ANY = 0x00,
  SRAM_TYPE_SENSOR = 0x01,
  /* ── USER TYPES (0x03 – 0xFE) ── */
  SRAM_TYPE_INVALID = 0xFE,
};

typedef struct __attribute__((packed)) {
  uint32_t timestamp_ms;
  uint16_t line_sensors[24];
  int16_t speed_left;
  int16_t speed_right;
  uint8_t battery_pct;
} sensor_log_t;

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

void sram_init(void);
void sram_format(void);

sram_result_t sram_write_record(sram_record_type_t type, const void *data, uint16_t size);

void sram_iterator_init(sram_iterator_t *iterator, sram_record_type_t type);
bool sram_iter_next(sram_iterator_t *iterator, void *buf, uint16_t buf_size, uint16_t *out_size);

uint16_t sram_get_total_records(void);
uint16_t sram_count_records(sram_record_type_t type);
char *sram_free_bytes(void);

// void sram_test(void);

#endif