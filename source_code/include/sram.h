#ifndef SRAM_H
#define SRAM_H
/*Model: 23AA04M */

#include <stdbool.h>
#include <stdint.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#include "delay.h"

void sram_set_mode(void);
uint8_t sram_get_mode(void);

#endif