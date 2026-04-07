#ifndef BMP280_H
#define BMP280_H

#include <stdint.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

uint8_t bmp280_who_am_i(void);

#endif