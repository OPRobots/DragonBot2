#include "bmp280.h"

#define BMP280_CHIPID 0xD0

static void bmp280_select(void) {
  gpio_clear(GPIOB, GPIO15);
}

static void bmp280_deselect(void) {
  gpio_set(GPIOB, GPIO15);
}

static uint8_t bmp280_read_reg(uint8_t reg) {
  bmp280_select();
  spi_xfer(SPI3, reg | 0x80);
  uint8_t val = spi_xfer(SPI3, 0xFF);
  bmp280_deselect();
  return val;
}

static void bmp280_write_reg(uint8_t reg, uint8_t value) {
  bmp280_select();
  spi_xfer(SPI3, reg & 0x7F);
  spi_xfer(SPI3, value);
  bmp280_deselect();
}

uint8_t bmp280_who_am_i(void) {
  return bmp280_read_reg(BMP280_CHIPID);
}