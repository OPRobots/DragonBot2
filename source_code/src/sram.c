#include "sram.h"

#define SRAM_SIZE_BYTES (512UL * 1024UL)

#define SRAM_MODE_BYTE 0x00
#define SRAM_MODE_SEQUENTIAL 0x40
#define SRAM_MODE_PAGE 0x80

#define SRAM_CMD_READ 0x03
#define SRAM_CMD_WRITE 0x02
#define SRAM_CMD_RDMR 0x05
#define SRAM_CMD_WRMR 0x01

static void sram_select(void) {
  gpio_clear(GPIOC, GPIO4);
  delay_us(1);
}

static void sram_deselect(void) {
  gpio_set(GPIOC, GPIO4);
  delay_us(1);
}

static uint8_t sram_transfer(uint8_t value) {
  // spi_send(SPI3, value);
  // return spi_read(SPI3);
  return spi_xfer(SPI3, value);
}

void sram_set_mode() {
  sram_select();
  sram_transfer(SRAM_CMD_WRMR);
  sram_transfer(SRAM_MODE_SEQUENTIAL);
  sram_deselect();
}

uint8_t sram_get_mode(void) {
  uint8_t mode;

  sram_select();
  sram_transfer(SRAM_CMD_RDMR);
  mode = sram_transfer(0x00);
  sram_deselect();

  return mode;
}