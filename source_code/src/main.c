#include "battery.h"
#include "bmp280.h"
#include "sram.h"

void sys_tick_handler(void) {
  clock_tick();
  update_battery_voltage();
}

int main(void) {

  setup();
  eeprom_load();
  handle_robot_version();
  show_battery_level();
  // sram_init();

  while (1) {
    // set_status_led(true);
    // set_info_leds();
    // set_RGB_rainbow();
    // set_RGB_color(0, 0, 0);
    // set_motors_speed(1000, 1000);
    // set_leds_wave(75);
    // gpio_set(GPIOA, GPIO11);
    // gpio_clear(GPIOA, GPIO10);
    // gpio_set(GPIOA, GPIO9);
    // gpio_clear(GPIOA, GPIO8);
    // set_fan_speed(0);

    // printf("Battery voltage: %.2f V\n", get_battery_voltage());
    // debug_from_main(DEBUG_TYPE_SENSORS_RAW);
    // debug_from_main(DEBUG_TYPE_MOTORS);
    // printf("MPU WhoAmI: 0x%02X\n", lsm6dsr_who_am_i());

    sram_set_mode();
    printf("SRAM Mode: 0x%02X\n", sram_get_mode());

    // printf("BMP280 WhoAmI: 0x%02X\n", bmp280_who_am_i());
    delay(150);
  }
}