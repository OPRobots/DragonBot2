#include "battery.h"
#include "bmp280.h"
#include "buttons.h"
#include "setup.h"
#include "sram.h"

void sys_tick_handler(void) {
  clock_tick();
  check_buttons();
  update_battery_voltage();
  update_encoder_readings();
}

int main(void) {
  setup();
  eeprom_load();
  handle_robot_version();
  show_battery_level();

  while (1) {

    if (!is_race_started()) {
      menu_handler();
      if (menu_run_can_start()) {

        if (is_race_started()) {
          // TODO: espera antes de iniciar
          // TODO: iniciar run segun tipo de competición
          // TODO: preparar kinematics
          // TODO: establecer velocidad ventiladores
        }
      }
    } else {
      // TODO loop de carrera
      set_race_started(false);
    }
  }

  //? ZONA DEBUG TEMPORAL
  while (1) {
    // set_status_led(true);
    // set_info_leds();
    // set_RGB_rainbow();
    // set_RGB_color(0, 0, 0);
    set_motors_speed(0, 0);
    set_leds_mpu(75);
    // set_info_led(INFO_LED_2, true);
    // set_info_led(INFO_LED_B, true);
    // gpio_set(GPIOA, GPIO11);
    // gpio_clear(GPIOA, GPIO10);
    // gpio_set(GPIOA, GPIO9);
    // gpio_clear(GPIOA, GPIO8);
    // set_fan_speed(0);

    // printf("Battery voltage: %.2f V\n", get_battery_voltage());
    // debug_from_main(DEBUG_TYPE_SENSORS_RAW);
    // debug_from_main(DEBUG_TYPE_ENCODERS);
    // debug_from_main(DEBUG_TYPE_MOTORS);
    // printf("MPU WhoAmI: 0x%02X\n", lsm6dsr_who_am_i());

    // sram_set_mode();
    // printf("SRAM Mode: 0x%02X\n", sram_get_mode());

    // printf("BMP280 WhoAmI: 0x%02X\n", bmp280_who_am_i());

    printf("SELECT: %d UP: %d DOWN: %d MODE: %d MM: %ld\n", get_menu_select_btn(), get_menu_up_btn(), get_menu_down_btn(), get_menu_mode_btn(), get_encoder_left_millimeters());
    delay(150);
  }
}