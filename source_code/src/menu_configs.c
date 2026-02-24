#include <menu_configs.h>

#define MODE_CALIBRATION 0
#define MODE_DEBUG 1
uint8_t modeConfig = MODE_CALIBRATION;

#define NUM_MODES 2

int8_t valueConfig[NUM_MODES] = {0, 0};
#define NUM_VALUES_CALIBRATION 4
#define NUM_VALUES_DEBUG 10

/**
 * @brief Indicar el tipo de menú que está actualmente activo mediante el led de estado
 *
 */
static void handle_menu_config_mode(void) {
  switch (modeConfig) {
    case MODE_CALIBRATION:
      warning_status_led(125);
      break;
    case MODE_DEBUG:
      set_status_led(true);
      break;
  }
}

static void handle_menu_config_value(void) {
  switch (modeConfig) {
    case MODE_CALIBRATION:
      set_RGB_color(0, 0, 0);
      switch (valueConfig[modeConfig]) {
        case CALIBRATE_NONE:
          clear_info_leds();
          break;
        case CALIBRATE_GYRO_Z:
          set_leds_wave(120);
          break;
        case CALIBRATE_SENSORS:
          set_leds_side_sensors(120);
          break;
        case CALIBRATE_STORE_EEPROM:
          set_leds_blink(250);
          break;
      }
      calibrate_from_config(valueConfig[modeConfig]);
      break;
    case MODE_DEBUG:
      for (uint8_t i = 1; i <= NUM_VALUES_DEBUG; i++) {
        set_info_led(i - 1, i == valueConfig[modeConfig]);
      }
      debug_from_config(valueConfig[modeConfig]);
      break;
  }
}

static void menu_config_up(void) {
  valueConfig[modeConfig]++;
  switch (modeConfig) {
    case MODE_CALIBRATION:
      if (valueConfig[modeConfig] > NUM_VALUES_CALIBRATION) {
        valueConfig[modeConfig] = 0;
      }
      break;
    case MODE_DEBUG:
      if (valueConfig[modeConfig] > NUM_VALUES_DEBUG) {
        valueConfig[modeConfig] = 0;
      }
      break;
  }
  clear_info_leds();
}

static void menu_config_down(void) {
  if (valueConfig[modeConfig] > 0) {
    valueConfig[modeConfig]--;
  }

  clear_info_leds();
}

bool menu_config_handler(void) {
  handle_menu_config_mode();
  handle_menu_config_value();

  // Comprueba cambios del modo de configuración
  if (valueConfig[modeConfig] == 0) {
    if (is_right_wheel_click_up()) {
      modeConfig = (modeConfig + 1) % NUM_MODES;
      delay(50);
    }

    if (is_right_wheel_click_down()) {
      return true;
    }
  }

  // Comprueba aumento de valor de configuración

  if (is_left_wheel_click_up()) {
    menu_config_up();
  }
  if (is_left_wheel_click_down()) {
    menu_config_down();
  }
  return false;
}

void menu_config_reset_values(void) {
  valueConfig[MODE_CALIBRATION] = 0;
  valueConfig[MODE_DEBUG] = 0;
}

void menu_config_reset_mode(void) {
  modeConfig = MODE_CALIBRATION;
}