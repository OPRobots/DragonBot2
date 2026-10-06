#include <calibrations.h>

bool calibration_enabled = false;

static void check_calibration_active(void) {
  if (get_menu_select_btn()) {
    calibration_enabled = !calibration_enabled;
  }
}

bool is_calibration_enabled(void) {
  return calibration_enabled;
}

void calibrate_from_config(uint8_t type) {
  if (type != CALIBRATE_NONE) {
    check_calibration_active();
  } else {
    calibration_enabled = false;
  }
  if (calibration_enabled) {
    clear_info_leds();
    set_RGB_color(0, 125, 0);
    delay(2000);
    switch (type) {
      case CALIBRATE_GYRO_Z:
        lsm6dsr_gyro_z_calibration();
        break;
      case CALIBRATE_SENSORS:
        sensors_calibration(true);
        break;
      case CALIBRATE_STORE_EEPROM:
        eeprom_save();
        break;
    }
    set_RGB_color(0, 0, 0);
    reset_encoders();
    calibration_enabled = false;
    menu_config_reset_values();
  }
}