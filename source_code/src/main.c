#include "battery.h"
#include "bmp280.h"
#include "buttons.h"
#include "setup.h"
#include "sram.h"

int main(void) {
  setup();
  eeprom_load();
  handle_robot_version();
  show_battery_level();

  // sensors_calibration(false);

  while (1) {

    //     printf("L: %4d mA ", get_motor_current_raw(0));
    // printf("R: %4d mA ", get_motor_current_raw(1));
    // printf(" | ");
    // printf("L: %5.2f m/s ", get_encoder_left_speed());
    // printf("R: %5.2f m/s ", get_encoder_right_speed());
    // printf("\n");
    // printf("z-rad/s: %7.4f z-angle: %5.2f\n", lsm6dsr_get_gyro_z_radps(), lsm6dsr_get_gyro_z_degrees());

    if (!is_race_started()) {
      menu_handler();
      if (menu_run_can_start()) {

        if (is_race_started()) {
          sram_format();
          menu_run_reset();
          set_RGB_color(0, 0, 0);
          set_status_led(false);
          clear_info_leds();
          configure_kinematics(menu_run_get_speed());
          delay(100);

          uint32_t start_ms = get_clock_ticks();
          uint16_t elapsed_ms = 5;
          while (get_clock_ticks() < (start_ms + get_start_millis())) {
            elapsed_ms = get_clock_ticks() - start_ms;
            uint8_t r = 0, g = 0;
            r = map(elapsed_ms, 0, get_start_millis(), 255, 0);
            g = map(elapsed_ms, 0, 1000, 0, 255);
            set_RGB_color(r, g, 0);
            if ((elapsed_ms > get_start_millis() * 0.75 || get_start_millis() == 0) && get_kinematics().fan_speed > 0) {
              set_target_fan_speed(get_kinematics().fan_speed / 2, 500);
            }
          }
          set_RGB_color(0, 0, 0);

          set_target_linear_speed(get_kinematics().linear_speed);
          // set_target_fan_speed(get_kinematics().fan_speed, 500);
          set_line_sensors_correction_enabled(false);
          delay(250);
          set_target_linear_speed(0);
          delay(250);
          set_race_started(false);
        }
      }
    } else {
      // TODO loop de carrera
      set_race_started(false);
    }
  }
}