#include "battery.h"
#include "bmp280.h"
#include "buttons.h"
#include "odometry.h"
#include "setup.h"
#include "sram.h"

int main(void) {
  setup();
  eeprom_load();
  handle_robot_version();
  show_battery_level();

  sensors_calibration(false);

  while (1) {

    //     printf("L: %4d mA ", get_motor_current_raw(0));
    // printf("R: %4d mA ", get_motor_current_raw(1));
    // printf(" | ");
    // printf("L: %5.2f m/s ", get_encoder_left_speed());
    // printf("R: %5.2f m/s ", get_encoder_right_speed());
    // printf("\n");
    // printf("z-rad/s: %7.4f z-angle: %5.2f z-bias:%5.6f\n", odometry_get_angular_speed(), odometry_get_theta_deg(), odometry_get_gyro_bias());

    // printf("theta_deg: %4.2f gyro_bias: %4.2f dt: %4.4f\n",odometry_get_theta_deg(), odometry_get_gyro_bias(), odometry_get_dt());
    // printf("encoder_omega: %4.2f measured_angular_speed %4.2f\n", odometry_get_encoder_omega(), odometry_get_angular_speed());
    // printf("theta_deg: %4.2f encoder_left_micrometers: %4ld encoder_right_micrometers: %4ld\n", odometry_get_theta_deg(), get_encoder_left_micrometers(), get_encoder_right_micrometers());

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
              set_target_fan_speed(get_kinematics().fan_speed, 500);
              set_line_sensors_correction_enabled(true);
            }
          }
          while (get_ideal_fan_speed() < get_kinematics().fan_speed) {
            set_RGB_rainbow();
          }
          set_RGB_color(0, 0, 0);
          delay(100);

          reset_encoders();
          odometry_reset();
          set_target_linear_speed(get_kinematics().linear_speed);
          // set_target_fan_speed(get_kinematics().fan_speed, 500);
          // delay(250);
          // set_target_linear_speed(0);
          // delay(250);
          // set_race_started(false);
        }
      }
    } else {
      // TODO loop de carrera
      // set_race_started(false);
    }
  }
}