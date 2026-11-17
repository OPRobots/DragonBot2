#include "battery.h"
#include "bmp280.h"
#include "buttons.h"
#include "dragrace.h"
#include "linefollower.h"
#include "odometry.h"
#include "robotracer.h"
#include "setup.h"
#include "sram.h"

int main(void) {
  setup();
  eeprom_load();
  handle_robot_version();
  show_battery_level();

  sensors_calibration(false);

  while (1) {
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

          switch (menu_run_get_run_algorithm()) {
            case RUN_LINEFOLLOWER:
              linefollower_start();
              break;
            case RUN_ROBOTRACER:
              robotracer_start();
              break;
            case RUN_DRAGRACE:
              dragrace_start();
              break;
            default:
              set_race_started(false);
              warning_status_led_timeout(50, 2000);
              break;
          }
        }
      }
    } else {
      switch (menu_run_get_run_algorithm()) {
        case RUN_LINEFOLLOWER:
          // linefollower_loop();
          break;
        case RUN_ROBOTRACER:
          robotracer_loop();
          break;
        case RUN_DRAGRACE:
          dragrace_loop();
          break;
        default:
          set_race_started(false);
          break;
      }
    }
  }
}