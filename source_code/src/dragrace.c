#include "dragrace.h"

void dragrace_start(void) {

  //TODO: start "disimulado"

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
}

void dragrace_loop(void) {
  //TODO: detención automática después de cierta distancia recorrida
}