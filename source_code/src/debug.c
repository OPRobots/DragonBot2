#include <debug.h>

bool debug_enabled = false;
uint32_t debug_last_print_ms = 0;

static void debug_macroarray(void) {
  macroarray_print();
  debug_enabled = false;
  menu_config_reset_values();
}

/**
 * @brief Imprime los valores de los sensores sin aplicar ninguna corrección
 *
 */
static void debug_sensors_raw(void) {
  if (get_clock_ticks() > debug_last_print_ms + 50) {
    for (uint8_t sensor = 0; sensor < get_line_sensor_count(); sensor++) {
      printf("%4d ", get_line_sensor_raw(sensor));
    }
    printf("| ");
    for (uint8_t sensor = 0; sensor < get_mark_sensor_count(); sensor++) {
      printf("%4d ", get_mark_sensor_raw(sensor));
    }
    printf("\n");
    debug_last_print_ms = get_clock_ticks();
  }
}

static void debug_sensors_calibrated(void) {
  if (get_clock_ticks() > debug_last_print_ms + 50) {
    for (uint8_t sensor = 0; sensor < get_line_sensor_count(); sensor++) {
      printf("S%02d: %4d\t", sensor + 1, get_line_sensor_calibrated(sensor));
    }
    for (uint8_t sensor = 0; sensor < get_mark_sensor_count(); sensor++) {
      printf("M%02d: %4d\t", sensor + 1, get_mark_sensor_calibrated(sensor));
    }
    printf("\n");
    debug_last_print_ms = get_clock_ticks();
  }
}

static void debug_all_leds(void) {
  set_RGB_rainbow();
  warning_status_led(125);
  set_leds_wave(50);
}

static void debug_posicion_correccion(void) {
  if (get_clock_ticks() > debug_last_print_ms + 50) {
    line_sensors_update_position();
    printf("%ld\n", get_line_sensor_position());
    debug_last_print_ms = get_clock_ticks();
  }
}

static void debug_line_position(void) {
  if (get_clock_ticks() > debug_last_print_ms + 50) {
    line_sensors_update_position();
    printf("%d\t%ld\t%d\n", -1200, get_line_sensor_position(), 1200);
    debug_last_print_ms = get_clock_ticks();
  }
}

static void debug_encoders(void) {
  if (get_clock_ticks() > debug_last_print_ms + 50) {
    printf("%ld (%ld)\t%ld (%ld)\n", get_encoder_left_ticks(), get_encoder_left_micrometers(), get_encoder_right_ticks(), get_encoder_right_micrometers());
    debug_last_print_ms = get_clock_ticks();
  }
}

static void debug_motors(void) {
  set_motors_speed(150, 150);
  if (get_clock_ticks() > debug_last_print_ms + 50) {
    printf("Motor Current: ");
    for (uint8_t i = 0; i < MOTOR_CURRENT_COUNT; i++) {
      if (i < MOTOR_CURRENT_COUNT / 2) {
        printf("L%d: ", i);
      } else {
        printf("R%d: ", i - MOTOR_CURRENT_COUNT / 2);
      }
      printf("%4d mA  ", get_motor_current_raw(i));
    }
    printf("\n");
    debug_last_print_ms = get_clock_ticks();
  }
}

static void debug_fans(void) {
  set_fan_speed(30);
}

static void check_debug_active(void) {
  if (is_right_wheel_click_down()) {
    debug_enabled = !debug_enabled;
    if (!debug_enabled) {
      menu_config_reset_values();
    }
  }
}

bool is_debug_enabled(void) {
  return debug_enabled;
}

void debug_from_config(uint8_t type) {
  if (type != DEBUG_NONE) {
    check_debug_active();
  } else {
    debug_enabled = false;
  }
  if (debug_enabled) {
    set_RGB_color(0, 50, 0);
    switch (type) {
      case DEBUG_MACROARRAY:
        debug_macroarray();
        break;
      case DEBUG_TYPE_SENSORS_RAW:
        debug_sensors_raw();
        break;
      case DEBUG_TYPE_SENSORS_CALIBRATED:
        debug_sensors_calibrated();
        break;
      case DEBUG_TYPE_LINE_POSITION:
        debug_line_position();
        break;
      case DEBUG_TYPE_MOTORS:
        debug_motors();
        break;
      case DEBUG_TYPE_ENCODERS:
        debug_encoders();
        break;
      case DEBUG_TYPE_CORRECCION_POSICION:
        debug_posicion_correccion();
        break;
      case DEBUG_TYPE_LEDS_PARTY:
        debug_all_leds();
        break;
      case DEBUG_TYPE_FANS_DEMO:
        debug_fans();
        break;
    }
  } else {
    set_RGB_color(0, 0, 0);
  }
}

void debug_from_main(uint8_t type) {
  debug_enabled = true;
  debug_from_config(type);
}