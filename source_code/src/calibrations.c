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

/**
 * @brief Calibra WHEELS_SEPARATION girando en el sitio hasta acumular 2*pi.
 *
 * Girado lento y sobre el suelo (las ruedas deben rodar sin derrapar), el gyro
 * mide el ángulo girado y los encoders el diferencial recorrido:
 *   sep = (derecha - izquierda) / theta
 *
 * El ángulo se integra AQUÍ, localmente, a partir del gyro (no se usa el EKF),
 * para que la calibración no dependa de la dinámica del estimador.
 */
static void calibrate_wheel_separation(void) {
  set_control_paused(true);

  set_motors_speed(-50, 50);
  set_info_leds();
  delay(500);
  reset_encoders();

  float theta = 0.0f;
  uint32_t last_cycles = read_cycle_counter();
  uint32_t start_ms = get_clock_ticks();
  uint32_t next_print_ms = 250;

  while (fabsf(theta) < 2.0f * (float)PI) {
    uint32_t now = read_cycle_counter();
    float dt = (float)(now - last_cycles) / (float)SYSCLK_FREQUENCY_HZ;
    last_cycles = now;
    if (dt > 0.0f && dt < 0.05f) {
      theta += lsm6dsr_get_gyro_z_radps() * dt;
    }

    uint32_t elapsed = get_clock_ticks() - start_ms;
    if (elapsed >= next_print_ms) {
      next_print_ms += 250;
      printf("sep cal: theta=%.3f rad, enc_delta=%.0f um\n", theta,
             (double)(get_encoder_right_micrometers() - get_encoder_left_micrometers()));
    }
    if (elapsed > 15000) {
      break;
    }
  }

  float measured_theta = theta;
  float delta_um = (float)(get_encoder_right_micrometers() - get_encoder_left_micrometers());

  set_motors_speed(0, 0);
  set_motors_brake();
  clear_info_leds();
  set_control_paused(false);

  float separation = (fabsf(measured_theta) > 1.0f) ? fabsf(delta_um / measured_theta) / 1000.0f : 0.0f;

  while (!get_menu_select_btn()) {
    warning_status_led(125);
  }
  set_status_led(false);
  delay(500);

  if (separation > 10.0f && separation < 300.0f) {
    config_set_wheels_separation(separation);
    int16_t stored = (int16_t)(separation * 100.0f);
    eeprom_set_data(DATA_INDEX_WHEEL_SEPARATION, &stored, 1);
    printf("theta= %.3frad, delta= %.1fum -> separation= %.3fmm\n", measured_theta, delta_um, separation);
  } else {
    uint32_t warning_start_ms = get_clock_ticks();
    while (get_clock_ticks() - warning_start_ms < 2000) {
      warning_status_led(50);
    }
  }
}

/**
 * @brief Calibra MICROMETERS_PER_TICK moviendo el robot una distancia patrón.
 *
 * El robot debe recorrerse exactamente WHEEL_SCALE_CALIBRATION_MM desde el reset
 * (que ocurre al arrancar esta rutina) hasta pulsar MENU.
 */
static void calibrate_wheel_scale(void) {
  reset_encoders();
  set_RGB_color(0, 0, 0);

  uint32_t start_ms = get_clock_ticks();
  while (!get_menu_select_btn()) {
    set_info_leds();
    if (get_clock_ticks() - start_ms > 60000) {
      break;
    }
  }
  float avg_ticks = 0.5f * (float)(get_encoder_left_ticks() + get_encoder_right_ticks());
  float micrometers_per_tick = (fabsf(avg_ticks) > 100.0f) ? WHEEL_SCALE_CALIBRATION_MM * 1000.0f / fabsf(avg_ticks) : 0.0f;
  while (get_menu_select_btn()) {
  }
  clear_info_leds();

  while (!get_menu_select_btn()) {
  }

  if (micrometers_per_tick > 10.0f && micrometers_per_tick < 1000.0f) {
    config_set_micrometers_per_tick(micrometers_per_tick);
    int16_t stored = (int16_t)(micrometers_per_tick * 100.0f);
    eeprom_set_data(DATA_INDEX_WHEEL_SCALE, &stored, 1);
    printf("avg_ticks=%.0f -> um/tick=%.3f\n", avg_ticks, micrometers_per_tick);
  } else {
    uint32_t warning_start_ms = get_clock_ticks();
    while (get_clock_ticks() - warning_start_ms < 2000) {
      warning_status_led(50);
    }
  }
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
      case CALIBRATE_WHEEL_SEPARATION:
        calibrate_wheel_separation();
        break;
      case CALIBRATE_WHEEL_SCALE:
        calibrate_wheel_scale();
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