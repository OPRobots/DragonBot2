#include "control.h"

static volatile bool race_started = false;
static volatile uint32_t race_finish_ms = 0;

static volatile uint32_t emergency_stop_ms = 0;

static volatile bool control_debug = false;
static volatile bool control_paused = false;

static volatile int32_t target_linear_speed = 0;
static volatile int32_t ideal_linear_speed = 0;
static volatile float ideal_angular_speed = 0.0;

static volatile int32_t target_fan_speed = 0;
static volatile float ideal_fan_speed = 0;
static volatile float fan_speed_accel = 0;

static volatile float linear_voltage;
static volatile float angular_voltage;

static volatile float linear_error;
static volatile float sum_linear_error;
static volatile float last_linear_error;

static volatile float angular_error;
static volatile float sum_angular_error;
static volatile float last_angular_error;

static volatile bool angular_correction_enabled = false;
static volatile bool line_sensors_correction_enabled = false;

static volatile float line_sensors_error;
static volatile float last_line_sensors_error;
static volatile float sum_line_sensors_error;

static volatile float voltage_left;
static volatile float voltage_right;
static volatile int16_t pwm_left;
static volatile int16_t pwm_right;

/* Trigger de logs por distancia (uno por log para que cada funcion decida). */
typedef struct {
  int32_t last_mm;
  int32_t acc_mm;
  bool armed;
} log_trigger_t;

static log_trigger_t control_log_trigger;
static log_trigger_t odometry_log_trigger;

/**
 * @brief Convierte un valor de voltaje dado a su correspondiente PWM
 *
 * @param voltage
 * @return int32_t PWM a aplicar al motor
 */
static int16_t voltage_to_motor_pwm(float voltage) {
  return voltage / get_battery_voltage() * MOTORS_MAX_PWM;
}

// static int32_t percentage_to_fan_pwm(float percentage) {
//   return percentage > 0 ? (int32_t)constrain((get_battery_high_limit_voltage() / get_battery_voltage()) * percentage, percentage, 100.0f) : 0;
// }

/**
 * @brief Actualiza la velocidad lineal ideal en función de la velocidad lineal objetivo y la aceleración
 *
 */
static void update_ideal_linear_speed(void) {
  if (ideal_linear_speed < target_linear_speed) {
    int16_t accel = get_kinematics().linear_accel.accel_soft;
    if (get_kinematics().linear_accel.speed_hard == 0 || ideal_linear_speed < get_kinematics().linear_accel.speed_hard) {
      accel = get_kinematics().linear_accel.accel_hard;
    }
    // printf("target: %ld ideal: %ld accel: %d\n", target_linear_speed, ideal_linear_speed, accel);
    ideal_linear_speed += accel / CONTROL_FREQUENCY_HZ;
    if (ideal_linear_speed > target_linear_speed) {
      ideal_linear_speed = target_linear_speed;
    }
  } else if (ideal_linear_speed > target_linear_speed) {
    // printf("target: %ld ideal: %ld accel: %d\n", target_linear_speed, ideal_linear_speed, get_kinematics().linear_accel.break_accel);
    ideal_linear_speed -= get_kinematics().linear_accel.break_accel / CONTROL_FREQUENCY_HZ;
    if (ideal_linear_speed < target_linear_speed) {
      ideal_linear_speed = target_linear_speed;
    }
  }
}

static void update_fan_speed(void) {
  if (ideal_fan_speed < target_fan_speed) {
    ideal_fan_speed += fan_speed_accel / CONTROL_FREQUENCY_HZ;
    if (ideal_fan_speed > target_fan_speed) {
      ideal_fan_speed = target_fan_speed;
    }
  } else if (ideal_fan_speed > target_fan_speed) {
    ideal_fan_speed += fan_speed_accel / CONTROL_FREQUENCY_HZ;
    if (ideal_fan_speed < target_fan_speed) {
      ideal_fan_speed = target_fan_speed;
    }
  }
}

static float get_measured_linear_speed(void) {
  return (get_encoder_left_speed() + get_encoder_right_speed()) / 2.0f;
}

static float get_measured_angular_speed(void) {
  return lsm6dsr_get_gyro_z_radps();
}

/* ── Logs ligeros (perfil de carrera) ───────────────────────────────────── */

static void log_control_sample(void) {
  control_log_t log = {
      .target_linear_speed = (int16_t)target_linear_speed,
      .ideal_linear_speed = (int16_t)ideal_linear_speed,
      .measured_linear_speed = get_measured_linear_speed(),
      .ideal_angular_speed = ideal_angular_speed,
      .measured_angular_speed = get_measured_angular_speed(),
      .line_offset = get_line_sensor_position(),
      .battery_voltage = get_battery_voltage(),
  };
  sram_log(SRAM_TYPE_CONTROL, &log, sizeof(log));
}

static void log_odometry_sample(void) {
  odometry_log_t log = {
      .position_x = odometry_get_x(),
      .position_y = odometry_get_y(),
      .angle = odometry_get_theta_deg(),
      .linear_speed = odometry_get_linear_speed(),
      .angular_speed = odometry_get_angular_speed(),
      .line_offset = get_line_sensor_position(),
      .slip = odometry_get_slip() ? 1 : 0,
  };
  sram_log(SRAM_TYPE_ODOMETRY, &log, sizeof(log));
}

/* ── Logs completos (perfil de depuración) ──────────────────────────────── */

static void log_control_debug_sample(void) {
  control_debug_log_t log = {
      .target_linear_speed = (int16_t)target_linear_speed,
      .ideal_linear_speed = (int16_t)ideal_linear_speed,
      .measured_linear_speed = get_measured_linear_speed(),
      .left_measured_linear_speed = get_encoder_left_speed(),
      .right_measured_linear_speed = get_encoder_right_speed(),
      .ideal_angular_speed = ideal_angular_speed,
      .measured_angular_speed = get_measured_angular_speed(),
      .pwm_left = pwm_left,
      .pwm_right = pwm_right,
      .angular_voltage = angular_voltage,
      .line_offset = get_line_sensor_position(),
      .battery_voltage = get_battery_voltage(),
  };
  sram_log(SRAM_TYPE_CONTROL_DEBUG, &log, sizeof(log));
}

static void log_odometry_debug_sample(void) {
  odometry_debug_log_t log = {
      .left_measured_linear_speed = get_encoder_left_speed(),
      .right_measured_linear_speed = get_encoder_right_speed(),
      .measured_angular_speed = get_measured_angular_speed(),
      .left_millimeters = get_encoder_left_millimeters(),
      .right_millimeters = get_encoder_right_millimeters(),
      .position_x = odometry_get_x(),
      .position_y = odometry_get_y(),
      .angle = odometry_get_theta_deg(),
      .linear_speed = odometry_get_linear_speed(),
      .encoder_omega = odometry_get_encoder_omega(),
      .encoder_v = odometry_get_encoder_v(),
      .gyro_rate = lsm6dsr_get_gyro_z_radps(),
      .gyro_bias = odometry_get_gyro_bias(),
      .slip = odometry_get_slip() ? 1 : 0,
      .dt = odometry_get_dt(),
      .line_offset = get_line_sensor_position(),
      .var_x = odometry_get_var_x(),
      .var_y = odometry_get_var_y(),
      .var_theta = odometry_get_var_theta(),
  };
  sram_log(SRAM_TYPE_ODOMETRY_DEBUG, &log, sizeof(log));
}

/**
 * @brief Comprueba si hay movimiento real en el robot
 *
 * @return true
 * @return false
 */
static bool log_is_moving(void) {
  return (ideal_linear_speed != 0) || (ideal_angular_speed != 0);
}

/**
 * @brief Reinicia el trigger; la primera actualización emitirá en el origen.
 *
 * @param trigger
 * @param distance_mm
 */
static void log_trigger_reset(log_trigger_t *trigger, int32_t distance_mm) {
  trigger->last_mm = 0;
  trigger->acc_mm = distance_mm;
  trigger->armed = false;
}

/**
 * @brief Actualiza el trigger por distancia. Devuelve true si toca emitir.
 *
 * Se arma en el primer ciclo con comando de movimiento, que es el origen real
 * de la pista (no durante el retardo de arranque). Resta el umbral al emitir
 * para no acumular error de redondeo.
 *
 * @param trigger
 * @param now_mm
 * @param distance_mm
 * @return true
 * @return false
 */
static bool log_trigger_update(log_trigger_t *trigger, int32_t now_mm, int32_t distance_mm) {
  if (!trigger->armed) {
    if (!log_is_moving()) {
      return false;
    }
    trigger->armed = true;
    trigger->last_mm = now_mm;
    trigger->acc_mm = distance_mm;
  }

  int32_t delta = now_mm - trigger->last_mm;
  trigger->last_mm = now_mm;
  trigger->acc_mm += (delta < 0) ? -delta : delta;

  if (trigger->acc_mm < distance_mm) {
    return false;
  }
  trigger->acc_mm -= distance_mm;
  return true;
}

/**
 * @brief  Decide y emite el log de control según el perfil.
 *
 * Normal: por distancia, en el mismo punto que la odometría.
 * Debug: cada ciclo de control mientras haya comando de movimiento.
 *
 * @return true
 * @return false
 */
static bool log_control(void) {
  if (is_log_debug_enabled()) {
    if (!log_is_moving()) {
      return false;
    }
    log_control_debug_sample();
    return true;
  }

  if (!log_trigger_update(&control_log_trigger, get_encoder_avg_millimeters(), get_log_distance_mm())) {
    return false;
  }
  log_control_sample();
  return true;
}

/**
 * @brief Decide y emite el log de odometría por distancia.
 *
 * @return true
 * @return false
 */
static bool log_odometry(void) {
  bool debug = is_log_debug_enabled();

  if (!log_trigger_update(&odometry_log_trigger, get_encoder_avg_millimeters(), get_log_distance_mm())) {
    return false;
  }

  if (debug) {
    log_odometry_debug_sample();
  } else {
    log_odometry_sample();
  }
  return true;
}

/**
 * @brief Reinicia ambos triggers al estado inicial de la pista
 *
 */
static void log_reset(void) {
  int32_t distance_mm = get_log_distance_mm();

  log_trigger_reset(&control_log_trigger, distance_mm);
  log_trigger_reset(&odometry_log_trigger, distance_mm);
}

/**
 * @brief Comprueba si el robot está en funcionamiento
 *
 * @return bool
 */

bool is_race_started(void) {
  return race_started;
}

/**
 * @brief Establece el estado actual del robot
 *
 * @param state Estado actual del robot
 */

void set_race_started(bool state) {
  race_started = state;

  reset_control_all();
  if (!state) {
    menu_reset();
    race_finish_ms = get_clock_ticks();
  } else {
    set_log_start();
  }
}

uint32_t get_race_finish_ms(void) {
  return race_finish_ms;
}

void set_control_debug(bool state) {
  control_debug = state;
}

void set_control_paused(bool paused) {
  control_paused = paused;
}

int8_t check_start_run(void) {
  return -1;
}

void set_angular_correction_enabled(bool enabled) {
  angular_correction_enabled = enabled;
}

void set_line_sensors_correction_enabled(bool enabled) {
  line_sensors_correction_enabled = enabled;
}

void reset_control_errors(void) {
  line_sensors_error = 0;
  sum_line_sensors_error = 0;
  last_line_sensors_error = 0;

  linear_error = 0;
  last_linear_error = 0;
  sum_linear_error = 0;

  angular_error = 0;
  last_angular_error = 0;
  sum_angular_error = 0;
}

void reset_control_speed(void) {
  target_linear_speed = 0;
  ideal_linear_speed = 0;

  ideal_angular_speed = 0.0;

  voltage_left = 0;
  voltage_right = 0;

  pwm_left = 0;
  pwm_right = 0;
}

void reset_control_all(void) {
  reset_control_errors();
  reset_control_speed();
  reset_motors_saturated();
  reset_encoders();
  odometry_reset();
  log_reset();
}

void set_target_linear_speed(int32_t linear_speed) {
  target_linear_speed = linear_speed;
}

void force_target_linear_speed(int32_t linear_speed) {
  target_linear_speed = linear_speed;
  ideal_linear_speed = linear_speed;
}

int32_t get_ideal_linear_speed(void) {
  return ideal_linear_speed;
}

void set_ideal_angular_speed(float angular_speed) {
  ideal_angular_speed = angular_speed;
}

float get_ideal_angular_speed(void) {
  return ideal_angular_speed;
}

void set_target_fan_speed(int32_t fan_speed, int32_t ms) {
  target_fan_speed = fan_speed; // percentage_to_fan_pwm(fan_speed);
  fan_speed_accel = (fan_speed - ideal_fan_speed) * CONTROL_FREQUENCY_HZ / ms;
}

int32_t get_ideal_fan_speed(void) {
  return (int32_t)ideal_fan_speed;
}

/**
 * @brief Función de control general del robot
 * · Gestiona velocidades, aceleraciones, correcciones, ...
 *
 */
void control_loop(void) {
  if (is_debug_enabled() || control_paused) {
    return;
  }
  if ((is_motor_pwm_saturated() || is_motor_angle_saturated()) && is_race_started()) {
    set_motors_speed(0, 0);
    set_fan_speed(0);
    if (get_clock_ticks() - get_motors_saturated_ms() < 3000) {
      blink_RGB_color(is_motor_pwm_saturated() ? 255 : 0, 0, is_motor_angle_saturated() ? 255 : 0, 50);
    } else {
      set_RGB_color(0, 0, 0);
      set_race_started(false);
    }
    return;
  }
  if (!is_race_started()) {
    if (race_finish_ms > 0 && get_clock_ticks() - race_finish_ms <= 3000) {
      set_motors_brake();
    } else {
      set_motors_speed(0, 0);
    }
    set_fan_speed(0);
    return;
  }

  update_ideal_linear_speed();
  update_fan_speed();
  set_fan_speed(ideal_fan_speed);

  linear_voltage = 0;
  angular_voltage = 0;

  last_linear_error = linear_error;
  linear_error = ideal_linear_speed - get_measured_linear_speed();
  sum_linear_error += linear_error;

  if (angular_correction_enabled) {
    last_angular_error = angular_error;
    angular_error = ideal_angular_speed - get_measured_angular_speed();
    sum_angular_error += angular_error;
  }

  if (line_sensors_correction_enabled) {
    last_line_sensors_error = line_sensors_error;
    line_sensors_error = get_line_sensor_position();
    sum_line_sensors_error += line_sensors_error;
  }

  linear_voltage =
      get_kinematics().kpi[KPI_LINEAR].kp * linear_error +
      get_kinematics().kpi[KPI_LINEAR].ki * sum_linear_error +
      get_kinematics().kpi[KPI_LINEAR].kd * (linear_error - last_linear_error);

  angular_voltage =
      get_kinematics().kpi[KPI_ANGULAR].kp * angular_error +
      get_kinematics().kpi[KPI_ANGULAR].ki * sum_angular_error +
      get_kinematics().kpi[KPI_ANGULAR].kd * (angular_error - last_angular_error) +

      get_kinematics().kpi[KPI_LINE_SENSORS].kp * line_sensors_error +
      get_kinematics().kpi[KPI_LINE_SENSORS].ki * sum_line_sensors_error +
      get_kinematics().kpi[KPI_LINE_SENSORS].kd * (line_sensors_error - last_line_sensors_error);

  voltage_left = linear_voltage + angular_voltage;
  voltage_right = linear_voltage - angular_voltage;
  pwm_left = voltage_to_motor_pwm(voltage_left);
  pwm_right = voltage_to_motor_pwm(voltage_right);
  set_motors_pwm(pwm_left, pwm_right);

  log_control();
  log_odometry();
}

void emergency_stop(void) {
  set_race_started(false);
  emergency_stop_ms = get_clock_ticks();
}