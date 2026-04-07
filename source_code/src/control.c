#include "control.h"

static volatile bool race_started = false;
static volatile uint32_t race_finish_ms = 0;

static volatile uint32_t emergency_stop_ms = 0;

static volatile bool control_debug = false;

static volatile int32_t target_linear_speed = 0;
static volatile int32_t ideal_linear_speed = 0;
static volatile float ideal_angular_speed = 0.0;

static volatile int32_t target_fan_speed = 0;
static volatile float ideal_fan_speed = 0;
static volatile float fan_speed_accel = 0;

static volatile float linear_error;
static volatile float sum_linear_error;
static volatile float last_linear_error;

static volatile float angular_error;
static volatile float sum_angular_error;
static volatile float last_angular_error;

static volatile bool sensors_correction_enabled = false;

static volatile float sensors_error;
static volatile float last_sensors_error;
static volatile float sum_sensors_error;

static volatile float voltage_left;
static volatile float voltage_right;
static volatile int32_t pwm_left;
static volatile int32_t pwm_right;

/**
 * @brief Convierte un valor de voltaje dado a su correspondiente PWM
 *
 * @param voltage
 * @return int32_t PWM a aplicar al motor
 */
static int32_t voltage_to_motor_pwm(float voltage) {
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
    ideal_linear_speed += accel / CONTROL_FREQUENCY_HZ;
    if (ideal_linear_speed > target_linear_speed) {
      ideal_linear_speed = target_linear_speed;
    }
  } else if (ideal_linear_speed > target_linear_speed) {
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
  return -lsm6dsr_get_gyro_z_radps();
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
  }
}

void set_control_debug(bool state) {
  control_debug = state;
}

int8_t check_start_run(void) {
  return -1;
}

void set_sensors_correction(bool enabled) {
  sensors_correction_enabled = enabled;
}

void reset_control_errors(void) {
  sensors_error = 0;
  sum_sensors_error = 0;
  last_sensors_error = 0;

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
  reset_encoder_avg();
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

/**
 * @brief Función de control general del robot
 * · Gestiona velocidades, aceleraciones, correcciones, ...
 *
 */
void control_loop(void) {
  // gpio_set(GPIOB, GPIO13);
  // delay_us(100);
  // gpio_clear(GPIOB, GPIO13);
  return;
  if (is_debug_enabled()) {
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
      set_motors_enable(false);
    }
    set_fan_speed(0);
    return;
  } else {
    set_motors_enable(true);
  }

  update_ideal_linear_speed();
  update_fan_speed();
  set_fan_speed(ideal_fan_speed);

  float linear_voltage = 0;
  float angular_voltage = 0;

  last_linear_error = linear_error;
  linear_error = ideal_linear_speed - get_measured_linear_speed();
  sum_linear_error += linear_error;

  last_angular_error = angular_error;
  angular_error = ideal_angular_speed - get_measured_angular_speed();
  sum_angular_error += angular_error;

  sensors_error = 0;
  if (sensors_correction_enabled) {
    sensors_error += get_line_sensor_position();
    sum_sensors_error += sensors_error;
  }

  linear_voltage = KP_LINEAR * linear_error + KI_LINEAR * sum_linear_error + KD_LINEAR * (linear_error - last_linear_error);

  angular_voltage =
      KP_ANGULAR * angular_error + KI_ANGULAR * sum_angular_error + KD_ANGULAR * (angular_error - last_angular_error) +
      KP_LINE_SENSORS * sensors_error + KI_LINE_SENSORS * sum_sensors_error + KD_LINE_SENSORS * (sensors_error - last_sensors_error);

  last_sensors_error = sensors_error;

  voltage_left = linear_voltage + angular_voltage;
  voltage_right = linear_voltage - angular_voltage;
  pwm_left = voltage_to_motor_pwm(voltage_left);
  pwm_right = voltage_to_motor_pwm(voltage_right);
  set_motors_pwm(pwm_left, pwm_right);

  if (ideal_linear_speed != 0 || ideal_angular_speed != 0) {
    static char *labels[] = {
        "target_linear_speed",
        "ideal_linear_speed",
        "measured_linear_speed",
        "measured_left_speed",
        "measured_right_speed",
        "ideal_angular_speed",
        "measured_angular_speed",
        "pwm_left",
        "pwm_right",
        // "encoder_avg_millimeters",
        "sensors_error",
        "battery_voltage"};
    macroarray_store(
        5,
        0b00000110011,
        labels,
        11,
        (int16_t)target_linear_speed,
        (int16_t)ideal_linear_speed,
        (int16_t)(get_measured_linear_speed()),
        (int16_t)(get_encoder_left_speed()),
        (int16_t)(get_encoder_right_speed()),
        (int16_t)(ideal_angular_speed * 100),
        (int16_t)(get_measured_angular_speed() * 100),
        (int16_t)pwm_left,
        (int16_t)pwm_right,
        // (int16_t)get_encoder_avg_millimeters(),
        (int16_t)(sensors_error * 100),
        (int16_t)(get_battery_voltage() * 100));
  }
}

void emergency_stop(void) {
  set_race_started(false);
  emergency_stop_ms = get_clock_ticks();
}