#include "sensors.h"

// ADC de derecha a izquierda
static uint8_t adc1_channels[ADC1_CHANNEL_COUNT] = {ADC_CHANNEL11, ADC_CHANNEL10, ADC_CHANNEL12};
static volatile uint16_t adc1_raw[ADC1_CHANNEL_COUNT];

static uint8_t adc2_channels[ADC2_CHANNEL_COUNT] = {ADC_CHANNEL0, ADC_CHANNEL1, ADC_CHANNEL2, ADC_CHANNEL3, ADC_CHANNEL4, ADC_CHANNEL5, ADC_CHANNEL6, ADC_CHANNEL7, ADC_CHANNEL13};
static volatile uint16_t adc2_raw[ADC2_CHANNEL_COUNT];

static volatile uint16_t line_sensors_raw[LINE_SENSOR_COUNT];
static volatile uint16_t mux_index = 0;

static int16_t line_sensors_max[LINE_SENSOR_COUNT];
static int16_t line_sensors_min[LINE_SENSOR_COUNT];
static int16_t line_sensors_thr[LINE_SENSOR_COUNT];

static volatile int32_t line_position = 0;
static uint32_t line_last_position = 0;

static volatile uint16_t mark_sensors_raw[MARK_SENSOR_COUNT];

static int16_t mark_sensors_max[MARK_SENSOR_COUNT];
static int16_t mark_sensors_min[MARK_SENSOR_COUNT];
static int16_t mark_sensors_thr[MARK_SENSOR_COUNT];

uint8_t *get_adc1_channels(void) {
  return adc1_channels;
}

uint8_t get_adc1_channel_count(void) {
  return ADC1_CHANNEL_COUNT;
}

volatile uint16_t *get_adc1_raw(void) {
  return adc1_raw;
}

uint8_t *get_adc2_channels(void) {
  return adc2_channels;
}

uint8_t get_adc2_channel_count(void) {
  return ADC2_CHANNEL_COUNT;
}

volatile uint16_t *get_adc2_raw(void) {
  return adc2_raw;
}

uint8_t get_line_sensor_count(void) {
  return LINE_SENSOR_COUNT;
}

uint16_t get_line_sensor_raw(uint8_t index) {
  if (index < LINE_SENSOR_COUNT) {
    return line_sensors_raw[index];
  } else {
    return 0;
  }
}

uint16_t get_line_sensor_calibrated(uint8_t index) {
  if (index < LINE_SENSOR_COUNT) {
    uint16_t sensor_value = get_line_sensor_raw(index);

    switch (menu_run_get_sensors_mode()) {
      case SENSORS_ANALOG:
        if (sensor_value < (line_sensors_min[index] + (line_sensors_max[index] - line_sensors_min[index]) * SENSORS_ANALOG_MIN_PERCENT)) {
          sensor_value = LINE_SENSOR_MIN_VALUE;
        } else if (sensor_value > (line_sensors_min[index] + (line_sensors_max[index] - line_sensors_min[index]) * SENSORS_ANALOG_MAX_PERCENT)) {
          sensor_value = LINE_SENSOR_MAX_VALUE;
        }
        break;
      case SENSORS_DIGITAL:
      default:
        if (sensor_value >= line_sensors_thr[index]) {
          sensor_value = LINE_SENSOR_MAX_VALUE;
        } else {
          sensor_value = LINE_SENSOR_MIN_VALUE;
        }
        break;
    }

#ifdef CONFIG_LINE_BLACK
    return sensor_value;
#elif CONFIG_LINE_WHITE
    return LINE_SENSOR_MAX_VALUE - sensor_value;
#else
    return sensor_value;
#endif

  } else {
    return 0;
  }
}

int32_t get_line_sensor_position(void) {
  return line_position;
}

uint8_t get_mark_sensor_count(void) {
  return MARK_SENSOR_COUNT;
}

uint16_t get_mark_sensor_raw(uint8_t index) {
  if (index < MARK_SENSOR_COUNT) {
    return mark_sensors_raw[index];
  } else {
    return 0;
  }
}

uint16_t get_mark_sensor_calibrated(uint8_t index) {
  if (index < MARK_SENSOR_COUNT) {
    uint16_t sensor_value = get_mark_sensor_raw(index);

    switch (menu_run_get_sensors_mode()) {
      case SENSORS_ANALOG:
        if (sensor_value < (mark_sensors_min[index] + (mark_sensors_max[index] - mark_sensors_min[index]) * SENSORS_ANALOG_MIN_PERCENT)) {
          sensor_value = LINE_SENSOR_MIN_VALUE;
        } else if (sensor_value > (mark_sensors_min[index] + (mark_sensors_max[index] - mark_sensors_min[index]) * SENSORS_ANALOG_MAX_PERCENT)) {
          sensor_value = LINE_SENSOR_MAX_VALUE;
        }
        break;
      case SENSORS_DIGITAL:
      default:
        if (sensor_value >= mark_sensors_thr[index]) {
          sensor_value = LINE_SENSOR_MAX_VALUE;
        } else {
          sensor_value = LINE_SENSOR_MIN_VALUE;
        }
        break;
    }

#ifdef CONFIG_LINE_BLACK
    return sensor_value;
#elif CONFIG_LINE_WHITE
    return LINE_SENSOR_MAX_VALUE - sensor_value;
#else
    return sensor_value;
#endif

  } else {
    return 0;
  }
}

void sensors_calibration(void) {
  bool use_eeprom_calibration = true;
  while (!is_right_wheel_click_up()) {
    if (is_left_wheel_click_down()) {
      use_eeprom_calibration = !use_eeprom_calibration;
      set_status_led(use_eeprom_calibration);
    }
    set_status_led(use_eeprom_calibration);
  }

  if (use_eeprom_calibration) {
    set_status_led(true);
    bool line_sensors_min_checked[get_line_sensor_count()];
    bool line_sensors_max_checked[get_line_sensor_count()];
    for (uint8_t sensor = 0; sensor < get_line_sensor_count(); sensor++) {
      line_sensors_min_checked[sensor] = false;
      line_sensors_max_checked[sensor] = false;
    }

    bool mark_sensors_min_checked[get_mark_sensor_count()];
    bool mark_sensors_max_checked[get_mark_sensor_count()];
    for (uint8_t sensor = 0; sensor < get_mark_sensor_count(); sensor++) {
      mark_sensors_min_checked[sensor] = false;
      mark_sensors_max_checked[sensor] = false;
    }

    uint32_t sensors_checked_last_ms = 0;
    uint8_t line_sensors_checked_count = 0;
    uint8_t mark_sensors_checked_count = 0;

    uint32_t ms_inicio = get_clock_ticks();
    while (ms_inicio + LINE_CALIBRATION_MS >= get_clock_ticks()) {
      if (get_clock_ticks() - sensors_checked_last_ms < 50) {
        continue;
      }
      for (uint8_t sensor = 0; sensor < get_line_sensor_count(); sensor++) {
        if (abs(get_line_sensor_raw(sensor) - line_sensors_min[sensor]) < 250) {
          line_sensors_min_checked[sensor] = true;
        }
        if (abs(get_line_sensor_raw(sensor) - line_sensors_max[sensor]) < 250 && line_sensors_min_checked[sensor]) {
          line_sensors_max_checked[sensor] = true;
        }
      }
      line_sensors_checked_count = 0;
      for (uint8_t sensor = 0; sensor < get_line_sensor_count(); sensor++) {
        if (line_sensors_min_checked[sensor] && line_sensors_max_checked[sensor]) {
          line_sensors_checked_count++;
        }
      }

      for (uint8_t sensor = 0; sensor < get_mark_sensor_count(); sensor++) {
        if (abs(get_mark_sensor_raw(sensor) - mark_sensors_min[sensor]) < 250) {
          mark_sensors_min_checked[sensor] = true;
        }
        if (abs(get_mark_sensor_raw(sensor) - mark_sensors_max[sensor]) < 250 && mark_sensors_min_checked[sensor]) {
          mark_sensors_max_checked[sensor] = true;
        }
      }

      if (line_sensors_checked_count == 0) {
        set_RGB_color(125, 0, 0);
      } else if (line_sensors_checked_count < get_line_sensor_count()) {
        set_RGB_color(125, 125, 0);
      } else {
        set_RGB_color(0, 125, 0);
      }

      mark_sensors_checked_count = 0;
      for (uint8_t sensor = 0; sensor < get_mark_sensor_count(); sensor++) {
        bool sensor_ok = mark_sensors_min_checked[sensor] && mark_sensors_max_checked[sensor];
        if (sensor_ok) {
          mark_sensors_checked_count++;
        }
        switch (sensor) {
          case 0:
            set_info_led(INFO_LED_2, sensor_ok);
            break;
          case 1:
            set_info_led(INFO_LED_1, sensor_ok);
            break;
          case 2:
            set_info_led(INFO_LED_D, sensor_ok);
            break;
          case 3:
            set_info_led(INFO_LED_E, sensor_ok);
            break;
        }
      }

      reset_wheel_clicks();
      if (line_sensors_checked_count == get_line_sensor_count() && mark_sensors_checked_count == get_mark_sensor_count()) {
        break;
      }
      sensors_checked_last_ms = get_clock_ticks();
    }

    if (line_sensors_checked_count == get_line_sensor_count() && mark_sensors_checked_count == get_mark_sensor_count()) {
      set_RGB_color(0, 125, 0);
      set_info_led(INFO_LED_2, true);
      set_info_led(INFO_LED_1, true);
      set_info_led(INFO_LED_D, true);
      set_info_led(INFO_LED_E, true);
      delay(500);
      set_RGB_color(0, 0, 0);
      clear_info_leds();
    } else {
      while (!is_right_wheel_click_up()) {
      }
    }

  } else {
    set_status_led(false);
    delay(1000);

    for (uint8_t sensor = 0; sensor < LINE_SENSOR_COUNT; sensor++) {
      line_sensors_max[sensor] = LINE_SENSOR_MIN_VALUE;
      line_sensors_min[sensor] = LINE_SENSOR_MAX_VALUE;
      line_sensors_thr[sensor] = LINE_SENSOR_MIN_VALUE;
    }
    for (uint8_t sensor = 0; sensor < MARK_SENSOR_COUNT; sensor++) {
      mark_sensors_max[sensor] = LINE_SENSOR_MIN_VALUE;
      mark_sensors_min[sensor] = LINE_SENSOR_MAX_VALUE;
      mark_sensors_thr[sensor] = LINE_SENSOR_MIN_VALUE;
    }

    uint32_t ms_inicio = get_clock_ticks();
    while (ms_inicio + LINE_CALIBRATION_MS >= get_clock_ticks()) {
      for (int sensor = 0; sensor < get_line_sensor_count(); sensor++) {
        if (line_sensors_raw[sensor] < line_sensors_min[sensor]) {
          line_sensors_min[sensor] = line_sensors_raw[sensor];
        }
        if (line_sensors_raw[sensor] > line_sensors_max[sensor]) {
          line_sensors_max[sensor] = line_sensors_raw[sensor];
        }
      }
      for (int sensor = 0; sensor < get_mark_sensor_count(); sensor++) {
        if (mark_sensors_raw[sensor] < mark_sensors_min[sensor]) {
          mark_sensors_min[sensor] = mark_sensors_raw[sensor];
        }
        if (mark_sensors_raw[sensor] > mark_sensors_max[sensor]) {
          mark_sensors_max[sensor] = mark_sensors_raw[sensor];
        }
      }

      set_RGB_rainbow();
    }

    bool line_calibration_ok = true;
    for (int sensor = 0; sensor < get_line_sensor_count(); sensor++) {
      if (abs(line_sensors_max[sensor] - line_sensors_min[sensor]) < 1000) {
        line_calibration_ok = false;
      }
      line_sensors_thr[sensor] = line_sensors_min[sensor] + ((line_sensors_max[sensor] - line_sensors_min[sensor]) * 2 / 3);
    }
    bool mark_calibration_ok = true;
    for (int sensor = 0; sensor < get_mark_sensor_count(); sensor++) {
      bool sensor_ok = true;
      if (abs(mark_sensors_max[sensor] - mark_sensors_min[sensor]) < 1000) {
        sensor_ok = false;
        mark_calibration_ok = false;
      }
      mark_sensors_thr[sensor] = mark_sensors_min[sensor] + ((mark_sensors_max[sensor] - mark_sensors_min[sensor]) * 2 / 3);

      switch (sensor) {
        case 0:
          set_info_led(INFO_LED_2, sensor_ok);
          break;
        case 1:
          set_info_led(INFO_LED_1, sensor_ok);
          break;
        case 2:
          set_info_led(INFO_LED_D, sensor_ok);
          break;
        case 3:
          set_info_led(INFO_LED_E, sensor_ok);
          break;
      }
    }

    eeprom_set_data(DATA_INDEX_LINE_SENSORS_MAX, line_sensors_max, LINE_SENSORS_DATA_LENGTH);
    eeprom_set_data(DATA_INDEX_LINE_SENSORS_MIM, line_sensors_min, LINE_SENSORS_DATA_LENGTH);
    eeprom_set_data(DATA_INDEX_LINE_SENSORS_THR, line_sensors_thr, LINE_SENSORS_DATA_LENGTH);

    eeprom_set_data(DATA_INDEX_MARK_SENSORS_MAX, mark_sensors_max, MARK_SENSORS_DATA_LENGTH);
    eeprom_set_data(DATA_INDEX_MARK_SENSORS_MIM, mark_sensors_min, MARK_SENSORS_DATA_LENGTH);
    eeprom_set_data(DATA_INDEX_MARK_SENSORS_THR, mark_sensors_thr, MARK_SENSORS_DATA_LENGTH);

    reset_wheel_clicks();
    if (line_calibration_ok && mark_calibration_ok) {
      set_RGB_color(0, 100, 0);
      delay(500);
      set_RGB_color(0, 0, 0);
      clear_info_leds();
    } else {
      if (!line_calibration_ok) {
        set_RGB_color(100, 0, 0);
      } else {
        set_RGB_color(100, 100, 0);
      }
      while (!is_right_wheel_click_up()) {
        // waiting
      }
    }
    reset_wheel_clicks();
  }
  set_RGB_color(0, 0, 0);
  sensors_print_calibration();
  delay(250);
}

void sensors_load_eeprom(void) {
  int16_t *eeprom_data = eeprom_get_data();
  for (uint16_t i = DATA_INDEX_LINE_SENSORS_MAX; i < DATA_INDEX_LINE_SENSORS_MAX + LINE_SENSOR_COUNT; i++) {
    line_sensors_max[i - DATA_INDEX_LINE_SENSORS_MAX] = eeprom_data[i];
  }
  for (uint16_t i = DATA_INDEX_LINE_SENSORS_MIM; i < DATA_INDEX_LINE_SENSORS_MIM + LINE_SENSOR_COUNT; i++) {
    line_sensors_min[i - DATA_INDEX_LINE_SENSORS_MIM] = eeprom_data[i];
  }
  for (uint16_t i = DATA_INDEX_LINE_SENSORS_THR; i < DATA_INDEX_LINE_SENSORS_THR + LINE_SENSOR_COUNT; i++) {
    line_sensors_thr[i - DATA_INDEX_LINE_SENSORS_THR] = eeprom_data[i];
  }
  for (uint16_t i = DATA_INDEX_MARK_SENSORS_MAX; i < DATA_INDEX_MARK_SENSORS_MAX + MARK_SENSOR_COUNT; i++) {
    mark_sensors_max[i - DATA_INDEX_MARK_SENSORS_MAX] = eeprom_data[i];
  }
  for (uint16_t i = DATA_INDEX_MARK_SENSORS_MIM; i < DATA_INDEX_MARK_SENSORS_MIM + MARK_SENSOR_COUNT; i++) {
    mark_sensors_min[i - DATA_INDEX_MARK_SENSORS_MIM] = eeprom_data[i];
  }
  for (uint16_t i = DATA_INDEX_MARK_SENSORS_THR; i < DATA_INDEX_MARK_SENSORS_THR + MARK_SENSOR_COUNT; i++) {
    mark_sensors_thr[i - DATA_INDEX_MARK_SENSORS_THR] = eeprom_data[i];
  }
  sensors_print_calibration();
}

void sensors_print_calibration(void) {
  for (int sensor = 0; sensor < get_line_sensor_count(); sensor++) {
    printf("Sensor %2d: %4d <> %4d <> %4d\n", sensor + 1, line_sensors_min[sensor], line_sensors_thr[sensor], line_sensors_max[sensor]);
  }
  printf("___________________________________________\n");
  for (int sensor = 0; sensor < get_mark_sensor_count(); sensor++) {
    printf("Mark %2d: %4d <> %4d <> %4d\n", sensor + 1, mark_sensors_min[sensor], mark_sensors_thr[sensor], mark_sensors_max[sensor]);
  }
}

void line_sensors_update_mux_readings(void) {
  // ! Caution: Parece que al activar el ADC2 (lectura de batería) se desordenan los canales del ADC1 wtf?
  // ! Al ir actualizando el código, a veces se cambian los canales por la puta cara wtffff??
  // mux_index = 7;
  line_sensors_raw[(MUX_CHANNEL_COUNT) + mux_index] = adc1_raw[0];
  line_sensors_raw[mux_index] = adc1_raw[1];
  line_sensors_raw[2 * (MUX_CHANNEL_COUNT) + mux_index] = adc1_raw[2];

  mux_index = (mux_index + 1) % MUX_CHANNEL_COUNT;
  int c = GPIO13;
  int b = GPIO14;
  int a = GPIO15;
  switch (mux_index) {
    case 0: // 000
      gpio_clear(GPIOC, a | b | c);
      break;
    case 1: // 001
      gpio_set(GPIOC, a);
      gpio_clear(GPIOC, c | b);
      break;
    case 2: // 010
      gpio_set(GPIOC, b);
      gpio_clear(GPIOC, c | a);
      break;
    case 3: // 011
      gpio_set(GPIOC, b | a);
      gpio_clear(GPIOC, c);
      break;
    case 4: // 100
      gpio_set(GPIOC, c);
      gpio_clear(GPIOC, b | a);
      break;
    case 5: // 101
      gpio_set(GPIOC, c | a);
      gpio_clear(GPIOC, b);
      break;
    case 6: // 110
      gpio_set(GPIOC, c | b);
      gpio_clear(GPIOC, a);
      break;
    case 7: // 111
      gpio_set(GPIOC, a | b | c);
      break;
  }
  delay_us(3);
}

void line_sensors_update_position(void) {
  int8_t sensor_first_line_index = -1;
  int8_t sensor_last_line_index = -1;
  int8_t sensor_reference_index = round(map(line_position, -1000, 1000, -1, get_line_sensor_count()));
  if (sensor_reference_index <= 0) {
    sensor_first_line_index = 0;
    sensor_last_line_index = 3;
  } else if (sensor_reference_index >= get_line_sensor_count() - 1) {
    sensor_first_line_index = get_line_sensor_count() - 4;
    sensor_last_line_index = get_line_sensor_count() - 1;
  } else {
    sensor_first_line_index = sensor_reference_index - 2;
    sensor_last_line_index = sensor_reference_index + 1;
    if (sensor_first_line_index < 0) {
      sensor_last_line_index += abs(sensor_first_line_index);
      sensor_first_line_index = 0;
    }
    if (sensor_last_line_index >= get_line_sensor_count()) {
      sensor_first_line_index -= abs(sensor_last_line_index - get_line_sensor_count());
      sensor_last_line_index = get_line_sensor_count() - 1;
    }
  }
  // printf("%d - %d\t|\t", sensor_first_line_index, sensor_last_line_index);

  uint32_t sum_wheigted_sensor_values = 0;
  uint32_t sum_sensor_values = 0;
  uint8_t sensors_line_count = 0;
  uint8_t sensors_unfiltered_line_count = 0;
  for (uint8_t sensor = 0; sensor < get_line_sensor_count(); sensor++) {
    uint16_t sensor_value = get_line_sensor_calibrated(sensor);

    if (sensor_value >= line_sensors_thr[sensor]) {
      sensors_unfiltered_line_count++;
    }

    if (menu_run_get_filter_mode() == FILTER_BIGGER) {
      if (sensor < sensor_first_line_index || sensor > sensor_last_line_index) {
        sensor_value = LINE_SENSOR_MIN_VALUE;
      }
    }

    if (sensor_value >= line_sensors_thr[sensor]) {
      sensors_line_count++;
    }
    sum_wheigted_sensor_values += (sensor + 1) * sensor_value * 1000;
    sum_sensor_values += sensor_value;
  }

  if (sensors_line_count > 0 && sensors_unfiltered_line_count < get_line_sensor_count() / 2) {
    line_last_position = get_clock_ticks();
  } else if (is_race_started()) {
    if (get_clock_ticks() > (line_last_position + get_offtrack_time())) {
      emergency_stop();
    }
    if (abs(line_position) < 800) {
      return;
    }
  }

  int32_t max_position = ((1000 * (get_line_sensor_count() + 1)) / 2);

  int32_t position;
  if (sensors_line_count > 0) {
    position = (sum_wheigted_sensor_values / sum_sensor_values) - max_position;
  } else {
    position = (line_position >= 0) ? max_position : -max_position;
  }
  line_position = map(position, -max_position, max_position, -1000, 1000);
}

void line_sensors_reset_position(void) {
  line_position = 0;
  line_last_position = 0;
}

void mark_sensors_update_readings(uint8_t index, uint16_t value) {
  mark_sensors_raw[index] = value;
}

bool is_mark_sensor_left(void) {
  return mark_sensors_raw[0] >= mark_sensors_thr[0] || mark_sensors_raw[1] >= mark_sensors_thr[1];
}
bool is_mark_sensor_right(void) {
  return mark_sensors_raw[2] >= mark_sensors_thr[2] || mark_sensors_raw[3] >= mark_sensors_thr[3];
}