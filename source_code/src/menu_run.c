#include <menu_run.h>

#define MODE_SPEED 0
#define MODE_RACE 1
#define MODE_ADAPTATIVE_SPEED 2
#define MODE_BIGGER_FILTER 3
#define MODE_DIGITAL_SENSORS 4
#define MODE_RUN_ALGORITHM 5
uint8_t modeRun = MODE_SPEED;

#define MODE_SPEED_VALUES 6
#define MODE_RACE_VALUES 2
#define MODE_ADAPTATIVE_SPEED_VALUES 2
#define MODE_BIGGER_FILTER_VALUES 2
#define MODE_DIGITAL_SENSORS_VALUES 2
#define MODE_RUN_ALGORITHM_VALUES 3

int16_t valueRun[MENU_RUN_NUM_MODES] = {0, 0, 0, 1, 1, 1};

uint32_t lastBlinkMs = 0;
bool blinkState = false;

static void handle_menu_run_values(void) {
  if (get_clock_ticks() - lastBlinkMs >= 125) {
    lastBlinkMs = get_clock_ticks();
    blinkState = !blinkState;
  }
  if (modeRun == MODE_SPEED) {
    set_RGB_color(0, 0, 0);

    if (valueRun[MODE_SPEED] == MODE_SPEED_VALUES - 1) {
      set_info_led(INFO_LED_1, blinkState);
      set_info_led(INFO_LED_2, blinkState);
      set_info_led(INFO_LED_3, blinkState);
      set_info_led(INFO_LED_4, blinkState);
      set_info_led(INFO_LED_5, blinkState);
    } else {
      for (uint8_t i = 0; i < MODE_SPEED_VALUES - 1; i++) {
        if ((valueRun[MODE_SPEED] == i && blinkState)) {
          set_info_led(i, true);
        } else {
          set_info_led(i, false);
        }
      }
    }
  } else {
    if (valueRun[MODE_SPEED] == MODE_SPEED_VALUES - 1) {
      set_info_led(INFO_LED_1, true);
      set_info_led(INFO_LED_2, true);
      set_info_led(INFO_LED_3, true);
      set_info_led(INFO_LED_4, true);
      set_info_led(INFO_LED_5, true);
    } else {
      for (uint8_t i = 0; i < MODE_SPEED_VALUES - 1; i++) {
        set_info_led(i, i == valueRun[MODE_SPEED]);
      }
    }
  }

  if (modeRun == MODE_RACE) {
    if (valueRun[modeRun] == 1) {
      check_line_color();
      if (abs(get_line_sensor_position()) < 50) {
        set_RGB_color(50, 0, 50);
      } else if (get_line_sensor_position() >= 50) {
        set_RGB_color(100, 0, 0);
      } else if (get_line_sensor_position() <= -50) {
        set_RGB_color(0, 0, 100);
      }

    } else {
      if (blinkState) {
        set_RGB_color(50, 0, 0);
      } else {
        set_RGB_color(0, 0, 0);
      }
    }
  }

  if (modeRun == MODE_ADAPTATIVE_SPEED) {
    if (valueRun[modeRun] == 1) {
      set_RGB_color(0, 50, 0);
    } else {
      set_RGB_color(0, 0, 0);
    }
    set_info_led(INFO_LED_A, blinkState);
  } else {
    set_info_led(INFO_LED_A, valueRun[MODE_ADAPTATIVE_SPEED] == 1);
  }

  if (modeRun == MODE_BIGGER_FILTER) {
    if (valueRun[modeRun] == 1) {
      set_RGB_color(0, 50, 0);
    } else {
      set_RGB_color(0, 0, 0);
    }
    set_info_led(INFO_LED_B, blinkState);
  } else {
    set_info_led(INFO_LED_B, valueRun[MODE_BIGGER_FILTER] == 1);
  }

  if (modeRun == MODE_DIGITAL_SENSORS) {
    if (valueRun[modeRun] == 1) {
      set_RGB_color(0, 50, 0);
    } else {
      set_RGB_color(0, 0, 0);
    }
    set_info_led(INFO_LED_D, blinkState);
  } else {
    set_info_led(INFO_LED_D, valueRun[MODE_DIGITAL_SENSORS] == 1);
  }

  if (modeRun == MODE_RUN_ALGORITHM) {
    switch (valueRun[modeRun]) {
      case RUN_LINEFOLLOWER:
        set_RGB_color(0, 0, 50);
        break;
      case RUN_ROBOTRACER:
        set_RGB_color(50, 0, 50);
        break;
      case RUN_DRAGRACE:
        set_RGB_color(50, 50, 50);
        break;
      default:
        set_RGB_color(0, 0, 0);
        break;
    }
    set_info_led(INFO_LED_E, blinkState);
  } else {
    set_info_led(INFO_LED_E, valueRun[MODE_RUN_ALGORITHM] == RUN_ROBOTRACER);
  }
}

static void handle_menu_run_btn(void) {
  if (get_menu_up_btn()) {
    menu_run_up();
  }
  if (get_menu_down_btn() || (modeRun == MODE_RACE && get_menu_select_btn())) {
    reset_encoders();
    menu_run_down();
  }
}

bool menu_run_handler(void) {
  set_status_led(false);

  if (get_menu_mode_btn()) {
    uint32_t ms = get_clock_ticks();
    while (get_menu_mode_btn()) {
      if (get_clock_ticks() - ms >= 200) {
        warning_status_led(50);
      }
    }
    if (get_clock_ticks() - ms >= 200) {
      return true;
    } else if (modeRun != MODE_RACE || !valueRun[MODE_RACE]) {
      menu_run_mode_change();
    } else {
      set_status_led(false);
    }
  }

  handle_menu_run_btn();
  handle_menu_run_values();
  return false;
}

void menu_run_reset(void) {
  modeRun = MODE_SPEED;
  valueRun[MODE_RACE] = 0;
}

void menu_run_load_values(void) {
  int16_t *data = eeprom_get_data();
  for (uint16_t i = DATA_INDEX_MENU; i < (DATA_INDEX_MENU + MENU_RUN_NUM_MODES); i++) {
    valueRun[i - DATA_INDEX_MENU] = data[i];
  }
  valueRun[MODE_RACE] = 0;
}

void menu_run_mode_change() {
  modeRun = (modeRun + 1) % MENU_RUN_NUM_MODES;
}

void menu_run_up() {
  uint8_t mode_values = 0;
  switch (modeRun) {
    case MODE_SPEED:
      mode_values = MODE_SPEED_VALUES;
      break;
    case MODE_RACE:
      mode_values = MODE_RACE_VALUES;
      break;
    case MODE_ADAPTATIVE_SPEED:
      mode_values = MODE_ADAPTATIVE_SPEED_VALUES;
      break;
    case MODE_BIGGER_FILTER:
      mode_values = MODE_BIGGER_FILTER_VALUES;
      break;
    case MODE_DIGITAL_SENSORS:
      mode_values = MODE_DIGITAL_SENSORS_VALUES;
      break;
    case MODE_RUN_ALGORITHM:
      mode_values = MODE_RUN_ALGORITHM_VALUES;
      break;
  }
  valueRun[modeRun] = (valueRun[modeRun] + 1) % mode_values;
  if (modeRun == MODE_RACE && valueRun[modeRun] == 1) {
    set_RGB_color(50, 0, 0);
    eeprom_set_data(DATA_INDEX_MENU, valueRun, MENU_RUN_NUM_MODES);
    eeprom_save();
  }
}

void menu_run_down() {
  if (valueRun[modeRun] > 0) {
    valueRun[modeRun]--;
  }
}

bool menu_run_can_start(void) {
  return modeRun == MODE_RACE && valueRun[MODE_RACE] > 0;
}

int16_t *get_menu_run_values(void) {
  return valueRun;
}

enum speed_strategy menu_run_get_speed(void) {
  return valueRun[MODE_SPEED];
}

enum adaptative_speed menu_run_get_adaptative_speed(void) {
  return valueRun[MODE_ADAPTATIVE_SPEED];
}

enum filter_mode menu_run_get_filter_mode(void) {
  return valueRun[MODE_BIGGER_FILTER];
}

enum sensors_mode menu_run_get_sensors_mode(void) {
  return valueRun[MODE_DIGITAL_SENSORS];
}

enum run_algorithm menu_run_get_run_algorithm(void) {
  return valueRun[MODE_RUN_ALGORITHM];
}