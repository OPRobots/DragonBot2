#include <menu_run.h>

#define MODE_SPEED 0
#define MODE_RACE 1
#define MODE_ADAPTATIVE_SPEED 2
#define MODE_BIGGER_FILTER 3
#define MODE_DIGITAL_SENSORS 4
#define MODE_ENCODERS 5
uint8_t modeRun = MODE_SPEED;

#define MODE_SPEED_VALUES 6
#define MODE_RACE_VALUES 2
#define MODE_ADAPTATIVE_SPEED_VALUES 2
#define MODE_BIGGER_FILTER_VALUES 2
#define MODE_DIGITAL_SENSORS_VALUES 2
#define MODE_ENCODERS_VALUES 2

int16_t valueRun[MENU_RUN_NUM_MODES] = {0, 0, 0, 1, 1, 1};

uint32_t lastBlinkMs = 0;
bool blinkState = false;

#ifndef MMSIM_ENABLED
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
      if (abs(get_line_sensor_position()) < 100) {
        set_RGB_color(50, 0, 50);
      } else if (blinkState) {
        set_RGB_color(50, 0, 50);
      } else {
        set_RGB_color(50, 0, 0);
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

  if (modeRun == MODE_ENCODERS) {
    if (valueRun[modeRun] == 1) {
      set_RGB_color(0, 50, 0);
    } else {
      set_RGB_color(0, 0, 0);
    }
    set_info_led(INFO_LED_E, blinkState);
  } else {
    set_info_led(INFO_LED_E, valueRun[MODE_ENCODERS] == 1);
  }
}

static void handle_menu_run_btn(void) {
  if (is_left_wheel_click_up()) {
    menu_run_up();
  }
  if (is_left_wheel_click_down()) {
    menu_run_down();
  }
}
#endif

bool menu_run_handler(void) {
#ifndef MMSIM_ENABLED
  set_status_led(false);

  if (is_right_wheel_click_up()) {
    if (modeRun != MODE_RACE || !valueRun[MODE_RACE]) {
      menu_run_mode_change();
    } else if (modeRun == MODE_RACE) {
      set_status_led(false);
    } else {
      set_status_led(false);
    }
  }
  if (is_right_wheel_click_down()) {
    return true;
  }

  handle_menu_run_btn();
  handle_menu_run_values();
#endif
  return false;
}

void menu_run_reset(void) {
  modeRun = MODE_SPEED;
  valueRun[MODE_RACE] = 0;
}

void menu_run_load_values(void) {
#ifndef MMSIM_ENABLED
  int16_t *data = eeprom_get_data();
  for (uint16_t i = DATA_INDEX_MENU; i < (DATA_INDEX_MENU + MENU_RUN_NUM_MODES); i++) {
    valueRun[i - DATA_INDEX_MENU] = data[i];
  }
  valueRun[MODE_RACE] = 0;
#endif
}

void menu_run_mode_change() {
  modeRun = (modeRun + 1) % MENU_RUN_NUM_MODES;
}

void menu_run_up() {
#ifndef MMSIM_ENABLED
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
    case MODE_ENCODERS:
      mode_values = MODE_ENCODERS_VALUES;
      break;
  }
  valueRun[modeRun] = (valueRun[modeRun] + 1) % mode_values;
  if (modeRun == MODE_RACE && valueRun[modeRun] == 1) {
    set_RGB_color(50, 0, 0);
    eeprom_set_data(DATA_INDEX_MENU, valueRun, MENU_RUN_NUM_MODES);
    eeprom_save();
  }
#endif
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

enum control_strategy menu_run_get_control_strategy(void) {
  return valueRun[MODE_ENCODERS];
}