#include <leds.h>

static uint32_t rainbow_last_ms = 0;
static uint32_t rainbow_rgb[3] = {LEDS_MAX_PWM, 0, 0};
static int16_t rainbow_color_desc = 0;
static int16_t rainbow_color_asc = 1;

static uint32_t rgb_while_ms = 0;

static uint32_t warning_last_ms = 0;

static uint32_t rgb_last_blink_ms = 0;
static bool rgb_blink_state = false;

static uint32_t wave_last_ms = 0;
static int8_t wave_current_step = 1;
static uint8_t wave_current_index = 0;

static uint32_t blink_last_ms = 0;

static uint32_t battery_warning_last_ms = 0;

void set_status_led(bool state) {
  if (state) {
    gpio_set(GPIOB, GPIO14);
  } else {
    gpio_clear(GPIOB, GPIO14);
  }
}

void toggle_status_led(void) {
  gpio_toggle(GPIOB, GPIO14);
}

void warning_status_led(uint32_t ms) {
  if (get_clock_ticks() > warning_last_ms + ms) {
    toggle_status_led();
    warning_last_ms = get_clock_ticks();
  }
}

bool is_status_led_on(void) {
  return gpio_get(GPIOB, GPIO14);
}

void set_RGB_color(uint32_t r, uint32_t g, uint32_t b) {
  timer_set_oc_value(TIM1, TIM_OC3, r);
  timer_set_oc_value(TIM1, TIM_OC4, b);
  timer_set_oc_value(TIM1, TIM_OC2, g);
  rgb_while_ms = 0;
}

void set_RGB_color_while(uint32_t r, uint32_t g, uint32_t b, uint32_t ms) {
  set_RGB_color(r, g, b);
  rgb_while_ms = get_clock_ticks() + ms;
}

void blink_RGB_color(uint32_t r, uint32_t g, uint32_t b, uint32_t ms) {
  if (get_clock_ticks() > rgb_last_blink_ms + ms) {
    rgb_blink_state = !rgb_blink_state;
    if (rgb_blink_state) {
      set_RGB_color(0, 0, 0);
    } else {
      set_RGB_color(r, g, b);
    }
    rgb_last_blink_ms = get_clock_ticks();
  }
}

void check_leds_while(void) {
  if (rgb_while_ms > 0 && get_clock_ticks() > rgb_while_ms) {
    set_RGB_color(0, 0, 0);
    rgb_while_ms = 0;
  }
}

void set_RGB_rainbow(void) {
  if (get_clock_ticks() > rainbow_last_ms + 10) {
    rainbow_last_ms = get_clock_ticks();
    rainbow_rgb[rainbow_color_desc] -= 20;
    rainbow_rgb[rainbow_color_asc] += 20;
    set_RGB_color(rainbow_rgb[0], rainbow_rgb[1], rainbow_rgb[2]);
    if (rainbow_rgb[rainbow_color_desc] <= 0 || rainbow_rgb[rainbow_color_asc] >= LEDS_MAX_PWM) {
      rainbow_rgb[rainbow_color_desc] = 0;
      rainbow_rgb[rainbow_color_asc] = LEDS_MAX_PWM;
      set_RGB_color(rainbow_rgb[0], rainbow_rgb[1], rainbow_rgb[2]);
      rainbow_color_desc++;
      if (rainbow_color_desc > 2) {
        rainbow_color_desc = 0;
      }
      rainbow_color_asc = rainbow_color_desc == 2 ? 0 : rainbow_color_desc + 1;
    }
  }
}

void set_leds_wave(int ms) {
  if (get_clock_ticks() > wave_last_ms + ms) {
    gpio_clear(GPIOC, GPIO4 | GPIO5);
    gpio_clear(GPIOB, GPIO0 | GPIO1 | GPIO2);
    gpio_clear(GPIOC, GPIO15 | GPIO14 | GPIO13);
    gpio_clear(GPIOB, GPIO9 | GPIO8);

    switch (wave_current_index) {
      case 0:
        gpio_set(GPIOC, GPIO4);
        gpio_set(GPIOC, GPIO5);
        break;
      case 1:
        gpio_set(GPIOB, GPIO0);
        gpio_set(GPIOB, GPIO1);
        break;
      case 2:
        gpio_set(GPIOB, GPIO2);
        gpio_set(GPIOC, GPIO15);
        break;
      case 3:
        gpio_set(GPIOC, GPIO14);
        gpio_set(GPIOC, GPIO13);
        break;
      case 4:
        gpio_set(GPIOB, GPIO9);
        gpio_set(GPIOB, GPIO8);
        break;
    }

    if (wave_current_index >= 4) {
      wave_current_step = -1;
    } else if (wave_current_index <= 0) {
      wave_current_step = 1;
    }

    wave_current_index += wave_current_step;
    wave_last_ms = get_clock_ticks();
  }
}

void set_leds_blink(int ms) {
  if (get_clock_ticks() > blink_last_ms + ms) {
    gpio_toggle(GPIOC, GPIO4 | GPIO5);
    gpio_toggle(GPIOB, GPIO0 | GPIO1 | GPIO2);
    gpio_toggle(GPIOC, GPIO15 | GPIO14 | GPIO13);
    gpio_toggle(GPIOB, GPIO9 | GPIO8);
    blink_last_ms = get_clock_ticks();
  }
}

void set_leds_battery_level(float battery_level) {
  if (battery_level <= 10) {
    gpio_clear(GPIOB, GPIO0 | GPIO1 | GPIO9 | GPIO8);
    gpio_clear(GPIOC, GPIO4 | GPIO5 | GPIO14 | GPIO13);
    if (get_clock_ticks() > battery_warning_last_ms + 50) {

      gpio_toggle(GPIOB, GPIO2);
      gpio_toggle(GPIOC, GPIO15);

      battery_warning_last_ms = get_clock_ticks();
    }

  } else if (battery_level <= 26) {
    gpio_clear(GPIOB, GPIO0 | GPIO1 | GPIO9 | GPIO8);
    gpio_clear(GPIOC, GPIO4 | GPIO5 | GPIO14 | GPIO13);

    gpio_set(GPIOB, GPIO2);
    gpio_set(GPIOC, GPIO15);

  } else if (battery_level <= 42) {
    gpio_clear(GPIOB, GPIO0 | GPIO9 | GPIO8);
    gpio_clear(GPIOC, GPIO4 | GPIO5 | GPIO13);

    gpio_set(GPIOB, GPIO1 | GPIO2);
    gpio_set(GPIOC, GPIO15 | GPIO14);

  } else if (battery_level <= 58) {
    gpio_clear(GPIOB, GPIO9 | GPIO8);
    gpio_clear(GPIOC, GPIO4 | GPIO5);

    gpio_set(GPIOB, GPIO1 | GPIO2 | GPIO0);
    gpio_set(GPIOC, GPIO15 | GPIO14 | GPIO13);

  } else if (battery_level <= 74) {
    gpio_clear(GPIOB, GPIO8);
    gpio_clear(GPIOC, GPIO4);

    gpio_set(GPIOB, GPIO1 | GPIO2 | GPIO0 | GPIO9);
    gpio_set(GPIOC, GPIO15 | GPIO14 | GPIO13 | GPIO5);
  } else if (battery_level <= 90) {
    gpio_set(GPIOB, GPIO1 | GPIO2 | GPIO0 | GPIO9 | GPIO8);
    gpio_set(GPIOC, GPIO15 | GPIO14 | GPIO13 | GPIO5 | GPIO4);
  } else {
    if (get_clock_ticks() > battery_warning_last_ms + 50) {

      gpio_toggle(GPIOB, GPIO1 | GPIO2 | GPIO0 | GPIO9 | GPIO8);
      gpio_toggle(GPIOC, GPIO15 | GPIO14 | GPIO13 | GPIO5 | GPIO4);

      battery_warning_last_ms = get_clock_ticks();
    }
  }
}

void all_leds_clear(void) {
  set_RGB_color(0, 0, 0);
  set_status_led(false);

  gpio_clear(GPIOC, GPIO4 | GPIO5);
  gpio_clear(GPIOB, GPIO0 | GPIO1 | GPIO2);
  gpio_clear(GPIOC, GPIO15 | GPIO14 | GPIO13);
  gpio_clear(GPIOB, GPIO9 | GPIO8);
}

void set_info_led(uint8_t index, bool state) {
  switch (index) {
    case 0:
      if (state) {
        gpio_set(GPIOC, GPIO4);
      } else {
        gpio_clear(GPIOC, GPIO4);
      }
      break;
    case 1:
      if (state) {
        gpio_set(GPIOC, GPIO5);
      } else {
        gpio_clear(GPIOC, GPIO5);
      }
      break;
    case 2:
      if (state) {
        gpio_set(GPIOB, GPIO0);
      } else {
        gpio_clear(GPIOB, GPIO0);
      }
      break;
    case 3:
      if (state) {
        gpio_set(GPIOB, GPIO1);
      } else {
        gpio_clear(GPIOB, GPIO1);
      }
      break;
    case 4:
      if (state) {
        gpio_set(GPIOB, GPIO2);
      } else {
        gpio_clear(GPIOB, GPIO2);
      }
      break;
    case 5:
      if (state) {
        gpio_set(GPIOC, GPIO15);
      } else {
        gpio_clear(GPIOC, GPIO15);
      }
      break;
    case 6:
      if (state) {
        gpio_set(GPIOC, GPIO14);
      } else {
        gpio_clear(GPIOC, GPIO14);
      }
      break;
    case 7:
      if (state) {
        gpio_set(GPIOC, GPIO13);
      } else {
        gpio_clear(GPIOC, GPIO13);
      }
      break;
    case 8:
      if (state) {
        gpio_set(GPIOB, GPIO9);
      } else {
        gpio_clear(GPIOB, GPIO9);
      }
      break;
    case 9:
      if (state) {
        gpio_set(GPIOB, GPIO8);
      } else {
        gpio_clear(GPIOB, GPIO8);
      }
      break;
  }
}

void set_info_leds(void) {
  gpio_set(GPIOC, GPIO4 | GPIO5);
  gpio_set(GPIOB, GPIO0 | GPIO1 | GPIO2);
  gpio_set(GPIOC, GPIO15 | GPIO14 | GPIO13);
  gpio_set(GPIOB, GPIO9 | GPIO8);
}

void show_robot_version(uint16_t version) {
  clear_info_leds();
  switch (version) {
    case DRAGONBOT2_A:
      printf("DragonBot2 A inited...\n");
      set_info_led(INFO_LED_A, true);
      delay(1000);
      break;
    case DRAGONBOT2_B:
      printf("DragonBot2 B inited...\n");
      set_info_led(INFO_LED_B, true);
      delay(1000);
      break;
    case DRAGONBOT2_C:
      printf("DragonBot2 C inited...\n");
      set_info_led(INFO_LED_C, true);
      delay(1000);
      break;
    default:
      printf("Robot version: Unknown\n");
      uint32_t ms = get_clock_ticks();
      while (get_clock_ticks() - ms < 1000) {
        set_leds_wave(100);
        warning_status_led(100);
      }
      break;
  }
  all_leds_clear();
  delay(250);
}

void clear_info_leds(void) {
  gpio_clear(GPIOC, GPIO4 | GPIO5);
  gpio_clear(GPIOB, GPIO0 | GPIO1 | GPIO2);
  gpio_clear(GPIOC, GPIO15 | GPIO14 | GPIO13);
  gpio_clear(GPIOB, GPIO9 | GPIO8);
}