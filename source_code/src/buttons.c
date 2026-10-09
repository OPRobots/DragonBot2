#include "buttons.h"

static uint32_t btn_menu_select_ms = 0;
static uint32_t btn_menu_mode_ms = 0;

static bool btn_menu_up = false;
static bool btn_menu_down = false;

static bool has_started_race = false;
static bool has_stopped_after_race = true;

void check_buttons(void) {
  if ((bool)gpio_get(GPIOA, GPIO6)) {
    if (btn_menu_select_ms == 0) {
      btn_menu_select_ms = get_clock_ticks();
    }
  } else {
    btn_menu_select_ms = 0;
  }

  if ((bool)gpio_get(GPIOA, GPIO7)) {
    if (btn_menu_mode_ms == 0) {
      btn_menu_mode_ms = get_clock_ticks();
    }
  } else {
    btn_menu_mode_ms = 0;
  }

  if (is_race_started()) {
    has_stopped_after_race = false;
  } else if (has_started_race && get_encoder_avg_speed() == 0) {
    has_stopped_after_race = true;
    has_started_race = false;
    reset_encoders();
  }

  if (!is_race_started() && !menu_run_can_start() && !is_debug_enabled() && !is_calibration_enabled() && has_stopped_after_race) {
    if (get_encoder_left_millimeters() >= 10) {
      btn_menu_down = false;
      btn_menu_up = true;
    } else if (get_encoder_left_millimeters() <= -10) {
      btn_menu_up = false;
      btn_menu_down = true;
    } else {
      btn_menu_down = false;
      btn_menu_up = false;
    }
  } else {
    btn_menu_down = false;
    btn_menu_up = false;
    if (is_race_started() && !has_started_race) {
      has_started_race = true;
    }
  }
}

bool get_menu_select_btn(void) {
  return btn_menu_select_ms > 0 && get_clock_ticks() - btn_menu_select_ms > 50;
}

bool get_menu_up_btn(void) {
  if (btn_menu_up) {
    reset_encoders();
    btn_menu_up = false;
    return true;
  }
  return false;
}

bool get_menu_down_btn(void) {
  if (btn_menu_down) {
    reset_encoders();
    btn_menu_down = false;
    return true;
  }
  return false;
}

bool get_menu_mode_btn(void) {
  return btn_menu_mode_ms > 0 && get_clock_ticks() - btn_menu_mode_ms > 50;
}