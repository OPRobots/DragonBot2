#include "buttons.h"

static uint32_t btn_menu_select_ms = 0;
static uint32_t btn_menu_mode_ms = 0;

static bool btn_menu_up = false;
static bool btn_menu_down = false;

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
}

void check_encoder_buttons(void) {
  if (!menu_run_can_start() && !is_debug_enabled()) {
    if (get_encoder_left_millimeters() >= 10) {
      btn_menu_down = false;
      btn_menu_up = true;
    } else if (get_encoder_left_millimeters() <= -10) {
      btn_menu_up = false;
      btn_menu_down = true;
    } /*  else {
       btn_menu_down = false;
       btn_menu_up = false;
     } */
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