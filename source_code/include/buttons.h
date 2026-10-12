#ifndef __BUTTONS_H
#define __BUTTONS_H

#include <math.h>
#include <stdio.h>

#include <libopencm3/stm32/gpio.h>

#include "config.h"
#include "control.h"
#include "delay.h"

void check_buttons(void);
void check_encoder_buttons(void);

bool get_menu_select_btn(void);
bool get_menu_up_btn(void);
bool get_menu_down_btn(void);
bool get_menu_mode_btn(void);

#endif