#ifndef __CONTROL_H
#define __CONTROL_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

#include "battery.h"
#include "config.h"
#include "constants.h"
#include "debug.h"
#include "kinematics.h"
#include "macroarray.h"
#include "menu.h"

bool is_race_started(void);
void set_race_started(bool state);
int8_t check_start_run(void);
void set_control_debug(bool state);

void set_sensors_correction(bool enabled);
void reset_control_errors(void);
void reset_control_speed(void);
void reset_control_all(void);

void set_target_linear_speed(int32_t linear_speed);
void force_target_linear_speed(int32_t linear_speed);
int32_t get_ideal_linear_speed(void);
void set_ideal_angular_speed(float angular_speed);
float get_ideal_angular_speed(void);
void set_target_fan_speed(int32_t fan_speed, int32_t ms);

void control_loop(void);

void emergency_stop(void);

#endif