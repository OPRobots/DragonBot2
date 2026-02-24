#ifndef MOVE_H
#define MOVE_H

#include "control.h"
#include "menu.h"
#include "menu_run.h"

struct linear_accel_params {
  int16_t break_accel;
  int16_t accel_hard;
  int16_t speed_hard;
  int16_t accel_soft;
};

struct kinematics {
  int16_t linear_speed;
  struct linear_accel_params linear_accel;
  int16_t fan_speed;
};

enum speed_strategy;
void configure_kinematics(enum speed_strategy speed);
struct kinematics get_kinematics(void);

#endif