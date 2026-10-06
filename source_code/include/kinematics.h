#ifndef MOVE_H
#define MOVE_H

#include "control.h"
#include "lsm6dsr.h"
#include "menu.h"
#include "menu_run.h"

enum kpi_controls {
  KPI_LINEAR,
  KPI_ANGULAR,
  KPI_LINE_SENSORS,
};

struct kpi_params {
  double kp;
  double ki;
  double kd;
};
struct mpu_params {
  uint8_t full_scale_dps;
  double low_pass_filter_alpha;
};

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
  struct kpi_params *kpi;
  struct mpu_params mpu;
};

enum speed_strategy;

void configure_kinematics(enum speed_strategy speed);
struct kinematics get_kinematics(void);

#endif