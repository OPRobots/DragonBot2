#ifndef CALIBRATIONS_H
#define CALIBRATIONS_H

#include <math.h>
#include <stdint.h>

#include "control.h"
#include "delay.h"
#include "eeprom.h"
#include "encoders.h"
#include "lsm6dsr.h"
#include "motors.h"
#include "odometry.h"
#include "sensors.h"
#include "sram.h"

#define CALIBRATE_NONE 0
#define CALIBRATE_GYRO_Z 1
#define CALIBRATE_SENSORS 2
#define CALIBRATE_WHEEL_SEPARATION 3
#define CALIBRATE_WHEEL_SCALE 4
#define CALIBRATE_YAW_STEP 5
#define CALIBRATE_STORE_EEPROM 6

void calibrate_from_config(uint8_t type);
bool is_calibration_enabled(void);

#endif