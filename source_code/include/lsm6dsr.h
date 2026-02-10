#ifndef LSM6DSR_H
#define LSM6DSR_H

#include <stdint.h>

#include <lsm6dsr_reg.h>

#include "constants.h"
#include "delay.h"
#include "eeprom.h"
#include "math.h"
#include "motors.h"
#include "setup.h"

#define GYRO_DATA_LENGTH 1

void lsm6dsr_init(void);
uint8_t lsm6dsr_who_am_i(void);

void lsm6dsr_gyro_z_calibration(void);
void lsm6dsr_load_eeprom(void);

void lsm6dsr_update(void);
float lsm6dsr_get_gyro_z_raw(void);
float lsm6dsr_get_gyro_z_radps(void);
float lsm6dsr_get_gyro_z_dps(void);
float lsm6dsr_get_gyro_z_degrees(void);
void lsm6dsr_set_gyro_z_degrees(float deg);

#endif // LSM6DSR_H