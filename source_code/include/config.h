#ifndef __CONFIG_H
#define __CONFIG_H

#include <stdint.h>
#include <stdio.h>

#include "leds.h"
#include "sensors.h"

#define STM32_UID_BASE (0x1FFF7A10U)
#define UID_WORD0 MMIO32(STM32_UID_BASE + 0x0)
#define UID_WORD1 MMIO32(STM32_UID_BASE + 0x4)
#define UID_WORD2 MMIO32(STM32_UID_BASE + 0x8)

enum ROBOT_VERSION {
  DRAGONBOT2_UNKNOWN = 0,
  DRAGONBOT2_A = 1,
  DRAGONBOT2_B = 2,
  DRAGONBOT2_C = 3
};

/** Características Físicas */
#define MICROMETERS_PER_TICK 10.494055
#define ROBOT_FRONT_LENGTH 48.121
#define ROBOT_BACK_LENGTH 40.706
#define ROBOT_WIDTH 70.2
#define WHEELS_SEPARATION 62
#define ROBOT_MIDDLE_WIDTH ((ROBOT_WIDTH / 2.0))

/** Movimiento */
#define MAX_MOTOR_SATURATION_COUNT 30
#define MAX_MOTOR_ANGULAR_SATURATION_COUNT 60

#define KP_LINEAR 0.010
#define KI_LINEAR 0.0005
#define KD_LINEAR 0

#define KP_ANGULAR 0.6
#define KI_ANGULAR 0.015
#define KD_ANGULAR 0.0

#define KP_LINE_SENSORS 0.08
#define KI_LINE_SENSORS 0.00 // 0.002
#define KD_LINE_SENSORS 0.12

/** Sensores */
#define SENSOR_LOW_PASS_FILTER_ALPHA 0.1
#define LINE_CALIBRATION_MS 5000
#define LINE_SENSOR_MAX_VALUE 4096
#define LINE_SENSOR_MIN_VALUE 0
#define ANALOG_SENSORS_MIN_PERCENT (20 / 100.0)
#define ANALOG_SENSORS_MAX_PERCENT (80 / 100.0)

/** Divisor de Voltage */
#define VOLT_DIV_FACTOR_3S 5.30
#define VOLT_DIV_FACTOR_2S 3.15
#define BATTERY_2S_HIGH_LIMIT_VOLTAGE 8.4
#define BATTERY_2S_LOW_LIMIT_VOLTAGE 7.4
#define BATTERY_3S_HIGH_LIMIT_VOLTAGE 12.6
#define BATTERY_3S_LOW_LIMIT_VOLTAGE 11.1

/** Modo RUN */
// #define CONFIG_RUN_RACE 1
#define CONFIG_RUN_DEBUG 0

#define OFFTRACK_MS 150
#define START_MS_DELAY_RUN 5000
#define START_MS_DELAY_DEBUG 2000

void handle_robot_version(void);
void set_all_configs(void);
uint16_t get_config_run(void);

uint16_t get_offtrack_time(void);
uint16_t get_start_millis(void);

#endif