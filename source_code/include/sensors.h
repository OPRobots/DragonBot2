#ifndef __SENSORS_H
#define __SENSORS_H

#include <stdint.h>

#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include "config.h"
#include "delay.h"
#include "eeprom.h"
#include "encoders.h"
#include "leds.h"
#include "menu_run.h"
#include "motors.h"

#define ADC1_CHANNEL_COUNT 3
#define LINE_SENSOR_COUNT 24
#define MUX_CHANNEL_COUNT (LINE_SENSOR_COUNT / ADC1_CHANNEL_COUNT)

#define ADC2_CHANNEL_COUNT 9
#define MARK_SENSOR_COUNT 4

#define LINE_SENSORS_DATA_LENGTH LINE_SENSOR_COUNT
#define MARK_SENSORS_DATA_LENGTH MARK_SENSOR_COUNT

uint8_t *get_adc1_channels(void);
uint8_t get_adc1_channel_count(void);
volatile uint16_t *get_adc1_raw(void);

uint8_t *get_adc2_channels(void);
uint8_t get_adc2_channel_count(void);
volatile uint16_t *get_adc2_raw(void);

uint8_t get_line_sensor_count(void);
uint16_t get_line_sensor_raw(uint8_t index);
uint16_t get_line_sensor_calibrated(uint8_t index);
int32_t get_line_sensor_position(void);

uint8_t get_mark_sensor_count(void);
uint16_t get_mark_sensor_raw(uint8_t index);
uint16_t get_mark_sensor_calibrated(uint8_t index);
bool is_mark_sensor_left(void);
bool is_mark_sensor_right(void);

void sensors_calibration(void);

void sensors_load_eeprom(void);
void sensors_print_calibration(void);

void line_sensors_update_mux_readings(void);
void line_sensors_update_position(void);
void line_sensors_reset_position(void);

void mark_sensors_update_readings(uint8_t index, uint16_t value);

#endif