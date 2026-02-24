#ifndef __EEPROM_H
#define __EEPROM_H

#include <libopencm3/stm32/flash.h>

#include "delay.h"
#include "leds.h"
#include "menu_run.h"
#include "rc5.h"
#include "sensors.h"

#define EEPROM_SECTOR 11
#define EEPROM_BASE_ADDRESS (0x080E0000)

#define DATA_LENGTH (GYRO_DATA_LENGTH + RC5_DATA_LENGTH + MENU_RUN_NUM_MODES + LINE_SENSORS_DATA_LENGTH * 3 + MARK_SENSORS_DATA_LENGTH * 3)

#define DATA_INDEX_GYRO_Z 0
#define DATA_INDEX_RC5 (DATA_INDEX_GYRO_Z + GYRO_DATA_LENGTH)
#define DATA_INDEX_MENU (DATA_INDEX_RC5 + RC5_DATA_LENGTH)
#define DATA_INDEX_LINE_SENSORS_MAX (DATA_INDEX_MENU + MENU_RUN_NUM_MODES)
#define DATA_INDEX_LINE_SENSORS_MIM (DATA_INDEX_LINE_SENSORS_MAX + LINE_SENSORS_DATA_LENGTH)
#define DATA_INDEX_LINE_SENSORS_THR (DATA_INDEX_LINE_SENSORS_MIM + LINE_SENSORS_DATA_LENGTH)
#define DATA_INDEX_MARK_SENSORS_MAX (DATA_INDEX_LINE_SENSORS_THR + LINE_SENSORS_DATA_LENGTH)
#define DATA_INDEX_MARK_SENSORS_MIM (DATA_INDEX_MARK_SENSORS_MAX + MARK_SENSORS_DATA_LENGTH)
#define DATA_INDEX_MARK_SENSORS_THR (DATA_INDEX_MARK_SENSORS_MIM + MARK_SENSORS_DATA_LENGTH)

void eeprom_save(void);
void eeprom_load(void);
void eeprom_clear(void);
void eeprom_backup(void);
void eeprom_restore(void);
void eeprom_set_data(uint16_t index, int16_t *data, uint16_t length);
int16_t *eeprom_get_data(void);

#endif
