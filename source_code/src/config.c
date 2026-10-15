#include <config.h>

static float micrometers_per_tick = MICROMETERS_PER_TICK;
static float wheels_separation = WHEELS_SEPARATION;

void handle_robot_version(void) {
  enum ROBOT_VERSION robot_version = DRAGONBOT2_UNKNOWN;
  if (UID_WORD0 == 0x00480049 && UID_WORD1 == 0x304E5005 && UID_WORD2 == 0x2030384B) {
    robot_version = DRAGONBOT2_A;
  } else if (UID_WORD0 != 0 && UID_WORD1 != 0 && UID_WORD2 != 0) {
    robot_version = DRAGONBOT2_B;
  } else if (UID_WORD0 != 0 && UID_WORD1 != 0 && UID_WORD2 != 0) {
    robot_version = DRAGONBOT2_C;
  }

  printf("UID: %08X %08X %08X\n", (unsigned int)UID_WORD0, (unsigned int)UID_WORD1, (unsigned int)UID_WORD2);

  set_battery_volt_div_factor(robot_version);
  show_robot_version(robot_version);
}

uint16_t get_offtrack_time(void) {
#ifdef CONFIG_RUN_RACE
  return (uint16_t)(OFFTRACK_MS * 1.33);
#elif CONFIG_RUN_DEBUG
  return OFFTRACK_MS;
#else
  return OFFTRACK_MS;
#endif
}

uint16_t get_start_millis(void) {
#ifdef CONFIG_RUN_RACE
  return START_MS_DELAY_RUN;
#elif CONFIG_RUN_DEBUG
  return START_MS_DELAY_DEBUG;
#else
  return START_MS_DELAY_DEBUG;
#endif
}

float get_micrometers_per_tick(void) {
  return micrometers_per_tick;
}

float get_wheels_separation(void) {
  return wheels_separation;
}

void config_set_micrometers_per_tick(float value) {
  if (value > 1.0f && value < 1000.0f) {
    micrometers_per_tick = value;
  }
}

void config_set_wheels_separation(float value) {
  if (value > 1.0f && value < 10000.0f) {
    wheels_separation = value;
  }
}

void config_load_eeprom(void) {
  int16_t *data = eeprom_get_data();
  int16_t separation = data[DATA_INDEX_WHEEL_SEPARATION];
  int16_t scale = data[DATA_INDEX_WHEEL_SCALE];
  if (separation > 0) {
    wheels_separation = separation / 100.0f;
  }
  if (scale > 0) {
    micrometers_per_tick = scale / 100.0f;
  }
  printf("Wheel calibration: um/tick=%.3f sep=%.3f mm\n", get_micrometers_per_tick(), get_wheels_separation());
}