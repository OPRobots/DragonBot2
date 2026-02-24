#include <config.h>

void handle_robot_version(void) {
  enum ROBOT_VERSION robot_version = DRAGONBOT2_UNKNOWN;
  if (UID_WORD0 == 0x00310016 && UID_WORD1 == 0x3038470E && UID_WORD2 == 0x39323137) {
    robot_version = DRAGONBOT2_B;
  } else if (UID_WORD0 == 0x0039001C && UID_WORD1 == 0x3235510F && UID_WORD2 == 0x32313338) {
    robot_version = DRAGONBOT2_C;
  } else if (UID_WORD0 != 0 && UID_WORD1 != 0 && UID_WORD2 != 0) {
    robot_version = DRAGONBOT2_A;
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