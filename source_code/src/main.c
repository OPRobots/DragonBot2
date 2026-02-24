#include <battery.h>

void sys_tick_handler(void) {
  clock_tick();
}

int main(void) {

  setup();
  eeprom_load();
  handle_robot_version();
  show_battery_level();

  while (1) {
  }
}