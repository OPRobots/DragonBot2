#include "sys_tick.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include "battery.h"
#include "buttons.h"
#include "constants.h"
#include "control.h"
#include "delay.h"
#include "encoders.h"
#include "lsm6dsr.h"
#include "sram.h"

static volatile uint8_t sys_tick_slot = 0;

void sys_tick_init(void) {
  systick_set_frequency(SYSTICK_FREQUENCY_HZ, SYSCLK_FREQUENCY_HZ);
  systick_counter_enable();
  systick_interrupt_enable();

  nvic_set_priority(NVIC_SYSTICK_IRQ, 16 * 0);
}

uint8_t sys_tick_get_slot(void) {
  return sys_tick_slot;
}

void sys_tick_handler(void) {
  switch (sys_tick_slot) {
    case SYS_TICK_SLOT_CLOCK_BUTTONS:
      clock_tick();
      check_buttons();
      break;
    case SYS_TICK_SLOT_ENCODERS_LINE:
      update_encoder_readings();
      // line_sensors_update_position();
      break;
    case SYS_TICK_SLOT_MPU_BMP:
      if (!sram_log_is_paused()) {
        lsm6dsr_update();
        // bmp_update();
      }
      break;
    case SYS_TICK_SLOT_CONTROL:
      control_loop();
      break;
    case SYS_TICK_SLOT_BATTERY:
      update_battery_voltage();
      break;
    case SYS_TICK_SLOT_FLUSH_A:
    case SYS_TICK_SLOT_FLUSH_B:
    case SYS_TICK_SLOT_FLUSH_C:
      sram_store(SRAM_LOG_FLUSH_BUDGET_CYCLES);
      break;
    default:
      break;
  }

  sys_tick_slot = (uint8_t)((sys_tick_slot + 1u) % SYS_TICK_SLOT_COUNT);
}
