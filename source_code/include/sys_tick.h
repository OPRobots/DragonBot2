#ifndef SYS_TICK_H
#define SYS_TICK_H

#include <stdint.h>

/** Frecuencia del SysTick: 8 kHz => 8 slots de 125 us = 1 kHz por tarea. */
#define SYSTICK_FREQUENCY_HZ 8000

/**
 * @brief Identificadores de los slots del scheduler del SysTick.
 *
 * El SysTick corre a 8 kHz; el contador avanza por estos valores de forma
 * cíclica, de modo que cada tarea se ejecuta a 1 kHz en su propio slot.
 */
typedef enum {
  SYS_TICK_SLOT_CLOCK_BUTTONS = 0,   /* clock_tick() + check_buttons() */
  SYS_TICK_SLOT_ENCODERS_LINE = 1,
  SYS_TICK_SLOT_MPU_BMP = 2,
  SYS_TICK_SLOT_CONTROL = 3,
  SYS_TICK_SLOT_BATTERY = 4,
  SYS_TICK_SLOT_FLUSH_A = 5, /* volcado SRAM */
  SYS_TICK_SLOT_FLUSH_B = 6, /* volcado SRAM */
  SYS_TICK_SLOT_FLUSH_C = 7, /* volcado SRAM */
  SYS_TICK_SLOT_COUNT = 8,
} sys_tick_slot_t;

/** @brief Configura el SysTick y su prioridad NVIC. */
void sys_tick_init(void);

/** @brief Slot que se está ejecutando (o se ejecutará) en el SysTick. */
uint8_t sys_tick_get_slot(void);

#endif
