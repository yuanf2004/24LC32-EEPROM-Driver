#ifndef SYSTICK_H
#define SYSTICK_H

#include "stm32f446xx.h"

/* Activate Systick on the Cortex-M4 Processor */
#define SYSTICK_CTRL *((uint32_t *)(0xE000E010))
#define SYSTICK_RLD *((uint32_t *)(0xE000E014))
#define SYSTICK_CUR_VAL *((uint32_t *)(0xE000E018))

extern volatile uint32_t ms_ticks;

void init_systick(void);

void SysTick_Handler(void);

void systick_sleep(uint32_t time);

#endif