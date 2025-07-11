#include "systick.h"

volatile uint32_t ms_ticks;

/* Initialize SysTick */
void init_systick(){
    __enable_irq();

    /* Match the 16 MHz clock cycle but minus 1 since it starts at 0 */
    SYSTICK_RLD = 15999; 
    SYSTICK_CUR_VAL = 0;
    /* Clear bits [2:0] */
    SYSTICK_CTRL &= ~(0x7);
    /* 
    CLKSOURCE = 1 = Processor Clock
    TICKINT = 1 = SysTick Exception Request on 0 for interrupt
    ENABLE = 1 
    */
    SYSTICK_CTRL |= 0x7;
}

void SysTick_Handler(void){
    ms_ticks++;
}

/* Sleep system in milliseconds */
void systick_sleep(uint32_t time){
    /* Stuck in while loop by comparing function start time and desired time */
    uint32_t ms_curr = ms_ticks;
    while(ms_ticks - ms_curr < time){
    }
}