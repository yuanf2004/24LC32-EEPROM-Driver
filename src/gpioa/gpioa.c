#include "gpioa.h"

void gpioa_init(void){
/* Initialize GPIOA */

    /* Init RCC */
    RCC_AHB1ENR &= ~(0x1);
    RCC_AHB1ENR |= 0x1;

    /* 
    Init PA0 
    - Output
    - Pull-down
    - Init reset
    */
    GPIOA_MODER &= ~(0x3);
    GPIOA_MODER |= 0x01;
    GPIOA_PUPDR &= ~(0x3);
    GPIOA_PUPDR |= (0x2);
    GPIOA_BSRR = (0x1);
}

