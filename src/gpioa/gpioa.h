#ifndef GPIOA_H
#define GPIOA_H

#include <stdint.h>

/* GPIOA is set up here for a button */

#define RCC_AHB1ENR *((uint32_t *)(0x40023800 + 0x30))

#define GPIOA_MODER *((uint32_t *)(0x40020000))
#define GPIOA_PUPDR *((uint32_t *)(0x40020000 + 0x0C))
#define GPIOA_BSRR *((uint32_t *)(0x40020000 + 0x18))

void gpioa_init(void);

#endif