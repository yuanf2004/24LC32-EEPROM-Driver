#ifndef I2C_H
#define I2C_H

/* GPIOB Set Up */
#define RCC_AHB1ENR *((uint32_t *)(0x40023800 + 0x30))

#define GPIOB_MODER *((uint32_t *)(0x40020400))
#define GPIOB_OTYPER *((uint32_t *)(0x40020400 + 0x04))
#define GPIOB_PUPDR *((uint32_t *)(0x40020400 + 0x0C))
#define GPIOB_AFRL *((uint32_t *)(0x40020400 + 0x20))

/* I2C Set Up */
#define RCC *((uint32_t *)(0x40023800))
#define RCC_APB1ENR *((uint32_t *)(0x40023800 + 0x40))

/* Control register 1 */
#define I2C1_CR1 *((uint32_t *)(0x40005400))
/* Control register 2 */
#define I2C1_CR2 *((uint32_t *)(0x40005400 + 0x04))
/* I2C data register */
#define I2C1_DR *((uint32_t *)(0x40005400 + 0x10))
/* I2C status register */
#define I2C1_SR1 *((uint32_t *)(0x40005400 + 0x14))
/* I2C status register 2 */
#define I2C1_SR2 *((uint32_t *)(0x40005400 + 0x18))
/* I2C clock control register */
#define I2C1_CCR *((uint32_t *)(0x40005400 + 0x1C))
/* TRISE register - Maximum  */
#define I2C1_TRISE *((uint32_t *)(0x40005400 + 0x20))

#include <stdint.h>

#endif

/* Page 781 I2C */