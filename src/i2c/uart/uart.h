#ifndef UART_H
#define UART_H

/* RCC BASE =  0x4002 3800*/

/* Pins PA2 and PA3 */

/* Enable GPIOA RCC */
#define RCC_AHB1ENR *((volatile uint32_t*)(0x40023800 + 0x30))
/* Enable USART2 RCC */
#define RCC_APB1ENR *((volatile uint32_t*)(0x40023800 + 0x40))

/* Config GPIOA Pins */
#define GPIOA_MODER *((volatile uint32_t*)(0x40020000))
/* Set for alternate function based on pin mapping */
#define GPIOA_AFRL *((volatile uint32_t*)(0x40020000 + 0x20))

/* Status register for data transfer information */
#define USART2_SR *((volatile uint32_t *)(0x40004400))
/* Data register for data */
#define USART2_DR *((volatile uint32_t *)(0x40004400 + 0x04))
/* Baud rate config to match PIO serial monitor */
#define USART2_BRR *((volatile uint32_t *)(0x40004400 + 0x08))
/* Config register, enable USART and enable transceiver */
#define USART2_CR1 *((volatile uint32_t *)(0x40004400 + 0x0C))

#include <stdint.h>

void init_uart(void);

void uart_transmit(char c);

void uart_print(char* msg);

#endif 