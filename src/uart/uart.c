#include "uart.h"

/* Initialize UART on pins */
void init_uart(void){
    /* Enable GPIOA RCC */
    RCC_AHB1ENR |= 0x1;
    /* Enable USART2 RCC */
    RCC_APB1ENR |= (0x1 << 17);

    /* Configure GPIOA PA2 and PA3 */
    GPIOA_MODER &= ~((3 << (2 * 2)) | (3 << (2 * 3)));
    GPIOA_MODER |= ((2 << (2 * 2)) | (2 << (2 * 3)));

    /* Set both pins to AF7 */
    GPIOA_AFRL &= ~((15 << (4 * 2)) | (15 << (4 * 3)));
    GPIOA_AFRL |= ( (7 << (4 * 2)) | (7 << (4 * 3)));


    /*
    USART2 Important Bits:
    Bit 13 = USART Enable
    Bit 3 = Transmitter Enable
    */

    USART2_CR1 = 0;
    USART2_CR1 = (1 << 13) | (1 << 3);
    
    /* fCK / baud rate */
    USART2_BRR = 139;
}

/* Transmit character */
void uart_transmit(char c){
    /* Everytime the transmit register is empty and ready ... */
    while(!(USART2_SR & (1 << 7)));
    /* Load it with a character */
    USART2_DR = (uint8_t)c;
}

/* Print a whole message */
void uart_print(char* msg){
    /* Loop as long as it doesn't reach null terminator of the message */
    while(*msg){
        /* Transmit and then move onto the next character's address */
        uart_transmit(*msg);
        msg++;
    }
}