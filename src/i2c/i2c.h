#ifndef I2C_H
#define I2C_H

/* 
############
GPIOB Set Up 
############
*/
#define RCC_AHB1ENR *((uint32_t *)(0x40023800 + 0x30))
#define GPIOB_MODER *((uint32_t *)(0x40020400))
#define GPIOB_OTYPER *((uint32_t *)(0x40020400 + 0x04))
#define GPIOB_PUPDR *((uint32_t *)(0x40020400 + 0x0C))
#define GPIOB_AFRL *((uint32_t *)(0x40020400 + 0x20))

/*
##########
I2C Set Up
##########
*/
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

/* 
#############################
Status Flag Macro Definitions
#############################
*/
/* Wait for receive data register to be empty */
#define WAIT_FOR_RXNE() while(!(I2C1_SR1 & (1 << 6))){};
/* Wait for transmit data register to be empty */
#define WAIT_FOR_TXE() while(!(I2C1_SR1 & (1 << 7))){};
/* Wait for the byte transfer to be finished flag */
#define WAIT_FOR_BTF() while(!(I2C1_SR1 & (1 << 2))){}
/* Send an acknowledgement to slave */
#define MASTER_SEND_ACK() I2C1_CR1 |= (1 << 10);
/* Send a negative acknowledgement to slave */
#define MASTER_SEND_NACK() I2C1_CR1 &= ~(1 << 10);

/* WIP - ACK POLLING */

/* Check if the master received a NACK from the slave */
#define MASTER_RECEIVED_NACK() (I2C1_SR1 & (1 << 10));
/* Clear the acknowledge failure (AF) bit after checking NACK */
#define MASTER_CLEAR_AF() I2C1_SR1 &= ~(1 << 10);

#include <stdint.h>
#include <stddef.h>

void gpiob_init(void);

void eeprom_i2c_init(void);

void i2c_start(void);

void i2c_stop(void);

void eeprom_read_control_byte(void);

void eeprom_write_control_byte(void);

void eeprom_high_byte(uint16_t addr);

void eeprom_low_byte(uint16_t addr);

void eeprom_write_byte(uint16_t addr, uint8_t data);

uint16_t eeprom_read_byte(uint16_t addr);

void test_eeprom_write_byte(uint16_t a, uint16_t d);

uint16_t test_eeprom_read_byte(uint16_t a);

void test_eeprom_write_page(uint16_t addr);

#endif

/* Page 781 I2C */