#include "i2c.h"
#include "systick/systick.h"

/*
Driver for Adafruit 24LC32 EEPROM
*/

/*
#########
Init Code
#########
*/

void gpiob_init(void){
/* Initialize GPIOB for SCL and SDA */

    /* Enable GPIOB RCC */
    RCC_AHB1ENR |= (0x1 << 1);

    /* 
    Set up PB6 and PB7 
    - Alternate Functions for both
    - Both open-drain
    - Both pull-up
    */

    /* Setting pins to alternate function */
    GPIOB_MODER &= ~((0x3 << (6 * 2))|(0x3 << (7 * 2)));
    GPIOB_MODER |= ((0x2 << (6 * 2)) | (0x2 << (7 * 2)));

    /* Setting to open-drain */
    GPIOB_OTYPER &= ~((0x1 << 6) | (0x1 << 7));
    GPIOB_OTYPER |= ((0x1 << 6) | (0x1 << 7));

    /* Setting to pull-up */
    GPIOB_PUPDR &= ~((0x3 << (6 * 2))|(0x3 << (7 * 2)));
    GPIOB_PUPDR |= ((0x1 << (6 * 2)) | (0x1 << (7 * 2)) );

    /* I2C is alternate function 4 on PB6 and PB7 */
    GPIOB_AFRL &= ~((0xF << (6 * 4)) | (0xF << (7 * 4)));
    GPIOB_AFRL |= ((0x4 << (6 * 4)) | (0x4 << (7 * 4)));
} 

void eeprom_i2c_init(void){
    /* I2C peripheral enable */

    /* Set up GPIOB - PB6 (SCL) and PB7 (SDA) */
    gpiob_init();

    /* Enable RCC for I2C */
    RCC_APB1ENR |= (0x1 << 21);

    /* Reset I2C1 peripheral */
    I2C1_CR1 |= (1 << 15);  // Set SWRST bit
    I2C1_CR1 &= ~(1 << 15); // Clear SWRST bit

    /* 
    Control Register 2:
    - Standard Mode
    - Frequency of APB1 (42 MHz) given to I2C
    */
    I2C1_CR2 = 0;
    I2C1_CR2 = 42;

    /* 
    CCR - Clock Control Register 
    - Fapb1 = 42 MHz
    - Standard Mode - 100kHz
    - CCR = Fapb1 / (I2C clock * 2)
    */
    I2C1_CCR = 0;
    I2C1_CCR = 210;

    /* 
    TRISE - 1000ns max for standard mode
    TRISE = (trise / (1/tapb1)) + 1
    therefore, TRISE = 43
    */
    I2C1_TRISE = 43;

    /* 
    Enable ACK (for reads) and enable the I2C peripheral
    */
    I2C1_CR1 = 0;
    I2C1_CR1 |= ((0x1 << 10) | (0x1 << 0));

    systick_sleep(10);
}

/*
###################
EEPROM Comm Helpers
###################
*/

void i2c_start(void){
/* Start condition */
/* Set START bit to 1 */
    I2C1_CR1 |= (1 << 8);
    
    /* Wait for bit start flag */
    while(!(I2C1_SR1 & 1)){};
}

void i2c_stop(void){
/* Stop condition */
    /* Set STOP bit to 1*/
    I2C1_CR1 |= (1 << 9);
}

void eeprom_read_control_byte(void){
/* Send the read control byte to EEPROM */
    /* Set the control byte for read */
    I2C1_DR = 0b10100001;
    /* Wait for address to be sent flag */
    while(!(I2C1_SR1 & (1 << 1))){};

    /* Read SR1 and then SR2 to clear ADDR */
    volatile uint16_t temp = I2C1_SR1;
    temp = I2C1_SR2;
    /* Compiler ignore */
    (void) temp;
    systick_sleep(1);
}

void eeprom_write_control_byte(void){
/* Send the write control byte to EEPROM */
    /* Send the write control byte to EEPROM */
    /* Set the control byte for write */
    I2C1_DR = 0b10100000;
    /* Wait for address to be sent flag */
    while(!(I2C1_SR1 & (1 << 1))){};

    /* Read SR1 and then SR2 to clear ADDR */
    volatile uint16_t temp = I2C1_SR1;
    temp = I2C1_SR2;
    /* Compiler ignore */
    (void) temp;
    systick_sleep(1);
}

void eeprom_high_byte(uint16_t addr){
/* Convert full address into only high byte for send */
    /* Wait for TxE (data register is empty) */
    while(!(I2C1_SR1 & (1 << 7))){};

    /* Extract the 4 high bytes of the addr and load DR */
    uint8_t hb = (addr >> 8) & 0xF; 

    I2C1_DR = hb;
    systick_sleep(1);
};

void eeprom_low_byte(uint16_t addr){
/* Convert full address into only low byte for send*/
    /* Wait for TxE (data register is empty) */
    while(!(I2C1_SR1 & (1 << 7))){};

    /* Extract the 8 low bytes of the addr and load DR */
    uint8_t lb = addr & 0xFF;
    I2C1_DR = lb;
    systick_sleep(1);
};

void eeprom_ack_poll(void){
/* Acknowledge polling for post-write */
    i2c_start();

    while(1){
        eeprom_write_control_byte();
            
            MASTER_CLEAR_AF();
        }
        else{
            break;
        }
    }
};

/* 
#################################
Application Programming Interface
#################################
*/

uint16_t eeprom_read_byte(uint16_t addr){
    /* Read a byte of data from specified address */
    i2c_start();
    /* Reading starts with write control byte to target address */
    eeprom_write_control_byte();
    eeprom_high_byte(addr);
    eeprom_low_byte(addr);

    /* Repeated start, then send read control byte */
    i2c_start();
    eeprom_read_control_byte();

    WAIT_FOR_RXNE();
    uint8_t r = I2C1_DR;
    i2c_stop();
    return r;
};

void eeprom_write_byte(uint16_t addr, uint8_t data){
    /* Write a single byte of data to specified address */
    i2c_start();
    eeprom_write_control_byte();
    eeprom_high_byte(addr);
    eeprom_low_byte(addr);
    
    WAIT_FOR_TXE();
    I2C1_DR = data;

    WAIT_FOR_BTF();

    i2c_stop();
};

void eeprom_write_page(uint16_t addr, uint8_t *arr, size_t arrlen){
    /* Write up to a page (8 bytes) of content */
    
    /* Return this function if the input is larger than 8 pages */
    if(arrlen > 8){
        return;
    }

    i2c_start();
    /* Write 0b10100000 */
    eeprom_write_control_byte();
    //systick_sleep(1);
    eeprom_high_byte(addr);
    //systick_sleep(1);
    eeprom_low_byte(addr);

    for(size_t i = 0; i < arrlen; i++){
        WAIT_FOR_TXE();
        I2C1_DR = arr[i];
        WAIT_FOR_BTF();
    }
    i2c_stop();
}

void eeprom_read_page(uint16_t addr, uint8_t *readarray, size_t readlen){
    /* Read from the EEPROM by providing an array and length of reading */

    i2c_start();
    eeprom_write_control_byte();
    eeprom_high_byte(addr);
    eeprom_low_byte(addr);

    i2c_start();
    eeprom_read_control_byte();

    /* Own code for handling readlen of 1 */
    if(readlen == 1){
        WAIT_FOR_RXNE();
        readarray[0] = I2C1_DR;
        MASTER_SEND_NACK();
        i2c_stop();
        return;
    }

    /* Set ACK for each byte read */
    MASTER_SEND_ACK();

    for(size_t i = 0; i < readlen; i++){
        WAIT_FOR_RXNE();
        readarray[i] = I2C1_DR;
            if(i == readlen-1){
                MASTER_SEND_NACK();
                break;
            }
    }
}

/*
##############
TEST FUNCTIONS
##############
*/

void test_eeprom_write_byte(uint16_t a, uint16_t d){
/* Test Function for EEPROM Writing */

/* Write to address 100 for 169 */
    uint16_t addr = a;
    uint8_t data = d;
    
    eeprom_i2c_init();
    eeprom_write_byte(addr, data);
}

void test_eeprom_write_page(uint16_t addr){
/* Test function for EEPROM writing page */
    uint8_t write_arr[8] = {21,21,21,21,21,21,21,21};
    
    eeprom_i2c_init(); 
    eeprom_write_page(addr, &write_arr, 8);
}   

uint16_t test_eeprom_read_byte(uint16_t a){
    eeprom_i2c_init();

    uint16_t r = 0;
    r = eeprom_read_byte(a);

    return r;
}
