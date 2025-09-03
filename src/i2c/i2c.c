#include "i2c.h"
#include "systick/systick.h"

/*
Driver for Adafruit 24LC32 EEPROM
Written by: Yuan Feng
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

    init_systick();
    init_uart();

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
    /* Wait for address to be sent to slave (slave ACKs) flag */
    while(!(I2C1_SR1 & (1 << 1))){};

    /* Read SR1 and then SR2 to clear ADDR */
    volatile uint16_t temp = I2C1_SR1;
    temp = I2C1_SR2;
    /* Compiler ignore */
    (void) temp;
    systick_sleep(1);
}

void ack_poll_write_control_byte(void){
/* Call this function when writing control byte for ACK polling */
    I2C1_DR = 0b10100000;
    /* Wait for ACK (ADDR) or NACK (AF)*/
    while(!(I2C1_SR1 & ((1 << 1) | (1 << 10)))){};
}

void eeprom_high_byte(uint16_t addr){
    if(addr > MAX_REGISTER_NUM){
        return;
    }
/* Convert full address into only high byte for send */
    /* Wait for TxE (data register is empty) */
    while(!(I2C1_SR1 & (1 << 7))){};

    /* Extract the 4 high bytes of the addr and load DR */
    uint8_t hb = (addr >> 8) & 0xF; 

    I2C1_DR = hb;
    systick_sleep(1);
};

void eeprom_low_byte(uint16_t addr){
    if(addr > MAX_REGISTER_NUM){
        return;
    }
/* Convert full address into only low byte for send*/
    /* Wait for TxE (data register is empty) */
    while(!(I2C1_SR1 & (1 << 7))){};

    /* Extract the 8 low bytes of the addr and load DR */
    uint8_t lb = addr & 0xFF;
    I2C1_DR = lb;
    systick_sleep(1);
};

/* 
#################################
Application Programming Interface
#################################
*/

/* Unsigned Integers */

uint8_t eeprom_read_byte_uint(uint16_t addr){
/* Read a byte of an unsigned integer */
    /* Error handling for invalid register numbers exceeding max 4096 */
    if(addr > MAX_REGISTER_NUM){
        return 0;
    }

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

void eeprom_read_uint(uint16_t addr, uint8_t *readarray, size_t readlen){
/* Read multiple bytes from the EEPROM */
    if(addr > MAX_REGISTER_NUM){
        return;
    }

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

void eeprom_write_byte_uint(uint16_t addr, uint8_t data){
/* Write a single byte of an unsigned integer to specified address */
    if(addr > MAX_REGISTER_NUM){
        return;
    }

    i2c_start();
    eeprom_write_control_byte();
    eeprom_high_byte(addr);
    eeprom_low_byte(addr);
    
    WAIT_FOR_TXE();
    I2C1_DR = data;

    WAIT_FOR_BTF();

    i2c_stop();
    systick_sleep(50);
};

void eeprom_write_pages_uint(uint16_t addr, uint8_t *arr, size_t arrlen){
/* Write up to 8 pages (64 bytes) of unsigned integers */
    if(addr > MAX_REGISTER_NUM){
        return;
    }
    
    /* Return this function if the input is larger than 8 pages (64 bytes) */
    if((arrlen < 0) | (arrlen > 64)){
        return;
    }
    i2c_start();
    /* Write 0b10100000 */
    eeprom_write_control_byte();
    eeprom_high_byte(addr);
    eeprom_low_byte(addr);

    for(size_t i = 0; i < arrlen; i++){
        WAIT_FOR_TXE();
        I2C1_DR = arr[i];
    }
    WAIT_FOR_BTF();
    i2c_stop();
    systick_sleep(5 * (int) arrlen);
}

/* Signed Integers */

int8_t eeprom_read_byte_int(uint16_t addr){
/* Read a single signed integer byte */
    if(addr > MAX_REGISTER_NUM){
        return -1;
    }
    i2c_start();
    /* Reading starts with write control byte to target address */
    eeprom_write_control_byte();
    eeprom_high_byte(addr);
    eeprom_low_byte(addr);

    /* Repeated start, then send read control byte */
    i2c_start();
    eeprom_read_control_byte();

    WAIT_FOR_RXNE();
    int8_t r = I2C1_DR;
    i2c_stop();
    return r;
}

void eeprom_read_int(uint16_t addr, int8_t *readarray, size_t readlen){
/* Read signed integers */
    if(addr > MAX_REGISTER_NUM){
        return;
    }
    i2c_start();
    eeprom_write_control_byte();
    eeprom_high_byte(addr);
    eeprom_low_byte(addr);

    i2c_start();
    eeprom_read_control_byte();

    /* Own code for handling readlen of 1 */
    if(readlen == 1){
        WAIT_FOR_RXNE();
        readarray[0] = (int8_t) I2C1_DR;
        MASTER_SEND_NACK();
        i2c_stop();
        return;
    }

    /* Set ACK for each byte read */
    MASTER_SEND_ACK();

    for(size_t i = 0; i < readlen; i++){
        WAIT_FOR_RXNE();
        readarray[i] = (int8_t) I2C1_DR;
        if(i == readlen-1){
            MASTER_SEND_NACK();
            break;
        }
    }
}

void eeprom_write_byte_int(uint16_t addr, int8_t data){
/* Write a single byte of a signed integer */
    if(addr > MAX_REGISTER_NUM){
        return;
    }
    i2c_start();
    eeprom_write_control_byte();
    eeprom_high_byte(addr);
    eeprom_low_byte(addr);
    
    WAIT_FOR_TXE();
    I2C1_DR = data;

    WAIT_FOR_BTF();

    i2c_stop();
    systick_sleep(50);
}

void eeprom_write_pages_int(uint16_t addr, int8_t *arr, size_t arrlen){
/* Write up to 8 pages (64 bytes) of signed integers */
    if(addr > MAX_REGISTER_NUM){
        return;
    }

    /* Return this function if the input is larger than 8 pages */
    if((arrlen < 0) | (arrlen > 64)){
        return;
    }
    i2c_start();
    /* Write 0b10100000 */
    eeprom_write_control_byte();
    eeprom_high_byte(addr);
    eeprom_low_byte(addr);

    for(size_t i = 0; i < arrlen; i++){
        WAIT_FOR_TXE();
        I2C1_DR = arr[i];
    }
    WAIT_FOR_BTF();
    i2c_stop();
    systick_sleep(5 * (int) arrlen);
};

/* Characters */

char eeprom_read_byte_char(uint16_t addr){
/* Read a single character */
    if(addr > MAX_REGISTER_NUM){
        return '\0';
    }
    return (char)eeprom_read_byte_uint(addr);
}

void eeprom_read_string(uint16_t addr, char *readstr, size_t readlen){
/* Read a string (up to 64 chars) */
    if(addr > MAX_REGISTER_NUM){
        return;
    }
    i2c_start();
    eeprom_write_control_byte();
    eeprom_high_byte(addr);
    eeprom_low_byte(addr);

    i2c_start();
    eeprom_read_control_byte();

    if(readlen == 1){
        WAIT_FOR_RXNE();
        readstr[0] = I2C1_DR;
        MASTER_SEND_NACK();
        i2c_stop();
        /* Null terminator for string */
        readstr[1] = '\0';
        return;
    }

    MASTER_SEND_ACK();

    for(size_t i = 0; i < readlen; i++){
        WAIT_FOR_RXNE();
        readstr[i] = (char) I2C1_DR;
        if(i == readlen - 1){
            MASTER_SEND_NACK();
            break;
        }
    }
    i2c_stop();
    readstr[readlen] = '\0'; 
}

void eeprom_write_byte_char(uint16_t addr, char data){
/* Write one character (ASCII) */
    if(addr > MAX_REGISTER_NUM){
        return;
    }
    i2c_start();
    eeprom_write_control_byte();
    eeprom_high_byte(addr);
    eeprom_low_byte(addr);

    WAIT_FOR_TXE();
    I2C1_DR = (uint8_t)data;

    WAIT_FOR_BTF();

    i2c_stop();
    systick_sleep(50);
}

void eeprom_write_string(uint16_t addr, char* str, size_t strlen){
/* Write up to 8 pages (64 chars) of string */
    if(addr > MAX_REGISTER_NUM){
        return;
    }
    if((strlen < 0) | (strlen > 64)){
        return;
    }
    i2c_start();

    eeprom_write_control_byte();
    eeprom_high_byte(addr);
    eeprom_low_byte(addr);

    for(size_t i = 0; i < strlen; i++){
        WAIT_FOR_TXE();
        I2C1_DR = (uint8_t) str[i];
    }

    WAIT_FOR_BTF();
    i2c_stop();
    systick_sleep(5 * (int) strlen);
}
