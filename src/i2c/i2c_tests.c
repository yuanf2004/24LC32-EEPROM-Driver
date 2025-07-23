#include "i2c_tests.h"

void test_eeprom_write_byte(uint16_t addr, uint16_t d){
/* Test Function for EEPROM Writing */
    eeprom_i2c_init();
    eeprom_write_byte_uint(addr, d);
}

void test_eeprom_write_char(uint16_t addr, char c){
    eeprom_i2c_init();
    eeprom_write_byte_char(addr, c);
}

void test_eeprom_read_char(uint16_t addr){
    init_uart();
    init_systick();

    eeprom_i2c_init();

    char c;
    c = eeprom_read_byte_char(addr);

    char buffer[12];
    sprintf(buffer, "%c", c);

    while(1){
        uart_print(buffer);
        systick_sleep(250);
    }
}

void test_eeprom_write_page(uint16_t addr){
/* Test function for EEPROM writing page */
    uint8_t write_arr[8] = {23,24,25,26,27,28,29,30};

    init_systick();
    eeprom_i2c_init(); 
    eeprom_write_pages_uint(addr, write_arr, 8);
}   

void test_eeprom_write_string(uint16_t addr){
    char test_str[6] = "Hello"; 

    init_systick();
    eeprom_i2c_init();
    eeprom_write_string(addr, test_str, 6);
}

void test_eeprom_read_byte(uint16_t addr){
    /* Test function for reading a byte */
    init_uart();
    init_systick();
    
    eeprom_i2c_init();

    uint16_t r = 0;
    r = eeprom_read_byte_uint(addr);

    char buffer[12];
    sprintf(buffer, "%u\r\n", r);

    while(1){
        uart_print(buffer);
        systick_sleep(250);
    }
}

void test_eeprom_read_page(uint16_t addr){
/* Test function for reading a page */
    init_uart();
    init_systick();

    eeprom_i2c_init();

    uint8_t test_arr[8];
    size_t arrsz = 8;

    eeprom_read_uint(addr, test_arr, arrsz);
    /* Keep printing every element in the array */
    int i = 0;
    while(1){
        /* Reset i if it goes over array bounds */
        if(i > 7){i = 0;}
        char buffer[12];
        sprintf(buffer, "%u - %u\r\n", i, test_arr[i]);

        uart_print(buffer);
        systick_sleep(250);
        i++;
    }
}

void test_eeprom_read_string(uint16_t addr){
/* Test function for reading a page */
    init_uart();
    init_systick();

    eeprom_i2c_init();

    char test_arr[6];
    size_t arrsz = 6;

    eeprom_read_string(addr, test_arr, arrsz);
    /* Keep printing every element in the array */
    int i = 0;
    while(1){
        /* Reset i if it goes over array bounds */
        if(i > 5){i = 0;}
        char buffer[12];
        sprintf(buffer, "%u - %c", i, test_arr[i]);

        uart_print(buffer);
        systick_sleep(250);
        i++;
    }
}

void test_eeprom_rw(uint16_t addr){
    eeprom_i2c_init();

    eeprom_write_byte_char(addr, 'z');
    eeprom_read_byte_char(addr);
}

void test_eeprom_rw_uint(uint16_t addr){
    eeprom_i2c_init();

    eeprom_write_byte_uint(addr, 99);
    eeprom_read_byte_uint(addr);
}

void test_eeprom_rw_uint_page(uint16_t addr){
    eeprom_i2c_init();

    uint8_t uint_arr[] = {1,2,3,4,5,6,7,8};
    uint8_t uint_read_arr[8];

    eeprom_write_pages_uint(addr, uint_arr, 8);
    eeprom_read_uint(addr, uint_read_arr, 8);
    }

void test_eeprom_rw_str(uint16_t addr){
    eeprom_i2c_init();
    
    char str[] = "Hello World!";
    char r_str[13];

    eeprom_write_string(addr, str, (sizeof(str) / sizeof(str[0])));
    eeprom_read_string(addr, r_str, 13);
}