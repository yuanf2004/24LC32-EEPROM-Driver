#include "main.h"

int main(){  

    init_uart();
    init_systick();

    test_eeprom_write_byte(99, 101);

    uint16_t a = test_eeprom_read_byte(99);
    char buf[10];
    sprintf(buf, "%u\r\n", a);

    while(1){
        uart_print(buf);
        systick_sleep(1000);
    };
}