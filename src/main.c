#include "main.h"

int main(){  

    init_uart();
    init_systick();

    //test_eeprom_write_byte();
    uint8_t a = test_eeprom_read_byte();
    char buf[10];
    sprintf(buf, "%u", a);

    while(1){
        uart_print(buf);
        systick_sleep(1000);
    };
}