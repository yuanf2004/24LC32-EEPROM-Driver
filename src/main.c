#include "main.h"

int main(){  

    init_uart();
    init_systick();

    /* 93 to 100 should all be 21 */
    //test_eeprom_write_page(93);

    uint16_t a = test_eeprom_read_byte(97);
    char buf[10];
    sprintf(buf, "%u\r\n", a);

    while(1){
        uart_print(buf);
        systick_sleep(500);
    };
}