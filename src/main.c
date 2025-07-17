#include "main.h"

int main(){  


    /* 93 to 100 should all be 21 */
    //test_eeprom_write_page(93);
    //test_eeprom_write_byte(80, 21);

    test_eeprom_read_page(93);

    while(1){
    };
}