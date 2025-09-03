#include "main.h"


int main(){  
    eeprom_i2c_init();
    eeprom_read_byte_char(100);
    while(1){
    };
}