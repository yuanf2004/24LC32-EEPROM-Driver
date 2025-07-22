#include "i2c.h"

void test_eeprom_write_byte(uint16_t addr, uint16_t d);

void test_eeprom_write_char(uint16_t addr, char c);

void test_eeprom_read_char(uint16_t addr);

void test_eeprom_write_page(uint16_t addr);

void test_eeprom_write_string(uint16_t addr);

void test_eeprom_read_byte(uint16_t addr);

void test_eeprom_read_page(uint16_t addr);

void test_eeprom_read_string(uint16_t addr);

void test_eeprom_rw(uint16_t addr);