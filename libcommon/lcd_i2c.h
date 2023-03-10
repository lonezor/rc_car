#pragma once

#include <stdint.h>
#include <stdbool.h>

void lcd_i2c_init(bool backlight);
void lcd_i2c_set_cursor(int x, int y);
void lcd_i2c_print(const char* msg);
void lcd_i2c_write(uint8_t c);
void lcd_i2c_clear();

