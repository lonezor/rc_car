
#include "lcd_i2c.h"

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

/**
 * I2C Address: 0x27
 * Character width: 20
 * Number of lines: 4
 */
static LiquidCrystal_I2C lcd(0x27,20,4);

void lcd_i2c_init(bool backlight)
{
    lcd.init();

    if (backlight) {
        lcd.backlight();
    }
}

void lcd_i2c_set_cursor(int x, int y)
{
    lcd.setCursor(x,y);
}

void lcd_i2c_print(const char* msg)
{
    lcd.print(msg);
}

void lcd_i2c_write(uint8_t c)
{
    lcd.write(c);
}

void lcd_i2c_clear()
{
    lcd.clear();
}

