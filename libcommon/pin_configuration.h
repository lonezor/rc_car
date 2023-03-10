#pragma once

#include "general.h"

#include <stdbool.h>


/** Pin type */
typedef enum 
{
    pin_type_unknown,

    /** Intended use: GPIO */
    pin_type_gpio,

    /** Intended use: PWM */
    pin_type_pwm,

    /** Analog input */
    pin_type_analog_input,

    /** UART */
    pin_type_uart,

    /** I2C */
    pin_type_i2c,
}pin_type_t;

void pin_configuration_allocate(int pin_nr, pin_type_t type, const char* subsystem, const char* name, const char* description);
void pin_configuration_set(int pin_nr, int pin_mode);
void pin_configuration_serial_print();
