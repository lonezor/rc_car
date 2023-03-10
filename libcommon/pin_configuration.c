
#include "pin_configuration.h"


void pin_configuration_allocate(int pin_nr, pin_type_t pin_type, const char* subsystem, const char* name, const char* description)
{

}

void pin_configuration_set(int pin_nr, int pin_mode)
{
    pinMode(pin_nr, pin_mode);
}

void pin_configuration_serial_print()
{

}