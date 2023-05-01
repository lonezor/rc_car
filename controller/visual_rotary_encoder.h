#pragma once

#include <stdint.h>

//-------------------------------------------------------------------------------------------------------------------

/*
 ****** PIN CONFIGURATION ******
 *
 * Num 	Label 	Description
 * 1 	VCC/+ 	Power+ (5V)
 * 2 	GND/- 	Power-
 * 3 	SCL/C 	I2C Clock line
 * 4 	SDA/D 	I2C Data line
 *
 ****** I2C ADDRESS ******
 * 1 	2 	ADDR
 * 0 	0 	0x54
 * 0 	1 	0x55
 * 1 	0 	0x56
 * 1 	1 	0x57
 *
 */

//-------------------------------------------------------------------------------------------------------------------

/** Initialize sensor */
void
visual_rotary_encoder_init();

/** Get value between 0-1023 */
uint16_t
visual_rotary_encoder_get_value();

/** Set value between 0-1023 */
void
visual_rotary_encoder_set_value(uint16_t v);

/** True if button is pressed */
bool
visual_rotary_encoder_btn_pressed();

//-------------------------------------------------------------------------------------------------------------------




