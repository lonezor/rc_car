#include "visual_rotary_encoder.h"

#include <DFRobot_VisualRotaryEncoder.h>

static DFRobot_VisualRotaryEncoder_I2C sensor(/*i2cAddr = */0x54, /*i2cBus = */&Wire);

uint16_t
visual_rotary_encoder_get_value()
{
    return sensor.getEncoderValue();
}

void
visual_rotary_encoder_set_value(uint16_t v)
{
    sensor.setEncoderValue(v);
}

bool
visual_rotary_encoder_btn_pressed()
{
    return sensor.detectButtonDown();
}



