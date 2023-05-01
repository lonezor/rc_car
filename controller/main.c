
#include "controller_pin_layout.h"
#include "general.h"
#include "pin_configuration.h"
#include "virtual_wire.h"
#include "visual_rotary_encoder.h"
#include "wireless_protocol.h"

//-------------------------------------------------------------------------------------------------------------------

#define MOTOR_RPM_RANGE_ANALOG_DEFAULT (320)

typedef enum {
    motor_range_button_state_released,
    motor_range_button_state_pressed,
} motor_range_button_state_t;

//-------------------------------------------------------------------------------------------------------------------

static void pin_configuration()
{
    // Engine control panel
    pin_configuration_allocate(CONTROLLER_PIN_LEFT_JOYSTICK_ANALOG_X, pin_type_analog_input, "Engine Control", "LEFT_JOYSTICK_ANALOG_X", "Reserved for future use");
    pin_configuration_allocate(CONTROLLER_PIN_LEFT_JOYSTICK_ANALOG_Y, pin_type_analog_input, "Engine Control", "LEFT_JOYSTICK_ANALOG_Y", "Left joystick y axis for forward/reverse steering");
    pin_configuration_allocate(CONTROLLER_PIN_LEFT_JOYSTICK_BUTTON, pin_type_gpio, "Engine Control", "LEFT_JOYSTICK_BUTTON", "Left joystick button (context specific behaviour)");
    pin_configuration_set(CONTROLLER_PIN_LEFT_JOYSTICK_BUTTON, INPUT_PULLUP);
    
    pin_configuration_allocate(CONTROLLER_PIN_RIGHT_JOYSTICK_ANALOG_X, pin_type_analog_input, "Engine Control", "RIGHT_JOYSTICK_ANALOG_X", "Right joystick x axis for left/right steering");
    pin_configuration_allocate(CONTROLLER_PIN_RIGHT_JOYSTICK_ANALOG_Y, pin_type_analog_input, "Engine Control", "RIGHT_JOYSTICK_ANALOG_Y", "Reserved for future use");
    pin_configuration_allocate(CONTROLLER_PIN_RIGHT_JOYSTICK_BUTTON, pin_type_gpio, "Engine Control", "RIGHT_JOYSTICK_BUTTON", "Right joystick button (context specific behaviour)");
    pin_configuration_set(CONTROLLER_PIN_RIGHT_JOYSTICK_BUTTON, INPUT_PULLUP);

    // Vehicle state panel
    pin_configuration_allocate(CONTROLLER_PIN_LEFT_INDICATOR_BUTTON, pin_type_gpio, "Vehicle State Buttons", "LEFT_INDICATOR_BUTTON", "Left indicator button");
    pin_configuration_set(CONTROLLER_PIN_LEFT_INDICATOR_BUTTON, INPUT_PULLUP);
    pin_configuration_allocate(CONTROLLER_PIN_LEFT_INDICATOR_BUTTON_LED, pin_type_gpio, "Vehicle State Buttons", "LEFT_INDICATOR_BUTTON_LED", "Left indicator button, LED indicator");
    pin_configuration_set(CONTROLLER_PIN_LEFT_INDICATOR_BUTTON_LED, OUTPUT);

    pin_configuration_allocate(CONTROLLER_PIN_SPOTLIGHT_LATCHING_BUTTON, pin_type_gpio, "Vehicle State Buttons", "SPOTLIGHT_LATCHING_BUTTON", "Spotlight latching button");
    pin_configuration_set(CONTROLLER_PIN_SPOTLIGHT_LATCHING_BUTTON, INPUT_PULLUP);
    pin_configuration_allocate(CONTROLLER_PIN_SPOTLIGHT_LATCHING_BUTTON_LED, pin_type_gpio, "Vehicle State Buttons", "SPOTLIGHT_LATCHING_BUTTON_LED", "Spotlight latching button, LED indicator");
    pin_configuration_set(CONTROLLER_PIN_SPOTLIGHT_LATCHING_BUTTON_LED, OUTPUT);

    pin_configuration_allocate(CONTROLLER_PIN_SPOTLIGHT_MOMENTARY_BUTTON, pin_type_gpio, "Vehicle State Buttons", "SPOTLIGHT_MOMENTARY_BUTTON", "Spotlight momentary button");
    pin_configuration_set(CONTROLLER_PIN_SPOTLIGHT_MOMENTARY_BUTTON, INPUT_PULLUP);
    pin_configuration_allocate(CONTROLLER_PIN_SPOTLIGHT_MOMENTARY_BUTTON_LED, pin_type_gpio, "Vehicle State Buttons", "SPOTLIGHT_MOMENTARY_BUTTON_LED", "Spotlight momentary button, LED indicator");
    pin_configuration_set(CONTROLLER_PIN_SPOTLIGHT_MOMENTARY_BUTTON_LED, OUTPUT);

    pin_configuration_allocate(CONTROLLER_PIN_LIVE_VIDEO_FEED_OVERRIDE_BUTTON, pin_type_gpio, "Vehicle State Buttons", "LIVE_VIDEO_FEED_OVERRIDE_BUTTON", "Live video feed override button");
    pin_configuration_set(CONTROLLER_PIN_LIVE_VIDEO_FEED_OVERRIDE_BUTTON, INPUT_PULLUP);
    pin_configuration_allocate(CONTROLLER_PIN_LIVE_VIDEO_FEED_OVERRIDE_BUTTON_LED, pin_type_gpio, "Vehicle State Buttons", "LIVE_VIDEO_FEED_OVERRIDE_BUTTON_LED", "Live video feed override button, LED indicator");
    pin_configuration_set(CONTROLLER_PIN_LIVE_VIDEO_FEED_OVERRIDE_BUTTON_LED, OUTPUT);

    pin_configuration_allocate(CONTROLLER_PIN_RIGHT_INDICATOR_BUTTON, pin_type_gpio, "Vehicle State Buttons", "RIGHT_INDICATOR_BUTTON", "Right indicator button");
    pin_configuration_set(CONTROLLER_PIN_RIGHT_INDICATOR_BUTTON, INPUT_PULLUP);
    pin_configuration_allocate(CONTROLLER_PIN_RIGHT_INDICATOR_BUTTON_LED, pin_type_gpio, "Vehicle State Buttons", "RIGHT_INDICATOR_BUTTON_LED", "Right indicator button, LED indicator");
    pin_configuration_set(CONTROLLER_PIN_RIGHT_INDICATOR_BUTTON_LED, OUTPUT);

    pin_configuration_allocate(CONTROLLER_PIN_CORE_LIGHTS_BUTTON, pin_type_gpio, "Vehicle State Buttons", "CORE_LIGHTS_BUTTON", "Core lights button");
    pin_configuration_set(CONTROLLER_PIN_CORE_LIGHTS_BUTTON, INPUT_PULLUP);
    pin_configuration_allocate(CONTROLLER_PIN_CORE_LIGHTS_BUTTON_LED, pin_type_gpio, "Vehicle State Buttons", "CORE_LIGHTS_BUTTON_LED", "Core lights button, LED indicator");
    pin_configuration_set(CONTROLLER_PIN_CORE_LIGHTS_BUTTON_LED, OUTPUT);

    pin_configuration_allocate(CONTROLLER_PIN_EXTRA_LIGHT_01_BUTTON, pin_type_gpio, "Vehicle State Buttons", "EXTRA_LIGHT_01_BUTTON", "Extra light #1 button");
    pin_configuration_set(CONTROLLER_PIN_EXTRA_LIGHT_01_BUTTON, INPUT_PULLUP);
    pin_configuration_allocate(CONTROLLER_PIN_EXTRA_LIGHT_01_BUTTON_LED, pin_type_gpio, "Vehicle State Buttons", "EXTRA_LIGHT_01_BUTTON_LED", "Extra light #1 button, LED indicator");
    pin_configuration_set(CONTROLLER_PIN_EXTRA_LIGHT_01_BUTTON_LED, OUTPUT);

    pin_configuration_allocate(CONTROLLER_PIN_EXTRA_LIGHT_02_BUTTON, pin_type_gpio, "Vehicle State Buttons", "EXTRA_LIGHT_02_BUTTON", "Extra light #2 button");
    pin_configuration_set(CONTROLLER_PIN_EXTRA_LIGHT_02_BUTTON, INPUT_PULLUP);
    pin_configuration_allocate(CONTROLLER_PIN_EXTRA_LIGHT_02_BUTTON_LED, pin_type_gpio, "Vehicle State Buttons", "EXTRA_LIGHT_02_BUTTON_LED", "Extra light #2 button, LED indicator");
    pin_configuration_set(CONTROLLER_PIN_EXTRA_LIGHT_02_BUTTON_LED, OUTPUT);

    pin_configuration_allocate(CONTROLLER_PIN_EXTRA_LIGHT_03_BUTTON, pin_type_gpio, "Vehicle State Buttons", "EXTRA_LIGHT_03_BUTTON", "Extra light #3 button");
    pin_configuration_set(CONTROLLER_PIN_EXTRA_LIGHT_03_BUTTON, INPUT_PULLUP);
    pin_configuration_allocate(CONTROLLER_PIN_EXTRA_LIGHT_03_BUTTON_LED, pin_type_gpio, "Vehicle State Buttons", "EXTRA_LIGHT_03_BUTTON_LED", "Extra light #3 button, LED indicator");
    pin_configuration_set(CONTROLLER_PIN_EXTRA_LIGHT_03_BUTTON_LED, OUTPUT);

    pin_configuration_allocate(CONTROLLER_PIN_FUNCTION_BUTTON, pin_type_gpio, "Vehicle State Buttons", "FUNCTION_BUTTON", "Function button");
    pin_configuration_set(CONTROLLER_PIN_FUNCTION_BUTTON, INPUT_PULLUP);
    pin_configuration_allocate(CONTROLLER_PIN_FUNCTION_BUTTON_LED, pin_type_gpio, "Vehicle State Buttons", "FUNCTION_BUTTON_LED", "Function button, LED indicator");
    pin_configuration_set(CONTROLLER_PIN_FUNCTION_BUTTON_LED, OUTPUT);
}

//-------------------------------------------------------------------------------------------------------------------

static void rotary_encoder_init()
{
    visual_rotary_encoder_init();
    visual_rotary_encoder_set_value(MOTOR_RPM_RANGE_ANALOG_DEFAULT);
}

//-------------------------------------------------------------------------------------------------------------------

static void wireless_init()
{
    virtual_wire_init(virtual_wire_role_transmitter);
}

//-------------------------------------------------------------------------------------------------------------------

static void wireless_send(uint32_t flags,
                          uint16_t analog_left_joystick,
                          uint16_t analog_right_joystick,
                          uint16_t analog_motor_rpm_range)
{
    wireless_msg_t msg;
    wireless_protocol_write(&msg,
                             flags,
                             analog_left_joystick,
                             analog_right_joystick,
                             analog_motor_rpm_range);
    virtual_wire_tx(&msg);
}

//-------------------------------------------------------------------------------------------------------------------

void setup() {
    pin_configuration();
    rotary_encoder_init();
    wireless_init();

#ifdef DEBUG
    Serial.begin(9600);
#endif /* DEBUG */
}

//-------------------------------------------------------------------------------------------------------------------

void loop() {
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////// SENSOR INPUT SECTION /////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    // Engine control panel
    int left_joystick_analog_x = analogRead(CONTROLLER_PIN_LEFT_JOYSTICK_ANALOG_X);
    int left_joystick_analog_y = analogRead(CONTROLLER_PIN_LEFT_JOYSTICK_ANALOG_Y);
    int left_joystick_button = digitalRead(CONTROLLER_PIN_LEFT_JOYSTICK_BUTTON);

    int motor_rpm_range = (int)visual_rotary_encoder_get_value();
    int motor_range_reset_button = (int)visual_rotary_encoder_btn_pressed();

    int right_joystick_analog_x = analogRead(CONTROLLER_PIN_RIGHT_JOYSTICK_ANALOG_X);
    int right_joystick_analog_y = analogRead(CONTROLLER_PIN_RIGHT_JOYSTICK_ANALOG_Y);
    int right_joystick_button = digitalRead(CONTROLLER_PIN_RIGHT_JOYSTICK_BUTTON);

    // Vehicle state panel
    int left_indicator_button = digitalRead(CONTROLLER_PIN_LEFT_INDICATOR_BUTTON);

    int spotlight_latching_button = digitalRead(CONTROLLER_PIN_SPOTLIGHT_LATCHING_BUTTON);

    int spotlight_momentary_button = digitalRead(CONTROLLER_PIN_SPOTLIGHT_MOMENTARY_BUTTON);

    int live_video_feed_override_button = digitalRead(CONTROLLER_PIN_LIVE_VIDEO_FEED_OVERRIDE_BUTTON);

    int right_indicator_button = digitalRead(CONTROLLER_PIN_RIGHT_INDICATOR_BUTTON);

    int core_lights_button = digitalRead(CONTROLLER_PIN_CORE_LIGHTS_BUTTON);

    int extra_light_01_button = digitalRead(CONTROLLER_PIN_EXTRA_LIGHT_01_BUTTON);

    int extra_light_02_button = digitalRead(CONTROLLER_PIN_EXTRA_LIGHT_02_BUTTON);

    int extra_light_03_button = digitalRead(CONTROLLER_PIN_EXTRA_LIGHT_03_BUTTON);

    int function_button = digitalRead(CONTROLLER_PIN_FUNCTION_BUTTON);

    ///////////////////////////////////////////////////////////////////////////
    //////////////////////// SENSOR ACTION SECTION ////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    uint32_t flags = 0;

    // Vehicle state panel
    if (left_indicator_button == 0) {
        // Only allow activation if other button is inactive
        if (right_indicator_button != 0) {
            flags |= WIRELESS_MSG_FLAG_LEFT_INDICATOR_BUTTON;
            digitalWrite(CONTROLLER_PIN_LEFT_INDICATOR_BUTTON_LED, HIGH);
        }
    } else {
        digitalWrite(CONTROLLER_PIN_LEFT_INDICATOR_BUTTON_LED, LOW);
    }

    if (spotlight_latching_button == 0) {
        flags |= WIRELESS_MSG_FLAG_SPOTLIGHT_LATCHING_BUTTON;
        digitalWrite(CONTROLLER_PIN_SPOTLIGHT_LATCHING_BUTTON_LED, HIGH);
    } else {
        digitalWrite(CONTROLLER_PIN_SPOTLIGHT_LATCHING_BUTTON_LED, LOW);
    }

    if (spotlight_momentary_button == 0) {
        // Only allow activation if other button is inactive
        if (spotlight_latching_button != 0) {
            flags |= WIRELESS_MSG_FLAG_SPOTLIGHT_MOMENTARY_BUTTON;
            digitalWrite(CONTROLLER_PIN_SPOTLIGHT_MOMENTARY_BUTTON_LED, HIGH);
        }
    } else {
        digitalWrite(CONTROLLER_PIN_SPOTLIGHT_MOMENTARY_BUTTON_LED, LOW);
    }

    if (live_video_feed_override_button == 0) {
        flags |= WIRELESS_MSG_FLAG_LIVE_VIDEO_FEED_OVERRIDE_BUTTON;
        digitalWrite(CONTROLLER_PIN_LIVE_VIDEO_FEED_OVERRIDE_BUTTON_LED, HIGH);
    } else {
        digitalWrite(CONTROLLER_PIN_LIVE_VIDEO_FEED_OVERRIDE_BUTTON_LED, LOW);
    }

    if (right_indicator_button == 0) {
        // Only allow activation if other button is inactive
        if (left_indicator_button != 0) {
            flags |= WIRELESS_MSG_FLAG_RIGHT_INDICATOR_BUTTON;
            digitalWrite(CONTROLLER_PIN_RIGHT_INDICATOR_BUTTON_LED, HIGH);
        }
    } else {
        digitalWrite(CONTROLLER_PIN_RIGHT_INDICATOR_BUTTON_LED, LOW);
    }

    if (core_lights_button == 0) {
        flags |= WIRELESS_MSG_FLAG_CORE_LIGHTS_BUTTON;
        digitalWrite(CONTROLLER_PIN_CORE_LIGHTS_BUTTON_LED, HIGH);
    } else {
        digitalWrite(CONTROLLER_PIN_CORE_LIGHTS_BUTTON_LED, LOW);
    }

    if (extra_light_01_button == 0) {
        flags |= WIRELESS_MSG_FLAG_EXTRA_LIGHT_01_BUTTON;
        digitalWrite(CONTROLLER_PIN_EXTRA_LIGHT_01_BUTTON_LED, HIGH);
    } else {
        digitalWrite(CONTROLLER_PIN_EXTRA_LIGHT_01_BUTTON_LED, LOW);
    }

    if (extra_light_02_button == 0) {
        flags |= WIRELESS_MSG_FLAG_EXTRA_LIGHT_02_BUTTON;
        digitalWrite(CONTROLLER_PIN_EXTRA_LIGHT_02_BUTTON_LED, HIGH);
    } else {
        digitalWrite(CONTROLLER_PIN_EXTRA_LIGHT_02_BUTTON_LED, LOW);
    }

    if (extra_light_03_button == 0) {
        flags |= WIRELESS_MSG_FLAG_EXTRA_LIGHT_03_BUTTON;
        digitalWrite(CONTROLLER_PIN_EXTRA_LIGHT_03_BUTTON_LED, HIGH);
    } else {
        digitalWrite(CONTROLLER_PIN_EXTRA_LIGHT_03_BUTTON_LED, LOW);
    }

    if (function_button == 0) {
        flags |= WIRELESS_MSG_FLAG_FUNCTION_BUTTON;
        digitalWrite(CONTROLLER_PIN_FUNCTION_BUTTON_LED, HIGH);
    } else {
        digitalWrite(CONTROLLER_PIN_FUNCTION_BUTTON_LED, LOW);
    }

    if (left_joystick_button == 0) {
        flags |= WIRELESS_MSG_FLAG_LEFT_JOYSTICK_BUTTON;
    }

    if (right_joystick_button == 0) {
        flags |= WIRELESS_MSG_FLAG_RIGHT_JOYSTICK_BUTTON;
    }

    static motor_range_button_state_t motor_range_button_state = motor_range_button_state_released;
    if (motor_range_reset_button > 0) {
        switch (motor_range_button_state) {
            case motor_range_button_state_released:
                motor_range_button_state = motor_range_button_state_pressed;
                visual_rotary_encoder_set_value(MOTOR_RPM_RANGE_ANALOG_DEFAULT);
                break;
            case motor_range_button_state_pressed:
                motor_range_button_state = motor_range_button_state_released;
                break;
        }
    }

    wireless_send(flags,
                  (uint16_t)left_joystick_analog_y,
                  (uint16_t)right_joystick_analog_x,
                  (uint16_t) motor_rpm_range);

    delay(1);
}

//-------------------------------------------------------------------------------------------------------------------