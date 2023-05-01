#include "vehicle.h"

void pcu_setup()
{
  neopixel_wheel_animation_setup();

  #ifdef DEBUG
    Serial.begin(115200);
  #endif /* DEBUG */

      pin_configuration_allocate(VEHICLE_PIN_LIVE_VIDEO_FEED_MOTOR_PWM, pin_type_pwm, "Video Feed Motor Controller", "LIVE_VIDEO_FEED_MOTOR_PWM", "Live video feed motor: PWM");

      pin_configuration_allocate(VEHICLE_PIN_LIVE_VIDEO_FEED_MOTOR_IN1, pin_type_gpio, "Video Feed Motor Controller", "LIVE_VIDEO_FEED_MOTOR_IN1", "Live video feed motor: IN1");
      pin_configuration_set(VEHICLE_PIN_LIVE_VIDEO_FEED_MOTOR_IN1, OUTPUT);

      pin_configuration_allocate(VEHICLE_PIN_LIVE_VIDEO_FEED_MOTOR_IN2, pin_type_gpio, "Video Feed Motor Controller", "LIVE_VIDEO_FEED_MOTOR_IN2", "Live video feed motor: IN2");
      pin_configuration_set(VEHICLE_PIN_LIVE_VIDEO_FEED_MOTOR_IN2, OUTPUT);

    virtual_wire_init(virtual_wire_role_receiver);
    
    //lcd_i2c_init(true);
}

void pcu_loop()
{
static uint32_t iterations = 0;
iterations++;

 static bool core_lights_button = false;
 static bool extra_light_01_button = false;
 static bool extra_light_02_button = false;
 static bool extra_light_03_button = false;
 static bool function_button = false;
 static bool left_indicator_button = false;
 static bool live_video_feed_override_button = false;
 static bool spotlight_momentary_button = false;
 static bool spotlight_latching_button = false;
 static bool right_indicator_button = false;
 static bool left_joystick_button = false;
 static bool  right_joystick_button = false;
 static uint16_t analog_left_joystick = 0;
 static uint16_t analog_right_joystick = 0;
 static uint16_t analog_motor_rpm_range = 0;

  // RF event has arrived
  wireless_msg_t msg;
  if (virtual_wire_rx(&msg)) {
    Serial.println("RX data");
    // Update vehicle state
    wireless_protocol_read_analog_sensor_data(&msg,
                                                &analog_left_joystick,
                                                &analog_right_joystick,
                                                &analog_motor_rpm_range);

    core_lights_button = wireless_protocol_flag_is_set(&msg, WIRELESS_MSG_FLAG_CORE_LIGHTS_BUTTON);
    extra_light_01_button = wireless_protocol_flag_is_set(&msg, WIRELESS_MSG_FLAG_EXTRA_LIGHT_01_BUTTON);
    extra_light_02_button = wireless_protocol_flag_is_set(&msg, WIRELESS_MSG_FLAG_EXTRA_LIGHT_02_BUTTON);
    extra_light_03_button = wireless_protocol_flag_is_set(&msg, WIRELESS_MSG_FLAG_EXTRA_LIGHT_03_BUTTON);
    function_button = wireless_protocol_flag_is_set(&msg, WIRELESS_MSG_FLAG_FUNCTION_BUTTON);
    left_indicator_button = wireless_protocol_flag_is_set(&msg, WIRELESS_MSG_FLAG_LEFT_INDICATOR_BUTTON);
    live_video_feed_override_button = wireless_protocol_flag_is_set(&msg, WIRELESS_MSG_FLAG_LIVE_VIDEO_FEED_OVERRIDE_BUTTON);
    spotlight_momentary_button = wireless_protocol_flag_is_set(&msg, WIRELESS_MSG_FLAG_SPOTLIGHT_MOMENTARY_BUTTON);
    spotlight_latching_button = wireless_protocol_flag_is_set(&msg, WIRELESS_MSG_FLAG_SPOTLIGHT_LATCHING_BUTTON);
    right_indicator_button = wireless_protocol_flag_is_set(&msg, WIRELESS_MSG_FLAG_RIGHT_INDICATOR_BUTTON);
    left_joystick_button = wireless_protocol_flag_is_set(&msg, WIRELESS_MSG_FLAG_LEFT_JOYSTICK_BUTTON);
    right_joystick_button = wireless_protocol_flag_is_set(&msg, WIRELESS_MSG_FLAG_RIGHT_JOYSTICK_BUTTON);
  }

  if (live_video_feed_override_button > 0) {
    live_video_feed_motor_tick(analog_right_joystick, right_joystick_button);
  }

 // lcd_i2c_set_cursor(0,0);
  //lcd_i2c_print(m);

  if (iterations % 50 == 0) {
    neopixel_wheel_animation_tick();
  }


  delay(1);
}

