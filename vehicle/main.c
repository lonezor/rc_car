
#include "general.h"
#include "pin_layout.h"
#include "pin_configuration.h"
#include "live_video_feed_motor.h"
#include "lcd_i2c.h"
#include "neopixel_wheel.h"

#include "visual_rotary_encoder.h"

void setup() {

  //Serial.begin(9600);
  //Serial1.begin(9600);

  //pin_configuration_allocate(PIN_LIVE_VIDEO_FEED_HORIZONTAL_JOYSTICK, pin_type_analog_input, "Live video feed rotation", "LIVE_VIDEO_FEED_HORIZONTAL_JOYSTICK", "Horizontal joystick for controlling motor direction");
  //pin_configuration_allocate(PIN_LIVE_VIDEO_FEED_MOTOR_BTN, pin_type_gpio, "Live video feed rotation", "LIVE_VIDEO_FEED_MOTOR_BTN", "Joystick button for instantly stopping motor rotation");
  //pin_configuration_allocate(PIN_LIVE_VIDEO_FEED_MOTOR_PWM, pin_type_pwm, "Live video feed rotation", "LIVE_VIDEO_FEED_MOTOR_PWM", "Motor controller: PWM signal");
  //pin_configuration_allocate(PIN_LIVE_VIDEO_FEED_MOTOR_IN1, pin_type_pwm, "Live video feed rotation", "LIVE_VIDEO_FEED_MOTOR_IN1", "Motor controller: IN1 signal");
  //pin_configuration_allocate(PIN_LIVE_VIDEO_FEED_MOTOR_IN2, pin_type_pwm, "Live video feed rotation", "LIVE_VIDEO_FEED_MOTOR_IN2", "Motor controller: IN2 signal");

  //pin_configuration_set(PIN_LIVE_VIDEO_FEED_MOTOR_BTN, INPUT_PULLUP);
  //pin_configuration_set(PIN_LIVE_VIDEO_FEED_MOTOR_PWM, OUTPUT);
  //pin_configuration_set(PIN_LIVE_VIDEO_FEED_MOTOR_IN1, OUTPUT);
  //pin_configuration_set(PIN_LIVE_VIDEO_FEED_MOTOR_IN2, OUTPUT);

  //pin_configuration_serial_print();

  //lcd_i2c_init(true);
  //lcd_i2c_print("hello world...");
  //lcd_i2c_set_cursor(0,1);
  //lcd_i2c_print("... line 2");

  //lcd_i2c_set_cursor(0,2);
  //lcd_i2c_print("... line 3");

  //lcd_i2c_set_cursor(0,3);
  //lcd_i2c_print("... line 4");

  //visual_rotary_encoder_set_value(512);

  neopixel_wheel_animation_setup();
}

int counter = 0;

void loop() {

  //////////////////////// INPUT SIGNALS //////////////////////////

#ifdef RADIO_INTERFACE_AVAILABLE
  //int joystick_rotation = ...
  //int joystick_button = ...
#else
  // Read horizontal joystick value
  //int joystick_rotation = analogRead(PIN_LIVE_VIDEO_FEED_HORIZONTAL_JOYSTICK);
  //int joystick_button = digitalRead(PIN_LIVE_VIDEO_FEED_MOTOR_BTN);
#endif /* RADIO_INTERFACE_AVAILABLE */

  //////////////////////// ACT ON SIGNALS //////////////////////////

  // Individually controlled regulators
  //live_video_feed_motor_tick(joystick_rotation, joystick_button);

  //if (visual_rotary_encoder_btn_pressed()) {
//    counter = 50;
//  }

//  if (counter > 0) {
    //counter--;
  //}
  
  //Serial1.println("helloa");
  //Serial2.println("hellob");
  //Serial3.println("helloc");



  //char msg[1024];
  //snprintf(msg, sizeof(msg), "v %d, btn %d", visual_rotary_encoder_get_value(), counter > 0);
  //Serial.println(msg);

  neopixel_wheel_animation_loop();

  // Tick delay
  delay(10);
}