#include "live_video_feed_motor.h"
#include "interpolation.h"
#include "pin_layout.h"

/* Lerp variables */
double g_pwm_left_rotation = 0;
double g_pwm_right_rotation = 0;

/** Current physical rotation state */
enum rotation g_physical_rotation = rotation_none;

void live_video_feed_motor_tick(int joystick_rotation, int joystick_button)
{
  // Determine target rotation
  enum rotation user_requested_rotation = rotation_none;
  if (joystick_rotation < LEFT_ROTATION_SENSOR_TRESHOLD) {
    user_requested_rotation = rotation_left;
  } else if (joystick_rotation > RIGHT_ROTATION_SENSOR_TRESHOLD) {
    user_requested_rotation = rotation_right;
  }
  // Override: Joystick button requests instant zero rotation
  bool zero_rpm_override = false;
  if (joystick_button == 0) {
    user_requested_rotation = rotation_none;
    zero_rpm_override = true;
    g_pwm_left_rotation = 0;
    g_pwm_right_rotation = 0;
  }

  // Only change direction of rotation when idle
  // The motor is capable of instant rotation, but a
  // smooth transition is needed here
  if ((g_pwm_left_rotation < DC_MOTOR_ROTATION_START_THRESHOLD && 
      g_pwm_right_rotation < DC_MOTOR_ROTATION_START_THRESHOLD) ||
      zero_rpm_override) {
    g_physical_rotation = user_requested_rotation;

    switch (g_physical_rotation) {
      case rotation_none:
        digitalWrite(PIN_LIVE_VIDEO_FEED_MOTOR_IN1,LOW);
        digitalWrite(PIN_LIVE_VIDEO_FEED_MOTOR_IN2,LOW);
        break;
      case rotation_left:
        digitalWrite(PIN_LIVE_VIDEO_FEED_MOTOR_IN1,HIGH);
        digitalWrite(PIN_LIVE_VIDEO_FEED_MOTOR_IN2,LOW);
        break;
      case rotation_right:
        digitalWrite(PIN_LIVE_VIDEO_FEED_MOTOR_IN1,LOW);
        digitalWrite(PIN_LIVE_VIDEO_FEED_MOTOR_IN2,HIGH);
        break;
    }
  }

  // Only half of the analog value range is used for mapping
  // PWM value range. Left and right direction are mutually
  // exclusive and they are value mapped separately.
  // Two PWM variables are updated in parallel, based on rotation target
  // The relevant variable in each situation is used for
  // the actual physical rotation. Pull down non-user requested
  // target to zero.

  /*** Calculate PWM values using lerp function ***/

 double lerp_min_value = DC_MOTOR_ROTATION_START_THRESHOLD - 1;

  switch (user_requested_rotation) {
    case rotation_left: {
      double mapped_value = (double)map(joystick_rotation,LEFT_ROTATION_SENSOR_TRESHOLD,0,PWM_RANGE_START,PWM_RANGE_END); 
      // Only permit user control for physically active rotation
      // The condition to transition direction is close to idle speed
      if (g_physical_rotation == user_requested_rotation) {
        lerp_update(&g_pwm_left_rotation, (double)mapped_value, lerp_min_value);
      }
      lerp_update(&g_pwm_right_rotation, (double)0, 0);
      break;
    }
    case rotation_right: {
      double mapped_value = (double)map(joystick_rotation,RIGHT_ROTATION_SENSOR_TRESHOLD,1023,PWM_RANGE_START,PWM_RANGE_END); 
      lerp_update(&g_pwm_left_rotation, (double)0, 0);

      // Only permit user control for physically active rotation
      // The condition to transition direction is close to idle speed
      if (g_physical_rotation == user_requested_rotation) {
        lerp_update(&g_pwm_right_rotation, (double)mapped_value, lerp_min_value);
      }
      break;
    }
    case rotation_none:
      lerp_update(&g_pwm_left_rotation, (double)0, 0);
      lerp_update(&g_pwm_right_rotation, (double)0, 0);
      break;
  }

  /*** Write PWM using for the physically active direction ***/
  switch (g_physical_rotation) {
    case rotation_left:
      analogWrite(PIN_LIVE_VIDEO_FEED_MOTOR_PWM,(int)g_pwm_left_rotation);
      break;
    case rotation_right:
      analogWrite(PIN_LIVE_VIDEO_FEED_MOTOR_PWM,(int)g_pwm_right_rotation);
      break;
  }
  
#ifdef DEBUG
  char msg[1024];
  snprintf(msg, sizeof(msg), "joystick_rotation %d, joystick_button %d, left %d, right %d", joystick_rotation, joystick_button, (int)g_pwm_left_rotation, (int)g_pwm_right_rotation);
  Serial.println(msg); 
#endif

  





}