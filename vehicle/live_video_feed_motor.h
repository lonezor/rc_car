#pragma once

#include "general.h"

//-------------------------------------------------------------------------------------------------------------------

/* Tuning for horizontal joystick */
#define LEFT_ROTATION_SENSOR_TRESHOLD (505)
#define RIGHT_ROTATION_SENSOR_TRESHOLD (520)

/* Tuning for 10 RPM motor */
#define DC_MOTOR_ROTATION_START_THRESHOLD (45) // visible rotation
#define PWM_RANGE_START (0)
#define PWM_RANGE_END (255)

/* Rotation states */
enum rotation {
  rotation_none,
  rotation_left,
  rotation_right,
};

//-------------------------------------------------------------------------------------------------------------------

void live_video_feed_motor_tick(int joystick_rotation, int joystick_button);

//-------------------------------------------------------------------------------------------------------------------


 