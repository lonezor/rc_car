#pragma once

#include "general.h"
#include "lcd_i2c.h"
#include "live_video_feed_motor.h"
#include "neopixel_wheel.h"
#include "pin_configuration.h"
#include "vehicle_pin_layout.h"
#include "vehicle.h"
#include "virtual_wire.h"
#include "wireless_protocol.h"

// Primary Control Unit (PCU)
void pcu_setup();
void pcu_loop();

// Secondary Control Unit (SCU)
void scu_setup();
void scu_loop();

// Voltage Measurement Unit (VMU)
void vmu_setup();
void vmu_loop();

extern uint16_t g_iteration_counter;

typedef enum {
  battery_status_normal,
  battery_status_warning,
  battery_status_alarm,
  battery_status_critical,
} battery_status_t;
