#pragma once

#include "vehicle.h"

/* After each idle period, an activation occurs:
 *
 * ..........***..........***..........
 *
 * The idle period is based on the iteration counter
 * so the active period consumes 'idle' duration.
 * 
 * To allow other tasks in the program the non-blocking
 * piezo_tick() is used.
 */
#define PIEZO_IDLE_TICKS_WARNING (5000)
#define PIEZO_IDLE_TICKS_ALARM (250)
#define PIEZO_IDLE_TICKS_CRITICAL (0)
#define PIEZO_ACTIVE_TICKS (100)

#define PIEZO_PIN (7)

void piezo_tick(battery_status_t status);