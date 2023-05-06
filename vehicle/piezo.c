#include "piezo.h"

static int g_piezo_active_ticks = 0;

void piezo_tick(battery_status_t status)
{
    switch (status) {
        case battery_status_warning:
            if (g_iteration_counter % PIEZO_IDLE_TICKS_WARNING == 0) {
                g_piezo_active_ticks = PIEZO_ACTIVE_TICKS;
            }
            break;
        case battery_status_alarm:
            if (g_iteration_counter % PIEZO_IDLE_TICKS_ALARM == 0) {
                g_piezo_active_ticks = PIEZO_ACTIVE_TICKS;
            }
            break;
        default:
            break;
    }

    if (status == battery_status_critical) {
        digitalWrite(PIEZO_PIN, HIGH);
    } else if (g_piezo_active_ticks > 0) {
        digitalWrite(PIEZO_PIN, HIGH);
        g_piezo_active_ticks--;
    } else {
        digitalWrite(PIEZO_PIN, LOW);
    }
}