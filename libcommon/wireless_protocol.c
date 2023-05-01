#include "wireless_protocol.h"

#include <string.h>

//-------------------------------------------------------------------------------------------------------------------

#define htons(x) ( (((x)<<8)&0xFF00) | (((x)>>8)&0xFF) )
#define ntohs(x) htons(x)
#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)

//-------------------------------------------------------------------------------------------------------------------

void wireless_protocol_write(wireless_msg_t* msg,
                             uint32_t flags,
                             uint16_t analog_left_joystick,
                             uint16_t analog_right_joystick,
                             uint16_t analog_motor_rpm_range)
{
    memset(msg, 0, sizeof(wireless_msg_t));

    msg->magic[0] = MAGIC_IDX_0;
    msg->magic[1] = MAGIC_IDX_1;
    msg->magic[2] = MAGIC_IDX_2;
    msg->magic[3] = MAGIC_IDX_3;

    msg->length = sizeof(wireless_msg_t);

    uint32_t network_order = htonl(flags);
    memcpy(&msg->flags[0], &network_order, WIRELESS_MSG_FLAG_BYTES);

    msg->analog_left_joystick = htons(analog_left_joystick);
    msg->analog_right_joystick = htons(analog_right_joystick);
    msg->analog_motor_rpm_range = htons(analog_motor_rpm_range);
}

//-------------------------------------------------------------------------------------------------------------------

bool wireless_protocol_valid_message(wireless_msg_t* msg, unsigned int buffer_len)
{
    // Message length
    if (buffer_len < sizeof(wireless_msg_t)) {
        return false;
    }

    // Message magic
    if (msg->magic[0] != MAGIC_IDX_0) {
        return false;
    }
    if (msg->magic[1] != MAGIC_IDX_1) {
        return false;
    }
    if (msg->magic[2] != MAGIC_IDX_2) {
        return false;
    }
    if (msg->magic[3] != MAGIC_IDX_3) {
        return false;
    }

    return true;
}

//-------------------------------------------------------------------------------------------------------------------

bool wireless_protocol_flag_is_set(wireless_msg_t* msg, uint32_t flag)
{
    uint32_t host_order = ntohl(*((uint32_t*)&msg->flags[0]));
    return (host_order & flag) > 0;
}

//-------------------------------------------------------------------------------------------------------------------

void wireless_protocol_read_analog_sensor_data(wireless_msg_t* msg,
                                               uint16_t* analog_left_joystick,
                                               uint16_t* analog_right_joystick,
                                               uint16_t* analog_motor_rpm_range)
{
    *analog_left_joystick = ntohs(msg->analog_left_joystick);
    *analog_right_joystick = ntohs(msg->analog_right_joystick);
    *analog_motor_rpm_range = ntohs(msg->analog_motor_rpm_range);
}

//-------------------------------------------------------------------------------------------------------------------
