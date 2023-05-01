#pragma once

#include <stdint.h>
#include <stdbool.h>

//-------------------------------------------------------------------------------------------------------------------

/** First magic byte */
#define MAGIC_IDX_0 (0x1e)

/** Second magic byte */
#define MAGIC_IDX_1 (0x91)

/** Third magic byte */
#define MAGIC_IDX_2 (0x4f)

/** Forth magic byte */
#define MAGIC_IDX_3 (0x3e)

/** Magic length (bytes) */
#define MAGIC_LEN (4)

/** Message flags bytes */
#define WIRELESS_MSG_FLAG_BYTES (4) // 32 flags

/** Flags bitfield: Initializer value */
#define WIRELESS_MSG_FLAG_NONE (0x0000)

/** Flags bitfield: Core lights button (front lights if any and break lights) */
#define WIRELESS_MSG_FLAG_CORE_LIGHTS_BUTTON (0x0001)

/** Flags bitfield: Extra light #1 button */
#define WIRELESS_MSG_FLAG_EXTRA_LIGHT_01_BUTTON (0x0002)

/** Flags bitfield: Extra light #2 button */
#define WIRELESS_MSG_FLAG_EXTRA_LIGHT_02_BUTTON (0x0004)

/** Flags bitfield: Extra light #3 button */
#define WIRELESS_MSG_FLAG_EXTRA_LIGHT_03_BUTTON (0x0008)

/** Flags bitfield: Left indicator button */
#define WIRELESS_MSG_FLAG_LEFT_INDICATOR_BUTTON (0x0010)

/** Flags bitfield: Right indicator button */
#define WIRELESS_MSG_FLAG_RIGHT_INDICATOR_BUTTON (0x0020)

/** Flags bitfield: Spotlight button, latching mode */
#define WIRELESS_MSG_FLAG_SPOTLIGHT_LATCHING_BUTTON (0x0040)

/** Flags bitfield: Spotlight button, momentary mode */
#define WIRELESS_MSG_FLAG_SPOTLIGHT_MOMENTARY_BUTTON (0x0080)

/** Flags bitfield: Left joystick button (context specific behaviour) */
#define WIRELESS_MSG_FLAG_LEFT_JOYSTICK_BUTTON (0x0100)

/** Flags bitfield: Right joystick button (context specific behaviour) */
#define WIRELESS_MSG_FLAG_RIGHT_JOYSTICK_BUTTON (0x0200)

/** Flags bitfield: Live video feed override button (right joystick used for video feed) */
#define WIRELESS_MSG_FLAG_LIVE_VIDEO_FEED_OVERRIDE_BUTTON (0x0400)

/** Function button (context specific behaviour) */
#define WIRELESS_MSG_FLAG_FUNCTION_BUTTON (0x0800)

/** Analog sensor data bytes */
#define ANALOG_SENSOR_DATA_BYTES (4)

//-------------------------------------------------------------------------------------------------------------------

#pragma pack(push, 1)

/**
 * @brief Wireless message structure
 * 
 * All fields are _always_ stored in network order. API functions
 * are intended for any access.
 */
typedef struct {
    /** Message magic (drop if not recognized)*/
    uint8_t magic[MAGIC_LEN];

    /** Message length bytes (including magic and length fields) */
    uint8_t length; // max: 27 bytes

    /** Bitfield of boolean flags */
    uint8_t flags[WIRELESS_MSG_FLAG_BYTES];

    /** Left joystick (context specific behaviour) */
    uint16_t analog_left_joystick;

    /** Right joystick (context specific behaviour) */
    uint16_t analog_right_joystick;

    /** Motor RPM range joystick */
    uint16_t analog_motor_rpm_range;
} wireless_msg_t;

#pragma pack(pop)

//-------------------------------------------------------------------------------------------------------------------

/** @brief Write wireless protocol message buffer
 * 
 * @param msg                      Message buffer
 * @param flags                    Flags
 * @param analog_left_joystick     Left joystick (0-1023)
 * @param analog_right_joystick    Right joystick (0-1023)
 * @param analog_motor_rpm_range   Motor RPM range joystick (0-1023)
 */
void wireless_protocol_write(wireless_msg_t* msg,
                             uint32_t flags,
                             uint16_t analog_left_joystick,
                             uint16_t analog_right_joystick,
                             uint16_t analog_motor_rpm_range);

/** @brief Validate message buffer
 * 
 * This validates magic and length field against the data
 * 
 * @param buffer       RX data buffer
 * @param buffer_len   RX data buffer length (bytes)
 * 
 * @return True when the message is valid. If false, ignore data
*/
bool wireless_protocol_valid_message(wireless_msg_t* msg,
                                     unsigned int buffer_len);

/** Parse message: Check if boolean flag has been set */
bool wireless_protocol_flag_is_set(wireless_msg_t* msg, uint32_t flag);

/** Parse message: retrieve analog sensor data */
void
wireless_protocol_read_analog_sensor_data(wireless_msg_t* msg,
                                          uint16_t* analog_left_joystick,
                                          uint16_t* analog_right_joystick,
                                          uint16_t* analog_motor_rpm_range);
                                              
//-------------------------------------------------------------------------------------------------------------------

