#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "wireless_protocol.h"

//-------------------------------------------------------------------------------------------------------------------

/** Virtual wire role */
typedef enum {
    /** Initializer value */
    virtual_wire_role_unknown,

    /** Transmitter */
    virtual_wire_role_transmitter,

    /** Receiver*/
    virtual_wire_role_receiver,
} virtual_wire_role_t;

//-------------------------------------------------------------------------------------------------------------------

void virtual_wire_init(virtual_wire_role_t role);
void virtual_wire_tx(wireless_msg_t* msg);
bool virtual_wire_rx(wireless_msg_t* msg);

//-------------------------------------------------------------------------------------------------------------------