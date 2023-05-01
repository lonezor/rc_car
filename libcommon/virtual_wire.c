#include "virtual_wire.h"
#include "common_pin_layout.h"

#include <VirtualWire.h>

static virtual_wire_role_t g_role = virtual_wire_role_unknown;

const int transmit_pin = PIN_VIRTUALWIRE_TX;
const int receive_pin = PIN_VIRTUALWIRE_RX;
const int transmit_en_pin = 3;

uint8_t buf[VW_MAX_MESSAGE_LEN];
uint8_t buflen = VW_MAX_MESSAGE_LEN;

void virtual_wire_init(virtual_wire_role_t role)
{
    g_role = role;

    switch (g_role) {
        case virtual_wire_role_transmitter:
            vw_set_tx_pin(transmit_pin);
            vw_set_rx_pin(receive_pin);
            vw_set_ptt_pin(transmit_en_pin);
            vw_set_ptt_inverted(true); // Required for DR3100
            vw_setup(2000);       // Bits per sec
            break;

        case virtual_wire_role_receiver:
            vw_set_tx_pin(transmit_pin);
            vw_set_rx_pin(receive_pin);
            vw_set_ptt_pin(transmit_en_pin);
            vw_set_ptt_inverted(true); // Required for DR3100
            vw_setup(2000);      // Bits per sec
            vw_rx_start();       // Start the receiver PLL running
            break;
    }
}

void virtual_wire_tx(wireless_msg_t* msg)
{
    vw_send((uint8_t *)msg, sizeof(wireless_msg_t));
    vw_wait_tx();
}

bool virtual_wire_rx(wireless_msg_t* msg)
{
    memset(msg, 0, sizeof(wireless_msg_t));

    // No data available
    if (!vw_get_message(buf, &buflen))
    {
        return false;
    }

    // Invalid data, ignore
    if (!wireless_protocol_valid_message((wireless_msg_t*)buf, (unsigned int)buflen)) {
        return false;
    }

    memcpy(msg, buf, sizeof(wireless_msg_t));

    return true;
}