#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define CONFIG_HAL_BOARD HAL_BOARD_SITL

#include "mavlink_globals.h"
#include <mavlink/v2.0/ardupilotmega/mavlink.h>
#include "AP_HAL_SITL/AP_HAL_SITL.h"
#include "AP_HAL_SITL/comm_send_ch.h"



mavlink_system_t mavlink_system = {
    1, // System ID (sysid)
    1  // Component ID (compid)
};

extern const AP_HAL::HAL& hal;

int main(int argc, char** argv)
{
    hal.console->printf("Hello from Catamaran!\n");

    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        mavlink_system.sysid,
        mavlink_system.compid,
        &msg,
        MAV_TYPE_GCS,
        MAV_AUTOPILOT_INVALID,
        MAV_MODE_MANUAL_ARMED,
        0,
        MAV_STATE_ACTIVE
    );

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    printf("Encoded MAVLink heartbeat message (len=%d):\n", len);
    for (uint16_t i = 0; i < len; ++i) {
        printf("0x%02X ", buf[i]);
    }
    printf("\n");

    return 0;
}
