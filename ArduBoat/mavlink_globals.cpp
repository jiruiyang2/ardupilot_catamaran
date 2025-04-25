#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include "mavlink_globals.h"  // Contains extern declaration
#include <mavlink/v2.0/ardupilotmega/mavlink.h>  // Now safe to include

#include <cstdio>
#include "../../libraries/AP_HAL_SITL/AP_HAL_SITL.h"

// Provide the actual definition
mavlink_system_t mavlink_system = {1, 1};

void comm_send_ch(mavlink_channel_t chan, uint8_t byte) {
    printf("[CH %d] 0x%02X\n", chan, byte);
}
