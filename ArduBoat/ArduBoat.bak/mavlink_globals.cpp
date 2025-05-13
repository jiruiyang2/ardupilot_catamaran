#include <AP_Common/AP_FWVersion.h>
#include "mavlink_globals.h"
#include <cstdio>
mavlink_system_t mavlink_system = {1,1};
void comm_send_ch(mavlink_channel_t chan, uint8_t byte) {
    printf("0x%02X ", byte);

// Provide the actual definition
mavlink_system_t mavlink_system = {1, 1};

void comm_send_ch(mavlink_channel_t chan, uint8_t byte) {
    printf("[CH %d] 0x%02X\n", chan, byte);
}
