#pragma once
#include <AP_Common/AP_FWVersion.h>
#include <mavlink.h>
#include <mavlink_helpers.h>



#ifdef __cplusplus
extern "C" {
#endif

extern mavlink_system_t mavlink_system;
void comm_send_ch(mavlink_channel_t chan, uint8_t byte);

#ifdef __cplusplus
}
#endif
