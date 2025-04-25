#pragma once
#include <mavlink/v2.0/mavlink_types.h>

#ifdef __cplusplus
extern "C" {
#endif

extern mavlink_system_t mavlink_system;
void comm_send_ch(mavlink_channel_t chan, uint8_t byte);

#ifdef __cplusplus
}
#endif
