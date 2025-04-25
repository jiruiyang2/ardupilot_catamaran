#pragma once

#include <stdint.h>
#include <mavlink/v2.0/ardupilotmega/mavlink.h>

// Declaration only
extern "C" void comm_send_ch(mavlink_channel_t chan, uint8_t ch);
