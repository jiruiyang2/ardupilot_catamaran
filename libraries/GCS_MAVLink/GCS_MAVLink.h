/// @file	GCS_MAVLink.h
/// @brief	One size fits all header for MAVLink integration.
#pragma once

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS  // <-- Put it right at the top

#include "GCS_MAVLink.h"

#include "mavlink/v2.0/ardupilotmega/version.h"
#include "mavlink/v2.0/ardupilotmega/mavlink.h"
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Networking/AP_Networking_Config.h>

// we have separate helpers disabled to make it possible
// to select MAVLink 1.0 in the arduino GUI build
#define MAVLINK_SEPARATE_HELPERS
#define MAVLINK_NO_CONVERSION_HELPERS

#ifdef MAVLINK_START_UART_SEND
#undef MAVLINK_START_UART_SEND
#endif
#define MAVLINK_START_UART_SEND(chan, size) comm_send_lock(chan, size)

#ifdef MAVLINK_END_UART_SEND
#undef MAVLINK_END_UART_SEND
#endif
#define MAVLINK_END_UART_SEND(chan, size) comm_send_unlock(chan)

#ifdef MAVLINK_SEND_UART_BYTES
#undef MAVLINK_SEND_UART_BYTES
#endif
#define MAVLINK_SEND_UART_BYTES(chan, buf, len) comm_send_buffer(chan, buf, len)

#ifdef MAVLINK_COMM_NUM_BUFFERS
#undef MAVLINK_COMM_NUM_BUFFERS
#endif

#if HAL_PROGRAM_SIZE_LIMIT_KB > 1024
// allow 8 telemetry ports, allowing for extra networking or CAN ports
#define MAVLINK_COMM_NUM_BUFFERS 8
#else
// allow five telemetry ports
#define MAVLINK_COMM_NUM_BUFFERS 5
#endif

#define MAVLINK_GET_CHANNEL_BUFFER 1
#define MAVLINK_GET_CHANNEL_STATUS 1


/*
  The MAVLink protocol code generator does its own alignment, so
  alignment cast warnings can be ignored
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"

#if defined(__GNUC__) && __GNUC__ >= 9
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#endif


#define MAVLINK_MAX_PAYLOAD_LEN 255

/// MAVLink streams used for each telemetry port
extern AP_HAL::UARTDriver	*mavlink_comm_port[MAVLINK_COMM_NUM_BUFFERS];
extern bool gcs_alternative_active[MAVLINK_COMM_NUM_BUFFERS];

/// MAVLink system definition
extern mavlink_system_t mavlink_system;

/// Sanity check MAVLink channel
///
/// @param chan		Channel to send to
static inline bool valid_channel(mavlink_channel_t chan)
{
    return static_cast<int>(chan) < MAVLINK_COMM_NUM_BUFFERS;
}

mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan);
mavlink_status_t* mavlink_get_channel_status(uint8_t chan);

void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len);

/// Check for available transmit space on the nominated MAVLink channel
///
/// @param chan		Channel to check 
/// @returns		Number of bytes available
uint16_t comm_get_txspace(mavlink_channel_t chan);

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

// lock and unlock a channel, for multi-threaded mavlink send
void comm_send_lock(mavlink_channel_t chan, uint16_t size);
void comm_send_unlock(mavlink_channel_t chan);
HAL_Semaphore &comm_chan_lock(mavlink_channel_t chan);

extern mavlink_system_t mavlink_system;

#pragma GCC diagnostic pop
