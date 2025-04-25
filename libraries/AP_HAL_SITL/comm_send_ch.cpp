#include "comm_send_ch.h"
#include "AP_HAL_SITL/AP_HAL_SITL.h"

extern "C" void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
    (void)chan;
    (void)ch;
}
