//
// functions to support precision landing
//

#include "Boat.h"

#if AC_PRECLAND_ENABLED

void Boat::init_precland()
{
    // scheduler table specifies 400Hz, but we can call it no faster
    // than the scheduler loop rate:
    boat.precland.init(MIN(400, scheduler.get_loop_rate_hz()));
}

void Boat::update_precland()
{
    // alt will be unused if we pass false through as the second parameter:
    return precland.update(0, false);
}
#endif
