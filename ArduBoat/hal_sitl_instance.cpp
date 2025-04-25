// hal_sitl_instance.cpp

#define CONFIG_HAL_BOARD HAL_BOARD_SITL
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include "mavlink_globals.h"  // this declares mavlink_system
#include "HAL_SITL_Class.h"
#include "AP_HAL/AP_HAL_Boards.h"  // ✅ Ensures HAL_BOARD_SITL is known
#include "AP_HAL_SITL/AP_HAL_SITL.h"
#include "mavlink/v2.0/ardupilotmega/mavlink.h"

HAL_SITL hal_sitl;                       // ✅ defined in HAL_SITL_Class.h
const AP_HAL::HAL& hal = hal_sitl;      // ✅ ArduPilot global
