// hal_sitl_instance.cpp

// Tell SITL_cmdline.cpp to emit the real `main()`
#define HAL_SITL_INSTANCE_MAIN

// Include the SITL launcher (console, map, UDP link, etc.)
#include "../libraries/AP_HAL_SITL/SITL_cmdline.cpp"
