// File: libraries/AP_Motors/AP_MotorsUGV_Catamaran.h
#pragma once

#include "AP_MotorsUGV_Catamaran.h"

class AP_MotorsUGV_Catamaran {
public:
    AP_MotorsUGV_Catamaran(AP_Motors &motors) : AP_MotorsUGV(motors) {}

    void init_outputs() override;
    void output_to_motors() override;

private:
    float _throttle; // forward/reverse
    float _yaw;      // turning left/right
};

