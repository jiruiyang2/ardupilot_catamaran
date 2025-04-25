// File: libraries/AP_Motors/AP_MotorsUGV_Catamaran.cpp
#include "AP_MotorsUGV_Catamaran.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

void AP_MotorsUGV_Catamaran::init_outputs() {
    add_motor_num(1); // S3
    add_motor_num(2); // S4
}

void AP_MotorsUGV_Catamaran::output_to_motors() {
    // Get the desired throttle and yaw from the controllers
    _throttle = get_throttle();
    _yaw = get_steering();

    hal.console->printf("[CatamaranMixer] Throttle: %.2f, Yaw: %.2f | M1: %.2f, M2: %.2f\n", _throttle, _yaw, m1, m2); 
    float gain_throttle = 1.0f;
    float gain_yaw = 0.5f;


    // Motor 1 (S3) = throttle - yaw
    // Motor 2 (S4) = throttle + yaw
    float m1 = _throttle - _yaw;
    float m2 = _throttle + _yaw;

    // Constrain outputs to [-1.0, 1.0]
    m1 = constrain_float(m1, -1.0f, 1.0f);
    m2 = constrain_float(m2, -1.0f, 1.0f);

    // Send outputs to motors
    set_motor_output(1, m1);
    set_motor_output(2, m2);
}
// File: Rover/motors.cpp (add this section)
#include "AP_MotorsUGV_Catamaran.h"

void Rover::setup_motors()
{
    switch (g2.frame_type) {
        case 99: // Custom Catamaran mixer
            motors = new AP_MotorsUGV_Catamaran(*AP_Motors::get_singleton());
            break;

        default:
            // fallback to standard setup if needed
            break;
    }
}
