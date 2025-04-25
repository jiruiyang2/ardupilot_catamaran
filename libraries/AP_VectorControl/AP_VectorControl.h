#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <stdint.h>

// Optional: Parameters for tuning from GCS
struct VectorControlParams {
    float yaw_gain;
    float pitch_gain;
    float thrust_scale;
};

class AP_VectorControl {
public:
    AP_VectorControl();

    // Set the current orientation
    void set_attitude(float yaw, float pitch, float roll);

    // Set target vector or desired movement
    void set_target_vector(float x, float y, float z);

    // Compute actuator outputs for vectored thrust
    void compute(float &out_left, float &out_right);

    // Access to params
    VectorControlParams params;

private:
    float _yaw, _pitch, _roll;
    float _target_x, _target_y, _target_z;
};
