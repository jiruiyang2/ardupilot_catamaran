#include "AP_VectorControl.h"
#include <cmath>

AP_VectorControl::AP_VectorControl()
{
    params.yaw_gain = 1.0f;
    params.pitch_gain = 1.0f;
    params.thrust_scale = 1.0f;
}

void AP_VectorControl::set_attitude(float yaw, float pitch, float roll)
{
    _yaw = yaw;
    _pitch = pitch;
    _roll = roll;
}

void AP_VectorControl::set_target_vector(float x, float y, float z)
{
    _target_x = x;
    _target_y = y;
    _target_z = z;
}

void AP_VectorControl::compute(float &out_left, float &out_right)
{
    // Very simple example logic
    float yaw_correction = _target_y * params.yaw_gain;
    float thrust = _target_x * params.thrust_scale;

    out_left = thrust - yaw_correction;
    out_right = thrust + yaw_correction;

    // Clamp
    out_left = constrain_float(out_left, -1.0f, 1.0f);
    out_right = constrain_float(out_right, -1.0f, 1.0f);
}
