#include<stdio.h>
#include "Boat.h"

// Function to set a desired pitch angle according to throttle
void Boat::balancebot_pitch_control(float &throttle)
{
    // calculate desired pitch angle
    const float demanded_pitch = radians(-throttle * 0.01f * g2.bal_pitch_max) + radians(g2.bal_pitch_trim);

    // calculate required throttle using PID controller
    throttle = g2.attitude_control.get_throttle_out_from_pitch(demanded_pitch, radians(g2.bal_pitch_max), g2.motors.limit.throttle_lower || g2.motors.limit.throttle_upper, G_Dt) * 100.0f;
}

// returns true if vehicle is a balance bot
// called in AP_MotorsUGV::output()
// this affects whether the vehicle tries to control its pitch with throttle output
bool Boat::is_balancebot() const
{
    return ((enum frame_class)g2.frame_class.get() == FRAME_BALANCEBOT);
}
