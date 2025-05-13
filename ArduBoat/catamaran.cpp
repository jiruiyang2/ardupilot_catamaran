// catamaran.cpp
#include <AP_Common/AP_FWVersion.h>
#include <AP_HAL/AP_HAL.h>
#include <mavlink/common/mavlink.h>
#include <mavlink/common/mavlink_helpers.h>
#include "mavlink_globals.h"
#include "AP_Motors/AP_MotorsUGV_Catamaran.h"

extern const struct AP_HAL::HAL& hal;

static AP_MotorsUGV_Catamaran *motors;

static void send_heartbeat() {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        mavlink_system.sysid,
        mavlink_system.compid,
        &msg,
        MAV_TYPE_SURFACE_BOAT,
        MAV_AUTOPILOT_GENERIC,
        MAV_MODE_GUIDED_ARMED,
        0, MAV_STATE_ACTIVE
    );
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    for (uint16_t i = 0; i < len; i++) {
        comm_send_ch(MAVLINK_COMM_0, buf[i]);
    }
}

int main(int argc, char *argv[]) {
    hal.init(argc, argv);

    // create & init your mixer
    motors = new AP_MotorsUGV_Catamaran(*AP_Motors::get_singleton());
    motors->init_outputs();

    uint32_t last = hal.scheduler->millis();
    while (1) {
        uint32_t now = hal.scheduler->millis();

        // simple forward 50%
        motors->set_throttle(0.5f);
        motors->set_steering(0.0f);
        motors->output_to_motors();

        if (now - last > 1000) {
            send_heartbeat();
            last = now;
        }
        hal.scheduler->delay(20);
    }
    return 0;
}
