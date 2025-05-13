#include "Boat.h"

/**
 * Detects failures of the ekf and triggers a failsafe
 */

#ifndef EKF_CHECK_ITERATIONS_MAX
 # define EKF_CHECK_ITERATIONS_MAX      10      // 1 second (10 iterations at 10Hz) of bad variances signals a failure
#endif

#ifndef EKF_CHECK_WARNING_TIME
 # define EKF_CHECK_WARNING_TIME        (30*1000) // warnings no more than every 30 seconds
#endif

// ekf_check state
static struct {
    uint8_t  fail_count;       // iterations out of tolerance
    uint8_t  bad_variance : 1; // true once fail_count exceeds threshold
    uint32_t last_warn_time;    // throttle warning rate
} ekf_check_state;

// ekf_check - called at 10Hz, triggers failsafe on bad variances
void Boat::ekf_check()
{
    // exit if no origin yet
    Location temp_loc;
    if (!ahrs.get_origin(temp_loc)) {
        return;
    }

    // return if disarmed or disabled
    if (!arming.is_armed() || (g.fs_ekf_thresh <= 0.0f)) {
        ekf_check_state.fail_count = 0;
        ekf_check_state.bad_variance = false;
        AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;
        failsafe_ekf_off_event();
        return;
    }

    // if variances exceed threshold
    if (ekf_over_threshold()) {
        if (!ekf_check_state.bad_variance) {
            ekf_check_state.fail_count++;
            if (ekf_check_state.fail_count >= EKF_CHECK_ITERATIONS_MAX) {
                ekf_check_state.fail_count = EKF_CHECK_ITERATIONS_MAX;
                ekf_check_state.bad_variance = true;
                LOGGER_WRITE_ERROR(LogErrorSubsystem::EKFCHECK,
                                  LogErrorCode::EKFCHECK_BAD_VARIANCE);
                if ((AP_HAL::millis() - ekf_check_state.last_warn_time) > EKF_CHECK_WARNING_TIME) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "EKF variance");
                    ekf_check_state.last_warn_time = AP_HAL::millis();
                }
                failsafe_ekf_event();
            }
        }
    } else {
        if (ekf_check_state.fail_count > 0) {
            ekf_check_state.fail_count--;
            if (ekf_check_state.bad_variance && ekf_check_state.fail_count == 0) {
                ekf_check_state.bad_variance = false;
                LOGGER_WRITE_ERROR(LogErrorSubsystem::EKFCHECK,
                                  LogErrorCode::EKFCHECK_VARIANCE_CLEARED);
                failsafe_ekf_off_event();
            }
        }
    }

    AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;
}

// return true if two variances exceed threshold
bool Boat::ekf_over_threshold()
{
    if (g.fs_ekf_thresh <= 0.0f) return false;

    float pos_var, vel_var, hgt_var, tas_var;
    Vector3f mag_var;
    ahrs.get_variances(vel_var, pos_var, hgt_var, mag_var, tas_var);

    uint8_t count = 0;
    if (mag_var.length() >= g.fs_ekf_thresh) count++;
    if (vel_var >= g.fs_ekf_thresh) count++;
    if (pos_var >= g.fs_ekf_thresh) count++;

    bool flow_ok = false;
#if AP_OPTICALFLOW_ENABLED
    flow_ok = optflow.healthy();
#endif
    if (!flow_ok && (vel_var >= 2.0f * g.fs_ekf_thresh)) {
        count += 2;
    } else if (vel_var >= g.fs_ekf_thresh) {
        count++;
    }
    if (count >= 2) return true;
    return !Boat::ekf_position_ok();
}

// returns true if EKF position is acceptable
bool Boat::ekf_position_ok()
{
    if (!ahrs.have_inertial_nav()) return false;

    nav_filter_status fs;
    boat.ahrs.get_filter_status(fs);

    if (!arming.is_armed()) {
        return (fs.flags.horiz_pos_abs  || fs.flags.pred_horiz_pos_abs ||
                fs.flags.horiz_pos_rel  || fs.flags.pred_horiz_pos_rel);
    }
    return ((fs.flags.horiz_pos_abs || fs.flags.horiz_pos_rel) && !fs.flags.const_pos_mode);
}

// EKF failsafe engaged
void Boat::failsafe_ekf_event()
{
    if (failsafe.ekf) return;
    failsafe.ekf = true;
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_EKFINAV,
                       LogErrorCode::FAILSAFE_OCCURRED);
    if (!control_mode->requires_position()) return;
    switch ((enum fs_ekf_action)g.fs_ekf_action.get()) {
        case FS_EKF_DISABLE:      return;
        case FS_EKF_REPORT_ONLY:  break;
        case FS_EKF_HOLD:
        default:
            set_mode(mode_hold, ModeReason::EKF_FAILSAFE);
            break;
    }
    gcs().send_text(MAV_SEVERITY_CRITICAL, "EKF failsafe");
}

// EKF failsafe cleared
void Boat::failsafe_ekf_off_event()
{
    if (!failsafe.ekf) return;
    failsafe.ekf = false;
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_EKFINAV,
                       LogErrorCode::FAILSAFE_RESOLVED);
    gcs().send_text(MAV_SEVERITY_CRITICAL, "EKF failsafe cleared");
}
