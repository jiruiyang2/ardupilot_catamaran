#include "Boat.h"

#if AP_FENCE_ENABLED

// async fence checking I/O callback at 1kHz
void Boat::fence_checks_async()
{
    const uint32_t now = AP_HAL::millis();

    // run at 10Hz
    if (!AP_HAL::timeout_expired(fence_breaches.last_check_ms, now, 100U)) {
        return;
    }
    // wait if main loop hasn't processed previous update
    if (fence_breaches.have_updates) {
        return;
    }

    fence_breaches.last_check_ms = now;
    const uint8_t orig_breaches = fence.get_breaches();
    // check for new breaches
    fence_breaches.new_breaches = fence.check();

    if (!fence_breaches.new_breaches && orig_breaches && fence.get_breaches() == 0) {
        // record clearing of breach
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode::ERROR_RESOLVED);
    }
    fence_breaches.have_updates = true;
}

// fence_check - ask fence library to check for breaches and respond
void Boat::fence_check()
{
    // only act on new breach
    if (!fence_breaches.have_updates) {
        return;
    }
    // clear update flag if disarmed
    if (!arming.is_armed()) {
        fence_breaches.have_updates = false;
        return;
    }

    if (fence_breaches.new_breaches) {
        // if action requested
        if ((FailsafeAction)fence.get_action() != FailsafeAction::None) {
            // if within give-up distance, take configured action
            if (fence.get_breach_distance(fence_breaches.new_breaches) <= AC_FENCE_GIVE_UP_DISTANCE) {
                switch ((FailsafeAction)fence.get_action()) {
                    case FailsafeAction::None:
                        break;
                    case FailsafeAction::SmartRTL:
                        if (set_mode(mode_smartrtl, ModeReason::FENCE_BREACHED)) break;
                        FALLTHROUGH;
                    case FailsafeAction::RTL:
                        if (set_mode(mode_rtl, ModeReason::FENCE_BREACHED)) break;
                        FALLTHROUGH;
                    case FailsafeAction::Hold:
                        set_mode(mode_hold, ModeReason::FENCE_BREACHED);
                        break;
                    case FailsafeAction::SmartRTL_Hold:
                        if (!set_mode(mode_smartrtl, ModeReason::FENCE_BREACHED)) {
                            set_mode(mode_hold, ModeReason::FENCE_BREACHED);
                        }
                        break;
                    case FailsafeAction::Terminate:
                        arming.disarm(AP_Arming::Method::FENCEBREACH);
                        break;
                }
            } else {
                // outside give-up distance, hold
                set_mode(mode_hold, ModeReason::FENCE_BREACHED);
            }
        }
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode(fence_breaches.new_breaches));
    }
    fence_breaches.have_updates = false;
}

#endif // AP_FENCE_ENABLED
