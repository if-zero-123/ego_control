#include "d_task_uav_control/payload_pulse.h"

#include <stdexcept>

namespace d_task_uav_control {

PayloadPulse::PayloadPulse(double pulse_duration_s)
    : pulse_duration_s_(pulse_duration_s) {
    if (pulse_duration_s_ < 0.0) {
        throw std::invalid_argument("payload pulse duration cannot be negative");
    }
}

bool PayloadPulse::trigger(double now_s) {
    if (triggered_) {
        return false;
    }
    triggered_ = true;
    active_ = true;
    neutral_pending_ = true;
    release_started_s_ = now_s;
    return true;
}

PayloadPulseCommand PayloadPulse::update(double now_s) {
    if (!active_ && !neutral_pending_) {
        return PayloadPulseCommand::NONE;
    }
    if (active_ && now_s - release_started_s_ < pulse_duration_s_) {
        return PayloadPulseCommand::RELEASE;
    }
    active_ = false;
    if (neutral_pending_) {
        neutral_pending_ = false;
        return PayloadPulseCommand::NEUTRAL;
    }
    return PayloadPulseCommand::NONE;
}

PayloadPulseCommand PayloadPulse::reset() {
    const bool needs_neutral = active_ || neutral_pending_;
    triggered_ = false;
    active_ = false;
    neutral_pending_ = false;
    return needs_neutral ? PayloadPulseCommand::NEUTRAL
                         : PayloadPulseCommand::NONE;
}

}  // namespace d_task_uav_control
