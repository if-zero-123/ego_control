#include "d_task_uav_control/mission_controller.h"

#include <algorithm>
#include <cmath>

namespace d_task_uav_control {

namespace {

constexpr double kTimeEpsilon = 1e-9;

}  // namespace

const char* missionModeName(MissionMode mode) {
    return mode == MissionMode::DROP ? "DROP" : "DYNAMIC_LANDING";
}

const char* missionStateName(MissionState state) {
    switch (state) {
        case MissionState::NOT_READY: return "NOT_READY";
        case MissionState::POSITIONING_INIT: return "POSITIONING_INIT";
        case MissionState::WAIT_START: return "WAIT_START";
        case MissionState::TAKEOFF: return "TAKEOFF";
        case MissionState::HOVER_3S: return "HOVER_3S";
        case MissionState::SEARCH_CAR: return "SEARCH_CAR";
        case MissionState::LOCK_CAR: return "LOCK_CAR";
        case MissionState::FOLLOW_CAR: return "FOLLOW_CAR";
        case MissionState::DROP_DESCEND: return "DROP_DESCEND";
        case MissionState::RELEASE: return "RELEASE";
        case MissionState::DESCEND_HIGH: return "DESCEND_HIGH";
        case MissionState::DESCEND_LOW: return "DESCEND_LOW";
        case MissionState::LAND_ON_PLATFORM: return "LAND_ON_PLATFORM";
        case MissionState::PLATFORM_HOLD: return "PLATFORM_HOLD";
        case MissionState::PLATFORM_TAKEOFF: return "PLATFORM_TAKEOFF";
        case MissionState::CLIMB_TO_CRUISE: return "CLIMB_TO_CRUISE";
        case MissionState::RETURN_HOME: return "RETURN_HOME";
        case MissionState::LAND_HOME: return "LAND_HOME";
        case MissionState::COMPLETE: return "COMPLETE";
        case MissionState::ABORT: return "ABORT";
    }
    return "NOT_READY";
}

MissionController::MissionController(const MissionControllerConfig& config)
    : config_(config) {}

void MissionController::reset() {
    state_ = MissionState::NOT_READY;
    mode_ = MissionMode::DROP;
    mission_id_.clear();
    home_ = HomePosition();
    positioning_ready_ = false;
    mission_started_ = false;
    payload_released_ = false;
    final_abort_ = false;
    retry_count_ = 0;
    climb_purpose_ = ClimbPurpose::RETURN_HOME;
    mission_start_s_ = 0.0;
    state_enter_s_ = 0.0;
    condition_start_s_ = -1.0;
    tracking_loss_start_s_ = -1.0;
    release_started_s_ = -1.0;
    last_takeoff_request_s_ = -1e9;
    last_override_request_s_ = -1e9;
    last_platform_land_request_s_ = -1e9;
    last_platform_takeoff_request_s_ = -1e9;
    last_home_land_request_s_ = -1e9;
}

bool MissionController::configure(const std::string& mission_id,
                                  MissionMode mode) {
    if (mission_id.empty() || mission_started_) {
        return false;
    }
    mission_id_ = mission_id;
    mode_ = mode;
    positioning_ready_ = false;
    payload_released_ = false;
    final_abort_ = false;
    retry_count_ = 0;
    state_ = MissionState::POSITIONING_INIT;
    state_enter_s_ = 0.0;
    condition_start_s_ = -1.0;
    tracking_loss_start_s_ = -1.0;
    release_started_s_ = -1.0;
    return true;
}

bool MissionController::markPositioningReady(const std::string& mission_id,
                                             const HomePosition& home) {
    if (mission_id != mission_id_ || mission_started_
        || state_ != MissionState::POSITIONING_INIT) {
        return false;
    }
    home_ = home;
    positioning_ready_ = true;
    state_ = MissionState::WAIT_START;
    return true;
}

bool MissionController::start(const std::string& mission_id, MissionMode mode,
                              const std::string& start_reason, double now_s) {
    if (!positioning_ready_ || mission_started_
        || state_ != MissionState::WAIT_START
        || mission_id != mission_id_ || mode != mode_
        || start_reason != "car_button") {
        return false;
    }
    mission_started_ = true;
    mission_start_s_ = now_s;
    transition(MissionState::TAKEOFF, now_s);
    return true;
}

void MissionController::transition(MissionState next, double now_s) {
    state_ = next;
    state_enter_s_ = now_s;
    condition_start_s_ = -1.0;
    tracking_loss_start_s_ = -1.0;
}

bool MissionController::conditionHeld(bool condition, double now_s,
                                      double required_s) {
    if (!condition) {
        condition_start_s_ = -1.0;
        return false;
    }
    if (condition_start_s_ < 0.0) {
        condition_start_s_ = now_s;
    }
    return now_s - condition_start_s_ + kTimeEpsilon >= required_s;
}

bool MissionController::requestDue(double now_s, double& last_request_s) {
    if (now_s - last_request_s + kTimeEpsilon < config_.request_retry_s) {
        return false;
    }
    last_request_s = now_s;
    return true;
}

bool MissionController::aligned(const MissionInput& input) const {
    const double position_error =
        std::hypot(input.platform_x - input.uav_x,
                   input.platform_y - input.uav_y);
    const double velocity_error =
        std::hypot(input.platform_vx - input.uav_vx,
                   input.platform_vy - input.uav_vy);
    return input.platform_valid
        && position_error <= config_.xy_tolerance_m
        && velocity_error <= config_.relative_speed_tolerance_mps;
}

bool MissionController::atRelativeHeight(const MissionInput& input,
                                         double height_m) const {
    return std::abs((input.uav_z - input.platform_z) - height_m)
        <= config_.height_tolerance_m;
}

double MissionController::cruiseZ() const {
    return home_.z + config_.cruise_height_m;
}

void MissionController::commandPosition(double x, double y, double z,
                                        double vx, double vy, double vz,
                                        MissionCommand& output) const {
    output.setpoint_valid = true;
    output.target_x = x;
    output.target_y = y;
    output.target_z = z;
    output.target_vx = vx;
    output.target_vy = vy;
    output.target_vz = vz;
    output.target_yaw = home_.yaw;
}

void MissionController::commandPlatform(const MissionInput& input,
                                        double target_z, double target_vz,
                                        MissionCommand& output) const {
    const double lead = config_.follow_lead_time_s;
    commandPosition(
        input.platform_x + input.platform_vx * lead,
        input.platform_y + input.platform_vy * lead,
        target_z,
        input.platform_vx,
        input.platform_vy,
        target_vz,
        output);
}

bool MissionController::stateTimedOut(double now_s, double timeout_s) const {
    return timeout_s > 0.0 && now_s - state_enter_s_ > timeout_s;
}

void MissionController::beginClimb(const MissionInput& input, double now_s,
                                   ClimbPurpose purpose) {
    climb_anchor_x_ = input.uav_x;
    climb_anchor_y_ = input.uav_y;
    climb_purpose_ = purpose;
    transition(MissionState::CLIMB_TO_CRUISE, now_s);
}

void MissionController::beginAbortReturn(const MissionInput& input,
                                         MissionCommand& output,
                                         int fault_code,
                                         const std::string& fault_text) {
    final_abort_ = true;
    output.fault_code = fault_code;
    output.fault_text = fault_text;
    if (input.bridge_state == "IDLE" || input.bridge_state.empty()) {
        transition(MissionState::ABORT, input.now_s);
        output.abort = true;
        return;
    }
    beginClimb(input, input.now_s, ClimbPurpose::RETURN_HOME);
}

void MissionController::handleTrackingFailure(
    const MissionInput& input, MissionCommand& output,
    const std::string& reason) {
    if (retry_count_ < config_.max_dynamic_landing_retries) {
        ++retry_count_;
        if (state_ == MissionState::LAND_ON_PLATFORM
            || input.bridge_state == "PLATFORM_LANDING") {
            output.request_platform_cancel = true;
        }
        beginClimb(input, input.now_s, ClimbPurpose::RETRY_LANDING);
        output.fault_code = 2201;
        output.fault_text = reason + "_retry";
        return;
    }
    if (state_ == MissionState::LAND_ON_PLATFORM
        || input.bridge_state == "PLATFORM_LANDING") {
        output.request_platform_cancel = true;
    }
    beginAbortReturn(input, output, 2202, reason + "_retry_exhausted");
}

MissionCommand MissionController::update(const MissionInput& input) {
    MissionCommand output;
    output.state = state_;
    if (!mission_started_) {
        return output;
    }
    if (!std::isfinite(input.now_s) || !input.uav_valid) {
        output.fault_code = 2101;
        output.fault_text = "uav_odometry_invalid";
        return output;
    }
    if (config_.total_timeout_s > 0.0
        && input.now_s - mission_start_s_ > config_.total_timeout_s
        && state_ != MissionState::LAND_HOME
        && state_ != MissionState::COMPLETE
        && state_ != MissionState::ABORT) {
        beginAbortReturn(input, output, 2102, "mission_timeout");
        output.state = state_;
        return output;
    }

    switch (state_) {
        case MissionState::TAKEOFF:
            if (input.bridge_state == "HOVER") {
                transition(MissionState::HOVER_3S, input.now_s);
            } else if (stateTimedOut(input.now_s, config_.takeoff_timeout_s)) {
                beginAbortReturn(input, output, 2103, "takeoff_timeout");
            } else if (requestDue(input.now_s, last_takeoff_request_s_)) {
                output.request_takeoff = true;
                output.target_z = cruiseZ();
            }
            break;

        case MissionState::HOVER_3S:
            if (input.now_s - state_enter_s_ + kTimeEpsilon
                    >= config_.hover_time_s) {
                transition(MissionState::SEARCH_CAR, input.now_s);
            }
            break;

        case MissionState::SEARCH_CAR:
            if (input.control_mode != 1
                && requestDue(input.now_s, last_override_request_s_)) {
                output.override_mode_request = 1;
            }
            if (input.platform_valid) {
                commandPlatform(input, cruiseZ(), 0.0, output);
            } else {
                commandPosition(input.uav_x, input.uav_y, cruiseZ(),
                                0.0, 0.0, 0.0, output);
            }
            if (conditionHeld(input.platform_valid
                                  && input.platform_vision_detected,
                              input.now_s, config_.vision_lock_time_s)) {
                transition(MissionState::LOCK_CAR, input.now_s);
            } else if (stateTimedOut(input.now_s, config_.search_timeout_s)) {
                beginAbortReturn(input, output, 2104, "platform_search_timeout");
            }
            break;

        case MissionState::LOCK_CAR:
            if (!input.platform_valid || !input.platform_vision_detected) {
                transition(MissionState::SEARCH_CAR, input.now_s);
                break;
            }
            commandPlatform(input, cruiseZ(), 0.0, output);
            if (conditionHeld(aligned(input), input.now_s,
                              config_.vision_lock_time_s)) {
                transition(MissionState::FOLLOW_CAR, input.now_s);
            }
            break;

        case MissionState::FOLLOW_CAR: {
            if (!input.platform_valid) {
                if (tracking_loss_start_s_ < 0.0) {
                    tracking_loss_start_s_ = input.now_s;
                }
                commandPosition(input.uav_x, input.uav_y, input.uav_z,
                                0.0, 0.0, 0.0, output);
                if (input.now_s - tracking_loss_start_s_
                        > config_.tracking_loss_timeout_s) {
                    if (mode_ == MissionMode::DYNAMIC_LANDING) {
                        handleTrackingFailure(input, output, "platform_tracking_lost");
                    } else {
                        beginAbortReturn(input, output, 2105,
                                         "platform_tracking_lost");
                    }
                }
                break;
            }
            tracking_loss_start_s_ = -1.0;
            commandPlatform(input, cruiseZ(), 0.0, output);
            const bool force_drop = mode_ == MissionMode::DROP
                && input.distance_to_d_m
                    <= config_.drop_force_descent_distance_to_d_m;
            if (conditionHeld(aligned(input), input.now_s,
                              config_.follow_stable_time_s)
                || force_drop) {
                transition(mode_ == MissionMode::DROP
                               ? MissionState::DROP_DESCEND
                               : MissionState::DESCEND_HIGH,
                           input.now_s);
            }
            break;
        }

        case MissionState::DROP_DESCEND:
            if (!input.platform_valid) {
                beginAbortReturn(input, output, 2106,
                                 "drop_platform_tracking_lost");
                break;
            }
            commandPlatform(input, input.platform_z + config_.drop_height_m,
                            -config_.high_descent_speed_mps, output);
            if (input.distance_to_d_m <= config_.drop_abort_distance_to_d_m) {
                beginAbortReturn(input, output, 2107, "drop_window_missed");
            } else if (conditionHeld(
                           aligned(input)
                               && atRelativeHeight(input, config_.drop_height_m),
                           input.now_s, config_.phase_stable_time_s)) {
                transition(MissionState::RELEASE, input.now_s);
            } else if (stateTimedOut(input.now_s, config_.descent_timeout_s)) {
                beginAbortReturn(input, output, 2108, "drop_descent_timeout");
            }
            break;

        case MissionState::RELEASE:
            if (!payload_released_) {
                payload_released_ = true;
                release_started_s_ = input.now_s;
                output.release_payload = true;
            }
            if (input.platform_valid) {
                commandPlatform(input, input.platform_z + config_.drop_height_m,
                                0.0, output);
            }
            if (input.now_s - release_started_s_ + kTimeEpsilon
                    >= config_.release_settle_time_s) {
                beginClimb(input, input.now_s, ClimbPurpose::RETURN_HOME);
            }
            break;

        case MissionState::DESCEND_HIGH:
        case MissionState::DESCEND_LOW: {
            const bool high = state_ == MissionState::DESCEND_HIGH;
            const double target_height = high
                ? config_.high_descent_height_m
                : config_.low_descent_height_m;
            const double speed = high
                ? config_.high_descent_speed_mps
                : config_.low_descent_speed_mps;
            if (!input.platform_valid) {
                if (tracking_loss_start_s_ < 0.0) {
                    tracking_loss_start_s_ = input.now_s;
                }
                commandPosition(input.uav_x, input.uav_y, input.uav_z,
                                0.0, 0.0, 0.0, output);
                if (input.now_s - tracking_loss_start_s_
                        > config_.tracking_loss_timeout_s) {
                    handleTrackingFailure(input, output,
                                          "dynamic_tracking_lost");
                }
                break;
            }
            tracking_loss_start_s_ = -1.0;
            if (!input.descent_allowed || input.safety_hold) {
                commandPlatform(input, input.uav_z, 0.0, output);
                condition_start_s_ = -1.0;
                break;
            }
            commandPlatform(input, input.platform_z + target_height,
                            -speed, output);
            if (conditionHeld(aligned(input)
                                  && atRelativeHeight(input, target_height),
                              input.now_s, config_.phase_stable_time_s)) {
                transition(high ? MissionState::DESCEND_LOW
                                : MissionState::LAND_ON_PLATFORM,
                           input.now_s);
            } else if (stateTimedOut(input.now_s, config_.descent_timeout_s)) {
                handleTrackingFailure(input, output,
                                      high ? "high_descent_timeout"
                                           : "low_descent_timeout");
            }
            break;
        }

        case MissionState::LAND_ON_PLATFORM:
            if (input.bridge_state == "PLATFORM_LANDED") {
                transition(MissionState::PLATFORM_HOLD, input.now_s);
                break;
            }
            if (!input.platform_valid || !input.descent_allowed
                || input.safety_hold) {
                commandPosition(input.uav_x, input.uav_y, input.uav_z,
                                0.0, 0.0, 0.0, output);
            } else {
                commandPlatform(
                    input,
                    input.platform_z - config_.platform_press_depth_m,
                    -config_.contact_descent_speed_mps,
                    output);
                if (requestDue(input.now_s, last_platform_land_request_s_)) {
                    output.request_platform_land = true;
                }
            }
            if (stateTimedOut(input.now_s,
                              config_.platform_contact_timeout_s)) {
                handleTrackingFailure(input, output,
                                      "platform_contact_timeout");
            }
            break;

        case MissionState::PLATFORM_HOLD:
            if (input.bridge_state != "PLATFORM_LANDED") {
                transition(MissionState::ABORT, input.now_s);
                output.abort = true;
                output.fault_code = 2203;
                output.fault_text = "unexpected_platform_disarm_state";
            } else if (input.now_s - state_enter_s_ + kTimeEpsilon
                    >= config_.platform_hold_time_s) {
                transition(MissionState::PLATFORM_TAKEOFF, input.now_s);
            }
            break;

        case MissionState::PLATFORM_TAKEOFF:
            if (input.platform_valid) {
                commandPlatform(input, cruiseZ(), config_.climb_speed_mps,
                                output);
            } else {
                commandPosition(input.uav_x, input.uav_y, cruiseZ(),
                                0.0, 0.0, config_.climb_speed_mps, output);
            }
            if (requestDue(input.now_s, last_platform_takeoff_request_s_)) {
                output.request_platform_takeoff = true;
            }
            if (input.bridge_state == "HOVER"
                && std::abs(input.uav_z - cruiseZ())
                    <= config_.height_tolerance_m) {
                beginClimb(input, input.now_s, ClimbPurpose::RETURN_HOME);
            } else if (stateTimedOut(input.now_s,
                                     config_.platform_takeoff_timeout_s)) {
                if (input.bridge_state == "PLATFORM_TAKEOFF"
                    || input.bridge_state == "HOVER"
                    || input.bridge_state == "TRACKING") {
                    final_abort_ = true;
                    beginClimb(input, input.now_s, ClimbPurpose::RETURN_HOME);
                } else {
                    transition(MissionState::ABORT, input.now_s);
                    output.abort = true;
                }
                output.fault_code = 2204;
                output.fault_text = "platform_takeoff_timeout";
            }
            break;

        case MissionState::CLIMB_TO_CRUISE:
            commandPosition(climb_anchor_x_, climb_anchor_y_, cruiseZ(),
                            0.0, 0.0, config_.climb_speed_mps, output);
            if (std::abs(input.uav_z - cruiseZ())
                    <= config_.height_tolerance_m) {
                transition(climb_purpose_ == ClimbPurpose::RETRY_LANDING
                               ? MissionState::SEARCH_CAR
                               : MissionState::RETURN_HOME,
                           input.now_s);
            } else if (stateTimedOut(input.now_s, config_.takeoff_timeout_s)) {
                final_abort_ = true;
                transition(MissionState::RETURN_HOME, input.now_s);
                output.fault_code = 2205;
                output.fault_text = "recovery_climb_timeout";
            }
            break;

        case MissionState::RETURN_HOME: {
            commandPosition(home_.x, home_.y, cruiseZ(),
                            0.0, 0.0, 0.0, output);
            const bool home_reached =
                std::hypot(input.uav_x - home_.x, input.uav_y - home_.y)
                    <= config_.home_xy_tolerance_m
                && std::abs(input.uav_z - cruiseZ())
                    <= config_.home_height_tolerance_m;
            if (conditionHeld(home_reached, input.now_s,
                              config_.phase_stable_time_s)) {
                transition(MissionState::LAND_HOME, input.now_s);
            } else if (stateTimedOut(input.now_s, config_.return_timeout_s)) {
                final_abort_ = true;
                transition(MissionState::LAND_HOME, input.now_s);
                output.fault_code = 2206;
                output.fault_text = "return_home_timeout";
            }
            break;
        }

        case MissionState::LAND_HOME:
            if (input.bridge_state == "IDLE") {
                transition(final_abort_ ? MissionState::ABORT
                                        : MissionState::COMPLETE,
                           input.now_s);
                output.abort = final_abort_;
                output.complete = !final_abort_;
            } else if (requestDue(input.now_s, last_home_land_request_s_)) {
                output.request_home_land = true;
            }
            if (stateTimedOut(input.now_s, config_.home_land_timeout_s)) {
                transition(MissionState::ABORT, input.now_s);
                output.abort = true;
                output.fault_code = 2207;
                output.fault_text = "home_landing_timeout";
            }
            break;

        case MissionState::COMPLETE:
            output.complete = true;
            break;
        case MissionState::ABORT:
            output.abort = true;
            break;
        case MissionState::NOT_READY:
        case MissionState::POSITIONING_INIT:
        case MissionState::WAIT_START:
            break;
    }

    output.state = state_;
    return output;
}

}  // namespace d_task_uav_control
