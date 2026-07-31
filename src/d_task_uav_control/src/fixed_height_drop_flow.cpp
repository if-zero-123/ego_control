#include "d_task_uav_control/fixed_height_drop_flow.h"

#include <cmath>
#include <stdexcept>

namespace d_task_uav_control {
namespace {

constexpr double kTimeEpsilon = 1e-9;

}  // namespace

const char* fixedHeightDropStateName(FixedHeightDropState state) {
    switch (state) {
        case FixedHeightDropState::NOT_READY: return "NOT_READY";
        case FixedHeightDropState::POSITIONING_INIT: return "POSITIONING_INIT";
        case FixedHeightDropState::WAIT_START: return "WAIT_START";
        case FixedHeightDropState::TAKEOFF: return "TAKEOFF";
        case FixedHeightDropState::MOVE_TO_SEARCH_START:
            return "MOVE_TO_SEARCH_START";
        case FixedHeightDropState::FORWARD_SEARCH: return "FORWARD_SEARCH";
        case FixedHeightDropState::FOLLOW_CAR: return "FOLLOW_CAR";
        case FixedHeightDropState::RELEASE: return "RELEASE";
        case FixedHeightDropState::RETURN_HOME: return "RETURN_HOME";
        case FixedHeightDropState::LAND_HOME: return "LAND_HOME";
        case FixedHeightDropState::COMPLETE: return "COMPLETE";
        case FixedHeightDropState::ABORT: return "ABORT";
    }
    return "NOT_READY";
}

FixedHeightDropFlow::FixedHeightDropFlow(
    const FixedHeightDropFlowConfig& config)
    : config_(config) {
    if (!std::isfinite(config_.alignment_stable_s)
        || !std::isfinite(config_.release_duration_s)
        || !std::isfinite(config_.release_settle_s)
        || !std::isfinite(config_.home_stable_s)
        || config_.alignment_stable_s < 0.0
        || config_.release_duration_s < 0.0
        || config_.release_settle_s < 0.0
        || config_.home_stable_s < 0.0) {
        throw std::invalid_argument("invalid fixed-height drop flow timing");
    }
}

void FixedHeightDropFlow::reset() {
    state_ = FixedHeightDropState::NOT_READY;
    final_abort_ = false;
    state_enter_s_ = 0.0;
    condition_start_s_ = -1.0;
}

void FixedHeightDropFlow::configure() {
    state_ = FixedHeightDropState::POSITIONING_INIT;
    final_abort_ = false;
    state_enter_s_ = 0.0;
    condition_start_s_ = -1.0;
}

bool FixedHeightDropFlow::markPositioningReady() {
    if (state_ != FixedHeightDropState::POSITIONING_INIT) {
        return false;
    }
    state_ = FixedHeightDropState::WAIT_START;
    condition_start_s_ = -1.0;
    return true;
}

bool FixedHeightDropFlow::start(double now_s) {
    if (state_ != FixedHeightDropState::WAIT_START || !std::isfinite(now_s)) {
        return false;
    }
    state_ = FixedHeightDropState::TAKEOFF;
    state_enter_s_ = now_s;
    condition_start_s_ = -1.0;
    return true;
}

void FixedHeightDropFlow::transition(
    FixedHeightDropState next, double now_s,
    FixedHeightDropFlowOutput& output) {
    output.previous_state = state_;
    state_ = next;
    state_enter_s_ = now_s;
    condition_start_s_ = -1.0;
    output.state = state_;
    output.state_changed = output.previous_state != state_;
}

bool FixedHeightDropFlow::conditionHeld(
    bool condition, double now_s, double required_s) {
    if (!condition) {
        condition_start_s_ = -1.0;
        return false;
    }
    if (condition_start_s_ < 0.0) {
        condition_start_s_ = now_s;
    }
    return now_s - condition_start_s_ + kTimeEpsilon >= required_s;
}

void FixedHeightDropFlow::beginAbortReturn(
    double now_s, FixedHeightDropFlowOutput& output,
    int fault_code, const char* fault_text) {
    final_abort_ = true;
    output.fault_code = fault_code;
    output.fault_text = fault_text;
    transition(FixedHeightDropState::RETURN_HOME, now_s, output);
}

FixedHeightDropFlowOutput FixedHeightDropFlow::update(
    const FixedHeightDropFlowInput& input) {
    FixedHeightDropFlowOutput output;
    output.previous_state = state_;
    output.state = state_;
    if (!std::isfinite(input.now_s)) {
        output.fault_code = 2101;
        output.fault_text = "invalid_mission_time";
        return output;
    }

    if (input.abort_requested
        && state_ != FixedHeightDropState::NOT_READY
        && state_ != FixedHeightDropState::POSITIONING_INIT
        && state_ != FixedHeightDropState::WAIT_START
        && state_ != FixedHeightDropState::COMPLETE
        && state_ != FixedHeightDropState::ABORT) {
        if (input.landed) {
            final_abort_ = true;
            transition(FixedHeightDropState::ABORT, input.now_s, output);
        } else if (state_ == FixedHeightDropState::LAND_HOME) {
            final_abort_ = true;
            output.fault_code = 2303;
            output.fault_text = "mission_abort_requested";
        } else {
            beginAbortReturn(input.now_s, output, 2303,
                             "mission_abort_requested");
        }
    } else {
        switch (state_) {
            case FixedHeightDropState::TAKEOFF:
                if (input.takeoff_complete) {
                    transition(FixedHeightDropState::MOVE_TO_SEARCH_START,
                               input.now_s, output);
                }
                break;
            case FixedHeightDropState::MOVE_TO_SEARCH_START:
                if (input.offset_reached) {
                    transition(FixedHeightDropState::FORWARD_SEARCH,
                               input.now_s, output);
                }
                break;
            case FixedHeightDropState::FORWARD_SEARCH:
                if (input.tag_detected) {
                    transition(FixedHeightDropState::FOLLOW_CAR,
                               input.now_s, output);
                } else if (input.search_endpoint_reached) {
                    beginAbortReturn(input.now_s, output, 2104,
                                     "tag_not_found_on_search_path");
                }
                break;
            case FixedHeightDropState::FOLLOW_CAR:
                if (conditionHeld(input.aligned, input.now_s,
                                  config_.alignment_stable_s)) {
                    transition(FixedHeightDropState::RELEASE,
                               input.now_s, output);
                    output.trigger_payload = true;
                }
                break;
            case FixedHeightDropState::RELEASE:
                if (input.now_s - state_enter_s_ + kTimeEpsilon
                    >= config_.release_duration_s
                       + config_.release_settle_s) {
                    transition(FixedHeightDropState::RETURN_HOME,
                               input.now_s, output);
                }
                break;
            case FixedHeightDropState::RETURN_HOME:
                if (conditionHeld(input.home_reached, input.now_s,
                                  config_.home_stable_s)) {
                    transition(FixedHeightDropState::LAND_HOME,
                               input.now_s, output);
                }
                break;
            case FixedHeightDropState::LAND_HOME:
                if (input.landed) {
                    transition(final_abort_ ? FixedHeightDropState::ABORT
                                            : FixedHeightDropState::COMPLETE,
                               input.now_s, output);
                }
                break;
            case FixedHeightDropState::NOT_READY:
            case FixedHeightDropState::POSITIONING_INIT:
            case FixedHeightDropState::WAIT_START:
            case FixedHeightDropState::COMPLETE:
            case FixedHeightDropState::ABORT:
                break;
        }
    }

    output.state = state_;
    output.terminal = state_ == FixedHeightDropState::COMPLETE
        || state_ == FixedHeightDropState::ABORT;
    output.aborted = state_ == FixedHeightDropState::ABORT;
    return output;
}

}  // namespace d_task_uav_control
