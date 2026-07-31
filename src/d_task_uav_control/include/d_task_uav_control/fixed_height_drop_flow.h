#ifndef D_TASK_UAV_CONTROL_FIXED_HEIGHT_DROP_FLOW_H_
#define D_TASK_UAV_CONTROL_FIXED_HEIGHT_DROP_FLOW_H_

#include <cstdint>

namespace d_task_uav_control {

enum class FixedHeightDropState : uint8_t {
    NOT_READY = 0,
    POSITIONING_INIT,
    WAIT_START,
    TAKEOFF,
    MOVE_TO_SEARCH_START,
    FORWARD_SEARCH,
    FOLLOW_CAR,
    RELEASE,
    RETURN_HOME,
    LAND_HOME,
    COMPLETE,
    ABORT,
};

const char* fixedHeightDropStateName(FixedHeightDropState state);

struct FixedHeightDropFlowConfig {
    double alignment_stable_s = 1.0;
    double release_duration_s = 5.0;
    double release_settle_s = 0.25;
    double home_stable_s = 0.50;
};

struct FixedHeightDropFlowInput {
    double now_s = 0.0;
    bool takeoff_complete = false;
    bool offset_reached = false;
    bool tag_detected = false;
    bool search_endpoint_reached = false;
    bool aligned = false;
    bool home_reached = false;
    bool landed = false;
    bool abort_requested = false;
};

struct FixedHeightDropFlowOutput {
    FixedHeightDropState previous_state = FixedHeightDropState::NOT_READY;
    FixedHeightDropState state = FixedHeightDropState::NOT_READY;
    bool state_changed = false;
    bool trigger_payload = false;
    bool terminal = false;
    bool aborted = false;
    int fault_code = 0;
    const char* fault_text = "";
};

class FixedHeightDropFlow {
public:
    explicit FixedHeightDropFlow(
        const FixedHeightDropFlowConfig& config = FixedHeightDropFlowConfig());

    void reset();
    void configure();
    bool markPositioningReady();
    bool start(double now_s);
    FixedHeightDropFlowOutput update(const FixedHeightDropFlowInput& input);

    FixedHeightDropState state() const { return state_; }
    bool finalAbort() const { return final_abort_; }

private:
    void transition(FixedHeightDropState next, double now_s,
                    FixedHeightDropFlowOutput& output);
    bool conditionHeld(bool condition, double now_s, double required_s);
    void beginAbortReturn(double now_s, FixedHeightDropFlowOutput& output,
                          int fault_code, const char* fault_text);

    FixedHeightDropFlowConfig config_;
    FixedHeightDropState state_ = FixedHeightDropState::NOT_READY;
    bool final_abort_ = false;
    double state_enter_s_ = 0.0;
    double condition_start_s_ = -1.0;
};

}  // namespace d_task_uav_control

#endif  // D_TASK_UAV_CONTROL_FIXED_HEIGHT_DROP_FLOW_H_
