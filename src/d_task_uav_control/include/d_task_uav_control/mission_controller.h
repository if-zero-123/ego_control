#ifndef D_TASK_UAV_CONTROL_MISSION_CONTROLLER_H_
#define D_TASK_UAV_CONTROL_MISSION_CONTROLLER_H_

#include <cstdint>
#include <limits>
#include <string>

namespace d_task_uav_control {

enum class MissionMode : uint8_t {
    DROP = 0,
    DYNAMIC_LANDING = 1,
};

enum class MissionState : uint8_t {
    NOT_READY = 0,
    POSITIONING_INIT,
    WAIT_START,
    TAKEOFF,
    HOVER_3S,
    SEARCH_CAR,
    LOCK_CAR,
    FOLLOW_CAR,
    DROP_DESCEND,
    RELEASE,
    DESCEND_HIGH,
    DESCEND_LOW,
    LAND_ON_PLATFORM,
    PLATFORM_HOLD,
    PLATFORM_TAKEOFF,
    CLIMB_TO_CRUISE,
    RETURN_HOME,
    LAND_HOME,
    COMPLETE,
    ABORT,
};

const char* missionModeName(MissionMode mode);
const char* missionStateName(MissionState state);

struct HomePosition {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
};

struct MissionControllerConfig {
    double cruise_height_m = 1.50;
    double hover_time_s = 3.0;
    double vision_lock_time_s = 0.50;
    double follow_stable_time_s = 1.0;
    double phase_stable_time_s = 0.50;
    double release_settle_time_s = 0.50;
    double platform_hold_time_s = 5.20;

    double drop_height_m = 0.80;
    double high_descent_height_m = 0.80;
    double low_descent_height_m = 0.30;
    double platform_press_depth_m = 0.10;
    double follow_lead_time_s = 0.15;

    double xy_tolerance_m = 0.12;
    double relative_speed_tolerance_mps = 0.20;
    double height_tolerance_m = 0.08;
    double home_xy_tolerance_m = 0.20;
    double home_height_tolerance_m = 0.12;

    double follow_vertical_speed_mps = 0.0;
    double high_descent_speed_mps = 0.25;
    double low_descent_speed_mps = 0.12;
    double contact_descent_speed_mps = 0.08;
    double climb_speed_mps = 0.35;

    double takeoff_timeout_s = 25.0;
    double search_timeout_s = 20.0;
    double tracking_loss_timeout_s = 1.0;
    double descent_timeout_s = 20.0;
    double platform_contact_timeout_s = 8.0;
    double platform_takeoff_timeout_s = 20.0;
    double return_timeout_s = 35.0;
    double home_land_timeout_s = 30.0;
    double total_timeout_s = 180.0;
    double request_retry_s = 0.50;

    double drop_force_descent_distance_to_d_m = 2.0;
    double drop_abort_distance_to_d_m = 0.10;
    int max_dynamic_landing_retries = 1;
};

struct MissionInput {
    double now_s = 0.0;
    bool uav_valid = false;
    double uav_x = 0.0;
    double uav_y = 0.0;
    double uav_z = 0.0;
    double uav_vx = 0.0;
    double uav_vy = 0.0;
    double uav_vz = 0.0;
    std::string bridge_state;
    uint8_t control_mode = 0;

    bool platform_valid = false;
    bool platform_vision_detected = false;
    double platform_x = 0.0;
    double platform_y = 0.0;
    double platform_z = 0.0;
    double platform_vx = 0.0;
    double platform_vy = 0.0;

    bool descent_allowed = false;
    bool safety_hold = false;
    double distance_to_d_m = std::numeric_limits<double>::infinity();
};

struct MissionCommand {
    MissionState state = MissionState::NOT_READY;
    bool request_takeoff = false;
    int override_mode_request = -1;
    bool setpoint_valid = false;
    double target_x = 0.0;
    double target_y = 0.0;
    double target_z = 0.0;
    double target_vx = 0.0;
    double target_vy = 0.0;
    double target_vz = 0.0;
    double target_yaw = 0.0;
    bool release_payload = false;
    bool request_platform_land = false;
    bool request_platform_cancel = false;
    bool request_platform_takeoff = false;
    bool request_home_land = false;
    bool complete = false;
    bool abort = false;
    int fault_code = 0;
    std::string fault_text;
};

class MissionController {
public:
    explicit MissionController(
        const MissionControllerConfig& config = MissionControllerConfig());

    void reset();
    bool configure(const std::string& mission_id, MissionMode mode);
    bool markPositioningReady(const std::string& mission_id,
                              const HomePosition& home);
    bool start(const std::string& mission_id, MissionMode mode,
               const std::string& start_reason, double now_s);
    MissionCommand update(const MissionInput& input);

    MissionState state() const { return state_; }
    MissionMode mode() const { return mode_; }
    const std::string& missionId() const { return mission_id_; }
    int retryCount() const { return retry_count_; }

private:
    enum class ClimbPurpose : uint8_t { RETURN_HOME, RETRY_LANDING };

    void transition(MissionState next, double now_s);
    bool conditionHeld(bool condition, double now_s, double required_s);
    bool requestDue(double now_s, double& last_request_s);
    bool aligned(const MissionInput& input) const;
    bool atRelativeHeight(const MissionInput& input, double height_m) const;
    double cruiseZ() const;
    void commandPlatform(const MissionInput& input, double target_z,
                         double target_vz, MissionCommand& output) const;
    void commandPosition(double x, double y, double z,
                         double vx, double vy, double vz,
                         MissionCommand& output) const;
    void beginClimb(const MissionInput& input, double now_s,
                    ClimbPurpose purpose);
    void handleTrackingFailure(const MissionInput& input,
                               MissionCommand& output,
                               const std::string& reason);
    void beginAbortReturn(const MissionInput& input,
                          MissionCommand& output,
                          int fault_code,
                          const std::string& fault_text);
    bool stateTimedOut(double now_s, double timeout_s) const;

    MissionControllerConfig config_;
    MissionState state_ = MissionState::NOT_READY;
    MissionMode mode_ = MissionMode::DROP;
    std::string mission_id_;
    HomePosition home_;
    bool positioning_ready_ = false;
    bool mission_started_ = false;
    bool payload_released_ = false;
    bool final_abort_ = false;
    int retry_count_ = 0;
    ClimbPurpose climb_purpose_ = ClimbPurpose::RETURN_HOME;

    double mission_start_s_ = 0.0;
    double state_enter_s_ = 0.0;
    double condition_start_s_ = -1.0;
    double tracking_loss_start_s_ = -1.0;
    double release_started_s_ = -1.0;
    double climb_anchor_x_ = 0.0;
    double climb_anchor_y_ = 0.0;

    double last_takeoff_request_s_ = -1e9;
    double last_override_request_s_ = -1e9;
    double last_platform_land_request_s_ = -1e9;
    double last_platform_takeoff_request_s_ = -1e9;
    double last_home_land_request_s_ = -1e9;
};

}  // namespace d_task_uav_control

#endif  // D_TASK_UAV_CONTROL_MISSION_CONTROLLER_H_
