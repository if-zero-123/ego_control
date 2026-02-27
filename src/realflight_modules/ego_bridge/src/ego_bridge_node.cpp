/**
 * @file ego_bridge_node.cpp
 * @brief EGO-Planner → PX4 bridge with 6-state FSM.
 *
 * States: IDLE → PRE_OFFBOARD → TAKEOFF → HOVER ⇄ TRACKING → LANDING → IDLE
 * TRACKING has an OVERRIDE sub-mode for external control handover.
 */

#include <ros/ros.h>
#include <Eigen/Dense>

// Messages
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64MultiArray.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>

// ─────────────────────────────────────────────────────────────
//  FSM states
// ─────────────────────────────────────────────────────────────
enum class State : uint8_t {
    IDLE          = 0,
    PRE_OFFBOARD  = 1,
    TAKEOFF       = 2,
    HOVER         = 3,
    TRACKING      = 4,
    LANDING       = 5
};

static const char* stateStr(State s) {
    switch (s) {
        case State::IDLE:          return "IDLE";
        case State::PRE_OFFBOARD:  return "PRE_OFFBOARD";
        case State::TAKEOFF:       return "TAKEOFF";
        case State::HOVER:         return "HOVER";
        case State::TRACKING:      return "TRACKING";
        case State::LANDING:       return "LANDING";
    }
    return "UNKNOWN";
}

// ─────────────────────────────────────────────────────────────
//  Parameters
// ─────────────────────────────────────────────────────────────
struct Params {
    double ctrl_freq;

    // Offboard
    int    pre_send_count;
    double offboard_timeout;
    bool   enable_auto_arm;

    // Takeoff
    double takeoff_height;
    double takeoff_speed;
    double motor_warmup_time;
    double motor_warmup_height;
    double decel_distance;
    double reach_threshold;
    double min_speed;
    double delay_trigger_time;

    // Landing
    double landing_speed;
    double slow_height;
    double slow_factor;
    double land_pos_deviation;
    double land_vel_threshold;
    double land_hold_time;

    // Timeout
    double cmd_timeout;
    double odom_timeout;

    // Reach detect
    double reach_pos_threshold;
    double reach_vel_threshold;
    double reach_hold_time;

    // Override
    bool   override_enable;
    double override_cmd_timeout;

    // Debug
    bool debug_enable;

    void load(ros::NodeHandle& nh) {
        nh.param("ctrl_freq",                   ctrl_freq,            50.0);

        nh.param("offboard/pre_send_count",     pre_send_count,       100);
        nh.param("offboard/timeout",            offboard_timeout,     5.0);
        nh.param("offboard/enable_auto_arm",    enable_auto_arm,      true);

        nh.param("takeoff/height",              takeoff_height,       1.0);
        nh.param("takeoff/speed",               takeoff_speed,        0.3);
        nh.param("takeoff/motor_warmup_time",   motor_warmup_time,    2.0);
        nh.param("takeoff/motor_warmup_height", motor_warmup_height,  0.05);
        nh.param("takeoff/decel_distance",      decel_distance,       0.3);
        nh.param("takeoff/reach_threshold",     reach_threshold,      0.05);
        nh.param("takeoff/min_speed",           min_speed,            0.05);
        nh.param("takeoff/delay_trigger_time",  delay_trigger_time,   2.0);

        nh.param("landing/speed",               landing_speed,        0.3);
        nh.param("landing/slow_height",         slow_height,          0.3);
        nh.param("landing/slow_factor",         slow_factor,          0.5);
        nh.param("landing/land_pos_deviation",  land_pos_deviation,  -0.5);
        nh.param("landing/land_vel_threshold",  land_vel_threshold,   0.1);
        nh.param("landing/land_hold_time",      land_hold_time,       3.0);

        nh.param("timeout/cmd",                 cmd_timeout,          0.5);
        nh.param("timeout/odom",                odom_timeout,         0.5);

        nh.param("reach_detect/pos_threshold",  reach_pos_threshold,  0.3);
        nh.param("reach_detect/vel_threshold",  reach_vel_threshold,  0.1);
        nh.param("reach_detect/hold_time",      reach_hold_time,      1.0);

        nh.param("override/enable",             override_enable,      true);
        nh.param("override/cmd_timeout",        override_cmd_timeout, 0.5);

        nh.param("debug/enable",                debug_enable,         false);
    }
};

// ─────────────────────────────────────────────────────────────
//  Bridge Node class
// ─────────────────────────────────────────────────────────────
class EgoBridgeNode {
public:
    EgoBridgeNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh)
    {
        params_.load(pnh_);

        // ---- Subscribers ----
        sub_mavros_state_    = nh_.subscribe("/mavros/state",          10, &EgoBridgeNode::mavrosStateCb, this);
        sub_mavros_ext_      = nh_.subscribe("/mavros/extended_state", 10, &EgoBridgeNode::extendedStateCb, this);
        sub_odom_            = pnh_.subscribe("odom",            10, &EgoBridgeNode::odomCb, this);
        sub_cmd_             = pnh_.subscribe("cmd",             10, &EgoBridgeNode::cmdCb, this);
        sub_takeoff_land_    = pnh_.subscribe("takeoff_land",    10, &EgoBridgeNode::takeoffLandCb, this);
        sub_target_point_    = pnh_.subscribe("target_point",    10, &EgoBridgeNode::targetPointCb, this);
        sub_set_ctrl_mode_   = pnh_.subscribe("set_control_mode",10, &EgoBridgeNode::setControlModeCb, this);
        sub_override_cmd_    = pnh_.subscribe("override_cmd",    10, &EgoBridgeNode::overrideCmdCb, this);
        sub_emergency_stop_  = pnh_.subscribe("emergency_stop",  10, &EgoBridgeNode::emergencyStopCb, this);

        // ---- Publishers ----
        pub_setpoint_    = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        pub_trigger_     = nh_.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);
        pub_reach_       = pnh_.advertise<std_msgs::UInt8>("reach_status", 10);
        pub_flight_state_= pnh_.advertise<std_msgs::String>("flight_state", 10);
        pub_ctrl_mode_   = pnh_.advertise<std_msgs::UInt8>("control_mode", 10);
        if (params_.debug_enable)
            pub_debug_  = pnh_.advertise<std_msgs::Float64MultiArray>("debug", 10);

        // ---- Service clients ----
        srv_set_mode_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        srv_arming_   = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

        // Timer
        timer_ = nh_.createTimer(ros::Duration(1.0 / params_.ctrl_freq),
                                 &EgoBridgeNode::mainLoop, this);

        ROS_INFO("[ego_bridge] Initialized. ctrl_freq=%.1f Hz", params_.ctrl_freq);
    }

private:
    // ── ROS handles ──
    ros::NodeHandle nh_, pnh_;
    ros::Timer timer_;

    // Subscribers
    ros::Subscriber sub_mavros_state_, sub_mavros_ext_, sub_odom_, sub_cmd_;
    ros::Subscriber sub_takeoff_land_, sub_target_point_;
    ros::Subscriber sub_set_ctrl_mode_, sub_override_cmd_, sub_emergency_stop_;

    // Publishers
    ros::Publisher pub_setpoint_, pub_trigger_;
    ros::Publisher pub_reach_, pub_flight_state_, pub_ctrl_mode_, pub_debug_;

    // Service clients
    ros::ServiceClient srv_set_mode_, srv_arming_;

    // Parameters
    Params params_;

    // ── FSM state ──
    State state_ = State::IDLE;
    uint8_t control_mode_ = 0;   // 0 = EGO, 1 = OVERRIDE

    // ── Data caches ──
    mavros_msgs::State          mavros_state_;
    mavros_msgs::ExtendedState  extended_state_;
    ros::Time                   mavros_state_stamp_ = ros::Time(0);

    Eigen::Vector3d odom_pos_   = Eigen::Vector3d::Zero();
    Eigen::Vector3d odom_vel_   = Eigen::Vector3d::Zero();
    double          odom_yaw_   = 0.0;
    ros::Time       odom_stamp_ = ros::Time(0);
    bool            odom_received_ = false;

    quadrotor_msgs::PositionCommand latest_cmd_;
    ros::Time                       cmd_stamp_  = ros::Time(0);
    bool                            cmd_received_ = false;

    quadrotor_msgs::PositionCommand override_cmd_;
    ros::Time                       override_cmd_stamp_ = ros::Time(0);

    Eigen::Vector3d target_point_ = Eigen::Vector3d::Zero();
    bool            target_set_   = false;

    // ── PRE_OFFBOARD variables ──
    int    pre_send_counter_    = 0;
    bool   offboard_requested_  = false;
    ros::Time offboard_first_try_time_;
    ros::Time last_offboard_try_time_;

    // ── TAKEOFF variables ──
    Eigen::Vector3d takeoff_start_pos_;
    double          takeoff_target_z_   = 0.0;
    ros::Time       takeoff_start_time_;
    bool            warmup_done_        = false;
    bool            trigger_sent_       = false;
    ros::Time       hover_enter_time_;

    // ── HOVER variables ──
    Eigen::Vector3d hover_pos_;
    bool            wait_new_traj_  = false;
    uint32_t        old_traj_id_    = 0;

    // ── LANDING variables ──
    Eigen::Vector3d landing_start_pos_;
    double          landing_start_z_    = 0.0;
    ros::Time       landing_start_time_;
    ros::Time       land_detect_start_;
    bool            land_detect_active_ = false;

    // ── Reach detection ──
    ros::Time reach_detect_start_;
    bool      reach_detecting_   = false;
    uint8_t   last_reach_status_ = 0;

    // ── Emergency ──
    bool emergency_stop_ = false;

    // ────────────────────────────────────────────
    //  Callbacks
    // ────────────────────────────────────────────
    void mavrosStateCb(const mavros_msgs::State::ConstPtr& msg) {
        mavros_state_ = *msg;
        mavros_state_stamp_ = ros::Time::now();
    }

    void extendedStateCb(const mavros_msgs::ExtendedState::ConstPtr& msg) {
        extended_state_ = *msg;
    }

    void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
        odom_pos_ << msg->pose.pose.position.x,
                     msg->pose.pose.position.y,
                     msg->pose.pose.position.z;
        odom_vel_ << msg->twist.twist.linear.x,
                     msg->twist.twist.linear.y,
                     msg->twist.twist.linear.z;

        // Extract yaw from quaternion
        double q_x = msg->pose.pose.orientation.x;
        double q_y = msg->pose.pose.orientation.y;
        double q_z = msg->pose.pose.orientation.z;
        double q_w = msg->pose.pose.orientation.w;
        odom_yaw_ = std::atan2(2.0 * (q_w * q_z + q_x * q_y),
                               1.0 - 2.0 * (q_y * q_y + q_z * q_z));

        odom_stamp_    = msg->header.stamp;
        odom_received_ = true;
    }

    void cmdCb(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
        latest_cmd_   = *msg;
        cmd_stamp_    = ros::Time::now();
        cmd_received_ = true;
    }

    void takeoffLandCb(const quadrotor_msgs::TakeoffLand::ConstPtr& msg) {
        if (msg->takeoff_land_cmd == quadrotor_msgs::TakeoffLand::TAKEOFF) {
            if (state_ == State::IDLE) {
                ROS_INFO("[ego_bridge] TAKEOFF command received");
                enterPreOffboard();
            } else {
                ROS_WARN("[ego_bridge] TAKEOFF ignored: state=%s", stateStr(state_));
            }
        } else if (msg->takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND) {
            if (state_ == State::HOVER || state_ == State::TRACKING) {
                ROS_INFO("[ego_bridge] LAND command received");
                enterLanding();
            } else {
                ROS_WARN("[ego_bridge] LAND ignored: state=%s", stateStr(state_));
            }
        }
    }

    void targetPointCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        target_point_ << msg->pose.position.x,
                         msg->pose.position.y,
                         msg->pose.position.z;
        target_set_ = true;

        // 重置到达检测，确保新目标的 reach_status 从 0 开始
        reach_detecting_ = false;
        if (last_reach_status_ != 0) {
            last_reach_status_ = 0;
            std_msgs::UInt8 rst;
            rst.data = 0;
            pub_reach_.publish(rst);
        }
        ROS_INFO("[ego_bridge] New target point: (%.2f, %.2f, %.2f)",
                 target_point_.x(), target_point_.y(), target_point_.z());
    }

    void setControlModeCb(const std_msgs::UInt8::ConstPtr& msg) {
        if (!params_.override_enable) {
            ROS_WARN("[ego_bridge] Override disabled in config, ignoring set_control_mode");
            return;
        }
        if (msg->data == 1 && control_mode_ == 0 &&
            (state_ == State::TRACKING || state_ == State::HOVER)) {
            // Enter OVERRIDE sub-mode
            control_mode_ = 1;
            ROS_INFO("[ego_bridge] Entering OVERRIDE mode (state=%s)", stateStr(state_));
        } else if (msg->data == 0 && control_mode_ == 1) {
            // Exit OVERRIDE → go to HOVER with wait_new_traj
            control_mode_ = 0;
            ROS_INFO("[ego_bridge] Exiting OVERRIDE → HOVER (wait_new_traj)");
            enterHoverFromOverride();
        }
    }

    void overrideCmdCb(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
        override_cmd_ = *msg;
        override_cmd_stamp_ = ros::Time::now();
    }

    void emergencyStopCb(const std_msgs::Empty::ConstPtr& /*msg*/) {
        ROS_ERROR("[ego_bridge] EMERGENCY STOP!");
        emergency_stop_ = true;
        state_ = State::IDLE;
        control_mode_ = 0;
    }

    // ────────────────────────────────────────────
    //  State transitions
    // ────────────────────────────────────────────
    void enterPreOffboard() {
        state_ = State::PRE_OFFBOARD;
        pre_send_counter_   = 0;
        offboard_requested_ = false;
        offboard_first_try_time_ = ros::Time(0);
        ROS_INFO("[ego_bridge] → PRE_OFFBOARD");
    }

    void enterTakeoff() {
        state_ = State::TAKEOFF;
        takeoff_start_pos_  = odom_pos_;
        takeoff_target_z_   = params_.takeoff_height;
        if (takeoff_target_z_ < odom_pos_.z()) {
            takeoff_target_z_ = odom_pos_.z();
        }
        takeoff_start_time_ = ros::Time::now();
        warmup_done_        = false;
        trigger_sent_       = false;
        ROS_INFO("[ego_bridge] → TAKEOFF (target_z=%.2f)", takeoff_target_z_);
    }

    void enterHover() {
        state_ = State::HOVER;
        hover_pos_ = odom_pos_;
        hover_enter_time_ = ros::Time::now();
        wait_new_traj_ = false;
        control_mode_  = 0;
        ROS_INFO("[ego_bridge] → HOVER at (%.2f, %.2f, %.2f)",
                 hover_pos_.x(), hover_pos_.y(), hover_pos_.z());
    }

    void enterHoverFromOverride() {
        state_ = State::HOVER;
        hover_pos_ = odom_pos_;
        hover_enter_time_ = ros::Time::now();
        wait_new_traj_ = true;
        old_traj_id_   = latest_cmd_.trajectory_id;
        control_mode_  = 0;
        ROS_INFO("[ego_bridge] → HOVER (wait_new_traj, old_id=%u) at (%.2f, %.2f, %.2f)",
                 old_traj_id_, hover_pos_.x(), hover_pos_.y(), hover_pos_.z());
    }

    void enterTracking() {
        state_ = State::TRACKING;
        control_mode_ = 0;
        reach_detecting_ = false;
        last_reach_status_ = 0;
        ROS_INFO("[ego_bridge] → TRACKING");
    }

    void enterLanding() {
        state_ = State::LANDING;
        landing_start_pos_ = odom_pos_;
        landing_start_z_   = odom_pos_.z();
        landing_start_time_ = ros::Time::now();
        land_detect_active_ = false;
        control_mode_ = 0;
        ROS_INFO("[ego_bridge] → LANDING from z=%.2f", landing_start_z_);
    }

    void enterIdle() {
        state_ = State::IDLE;
        control_mode_ = 0;
        emergency_stop_ = false;
        ROS_INFO("[ego_bridge] → IDLE");
    }

    // ────────────────────────────────────────────
    //  Setpoint builders
    // ────────────────────────────────────────────
    mavros_msgs::PositionTarget buildPositionTarget(
        const Eigen::Vector3d& pos,
        const Eigen::Vector3d& vel = Eigen::Vector3d::Zero(),
        const Eigen::Vector3d& acc = Eigen::Vector3d::Zero(),
        double yaw = NAN,
        uint16_t ignore_mask = 0)
    {
        mavros_msgs::PositionTarget pt;
        pt.header.stamp    = ros::Time::now();
        pt.header.frame_id = "map";
        pt.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

        pt.position.x = pos.x();
        pt.position.y = pos.y();
        pt.position.z = pos.z();
        pt.velocity.x = vel.x();
        pt.velocity.y = vel.y();
        pt.velocity.z = vel.z();
        pt.acceleration_or_force.x = acc.x();
        pt.acceleration_or_force.y = acc.y();
        pt.acceleration_or_force.z = acc.z();
        pt.yaw = std::isnan(yaw) ? static_cast<float>(odom_yaw_) : static_cast<float>(yaw);
        pt.yaw_rate = 0.0f;

        // Default: ignore nothing except yaw_rate
        pt.type_mask = ignore_mask | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        return pt;
    }

    /** Position-only setpoint (ignore vel + acc). */
    mavros_msgs::PositionTarget posOnlySetpoint(const Eigen::Vector3d& pos, double yaw = NAN) {
        uint16_t mask =
            mavros_msgs::PositionTarget::IGNORE_VX  |
            mavros_msgs::PositionTarget::IGNORE_VY  |
            mavros_msgs::PositionTarget::IGNORE_VZ  |
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ;
        return buildPositionTarget(pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), yaw, mask);
    }

    /** Position + velocity setpoint (ignore acc). */
    mavros_msgs::PositionTarget posVelSetpoint(const Eigen::Vector3d& pos,
                                                const Eigen::Vector3d& vel,
                                                double yaw = NAN)
    {
        uint16_t mask =
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ;
        return buildPositionTarget(pos, vel, Eigen::Vector3d::Zero(), yaw, mask);
    }

    /** Full tracking: position + velocity + acceleration + yaw. */
    mavros_msgs::PositionTarget fullTrackingSetpoint(
        const quadrotor_msgs::PositionCommand& cmd)
    {
        Eigen::Vector3d pos(cmd.position.x, cmd.position.y, cmd.position.z);
        Eigen::Vector3d vel(cmd.velocity.x, cmd.velocity.y, cmd.velocity.z);
        Eigen::Vector3d acc(cmd.acceleration.x, cmd.acceleration.y, cmd.acceleration.z);
        return buildPositionTarget(pos, vel, acc, cmd.yaw, 0);
    }

    // ────────────────────────────────────────────
    //  Service calls
    // ────────────────────────────────────────────
    bool setOffboardMode() {
        mavros_msgs::SetMode req;
        req.request.custom_mode = "OFFBOARD";
        if (srv_set_mode_.call(req) && req.response.mode_sent) {
            ROS_INFO("[ego_bridge] OFFBOARD mode set");
            return true;
        }
        ROS_WARN("[ego_bridge] Failed to set OFFBOARD mode");
        return false;
    }

    bool armDrone() {
        mavros_msgs::CommandBool req;
        req.request.value = true;
        if (srv_arming_.call(req) && req.response.success) {
            ROS_INFO("[ego_bridge] Armed");
            return true;
        }
        ROS_WARN("[ego_bridge] Failed to ARM");
        return false;
    }

    bool disarmDrone() {
        mavros_msgs::CommandBool req;
        req.request.value = false;
        if (srv_arming_.call(req) && req.response.success) {
            ROS_INFO("[ego_bridge] Disarmed");
            return true;
        }
        ROS_WARN("[ego_bridge] Failed to DISARM");
        return false;
    }

    // ────────────────────────────────────────────
    //  Main control loop (timer callback)
    // ────────────────────────────────────────────
    void mainLoop(const ros::TimerEvent& /*e*/) {
        ros::Time now = ros::Time::now();

        // ── Safety: emergency stop ──
        if (emergency_stop_) {
            // Don't send any setpoint → PX4 failsafe
            publishStatus();
            return;
        }

        // ── Safety: odom timeout (in flight states) ──
        if (state_ != State::IDLE && odom_received_) {
            if ((now - odom_stamp_).toSec() > params_.odom_timeout) {
                ROS_ERROR("[ego_bridge] Odom timeout! Stopping setpoints → PX4 failsafe");
                // Don't send setpoint, PX4 will handle failsafe
                publishStatus();
                return;
            }
        }

        // ── Safety: OFFBOARD mode lost (in flight) ──
        if (state_ != State::IDLE && state_ != State::PRE_OFFBOARD) {
            if (mavros_state_stamp_ != ros::Time(0) &&
                mavros_state_.mode != "OFFBOARD")
            {
                ROS_WARN_THROTTLE(2.0, "[ego_bridge] PX4 mode is '%s', not OFFBOARD!",
                                  mavros_state_.mode.c_str());
            }
        }

        // ── State machine ──
        switch (state_) {
            case State::IDLE:
                runIdle(now);
                break;
            case State::PRE_OFFBOARD:
                runPreOffboard(now);
                break;
            case State::TAKEOFF:
                runTakeoff(now);
                break;
            case State::HOVER:
                runHover(now);
                break;
            case State::TRACKING:
                runTracking(now);
                break;
            case State::LANDING:
                runLanding(now);
                break;
        }

        publishStatus();
    }

    // ────────────────────────────────────────────
    //  State handlers
    // ────────────────────────────────────────────

    // ── IDLE ──
    void runIdle(const ros::Time& /*now*/) {
        // Do nothing, wait for TAKEOFF command via takeoffLandCb
    }

    // ── PRE_OFFBOARD ──
    void runPreOffboard(const ros::Time& now) {
        if (!odom_received_) {
            ROS_WARN_THROTTLE(1.0, "[ego_bridge] PRE_OFFBOARD: Waiting for odometry...");
            return;
        }

        // Send current position as setpoint
        pub_setpoint_.publish(posOnlySetpoint(odom_pos_));
        pre_send_counter_++;

        // Phase 1: Pre-send enough frames
        if (!offboard_requested_ && pre_send_counter_ < params_.pre_send_count) {
            return;  // Keep sending, not ready to switch yet
        }

        // Phase 2: Try to switch to OFFBOARD
        if (!offboard_requested_) {
            offboard_first_try_time_ = now;
            last_offboard_try_time_  = ros::Time(0);
            offboard_requested_ = true;
        }

        // Check if already in OFFBOARD
        if (mavros_state_.mode == "OFFBOARD") {
            // Arm if needed
            if (params_.enable_auto_arm && !mavros_state_.armed) {
                armDrone();
                return;  // Wait one cycle for ARM confirmation
            }
            if (mavros_state_.armed) {
                enterTakeoff();
                return;
            }
            // Not armed and auto_arm disabled — just wait
            ROS_WARN_THROTTLE(1.0, "[ego_bridge] In OFFBOARD but not armed. Waiting for manual arm.");
            return;
        }

        // Check timeout
        if ((now - offboard_first_try_time_).toSec() > params_.offboard_timeout) {
            ROS_ERROR("[ego_bridge] OFFBOARD switch timeout (%.1fs). Returning to IDLE.",
                      params_.offboard_timeout);
            enterIdle();
            return;
        }

        // Try every ~1 second
        if ((now - last_offboard_try_time_).toSec() >= 1.0) {
            last_offboard_try_time_ = now;
            setOffboardMode();
        }
    }

    // ── TAKEOFF ──
    void runTakeoff(const ros::Time& now) {
        double dt = (now - takeoff_start_time_).toSec();

        // Phase 1: Motor warmup
        if (!warmup_done_) {
            if (dt < params_.motor_warmup_time) {
                Eigen::Vector3d target = takeoff_start_pos_;
                target.z() += params_.motor_warmup_height;
                pub_setpoint_.publish(posOnlySetpoint(target));
                return;
            }
            warmup_done_ = true;
            takeoff_start_time_ = now;  // Reset timer for climb phase
            dt = 0.0;
        }

        // Phase 2 & 3: Climb with deceleration near target (use odom feedback)
        double remaining = takeoff_target_z_ - odom_pos_.z();

        if (remaining <= params_.reach_threshold) {
            // Reached target height → HOVER
            Eigen::Vector3d target_pos = odom_pos_;
            target_pos.z() = takeoff_target_z_;
            pub_setpoint_.publish(posOnlySetpoint(target_pos));
            enterHover();
            return;
        }

        // Deceleration zone
        double speed = params_.takeoff_speed;
        if (remaining < params_.decel_distance && params_.decel_distance > 1e-4) {
            speed *= remaining / params_.decel_distance;
            speed = std::max(speed, params_.min_speed);
        }

        Eigen::Vector3d target_pos = takeoff_start_pos_;
        target_pos.z() = std::min(takeoff_start_pos_.z() + params_.takeoff_speed * dt,
                                  takeoff_target_z_);
        Eigen::Vector3d vel(0.0, 0.0, speed);
        pub_setpoint_.publish(posVelSetpoint(target_pos, vel));
    }

    // ── HOVER ──
    void runHover(const ros::Time& now) {
        // OVERRIDE sub-mode（在 HOVER 中也支持接管）
        if (control_mode_ == 1) {
            runOverride(now);
            return;
        }

        pub_setpoint_.publish(posOnlySetpoint(hover_pos_));

        // Send trigger after delay (only once after entering HOVER from TAKEOFF)
        if (!trigger_sent_) {
            if ((now - hover_enter_time_).toSec() >= params_.delay_trigger_time) {
                trigger_sent_ = true;
                geometry_msgs::PoseStamped trig;
                trig.header.stamp = now;
                trig.header.frame_id = "map";
                trig.pose.position.x = odom_pos_.x();
                trig.pose.position.y = odom_pos_.y();
                trig.pose.position.z = odom_pos_.z();
                trig.pose.orientation.w = 1.0;
                pub_trigger_.publish(trig);
                ROS_INFO("[ego_bridge] Published /traj_start_trigger");
            }
        }

        // Transition to TRACKING on EGO cmd
        if (cmd_received_ && (now - cmd_stamp_).toSec() < params_.cmd_timeout) {
            Eigen::Vector3d cmd_vel(latest_cmd_.velocity.x,
                                    latest_cmd_.velocity.y,
                                    latest_cmd_.velocity.z);
            bool has_velocity = cmd_vel.norm() > 0.01;

            if (wait_new_traj_) {
                // Must wait for trajectory_id change
                if (latest_cmd_.trajectory_id != old_traj_id_ && has_velocity) {
                    ROS_INFO("[ego_bridge] New trajectory_id=%u (old=%u), entering TRACKING",
                             latest_cmd_.trajectory_id, old_traj_id_);
                    enterTracking();
                }
            } else {
                if (has_velocity) {
                    enterTracking();
                }
            }
        }
    }

    // ── TRACKING ──
    void runTracking(const ros::Time& now) {
        // OVERRIDE sub-mode
        if (control_mode_ == 1) {
            runOverride(now);
            return;
        }

        // Normal EGO tracking
        if (!cmd_received_ || (now - cmd_stamp_).toSec() > params_.cmd_timeout) {
            // Cmd timeout → lock position → HOVER
            ROS_WARN("[ego_bridge] EGO cmd timeout → HOVER");
            enterHover();
            trigger_sent_ = true;  // Don't re-trigger after cmd timeout
            return;
        }

        // Publish full tracking setpoint
        pub_setpoint_.publish(fullTrackingSetpoint(latest_cmd_));

        // Reach detection
        runReachDetection(now);
    }

    void runOverride(const ros::Time& now) {
        if ((now - override_cmd_stamp_).toSec() < params_.override_cmd_timeout) {
            // Forward override command
            pub_setpoint_.publish(fullTrackingSetpoint(override_cmd_));
        } else {
            // Override cmd timeout → hover at current position
            pub_setpoint_.publish(posOnlySetpoint(odom_pos_));
        }
    }

    // ── LANDING ──
    void runLanding(const ros::Time& now) {
        double dt = (now - landing_start_time_).toSec();

        // Compute target z
        double target_z = landing_start_z_ - params_.landing_speed * dt;
        double speed = params_.landing_speed;

        // Slow down near ground
        if (odom_pos_.z() < params_.slow_height) {
            speed *= params_.slow_factor;
        }

        Eigen::Vector3d target_pos = landing_start_pos_;
        target_pos.z() = target_z;
        Eigen::Vector3d vel(0.0, 0.0, -speed);

        pub_setpoint_.publish(posVelSetpoint(target_pos, vel));

        // Landing detection — Condition B: PX4 says on ground
        if (extended_state_.landed_state ==
            mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
        {
            ROS_INFO("[ego_bridge] PX4 reports ON_GROUND");
            completeLanding();
            return;
        }

        // Landing detection — Condition A: position & velocity check
        double pos_error_z = target_pos.z() - odom_pos_.z();
        double vel_norm = odom_vel_.norm();

        if (pos_error_z < params_.land_pos_deviation &&
            vel_norm < params_.land_vel_threshold)
        {
            if (!land_detect_active_) {
                land_detect_active_ = true;
                land_detect_start_  = now;
            } else if ((now - land_detect_start_).toSec() >= params_.land_hold_time) {
                ROS_INFO("[ego_bridge] Landing detected (pos+vel hold)");
                completeLanding();
                return;
            }
        } else {
            land_detect_active_ = false;
        }
    }

    void completeLanding() {
        disarmDrone();

        // Exit OFFBOARD mode (switch to POSCTL or AUTO.LAND)
        mavros_msgs::SetMode req;
        req.request.custom_mode = "AUTO.LAND";
        srv_set_mode_.call(req);

        enterIdle();
    }

    // ────────────────────────────────────────────
    //  Reach detection
    // ────────────────────────────────────────────
    void runReachDetection(const ros::Time& now) {
        Eigen::Vector3d cmd_vel(latest_cmd_.velocity.x,
                                latest_cmd_.velocity.y,
                                latest_cmd_.velocity.z);

        bool vel_low = cmd_vel.norm() < params_.reach_vel_threshold;

        bool pos_close = true;
        if (target_set_) {
            Eigen::Vector3d diff = odom_pos_ - target_point_;
            pos_close = diff.norm() < params_.reach_pos_threshold;
        }

        if (vel_low && pos_close) {
            if (!reach_detecting_) {
                reach_detecting_ = true;
                reach_detect_start_ = now;
            } else if ((now - reach_detect_start_).toSec() >= params_.reach_hold_time) {
                if (last_reach_status_ != 1) {
                    last_reach_status_ = 1;
                    std_msgs::UInt8 msg;
                    msg.data = 1;
                    pub_reach_.publish(msg);
                    ROS_INFO("[ego_bridge] Reached target!");
                }
            }
        } else {
            reach_detecting_ = false;
            if (last_reach_status_ != 0) {
                last_reach_status_ = 0;
                std_msgs::UInt8 msg;
                msg.data = 0;
                pub_reach_.publish(msg);
            }
        }
    }

    // ────────────────────────────────────────────
    //  Status publishing
    // ────────────────────────────────────────────
    void publishStatus() {
        // Flight state
        std_msgs::String state_msg;
        state_msg.data = stateStr(state_);
        pub_flight_state_.publish(state_msg);

        // Control mode
        std_msgs::UInt8 mode_msg;
        mode_msg.data = control_mode_;
        pub_ctrl_mode_.publish(mode_msg);

        // Debug
        if (params_.debug_enable) {
            std_msgs::Float64MultiArray dbg;
            dbg.data.resize(6);
            dbg.data[0] = static_cast<double>(state_);
            dbg.data[1] = target_set_
                ? (odom_pos_ - target_point_).norm() : -1.0;
            dbg.data[2] = cmd_received_
                ? (ros::Time::now() - cmd_stamp_).toSec() : -1.0;
            dbg.data[3] = static_cast<double>(control_mode_);
            dbg.data[4] = odom_pos_.z();
            dbg.data[5] = static_cast<double>(last_reach_status_);
            pub_debug_.publish(dbg);
        }
    }
};

// ─────────────────────────────────────────────────────────────
//  Main
// ─────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    ros::init(argc, argv, "ego_bridge_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    EgoBridgeNode node(nh, pnh);
    ros::spin();
    return 0;
}
