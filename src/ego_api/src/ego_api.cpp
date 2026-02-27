#include "ego_api/ego_api.h"

#include <cmath>

// ─────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────
EgoApi::EgoApi(ros::NodeHandle& nh, const std::string& bridge_ns)
    : nh_(nh), bridge_ns_(bridge_ns)
{
    // ── Subscribers ──
    sub_flight_state_    = nh_.subscribe(bridge_ns_ + "/flight_state",    10, &EgoApi::flightStateCb, this);
    sub_reach_status_    = nh_.subscribe(bridge_ns_ + "/reach_status",    10, &EgoApi::reachStatusCb, this);
    sub_control_mode_    = nh_.subscribe(bridge_ns_ + "/control_mode",    10, &EgoApi::controlModeCb, this);
    sub_odom_            = nh_.subscribe("/mavros/local_position/odom",   10, &EgoApi::odomCb, this);
    sub_override_trigger_= nh_.subscribe("/ego_api/override_trigger",     10, &EgoApi::overrideTriggerCb, this);

    // ── Publishers ──
    pub_takeoff_land_    = nh_.advertise<quadrotor_msgs::TakeoffLand>(bridge_ns_ + "/takeoff_land", 1);
    pub_goal_            = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    pub_target_point_    = nh_.advertise<geometry_msgs::PoseStamped>(bridge_ns_ + "/target_point", 1);
    pub_set_ctrl_mode_   = nh_.advertise<std_msgs::UInt8>(bridge_ns_ + "/set_control_mode", 1);
    pub_override_cmd_    = nh_.advertise<quadrotor_msgs::PositionCommand>(bridge_ns_ + "/override_cmd", 10);
    pub_emergency_stop_  = nh_.advertise<std_msgs::Empty>(bridge_ns_ + "/emergency_stop", 1);
    pub_override_trigger_= nh_.advertise<std_msgs::Int32>("/ego_api/override_trigger", 1);

    // Wait a moment for connections
    ros::Duration(0.5).sleep();

    ROS_INFO("[EgoApi] Initialized. bridge_ns=%s", bridge_ns_.c_str());
}

// ─────────────────────────────────────────────────────────────
//  Callbacks
// ─────────────────────────────────────────────────────────────
void EgoApi::flightStateCb(const std_msgs::String::ConstPtr& msg) {
    flight_state_ = msg->data;
    connected_ = true;
}

void EgoApi::reachStatusCb(const std_msgs::UInt8::ConstPtr& msg) {
    reach_status_ = msg->data;
}

void EgoApi::controlModeCb(const std_msgs::UInt8::ConstPtr& msg) {
    control_mode_ = msg->data;
}

void EgoApi::odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
    odom_pos_ << msg->pose.pose.position.x,
                 msg->pose.pose.position.y,
                 msg->pose.pose.position.z;

    double q_x = msg->pose.pose.orientation.x;
    double q_y = msg->pose.pose.orientation.y;
    double q_z = msg->pose.pose.orientation.z;
    double q_w = msg->pose.pose.orientation.w;
    odom_yaw_ = std::atan2(2.0 * (q_w * q_z + q_x * q_y),
                           1.0 - 2.0 * (q_y * q_y + q_z * q_z));
}

void EgoApi::overrideTriggerCb(const std_msgs::Int32::ConstPtr& msg) {
    pending_task_id_ = msg->data;
    trigger_received_ = true;
}

// ─────────────────────────────────────────────────────────────
//  Status queries
// ─────────────────────────────────────────────────────────────
std::string     EgoApi::getFlightState()  const { return flight_state_; }
uint8_t         EgoApi::getReachStatus()  const { return reach_status_; }
uint8_t         EgoApi::getControlMode()  const { return control_mode_; }
Eigen::Vector3d EgoApi::getOdomPosition() const { return odom_pos_; }
double          EgoApi::getOdomYaw()      const { return odom_yaw_; }
bool            EgoApi::isConnected()     const { return connected_; }

// ─────────────────────────────────────────────────────────────
//  Internal helpers
// ─────────────────────────────────────────────────────────────
void EgoApi::publishGoal(double x, double y, double z, double yaw) {
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "world";
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = z;

    // yaw → quaternion (only around z-axis)
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = std::sin(yaw / 2.0);
    goal.pose.orientation.w = std::cos(yaw / 2.0);

    // 发给 ego_planner（规划路径）
    pub_goal_.publish(goal);

    // 发给 ego_bridge（到达检测）
    pub_target_point_.publish(goal);

    ROS_INFO("[EgoApi] Goal published: (%.2f, %.2f, %.2f) yaw=%.2f", x, y, z, yaw);
}

quadrotor_msgs::PositionCommand EgoApi::buildPositionCmd(
    double x, double y, double z, double yaw,
    double vx, double vy, double vz)
{
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "world";
    cmd.position.x = x;
    cmd.position.y = y;
    cmd.position.z = z;
    cmd.velocity.x = vx;
    cmd.velocity.y = vy;
    cmd.velocity.z = vz;
    cmd.acceleration.x = 0.0;
    cmd.acceleration.y = 0.0;
    cmd.acceleration.z = 0.0;
    cmd.yaw = yaw;
    cmd.yaw_dot = 0.0;
    cmd.trajectory_id = 0;
    cmd.trajectory_flag = 0;
    return cmd;
}

// ─────────────────────────────────────────────────────────────
//  Blocking flight control
// ─────────────────────────────────────────────────────────────
bool EgoApi::takeoff(double timeout) {
    ROS_INFO("[EgoApi] Sending TAKEOFF command...");

    quadrotor_msgs::TakeoffLand msg;
    msg.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::TAKEOFF;
    pub_takeoff_land_.publish(msg);

    ros::Rate rate(10);
    ros::Time start = ros::Time::now();

    while (ros::ok()) {
        ros::spinOnce();
        if (flight_state_ == "HOVER") {
            ROS_INFO("[EgoApi] Takeoff complete: HOVER");
            return true;
        }
        if ((ros::Time::now() - start).toSec() > timeout) {
            ROS_WARN("[EgoApi] Takeoff timeout (%.1fs), state=%s", timeout, flight_state_.c_str());
            return false;
        }
        rate.sleep();
    }
    return false;
}

bool EgoApi::land(double timeout) {
    ROS_INFO("[EgoApi] Sending LAND command...");

    quadrotor_msgs::TakeoffLand msg;
    msg.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::LAND;
    pub_takeoff_land_.publish(msg);

    ros::Rate rate(10);
    ros::Time start = ros::Time::now();

    while (ros::ok()) {
        ros::spinOnce();
        if (flight_state_ == "IDLE") {
            ROS_INFO("[EgoApi] Landing complete: IDLE");
            return true;
        }
        if ((ros::Time::now() - start).toSec() > timeout) {
            ROS_WARN("[EgoApi] Land timeout (%.1fs), state=%s", timeout, flight_state_.c_str());
            return false;
        }
        rate.sleep();
    }
    return false;
}

bool EgoApi::sendGoal(double x, double y, double z, double timeout) {
    return sendGoalWithYaw(x, y, z, odom_yaw_, timeout);
}

bool EgoApi::sendGoalWithYaw(double x, double y, double z, double yaw, double timeout) {
    ROS_INFO("[EgoApi] sendGoalWithYaw(%.2f, %.2f, %.2f, yaw=%.2f, timeout=%.1f)",
             x, y, z, yaw, timeout);

    publishGoal(x, y, z, yaw);

    ros::Rate rate(10);
    ros::Time start = ros::Time::now();

    while (ros::ok()) {
        ros::spinOnce();

        if (reach_status_ == 1) {
            ROS_INFO("[EgoApi] Goal reached!");
            return true;
        }

        double elapsed = (ros::Time::now() - start).toSec();
        if (elapsed > timeout) {
            ROS_WARN("[EgoApi] Goal timeout (%.1fs), skipping.", timeout);
            return false;
        }

        rate.sleep();
    }
    return false;
}

// ─────────────────────────────────────────────────────────────
//  Override control
// ─────────────────────────────────────────────────────────────
bool EgoApi::enableOverride() {
    ROS_INFO("[EgoApi] Requesting OVERRIDE mode...");

    std_msgs::UInt8 msg;
    msg.data = 1;

    ros::Rate rate(20);
    ros::Time start = ros::Time::now();
    double confirm_timeout = 2.0;

    while (ros::ok()) {
        ros::spinOnce();
        pub_set_ctrl_mode_.publish(msg);

        if (control_mode_ == 1) {
            ROS_INFO("[EgoApi] OVERRIDE enabled.");
            return true;
        }
        if ((ros::Time::now() - start).toSec() > confirm_timeout) {
            ROS_ERROR("[EgoApi] enableOverride timeout! state=%s, control_mode=%d",
                      flight_state_.c_str(), control_mode_);
            return false;
        }
        rate.sleep();
    }
    return false;
}

bool EgoApi::disableOverride() {
    ROS_INFO("[EgoApi] Exiting OVERRIDE mode...");

    std_msgs::UInt8 msg;
    msg.data = 0;

    ros::Rate rate(20);
    ros::Time start = ros::Time::now();
    double confirm_timeout = 2.0;

    while (ros::ok()) {
        ros::spinOnce();
        pub_set_ctrl_mode_.publish(msg);

        if (control_mode_ == 0) {
            ROS_INFO("[EgoApi] OVERRIDE disabled, control returned.");
            return true;
        }
        if ((ros::Time::now() - start).toSec() > confirm_timeout) {
            ROS_ERROR("[EgoApi] disableOverride timeout!");
            return false;
        }
        rate.sleep();
    }
    return false;
}

void EgoApi::sendOverrideCmd(const quadrotor_msgs::PositionCommand& cmd) {
    pub_override_cmd_.publish(cmd);
}

void EgoApi::holdPosition() {
    auto cmd = buildPositionCmd(odom_pos_.x(), odom_pos_.y(), odom_pos_.z(), odom_yaw_);
    pub_override_cmd_.publish(cmd);
}

bool EgoApi::moveToOverride(double x, double y, double z, double yaw,
                             double pos_threshold, double timeout)
{
    ROS_INFO("[EgoApi] moveToOverride(%.2f, %.2f, %.2f, yaw=%.2f, thresh=%.2f)",
             x, y, z, yaw, pos_threshold);

    ros::Rate rate(50);
    ros::Time start = ros::Time::now();

    while (ros::ok()) {
        ros::spinOnce();

        // 持续发送 override cmd
        auto cmd = buildPositionCmd(x, y, z, yaw);
        pub_override_cmd_.publish(cmd);

        // 检查是否到达
        Eigen::Vector3d target(x, y, z);
        double dist = (odom_pos_ - target).norm();
        if (dist < pos_threshold) {
            ROS_INFO("[EgoApi] moveToOverride reached (dist=%.3f)", dist);
            return true;
        }

        if ((ros::Time::now() - start).toSec() > timeout) {
            ROS_WARN("[EgoApi] moveToOverride timeout (dist=%.3f)", dist);
            return false;
        }

        rate.sleep();
    }
    return false;
}

// ─────────────────────────────────────────────────────────────
//  Override task trigger
// ─────────────────────────────────────────────────────────────
void EgoApi::triggerOverrideTask(int task_id) {
    ROS_INFO("[EgoApi] Triggering override task %d", task_id);
    std_msgs::Int32 msg;
    msg.data = task_id;
    pub_override_trigger_.publish(msg);
}

int EgoApi::waitForOverrideTrigger(double timeout) {
    ROS_INFO("[EgoApi] Waiting for override trigger...");

    trigger_received_ = false;
    pending_task_id_ = -1;

    ros::Rate rate(10);
    ros::Time start = ros::Time::now();

    while (ros::ok()) {
        ros::spinOnce();

        if (trigger_received_) {
            int id = pending_task_id_;
            trigger_received_ = false;
            pending_task_id_ = -1;
            ROS_INFO("[EgoApi] Override trigger received: task_id=%d", id);
            return id;
        }

        if (timeout > 0 && (ros::Time::now() - start).toSec() > timeout) {
            ROS_WARN("[EgoApi] waitForOverrideTrigger timeout");
            return -1;
        }

        rate.sleep();
    }
    return -1;
}

bool EgoApi::waitOverrideComplete(double timeout) {
    ROS_INFO("[EgoApi] Waiting for override to complete...");

    ros::Rate rate(10);
    ros::Time start = ros::Time::now();

    while (ros::ok()) {
        ros::spinOnce();

        if (control_mode_ == 0) {
            ROS_INFO("[EgoApi] Override complete (control_mode=0).");
            return true;
        }

        if ((ros::Time::now() - start).toSec() > timeout) {
            ROS_WARN("[EgoApi] waitOverrideComplete timeout (%.1fs). Force disabling...", timeout);
            // 超时保护：强制归还控制权
            std_msgs::UInt8 msg;
            msg.data = 0;
            pub_set_ctrl_mode_.publish(msg);
            ros::Duration(0.5).sleep();
            ros::spinOnce();
            return false;
        }

        rate.sleep();
    }
    return false;
}

// ─────────────────────────────────────────────────────────────
//  Emergency
// ─────────────────────────────────────────────────────────────
void EgoApi::emergencyStop() {
    ROS_ERROR("[EgoApi] EMERGENCY STOP!");
    std_msgs::Empty msg;
    pub_emergency_stop_.publish(msg);
}
