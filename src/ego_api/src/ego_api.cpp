#include "ego_api/ego_api.h"

#include <cmath>

// ─────────────────────────────────────────────────────────────
//  构造函数：初始化订阅者、发布者，建立与 ego_bridge 的连接
// ─────────────────────────────────────────────────────────────
EgoApi::EgoApi(ros::NodeHandle& nh, const std::string& bridge_ns)
    : nh_(nh), bridge_ns_(bridge_ns)
{
    // ── 订阅者：接收 ego_bridge 的状态反馈 ──
    sub_flight_state_    = nh_.subscribe(bridge_ns_ + "/flight_state",    10, &EgoApi::flightStateCb, this);   // FSM 状态
    sub_reach_status_    = nh_.subscribe(bridge_ns_ + "/reach_status",    10, &EgoApi::reachStatusCb, this);   // 到达状态
    sub_control_mode_    = nh_.subscribe(bridge_ns_ + "/control_mode",    10, &EgoApi::controlModeCb, this);   // EGO/OVERRIDE
    sub_odom_            = nh_.subscribe("/mavros/local_position/odom",   10, &EgoApi::odomCb, this);          // 里程计
    sub_override_trigger_= nh_.subscribe("/ego_api/override_trigger",     10, &EgoApi::overrideTriggerCb, this);// 任务触发

    // ── 发布者：向 ego_bridge / ego_planner / override 节点发送指令 ──
    pub_takeoff_land_    = nh_.advertise<quadrotor_msgs::TakeoffLand>(bridge_ns_ + "/takeoff_land", 1);        // 起飞/降落
    pub_goal_            = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);             // EGO 目标点
    pub_target_point_    = nh_.advertise<geometry_msgs::PoseStamped>(bridge_ns_ + "/target_point", 1);         // 到达检测目标
    pub_set_ctrl_mode_   = nh_.advertise<std_msgs::UInt8>(bridge_ns_ + "/set_control_mode", 1);                // 模式切换
    pub_override_cmd_    = nh_.advertise<quadrotor_msgs::PositionCommand>(bridge_ns_ + "/override_cmd", 10);   // OVERRIDE 命令
    pub_emergency_stop_  = nh_.advertise<std_msgs::Empty>(bridge_ns_ + "/emergency_stop", 1);                  // 紧急停止
    pub_override_trigger_= nh_.advertise<std_msgs::Int32>("/ego_api/override_trigger", 1);                     // 任务触发

    // 等待连接建立
    ros::Duration(0.5).sleep();

    ROS_INFO("[EgoApi] Initialized. bridge_ns=%s", bridge_ns_.c_str());
}

// ─────────────────────────────────────────────────────────────
//  回调函数：更新本地缓存数据
// ─────────────────────────────────────────────────────────────
void EgoApi::flightStateCb(const std_msgs::String::ConstPtr& msg) {
    flight_state_ = msg->data;   // 缓存 FSM 状态字符串
    connected_ = true;           // 首次收到即标记已连接
}

void EgoApi::reachStatusCb(const std_msgs::UInt8::ConstPtr& msg) {
    reach_status_ = msg->data;
}

void EgoApi::controlModeCb(const std_msgs::UInt8::ConstPtr& msg) {
    control_mode_ = msg->data;
}

void EgoApi::odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
    // 提取位置
    odom_pos_ << msg->pose.pose.position.x,
                 msg->pose.pose.position.y,
                 msg->pose.pose.position.z;

    // 从四元数提取 yaw
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
//  状态查询：直接返回缓存值（由回调实时更新）
// ─────────────────────────────────────────────────────────────
std::string     EgoApi::getFlightState()  const { return flight_state_; }
uint8_t         EgoApi::getReachStatus()  const { return reach_status_; }
uint8_t         EgoApi::getControlMode()  const { return control_mode_; }
Eigen::Vector3d EgoApi::getOdomPosition() const { return odom_pos_; }
double          EgoApi::getOdomYaw()      const { return odom_yaw_; }
bool            EgoApi::isConnected()     const { return connected_; }

// ─────────────────────────────────────────────────────────────
//  内部工具：发布目标点 + 构建 PositionCommand
// ─────────────────────────────────────────────────────────────

/// 同时发布目标到 EGO-Planner(规划路径) 和 ego_bridge(到达检测)
void EgoApi::publishGoal(double x, double y, double z, double yaw) {
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "world";
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = z;

    // yaw 转四元数（仅绕 z 轴旋转）
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = std::sin(yaw / 2.0);
    goal.pose.orientation.w = std::cos(yaw / 2.0);

    // 发给 ego_planner（规划新路径）
    pub_goal_.publish(goal);

    // 发给 ego_bridge（用于到达检测）
    pub_target_point_.publish(goal);

    ROS_INFO("[EgoApi] Goal published: (%.2f, %.2f, %.2f) yaw=%.2f", x, y, z, yaw);
}

/// 构建一帧 PositionCommand（仅填 pos/vel/yaw，加速度全零）
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
//  阻塞式飞行控制：发指令 → 轮询状态 → 达成/超时后返回
// ─────────────────────────────────────────────────────────────

/// 起飞：发 TAKEOFF 指令 → 轮询等待 flight_state 变为 HOVER
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

/// 降落：发 LAND 指令 → 轮询等待 flight_state 变为 IDLE
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

/// 发送目标点（自动使用当前 yaw）：内部转调 sendGoalWithYaw
bool EgoApi::sendGoal(double x, double y, double z, double timeout) {
    return sendGoalWithYaw(x, y, z, odom_yaw_, timeout);  // 使用当前航向角
}

/// 发送目标点（指定 yaw）：发布目标 → 轮询 reach_status → 到达/超时
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
//  Override 控制：接管/归还控制权 + 发命令 + 移动
// ─────────────────────────────────────────────────────────────

/// 请求进入 OVERRIDE：持续发 set_control_mode=1 直到 control_mode_ 确认
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

/// 退出 OVERRIDE：持续发 set_control_mode=0 直到 control_mode_ 确认
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

/// 发送一帧 OVERRIDE 控制指令（调用方需自行以 ≥2Hz 持续发送）
void EgoApi::sendOverrideCmd(const quadrotor_msgs::PositionCommand& cmd) {
    pub_override_cmd_.publish(cmd);
}

/// 在当前位置发一帧悬停命令（纯位置指令，速度全零）
void EgoApi::holdPosition() {
    auto cmd = buildPositionCmd(odom_pos_.x(), odom_pos_.y(), odom_pos_.z(), odom_yaw_);
    pub_override_cmd_.publish(cmd);
}

/// OVERRIDE 模式下阻塞移动：以 50Hz 持续发 cmd → 检查位置距离 → 到达/超时
bool EgoApi::moveToOverride(double x, double y, double z, double yaw,
                             double pos_threshold, double timeout)
{
    ROS_INFO("[EgoApi] moveToOverride(%.2f, %.2f, %.2f, yaw=%.2f, thresh=%.2f)",
             x, y, z, yaw, pos_threshold);

    ros::Rate rate(50);  // 50Hz 发送频率
    ros::Time start = ros::Time::now();

    while (ros::ok()) {
        ros::spinOnce();

        // 持续发送目标位置的 override cmd
        auto cmd = buildPositionCmd(x, y, z, yaw);
        pub_override_cmd_.publish(cmd);

        // 检查是否到达（欧氏距离）
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
//  Override 任务触发：主示例 ↔ override 示例之间的协调机制
// ─────────────────────────────────────────────────────────────

/// 主示例调用：发布任务触发信号，override 示例会收到并执行对应任务
void EgoApi::triggerOverrideTask(int task_id) {
    ROS_INFO("[EgoApi] Triggering override task %d", task_id);
    std_msgs::Int32 msg;
    msg.data = task_id;
    pub_override_trigger_.publish(msg);
}

/// override 示例调用：阻塞等待触发信号，返回任务 ID
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

/// 主示例调用：阻塞等待 override 完成（control_mode 回到 0）
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
//  紧急停止：发 empty 消息 → ego_bridge 停发 setpoint → PX4 failsafe
// ─────────────────────────────────────────────────────────────
void EgoApi::emergencyStop() {
    ROS_ERROR("[EgoApi] EMERGENCY STOP!");
    std_msgs::Empty msg;
    pub_emergency_stop_.publish(msg);
}
