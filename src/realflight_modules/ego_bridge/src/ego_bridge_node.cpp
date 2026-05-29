/**
 * @file ego_bridge_node.cpp
 * @brief EGO-Planner → PX4 bridge with 6-state FSM.
 *
 * States: IDLE → PRE_OFFBOARD → TAKEOFF → HOVER ⇄ TRACKING → LANDING → IDLE
 * TRACKING has an OVERRIDE sub-mode for external control handover.
 *
 * 【中文概述】
 * 本节点是 EGO-Planner 到 PX4 的桥接核心，包含 6 状态有限状态机。
 * 控制链路：EGO-Planner 输出轨迹命令(PositionCommand) → 本节点转换 →
 *          发送 PositionTarget 给 PX4 内置位置控制器(通过 MAVROS)。
 *
 * 状态流转：
 *   IDLE(地面待机) → PRE_OFFBOARD(预发setpoint+切OFFBOARD+解锁)
 *   → TAKEOFF(电机预热+匀速爬升) → HOVER(悬停等待EGO命令)
 *   ⇄ TRACKING(转发EGO轨迹 / OVERRIDE子模式转发用户命令)
 *   → LANDING(匀速下降+着陆检测) → IDLE
 *
 * OVERRIDE 子模式：在 TRACKING 或 HOVER 状态下，外部代码可以接管控制权，
 * 直接发送 PositionCommand 控制飞机（用于视觉伺服、精准投递等场景）。
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
//  FSM 状态枚举 — 6 个主状态
// ─────────────────────────────────────────────────────────────
enum class State : uint8_t {
    IDLE          = 0,   // 地面待机，等待起飞命令
    PRE_OFFBOARD  = 1,   // 预发 setpoint → 切 OFFBOARD 模式 → 解锁电机
    TAKEOFF       = 2,   // 电机预热 → 匀速爬升 → 接近目标高度减速
    HOVER         = 3,   // 锁定位置悬停，等待 EGO 命令或降落指令
    TRACKING      = 4,   // 转发 EGO 轨迹命令（或 OVERRIDE 时转发用户命令）
    LANDING       = 5    // 匀速下降 → 近地减速 → 着陆检测 → 锁桨
};

// 将状态枚举转为字符串，用于日志输出和 flight_state 话题发布
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
//  参数结构体 — 所有参数均可在 YAML 配置文件中修改
// ─────────────────────────────────────────────────────────────
struct Params {
    double ctrl_freq;               // 主控制循环频率(Hz)，默认 50

    // ── OFFBOARD 切换参数 ──
    int    pre_send_count;           // 切 OFFBOARD 前需要预发的 setpoint 帧数
    double offboard_timeout;         // 切 OFFBOARD 的超时时间(秒)
    bool   enable_auto_arm;          // 是否自动解锁电机（调试时建议 false）

    // ── 起飞参数 ──
    double takeoff_height;           // 起飞目标高度(m)
    double takeoff_speed;            // 爬升速度(m/s)
    double motor_warmup_time;        // 电机预热时间(秒)，让电机转起来再爬升
    double motor_warmup_height;      // 预热阶段发送的微小高度偏移(m)
    double decel_distance;           // 接近目标高度时开始减速的距离(m)
    double reach_threshold;          // 判定到达起飞高度的阈值(m)
    double min_speed;                // 减速阶段的最小速度(m/s)，防止速度为零卡住
    double delay_trigger_time;       // 进入 HOVER 后延迟多久发 traj_start_trigger(秒)

    // ── 降落参数 ──
    double landing_speed;            // 降落速度(m/s)
    double slow_height;              // 低于此高度时减速(m)
    double slow_factor;              // 近地减速因子(0~1)，速度乘以此值
    double land_pos_deviation;       // 着陆检测：目标z与实际z的偏差阈值(m)，负值
    double land_vel_threshold;       // 着陆检测：速度阈值(m/s)
    double land_hold_time;           // 着陆检测：条件持续满足多久判定着陆(秒)

    // ── 超时保护 ──
    double cmd_timeout;              // EGO 命令超时(秒)，超时则 TRACKING→HOVER
    double odom_timeout;             // 里程计超时(秒)，超时则停发 setpoint 触发 PX4 failsafe

    // ── 到达检测参数 ──
    double reach_pos_threshold;      // 位置距离阈值(m)
    double reach_vel_threshold;      // EGO 输出速度阈值(m/s)，低于此视为停下
    double reach_hold_time;          // 满足条件持续多久后判定到达(秒)

    // ── OVERRIDE 参数 ──
    bool   override_enable;          // 是否启用 OVERRIDE 功能
    double override_cmd_timeout;     // OVERRIDE 命令超时(秒)，超时则原地悬停

    // ── 调试 ──
    bool debug_enable;               // 是否发布 debug 话题

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
//  桥接节点主类 — 状态机、数据缓存、回调、控制循环都在这里
// ─────────────────────────────────────────────────────────────
class EgoBridgeNode {
public:
    /// 构造函数：加载参数、初始化订阅者/发布者/服务客户端、启动定时控制循环
    EgoBridgeNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh)
    {
        params_.load(pnh_);  // 从 ROS 参数服务器读取配置

        // ---- 订阅者：接收外部数据 和 控制指令 ----
        sub_mavros_state_    = nh_.subscribe("/mavros/state",          10, &EgoBridgeNode::mavrosStateCb, this);    // PX4 模式/解锁状态
        sub_mavros_ext_      = nh_.subscribe("/mavros/extended_state", 10, &EgoBridgeNode::extendedStateCb, this); // PX4 着陆状态
        sub_odom_            = pnh_.subscribe("odom",            10, &EgoBridgeNode::odomCb, this);               // 无人机里程计位姿+速度
        sub_cmd_             = pnh_.subscribe("cmd",             10, &EgoBridgeNode::cmdCb, this);                // EGO-Planner 轨迹命令
        sub_takeoff_land_    = pnh_.subscribe("takeoff_land",    10, &EgoBridgeNode::takeoffLandCb, this);        // 起飞/降落指令
        sub_target_point_    = pnh_.subscribe("target_point",    10, &EgoBridgeNode::targetPointCb, this);        // 目标点（用于到达检测）
        sub_set_ctrl_mode_   = pnh_.subscribe("set_control_mode",10, &EgoBridgeNode::setControlModeCb, this);     // 切换 EGO/OVERRIDE 控制模式
        sub_override_cmd_    = pnh_.subscribe("override_cmd",    10, &EgoBridgeNode::overrideCmdCb, this);        // OVERRIDE 模式下的外部命令
        sub_emergency_stop_  = pnh_.subscribe("emergency_stop",  10, &EgoBridgeNode::emergencyStopCb, this);      // 紧急停止

        // ---- 发布者：向 MAVROS / EGO / 上层 发送控制指令和状态 ----
        pub_setpoint_    = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);  // 发给 PX4 的位置/速度/加速度指令
        pub_trigger_     = nh_.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);          // 触发 EGO 开始规划
        pub_reach_       = pnh_.advertise<std_msgs::UInt8>("reach_status", 10);                            // 到达状态: 0=未到达, 1=已到达
        pub_flight_state_= pnh_.advertise<std_msgs::String>("flight_state", 10);                           // FSM 状态字符串
        pub_ctrl_mode_   = pnh_.advertise<std_msgs::UInt8>("control_mode", 10);                            // 控制模式: 0=EGO, 1=OVERRIDE
        if (params_.debug_enable)
            pub_debug_  = pnh_.advertise<std_msgs::Float64MultiArray>("debug", 10);                        // 调试信息

        // ---- 服务客户端：调用 MAVROS 切模式 / 解锁 ----
        srv_set_mode_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");       // 切换飞控模式（OFFBOARD 等）
        srv_arming_   = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");  // 解锁/锁定电机

        // 主控制定时器，按 ctrl_freq 频率触发 mainLoop
        timer_ = nh_.createTimer(ros::Duration(1.0 / params_.ctrl_freq),
                                 &EgoBridgeNode::mainLoop, this);

        ROS_INFO("[ego_bridge] Initialized. ctrl_freq=%.1f Hz", params_.ctrl_freq);
    }

private:
    // ── ROS 句柄 ──
    ros::NodeHandle nh_, pnh_;      // 全局 + 私有命名空间句柄
    ros::Timer timer_;              // 主控制循环定时器

    // 订阅者
    ros::Subscriber sub_mavros_state_, sub_mavros_ext_, sub_odom_, sub_cmd_;
    ros::Subscriber sub_takeoff_land_, sub_target_point_;
    ros::Subscriber sub_set_ctrl_mode_, sub_override_cmd_, sub_emergency_stop_;

    // 发布者
    ros::Publisher pub_setpoint_, pub_trigger_;
    ros::Publisher pub_reach_, pub_flight_state_, pub_ctrl_mode_, pub_debug_;

    // 服务客户端
    ros::ServiceClient srv_set_mode_, srv_arming_;

    // 参数集合
    Params params_;

    // ── FSM 状态 ──
    State state_ = State::IDLE;        // 当前主状态
    uint8_t control_mode_ = 0;         // 控制模式：0=EGO(自动跟踪), 1=OVERRIDE(外部接管)

    // ── 数据缓存 ──
    mavros_msgs::State          mavros_state_;                          // PX4 当前模式、解锁状态
    mavros_msgs::ExtendedState  extended_state_;                        // PX4 着陆状态
    ros::Time                   mavros_state_stamp_ = ros::Time(0);    // 上次收到 MAVROS State 的时间

    Eigen::Vector3d odom_pos_   = Eigen::Vector3d::Zero();             // 当前位置 (ENU 坐标系)
    Eigen::Vector3d odom_vel_   = Eigen::Vector3d::Zero();             // 当前速度
    double          odom_yaw_   = 0.0;                                 // 当前航向角(rad)
    ros::Time       odom_stamp_ = ros::Time(0);                        // 里程计时间戳（用于超时检测）
    bool            odom_received_ = false;                            // 是否收到过里程计数据

    quadrotor_msgs::PositionCommand latest_cmd_;                       // EGO-Planner 最新轨迹命令
    ros::Time                       cmd_stamp_  = ros::Time(0);        // EGO 命令时间戳（用于超时检测）
    bool                            cmd_received_ = false;             // 是否收到过 EGO 命令

    quadrotor_msgs::PositionCommand override_cmd_;                     // OVERRIDE 模式下的外部命令
    ros::Time                       override_cmd_stamp_ = ros::Time(0);// OVERRIDE 命令时间戳

    Eigen::Vector3d target_point_ = Eigen::Vector3d::Zero();   // 上层设定的目标点（与到达检测配合使用）
    bool            target_set_   = false;                     // 是否收到过目标点

    // ── PRE_OFFBOARD 状态变量：切 OFFBOARD 模式的临时变量 ──
    int    pre_send_counter_    = 0;     // 已预发的 setpoint 帧数
    bool   offboard_requested_  = false; // 是否已开始尝试切 OFFBOARD
    ros::Time offboard_first_try_time_;  // 首次尝试切 OFFBOARD 的时间（用于超时判断）
    ros::Time last_offboard_try_time_;   // 上次调用 setMode 的时间（每秒重试一次）

    // ── TAKEOFF 状态变量：起飞过程跟踪 ──
    Eigen::Vector3d takeoff_start_pos_;  // 起飞时的起始位置
    double          takeoff_target_z_   = 0.0;  // 起飞目标高度
    double          takeoff_start_yaw_  = 0.0;  // 起飞时锁定的航向角
    ros::Time       takeoff_start_time_;         // 起飞计时起点
    bool            warmup_done_        = false; // 电机预热是否完成
    bool            trigger_sent_       = false; // 是否已发送 traj_start_trigger
    ros::Time       hover_enter_time_;           // 进入 HOVER 的时间（用于延迟触发）

    // ── HOVER 状态变量：悬停位置 + 新轨迹等待 ──
    Eigen::Vector3d hover_pos_;             // 悬停锁定位置
    double          hover_yaw_       = 0.0; // 悬停锁定航向角
    bool            wait_new_traj_  = false;// 是否等待新 trajectory_id 才进 TRACKING
    uint32_t        old_traj_id_    = 0;    // 进入 HOVER 时的旧 trajectory_id

    // ── LANDING 状态变量：降落过程跟踪 ──
    Eigen::Vector3d landing_start_pos_;     // 降落起始位置（xy 保持不变）
    double          landing_start_z_    = 0.0;  // 降落起始高度
    ros::Time       landing_start_time_;         // 降落计时起点
    ros::Time       land_detect_start_;          // 着陆检测条件开始满足的时间
    bool            land_detect_active_ = false; // 着陆检测是否激活

    // ── 到达检测：EGO 到目标点后发布 reach_status=1 ──
    ros::Time reach_detect_start_;          // 满足条件开始计时
    bool      reach_detecting_   = false;   // 是否正在检测中
    uint8_t   last_reach_status_ = 0;       // 上次发布的到达状态

    // ── 紧急停止 ──
    bool emergency_stop_ = false;           // 收到紧急停止后置 true，停止发 setpoint

    // ────────────────────────────────────────────
    //  【回调函数】— 各 Topic 数据更新入口
    // ────────────────────────────────────────────

    /// 【回调】更新 PX4 飞控模式和解锁状态
    void mavrosStateCb(const mavros_msgs::State::ConstPtr& msg) {
        mavros_state_ = *msg;
        mavros_state_stamp_ = ros::Time::now();
    }

    /// 【回调】更新 PX4 扩展状态（主要用到 landed_state 判断着陆）
    void extendedStateCb(const mavros_msgs::ExtendedState::ConstPtr& msg) {
        extended_state_ = *msg;
    }

    /// 【回调】里程计数据：提取位置、速度、航向角（yaw）
    void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
        odom_pos_ << msg->pose.pose.position.x,
                     msg->pose.pose.position.y,
                     msg->pose.pose.position.z;
        odom_vel_ << msg->twist.twist.linear.x,
                     msg->twist.twist.linear.y,
                     msg->twist.twist.linear.z;

        // 从四元数提取航向角 yaw
        double q_x = msg->pose.pose.orientation.x;
        double q_y = msg->pose.pose.orientation.y;
        double q_z = msg->pose.pose.orientation.z;
        double q_w = msg->pose.pose.orientation.w;
        odom_yaw_ = std::atan2(2.0 * (q_w * q_z + q_x * q_y),
                               1.0 - 2.0 * (q_y * q_y + q_z * q_z));

        odom_stamp_    = msg->header.stamp;
        odom_received_ = true;
    }

    /// 【回调】EGO-Planner 的轨迹命令：缓存命令并记录时间戳
    void cmdCb(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
        latest_cmd_   = *msg;
        cmd_stamp_    = ros::Time::now();
        cmd_received_ = true;
    }

    /// 【回调】起飞/降落指令：根据当前状态决定是否执行
    void takeoffLandCb(const quadrotor_msgs::TakeoffLand::ConstPtr& msg) {
        if (msg->takeoff_land_cmd == quadrotor_msgs::TakeoffLand::TAKEOFF) {
            if (state_ == State::IDLE) {
                ROS_INFO("[ego_bridge] TAKEOFF command received");
                enterPreOffboard();  // 只有 IDLE 状态才接受起飞
            } else {
                ROS_WARN("[ego_bridge] TAKEOFF ignored: state=%s", stateStr(state_));
            }
        } else if (msg->takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND) {
            if (state_ == State::HOVER || state_ == State::TRACKING) {
                ROS_INFO("[ego_bridge] LAND command received");
                enterLanding();  // 只有悬停/跟踪状态才接受降落
            } else {
                ROS_WARN("[ego_bridge] LAND ignored: state=%s", stateStr(state_));
            }
        }
    }

    /// 【回调】目标点更新：保存目标 + 重置到达检测（确保新目标从 reach_status=0 开始）
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

    /// 【回调】切换控制模式：1=进入 OVERRIDE，0=退出 OVERRIDE → HOVER(wait_new_traj)
    void setControlModeCb(const std_msgs::UInt8::ConstPtr& msg) {
        if (!params_.override_enable) {
            ROS_WARN("[ego_bridge] Override disabled in config, ignoring set_control_mode");
            return;
        }
        if (msg->data == 1 && control_mode_ == 0 &&
            (state_ == State::TRACKING || state_ == State::HOVER)) {
            // 进入 OVERRIDE 子模式（当前必须在 TRACKING 或 HOVER）
            control_mode_ = 1;
            ROS_INFO("[ego_bridge] Entering OVERRIDE mode (state=%s)", stateStr(state_));
        } else if (msg->data == 0 && control_mode_ == 1) {
            // 退出 OVERRIDE → 回到 HOVER，并等待新的轨迹 ID 再进 TRACKING
            control_mode_ = 0;
            ROS_INFO("[ego_bridge] Exiting OVERRIDE → HOVER (wait_new_traj)");
            enterHoverFromOverride();
        }
    }

    /// 【回调】OVERRIDE 命令：缓存外部控制指令
    void overrideCmdCb(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
        override_cmd_ = *msg;
        override_cmd_stamp_ = ros::Time::now();
    }

    /// 【回调】紧急停止：立即停发 setpoint → PX4 触发 failsafe
    void emergencyStopCb(const std_msgs::Empty::ConstPtr& /*msg*/) {
        ROS_ERROR("[ego_bridge] EMERGENCY STOP!");
        emergency_stop_ = true;
        state_ = State::IDLE;
        control_mode_ = 0;
    }

    // ────────────────────────────────────────────
    //  【状态转换】— 进入各状态时的初始化逻辑
    // ────────────────────────────────────────────

    /// 进入 PRE_OFFBOARD：重置计数器，准备开始预发 setpoint
    void enterPreOffboard() {
        state_ = State::PRE_OFFBOARD;
        pre_send_counter_   = 0;
        offboard_requested_ = false;
        offboard_first_try_time_ = ros::Time(0);
        ROS_INFO("[ego_bridge] → PRE_OFFBOARD");
    }

    /// 进入 TAKEOFF：记录起始位置，计算目标高度，启动预热计时
    void enterTakeoff() {
        state_ = State::TAKEOFF;
        takeoff_start_pos_  = odom_pos_;                  // 记录当前位置作为起飞基准
        takeoff_start_yaw_  = odom_yaw_;                  // 锁定起飞航向，避免起飞阶段 yaw 跟随漂动
        takeoff_target_z_   = params_.takeoff_height;     // 目标绝对高度
        if (takeoff_target_z_ < odom_pos_.z()) {
            takeoff_target_z_ = odom_pos_.z();            // 保护：如果已经高于目标则保持当前高度
        }
        takeoff_start_time_ = ros::Time::now();
        warmup_done_        = false;
        trigger_sent_       = false;
        ROS_INFO("[ego_bridge] → TAKEOFF (target_z=%.2f)", takeoff_target_z_);
    }

    /// 进入 HOVER：锁定当前位置作为悬停点
    void enterHover() {
        state_ = State::HOVER;
        hover_pos_ = odom_pos_;
        hover_yaw_ = odom_yaw_;
        hover_enter_time_ = ros::Time::now();
        wait_new_traj_ = false;
        control_mode_  = 0;
        ROS_INFO("[ego_bridge] → HOVER at (%.2f, %.2f, %.2f)",
                 hover_pos_.x(), hover_pos_.y(), hover_pos_.z());
    }

    /// 从 OVERRIDE 退出进 HOVER：锁定当前位 + 设 wait_new_traj 等待新轨迹 ID
    void enterHoverFromOverride() {
        state_ = State::HOVER;
        hover_pos_ = odom_pos_;
        hover_yaw_ = odom_yaw_;
        hover_enter_time_ = ros::Time::now();
        wait_new_traj_ = true;
        old_traj_id_   = latest_cmd_.trajectory_id;
        control_mode_  = 0;
        ROS_INFO("[ego_bridge] → HOVER (wait_new_traj, old_id=%u) at (%.2f, %.2f, %.2f)",
                 old_traj_id_, hover_pos_.x(), hover_pos_.y(), hover_pos_.z());
    }

    /// 进入 TRACKING：重置控制模式和到达检测
    void enterTracking() {
        state_ = State::TRACKING;
        control_mode_ = 0;
        reach_detecting_ = false;
        last_reach_status_ = 0;
        ROS_INFO("[ego_bridge] → TRACKING");
    }

    /// 进入 LANDING：记录当前高度作为降落起点
    void enterLanding() {
        state_ = State::LANDING;
        landing_start_pos_ = odom_pos_;
        landing_start_z_   = odom_pos_.z();
        landing_start_time_ = ros::Time::now();
        land_detect_active_ = false;
        control_mode_ = 0;
        ROS_INFO("[ego_bridge] → LANDING from z=%.2f", landing_start_z_);
    }

    /// 进入 IDLE：重置所有标志
    void enterIdle() {
        state_ = State::IDLE;
        control_mode_ = 0;
        emergency_stop_ = false;
        ROS_INFO("[ego_bridge] → IDLE");
    }

    // ────────────────────────────────────────────
    //  【Setpoint 构建器】— 将位置/速度/加速度 封装成 MAVROS PositionTarget
    // ────────────────────────────────────────────

    /// 通用构建器：填充 pos/vel/acc/yaw 并设置 type_mask
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

    /** 纯位置 setpoint：忽略速度+加速度（用于悬停、等待等场景） */
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

    /** 位置+速度 setpoint：忽略加速度（用于起飞爬升、降落等场景） */
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

    /** 全量轨迹 setpoint：位置+速度+加速度+yaw（用于 EGO/OVERRIDE 跟踪） */
    mavros_msgs::PositionTarget fullTrackingSetpoint(
        const quadrotor_msgs::PositionCommand& cmd)
    {
        Eigen::Vector3d pos(cmd.position.x, cmd.position.y, cmd.position.z);
        Eigen::Vector3d vel(cmd.velocity.x, cmd.velocity.y, cmd.velocity.z);
        Eigen::Vector3d acc(cmd.acceleration.x, cmd.acceleration.y, cmd.acceleration.z);
        return buildPositionTarget(pos, vel, acc, cmd.yaw, 0);
    }

    // ────────────────────────────────────────────
    //  【服务调用】— 通过 MAVROS 控制 PX4
    // ────────────────────────────────────────────

    /// 调用 MAVROS 服务切到 OFFBOARD 模式
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

    /// 解锁电机
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

    /// 锁定电机（降落后调用）
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
    //  【主控制循环】— 定时器回调，每周期执行安全检查 + 状态机
    // ────────────────────────────────────────────
    void mainLoop(const ros::TimerEvent& /*e*/) {
        ros::Time now = ros::Time::now();

        // ── 安全检查①：紧急停止 → 停发 setpoint，由 PX4 failsafe 处理 ──
        if (emergency_stop_) {
            // Don't send any setpoint → PX4 failsafe
            publishStatus();
            return;
        }

        // ── 安全检查②：里程计超时 → 停发 setpoint 触发 PX4 failsafe ──
        if (state_ != State::IDLE && odom_received_) {
            if ((now - odom_stamp_).toSec() > params_.odom_timeout) {
                ROS_ERROR("[ego_bridge] Odom timeout! Stopping setpoints → PX4 failsafe");
                // Don't send setpoint, PX4 will handle failsafe
                publishStatus();
                return;
            }
        }

        // ── 安全检查③：OFFBOARD 模式丢失检测（仅警告，由飞手或 PX4 处理） ──
        if (state_ != State::IDLE && state_ != State::PRE_OFFBOARD) {
            if (mavros_state_stamp_ != ros::Time(0) &&
                mavros_state_.mode != "OFFBOARD")
            {
                ROS_WARN_THROTTLE(2.0, "[ego_bridge] PX4 mode is '%s', not OFFBOARD!",
                                  mavros_state_.mode.c_str());
            }
        }

        // ── 状态机分发：根据当前状态调用对应的处理函数 ──
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
    //  【状态处理器】— 每个状态的每周期逻辑
    // ────────────────────────────────────────────

    /// 【IDLE】空闲状态，什么都不做，等待 takeoffLandCb 触发起飞
    // ── IDLE ──
    void runIdle(const ros::Time& /*now*/) {
        // 无操作，等待 TAKEOFF 命令
    }

    /// 【PRE_OFFBOARD】三阶段：预发 setpoint → 尝试切 OFFBOARD → 解锁 → 进 TAKEOFF
    // ── PRE_OFFBOARD ──
    void runPreOffboard(const ros::Time& now) {
        if (!odom_received_) {
            ROS_WARN_THROTTLE(1.0, "[ego_bridge] PRE_OFFBOARD: Waiting for odometry...");
            return;  // 还没有里程计数据，无法发 setpoint
        }

        // 每周期发一帧“当前位置”的 setpoint（PX4 要求切 OFFBOARD 前必须收到 setpoint）
        pub_setpoint_.publish(posOnlySetpoint(odom_pos_));
        pre_send_counter_++;

        // 第一阶段：累计发送帧数，不够则继续
        if (!offboard_requested_ && pre_send_counter_ < params_.pre_send_count) {
            return;
        }

        // 第二阶段：首次进入尝试切 OFFBOARD 流程
        if (!offboard_requested_) {
            offboard_first_try_time_ = now;
            last_offboard_try_time_  = ros::Time(0);
            offboard_requested_ = true;
        }

        // 第三阶段：检查是否已进入 OFFBOARD 模式
        if (mavros_state_.mode == "OFFBOARD") {
            // 已进 OFFBOARD，检查是否需要自动解锁
            if (params_.enable_auto_arm && !mavros_state_.armed) {
                armDrone();
                return;  // 等一个周期确认解锁结果
            }
            if (mavros_state_.armed) {
                enterTakeoff();  // 已解锁，进入起飞状态
                return;
            }
            // 未解锁且禁用自动解锁 — 等待手动解锁
            ROS_WARN_THROTTLE(1.0, "[ego_bridge] In OFFBOARD but not armed. Waiting for manual arm.");
            return;
        }

        // 超时检测：切 OFFBOARD 太久则放弃，回 IDLE
        if ((now - offboard_first_try_time_).toSec() > params_.offboard_timeout) {
            ROS_ERROR("[ego_bridge] OFFBOARD switch timeout (%.1fs). Returning to IDLE.",
                      params_.offboard_timeout);
            enterIdle();
            return;
        }

        // 每秒重试一次 setMode 调用
        if ((now - last_offboard_try_time_).toSec() >= 1.0) {
            last_offboard_try_time_ = now;
            setOffboardMode();
        }
    }

    /// 【TAKEOFF】两阶段：电机预热 → 匀速爬升(接近目标时减速) → 到达后进 HOVER
    // ── TAKEOFF ──
    void runTakeoff(const ros::Time& now) {
        double dt = (now - takeoff_start_time_).toSec();

        // 阶段一：电机预热 — 发送微小高度偏移让电机转起来，不足以离地
        if (!warmup_done_) {
            if (dt < params_.motor_warmup_time) {
                Eigen::Vector3d target = takeoff_start_pos_;
                target.z() += params_.motor_warmup_height;
                pub_setpoint_.publish(posOnlySetpoint(target, takeoff_start_yaw_));
                return;  // 继续预热
            }
            warmup_done_ = true;
            takeoff_start_time_ = now;  // 重置计时器，开始爬升阶段
            dt = 0.0;
        }

        // 阶段二：爬升 — 根据里程计反馈计算剩余距离，接近时减速
        double remaining = takeoff_target_z_ - odom_pos_.z();  // 距目标高度还有多远

        if (remaining <= params_.reach_threshold) {
            // 到达目标高度 → 进入 HOVER
            Eigen::Vector3d target_pos = odom_pos_;
            target_pos.z() = takeoff_target_z_;
            pub_setpoint_.publish(posOnlySetpoint(target_pos, takeoff_start_yaw_));
            enterHover();
            return;
        }

        // 减速区：进入 decel_distance 范围后线性降速，但不低于 min_speed
        double speed = params_.takeoff_speed;
        if (remaining < params_.decel_distance && params_.decel_distance > 1e-4) {
            speed *= remaining / params_.decel_distance;
            speed = std::max(speed, params_.min_speed);
        }

        Eigen::Vector3d target_pos = takeoff_start_pos_;
        target_pos.z() = std::min(takeoff_start_pos_.z() + params_.takeoff_speed * dt,
                                  takeoff_target_z_);
        Eigen::Vector3d vel(0.0, 0.0, speed);
        pub_setpoint_.publish(posVelSetpoint(target_pos, vel, takeoff_start_yaw_));
    }

    /// 【HOVER】悬停状态：锁定位置 + 延时触发 EGO + 等待 EGO 命令进 TRACKING
    // ── HOVER ──
    void runHover(const ros::Time& now) {
        // OVERRIDE 子模式：在 HOVER 中也支持外部接管
        if (control_mode_ == 1) {
            runOverride(now);
            return;
        }

        // 发送悬停位置的 setpoint，维持当前位置
        pub_setpoint_.publish(posOnlySetpoint(hover_pos_, hover_yaw_));

        // 延迟触发：从 TAKEOFF 刚进入 HOVER 后，等 delay_trigger_time 秒再发 trigger
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

        // 检测 EGO 命令，条件满足则转 TRACKING
        if (cmd_received_ && (now - cmd_stamp_).toSec() < params_.cmd_timeout) {
            Eigen::Vector3d cmd_vel(latest_cmd_.velocity.x,
                                    latest_cmd_.velocity.y,
                                    latest_cmd_.velocity.z);
            bool has_velocity = cmd_vel.norm() > 0.01;  // 有速度才算有效命令

            if (wait_new_traj_) {
                // 从 OVERRIDE 回来时，必须等到新 trajectory_id 才进 TRACKING
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

    /// 【TRACKING】轨迹跟踪：转发 EGO 命令(pos+vel+acc+yaw) + 到达检测
    // ── TRACKING ──
    void runTracking(const ros::Time& now) {
        // OVERRIDE 子模式：转发外部命令而非 EGO 命令
        if (control_mode_ == 1) {
            runOverride(now);
            return;
        }

        // 正常 EGO 跟踪：检查命令是否超时
        if (!cmd_received_ || (now - cmd_stamp_).toSec() > params_.cmd_timeout) {
            // EGO 命令超时 → 立即悬停
            ROS_WARN("[ego_bridge] EGO cmd timeout → HOVER");
            enterHover();
            trigger_sent_ = true;  // 超时回 HOVER 不需要重新触发 EGO
            return;
        }

        // 发布全量轨迹 setpoint（pos + vel + acc + yaw）
        pub_setpoint_.publish(fullTrackingSetpoint(latest_cmd_));

        // 每周期检测是否到达目标点
        runReachDetection(now);
    }

    /// 【OVERRIDE】子模式：转发外部命令，超时则悬停在当前位置
    void runOverride(const ros::Time& now) {
        if ((now - override_cmd_stamp_).toSec() < params_.override_cmd_timeout) {
            // OVERRIDE 命令有效，转发为全量轨迹 setpoint
            pub_setpoint_.publish(fullTrackingSetpoint(override_cmd_));
        } else {
            // OVERRIDE 命令超时，就地悬停保护
            pub_setpoint_.publish(posOnlySetpoint(odom_pos_));
        }
    }

    /// 【LANDING】匀速下降 + 近地减速 + 着陆检测（两种方式）
    // ── LANDING ──
    void runLanding(const ros::Time& now) {
        double dt = (now - landing_start_time_).toSec();

        // 计算目标 z：线性下降
        double target_z = landing_start_z_ - params_.landing_speed * dt;
        double speed = params_.landing_speed;

        // 近地减速：低于 slow_height 时乘以 slow_factor
        if (odom_pos_.z() < params_.slow_height) {
            speed *= params_.slow_factor;
        }

        Eigen::Vector3d target_pos = landing_start_pos_;
        target_pos.z() = target_z;
        Eigen::Vector3d vel(0.0, 0.0, -speed);

        pub_setpoint_.publish(posVelSetpoint(target_pos, vel));

        // 着陆检测方式B：PX4 内部报告已着地
        if (extended_state_.landed_state ==
            mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
        {
            ROS_INFO("[ego_bridge] PX4 reports ON_GROUND");
            completeLanding();
            return;
        }

        // 着陆检测方式A：目标z已远低于实际z(pos_deviation<0) + 速度很小 → 持续 hold_time 则判定着陆
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

    /// 着陆完成：锁定电机 + 切到 AUTO.LAND + 进入 IDLE
    void completeLanding() {
        disarmDrone();  // 锁定电机

        // 退出 OFFBOARD（切到 AUTO.LAND 让 PX4 自行处理剩余流程）
        mavros_msgs::SetMode req;
        req.request.custom_mode = "AUTO.LAND";
        srv_set_mode_.call(req);

        enterIdle();
    }

    // ────────────────────────────────────────────
    //  【到达检测】— EGO 速度趋近零 + 位置接近目标 → 持续 hold_time 后判定到达
    // ────────────────────────────────────────────
    void runReachDetection(const ros::Time& now) {
        // 取 EGO 输出的速度向量
        Eigen::Vector3d cmd_vel(latest_cmd_.velocity.x,
                                latest_cmd_.velocity.y,
                                latest_cmd_.velocity.z);

        bool vel_low = cmd_vel.norm() < params_.reach_vel_threshold;  // EGO 输出速度足够小

        // 如果设定了目标点，检查位置距离
        bool pos_close = true;
        if (target_set_) {
            Eigen::Vector3d diff = odom_pos_ - target_point_;
            pos_close = diff.norm() < params_.reach_pos_threshold;
        }

        if (vel_low && pos_close) {
            // 条件满足：开始计时
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
    //  【状态发布】— 每周期广播 FSM 状态、控制模式、调试信息
    // ────────────────────────────────────────────
    void publishStatus() {
        // 发布 FSM 状态字符串
        std_msgs::String state_msg;
        state_msg.data = stateStr(state_);
        pub_flight_state_.publish(state_msg);

        // 发布控制模式 (0=EGO, 1=OVERRIDE)
        std_msgs::UInt8 mode_msg;
        mode_msg.data = control_mode_;
        pub_ctrl_mode_.publish(mode_msg);

        // 调试信息：包含状态编号、目标距离、命令延迟、控制模式、高度、到达状态
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
//  程序入口
// ─────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    ros::init(argc, argv, "ego_bridge_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    EgoBridgeNode node(nh, pnh);
    ros::spin();
    return 0;
}
