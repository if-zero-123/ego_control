#ifndef EGO_API_EGO_API_H
#define EGO_API_EGO_API_H

#include <ros/ros.h>
#include <Eigen/Dense>

#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>

/**
 * @class EgoApi
 * @brief 封装 ego_bridge 接口的高级阻塞式飞行控制 API。
 *
 * 提供起飞/降落/目标点/Override 等阻塞函数，适合在 main 中像脚本一样逐行调用。
 *
 * 使用示例:
 * @code
 *   EgoApi api(nh);
 *   api.takeoff(30.0);
 *   api.sendGoal(3.0, 0.0, 1.5, 60.0);
 *   api.triggerOverrideTask(1);
 *   api.waitOverrideComplete(120.0);
 *   api.land(30.0);
 * @endcode
 */
class EgoApi {
public:
    /**
     * @brief 构造函数，初始化所有订阅/发布。
     * @param nh ROS NodeHandle
     * @param bridge_ns ego_bridge 的命名空间，默认 "/ego_bridge"
     */
    EgoApi(ros::NodeHandle& nh, const std::string& bridge_ns = "/ego_bridge");

    // ═══════════ 状态查询 ═══════════

    /** @brief 获取当前飞行状态字符串 (IDLE/PRE_OFFBOARD/TAKEOFF/HOVER/TRACKING/LANDING) */
    std::string getFlightState() const;

    /** @brief 获取到达状态 (0=未到达, 1=已到达) */
    uint8_t getReachStatus() const;

    /** @brief 获取控制模式 (0=EGO, 1=OVERRIDE) */
    uint8_t getControlMode() const;

    /** @brief 获取当前里程计位置 */
    Eigen::Vector3d getOdomPosition() const;

    /** @brief 获取当前 yaw 角 (弧度) */
    double getOdomYaw() const;

    /** @brief 是否已连接到 ego_bridge (收到过 flight_state) */
    bool isConnected() const;

    // ═══════════ 阻塞式飞行控制 ═══════════

    /**
     * @brief 起飞并等待进入 HOVER 状态。
     * @param timeout 超时时间(秒)
     * @return true=成功进入HOVER, false=超时
     */
    bool takeoff(double timeout = 30.0);

    /**
     * @brief 降落并等待进入 IDLE 状态。
     * @param timeout 超时时间(秒)
     * @return true=成功降落, false=超时
     */
    bool land(double timeout = 30.0);

    /**
     * @brief 发送目标点（自动使用当前 yaw），阻塞等待到达。
     * @param x, y, z 目标坐标
     * @param timeout 超时时间(秒)
     * @return true=到达, false=超时(跳过)
     */
    bool sendGoal(double x, double y, double z, double timeout = 60.0);

    /**
     * @brief 发送目标点（指定 yaw），阻塞等待到达。
     * @param x, y, z 目标坐标
     * @param yaw 目标朝向(弧度)
     * @param timeout 超时时间(秒)
     * @return true=到达, false=超时(跳过)
     */
    bool sendGoalWithYaw(double x, double y, double z, double yaw, double timeout = 60.0);

    /**
     * @brief 只发布目标点，不阻塞等待到达。
     *
     * 适合上层自己同时监控到达状态和任务阶段切换条件。
     */
    void publishGoalOnly(double x, double y, double z, double yaw);

    // ═══════════ Override 控制 ═══════════

    /**
     * @brief 请求进入 OVERRIDE 模式。
     * @return true=成功, false=超时(2秒内未确认)
     */
    bool enableOverride();

    /**
     * @brief 退出 OVERRIDE 模式，归还控制权。
     * @return true=成功, false=超时
     */
    bool disableOverride();

    /**
     * @brief 发送一帧 override 控制指令。
     * @note 需持续 ≥2Hz 发送，否则 ego_bridge 会自动原地悬停。
     */
    void sendOverrideCmd(const quadrotor_msgs::PositionCommand& cmd);

    /**
     * @brief 发送一帧速度控制指令（世界坐标系）。
     *
     * 非阻塞，发一帧后立即返回。调用方需以 ≥2Hz 持续调用，
     * 否则 ego_bridge 会因超时自动切回悬停。
     * 适合外环 PID 控制器在循环中调用。
     *
     * @param vx  世界系 X 轴线速度 (m/s)
     * @param vy  世界系 Y 轴线速度 (m/s)
     * @param vz  世界系 Z 轴线速度 (m/s)
     * @param yaw_rate  偏航角速度 (rad/s)
     */
    void sendVelocityCmd(double vx, double vy, double vz, double yaw_rate = 0.0);

    /**
     * @brief 在当前位置发送一帧悬停 override cmd。
     */
    void holdPosition();

    /**
     * @brief 在 OVERRIDE 模式下阻塞移动到指定位置。
     * @param x, y, z 目标坐标
     * @param yaw 目标朝向(弧度)
     * @param pos_threshold 到达判定阈值(米)
     * @param timeout 超时时间(秒)
     * @return true=到达, false=超时
     */
    bool moveToOverride(double x, double y, double z, double yaw,
                        double pos_threshold = 0.2, double timeout = 30.0);

    // ═══════════ Override 任务触发 ═══════════

    /**
     * @brief 触发 override 任务（主示例调用）。
     * @param task_id 任务编号 (1-6)
     */
    void triggerOverrideTask(int task_id);

    /**
     * @brief 阻塞等待 override 触发信号（override 示例调用）。
     * @param timeout 超时时间(秒)，0=永不超时
     * @return 任务 ID，超时返回 -1
     */
    int waitForOverrideTrigger(double timeout = 0);

    /**
     * @brief 阻塞等待 override 完成（control_mode 回到 0）。
     * @param timeout 超时时间(秒)
     * @return true=完成, false=超时
     */
    bool waitOverrideComplete(double timeout = 60.0);

    // ═══════════ 紧急 ═══════════

    /** @brief 紧急停止 */
    void emergencyStop();

private:
    ros::NodeHandle nh_;       // ROS 句柄
    std::string bridge_ns_;    // ego_bridge 命名空间（默认 "/ego_bridge"）

    // ── 订阅者：从 ego_bridge 接收状态更新 ──
    ros::Subscriber sub_flight_state_;      // FSM 状态字符串
    ros::Subscriber sub_reach_status_;      // 到达状态 (0/1)
    ros::Subscriber sub_control_mode_;      // 控制模式 (0=EGO, 1=OVERRIDE)
    ros::Subscriber sub_odom_;              // 里程计位姿
    ros::Subscriber sub_override_trigger_;  // override 任务触发信号

    // ── 发布者：向 ego_bridge / ego_planner / override 节点发送指令 ──
    ros::Publisher pub_takeoff_land_;       // 起飞/降落指令
    ros::Publisher pub_goal_;               // 发给 EGO-Planner 的目标点
    ros::Publisher pub_target_point_;       // 发给 ego_bridge 的目标点（到达检测用）
    ros::Publisher pub_set_ctrl_mode_;      // 切换 EGO/OVERRIDE 模式
    ros::Publisher pub_override_cmd_;       // OVERRIDE 模式的控制指令
    ros::Publisher pub_emergency_stop_;     // 紧急停止
    ros::Publisher pub_override_trigger_;   // 触发 override 任务

    // ── 缓存数据：由回调更新，供阻塞函数查询 ──
    std::string     flight_state_   = "UNKNOWN";              // 当前 FSM 状态
    uint8_t         reach_status_   = 0;                      // 0=未到达, 1=已到达
    uint8_t         control_mode_   = 0;                      // 0=EGO, 1=OVERRIDE
    Eigen::Vector3d odom_pos_       = Eigen::Vector3d::Zero();// 当前位置
    double          odom_yaw_       = 0.0;                    // 当前航向角(rad)
    bool            connected_      = false;                  // 是否收到过 flight_state

    // ── Override 任务触发机制 ──
    int  pending_task_id_    = -1;    // 待处理的任务 ID
    bool trigger_received_   = false; // 是否收到触发信号

    // ── 回调函数：更新缓存数据 ──
    void flightStateCb(const std_msgs::String::ConstPtr& msg);    // 更新 flight_state_
    void reachStatusCb(const std_msgs::UInt8::ConstPtr& msg);     // 更新 reach_status_
    void controlModeCb(const std_msgs::UInt8::ConstPtr& msg);     // 更新 control_mode_
    void odomCb(const nav_msgs::Odometry::ConstPtr& msg);         // 更新 odom_pos_ 和 odom_yaw_
    void overrideTriggerCb(const std_msgs::Int32::ConstPtr& msg); // 更新 pending_task_id_

    // ── 内部工具 ──
    /// 同时发布目标到 EGO-Planner(规划路径) 和 ego_bridge(到达检测)
    void publishGoal(double x, double y, double z, double yaw);

    /** @brief 构造一帧 PositionCommand（用于 OVERRIDE 模式发送命令） */
    quadrotor_msgs::PositionCommand buildPositionCmd(
        double x, double y, double z, double yaw,
        double vx = 0, double vy = 0, double vz = 0);
};

#endif // EGO_API_EGO_API_H
