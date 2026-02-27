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
    ros::NodeHandle nh_;
    std::string bridge_ns_;

    // ── Subscribers ──
    ros::Subscriber sub_flight_state_;
    ros::Subscriber sub_reach_status_;
    ros::Subscriber sub_control_mode_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_override_trigger_;

    // ── Publishers ──
    ros::Publisher pub_takeoff_land_;
    ros::Publisher pub_goal_;
    ros::Publisher pub_target_point_;
    ros::Publisher pub_set_ctrl_mode_;
    ros::Publisher pub_override_cmd_;
    ros::Publisher pub_emergency_stop_;
    ros::Publisher pub_override_trigger_;

    // ── 缓存数据 ──
    std::string     flight_state_   = "UNKNOWN";
    uint8_t         reach_status_   = 0;
    uint8_t         control_mode_   = 0;
    Eigen::Vector3d odom_pos_       = Eigen::Vector3d::Zero();
    double          odom_yaw_       = 0.0;
    bool            connected_      = false;

    // ── Override trigger ──
    int  pending_task_id_    = -1;
    bool trigger_received_   = false;

    // ── Callbacks ──
    void flightStateCb(const std_msgs::String::ConstPtr& msg);
    void reachStatusCb(const std_msgs::UInt8::ConstPtr& msg);
    void controlModeCb(const std_msgs::UInt8::ConstPtr& msg);
    void odomCb(const nav_msgs::Odometry::ConstPtr& msg);
    void overrideTriggerCb(const std_msgs::Int32::ConstPtr& msg);

    // ── 内部工具 ──
    void publishGoal(double x, double y, double z, double yaw);

    /** @brief 构造一帧 PositionCommand */
    quadrotor_msgs::PositionCommand buildPositionCmd(
        double x, double y, double z, double yaw,
        double vx = 0, double vy = 0, double vz = 0);
};

#endif // EGO_API_EGO_API_H
