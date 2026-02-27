/**
 * @file ego_api_demo.cpp
 * @brief 主飞行示例 — 函数式航点编排 + Override 任务触发。
 *
 * 【中文概述】
 * 本文件演示如何用 EgoApi 编排一条完整的飞行任务：
 *   起飞 → 飞往 A → 飞往 B(指定 yaw) → 触发 Override 任务 1
 *   → 飞往 C → 飞往 D(指定 yaw) → 触发 Override 任务 2 → 降落
 *
 * 每个 sendGoal/sendGoalWithYaw 都是阻塞调用，到达后才执行下一行。
 * triggerOverrideTask 触发 override_task_demo 中对应的任务函数。
 *
 * 用法：直接修改 main 中的 sendGoal / triggerOverrideTask 行来编排飞行任务。
 *       sendGoal(x, y, z, timeout)         — 自动 yaw
 *       sendGoalWithYaw(x, y, z, yaw, timeout) — 指定 yaw
 *       triggerOverrideTask(id)            — 触发 override 示例中的对应任务
 *       waitOverrideComplete(timeout)      — 等 override 完成后再继续
 */

#include "ego_api/ego_api.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "ego_api_demo");
    ros::NodeHandle nh;

    // 从参数服务器读取 bridge 命名空间（默认 /ego_bridge）
    std::string bridge_ns;
    nh.param<std::string>("~bridge_ns", bridge_ns, "/ego_bridge");

    EgoApi api(nh, bridge_ns);  // 初始化 API，自动建立订阅/发布连接

    // 等待连接到 ego_bridge（收到第一个 flight_state 消息）
    ROS_INFO("[demo] Waiting for ego_bridge...");
    ros::Rate wait_rate(5);
    while (ros::ok() && !api.isConnected()) {
        ros::spinOnce();
        wait_rate.sleep();
    }
    ROS_INFO("[demo] Connected to ego_bridge. flight_state=%s", api.getFlightState().c_str());

    // ═══════════════════════════════════════════
    //  起飞
    // ═══════════════════════════════════════════
    ROS_INFO("[demo] ====== TAKEOFF ======");
    if (!api.takeoff(30.0)) {
        ROS_ERROR("[demo] Takeoff failed! Aborting.");
        return 1;
    }
    ROS_INFO("[demo] Takeoff complete, now hovering.\n");

    // ═══════════════════════════════════════════
    //  航段 1: 飞往 A
    // ═══════════════════════════════════════════
    ROS_INFO("[demo] ====== Waypoint A ======");
    bool reached = api.sendGoal(3.0, 0.0, 1.5, 60.0);
    ROS_INFO("[demo] A: %s\n", reached ? "REACHED" : "TIMEOUT, skip");

    // ═══════════════════════════════════════════
    //  航段 2: 飞往 B（指定 yaw 朝北 π/2）
    // ═══════════════════════════════════════════
    ROS_INFO("[demo] ====== Waypoint B (yaw=1.57) ======");
    reached = api.sendGoalWithYaw(3.0, 3.0, 1.5, 1.57, 60.0);
    ROS_INFO("[demo] B: %s\n", reached ? "REACHED" : "TIMEOUT, skip");

    // ── 到达 B 后触发 override 任务 1 ──
    ROS_INFO("[demo] Triggering override task 1...");
    api.triggerOverrideTask(1);
    api.waitOverrideComplete(120.0);
    ROS_INFO("[demo] Override task 1 complete, resuming.\n");

    // ═══════════════════════════════════════════
    //  航段 3: 飞往 C
    // ═══════════════════════════════════════════
    ROS_INFO("[demo] ====== Waypoint C ======");
    reached = api.sendGoal(0.0, 3.0, 1.5, 60.0);
    ROS_INFO("[demo] C: %s\n", reached ? "REACHED" : "TIMEOUT, skip");

    // ═══════════════════════════════════════════
    //  航段 4: 飞往 D（指定 yaw=0）
    // ═══════════════════════════════════════════
    ROS_INFO("[demo] ====== Waypoint D (yaw=0) ======");
    reached = api.sendGoalWithYaw(0.0, 0.0, 1.5, 0.0, 60.0);
    ROS_INFO("[demo] D: %s\n", reached ? "REACHED" : "TIMEOUT, skip");

    // ── 到达 D 后触发 override 任务 2 ──
    ROS_INFO("[demo] Triggering override task 2...");
    api.triggerOverrideTask(2);
    api.waitOverrideComplete(120.0);
    ROS_INFO("[demo] Override task 2 complete, resuming.\n");

    // ═══════════════════════════════════════════
    //  降落
    // ═══════════════════════════════════════════
    ROS_INFO("[demo] ====== LANDING ======");
    if (!api.land(30.0)) {
        ROS_WARN("[demo] Land timeout.");
    }
    ROS_INFO("[demo] ====== DONE ======");

    return 0;
}
