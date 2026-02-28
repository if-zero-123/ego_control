/**
 * @file override_task_demo.cpp
 * @brief Override 多任务示例 — while 循环 + switch-case 分发。
 *
 * 【中文概述】
 * 本节点在后台运行，通过 /ego_api/override_trigger 话题接收主示例发来的任务 ID。
 * 收到触发后：
 *   1. 调用 enableOverride() 接管控制权
 *   2. 根据 task_id 分发到对应的任务函数
 *   3. 任务函数返回后调用 disableOverride() 归还控制权
 *   4. 主示例的 waitOverrideComplete() 会感知到 control_mode 回 0 而继续
 *
 * 预留 6 个任务槽位，用户在对应的 taskN_xxx() 函数中填入自己的逻辑。
 *
 * 扩展方法：
 *   1. 写一个 void taskN_xxx(EgoApi& api) 函数
 *   2. 在 switch 中加 case N: taskN_xxx(api); break;
 *   3. 在主示例中加 api.triggerOverrideTask(N); api.waitOverrideComplete();
 *
 * 任务内可用 API：
 *   - api.moveToOverride(x,y,z,yaw,threshold,timeout)  阻塞精准移动
 *   - api.holdPosition()                                 原地悬停(发一帧cmd)
 *   - api.sendVelocityCmd(vx,vy,vz,yaw_rate)             速度控制(世界系,需≥2Hz)
 *   - api.sendOverrideCmd(cmd)                            自由发控制指令(需≥2Hz)
 *   - api.getOdomPosition() / api.getOdomYaw()            获取当前位姿
 */

#include "ego_api/ego_api.h"

// ══════════════════════════════════════════════════════════════
//  任务函数声明
// ══════════════════════════════════════════════════════════════
void task1_photo(EgoApi& api);
void task2_delivery(EgoApi& api);
void task3_circle(EgoApi& api);
void task4_inspect(EgoApi& api);
void task5_reserve(EgoApi& api);
void task6_reserve(EgoApi& api);

// ══════════════════════════════════════════════════════════════
//  Main
// ══════════════════════════════════════════════════════════════
int main(int argc, char** argv) {
    ros::init(argc, argv, "override_task_demo");
    ros::NodeHandle nh;

    std::string bridge_ns;
    nh.param<std::string>("~bridge_ns", bridge_ns, "/ego_bridge");

    EgoApi api(nh, bridge_ns);

    ROS_INFO("[override] Node started, waiting for trigger...");

    // 主循环：持续等待触发信号 → 接管 → 执行任务 → 归还 → 继续等待
    while (ros::ok()) {
        // 阻塞等待主示例的触发信号，返回任务 ID
        int task_id = api.waitForOverrideTrigger(0);  // 0 = 永不超时

        if (task_id < 0) {
            continue;  // 异常返回，重新等待
        }

        ROS_INFO("[override] ===== Received task %d =====", task_id);

        // 接管控制权：通过 ego_bridge 的 set_control_mode=1
        if (!api.enableOverride()) {
            ROS_ERROR("[override] Failed to enable override, skip task %d", task_id);
            continue;  // 接管失败，跳过本次任务
        }
        ROS_INFO("[override] Override enabled, executing task %d...", task_id);

        // 按任务 ID 分发到对应的任务函数
        switch (task_id) {
            case 1: task1_photo(api);     break;
            case 2: task2_delivery(api);  break;
            case 3: task3_circle(api);    break;
            case 4: task4_inspect(api);   break;
            case 5: task5_reserve(api);   break;
            case 6: task6_reserve(api);   break;
            default:
                ROS_WARN("[override] Unknown task_id=%d, holding position 3s", task_id);
                api.holdPosition();
                ros::Duration(3.0).sleep();
                break;
        }

        // 归还控制权：set_control_mode=0 → ego_bridge 回到 HOVER(wait_new_traj)
        api.disableOverride();
        ROS_INFO("[override] Task %d complete, control returned. Waiting for next trigger...\n",
                 task_id);
    }

    return 0;
}

// ════════════════════════════════════════════════════════════
//  任务实现（用户修改区域）
//  每个任务函数接收 EgoApi& 引用，在 OVERRIDE 模式下执行。
//  函数返回后会自动 disableOverride() 归还控制权。
// ════════════════════════════════════════════════════════════

// ───────────────────────────────────────
//  任务 1: 拍照（精准定位 + 悬停等待拍照）
// ───────────────────────────────────────
void task1_photo(EgoApi& api) {
    ROS_INFO("[task1] === Photo Task ===");

    // 精准移动到拍照位置（到达阈值 0.15m）
    bool ok = api.moveToOverride(3.1, 3.1, 1.5, 0.0, 0.15, 30.0);
    if (!ok) {
        ROS_WARN("[task1] Failed to reach photo position");
    }

    // 悬停等待拍照（5秒）
    ROS_INFO("[task1] Holding position for photo...");
    // TODO: 在此添加拍照逻辑，例如调用相机服务 srv_camera_.call(...)
    ros::Duration(5.0).sleep();

    ROS_INFO("[task1] Photo task done.");
}

// ───────────────────────────────────────
//  任务 2: 投递（精准定位 → 下降 → 释放货物 → 回升）
// ───────────────────────────────────────
void task2_delivery(EgoApi& api) {
    ROS_INFO("[task2] === Delivery Task ===");

    // 第一步：移动到投递点上方
    api.moveToOverride(0.0, 0.0, 1.0, 0.0, 0.1, 30.0);

    // 第二步：下降到投递高度
    api.moveToOverride(0.0, 0.0, 0.5, 0.0, 0.1, 30.0);

    // 第三步：执行投递
    ROS_INFO("[task2] Delivering payload...");
    // TODO: 在此添加投递/释放逻辑，例如控制舵机 pub_servo_.publish(...)
    ros::Duration(2.0).sleep();

    // 第四步：回到安全高度
    api.moveToOverride(0.0, 0.0, 1.5, 0.0, 0.2, 30.0);

    ROS_INFO("[task2] Delivery task done.");
}

// ─────────────────────────────────────────
//  任务 3: 绕圈（预留）
// ─────────────────────────────────────────
void task3_circle(EgoApi& api) {
    ROS_INFO("[task3] === Circle Task (NOT IMPLEMENTED) ===");

    // TODO: 用 sendOverrideCmd 在循环中发送圆形轨迹点
    // 示例框架：
    // Eigen::Vector3d center = api.getOdomPosition();
    // double radius = 1.0, omega = 0.5;  // 半径1m, 角速度0.5rad/s
    // ros::Rate rate(50);
    // ros::Time t0 = ros::Time::now();
    // while (ros::ok()) {
    //     double t = (ros::Time::now() - t0).toSec();
    //     if (t > 2 * M_PI / omega) break;  // 一圈
    //     double x = center.x() + radius * cos(omega * t);
    //     double y = center.y() + radius * sin(omega * t);
    //     double yaw = omega * t + M_PI / 2;
    //     auto cmd = ...; // buildPositionCmd
    //     api.sendOverrideCmd(cmd);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    api.holdPosition();
    ros::Duration(3.0).sleep();

    ROS_INFO("[task3] Circle task done (placeholder).");
}

// ─────────────────────────────────────────
//  任务 4: 巡检（预留）
// ─────────────────────────────────────────
void task4_inspect(EgoApi& api) {
    ROS_INFO("[task4] === Inspect Task (NOT IMPLEMENTED) ===");

    // TODO: 用户实现巡检逻辑
    // 例如：依次移动到多个检查点并采集数据
    // api.moveToOverride(x1, y1, z1, yaw1, 0.2, 30.0);
    // collectData();
    // api.moveToOverride(x2, y2, z2, yaw2, 0.2, 30.0);
    // collectData();

    api.holdPosition();
    ros::Duration(3.0).sleep();

    ROS_INFO("[task4] Inspect task done (placeholder).");
}

// ─────────────────────────────────────────
//  任务 5: 预留
// ─────────────────────────────────────────
void task5_reserve(EgoApi& api) {
    ROS_INFO("[task5] === Reserve Task 5 (NOT IMPLEMENTED) ===");

    // TODO: 用户自定义任务

    api.holdPosition();
    ros::Duration(3.0).sleep();

    ROS_INFO("[task5] Task 5 done (placeholder).");
}

// ─────────────────────────────────────────
//  任务 6: 预留
// ─────────────────────────────────────────
void task6_reserve(EgoApi& api) {
    ROS_INFO("[task6] === Reserve Task 6 (NOT IMPLEMENTED) ===");

    // TODO: 用户自定义任务

    api.holdPosition();
    ros::Duration(3.0).sleep();

    ROS_INFO("[task6] Task 6 done (placeholder).");
}
