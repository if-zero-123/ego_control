#include "API_pkg/flight.h"
#include <cmath>

extern nav_msgs::Odometry local_pos;  // 由主程序更新

bool hasReachedTarget(const geometry_msgs::PoseStamped& current,
                      const geometry_msgs::PoseStamped& target,
                      float tolerance) {
    double dx = current.pose.position.x - target.pose.position.x;
    double dy = current.pose.position.y - target.pose.position.y;
    double dz = current.pose.position.z - target.pose.position.z;
    double dist = sqrt(dx*dx + dy*dy + dz*dz);
    return dist < tolerance;
}

#include <geometry_msgs/PoseStamped.h>

geometry_msgs::PoseStamped createTargetPose(float x, float y, float z) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    return pose;
}

bool moveToTarget(ros::Publisher& pub,
                  const geometry_msgs::PoseStamped& input_target,
                  float tolerance,
                  float timeout_sec) {
    ros::Rate rate(20.0);
    ros::Time start_time = ros::Time::now();

    while (ros::ok()) {
        // 当前姿态
        float current_yaw = getCurrentYaw();

        // 复制目标，并保留当前的朝向
        geometry_msgs::PoseStamped target = input_target;
        target.pose.orientation = toQuaternion(current_yaw);

        // 发布带方向的目标点
        pub.publish(target);

        // 获取当前位置
        geometry_msgs::PoseStamped current;
        current.header = local_pos.header;
        current.pose = local_pos.pose.pose;

        if (hasReachedTarget(current, target, tolerance)) {
            ROS_INFO("Reached target (%.2f, %.2f, %.2f)",
                     target.pose.position.x, target.pose.position.y, target.pose.position.z);
            return true;
        }

        if ((ros::Time::now() - start_time).toSec() > timeout_sec) {
            ROS_WARN("Timeout before reaching (%.2f, %.2f, %.2f)",
                     target.pose.position.x, target.pose.position.y, target.pose.position.z);
            return false;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return false;
}




// 提取当前 yaw（单位：弧度）
float getCurrentYaw() {
    tf::Quaternion q(
        local_pos.pose.pose.orientation.x,
        local_pos.pose.pose.orientation.y,
        local_pos.pose.pose.orientation.z,
        local_pos.pose.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

// 将 yaw 转为四元数
geometry_msgs::Quaternion toQuaternion(float yaw_rad) {
    tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, yaw_rad);
    geometry_msgs::Quaternion q_msg;
    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();
    return q_msg;
}


//弧度制单位M_PI

bool rotateToYaw(ros::Publisher& pub,
                 ros::Publisher& pos_pub,
                 float target_yaw_rad,
                 float yaw_tolerance_rad,
                 float timeout_sec) {
    ros::Rate rate(20.0);
    ros::Time start_time = ros::Time::now();

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = local_pos.pose.pose.position.x;  // 保持当前位置
    pose.pose.position.y = local_pos.pose.pose.position.y;
    pose.pose.position.z = local_pos.pose.pose.position.z;
    pose.pose.orientation = toQuaternion(target_yaw_rad);  // 设置目标朝向

    while (ros::ok()) {
        // 发布目标位置和朝向
        pub.publish(pose);
        ros::spinOnce();

        // 获取当前的 yaw 值
        float current_yaw = getCurrentYaw();
        float yaw_error = fabs(angles::shortest_angular_distance(current_yaw, target_yaw_rad));

        // 如果 yaw 误差在容忍范围内，说明已经达到了目标朝向
        if (yaw_error < yaw_tolerance_rad) {
            ROS_INFO("Reached target yaw: %.2f deg", target_yaw_rad * 180.0 / M_PI);
            return true;  // 旋转成功，目标达成
        }

        // 如果超时，返回 false
        if ((ros::Time::now() - start_time).toSec() > timeout_sec) {
            ROS_WARN("Timeout before reaching target yaw.");
            return false;  // 超时，旋转失败
        }

        // 发布当前目标位置，确保飞行器保持当前位置
        geometry_msgs::PoseStamped target_pose = pose;  // 目标位置保持不变
        pos_pub.publish(target_pose);  // 发布位置，保持当前坐标

        rate.sleep();
    }

    return false;  // 如果退出循环，则表示未完成旋转任务
}
