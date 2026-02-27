#ifndef FLIGHT_H
#define FLIGHT_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>  // tf 库的头文件
#include <angles/angles.h>  // 引入 angles 库

extern nav_msgs::Odometry local_pos;  // 外部声明

// 生成目标点
geometry_msgs::PoseStamped createTargetPose(float x, float y, float z);

// 判断是否到达目标
bool hasReachedTarget(const geometry_msgs::PoseStamped& current,
                      const geometry_msgs::PoseStamped& target,
                      float tolerance);

// 发布并等待到达
bool moveToTarget(ros::Publisher& pub,
                  const geometry_msgs::PoseStamped& target,
                  float tolerance=0.1,
                  float timeout_sec=10);

bool rotateToYaw(ros::Publisher& pub,
                 ros::Publisher& pos_pub,
                 float target_yaw_rad,
                 float yaw_tolerance_rad=0.05,
                 float timeout_sec=10);


float getCurrentYaw();



geometry_msgs::Quaternion toQuaternion(float yaw_rad);

#endif