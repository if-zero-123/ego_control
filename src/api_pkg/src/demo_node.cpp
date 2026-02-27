#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include "API_pkg/flight.h" // 确保flight.h已经实现了必要的函数

int flag  = 1;  // 解锁标志位
bool offboard_ok = false;
bool armed_ok = false;

mavros_msgs::State current_state;
nav_msgs::Odometry local_pos;

// 回调函数声明
void state_cb(const mavros_msgs::State::ConstPtr& msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_multi_position");

    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // 20Hz
    ros::Rate rate(20.0);

    // 等待连接
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x =   0;
    pose.pose.position.y =   0;
    pose.pose.position.z =   0.4;

    // 发送数据保持连接
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // 设置offboard模式消息包
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // 设置解锁
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        // 请求进入OFFBOARD模式，每5秒轮询一次
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
                offboard_ok = true;
            }
            last_request = ros::Time::now();
        }
        else
        {
            // 请求解锁
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                    armed_ok = true;
                }
                last_request = ros::Time::now();
            }
        }

        if (offboard_ok && armed_ok)
        {
            break;
        }

        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // 设置目标并移动
    geometry_msgs::PoseStamped target_pose = createTargetPose(0.0, 0.0, 1.0);
    geometry_msgs::PoseStamped target_pose2 = createTargetPose(2.0, 0.0, 1.0);
    geometry_msgs::PoseStamped target_pose3 = createTargetPose(2.0, 2.0, 1.0);
    geometry_msgs::PoseStamped target_pose4 = createTargetPose(0.0, 1.0, 1.0);  // 设置目标位置
    //位移函数
    moveToTarget(local_pos_pub, target_pose);
    // rotateToYaw(local_pos_pub, local_pos_pub, M_PI/2);


    moveToTarget(local_pos_pub, target_pose2);
    // rotateToYaw(local_pos_pub, local_pos_pub, M_PI/2);


    moveToTarget(local_pos_pub, target_pose3);
    // rotateToYaw(local_pos_pub, local_pos_pub, M_PI/2);


    // moveToTarget(local_pos_pub, target_pose4);
    // rotateToYaw(local_pos_pub, local_pos_pub, M_PI/2);


    // // 进入自动降落模式
    // ROS_INFO("AUTO.LAND");
    // offb_set_mode.request.custom_mode = "AUTO.LAND";
    // set_mode_client.call(offb_set_mode);

    return 0;
}

// 获取状态信息
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

// 获取当前位置信息
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    local_pos = *msg;
}
