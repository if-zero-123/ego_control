//包含ROS和MAVROS相关头文件 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>

//定义变量，用于接收无人机的状态信息
mavros_msgs::State current_state;
//定义变量，用于接收无人机的里程计信息
nav_msgs::Odometry local_pos;

//回调函数，获取无人机的里程计信息
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg);
//回调函数，获取无人机的实时状态信息
void state_cb(const mavros_msgs::State::ConstPtr& msg);

//回调函数接收无人机的状态信息
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

//回调函数接收无人机的里程计信息
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    local_pos = *msg;
}

int main(int argc, char **argv)
{
	//防止中文乱码
	setlocale(LC_ALL, "");

    //ROS节点初始化，节点名为offboard_single_position
    ros::init(argc, argv, "offboard_single_position");

    //创建节点句柄
    ros::NodeHandle nh;

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    //创建一个Subscriber订阅者，订阅名为/mavros/state的topic，注册回调函数state_cb
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
   
    //创建一个Subscriber订阅者，订阅名为/mavros/local_position/odom的topic，注册回调函数local_pos_cb
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

    //创建一个服务客户端，连接名为/mavros/cmd/arming的服务，用于请求无人机解锁
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    //创建一个服务客户端，连接名为/mavros/set_mode的服务，用于请求无人机进入offboard模式
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
    ros::Rate rate(20.0);

    //等待连接到飞控
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    //设置无人机的期望位置
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x =    0;
    pose.pose.position.y =    0;
    pose.pose.position.z =    0.4;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    //定义客户端变量，设置为offboard模式
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    
    //定义客户端变量，请求无人机解锁
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    //记录当前时间，并赋值给变量last_request
    ros::Time last_request = ros::Time::now();
 
// 修改主循环如下：
    while(ros::ok()) {
        // 发布期望位置（必须持续发布，否则飞控会退出Offboard模式）
        local_pos_pub.publish(pose);
        
        // 请求进入OFFBOARD模式（只尝试一次或条件改变时尝试）
        if(!current_state.armed && current_state.mode != "OFFBOARD" && 
        (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 
        // 在OFFBOARD模式下请求解锁
        else if(current_state.mode == "OFFBOARD" && !current_state.armed && 
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if(arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
        
        // 如果已解锁且处于Offboard模式，则跳出循环执行位置控制
        if(current_state.armed && current_state.mode == "OFFBOARD") {
            break;
        }
        
        ros::spinOnce();
        rate.sleep();
    } 
    while(ros::ok())
    {
        pose.pose.position.x =  0;
    	pose.pose.position.y =  0;
    	pose.pose.position.z =  0.4;                        
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}


