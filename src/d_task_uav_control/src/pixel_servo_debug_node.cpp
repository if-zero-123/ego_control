#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/TwistStamped.h>
#include <json/json.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "d_task_uav_control/PlatformDetection.h"
#include "d_task_uav_control/pixel_servo.h"

namespace d_task_uav_control {
namespace {

double stampOrNow(const ros::Time& stamp) {
    return stamp.isZero() ? ros::Time::now().toSec() : stamp.toSec();
}

double yawFromQuaternion(const geometry_msgs::Quaternion& quaternion) {
    return std::atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
}

class PixelServoDebugNode {
public:
    PixelServoDebugNode()
        : private_node_("~"), config_(loadConfig()), servo_(config_) {
        const std::string detection_topic = private_node_.param<std::string>(
            "topics/platform_detection", "/d_task/vision/platform_detection");
        const std::string odom_topic = private_node_.param<std::string>(
            "topics/px4_odom", "/mavros/local_position/odom");
        detection_subscriber_ = node_.subscribe(
            detection_topic, 10, &PixelServoDebugNode::detectionCallback, this);
        odom_subscriber_ = node_.subscribe(
            odom_topic, 10, &PixelServoDebugNode::odomCallback, this);
        body_publisher_ = node_.advertise<geometry_msgs::TwistStamped>(
            private_node_.param<std::string>(
                "topics/pixel_velocity_body_debug",
                "/d_task/vision/pixel_velocity_body_debug"), 5);
        world_publisher_ = node_.advertise<geometry_msgs::TwistStamped>(
            private_node_.param<std::string>(
                "topics/pixel_velocity_world_debug",
                "/d_task/vision/pixel_velocity_world_debug"), 5);
        status_publisher_ = node_.advertise<std_msgs::String>(
            private_node_.param<std::string>(
                "topics/pixel_servo_debug", "/d_task/vision/pixel_servo_debug"),
            5);
        double publish_rate_hz = 20.0;
        private_node_.param("pixel_servo/debug_publish_rate_hz",
                            publish_rate_hz, publish_rate_hz);
        if (publish_rate_hz <= 0.0) {
            throw std::runtime_error("pixel_servo/debug_publish_rate_hz must be positive");
        }
        timer_ = node_.createTimer(ros::Duration(1.0 / publish_rate_hz),
                                   &PixelServoDebugNode::timerCallback, this);
        ROS_INFO("[pixel_servo_debug] detection=%s odom=%s; no flight command output",
                 detection_topic.c_str(), odom_topic.c_str());
    }

private:
    PixelServoConfig loadConfig() {
        PixelServoConfig config;
#define LOAD_PIXEL_PARAM(name, field) \
        private_node_.param("pixel_servo/" name, config.field, config.field)
        LOAD_PIXEL_PARAM("filter_alpha", filter_alpha);
        LOAD_PIXEL_PARAM("filter_beta", filter_beta);
        LOAD_PIXEL_PARAM("source_timeout_s", source_timeout_s);
        LOAD_PIXEL_PARAM("min_confidence", min_confidence);
        LOAD_PIXEL_PARAM("deadband", deadband);
        LOAD_PIXEL_PARAM("gain_mps", gain_mps);
        LOAD_PIXEL_PARAM("max_speed_mps", max_speed_mps);
        LOAD_PIXEL_PARAM("body_x_from_u", body_x_from_u);
        LOAD_PIXEL_PARAM("body_x_from_v", body_x_from_v);
        LOAD_PIXEL_PARAM("body_y_from_u", body_y_from_u);
        LOAD_PIXEL_PARAM("body_y_from_v", body_y_from_v);
#undef LOAD_PIXEL_PARAM
        if (!pixelServoConfigValid(config)) {
            throw std::runtime_error("invalid pixel_servo configuration");
        }
        return config;
    }

    void detectionCallback(const PlatformDetection::ConstPtr& message) {
        if (!message->found) {
            return;
        }
        servo_.update(PixelMeasurement{
            stampOrNow(message->header.stamp), message->center_u, message->center_v,
            message->image_width, message->image_height, message->confidence});
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& message) {
        yaw_rad_ = yawFromQuaternion(message->pose.pose.orientation);
        has_odom_ = true;
    }

    void timerCallback(const ros::TimerEvent&) {
        const PixelServoState state = servo_.stateAt(ros::Time::now().toSec(), yaw_rad_);
        const ros::Time stamp = ros::Time::now();
        geometry_msgs::TwistStamped body;
        body.header.stamp = stamp;
        body.header.frame_id = "base_link";
        body.twist.linear.x = state.body_vx;
        body.twist.linear.y = state.body_vy;
        body_publisher_.publish(body);
        geometry_msgs::TwistStamped world;
        world.header.stamp = stamp;
        world.header.frame_id = "map";
        world.twist.linear.x = has_odom_ ? state.world_vx : 0.0;
        world.twist.linear.y = has_odom_ ? state.world_vy : 0.0;
        world_publisher_.publish(world);

        Json::Value debug;
        debug["valid"] = state.valid;
        debug["yaw_valid"] = has_odom_;
        debug["error"]["u"] = state.error_u;
        debug["error"]["v"] = state.error_v;
        debug["body_velocity"]["x"] = state.body_vx;
        debug["body_velocity"]["y"] = state.body_vy;
        debug["world_velocity"]["x"] = world.twist.linear.x;
        debug["world_velocity"]["y"] = world.twist.linear.y;
        debug["measurement_age_ms"] = state.measurement_age_s * 1000.0;
        Json::StreamWriterBuilder writer;
        writer["indentation"] = "";
        std_msgs::String status;
        status.data = Json::writeString(writer, debug);
        status_publisher_.publish(status);
    }

    ros::NodeHandle node_;
    ros::NodeHandle private_node_;
    PixelServoConfig config_;
    PixelServo servo_;
    ros::Subscriber detection_subscriber_;
    ros::Subscriber odom_subscriber_;
    ros::Publisher body_publisher_;
    ros::Publisher world_publisher_;
    ros::Publisher status_publisher_;
    ros::Timer timer_;
    bool has_odom_ = false;
    double yaw_rad_ = 0.0;
};

}  // namespace
}  // namespace d_task_uav_control

int main(int argc, char** argv) {
    ros::init(argc, argv, "pixel_servo_debug");
    try {
        d_task_uav_control::PixelServoDebugNode node;
        ros::spin();
    } catch (const std::exception& error) {
        ROS_FATAL("[pixel_servo_debug] startup failed: %s", error.what());
        return 1;
    }
    return 0;
}
