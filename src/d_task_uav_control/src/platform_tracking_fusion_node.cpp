#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <json/json.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include "d_task_uav_control/PlatformDetection.h"
#include "d_task_uav_control/PlatformEstimate.h"
#include "d_task_uav_control/pixel_projector.h"
#include "d_task_uav_control/platform_tracker.h"

namespace d_task_uav_control {
namespace {

double stampOrNow(const ros::Time& stamp) {
    return stamp.isZero() ? ros::Time::now().toSec() : stamp.toSec();
}

double yawFromQuaternion(const geometry_msgs::Quaternion& quaternion) {
    return std::atan2(
        2.0 * (quaternion.w * quaternion.z
               + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y
                     + quaternion.z * quaternion.z));
}

const char* filterModeName(FilterMode mode) {
    switch (mode) {
        case FilterMode::MEASURED:
            return "MEASURED";
        case FilterMode::PREDICTED:
            return "PREDICTED";
        case FilterMode::STALE:
            return "STALE";
        case FilterMode::INVALID:
        default:
            return "INVALID";
    }
}

uint8_t filterModeValue(FilterMode mode) {
    return static_cast<uint8_t>(mode);
}

Eigen::Matrix3d matrixFromParam(const std::vector<double>& values) {
    Eigen::Matrix3d result;
    result << values[0], values[1], values[2],
              values[3], values[4], values[5],
              values[6], values[7], values[8];
    return result;
}

Eigen::Vector3d vectorFromParam(const std::vector<double>& values) {
    return Eigen::Vector3d(values[0], values[1], values[2]);
}

class PlatformTrackingFusionNode {
public:
    PlatformTrackingFusionNode() : private_node_("~"), tracker_(loadTrackerConfig()) {
        private_node_.param("tracking/platform_height_m", platform_height_m_, 0.30);
        private_node_.param("tracking/uav_odom_timeout_s", uav_odom_timeout_s_, 0.30);
        private_node_.param("tracking/use_visual_projection",
                            use_visual_projection_, false);
        private_node_.param("tracking/release_max_xy_error_m",
                            release_max_xy_error_m_, 0.15);
        private_node_.param("tracking/release_max_relative_speed_mps",
                            release_max_relative_speed_mps_, 0.20);
        private_node_.param("tracking/release_target_height_m",
                            release_target_height_m_, 0.80);
        private_node_.param("tracking/release_height_tolerance_m",
                            release_height_tolerance_m_, 0.15);
        private_node_.param("tracking/landing_max_xy_error_m",
                            landing_max_xy_error_m_, 0.10);
        private_node_.param("tracking/landing_max_relative_speed_mps",
                            landing_max_relative_speed_mps_, 0.15);
        private_node_.param<std::string>("tracking/world_frame", world_frame_, "map");

        private_node_.param("camera/fx", intrinsics_.fx, 400.0);
        private_node_.param("camera/fy", intrinsics_.fy, 400.0);
        private_node_.param("camera/cx", intrinsics_.cx, 320.0);
        private_node_.param("camera/cy", intrinsics_.cy, 240.0);
        private_node_.param("camera/use_camera_info", use_camera_info_, true);

        std::vector<double> rotation{
            0.0, -1.0, 0.0,
            -1.0, 0.0, 0.0,
            0.0, 0.0, -1.0,
        };
        std::vector<double> translation{0.0, 0.0, 0.0};
        private_node_.getParam("camera/body_r_camera", rotation);
        private_node_.getParam("camera/body_t_camera_m", translation);
        if (rotation.size() != 9U || translation.size() != 3U) {
            throw std::runtime_error(
                "camera/body_r_camera must contain 9 values and "
                "camera/body_t_camera_m must contain 3 values");
        }
        body_r_camera_ = matrixFromParam(rotation);
        body_t_camera_ = vectorFromParam(translation);
        rebuildProjector();

        const std::string detection_topic = private_node_.param<std::string>(
            "topics/platform_detection", "/d_task/vision/platform_detection");
        const std::string camera_info_topic = private_node_.param<std::string>(
            "topics/camera_info", "/usb_camera_vision/usb_cam/camera_info");
        const std::string uav_odom_topic = private_node_.param<std::string>(
            "topics/px4_odom", "/mavros/local_position/odom");
        const std::string car_pose_topic = private_node_.param<std::string>(
            "topics/car_pose", "/uav_protocol/car/pose");
        const std::string mission_config_topic = private_node_.param<std::string>(
            "topics/mission_config", "/uav_protocol/mission_config");

        detection_subscriber_ = node_.subscribe(
            detection_topic, 5, &PlatformTrackingFusionNode::detectionCallback, this);
        camera_info_subscriber_ = node_.subscribe(
            camera_info_topic, 1, &PlatformTrackingFusionNode::cameraInfoCallback, this);
        uav_odom_subscriber_ = node_.subscribe(
            uav_odom_topic, 10, &PlatformTrackingFusionNode::uavOdomCallback, this);
        car_pose_subscriber_ = node_.subscribe(
            car_pose_topic, 10, &PlatformTrackingFusionNode::carPoseCallback, this);
        mission_config_subscriber_ = node_.subscribe(
            mission_config_topic, 2,
            &PlatformTrackingFusionNode::missionConfigCallback, this);

        estimate_publisher_ = node_.advertise<PlatformEstimate>(
            private_node_.param<std::string>(
                "topics/platform_estimate", "/d_task/tracking/platform_estimate"),
            5);
        tracking_publisher_ = node_.advertise<std_msgs::String>(
            private_node_.param<std::string>(
                "topics/local_tracking", "/uav_protocol/local_tracking"),
            5);
        health_publisher_ = node_.advertise<std_msgs::Bool>(
            private_node_.param<std::string>(
                "topics/vision_health", "/d_task/vision/health"),
            1, true);

        double publish_rate_hz = 20.0;
        private_node_.param("tracking/publish_rate_hz", publish_rate_hz, 20.0);
        if (publish_rate_hz <= 0.0 || !std::isfinite(publish_rate_hz)) {
            throw std::runtime_error("tracking/publish_rate_hz must be positive");
        }
        publish_timer_ = node_.createTimer(
            ros::Duration(1.0 / publish_rate_hz),
            &PlatformTrackingFusionNode::publishTimerCallback, this);

        ROS_INFO("[platform_tracking] detection=%s car_pose=%s odom=%s",
                 detection_topic.c_str(), car_pose_topic.c_str(),
                 uav_odom_topic.c_str());
    }

private:
    TrackerConfig loadTrackerConfig() {
        TrackerConfig config;
        private_node_.param("tracking/process_noise", config.process_noise,
                            config.process_noise);
        private_node_.param("tracking/car_position_noise_m",
                            config.car_position_noise,
                            config.car_position_noise);
        private_node_.param("tracking/car_velocity_noise_mps",
                            config.car_velocity_noise,
                            config.car_velocity_noise);
        private_node_.param("tracking/vision_position_noise_m",
                            config.vision_position_noise,
                            config.vision_position_noise);
        private_node_.param("tracking/max_visual_residual_m",
                            config.max_visual_residual_m,
                            config.max_visual_residual_m);
        private_node_.param("tracking/source_timeout_s",
                            config.source_timeout_s,
                            config.source_timeout_s);
        private_node_.param("tracking/prediction_timeout_s",
                            config.prediction_timeout_s,
                            config.prediction_timeout_s);
        private_node_.param("tracking/car_frame_offset_x_m",
                            config.frame_offset_x_m,
                            config.frame_offset_x_m);
        private_node_.param("tracking/car_frame_offset_y_m",
                            config.frame_offset_y_m,
                            config.frame_offset_y_m);
        private_node_.param("tracking/car_frame_yaw_offset_rad",
                            config.frame_yaw_offset_rad,
                            config.frame_yaw_offset_rad);
        private_node_.param("tracking/platform_offset_body_x_m",
                            config.platform_offset_body_x_m,
                            config.platform_offset_body_x_m);
        private_node_.param("tracking/platform_offset_body_y_m",
                            config.platform_offset_body_y_m,
                            config.platform_offset_body_y_m);
        return config;
    }

    void rebuildProjector() {
        projector_.reset(new PixelProjector(
            intrinsics_, body_r_camera_, body_t_camera_));
    }

    void missionConfigCallback(const std_msgs::String::ConstPtr& message) {
        if (message->data.empty()) {
            return;
        }
        tracker_.reset();
        last_vision_accepted_s_ = -1.0;
        last_detection_received_s_ = -1.0;
        last_detection_found_ = false;
        ROS_INFO("[platform_tracking] tracker reset for mission configuration");
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& message) {
        if (!use_camera_info_ || message->K[0] <= 0.0 || message->K[4] <= 0.0) {
            return;
        }
        intrinsics_.fx = message->K[0];
        intrinsics_.fy = message->K[4];
        intrinsics_.cx = message->K[2];
        intrinsics_.cy = message->K[5];
        rebuildProjector();
        use_camera_info_ = false;
        ROS_INFO("[platform_tracking] camera intrinsics loaded from CameraInfo");
    }

    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& message) {
        latest_uav_odom_ = *message;
        has_uav_odom_ = true;
        last_uav_odom_received_s_ = ros::Time::now().toSec();
    }

    void carPoseCallback(const nav_msgs::Odometry::ConstPtr& message) {
        CarMeasurement measurement;
        measurement.stamp_s = stampOrNow(message->header.stamp);
        measurement.x = message->pose.pose.position.x;
        measurement.y = message->pose.pose.position.y;
        measurement.yaw = yawFromQuaternion(message->pose.pose.orientation);
        measurement.vx = message->twist.twist.linear.x;
        measurement.vy = message->twist.twist.linear.y;
        measurement.confidence = 1.0;
        tracker_.updateCar(measurement);
    }

    void detectionCallback(const PlatformDetection::ConstPtr& message) {
        const double now_s = ros::Time::now().toSec();
        last_detection_received_s_ = now_s;
        last_detection_found_ = message->found;
        last_center_u_ = message->center_u;
        last_center_v_ = message->center_v;
        if (!message->found || !use_visual_projection_
            || !has_uav_odom_ || !projector_) {
            return;
        }
        if (now_s - last_uav_odom_received_s_ > uav_odom_timeout_s_) {
            ROS_WARN_THROTTLE(1.0,
                              "[platform_tracking] cannot project: UAV odom stale");
            return;
        }

        const geometry_msgs::Point& position = latest_uav_odom_.pose.pose.position;
        const geometry_msgs::Quaternion& orientation =
            latest_uav_odom_.pose.pose.orientation;
        const Eigen::Vector3d world_t_body(position.x, position.y, position.z);
        const Eigen::Quaterniond world_q_body(
            orientation.w, orientation.x, orientation.y, orientation.z);
        Eigen::Vector3d projected;
        if (!projector_->project(message->center_u, message->center_v,
                                 world_t_body, world_q_body,
                                 platform_height_m_, projected)) {
            ROS_WARN_THROTTLE(1.0,
                              "[platform_tracking] pixel ray does not intersect platform plane");
            return;
        }

        VisionMeasurement measurement;
        measurement.stamp_s = stampOrNow(message->header.stamp);
        measurement.x = projected.x();
        measurement.y = projected.y();
        measurement.confidence = message->confidence;
        if (tracker_.updateVision(measurement)) {
            last_vision_accepted_s_ = now_s;
        } else {
            ROS_WARN_THROTTLE(1.0,
                              "[platform_tracking] rejected visual position outlier");
        }
    }

    void publishTimerCallback(const ros::TimerEvent&) {
        const double now_s = ros::Time::now().toSec();
        const PlatformState state = tracker_.stateAt(now_s);
        const bool odom_fresh = has_uav_odom_
            && now_s - last_uav_odom_received_s_ <= uav_odom_timeout_s_;
        const bool vision_detected = last_detection_found_
            && last_detection_received_s_ >= 0.0
            && now_s - last_detection_received_s_ <= sourceTimeout();

        PlatformEstimate estimate;
        estimate.header.stamp = ros::Time::now();
        estimate.header.frame_id = world_frame_;
        estimate.valid = state.valid && odom_fresh;
        estimate.filter_mode = filterModeValue(state.mode);
        estimate.x = state.x;
        estimate.y = state.y;
        estimate.z = platform_height_m_;
        estimate.vx = state.vx;
        estimate.vy = state.vy;
        estimate.confidence = state.confidence;
        estimate.measurement_age_ms = static_cast<uint32_t>(std::min(
            state.measurement_age_s * 1000.0,
            static_cast<double>(std::numeric_limits<uint32_t>::max())));
        estimate.vision_detected = vision_detected;
        estimate.center_u = last_center_u_;
        estimate.center_v = last_center_v_;
        estimate_publisher_.publish(estimate);

        const double relative_x = state.x - latest_uav_odom_.pose.pose.position.x;
        const double relative_y = state.y - latest_uav_odom_.pose.pose.position.y;
        const double relative_z =
            platform_height_m_ - latest_uav_odom_.pose.pose.position.z;
        const double relative_vx =
            state.vx - latest_uav_odom_.twist.twist.linear.x;
        const double relative_vy =
            state.vy - latest_uav_odom_.twist.twist.linear.y;
        const double xy_error = std::hypot(relative_x, relative_y);
        const double relative_speed = std::hypot(relative_vx, relative_vy);
        const double height_above_platform = -relative_z;

        const bool release_gate = estimate.valid
            && xy_error <= release_max_xy_error_m_
            && relative_speed <= release_max_relative_speed_mps_
            && std::abs(height_above_platform - release_target_height_m_)
                <= release_height_tolerance_m_;
        const bool landing_gate = estimate.valid
            && xy_error <= landing_max_xy_error_m_
            && relative_speed <= landing_max_relative_speed_mps_;

        Json::Value tracking;
        tracking["track_state"] = !estimate.valid
            ? "INVALID"
            : (vision_detected ? "LOCKED" : "PREDICTED");
        tracking["detected"] = vision_detected;
        tracking["confidence"] = state.confidence;
        tracking["pixel_center"]["u"] = last_center_u_;
        tracking["pixel_center"]["v"] = last_center_v_;
        tracking["relative_position"]["x"] = relative_x;
        tracking["relative_position"]["y"] = relative_y;
        tracking["relative_position"]["z"] = relative_z;
        tracking["relative_velocity"]["x"] = relative_vx;
        tracking["relative_velocity"]["y"] = relative_vy;
        tracking["vision_age_ms"] = visionAgeMs(now_s);
        tracking["filter_mode"] = filterModeName(state.mode);
        tracking["release_gate"] = release_gate;
        tracking["landing_gate"] = landing_gate;
        Json::StreamWriterBuilder writer;
        writer["indentation"] = "";
        std_msgs::String raw_tracking;
        raw_tracking.data = Json::writeString(writer, tracking);
        tracking_publisher_.publish(raw_tracking);

        std_msgs::Bool health;
        health.data = odom_fresh && last_detection_received_s_ >= 0.0
            && now_s - last_detection_received_s_ <= 1.0;
        health_publisher_.publish(health);
    }

    double sourceTimeout() const {
        double value = 0.30;
        private_node_.getParamCached("tracking/source_timeout_s", value);
        return value;
    }

    Json::UInt64 visionAgeMs(double now_s) const {
        if (last_vision_accepted_s_ < 0.0) {
            return std::numeric_limits<uint32_t>::max();
        }
        return static_cast<Json::UInt64>(std::max(
            0.0, (now_s - last_vision_accepted_s_) * 1000.0));
    }

    ros::NodeHandle node_;
    ros::NodeHandle private_node_;
    PlatformTracker tracker_;
    CameraIntrinsics intrinsics_;
    Eigen::Matrix3d body_r_camera_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d body_t_camera_ = Eigen::Vector3d::Zero();
    std::unique_ptr<PixelProjector> projector_;

    ros::Subscriber detection_subscriber_;
    ros::Subscriber camera_info_subscriber_;
    ros::Subscriber uav_odom_subscriber_;
    ros::Subscriber car_pose_subscriber_;
    ros::Subscriber mission_config_subscriber_;
    ros::Publisher estimate_publisher_;
    ros::Publisher tracking_publisher_;
    ros::Publisher health_publisher_;
    ros::Timer publish_timer_;

    nav_msgs::Odometry latest_uav_odom_;
    bool has_uav_odom_ = false;
    bool use_camera_info_ = true;
    bool use_visual_projection_ = false;
    bool last_detection_found_ = false;
    double last_uav_odom_received_s_ = -1.0;
    double last_detection_received_s_ = -1.0;
    double last_vision_accepted_s_ = -1.0;
    float last_center_u_ = 0.0F;
    float last_center_v_ = 0.0F;

    double platform_height_m_ = 0.30;
    double uav_odom_timeout_s_ = 0.30;
    double release_max_xy_error_m_ = 0.15;
    double release_max_relative_speed_mps_ = 0.20;
    double release_target_height_m_ = 0.80;
    double release_height_tolerance_m_ = 0.15;
    double landing_max_xy_error_m_ = 0.10;
    double landing_max_relative_speed_mps_ = 0.15;
    std::string world_frame_ = "map";
};

}  // namespace
}  // namespace d_task_uav_control

int main(int argc, char** argv) {
    ros::init(argc, argv, "platform_tracking_fusion");
    try {
        d_task_uav_control::PlatformTrackingFusionNode node;
        ros::spin();
    } catch (const std::exception& error) {
        ROS_FATAL("[platform_tracking] startup failed: %s", error.what());
        return 1;
    }
    return 0;
}
