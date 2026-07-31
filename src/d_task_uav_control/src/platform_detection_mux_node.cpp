#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <json/json.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "d_task_uav_control/PlatformDetection.h"
#include "d_task_uav_control/detection_source_selector.h"

namespace d_task_uav_control {
namespace {

struct DetectionCache {
    PlatformDetection message;
    bool received = false;
    double received_s = -1.0;
};

bool parseTaskState(const std::string& raw, std::string& state) {
    Json::CharReaderBuilder builder;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    Json::Value value;
    std::string error;
    if (!reader->parse(
            raw.data(), raw.data() + raw.size(), &value, &error)
        || !value.isObject()
        || !value.isMember("state")
        || !value["state"].isString()) {
        return false;
    }
    state = value["state"].asString();
    return true;
}

class PlatformDetectionMuxNode {
public:
    PlatformDetectionMuxNode() : private_node_("~") {
        private_node_.param(
            "detection_mux/source_timeout_s", source_timeout_s_, 0.35);
        double publish_rate_hz = 30.0;
        private_node_.param(
            "detection_mux/publish_rate_hz", publish_rate_hz, 30.0);
        if (!std::isfinite(source_timeout_s_) || source_timeout_s_ <= 0.0
            || !std::isfinite(publish_rate_hz) || publish_rate_hz <= 0.0) {
            throw std::runtime_error(
                "detection_mux timeout and publish rate must be positive");
        }

        const std::string yolo_topic = private_node_.param<std::string>(
            "detection_mux/yolo_topic",
            "/d_task/vision/platform_detection/yolo");
        const std::string apriltag_topic = private_node_.param<std::string>(
            "detection_mux/apriltag_topic",
            "/d_task/vision/platform_detection/apriltag");
        const std::string output_topic = private_node_.param<std::string>(
            "detection_mux/output_topic",
            "/d_task/vision/platform_detection");
        const std::string task_state_topic = private_node_.param<std::string>(
            "detection_mux/task_state_topic",
            "/uav_protocol/task_state");

        yolo_subscriber_ = node_.subscribe(
            yolo_topic, 2, &PlatformDetectionMuxNode::yoloCallback, this);
        apriltag_subscriber_ = node_.subscribe(
            apriltag_topic, 2,
            &PlatformDetectionMuxNode::apriltagCallback, this);
        task_state_subscriber_ = node_.subscribe(
            task_state_topic, 2,
            &PlatformDetectionMuxNode::taskStateCallback, this);
        publisher_ = node_.advertise<PlatformDetection>(output_topic, 2);
        timer_ = node_.createTimer(
            ros::Duration(1.0 / publish_rate_hz),
            &PlatformDetectionMuxNode::timerCallback, this);
        ROS_INFO(
            "[platform_detection_mux] yolo=%s apriltag=%s output=%s state=%s",
            yolo_topic.c_str(), apriltag_topic.c_str(), output_topic.c_str(),
            task_state_topic.c_str());
    }

private:
    void yoloCallback(const PlatformDetection::ConstPtr& message) {
        yolo_.message = *message;
        yolo_.received = true;
        yolo_.received_s = ros::Time::now().toSec();
    }

    void apriltagCallback(const PlatformDetection::ConstPtr& message) {
        apriltag_.message = *message;
        apriltag_.received = true;
        apriltag_.received_s = ros::Time::now().toSec();
    }

    void taskStateCallback(const std_msgs::String::ConstPtr& message) {
        std::string state;
        if (!parseTaskState(message->data, state)) {
            ROS_WARN_THROTTLE(
                2.0, "[platform_detection_mux] invalid task-state JSON");
            return;
        }
        const bool enabled =
            state == "DROP_DESCEND" || state == "RELEASE";
        if (enabled != apriltag_enabled_) {
            ROS_INFO(
                "[platform_detection_mux] apriltag %s in state=%s",
                enabled ? "enabled" : "disabled", state.c_str());
        }
        apriltag_enabled_ = enabled;
    }

    void timerCallback(const ros::TimerEvent&) {
        const double now_s = ros::Time::now().toSec();
        const DetectionSource selected = selectDetectionSource(
            now_s,
            source_timeout_s_,
            apriltag_enabled_,
            yolo_.received,
            yolo_.message.found,
            yolo_.received_s,
            apriltag_.received,
            apriltag_.message.found,
            apriltag_.received_s);
        if (selected == DetectionSource::NONE) {
            if (selected != last_source_) {
                ROS_WARN("[platform_detection_mux] both detection sources stale");
                last_source_ = selected;
            }
            return;
        }

        const DetectionCache& cache =
            selected == DetectionSource::APRILTAG ? apriltag_ : yolo_;
        publisher_.publish(cache.message);
        if (selected != last_source_) {
            ROS_INFO(
                "[platform_detection_mux] selected source=%s found=%s",
                detectionSourceName(selected),
                cache.message.found ? "true" : "false");
            last_source_ = selected;
        }
    }

    ros::NodeHandle node_;
    ros::NodeHandle private_node_;
    ros::Subscriber yolo_subscriber_;
    ros::Subscriber apriltag_subscriber_;
    ros::Subscriber task_state_subscriber_;
    ros::Publisher publisher_;
    ros::Timer timer_;
    DetectionCache yolo_;
    DetectionCache apriltag_;
    DetectionSource last_source_ = DetectionSource::NONE;
    bool apriltag_enabled_ = false;
    double source_timeout_s_ = 0.35;
};

}  // namespace
}  // namespace d_task_uav_control

int main(int argc, char** argv) {
    ros::init(argc, argv, "platform_detection_mux");
    try {
        d_task_uav_control::PlatformDetectionMuxNode node;
        ros::spin();
    } catch (const std::exception& error) {
        ROS_FATAL(
            "[platform_detection_mux] startup failed: %s", error.what());
        return 1;
    }
    return 0;
}
