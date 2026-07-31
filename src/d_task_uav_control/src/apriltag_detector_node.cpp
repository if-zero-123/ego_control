#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include "d_task_uav_control/PlatformDetection.h"
#include "d_task_uav_control/apriltag_detector.h"

namespace d_task_uav_control {
namespace {

class AprilTagDetectorNode {
public:
    AprilTagDetectorNode()
        : private_node_("~"),
          image_transport_(node_),
          detector_(loadTargetId(), loadMinSidePx()) {
        private_node_.param<std::string>(
            "apriltag/image_topic", image_topic_,
            "/usb_camera_vision/usb_cam/image_raw");
        private_node_.param<std::string>(
            "apriltag/detection_topic", detection_topic_,
            "/d_task/vision/platform_detection/apriltag");
        private_node_.param<std::string>(
            "apriltag/debug_image_topic", debug_image_topic_,
            "/d_task/vision/apriltag_debug");
        private_node_.param(
            "apriltag/tag_size_m", tag_size_m_, 0.080);
        private_node_.param(
            "apriltag/publish_debug_image", publish_debug_image_, true);
        if (!std::isfinite(tag_size_m_) || tag_size_m_ <= 0.0) {
            throw std::runtime_error("apriltag/tag_size_m must be positive");
        }

        detection_publisher_ = node_.advertise<PlatformDetection>(
            detection_topic_, 1);
        if (publish_debug_image_) {
            debug_publisher_ = image_transport_.advertise(debug_image_topic_, 1);
        }
        image_subscriber_ = image_transport_.subscribe(
            image_topic_, 1, &AprilTagDetectorNode::imageCallback, this);
        ROS_INFO(
            "[apriltag_detector] family=36h11 id=%d size=%.3fm image=%s output=%s",
            target_id_, tag_size_m_, image_topic_.c_str(),
            detection_topic_.c_str());
    }

private:
    int loadTargetId() {
        private_node_.param("apriltag/tag_id", target_id_, 0);
        if (target_id_ < 0) {
            throw std::runtime_error("apriltag/tag_id must be non-negative");
        }
        return target_id_;
    }

    double loadMinSidePx() {
        private_node_.param("apriltag/min_side_px", min_side_px_, 8.0);
        return min_side_px_;
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& message) {
        try {
            const cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(
                message, sensor_msgs::image_encodings::BGR8);
            const AprilTagDetection detection =
                detector_.detect(image->image);

            PlatformDetection output;
            output.header = message->header;
            output.found = detection.found;
            output.class_id = target_id_;
            output.image_width = message->width;
            output.image_height = message->height;
            if (detection.found) {
                output.class_id = detection.id;
                output.confidence = detection.confidence;
                output.center_u = detection.center.x;
                output.center_v = detection.center.y;
                output.bbox_x = detection.bbox.x;
                output.bbox_y = detection.bbox.y;
                output.bbox_width = detection.bbox.width;
                output.bbox_height = detection.bbox.height;
            }
            detection_publisher_.publish(output);

            if (publish_debug_image_) {
                publishDebug(message->header, image->image, detection);
            }
        } catch (const cv_bridge::Exception& error) {
            ROS_WARN_THROTTLE(
                2.0, "[apriltag_detector] cv_bridge error: %s", error.what());
        } catch (const std::exception& error) {
            ROS_WARN_THROTTLE(
                2.0, "[apriltag_detector] frame error: %s", error.what());
        }
    }

    void publishDebug(
        const std_msgs::Header& header,
        const cv::Mat& input,
        const AprilTagDetection& detection) {
        cv::Mat annotated = input.clone();
        if (detection.found) {
            std::vector<std::vector<cv::Point2f>> corners{detection.corners};
            std::vector<int> ids{detection.id};
            cv::aruco::drawDetectedMarkers(annotated, corners, ids);
            cv::circle(annotated, detection.center, 4, cv::Scalar(0, 255, 255), -1);
        }
        const std::string label = detection.found
            ? "APRILTAG 36h11 ID 0"
            : "APRILTAG SEARCH";
        cv::putText(
            annotated, label, cv::Point(8, 24), cv::FONT_HERSHEY_SIMPLEX,
            0.65, detection.found ? cv::Scalar(0, 255, 0)
                                  : cv::Scalar(0, 165, 255),
            2, cv::LINE_AA);
        debug_publisher_.publish(
            cv_bridge::CvImage(
                header, sensor_msgs::image_encodings::BGR8, annotated)
                .toImageMsg());
    }

    ros::NodeHandle node_;
    ros::NodeHandle private_node_;
    image_transport::ImageTransport image_transport_;
    image_transport::Subscriber image_subscriber_;
    image_transport::Publisher debug_publisher_;
    ros::Publisher detection_publisher_;
    int target_id_ = 0;
    double min_side_px_ = 8.0;
    double tag_size_m_ = 0.080;
    bool publish_debug_image_ = true;
    std::string image_topic_;
    std::string detection_topic_;
    std::string debug_image_topic_;
    AprilTagDetector detector_;
};

}  // namespace
}  // namespace d_task_uav_control

int main(int argc, char** argv) {
    ros::init(argc, argv, "apriltag_detector");
    try {
        d_task_uav_control::AprilTagDetectorNode node;
        ros::spin();
    } catch (const std::exception& error) {
        ROS_FATAL("[apriltag_detector] startup failed: %s", error.what());
        return 1;
    }
    return 0;
}
