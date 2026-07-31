#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include "d_task_uav_control/AprilTagRange.h"
#include "d_task_uav_control/PlatformDetection.h"
#include "d_task_uav_control/apriltag_detector.h"
#include "d_task_uav_control/apriltag_track_filter.h"

namespace d_task_uav_control {
namespace {

class AprilTagDetectorNode {
public:
    AprilTagDetectorNode()
        : private_node_("~"),
          image_transport_(node_),
          detector_(loadTargetId(), loadMinSidePx()),
          track_filter_(loadTrackFilterConfig()) {
        private_node_.param<std::string>(
            "apriltag/image_topic", image_topic_,
            "/usb_camera_vision/usb_cam/image_raw");
        private_node_.param<std::string>(
            "apriltag/detection_topic", detection_topic_,
            "/d_task/vision/platform_detection/apriltag");
        private_node_.param<std::string>(
            "apriltag/camera_info_topic", camera_info_topic_,
            "/usb_camera_vision/usb_cam/camera_info");
        private_node_.param<std::string>(
            "apriltag/range_topic", range_topic_,
            "/d_task/vision/apriltag_range");
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
        loadFallbackIntrinsics();

        detection_publisher_ = node_.advertise<PlatformDetection>(
            detection_topic_, 1);
        range_publisher_ = node_.advertise<AprilTagRange>(range_topic_, 1);
        if (publish_debug_image_) {
            debug_publisher_ = image_transport_.advertise(debug_image_topic_, 1);
        }
        camera_info_subscriber_ = node_.subscribe(
            camera_info_topic_, 1,
            &AprilTagDetectorNode::cameraInfoCallback, this);
        image_subscriber_ = image_transport_.subscribe(
            image_topic_, 1, &AprilTagDetectorNode::imageCallback, this);
        ROS_INFO(
            "[apriltag_detector] family=36h11 id=%d size=%.3fm image=%s "
            "output=%s range=%s camera_info=%s intrinsics=%s "
            "prediction=%.3fs",
            target_id_, tag_size_m_, image_topic_.c_str(),
            detection_topic_.c_str(), range_topic_.c_str(),
            camera_info_topic_.c_str(), intrinsics_source_.c_str(),
            track_filter_config_.prediction_timeout_s);
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

    AprilTagTrackFilterConfig loadTrackFilterConfig() {
        private_node_.param(
            "apriltag/track_filter_alpha",
            track_filter_config_.filter_alpha,
            track_filter_config_.filter_alpha);
        private_node_.param(
            "apriltag/track_filter_beta",
            track_filter_config_.filter_beta,
            track_filter_config_.filter_beta);
        private_node_.param(
            "apriltag/prediction_timeout_s",
            track_filter_config_.prediction_timeout_s,
            track_filter_config_.prediction_timeout_s);
        private_node_.param(
            "apriltag/max_velocity_px_s",
            track_filter_config_.max_velocity_px_s,
            track_filter_config_.max_velocity_px_s);
        private_node_.param(
            "apriltag/reacquire_distance_px",
            track_filter_config_.reacquire_distance_px,
            track_filter_config_.reacquire_distance_px);
        return track_filter_config_;
    }

    void loadFallbackIntrinsics() {
        double fx = 400.0;
        double fy = 400.0;
        double cx = 320.0;
        double cy = 240.0;
        private_node_.param("camera/fx", fx, fx);
        private_node_.param("camera/fy", fy, fy);
        private_node_.param("camera/cx", cx, cx);
        private_node_.param("camera/cy", cy, cy);
        if (!std::isfinite(fx) || !std::isfinite(fy)
            || !std::isfinite(cx) || !std::isfinite(cy)
            || fx <= 0.0 || fy <= 0.0) {
            throw std::runtime_error(
                "camera fallback intrinsics must be finite with positive fx/fy");
        }
        camera_matrix_ = (cv::Mat_<double>(3, 3)
            << fx, 0.0, cx,
               0.0, fy, cy,
               0.0, 0.0, 1.0);
        distortion_coefficients_ = cv::Mat::zeros(1, 5, CV_64F);
        intrinsics_source_ = "config_fallback";
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& message) {
        if (!std::isfinite(message->K[0]) || !std::isfinite(message->K[4])
            || !std::isfinite(message->K[2]) || !std::isfinite(message->K[5])
            || message->K[0] <= 0.0 || message->K[4] <= 0.0) {
            ROS_WARN_THROTTLE(
                5.0,
                "[apriltag_detector] CameraInfo is uncalibrated; "
                "using YAML fallback intrinsics");
            return;
        }
        camera_matrix_ = (cv::Mat_<double>(3, 3)
            << message->K[0], message->K[1], message->K[2],
               message->K[3], message->K[4], message->K[5],
               message->K[6], message->K[7], message->K[8]);
        distortion_coefficients_ =
            cv::Mat::zeros(1, std::max<std::size_t>(5U, message->D.size()),
                           CV_64F);
        bool distortion_valid = true;
        for (std::size_t index = 0; index < message->D.size(); ++index) {
            if (!std::isfinite(message->D[index])) {
                distortion_valid = false;
                break;
            }
            distortion_coefficients_.at<double>(0, index) = message->D[index];
        }
        if (!distortion_valid) {
            distortion_coefficients_ = cv::Mat::zeros(1, 5, CV_64F);
            ROS_WARN_THROTTLE(
                5.0,
                "[apriltag_detector] CameraInfo distortion is invalid; "
                "using zero distortion");
        }
        intrinsics_source_ = "camera_info";
    }

    static double meanSidePixels(const AprilTagDetection& detection) {
        if (!detection.found) {
            return 0.0;
        }
        if (detection.corners.size() != 4U) {
            return 0.5 * (
                static_cast<double>(detection.bbox.width)
                + static_cast<double>(detection.bbox.height));
        }
        double total = 0.0;
        for (std::size_t index = 0; index < 4U; ++index) {
            total += cv::norm(
                detection.corners[(index + 1U) % 4U]
                - detection.corners[index]);
        }
        return total * 0.25;
    }

    void publishRange(
        const std_msgs::Header& header,
        const AprilTagDetection& detection,
        const AprilTagPoseEstimate& pose) {
        AprilTagRange output;
        output.header = header;
        output.detected = detection.found;
        output.pose_valid = pose.valid;
        output.tag_id = detection.found ? detection.id : target_id_;
        output.intrinsics_source = intrinsics_source_;
        output.tag_size_m = tag_size_m_;
        output.mean_side_px =
            pose.valid ? pose.mean_side_px : meanSidePixels(detection);
        const double invalid = std::numeric_limits<double>::quiet_NaN();
        output.camera_x_m = pose.valid ? pose.translation_m[0] : invalid;
        output.camera_y_m = pose.valid ? pose.translation_m[1] : invalid;
        output.optical_axis_distance_m =
            pose.valid ? pose.optical_axis_distance_m : invalid;
        output.slant_range_m = pose.valid ? pose.slant_range_m : invalid;
        output.plane_distance_m = pose.valid ? pose.plane_distance_m : invalid;
        output.tag_tilt_deg = pose.valid ? pose.tag_tilt_deg : invalid;
        output.reprojection_error_px =
            pose.valid ? pose.reprojection_error_px : invalid;
        range_publisher_.publish(output);
    }

    static std::string fixed(double value, int precision) {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(precision) << value;
        return stream.str();
    }

    static void putDebugLabel(
        cv::Mat& image,
        const std::string& text,
        const cv::Point& preferred_top_left,
        const cv::Scalar& foreground) {
        constexpr double font_scale = 0.48;
        constexpr int thickness = 1;
        constexpr int padding = 3;
        int baseline = 0;
        const cv::Size text_size = cv::getTextSize(
            text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline);
        const int box_width = text_size.width + 2 * padding;
        const int box_height = text_size.height + baseline + 2 * padding;
        const int left = std::max(
            0, std::min(preferred_top_left.x, image.cols - box_width));
        const int top = std::max(
            0, std::min(preferred_top_left.y, image.rows - box_height));
        cv::rectangle(
            image, cv::Rect(left, top, box_width, box_height),
            cv::Scalar(0, 0, 0), cv::FILLED);
        cv::putText(
            image, text,
            cv::Point(left + padding, top + padding + text_size.height),
            cv::FONT_HERSHEY_SIMPLEX, font_scale, foreground,
            thickness, cv::LINE_AA);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& message) {
        try {
            const cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(
                message, sensor_msgs::image_encodings::BGR8);
            const AprilTagDetection raw_detection =
                detector_.detect(image->image);
            const AprilTagPoseEstimate pose = estimateAprilTagPose(
                raw_detection, tag_size_m_, camera_matrix_,
                distortion_coefficients_);
            const double stamp_s = message->header.stamp.isZero()
                ? ros::Time::now().toSec()
                : message->header.stamp.toSec();
            const AprilTagDetection detection = track_filter_.update(
                raw_detection, stamp_s, message->width, message->height);

            PlatformDetection output;
            output.header = message->header;
            output.found = detection.found;
            output.predicted = detection.predicted;
            output.measurement_age_s = detection.measurement_age_s;
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
            publishRange(message->header, raw_detection, pose);

            if (publish_debug_image_) {
                publishDebug(message->header, image->image, detection, pose);
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
        const AprilTagDetection& detection,
        const AprilTagPoseEstimate& pose) {
        cv::Mat annotated = input.clone();
        if (detection.found) {
            const cv::Scalar track_color = detection.predicted
                ? cv::Scalar(0, 165, 255)
                : cv::Scalar(0, 255, 255);
            if (!detection.predicted && detection.corners.size() == 4U) {
                std::vector<std::vector<cv::Point2f>> corners{
                    detection.corners};
                std::vector<int> ids{detection.id};
                cv::aruco::drawDetectedMarkers(annotated, corners, ids);
            } else {
                cv::rectangle(
                    annotated, detection.bbox, track_color, 2, cv::LINE_AA);
            }
            const cv::Point center(
                cvRound(detection.center.x), cvRound(detection.center.y));
            cv::line(
                annotated, center + cv::Point(-10, 0),
                center + cv::Point(10, 0), track_color,
                2, cv::LINE_AA);
            cv::line(
                annotated, center + cv::Point(0, -10),
                center + cv::Point(0, 10), track_color,
                2, cv::LINE_AA);
            cv::circle(
                annotated, center, 4, track_color, -1,
                cv::LINE_AA);

            if (pose.valid && !detection.predicted) {
                cv::drawFrameAxes(
                    annotated, camera_matrix_, distortion_coefficients_,
                    pose.rotation_vector, pose.translation_m,
                    static_cast<float>(tag_size_m_ * 0.5), 2);
            }

            constexpr int label_step = 24;
            constexpr int label_count = 3;
            const int label_stack_height = label_step * label_count;
            int label_top = cvRound(
                detection.bbox.y + detection.bbox.height + 6.0F);
            if (label_top + label_stack_height >= annotated.rows) {
                label_top = cvRound(detection.bbox.y) - label_stack_height - 6;
            }
            const int label_left = cvRound(detection.bbox.x);
            putDebugLabel(
                annotated,
                "pixel u=" + fixed(detection.center.x, 1)
                    + " v=" + fixed(detection.center.y, 1),
                cv::Point(label_left, label_top),
                track_color);
            if (pose.valid && !detection.predicted) {
                putDebugLabel(
                    annotated,
                    "cam X=" + fixed(pose.translation_m[0], 3)
                        + " Y=" + fixed(pose.translation_m[1], 3)
                        + " Z=" + fixed(pose.translation_m[2], 3) + " m",
                    cv::Point(label_left, label_top + label_step),
                    cv::Scalar(0, 255, 0));
                putDebugLabel(
                    annotated,
                    "range=" + fixed(pose.slant_range_m, 3)
                        + " plane=" + fixed(pose.plane_distance_m, 3)
                        + " m",
                    cv::Point(label_left, label_top + 2 * label_step),
                    cv::Scalar(255, 255, 0));
            } else {
                putDebugLabel(
                    annotated, "cam X/Y/Z=N/A",
                    cv::Point(label_left, label_top + label_step),
                    cv::Scalar(0, 165, 255));
                putDebugLabel(
                    annotated, "range/plane=N/A",
                    cv::Point(label_left, label_top + 2 * label_step),
                    cv::Scalar(0, 165, 255));
            }
        }
        const std::string label = !detection.found
            ? "APRILTAG SEARCH"
            : (detection.predicted
                ? "APRILTAG PREDICT ID " + std::to_string(detection.id)
                    + " age="
                    + fixed(detection.measurement_age_s * 1000.0, 0)
                    + "ms conf=" + fixed(detection.confidence, 2)
                : "APRILTAG FILTER ID " + std::to_string(detection.id)
                    + " side=" + fixed(meanSidePixels(detection), 1) + "px");
        const cv::Scalar state_color = detection.predicted
            ? cv::Scalar(0, 165, 255)
            : (detection.found ? cv::Scalar(0, 255, 0)
                               : cv::Scalar(0, 165, 255));
        cv::putText(
            annotated, label, cv::Point(8, 24), cv::FONT_HERSHEY_SIMPLEX,
            0.65, state_color, 2, cv::LINE_AA);
        const std::string range_label =
            pose.valid && !detection.predicted
            ? "z=" + fixed(pose.optical_axis_distance_m, 3)
                + "m range=" + fixed(pose.slant_range_m, 3)
                + "m plane=" + fixed(pose.plane_distance_m, 3) + "m"
            : "distance=N/A";
        cv::putText(
            annotated, range_label, cv::Point(8, 50),
            cv::FONT_HERSHEY_SIMPLEX, 0.58,
            pose.valid && !detection.predicted
                ? cv::Scalar(255, 255, 0)
                : cv::Scalar(0, 165, 255),
            2, cv::LINE_AA);
        const std::string quality_label = detection.predicted
            ? "prediction age="
                + fixed(detection.measurement_age_s * 1000.0, 0)
                + "ms conf=" + fixed(detection.confidence, 2)
                + " K=" + intrinsics_source_
            : (pose.valid
                ? "tilt=" + fixed(pose.tag_tilt_deg, 1)
                    + "deg reproj=" + fixed(
                        pose.reprojection_error_px, 2)
                    + "px K=" + intrinsics_source_
                : "K=" + intrinsics_source_);
        cv::putText(
            annotated, quality_label, cv::Point(8, 76),
            cv::FONT_HERSHEY_SIMPLEX, 0.52,
            cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
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
    ros::Subscriber camera_info_subscriber_;
    ros::Publisher detection_publisher_;
    ros::Publisher range_publisher_;
    int target_id_ = 0;
    double min_side_px_ = 8.0;
    double tag_size_m_ = 0.080;
    bool publish_debug_image_ = true;
    cv::Mat camera_matrix_;
    cv::Mat distortion_coefficients_;
    std::string intrinsics_source_;
    std::string image_topic_;
    std::string camera_info_topic_;
    std::string detection_topic_;
    std::string range_topic_;
    std::string debug_image_topic_;
    AprilTagDetector detector_;
    AprilTagTrackFilterConfig track_filter_config_;
    AprilTagTrackFilter track_filter_;
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
