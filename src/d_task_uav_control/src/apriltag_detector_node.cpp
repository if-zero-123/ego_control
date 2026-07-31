#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <json/json.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include "d_task_uav_control/AprilTagRange.h"
#include "d_task_uav_control/PlatformDetection.h"
#include "d_task_uav_control/apriltag_detector.h"
#include "d_task_uav_control/apriltag_track_filter.h"

namespace d_task_uav_control {
namespace {

constexpr int kVirtualPlatformId = 5;

double xmlNumber(const XmlRpc::XmlRpcValue& value,
                 const std::string& field_name) {
    if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        return static_cast<double>(value);
    }
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        return static_cast<int>(value);
    }
    throw std::runtime_error(
        "apriltag/layout field is not numeric: " + field_name);
}

std::vector<int> layoutIds(const std::vector<AprilTagLayoutEntry>& layout) {
    std::vector<int> ids;
    ids.reserve(layout.size());
    for (const AprilTagLayoutEntry& entry : layout) {
        ids.push_back(entry.id);
    }
    return ids;
}

bool closeRangeState(const std::string& state) {
    static const std::set<std::string> states{
        "FOLLOW_CAR", "DROP_DESCEND", "RELEASE", "DESCEND_HIGH",
        "DESCEND_LOW",
        "LAND_ON_PLATFORM"};
    return states.count(state) != 0U;
}

std::string fixed(double value, int precision) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
}

struct DetectionPass {
    std::vector<AprilTagDetection> detections;
    cv::Rect region;
    std::string region_name = "FULL";
    std::string preprocessing = "GRAY";
};

class AprilTagDetectorNode {
public:
    AprilTagDetectorNode()
        : private_node_("~"), image_transport_(node_) {
        layout_ = loadLayout();

        double min_side_px = 8.0;
        private_node_.param("apriltag/min_side_px", min_side_px, min_side_px);
        detector_.reset(new AprilTagDetector(layoutIds(layout_), min_side_px));
        track_filter_.reset(new AprilTagTrackFilter(loadTrackFilterConfig()));
        range_filter_.reset(new AprilTagRangeFilter(loadRangeFilterConfig()));

        private_node_.param<std::string>(
            "apriltag/image_topic", image_topic_,
            "/usb_camera_vision/usb_cam/image_raw");
        private_node_.param<std::string>(
            "apriltag/detection_topic", detection_topic_,
            "/d_task/vision/platform_detection");
        private_node_.param<std::string>(
            "apriltag/camera_info_topic", camera_info_topic_,
            "/usb_camera_vision/usb_cam/camera_info");
        private_node_.param<std::string>(
            "apriltag/range_topic", range_topic_,
            "/d_task/vision/apriltag_range");
        private_node_.param<std::string>(
            "apriltag/debug_image_topic", debug_image_topic_,
            "/d_task/vision/apriltag_debug");
        private_node_.param<std::string>(
            "apriltag/task_state_topic", task_state_topic_,
            "/uav_protocol/task_state");
        private_node_.param(
            "apriltag/phase_selection_enabled",
            phase_selection_enabled_, true);
        private_node_.param(
            "apriltag/publish_debug_image", publish_debug_image_, true);
        private_node_.param(
            "apriltag/max_reprojection_error_px",
            max_reprojection_error_px_, 3.0);
        private_node_.param(
            "apriltag/full_frame_interval", full_frame_interval_, 10);
        private_node_.param(
            "apriltag/roi_expand_scale", roi_expand_scale_, 1.50);
        private_node_.param(
            "apriltag/roi_min_size_px", roi_min_size_px_, 96);
        private_node_.param(
            "apriltag/clahe_fallback", clahe_fallback_, true);
        private_node_.param(
            "apriltag/contrast_stddev_threshold",
            contrast_stddev_threshold_, 18.0);
        double clahe_clip_limit = 2.0;
        int clahe_grid_size = 8;
        private_node_.param(
            "apriltag/clahe_clip_limit", clahe_clip_limit, 2.0);
        private_node_.param(
            "apriltag/clahe_grid_size", clahe_grid_size, 8);
        if (!std::isfinite(max_reprojection_error_px_)
            || max_reprojection_error_px_ <= 0.0
            || full_frame_interval_ <= 0
            || !std::isfinite(roi_expand_scale_) || roi_expand_scale_ < 1.0
            || roi_min_size_px_ <= 0
            || !std::isfinite(contrast_stddev_threshold_)
            || contrast_stddev_threshold_ <= 0.0
            || !std::isfinite(clahe_clip_limit) || clahe_clip_limit <= 0.0
            || clahe_grid_size <= 0) {
            throw std::runtime_error("invalid AprilTag image-pipeline config");
        }
        clahe_ = cv::createCLAHE(
            clahe_clip_limit, cv::Size(clahe_grid_size, clahe_grid_size));
        close_range_ = !phase_selection_enabled_;
        frames_since_full_ = full_frame_interval_;
        loadFallbackIntrinsics();

        detection_publisher_ = node_.advertise<PlatformDetection>(
            detection_topic_, 1);
        range_publisher_ = node_.advertise<AprilTagRange>(range_topic_, 1);
        if (publish_debug_image_) {
            debug_publisher_ = image_transport_.advertise(
                debug_image_topic_, 1);
        }
        camera_info_subscriber_ = node_.subscribe(
            camera_info_topic_, 1,
            &AprilTagDetectorNode::cameraInfoCallback, this);
        if (phase_selection_enabled_) {
            task_state_subscriber_ = node_.subscribe(
                task_state_topic_, 5,
                &AprilTagDetectorNode::taskStateCallback, this);
        }
        image_subscriber_ = image_transport_.subscribe(
            image_topic_, 1, &AprilTagDetectorNode::imageCallback, this);

        ROS_INFO(
            "[apriltag_detector] family=36h11 layout=0..4 image=%s "
            "output=%s range=%s phase_selection=%s",
            image_topic_.c_str(), detection_topic_.c_str(),
            range_topic_.c_str(), phase_selection_enabled_ ? "on" : "off");
    }

private:
    std::vector<AprilTagLayoutEntry> loadLayout() {
        XmlRpc::XmlRpcValue raw;
        if (!private_node_.getParam("apriltag/layout", raw)
            || raw.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            throw std::runtime_error("apriltag/layout must be an array");
        }
        std::vector<AprilTagLayoutEntry> layout;
        std::set<int> ids;
        for (int index = 0; index < raw.size(); ++index) {
            XmlRpc::XmlRpcValue& value = raw[index];
            if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct
                || !value.hasMember("id") || !value.hasMember("size_m")
                || !value.hasMember("x_m") || !value.hasMember("y_m")
                || !value.hasMember("yaw_rad")) {
                throw std::runtime_error(
                    "each apriltag/layout entry needs id/size_m/x_m/y_m/yaw_rad");
            }
            AprilTagLayoutEntry entry;
            entry.id = static_cast<int>(xmlNumber(value["id"], "id"));
            entry.size_m = xmlNumber(value["size_m"], "size_m");
            entry.x_m = xmlNumber(value["x_m"], "x_m");
            entry.y_m = xmlNumber(value["y_m"], "y_m");
            entry.yaw_rad = xmlNumber(value["yaw_rad"], "yaw_rad");
            if (aprilTagLayoutObjectCorners(entry).size() != 4U
                || !ids.insert(entry.id).second) {
                throw std::runtime_error(
                    "apriltag/layout contains invalid or duplicate entry");
            }
            layout.push_back(entry);
        }
        if (ids != std::set<int>({0, 1, 2, 3, 4})) {
            throw std::runtime_error(
                "apriltag/layout must contain exactly IDs 0,1,2,3,4");
        }
        return layout;
    }

    AprilTagTrackFilterConfig loadTrackFilterConfig() {
        AprilTagTrackFilterConfig config;
        private_node_.param(
            "apriltag/track_filter_alpha", config.filter_alpha,
            config.filter_alpha);
        private_node_.param(
            "apriltag/track_filter_beta", config.filter_beta,
            config.filter_beta);
        private_node_.param(
            "apriltag/prediction_timeout_s", config.prediction_timeout_s,
            config.prediction_timeout_s);
        private_node_.param(
            "apriltag/max_velocity_px_s", config.max_velocity_px_s,
            config.max_velocity_px_s);
        private_node_.param(
            "apriltag/reacquire_distance_px", config.reacquire_distance_px,
            config.reacquire_distance_px);
        return config;
    }

    AprilTagRangeFilterConfig loadRangeFilterConfig() {
        AprilTagRangeFilterConfig config;
        private_node_.param(
            "apriltag/range_filter_alpha", config.alpha, config.alpha);
        private_node_.param(
            "apriltag/range_max_jump_m", config.max_jump_m,
            config.max_jump_m);
        private_node_.param(
            "apriltag/range_filter_reset_timeout_s", config.reset_timeout_s,
            config.reset_timeout_s);
        return config;
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
                5.0, "[apriltag_detector] invalid CameraInfo, using YAML intrinsics");
            return;
        }
        camera_matrix_ = (cv::Mat_<double>(3, 3)
            << message->K[0], message->K[1], message->K[2],
               message->K[3], message->K[4], message->K[5],
               message->K[6], message->K[7], message->K[8]);
        distortion_coefficients_ =
            cv::Mat::zeros(1, std::max<std::size_t>(5U, message->D.size()),
                           CV_64F);
        for (std::size_t index = 0; index < message->D.size(); ++index) {
            if (!std::isfinite(message->D[index])) {
                distortion_coefficients_ = cv::Mat::zeros(1, 5, CV_64F);
                ROS_WARN_THROTTLE(
                    5.0, "[apriltag_detector] invalid distortion, using zeros");
                break;
            }
            distortion_coefficients_.at<double>(0, index) = message->D[index];
        }
        intrinsics_source_ = "camera_info";
    }

    void taskStateCallback(const std_msgs::String::ConstPtr& message) {
        Json::CharReaderBuilder builder;
        std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
        Json::Value root;
        std::string error;
        if (!reader->parse(
                message->data.data(),
                message->data.data() + message->data.size(),
                &root, &error)
            || !root.isObject() || !root["state"].isString()) {
            ROS_WARN_THROTTLE(
                2.0, "[apriltag_detector] invalid task-state JSON");
            return;
        }
        const std::string state = root["state"].asString();
        const bool next_close_range = closeRangeState(state);
        if (next_close_range != close_range_) {
            ROS_INFO(
                "[apriltag_detector] tag policy=%s state=%s",
                next_close_range ? "CENTER+OUTER" : "OUTER_ONLY",
                state.c_str());
        }
        close_range_ = next_close_range;
    }

    DetectionPass detectRegion(
        const cv::Mat& gray, const cv::Rect& region,
        const std::string& region_name) {
        DetectionPass output;
        output.region = region;
        output.region_name = region_name;
        const cv::Mat view = gray(region);
        output.detections = detector_->detectAll(view);
        if (clahe_fallback_
            && (output.detections.empty()
                || imageNeedsContrastEnhancement(
                    view, contrast_stddev_threshold_))) {
            cv::Mat enhanced;
            clahe_->apply(view, enhanced);
            std::vector<AprilTagDetection> enhanced_detections =
                detector_->detectAll(enhanced);
            if (!enhanced_detections.empty() || output.detections.empty()) {
                output.detections = enhanced_detections;
                output.preprocessing = "CLAHE";
            }
        }
        translateAprilTagDetections(region.tl(), output.detections);
        return output;
    }

    DetectionPass detectFrame(const cv::Mat& gray) {
        const cv::Rect full(0, 0, gray.cols, gray.rows);
        const bool periodic_full = frames_since_full_ >= full_frame_interval_ - 1;
        cv::Rect roi;
        if (has_tracking_roi_ && !periodic_full) {
            roi = expandedImageRoi(
                last_tracking_bbox_, roi_expand_scale_, gray.size(),
                roi_min_size_px_);
        }
        if (roi.empty() || roi == full) {
            frames_since_full_ = 0;
            return detectRegion(gray, full, "FULL");
        }

        ++frames_since_full_;
        DetectionPass output = detectRegion(gray, roi, "ROI");
        if (!output.detections.empty()) {
            return output;
        }
        frames_since_full_ = 0;
        output = detectRegion(gray, full, "FULL_REACQUIRE");
        return output;
    }

    AprilTagDetection boardMeasurement(
        const AprilTagBoardEstimate& board) const {
        AprilTagDetection output;
        if (!board.valid) {
            return output;
        }
        output.found = true;
        output.id = kVirtualPlatformId;
        output.confidence = board.confidence;
        output.center = board.center;
        output.bbox = board.bbox;
        return output;
    }

    double tagSizeForId(int id) const {
        for (const AprilTagLayoutEntry& entry : layout_) {
            if (entry.id == id) {
                return entry.size_m;
            }
        }
        return 0.0;
    }

    static std::string measurementSource(
        const AprilTagBoardEstimate& board) {
        if (!board.valid) {
            return "NONE";
        }
        if (board.center_tag_visible) {
            return board.used_tag_ids.size() > 1U
                ? "CENTER+OUTER" : "CENTER";
        }
        return board.used_tag_ids.size() > 1U
            ? "OUTER_FUSED" : "OUTER_SINGLE";
    }

    void publishDetection(
        const std_msgs::Header& header,
        const AprilTagDetection& detection,
        unsigned int image_width,
        unsigned int image_height) {
        PlatformDetection output;
        output.header = header;
        output.found = detection.found;
        output.predicted = detection.predicted;
        output.measurement_age_s = detection.measurement_age_s;
        output.class_id = kVirtualPlatformId;
        output.image_width = image_width;
        output.image_height = image_height;
        if (detection.found) {
            output.confidence = detection.confidence;
            output.center_u = detection.center.x;
            output.center_v = detection.center.y;
            output.bbox_x = detection.bbox.x;
            output.bbox_y = detection.bbox.y;
            output.bbox_width = detection.bbox.width;
            output.bbox_height = detection.bbox.height;
        }
        detection_publisher_.publish(output);
    }

    void publishRange(
        const std_msgs::Header& header,
        const std::vector<AprilTagDetection>& visible,
        const AprilTagBoardEstimate& board,
        bool range_valid,
        double filtered_plane_distance_m,
        double processing_time_ms,
        const DetectionPass& pass) {
        AprilTagRange output;
        output.header = header;
        output.detected = board.valid;
        output.pose_valid = board.valid && range_valid;
        output.center_tag_visible =
            board.valid && board.center_tag_visible;
        output.tag_id = board.used_tag_ids.empty()
            ? -1
            : (board.center_tag_visible ? 0 : board.used_tag_ids.front());
        for (const AprilTagDetection& detection : visible) {
            output.visible_tag_ids.push_back(detection.id);
        }
        output.used_tag_ids = board.used_tag_ids;
        output.intrinsics_source = intrinsics_source_;
        output.measurement_source = measurementSource(board);
        output.tag_size_m = tagSizeForId(output.tag_id);
        output.mean_side_px = board.valid ? board.mean_side_px : 0.0;
        const double invalid = std::numeric_limits<double>::quiet_NaN();
        output.camera_x_m = board.valid ? board.translation_m[0] : invalid;
        output.camera_y_m = board.valid ? board.translation_m[1] : invalid;
        output.optical_axis_distance_m =
            board.valid ? board.optical_axis_distance_m : invalid;
        output.slant_range_m = board.valid ? board.slant_range_m : invalid;
        output.plane_distance_m = range_valid
            ? filtered_plane_distance_m : invalid;
        output.raw_plane_distance_m =
            board.valid ? board.plane_distance_m : invalid;
        output.tag_tilt_deg = board.valid ? board.tag_tilt_deg : invalid;
        output.reprojection_error_px =
            board.valid ? board.reprojection_error_px : invalid;
        output.processing_time_ms = processing_time_ms;
        output.detection_region = pass.region_name;
        output.preprocessing = pass.preprocessing;
        range_publisher_.publish(output);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& message) {
        try {
            const auto started = std::chrono::steady_clock::now();
            const cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(
                message, sensor_msgs::image_encodings::BGR8);
            cv::Mat gray;
            cv::cvtColor(image->image, gray, cv::COLOR_BGR2GRAY);
            DetectionPass pass = detectFrame(gray);
            const std::vector<AprilTagDetection> phase_detections =
                filterPlatformTagDetections(pass.detections, close_range_);
            const AprilTagBoardEstimate board = estimateAprilTagBoardPose(
                phase_detections, layout_, camera_matrix_,
                distortion_coefficients_, max_reprojection_error_px_);
            const double stamp_s = message->header.stamp.isZero()
                ? ros::Time::now().toSec()
                : message->header.stamp.toSec();
            const AprilTagDetection tracked = track_filter_->update(
                boardMeasurement(board), stamp_s,
                message->width, message->height);
            publishDetection(
                message->header, tracked, message->width, message->height);

            if (tracked.found) {
                last_tracking_bbox_ = tracked.bbox;
                has_tracking_roi_ = true;
            } else {
                has_tracking_roi_ = false;
            }

            double filtered_plane_distance_m =
                std::numeric_limits<double>::quiet_NaN();
            const bool range_valid = board.valid
                && range_filter_->update(
                    board.plane_distance_m, stamp_s,
                    filtered_plane_distance_m);
            const auto finished = std::chrono::steady_clock::now();
            const double processing_time_ms =
                std::chrono::duration<double, std::milli>(
                    finished - started).count();
            publishRange(
                message->header, pass.detections, board, range_valid,
                filtered_plane_distance_m, processing_time_ms, pass);
            if (publish_debug_image_) {
                publishDebug(
                    message->header, image->image, pass, board, tracked,
                    range_valid, filtered_plane_distance_m,
                    processing_time_ms);
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
        const DetectionPass& pass,
        const AprilTagBoardEstimate& board,
        const AprilTagDetection& tracked,
        bool range_valid,
        double filtered_plane_distance_m,
        double processing_time_ms) {
        cv::Mat annotated = input.clone();
        if (!pass.detections.empty()) {
            std::vector<std::vector<cv::Point2f>> corners;
            std::vector<int> ids;
            for (const AprilTagDetection& detection : pass.detections) {
                corners.push_back(detection.corners);
                ids.push_back(detection.id);
            }
            cv::aruco::drawDetectedMarkers(annotated, corners, ids);
        }
        if (pass.region_name == "ROI") {
            cv::rectangle(
                annotated, pass.region, cv::Scalar(255, 180, 0), 1,
                cv::LINE_AA);
        }
        if (tracked.found) {
            const cv::Scalar color = tracked.predicted
                ? cv::Scalar(0, 165, 255) : cv::Scalar(0, 255, 0);
            // The green outlines above are the real decoded tag corners.
            // Do not draw the synthetic platform bounding box here: it is an
            // inferred tracking region, not a physical image rectangle.
            if (tracked.predicted) {
                const cv::Point center(
                    cvRound(tracked.center.x), cvRound(tracked.center.y));
                cv::drawMarker(
                    annotated, center, color, cv::MARKER_CROSS, 18, 2,
                    cv::LINE_AA);
            }
        }
        if (board.valid && !tracked.predicted) {
            const cv::Point center(
                cvRound(board.center.x), cvRound(board.center.y));
            cv::drawMarker(
                annotated, center, cv::Scalar(255, 0, 255),
                cv::MARKER_TILTED_CROSS, 20, 2, cv::LINE_AA);
        }
        if (board.valid) {
            cv::drawFrameAxes(
                annotated, camera_matrix_, distortion_coefficients_,
                board.rotation_vector, board.translation_m, 0.10F, 2);
        }

        const std::string ids = board.used_tag_ids.empty()
            ? "none"
            : [&board]() {
                std::ostringstream stream;
                for (std::size_t index = 0;
                     index < board.used_tag_ids.size(); ++index) {
                    if (index > 0U) {
                        stream << ',';
                    }
                    stream << board.used_tag_ids[index];
                }
                return stream.str();
            }();
        const std::string state = tracked.found
            ? (tracked.predicted ? "PREDICT" : "TRACK") : "SEARCH";
        const std::vector<std::string> labels{
            "APRILTAG BOARD " + state + " ids=" + ids,
            pass.region_name + " " + pass.preprocessing
                + " time=" + fixed(processing_time_ms, 1) + "ms",
            range_valid
                ? "plane=" + fixed(filtered_plane_distance_m, 3)
                    + "m raw=" + fixed(board.plane_distance_m, 3) + "m"
                : "plane=N/A",
            board.valid
                ? "center u=" + fixed(board.center.x, 1)
                    + " v=" + fixed(board.center.y, 1)
                    + " reproj=" + fixed(board.reprojection_error_px, 2)
                : "center=N/A",
            std::string("policy=")
                + (close_range_ ? "CENTER+OUTER" : "OUTER_ONLY")
                + " K=" + intrinsics_source_,
        };
        for (std::size_t index = 0; index < labels.size(); ++index) {
            cv::putText(
                annotated, labels[index],
                cv::Point(8, 24 + static_cast<int>(index) * 24),
                cv::FONT_HERSHEY_SIMPLEX, 0.52,
                cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
        }
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
    ros::Subscriber task_state_subscriber_;
    ros::Publisher detection_publisher_;
    ros::Publisher range_publisher_;

    std::vector<AprilTagLayoutEntry> layout_;
    std::unique_ptr<AprilTagDetector> detector_;
    std::unique_ptr<AprilTagTrackFilter> track_filter_;
    std::unique_ptr<AprilTagRangeFilter> range_filter_;
    cv::Ptr<cv::CLAHE> clahe_;
    cv::Mat camera_matrix_;
    cv::Mat distortion_coefficients_;
    std::string intrinsics_source_;
    std::string image_topic_;
    std::string camera_info_topic_;
    std::string detection_topic_;
    std::string range_topic_;
    std::string debug_image_topic_;
    std::string task_state_topic_;

    bool phase_selection_enabled_ = true;
    bool close_range_ = false;
    bool publish_debug_image_ = true;
    bool clahe_fallback_ = true;
    bool has_tracking_roi_ = false;
    int full_frame_interval_ = 10;
    int frames_since_full_ = 10;
    int roi_min_size_px_ = 96;
    double roi_expand_scale_ = 1.50;
    double contrast_stddev_threshold_ = 18.0;
    double max_reprojection_error_px_ = 3.0;
    cv::Rect2f last_tracking_bbox_;
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
