#include "d_task_uav_control/apriltag_detector.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace d_task_uav_control {

AprilTagDetector::AprilTagDetector(int target_id, double min_side_px)
    : target_id_(target_id),
      min_side_px_(min_side_px),
      dictionary_(cv::aruco::getPredefinedDictionary(
          cv::aruco::DICT_APRILTAG_36h11)),
      parameters_(cv::aruco::DetectorParameters::create()) {
    if (target_id_ < 0 || !std::isfinite(min_side_px_) || min_side_px_ <= 0.0) {
        throw std::invalid_argument(
            "AprilTag target_id must be non-negative and min_side_px positive");
    }
}

AprilTagDetection AprilTagDetector::detect(const cv::Mat& image) const {
    AprilTagDetection output;
    if (image.empty()) {
        return output;
    }

    cv::Mat gray;
    if (image.channels() == 1) {
        gray = image;
    } else if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else if (image.channels() == 4) {
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
    } else {
        return output;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<cv::Point2f>> rejected;
    cv::aruco::detectMarkers(
        gray, dictionary_, corners, ids, parameters_, rejected);

    for (std::size_t index = 0; index < ids.size(); ++index) {
        if (ids[index] != target_id_ || corners[index].size() != 4U) {
            continue;
        }
        double min_side = std::numeric_limits<double>::infinity();
        cv::Point2f center(0.0F, 0.0F);
        for (std::size_t corner = 0; corner < 4U; ++corner) {
            center += corners[index][corner];
            min_side = std::min(
                min_side,
                cv::norm(corners[index][(corner + 1U) % 4U]
                         - corners[index][corner]));
        }
        if (min_side < min_side_px_) {
            continue;
        }

        center *= 0.25F;
        const cv::Rect bbox = cv::boundingRect(corners[index]);
        output.found = true;
        output.id = ids[index];
        output.confidence = static_cast<float>(
            std::max(0.0, std::min(1.0, min_side / 40.0)));
        output.center = center;
        output.bbox = cv::Rect2f(
            static_cast<float>(bbox.x),
            static_cast<float>(bbox.y),
            static_cast<float>(bbox.width),
            static_cast<float>(bbox.height));
        output.corners = corners[index];
        return output;
    }
    return output;
}

AprilTagPoseEstimate estimateAprilTagPose(
    const AprilTagDetection& detection,
    double tag_size_m,
    const cv::Mat& camera_matrix,
    const cv::Mat& distortion_coefficients) {
    AprilTagPoseEstimate output;
    if (!detection.found || detection.predicted
        || detection.corners.size() != 4U
        || !std::isfinite(tag_size_m) || tag_size_m <= 0.0
        || camera_matrix.rows != 3 || camera_matrix.cols != 3) {
        return output;
    }

    cv::Mat camera_matrix_64;
    camera_matrix.convertTo(camera_matrix_64, CV_64F);
    const double fx = camera_matrix_64.at<double>(0, 0);
    const double fy = camera_matrix_64.at<double>(1, 1);
    if (!std::isfinite(fx) || !std::isfinite(fy) || fx <= 0.0 || fy <= 0.0) {
        return output;
    }

    cv::Mat distortion_64;
    if (distortion_coefficients.empty()) {
        distortion_64 = cv::Mat::zeros(1, 5, CV_64F);
    } else {
        distortion_coefficients.convertTo(distortion_64, CV_64F);
    }

    std::vector<std::vector<cv::Point2f>> marker_corners{
        detection.corners};
    std::vector<cv::Vec3d> rotation_vectors;
    std::vector<cv::Vec3d> translation_vectors;
    cv::aruco::estimatePoseSingleMarkers(
        marker_corners, static_cast<float>(tag_size_m),
        camera_matrix_64, distortion_64,
        rotation_vectors, translation_vectors);
    if (rotation_vectors.size() != 1U || translation_vectors.size() != 1U) {
        return output;
    }

    const cv::Vec3d& rotation_vector = rotation_vectors.front();
    const cv::Vec3d& translation = translation_vectors.front();
    if (!std::isfinite(translation[0]) || !std::isfinite(translation[1])
        || !std::isfinite(translation[2]) || translation[2] <= 0.0) {
        return output;
    }

    double side_sum = 0.0;
    for (std::size_t index = 0; index < 4U; ++index) {
        side_sum += cv::norm(
            detection.corners[(index + 1U) % 4U]
            - detection.corners[index]);
    }

    const double half_size = tag_size_m * 0.5;
    const std::vector<cv::Point3f> object_points{
        cv::Point3f(-half_size, half_size, 0.0),
        cv::Point3f(half_size, half_size, 0.0),
        cv::Point3f(half_size, -half_size, 0.0),
        cv::Point3f(-half_size, -half_size, 0.0)};
    std::vector<cv::Point2f> reprojected;
    cv::projectPoints(
        object_points, rotation_vector, translation,
        camera_matrix_64, distortion_64, reprojected);
    double squared_error_sum = 0.0;
    for (std::size_t index = 0; index < 4U; ++index) {
        const cv::Point2f residual =
            reprojected[index] - detection.corners[index];
        squared_error_sum += residual.dot(residual);
    }

    cv::Mat rotation_matrix;
    cv::Rodrigues(rotation_vector, rotation_matrix);
    const cv::Vec3d tag_normal(
        rotation_matrix.at<double>(0, 2),
        rotation_matrix.at<double>(1, 2),
        rotation_matrix.at<double>(2, 2));
    const double normal_cosine = std::max(
        0.0, std::min(1.0, std::abs(tag_normal[2])));
    constexpr double kRadiansToDegrees =
        57.295779513082320876798154814105;

    output.valid = true;
    output.rotation_vector = rotation_vector;
    output.translation_m = translation;
    output.mean_side_px = side_sum * 0.25;
    output.optical_axis_distance_m = translation[2];
    output.slant_range_m = cv::norm(translation);
    output.plane_distance_m = std::abs(tag_normal.dot(translation));
    output.tag_tilt_deg = std::acos(normal_cosine) * kRadiansToDegrees;
    output.reprojection_error_px =
        std::sqrt(squared_error_sum * 0.25);
    return output;
}

}  // namespace d_task_uav_control
