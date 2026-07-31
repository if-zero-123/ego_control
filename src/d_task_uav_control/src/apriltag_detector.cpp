#include "d_task_uav_control/apriltag_detector.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace d_task_uav_control {
namespace {

cv::Mat validDistortion(const cv::Mat& distortion_coefficients) {
    cv::Mat distortion_64;
    if (distortion_coefficients.empty()) {
        return cv::Mat::zeros(1, 5, CV_64F);
    }
    distortion_coefficients.convertTo(distortion_64, CV_64F);
    return distortion_64;
}

bool validCameraMatrix(const cv::Mat& camera_matrix, cv::Mat& output) {
    if (camera_matrix.rows != 3 || camera_matrix.cols != 3) {
        return false;
    }
    camera_matrix.convertTo(output, CV_64F);
    return std::isfinite(output.at<double>(0, 0))
        && std::isfinite(output.at<double>(1, 1))
        && output.at<double>(0, 0) > 0.0
        && output.at<double>(1, 1) > 0.0;
}

double meanSidePixels(const AprilTagDetection& detection) {
    if (detection.corners.size() != 4U) {
        return 0.0;
    }
    double total = 0.0;
    for (std::size_t index = 0; index < 4U; ++index) {
        total += cv::norm(
            detection.corners[(index + 1U) % 4U] - detection.corners[index]);
    }
    return total * 0.25;
}

}  // namespace

AprilTagDetector::AprilTagDetector(int target_id, double min_side_px)
    : target_id_(target_id),
      target_ids_{target_id},
      min_side_px_(min_side_px),
      dictionary_(cv::aruco::getPredefinedDictionary(
          cv::aruco::DICT_APRILTAG_36h11)),
      parameters_(cv::aruco::DetectorParameters::create()) {
    if (target_id_ < 0 || !std::isfinite(min_side_px_) || min_side_px_ <= 0.0) {
        throw std::invalid_argument(
            "AprilTag target_id must be non-negative and min_side_px positive");
    }
    parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    parameters_->minMarkerPerimeterRate = 0.02;
    parameters_->aprilTagQuadDecimate = 1.0F;
}

AprilTagDetector::AprilTagDetector(const std::vector<int>& target_ids,
                                   double min_side_px)
    : target_id_(target_ids.empty() ? -1 : target_ids.front()),
      target_ids_(target_ids.begin(), target_ids.end()),
      min_side_px_(min_side_px),
      dictionary_(cv::aruco::getPredefinedDictionary(
          cv::aruco::DICT_APRILTAG_36h11)),
      parameters_(cv::aruco::DetectorParameters::create()) {
    if (target_ids.empty() || target_ids_.size() != target_ids.size()
        || *target_ids_.begin() < 0 || !std::isfinite(min_side_px_)
        || min_side_px_ <= 0.0) {
        throw std::invalid_argument(
            "AprilTag target_ids must be unique/non-negative and min_side_px positive");
    }
    parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    parameters_->minMarkerPerimeterRate = 0.02;
    parameters_->aprilTagQuadDecimate = 1.0F;
}

std::vector<AprilTagDetection> AprilTagDetector::detectAll(
    const cv::Mat& image) const {
    std::vector<AprilTagDetection> output;
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
        if (target_ids_.count(ids[index]) == 0U
            || corners[index].size() != 4U) {
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
        AprilTagDetection detection;
        detection.found = true;
        detection.id = ids[index];
        detection.confidence = static_cast<float>(
            std::max(0.0, std::min(1.0, min_side / 40.0)));
        detection.center = center;
        detection.bbox = cv::Rect2f(
            static_cast<float>(bbox.x),
            static_cast<float>(bbox.y),
            static_cast<float>(bbox.width),
            static_cast<float>(bbox.height));
        detection.corners = corners[index];
        output.push_back(detection);
    }
    std::sort(output.begin(), output.end(),
              [](const AprilTagDetection& left,
                 const AprilTagDetection& right) {
                  return left.id < right.id;
              });
    return output;
}

AprilTagDetection AprilTagDetector::detect(const cv::Mat& image) const {
    const std::vector<AprilTagDetection> detections = detectAll(image);
    const auto match = std::find_if(
        detections.begin(), detections.end(),
        [this](const AprilTagDetection& detection) {
            return detection.id == target_id_;
        });
    return match == detections.end() ? AprilTagDetection() : *match;
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
    if (!validCameraMatrix(camera_matrix, camera_matrix_64)) {
        return output;
    }

    const cv::Mat distortion_64 = validDistortion(distortion_coefficients);

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

std::vector<cv::Point3f> aprilTagLayoutObjectCorners(
    const AprilTagLayoutEntry& entry) {
    if (entry.id < 0 || !std::isfinite(entry.size_m)
        || !std::isfinite(entry.x_m) || !std::isfinite(entry.y_m)
        || !std::isfinite(entry.yaw_rad) || entry.size_m <= 0.0) {
        return {};
    }
    const double half = entry.size_m * 0.5;
    const double cosine = std::cos(entry.yaw_rad);
    const double sine = std::sin(entry.yaw_rad);
    const std::vector<cv::Point2d> local{
        {-half, half}, {half, half}, {half, -half}, {-half, -half}};
    std::vector<cv::Point3f> output;
    output.reserve(4U);
    for (const cv::Point2d& point : local) {
        output.emplace_back(
            static_cast<float>(entry.x_m + cosine * point.x - sine * point.y),
            static_cast<float>(entry.y_m + sine * point.x + cosine * point.y),
            0.0F);
    }
    return output;
}

AprilTagBoardEstimate estimateAprilTagBoardPose(
    const std::vector<AprilTagDetection>& detections,
    const std::vector<AprilTagLayoutEntry>& layout,
    const cv::Mat& camera_matrix,
    const cv::Mat& distortion_coefficients,
    double max_reprojection_error_px) {
    AprilTagBoardEstimate output;
    cv::Mat camera_matrix_64;
    if (!validCameraMatrix(camera_matrix, camera_matrix_64)
        || layout.empty() || !std::isfinite(max_reprojection_error_px)
        || max_reprojection_error_px <= 0.0) {
        return output;
    }
    const cv::Mat distortion_64 = validDistortion(distortion_coefficients);

    std::map<int, AprilTagLayoutEntry> entries;
    for (const AprilTagLayoutEntry& entry : layout) {
        if (aprilTagLayoutObjectCorners(entry).size() != 4U
            || !entries.emplace(entry.id, entry).second) {
            return output;
        }
    }

    struct MarkerPoints {
        int id;
        float confidence;
        double mean_side_px;
        std::vector<cv::Point3f> object;
        std::vector<cv::Point2f> image;
    };
    std::vector<MarkerPoints> markers;
    for (const AprilTagDetection& detection : detections) {
        const auto entry = entries.find(detection.id);
        if (!detection.found || detection.predicted
            || detection.corners.size() != 4U || entry == entries.end()) {
            continue;
        }
        markers.push_back({
            detection.id, detection.confidence, meanSidePixels(detection),
            aprilTagLayoutObjectCorners(entry->second), detection.corners});
    }
    if (markers.empty()) {
        return output;
    }

    auto appendPoints = [](const std::vector<MarkerPoints>& source,
                           std::vector<cv::Point3f>& object_points,
                           std::vector<cv::Point2f>& image_points) {
        for (const MarkerPoints& marker : source) {
            object_points.insert(object_points.end(),
                                 marker.object.begin(), marker.object.end());
            image_points.insert(image_points.end(),
                                marker.image.begin(), marker.image.end());
        }
    };

    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;
    appendPoints(markers, object_points, image_points);
    cv::Vec3d rotation;
    cv::Vec3d translation;
    bool solved = false;
    if (markers.size() == 1U) {
        solved = cv::solvePnP(
            object_points, image_points, camera_matrix_64, distortion_64,
            rotation, translation, false, cv::SOLVEPNP_ITERATIVE);
    } else {
        cv::Mat inliers;
        solved = cv::solvePnPRansac(
            object_points, image_points, camera_matrix_64, distortion_64,
            rotation, translation, false, 100,
            static_cast<float>(max_reprojection_error_px), 0.99,
            inliers, cv::SOLVEPNP_ITERATIVE);
    }
    if (!solved || !std::isfinite(translation[0])
        || !std::isfinite(translation[1]) || !std::isfinite(translation[2])
        || translation[2] <= 0.0) {
        return output;
    }

    std::vector<cv::Point2f> initial_projection;
    cv::projectPoints(object_points, rotation, translation,
                      camera_matrix_64, distortion_64, initial_projection);
    std::vector<MarkerPoints> accepted;
    for (std::size_t marker_index = 0; marker_index < markers.size();
         ++marker_index) {
        double squared_error = 0.0;
        for (std::size_t corner = 0; corner < 4U; ++corner) {
            const std::size_t index = marker_index * 4U + corner;
            const cv::Point2f residual =
                initial_projection[index] - image_points[index];
            squared_error += residual.dot(residual);
        }
        if (std::sqrt(squared_error * 0.25)
            <= max_reprojection_error_px) {
            accepted.push_back(markers[marker_index]);
        }
    }
    if (accepted.empty()) {
        return output;
    }

    object_points.clear();
    image_points.clear();
    appendPoints(accepted, object_points, image_points);
    if (!cv::solvePnP(
            object_points, image_points, camera_matrix_64, distortion_64,
            rotation, translation, true, cv::SOLVEPNP_ITERATIVE)
        || translation[2] <= 0.0) {
        return output;
    }

    std::vector<cv::Point2f> reprojected;
    cv::projectPoints(object_points, rotation, translation,
                      camera_matrix_64, distortion_64, reprojected);
    double squared_error_sum = 0.0;
    for (std::size_t index = 0; index < reprojected.size(); ++index) {
        const cv::Point2f residual = reprojected[index] - image_points[index];
        squared_error_sum += residual.dot(residual);
    }
    const double reprojection_error = std::sqrt(
        squared_error_sum / static_cast<double>(reprojected.size()));
    if (!std::isfinite(reprojection_error)
        || reprojection_error > max_reprojection_error_px) {
        return output;
    }

    std::vector<cv::Point2f> projected_center;
    if (accepted.size() == 1U) {
        const std::vector<cv::Point2f> object_plane{
            {accepted.front().object[0].x, accepted.front().object[0].y},
            {accepted.front().object[1].x, accepted.front().object[1].y},
            {accepted.front().object[2].x, accepted.front().object[2].y},
            {accepted.front().object[3].x, accepted.front().object[3].y}};
        std::vector<cv::Point2f> undistorted_image;
        cv::undistortPoints(
            accepted.front().image, undistorted_image,
            camera_matrix_64, distortion_64);
        const cv::Mat homography = cv::findHomography(
            object_plane, undistorted_image, 0);
        if (homography.empty()) {
            return output;
        }
        std::vector<cv::Point2f> normalized_center;
        cv::perspectiveTransform(
            std::vector<cv::Point2f>{{0.0F, 0.0F}}, normalized_center,
            homography);
        if (normalized_center.size() != 1U
            || !std::isfinite(normalized_center.front().x)
            || !std::isfinite(normalized_center.front().y)) {
            return output;
        }
        projected_center.emplace_back(
            static_cast<float>(camera_matrix_64.at<double>(0, 0)
                               * normalized_center.front().x
                               + camera_matrix_64.at<double>(0, 2)),
            static_cast<float>(camera_matrix_64.at<double>(1, 1)
                               * normalized_center.front().y
                               + camera_matrix_64.at<double>(1, 2)));
    } else {
        cv::projectPoints(
            std::vector<cv::Point3f>{cv::Point3f(0.0F, 0.0F, 0.0F)},
            rotation, translation, camera_matrix_64, distortion_64,
            projected_center);
    }
    if (projected_center.size() != 1U) {
        return output;
    }

    cv::Mat rotation_matrix;
    cv::Rodrigues(rotation, rotation_matrix);
    const cv::Vec3d normal(
        rotation_matrix.at<double>(0, 2),
        rotation_matrix.at<double>(1, 2),
        rotation_matrix.at<double>(2, 2));
    const double normal_cosine = std::max(
        0.0, std::min(1.0, std::abs(normal[2])));
    constexpr double kRadiansToDegrees =
        57.295779513082320876798154814105;

    double side_sum = 0.0;
    double confidence_sum = 0.0;
    for (std::size_t index = 0; index < accepted.size(); ++index) {
        side_sum += accepted[index].mean_side_px;
        confidence_sum += accepted[index].confidence;
        output.used_tag_ids.push_back(accepted[index].id);
        output.center_tag_visible = output.center_tag_visible
            || accepted[index].id == 0;
    }
    float half_width = 1.0F;
    float half_height = 1.0F;
    for (const MarkerPoints& marker : accepted) {
        for (const cv::Point2f& corner : marker.image) {
            half_width = std::max(
                half_width, std::abs(corner.x - projected_center.front().x));
            half_height = std::max(
                half_height, std::abs(corner.y - projected_center.front().y));
        }
    }
    std::sort(output.used_tag_ids.begin(), output.used_tag_ids.end());
    output.valid = true;
    output.rotation_vector = rotation;
    output.translation_m = translation;
    output.center = projected_center.front();
    output.bbox = cv::Rect2f(
        output.center.x - half_width, output.center.y - half_height,
        2.0F * half_width, 2.0F * half_height);
    output.confidence = static_cast<float>(std::max(
        0.0, std::min(1.0, confidence_sum / accepted.size())));
    output.mean_side_px = side_sum / accepted.size();
    output.optical_axis_distance_m = translation[2];
    output.slant_range_m = cv::norm(translation);
    output.plane_distance_m = std::abs(normal.dot(translation));
    output.tag_tilt_deg = std::acos(normal_cosine) * kRadiansToDegrees;
    output.reprojection_error_px = reprojection_error;
    return output;
}

std::vector<AprilTagDetection> filterPlatformTagDetections(
    const std::vector<AprilTagDetection>& detections,
    bool close_range) {
    std::vector<AprilTagDetection> output;
    for (const AprilTagDetection& detection : detections) {
        const bool is_platform_id = detection.id >= 0 && detection.id <= 4;
        const bool phase_allowed = close_range || detection.id != 0;
        if (detection.found && is_platform_id && phase_allowed) {
            output.push_back(detection);
        }
    }
    std::sort(output.begin(), output.end(),
              [](const AprilTagDetection& left,
                 const AprilTagDetection& right) {
                  return left.id < right.id;
              });
    return output;
}

cv::Rect expandedImageRoi(const cv::Rect2f& tracked_bbox,
                          double expand_scale,
                          const cv::Size& image_size,
                          int minimum_size_px) {
    if (image_size.width <= 0 || image_size.height <= 0
        || !std::isfinite(expand_scale) || expand_scale < 1.0
        || minimum_size_px <= 0 || tracked_bbox.width <= 0.0F
        || tracked_bbox.height <= 0.0F) {
        return {};
    }
    const double width = std::max(
        static_cast<double>(minimum_size_px),
        static_cast<double>(tracked_bbox.width) * expand_scale);
    const double height = std::max(
        static_cast<double>(minimum_size_px),
        static_cast<double>(tracked_bbox.height) * expand_scale);
    const double center_x = tracked_bbox.x + tracked_bbox.width * 0.5;
    const double center_y = tracked_bbox.y + tracked_bbox.height * 0.5;
    const int left = std::max(0, cvFloor(center_x - width * 0.5));
    const int top = std::max(0, cvFloor(center_y - height * 0.5));
    const int right = std::min(
        image_size.width, cvCeil(center_x + width * 0.5));
    const int bottom = std::min(
        image_size.height, cvCeil(center_y + height * 0.5));
    if (right <= left || bottom <= top) {
        return {};
    }
    return cv::Rect(left, top, right - left, bottom - top);
}

bool imageNeedsContrastEnhancement(const cv::Mat& gray,
                                   double minimum_stddev) {
    if (gray.empty() || gray.channels() != 1
        || !std::isfinite(minimum_stddev) || minimum_stddev <= 0.0) {
        return false;
    }
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(gray, mean, stddev);
    return stddev[0] < minimum_stddev;
}

void translateAprilTagDetections(
    const cv::Point& offset,
    std::vector<AprilTagDetection>& detections) {
    const cv::Point2f delta(
        static_cast<float>(offset.x), static_cast<float>(offset.y));
    for (AprilTagDetection& detection : detections) {
        detection.center += delta;
        detection.bbox.x += delta.x;
        detection.bbox.y += delta.y;
        for (cv::Point2f& corner : detection.corners) {
            corner += delta;
        }
    }
}

}  // namespace d_task_uav_control
