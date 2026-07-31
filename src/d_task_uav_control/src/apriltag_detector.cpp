#include "d_task_uav_control/apriltag_detector.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

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

}  // namespace d_task_uav_control
