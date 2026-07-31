#include "d_task_uav_control/apriltag_track_filter.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace d_task_uav_control {
namespace {

double clamp(double value, double lower, double upper) {
    return std::max(lower, std::min(upper, value));
}

double finiteOr(double value, double fallback) {
    return std::isfinite(value) ? value : fallback;
}

bool measurementValid(
    const AprilTagDetection& measurement,
    unsigned int image_width,
    unsigned int image_height) {
    return measurement.found
        && !measurement.predicted
        && measurement.id >= 0
        && image_width > 0U
        && image_height > 0U
        && std::isfinite(measurement.center.x)
        && std::isfinite(measurement.center.y)
        && std::isfinite(measurement.bbox.width)
        && std::isfinite(measurement.bbox.height)
        && measurement.center.x >= -static_cast<float>(image_width)
        && measurement.center.y >= -static_cast<float>(image_height)
        && measurement.center.x < 2.0F * static_cast<float>(image_width)
        && measurement.center.y < 2.0F * static_cast<float>(image_height)
        && measurement.bbox.width > 0.0F
        && measurement.bbox.height > 0.0F;
}

}  // namespace

bool aprilTagTrackFilterConfigValid(
    const AprilTagTrackFilterConfig& config) {
    return std::isfinite(config.filter_alpha)
        && std::isfinite(config.filter_beta)
        && std::isfinite(config.prediction_timeout_s)
        && std::isfinite(config.max_velocity_px_s)
        && std::isfinite(config.reacquire_distance_px)
        && config.filter_alpha > 0.0
        && config.filter_alpha <= 1.0
        && config.filter_beta >= 0.0
        && config.filter_beta <= 1.0
        && config.prediction_timeout_s > 0.0
        && config.max_velocity_px_s > 0.0
        && config.reacquire_distance_px > 0.0;
}

AprilTagTrackFilter::AprilTagTrackFilter(
    const AprilTagTrackFilterConfig& config)
    : config_(config) {
    if (!aprilTagTrackFilterConfigValid(config_)) {
        throw std::invalid_argument("invalid AprilTag track-filter config");
    }
    reset();
}

void AprilTagTrackFilter::reset() {
    initialised_ = false;
    id_ = -1;
    center_x_ = 0.0;
    center_y_ = 0.0;
    width_ = 0.0;
    height_ = 0.0;
    center_vx_ = 0.0;
    center_vy_ = 0.0;
    width_rate_ = 0.0;
    height_rate_ = 0.0;
    last_update_s_ = -1.0;
    last_measurement_s_ = -1.0;
    last_confidence_ = 0.0F;
}

void AprilTagTrackFilter::initialise(
    const AprilTagDetection& measurement, double stamp_s) {
    initialised_ = true;
    id_ = measurement.id;
    center_x_ = measurement.center.x;
    center_y_ = measurement.center.y;
    width_ = measurement.bbox.width;
    height_ = measurement.bbox.height;
    center_vx_ = 0.0;
    center_vy_ = 0.0;
    width_rate_ = 0.0;
    height_rate_ = 0.0;
    last_update_s_ = stamp_s;
    last_measurement_s_ = stamp_s;
    last_confidence_ = measurement.confidence;
}

bool AprilTagTrackFilter::predictTo(double stamp_s) {
    if (!initialised_ || !std::isfinite(stamp_s)
        || stamp_s <= last_update_s_) {
        return false;
    }
    const double dt = stamp_s - last_update_s_;
    center_x_ += center_vx_ * dt;
    center_y_ += center_vy_ * dt;
    width_ = std::max(1.0, width_ + width_rate_ * dt);
    height_ = std::max(1.0, height_ + height_rate_ * dt);
    last_update_s_ = stamp_s;
    return true;
}

cv::Rect2f AprilTagTrackFilter::boundedBbox(
    unsigned int image_width, unsigned int image_height) const {
    const double bounded_width = clamp(
        finiteOr(width_, 1.0), 1.0, static_cast<double>(image_width));
    const double bounded_height = clamp(
        finiteOr(height_, 1.0), 1.0, static_cast<double>(image_height));
    const double left = clamp(
        finiteOr(center_x_, 0.0) - bounded_width * 0.5,
        0.0, static_cast<double>(image_width) - bounded_width);
    const double top = clamp(
        finiteOr(center_y_, 0.0) - bounded_height * 0.5,
        0.0, static_cast<double>(image_height) - bounded_height);
    return cv::Rect2f(
        static_cast<float>(left), static_cast<float>(top),
        static_cast<float>(bounded_width),
        static_cast<float>(bounded_height));
}

AprilTagDetection AprilTagTrackFilter::makeMeasuredOutput(
    const AprilTagDetection& measurement,
    unsigned int image_width,
    unsigned int image_height) const {
    AprilTagDetection output = measurement;
    output.predicted = false;
    output.measurement_age_s = 0.0;
    output.center = cv::Point2f(
        static_cast<float>(clamp(
            center_x_, 0.0, static_cast<double>(image_width - 1U))),
        static_cast<float>(clamp(
            center_y_, 0.0, static_cast<double>(image_height - 1U))));
    output.bbox = boundedBbox(image_width, image_height);
    const cv::Point2f shift = output.center - measurement.center;
    for (cv::Point2f& corner : output.corners) {
        corner += shift;
    }
    return output;
}

AprilTagDetection AprilTagTrackFilter::makePredictedOutput(
    double measurement_age_s,
    unsigned int image_width,
    unsigned int image_height) const {
    AprilTagDetection output;
    output.found = true;
    output.predicted = true;
    output.measurement_age_s = measurement_age_s;
    output.id = id_;
    const double remaining = clamp(
        1.0 - measurement_age_s / config_.prediction_timeout_s,
        0.0, 1.0);
    output.confidence = static_cast<float>(
        clamp(last_confidence_ * remaining, 0.0, 1.0));
    output.center = cv::Point2f(
        static_cast<float>(clamp(
            center_x_, 0.0, static_cast<double>(image_width - 1U))),
        static_cast<float>(clamp(
            center_y_, 0.0, static_cast<double>(image_height - 1U))));
    output.bbox = boundedBbox(image_width, image_height);
    return output;
}

AprilTagDetection AprilTagTrackFilter::update(
    const AprilTagDetection& measurement,
    double stamp_s,
    unsigned int image_width,
    unsigned int image_height) {
    AprilTagDetection empty;
    if (!std::isfinite(stamp_s) || image_width == 0U || image_height == 0U) {
        return empty;
    }

    const bool measured = measurementValid(
        measurement, image_width, image_height);
    if (!initialised_) {
        if (!measured) {
            return empty;
        }
        initialise(measurement, stamp_s);
        return makeMeasuredOutput(measurement, image_width, image_height);
    }
    if (stamp_s <= last_update_s_) {
        return empty;
    }

    const double dt = stamp_s - last_update_s_;
    if (!predictTo(stamp_s)) {
        return empty;
    }

    if (measured) {
        const double age_before_measurement = stamp_s - last_measurement_s_;
        const double residual_x = measurement.center.x - center_x_;
        const double residual_y = measurement.center.y - center_y_;
        const double residual_distance = std::hypot(residual_x, residual_y);
        if (measurement.id != id_
            || age_before_measurement > config_.prediction_timeout_s
            || residual_distance > config_.reacquire_distance_px) {
            initialise(measurement, stamp_s);
            return makeMeasuredOutput(
                measurement, image_width, image_height);
        }

        const double residual_width = measurement.bbox.width - width_;
        const double residual_height = measurement.bbox.height - height_;
        center_x_ += config_.filter_alpha * residual_x;
        center_y_ += config_.filter_alpha * residual_y;
        width_ += config_.filter_alpha * residual_width;
        height_ += config_.filter_alpha * residual_height;
        center_vx_ = clamp(
            center_vx_ + config_.filter_beta * residual_x / dt,
            -config_.max_velocity_px_s, config_.max_velocity_px_s);
        center_vy_ = clamp(
            center_vy_ + config_.filter_beta * residual_y / dt,
            -config_.max_velocity_px_s, config_.max_velocity_px_s);
        width_rate_ = clamp(
            width_rate_ + config_.filter_beta * residual_width / dt,
            -config_.max_velocity_px_s, config_.max_velocity_px_s);
        height_rate_ = clamp(
            height_rate_ + config_.filter_beta * residual_height / dt,
            -config_.max_velocity_px_s, config_.max_velocity_px_s);
        last_measurement_s_ = stamp_s;
        last_confidence_ = measurement.confidence;
        return makeMeasuredOutput(
            measurement, image_width, image_height);
    }

    const double measurement_age_s = stamp_s - last_measurement_s_;
    if (measurement_age_s > config_.prediction_timeout_s) {
        reset();
        return empty;
    }
    return makePredictedOutput(
        measurement_age_s, image_width, image_height);
}

AprilTagRangeFilter::AprilTagRangeFilter(
    const AprilTagRangeFilterConfig& config)
    : config_(config) {
    if (!std::isfinite(config_.alpha) || config_.alpha <= 0.0
        || config_.alpha > 1.0 || !std::isfinite(config_.max_jump_m)
        || config_.max_jump_m <= 0.0
        || !std::isfinite(config_.reset_timeout_s)
        || config_.reset_timeout_s <= 0.0) {
        throw std::invalid_argument("invalid AprilTag range-filter config");
    }
    reset();
}

void AprilTagRangeFilter::reset() {
    initialised_ = false;
    value_m_ = 0.0;
    last_measurement_s_ = -1.0;
}

bool AprilTagRangeFilter::update(
    double measurement_m, double stamp_s, double& filtered_m) {
    if (!std::isfinite(measurement_m) || measurement_m <= 0.0
        || !std::isfinite(stamp_s) || stamp_s < 0.0
        || (initialised_ && stamp_s <= last_measurement_s_)) {
        return false;
    }
    if (!initialised_
        || stamp_s - last_measurement_s_ > config_.reset_timeout_s) {
        initialised_ = true;
        value_m_ = measurement_m;
        last_measurement_s_ = stamp_s;
        filtered_m = value_m_;
        return true;
    }
    if (std::abs(measurement_m - value_m_) > config_.max_jump_m) {
        return false;
    }
    value_m_ += config_.alpha * (measurement_m - value_m_);
    last_measurement_s_ = stamp_s;
    filtered_m = value_m_;
    return true;
}

}  // namespace d_task_uav_control
