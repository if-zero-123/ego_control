#include "d_task_uav_control/platform_tracker.h"

#include <algorithm>
#include <cmath>

namespace d_task_uav_control {

namespace {

double safeVariance(double base_noise, double confidence) {
    const double quality = std::max(0.1, std::min(1.0, confidence));
    const double sigma = base_noise / quality;
    return sigma * sigma;
}

Eigen::Matrix2d yawRotation(double yaw) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    Eigen::Matrix2d rotation;
    rotation << c, -s, s, c;
    return rotation;
}

}  // namespace

PlatformTracker::PlatformTracker(const TrackerConfig& config) : config_(config) {
    reset();
}

void PlatformTracker::reset() {
    initialised_ = false;
    state_.setZero();
    covariance_.setIdentity();
    covariance_ *= 1.0;
    filter_stamp_s_ = 0.0;
    last_car_stamp_s_ = -1.0;
    last_vision_stamp_s_ = -1.0;
    last_measurement_stamp_s_ = -1.0;
    last_confidence_ = 0.0;
    has_car_ = false;
    has_visual_offset_ = false;
    visual_offset_body_.setZero();
}

void PlatformTracker::initialise(double stamp_s, double x, double y,
                                 double vx, double vy) {
    state_ << x, y, vx, vy;
    covariance_.setIdentity();
    covariance_ *= 0.25;
    filter_stamp_s_ = stamp_s;
    initialised_ = true;
}

void PlatformTracker::predictTo(double stamp_s) {
    if (!initialised_ || stamp_s <= filter_stamp_s_) {
        return;
    }
    const double dt = std::min(1.0, stamp_s - filter_stamp_s_);
    Eigen::Matrix4d transition = Eigen::Matrix4d::Identity();
    transition(0, 2) = dt;
    transition(1, 3) = dt;
    const double q = config_.process_noise * config_.process_noise;
    Eigen::Matrix4d process = Eigen::Matrix4d::Zero();
    process(0, 0) = 0.25 * dt * dt * dt * dt * q;
    process(1, 1) = process(0, 0);
    process(0, 2) = 0.5 * dt * dt * dt * q;
    process(2, 0) = process(0, 2);
    process(1, 3) = process(0, 2);
    process(3, 1) = process(0, 2);
    process(2, 2) = dt * dt * q;
    process(3, 3) = process(2, 2);
    state_ = transition * state_;
    covariance_ = transition * covariance_ * transition.transpose() + process;
    filter_stamp_s_ = stamp_s;
}

Eigen::Vector2d PlatformTracker::carPlatformPosition(
    const CarMeasurement& measurement) const {
    const Eigen::Vector2d car_world(
        measurement.x + config_.frame_offset_x_m,
        measurement.y + config_.frame_offset_y_m);
    const Eigen::Vector2d offset = has_visual_offset_
        ? visual_offset_body_
        : Eigen::Vector2d(config_.platform_offset_body_x_m,
                          config_.platform_offset_body_y_m);
    return car_world + yawRotation(measurement.yaw) * offset;
}

void PlatformTracker::updateCar(const CarMeasurement& measurement) {
    const Eigen::Vector2d position = carPlatformPosition(measurement);
    if (!initialised_) {
        initialise(measurement.stamp_s, position.x(), position.y(),
                   measurement.vx, measurement.vy);
    } else {
        predictTo(measurement.stamp_s);
        updateFull(
            position.x(), position.y(), measurement.vx, measurement.vy,
            safeVariance(config_.car_position_noise, measurement.confidence),
            safeVariance(config_.car_velocity_noise, measurement.confidence));
    }
    has_car_ = true;
    last_car_ = measurement;
    last_car_stamp_s_ = measurement.stamp_s;
    last_measurement_stamp_s_ = measurement.stamp_s;
    last_confidence_ = measurement.confidence;
}

bool PlatformTracker::updateVision(const VisionMeasurement& measurement) {
    if (!initialised_) {
        initialise(measurement.stamp_s, measurement.x, measurement.y, 0.0, 0.0);
    } else {
        predictTo(measurement.stamp_s);
        const double residual =
            std::hypot(measurement.x - state_.x(), measurement.y - state_.y());
        if (residual > config_.max_visual_residual_m) {
            return false;
        }
        updatePosition(
            measurement.x, measurement.y,
            safeVariance(config_.vision_position_noise, measurement.confidence));
    }
    if (has_car_) {
        const Eigen::Vector2d car_world(
            last_car_.x + config_.frame_offset_x_m,
            last_car_.y + config_.frame_offset_y_m);
        const Eigen::Vector2d world_offset =
            Eigen::Vector2d(measurement.x, measurement.y) - car_world;
        visual_offset_body_ = yawRotation(-last_car_.yaw) * world_offset;
        has_visual_offset_ = true;
    }
    last_vision_stamp_s_ = measurement.stamp_s;
    last_measurement_stamp_s_ = measurement.stamp_s;
    last_confidence_ = measurement.confidence;
    return true;
}

void PlatformTracker::updatePosition(double x, double y, double variance) {
    Eigen::Matrix<double, 2, 4> observation;
    observation << 1.0, 0.0, 0.0, 0.0,
                   0.0, 1.0, 0.0, 0.0;
    Eigen::Vector2d innovation(x - state_.x(), y - state_.y());
    Eigen::Matrix2d innovation_covariance =
        observation * covariance_ * observation.transpose()
        + Eigen::Matrix2d::Identity() * variance;
    const Eigen::Matrix<double, 4, 2> gain =
        covariance_ * observation.transpose() * innovation_covariance.inverse();
    state_ += gain * innovation;
    covariance_ = (Eigen::Matrix4d::Identity() - gain * observation) * covariance_;
}

void PlatformTracker::updateFull(double x, double y, double vx, double vy,
                                 double position_variance,
                                 double velocity_variance) {
    Eigen::Vector4d measurement(x, y, vx, vy);
    Eigen::Matrix4d noise = Eigen::Matrix4d::Zero();
    noise(0, 0) = position_variance;
    noise(1, 1) = position_variance;
    noise(2, 2) = velocity_variance;
    noise(3, 3) = velocity_variance;
    const Eigen::Matrix4d innovation_covariance = covariance_ + noise;
    const Eigen::Matrix4d gain = covariance_ * innovation_covariance.inverse();
    state_ += gain * (measurement - state_);
    covariance_ = (Eigen::Matrix4d::Identity() - gain) * covariance_;
}

PlatformState PlatformTracker::stateAt(double stamp_s) {
    PlatformState output;
    if (!initialised_) {
        return output;
    }
    predictTo(stamp_s);
    output.x = state_.x();
    output.y = state_.y();
    output.vx = state_(2);
    output.vy = state_(3);
    output.confidence = last_confidence_;
    output.measurement_age_s = std::max(0.0, stamp_s - last_measurement_stamp_s_);
    if (last_vision_stamp_s_ >= 0.0
        && stamp_s - last_vision_stamp_s_ <= config_.source_timeout_s) {
        output.valid = true;
        output.mode = FilterMode::MEASURED;
    } else if ((last_car_stamp_s_ >= 0.0
                && stamp_s - last_car_stamp_s_ <= config_.source_timeout_s)
               || output.measurement_age_s <= config_.prediction_timeout_s) {
        output.valid = true;
        output.mode = FilterMode::PREDICTED;
    } else {
        output.valid = false;
        output.mode = FilterMode::STALE;
    }
    return output;
}

}  // namespace d_task_uav_control
