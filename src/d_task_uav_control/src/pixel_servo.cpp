#include "d_task_uav_control/pixel_servo.h"

#include <algorithm>
#include <cmath>

namespace d_task_uav_control {

namespace {

double clamp(double value, double lower, double upper) {
    return std::max(lower, std::min(upper, value));
}

}  // namespace

PixelServo::PixelServo(const PixelServoConfig& config) : config_(config) {
    reset();
}

bool pixelServoConfigValid(const PixelServoConfig& config) {
    if (!std::isfinite(config.filter_alpha) || !std::isfinite(config.filter_beta)
        || !std::isfinite(config.source_timeout_s)
        || !std::isfinite(config.min_confidence) || !std::isfinite(config.deadband)
        || !std::isfinite(config.gain_mps) || !std::isfinite(config.max_speed_mps)
        || !std::isfinite(config.body_x_from_u)
        || !std::isfinite(config.body_x_from_v)
        || !std::isfinite(config.body_y_from_u)
        || !std::isfinite(config.body_y_from_v)
        || config.filter_alpha < 0.0 || config.filter_alpha > 1.0
        || config.filter_beta < 0.0 || config.filter_beta > 1.0
        || config.source_timeout_s <= 0.0 || config.min_confidence < 0.0
        || config.min_confidence > 1.0 || config.deadband < 0.0
        || config.gain_mps < 0.0 || config.max_speed_mps <= 0.0) {
        return false;
    }
    const double mapping_norm = std::hypot(
        std::hypot(config.body_x_from_u, config.body_x_from_v),
        std::hypot(config.body_y_from_u, config.body_y_from_v));
    return mapping_norm > 1e-6;
}

void PixelServo::reset() {
    initialised_ = false;
    error_u_ = 0.0;
    error_v_ = 0.0;
    error_u_rate_ = 0.0;
    error_v_rate_ = 0.0;
    last_stamp_s_ = -1.0;
}

bool PixelServo::update(const PixelMeasurement& measurement) {
    if (measurement.image_width == 0U || measurement.image_height == 0U
        || !std::isfinite(measurement.stamp_s)
        || !std::isfinite(measurement.center_u)
        || !std::isfinite(measurement.center_v)
        || measurement.center_u < 0.0
        || measurement.center_u > static_cast<double>(measurement.image_width)
        || measurement.center_v < 0.0
        || measurement.center_v > static_cast<double>(measurement.image_height)
        || measurement.confidence < config_.min_confidence) {
        return false;
    }

    const double measured_u = 2.0 * measurement.center_u
        / static_cast<double>(measurement.image_width) - 1.0;
    const double measured_v = 2.0 * measurement.center_v
        / static_cast<double>(measurement.image_height) - 1.0;
    if (!std::isfinite(measured_u) || !std::isfinite(measured_v)
        || (initialised_ && measurement.stamp_s <= last_stamp_s_)) {
        return false;
    }

    if (!initialised_) {
        error_u_ = measured_u;
        error_v_ = measured_v;
        error_u_rate_ = 0.0;
        error_v_rate_ = 0.0;
        initialised_ = true;
    } else {
        const double dt = std::min(1.0, measurement.stamp_s - last_stamp_s_);
        const double predicted_u = error_u_ + error_u_rate_ * dt;
        const double predicted_v = error_v_ + error_v_rate_ * dt;
        const double alpha = clamp(config_.filter_alpha * measurement.confidence,
                                   0.0, 1.0);
        const double beta = clamp(config_.filter_beta * measurement.confidence,
                                  0.0, 1.0);
        const double residual_u = measured_u - predicted_u;
        const double residual_v = measured_v - predicted_v;
        error_u_ = predicted_u + alpha * residual_u;
        error_v_ = predicted_v + alpha * residual_v;
        error_u_rate_ += beta * residual_u / dt;
        error_v_rate_ += beta * residual_v / dt;
    }
    last_stamp_s_ = measurement.stamp_s;
    return true;
}

PixelServoState PixelServo::stateAt(double stamp_s, double yaw_rad) const {
    PixelServoState output;
    if (!initialised_ || !std::isfinite(stamp_s) || !std::isfinite(yaw_rad)) {
        return output;
    }

    const double raw_measurement_age_s = stamp_s - last_stamp_s_;
    if (raw_measurement_age_s < -1e-3) {
        return output;
    }
    output.measurement_age_s = std::max(0.0, raw_measurement_age_s);
    if (output.measurement_age_s > config_.source_timeout_s) {
        return output;
    }

    const double predicted_u = error_u_ + error_u_rate_ * output.measurement_age_s;
    const double predicted_v = error_v_ + error_v_rate_ * output.measurement_age_s;
    output.error_u = applyDeadband(predicted_u);
    output.error_v = applyDeadband(predicted_v);
    output.error_u_rate = error_u_rate_;
    output.error_v_rate = error_v_rate_;
    output.body_vx = config_.gain_mps
        * (config_.body_x_from_u * output.error_u
           + config_.body_x_from_v * output.error_v);
    output.body_vy = config_.gain_mps
        * (config_.body_y_from_u * output.error_u
           + config_.body_y_from_v * output.error_v);

    const double body_speed = std::hypot(output.body_vx, output.body_vy);
    if (body_speed > config_.max_speed_mps && body_speed > 1e-9) {
        const double scale = config_.max_speed_mps / body_speed;
        output.body_vx *= scale;
        output.body_vy *= scale;
    }

    const double cosine = std::cos(yaw_rad);
    const double sine = std::sin(yaw_rad);
    output.world_vx = cosine * output.body_vx - sine * output.body_vy;
    output.world_vy = sine * output.body_vx + cosine * output.body_vy;
    output.valid = true;
    return output;
}

double PixelServo::applyDeadband(double error) const {
    return std::abs(error) <= config_.deadband ? 0.0 : error;
}

}  // namespace d_task_uav_control
