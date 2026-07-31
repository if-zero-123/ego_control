#ifndef D_TASK_UAV_CONTROL_PLATFORM_TRACKER_H_
#define D_TASK_UAV_CONTROL_PLATFORM_TRACKER_H_

#include <cstdint>

#include <Eigen/Dense>

namespace d_task_uav_control {

enum class FilterMode : uint8_t {
    INVALID = 0,
    MEASURED = 1,
    PREDICTED = 2,
    STALE = 3,
};

struct TrackerConfig {
    double process_noise = 0.2;
    double car_position_noise = 0.20;
    double car_velocity_noise = 0.15;
    double vision_position_noise = 0.05;
    double max_visual_residual_m = 0.80;
    double source_timeout_s = 0.30;
    double prediction_timeout_s = 1.00;
    double frame_offset_x_m = 0.0;
    double frame_offset_y_m = 0.0;
    double frame_yaw_offset_rad = 0.0;
    double platform_offset_body_x_m = 0.0;
    double platform_offset_body_y_m = 0.0;
};

struct CarMeasurement {
    double stamp_s;
    double x;
    double y;
    double yaw;
    double vx;
    double vy;
    double confidence;
};

struct VisionMeasurement {
    double stamp_s;
    double x;
    double y;
    double confidence;
};

struct PlatformState {
    bool valid = false;
    FilterMode mode = FilterMode::INVALID;
    double x = 0.0;
    double y = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double confidence = 0.0;
    double measurement_age_s = 0.0;
};

class PlatformTracker {
public:
    explicit PlatformTracker(const TrackerConfig& config = TrackerConfig());

    void reset();
    void updateCar(const CarMeasurement& measurement);
    bool updateVision(const VisionMeasurement& measurement);
    PlatformState stateAt(double stamp_s);
    bool hasLockedVisualOffset() const { return has_visual_offset_; }
    Eigen::Vector2d visualOffsetBody() const { return visual_offset_body_; }

private:
    void predictTo(double stamp_s);
    void initialise(double stamp_s, double x, double y, double vx, double vy);
    void updatePosition(double x, double y, double variance);
    void updateFull(double x, double y, double vx, double vy,
                    double position_variance, double velocity_variance);
    Eigen::Vector2d carWorldPosition(const CarMeasurement& measurement) const;
    Eigen::Vector2d carWorldVelocity(const CarMeasurement& measurement) const;
    double carWorldYaw(const CarMeasurement& measurement) const;
    Eigen::Vector2d carPlatformPosition(const CarMeasurement& measurement) const;

    TrackerConfig config_;
    bool initialised_ = false;
    Eigen::Vector4d state_ = Eigen::Vector4d::Zero();
    Eigen::Matrix4d covariance_ = Eigen::Matrix4d::Identity();
    double filter_stamp_s_ = 0.0;
    double last_car_stamp_s_ = -1.0;
    double last_vision_stamp_s_ = -1.0;
    double last_measurement_stamp_s_ = -1.0;
    double last_confidence_ = 0.0;
    bool has_car_ = false;
    CarMeasurement last_car_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    bool has_visual_offset_ = false;
    Eigen::Vector2d visual_offset_body_ = Eigen::Vector2d::Zero();
};

}  // namespace d_task_uav_control

#endif  // D_TASK_UAV_CONTROL_PLATFORM_TRACKER_H_
