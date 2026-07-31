#ifndef D_TASK_UAV_CONTROL_APRILTAG_TRACK_FILTER_H_
#define D_TASK_UAV_CONTROL_APRILTAG_TRACK_FILTER_H_

#include "d_task_uav_control/apriltag_detector.h"

namespace d_task_uav_control {

struct AprilTagTrackFilterConfig {
    double filter_alpha = 0.75;
    double filter_beta = 0.10;
    double prediction_timeout_s = 0.18;
    double max_velocity_px_s = 800.0;
    double reacquire_distance_px = 120.0;
};

bool aprilTagTrackFilterConfigValid(
    const AprilTagTrackFilterConfig& config);

class AprilTagTrackFilter {
public:
    explicit AprilTagTrackFilter(
        const AprilTagTrackFilterConfig& config =
            AprilTagTrackFilterConfig());

    void reset();
    AprilTagDetection update(
        const AprilTagDetection& measurement,
        double stamp_s,
        unsigned int image_width,
        unsigned int image_height);

private:
    void initialise(
        const AprilTagDetection& measurement, double stamp_s);
    bool predictTo(double stamp_s);
    AprilTagDetection makeMeasuredOutput(
        const AprilTagDetection& measurement,
        unsigned int image_width,
        unsigned int image_height) const;
    AprilTagDetection makePredictedOutput(
        double measurement_age_s,
        unsigned int image_width,
        unsigned int image_height) const;
    cv::Rect2f boundedBbox(
        unsigned int image_width, unsigned int image_height) const;

    AprilTagTrackFilterConfig config_;
    bool initialised_ = false;
    int id_ = -1;
    double center_x_ = 0.0;
    double center_y_ = 0.0;
    double width_ = 0.0;
    double height_ = 0.0;
    double center_vx_ = 0.0;
    double center_vy_ = 0.0;
    double width_rate_ = 0.0;
    double height_rate_ = 0.0;
    double last_update_s_ = -1.0;
    double last_measurement_s_ = -1.0;
    float last_confidence_ = 0.0F;
};

}  // namespace d_task_uav_control

#endif  // D_TASK_UAV_CONTROL_APRILTAG_TRACK_FILTER_H_
