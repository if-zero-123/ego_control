#ifndef D_TASK_UAV_CONTROL_PIXEL_SERVO_H_
#define D_TASK_UAV_CONTROL_PIXEL_SERVO_H_

namespace d_task_uav_control {

struct PixelServoConfig {
    double filter_alpha = 0.45;
    double filter_beta = 0.08;
    double source_timeout_s = 0.30;
    double min_confidence = 0.25;
    double deadband = 0.04;
    double gain_mps = 0.40;
    double max_speed_mps = 0.35;
    double body_x_from_u = 0.0;
    double body_x_from_v = -1.0;
    double body_y_from_u = -1.0;
    double body_y_from_v = 0.0;
};

struct PixelMeasurement {
    double stamp_s;
    double center_u;
    double center_v;
    unsigned int image_width;
    unsigned int image_height;
    double confidence;
};

struct PixelServoState {
    bool valid = false;
    double error_u = 0.0;
    double error_v = 0.0;
    double error_u_rate = 0.0;
    double error_v_rate = 0.0;
    double body_vx = 0.0;
    double body_vy = 0.0;
    double world_vx = 0.0;
    double world_vy = 0.0;
    double measurement_age_s = 0.0;
};

bool pixelServoConfigValid(const PixelServoConfig& config);

class PixelServo {
public:
    explicit PixelServo(const PixelServoConfig& config = PixelServoConfig());

    void reset();
    bool update(const PixelMeasurement& measurement);
    PixelServoState stateAt(double stamp_s, double yaw_rad) const;

private:
    double applyDeadband(double error) const;

    PixelServoConfig config_;
    bool initialised_ = false;
    double error_u_ = 0.0;
    double error_v_ = 0.0;
    double error_u_rate_ = 0.0;
    double error_v_rate_ = 0.0;
    double last_stamp_s_ = -1.0;
};

}  // namespace d_task_uav_control

#endif  // D_TASK_UAV_CONTROL_PIXEL_SERVO_H_
