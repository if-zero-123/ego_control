#ifndef D_TASK_UAV_CONTROL_APRILTAG_DETECTOR_H_
#define D_TASK_UAV_CONTROL_APRILTAG_DETECTOR_H_

#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

namespace d_task_uav_control {

struct AprilTagDetection {
    bool found = false;
    bool predicted = false;
    double measurement_age_s = 0.0;
    int id = -1;
    float confidence = 0.0F;
    cv::Point2f center;
    cv::Rect2f bbox;
    std::vector<cv::Point2f> corners;
};

struct AprilTagPoseEstimate {
    bool valid = false;
    cv::Vec3d rotation_vector;
    cv::Vec3d translation_m;
    double mean_side_px = 0.0;
    double optical_axis_distance_m = 0.0;
    double slant_range_m = 0.0;
    double plane_distance_m = 0.0;
    double tag_tilt_deg = 0.0;
    double reprojection_error_px = 0.0;
};

class AprilTagDetector {
public:
    explicit AprilTagDetector(int target_id = 0, double min_side_px = 8.0);

    AprilTagDetection detect(const cv::Mat& image) const;

private:
    int target_id_ = 0;
    double min_side_px_ = 8.0;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
};

AprilTagPoseEstimate estimateAprilTagPose(
    const AprilTagDetection& detection,
    double tag_size_m,
    const cv::Mat& camera_matrix,
    const cv::Mat& distortion_coefficients);

}  // namespace d_task_uav_control

#endif  // D_TASK_UAV_CONTROL_APRILTAG_DETECTOR_H_
