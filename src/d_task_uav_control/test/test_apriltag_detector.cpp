#include <gtest/gtest.h>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "d_task_uav_control/apriltag_detector.h"

namespace d_task_uav_control {
namespace {

cv::Mat markerCanvas(int id) {
    const cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    cv::Mat marker;
    cv::aruco::drawMarker(dictionary, id, 160, marker, 1);
    cv::Mat canvas(320, 320, CV_8UC1, cv::Scalar(255));
    marker.copyTo(canvas(cv::Rect(80, 80, 160, 160)));
    return canvas;
}

TEST(AprilTagDetector, DetectsConfigured36h11IdZero) {
    AprilTagDetector detector(0, 8.0);

    const AprilTagDetection detection = detector.detect(markerCanvas(0));

    ASSERT_TRUE(detection.found);
    EXPECT_FALSE(detection.predicted);
    EXPECT_DOUBLE_EQ(detection.measurement_age_s, 0.0);
    EXPECT_EQ(detection.id, 0);
    EXPECT_NEAR(detection.center.x, 159.5, 1.0);
    EXPECT_NEAR(detection.center.y, 159.5, 1.0);
    EXPECT_GT(detection.confidence, 0.9);
}

TEST(AprilTagDetector, RejectsAnotherId) {
    AprilTagDetector detector(0, 8.0);

    EXPECT_FALSE(detector.detect(markerCanvas(1)).found);
}

TEST(AprilTagDetector, EstimatesMetricPoseFromKnownTagSize) {
    const double tag_size_m = 0.080;
    const double half_size = tag_size_m * 0.5;
    const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3)
        << 520.0, 0.0, 320.0,
           0.0, 515.0, 240.0,
           0.0, 0.0, 1.0);
    const cv::Mat distortion = cv::Mat::zeros(1, 5, CV_64F);
    const cv::Vec3d expected_rotation(0.0, 0.0, 0.0);
    const cv::Vec3d expected_translation(0.04, -0.02, 0.60);
    const std::vector<cv::Point3f> object_points{
        cv::Point3f(-half_size, half_size, 0.0),
        cv::Point3f(half_size, half_size, 0.0),
        cv::Point3f(half_size, -half_size, 0.0),
        cv::Point3f(-half_size, -half_size, 0.0)};

    AprilTagDetection detection;
    detection.found = true;
    detection.id = 0;
    cv::projectPoints(
        object_points, expected_rotation, expected_translation,
        camera_matrix, distortion, detection.corners);

    const AprilTagPoseEstimate estimate = estimateAprilTagPose(
        detection, tag_size_m, camera_matrix, distortion);

    ASSERT_TRUE(estimate.valid);
    EXPECT_NEAR(estimate.translation_m[0], expected_translation[0], 1e-4);
    EXPECT_NEAR(estimate.translation_m[1], expected_translation[1], 1e-4);
    EXPECT_NEAR(
        estimate.optical_axis_distance_m, expected_translation[2], 1e-4);
    EXPECT_NEAR(
        estimate.slant_range_m, cv::norm(expected_translation), 1e-4);
    EXPECT_NEAR(estimate.plane_distance_m, expected_translation[2], 1e-4);
    EXPECT_NEAR(estimate.tag_tilt_deg, 0.0, 1e-3);
    EXPECT_LT(estimate.reprojection_error_px, 1e-3);
    EXPECT_GT(estimate.mean_side_px, 60.0);
}

TEST(AprilTagDetector, RejectsPoseWithoutValidIntrinsics) {
    AprilTagDetection detection;
    detection.found = true;
    detection.corners = {
        cv::Point2f(100.0F, 100.0F),
        cv::Point2f(200.0F, 100.0F),
        cv::Point2f(200.0F, 200.0F),
        cv::Point2f(100.0F, 200.0F)};

    EXPECT_FALSE(estimateAprilTagPose(
        detection, 0.080, cv::Mat::zeros(3, 3, CV_64F), cv::Mat()).valid);
}

}  // namespace
}  // namespace d_task_uav_control

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
