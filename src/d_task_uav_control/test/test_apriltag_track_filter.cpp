#include <gtest/gtest.h>

#include <stdexcept>

#include "d_task_uav_control/apriltag_track_filter.h"

namespace d_task_uav_control {
namespace {

AprilTagTrackFilterConfig testConfig() {
    AprilTagTrackFilterConfig config;
    config.filter_alpha = 0.50;
    config.filter_beta = 0.10;
    config.prediction_timeout_s = 0.20;
    config.max_velocity_px_s = 1000.0;
    config.reacquire_distance_px = 1000.0;
    return config;
}

AprilTagDetection measurement(float center_x, float center_y) {
    AprilTagDetection detection;
    detection.found = true;
    detection.id = 0;
    detection.confidence = 1.0F;
    detection.center = cv::Point2f(center_x, center_y);
    detection.bbox = cv::Rect2f(center_x - 20.0F, center_y - 20.0F,
                                40.0F, 40.0F);
    detection.corners = {
        cv::Point2f(center_x - 20.0F, center_y - 20.0F),
        cv::Point2f(center_x + 20.0F, center_y - 20.0F),
        cv::Point2f(center_x + 20.0F, center_y + 20.0F),
        cv::Point2f(center_x - 20.0F, center_y + 20.0F)};
    return detection;
}

TEST(AprilTagTrackFilter, RejectsInvalidConfiguration) {
    AprilTagTrackFilterConfig config = testConfig();
    config.prediction_timeout_s = 0.0;

    EXPECT_THROW(
        { AprilTagTrackFilter filter(config); },
        std::invalid_argument);
}

TEST(AprilTagTrackFilter, FiltersMeasuredCenterAndKeepsRealSemantics) {
    AprilTagTrackFilter filter(testConfig());
    ASSERT_TRUE(filter.update(measurement(100.0F, 120.0F),
                              1.0, 640U, 480U).found);

    const AprilTagDetection output = filter.update(
        measurement(110.0F, 120.0F), 1.1, 640U, 480U);

    ASSERT_TRUE(output.found);
    EXPECT_FALSE(output.predicted);
    EXPECT_DOUBLE_EQ(output.measurement_age_s, 0.0);
    EXPECT_NEAR(output.center.x, 105.0, 1e-6);
    EXPECT_NEAR(output.center.y, 120.0, 1e-6);
    ASSERT_EQ(output.corners.size(), 4U);
    EXPECT_NEAR(output.bbox.width, 40.0, 1e-6);
}

TEST(AprilTagTrackFilter, PredictsVelocityThroughShortMiss) {
    AprilTagTrackFilter filter(testConfig());
    filter.update(measurement(100.0F, 120.0F), 1.0, 640U, 480U);
    filter.update(measurement(110.0F, 120.0F), 1.1, 640U, 480U);

    const AprilTagDetection output = filter.update(
        AprilTagDetection(), 1.2, 640U, 480U);

    ASSERT_TRUE(output.found);
    EXPECT_TRUE(output.predicted);
    EXPECT_NEAR(output.measurement_age_s, 0.1, 1e-9);
    EXPECT_NEAR(output.center.x, 106.0, 1e-5);
    EXPECT_NEAR(output.confidence, 0.5, 1e-5);
    EXPECT_TRUE(output.corners.empty());
}

TEST(AprilTagTrackFilter, ExpiresPredictionAtConfiguredTimeout) {
    AprilTagTrackFilter filter(testConfig());
    filter.update(measurement(100.0F, 120.0F), 1.0, 640U, 480U);

    EXPECT_FALSE(filter.update(
        AprilTagDetection(), 1.201, 640U, 480U).found);
    EXPECT_FALSE(filter.update(
        AprilTagDetection(), 1.3, 640U, 480U).found);
}

TEST(AprilTagTrackFilter, ReinitialisesOnLargeReacquisitionJump) {
    AprilTagTrackFilterConfig config = testConfig();
    config.reacquire_distance_px = 20.0;
    AprilTagTrackFilter filter(config);
    filter.update(measurement(100.0F, 120.0F), 1.0, 640U, 480U);

    const AprilTagDetection output = filter.update(
        measurement(200.0F, 220.0F), 1.1, 640U, 480U);

    ASSERT_TRUE(output.found);
    EXPECT_FALSE(output.predicted);
    EXPECT_NEAR(output.center.x, 200.0, 1e-6);
    EXPECT_NEAR(output.center.y, 220.0, 1e-6);
}

TEST(AprilTagTrackFilter, NeverTreatsPredictedDetectionAsPoseMeasurement) {
    AprilTagDetection predicted = measurement(100.0F, 120.0F);
    predicted.predicted = true;

    const AprilTagPoseEstimate pose = estimateAprilTagPose(
        predicted, 0.080, cv::Mat::eye(3, 3, CV_64F), cv::Mat());

    EXPECT_FALSE(pose.valid);
}

}  // namespace
}  // namespace d_task_uav_control

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
