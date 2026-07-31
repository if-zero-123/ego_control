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

std::vector<AprilTagLayoutEntry> platformLayout() {
    return {
        {0, 0.045, 0.000, 0.000, 0.0},
        {1, 0.120, -0.195, 0.195, 0.0},
        {2, 0.120, 0.195, 0.195, 0.0},
        {3, 0.120, 0.195, -0.195, 0.0},
        {4, 0.120, -0.195, -0.195, 0.0},
    };
}

AprilTagDetection projectedDetection(
    const AprilTagLayoutEntry& entry,
    const cv::Vec3d& rotation,
    const cv::Vec3d& translation,
    const cv::Mat& camera_matrix,
    const cv::Mat& distortion) {
    AprilTagDetection detection;
    detection.found = true;
    detection.id = entry.id;
    detection.confidence = 1.0F;
    const std::vector<cv::Point3f> object_points =
        aprilTagLayoutObjectCorners(entry);
    cv::projectPoints(
        object_points, rotation, translation,
        camera_matrix, distortion, detection.corners);
    for (const cv::Point2f& corner : detection.corners) {
        detection.center += corner;
    }
    detection.center *= 0.25F;
    detection.bbox = cv::boundingRect(detection.corners);
    return detection;
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

TEST(AprilTagDetector, DetectsAllConfiguredPlatformIdsInOneFrame) {
    const cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    cv::Mat canvas(360, 640, CV_8UC1, cv::Scalar(255));
    cv::Mat marker_zero;
    cv::Mat marker_two;
    cv::Mat unrelated;
    cv::aruco::drawMarker(dictionary, 0, 140, marker_zero, 1);
    cv::aruco::drawMarker(dictionary, 2, 140, marker_two, 1);
    cv::aruco::drawMarker(dictionary, 9, 140, unrelated, 1);
    marker_zero.copyTo(canvas(cv::Rect(40, 100, 140, 140)));
    marker_two.copyTo(canvas(cv::Rect(250, 100, 140, 140)));
    unrelated.copyTo(canvas(cv::Rect(460, 100, 140, 140)));
    AprilTagDetector detector(std::vector<int>{0, 1, 2, 3, 4}, 8.0);

    const std::vector<AprilTagDetection> detections = detector.detectAll(canvas);

    ASSERT_EQ(detections.size(), 2U);
    EXPECT_EQ(detections[0].id, 0);
    EXPECT_EQ(detections[1].id, 2);
}

TEST(AprilTagBoardPose, InfersPlatformCenterFromOneOuterTag) {
    const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3)
        << 900.0, 0.0, 320.0,
           0.0, 900.0, 240.0,
           0.0, 0.0, 1.0);
    const cv::Mat distortion = cv::Mat::zeros(1, 5, CV_64F);
    const cv::Vec3d expected_rotation(0.04, -0.06, 0.10);
    const cv::Vec3d expected_translation(0.05, -0.03, 1.40);
    const std::vector<AprilTagLayoutEntry> layout = platformLayout();
    const AprilTagDetection outer = projectedDetection(
        layout[1], expected_rotation, expected_translation,
        camera_matrix, distortion);

    const AprilTagBoardEstimate estimate = estimateAprilTagBoardPose(
        {outer}, layout, camera_matrix, distortion, 3.0);

    ASSERT_TRUE(estimate.valid);
    ASSERT_EQ(estimate.used_tag_ids.size(), 1U);
    EXPECT_EQ(estimate.used_tag_ids.front(), 1);
    EXPECT_FALSE(estimate.center_tag_visible);
    EXPECT_NEAR(estimate.translation_m[0], expected_translation[0], 1e-3);
    EXPECT_NEAR(estimate.translation_m[1], expected_translation[1], 1e-3);
    EXPECT_NEAR(estimate.translation_m[2], expected_translation[2], 1e-3);
    EXPECT_NEAR(estimate.plane_distance_m, expected_translation[2], 0.02);
}

TEST(AprilTagBoardPose, UsesPlanarHomographyForNoisySingleOuterTagCenter) {
    const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3)
        << 900.0, 0.0, 320.0,
           0.0, 900.0, 240.0,
           0.0, 0.0, 1.0);
    const cv::Mat distortion = cv::Mat::zeros(1, 5, CV_64F);
    const cv::Vec3d rotation(0.04, -0.06, 0.10);
    const cv::Vec3d translation(0.05, -0.03, 1.40);
    const std::vector<AprilTagLayoutEntry> layout = platformLayout();
    AprilTagDetection outer = projectedDetection(
        layout[1], rotation, translation, camera_matrix, distortion);
    const std::vector<cv::Point2f> perturbation{
        {1.0F, -0.5F}, {0.2F, 0.7F}, {-0.8F, 0.3F}, {0.4F, -0.9F}};
    for (std::size_t index = 0; index < outer.corners.size(); ++index) {
        outer.corners[index] += perturbation[index];
    }

    const std::vector<cv::Point2f> object_plane{
        {-0.195F - 0.060F, 0.195F + 0.060F},
        {-0.195F + 0.060F, 0.195F + 0.060F},
        {-0.195F + 0.060F, 0.195F - 0.060F},
        {-0.195F - 0.060F, 0.195F - 0.060F}};
    const cv::Mat homography = cv::findHomography(
        object_plane, outer.corners, 0);
    std::vector<cv::Point2f> projected_center;
    cv::perspectiveTransform(
        std::vector<cv::Point2f>{{0.0F, 0.0F}}, projected_center,
        homography);

    const AprilTagBoardEstimate estimate = estimateAprilTagBoardPose(
        {outer}, layout, camera_matrix, distortion, 3.0);

    ASSERT_TRUE(estimate.valid);
    ASSERT_EQ(projected_center.size(), 1U);
    EXPECT_NEAR(estimate.center.x, projected_center.front().x, 0.5);
    EXPECT_NEAR(estimate.center.y, projected_center.front().y, 0.5);
}

TEST(AprilTagBoardPose, FusesCenterAndOuterTagsIntoPlatformPose) {
    const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3)
        << 900.0, 0.0, 320.0,
           0.0, 905.0, 240.0,
           0.0, 0.0, 1.0);
    const cv::Mat distortion = cv::Mat::zeros(1, 5, CV_64F);
    const cv::Vec3d expected_rotation(0.03, -0.04, -0.08);
    const cv::Vec3d expected_translation(-0.04, 0.02, 0.85);
    const std::vector<AprilTagLayoutEntry> layout = platformLayout();
    const std::vector<AprilTagDetection> detections{
        projectedDetection(layout[0], expected_rotation, expected_translation,
                           camera_matrix, distortion),
        projectedDetection(layout[1], expected_rotation, expected_translation,
                           camera_matrix, distortion),
        projectedDetection(layout[2], expected_rotation, expected_translation,
                           camera_matrix, distortion),
    };

    const AprilTagBoardEstimate estimate = estimateAprilTagBoardPose(
        detections, layout, camera_matrix, distortion, 3.0);

    ASSERT_TRUE(estimate.valid);
    EXPECT_TRUE(estimate.center_tag_visible);
    EXPECT_EQ(estimate.used_tag_ids.size(), 3U);
    EXPECT_NEAR(estimate.translation_m[0], expected_translation[0], 1e-4);
    EXPECT_NEAR(estimate.translation_m[1], expected_translation[1], 1e-4);
    EXPECT_NEAR(estimate.translation_m[2], expected_translation[2], 1e-4);
    EXPECT_LT(estimate.reprojection_error_px, 1e-3);
}

TEST(AprilTagBoardPose, RejectsOneGeometricallyInconsistentOuterTag) {
    const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3)
        << 900.0, 0.0, 320.0,
           0.0, 900.0, 240.0,
           0.0, 0.0, 1.0);
    const cv::Mat distortion = cv::Mat::zeros(1, 5, CV_64F);
    const cv::Vec3d expected_rotation(0.02, -0.03, 0.04);
    const cv::Vec3d expected_translation(0.02, -0.01, 1.20);
    const std::vector<AprilTagLayoutEntry> layout = platformLayout();
    std::vector<AprilTagDetection> detections;
    for (int id = 1; id <= 4; ++id) {
        detections.push_back(projectedDetection(
            layout[static_cast<std::size_t>(id)], expected_rotation,
            expected_translation, camera_matrix, distortion));
    }
    for (cv::Point2f& corner : detections.back().corners) {
        corner += cv::Point2f(90.0F, -70.0F);
    }

    const AprilTagBoardEstimate estimate = estimateAprilTagBoardPose(
        detections, layout, camera_matrix, distortion, 3.0);

    ASSERT_TRUE(estimate.valid);
    EXPECT_EQ(estimate.used_tag_ids, (std::vector<int>{1, 2, 3}));
    EXPECT_NEAR(estimate.translation_m[0], expected_translation[0], 1e-3);
    EXPECT_NEAR(estimate.translation_m[1], expected_translation[1], 1e-3);
    EXPECT_NEAR(estimate.translation_m[2], expected_translation[2], 1e-3);
}

TEST(AprilTagImagePipeline, FiltersCenterTagOnlyForOuterPolicy) {
    std::vector<AprilTagDetection> detections(3);
    detections[0].found = true;
    detections[0].id = 0;
    detections[1].found = true;
    detections[1].id = 1;
    detections[2].found = true;
    detections[2].id = 4;

    const std::vector<AprilTagDetection> cruise =
        filterPlatformTagDetections(detections, false);
    const std::vector<AprilTagDetection> close_range =
        filterPlatformTagDetections(detections, true);

    ASSERT_EQ(cruise.size(), 2U);
    EXPECT_EQ(cruise[0].id, 1);
    EXPECT_EQ(cruise[1].id, 4);
    EXPECT_EQ(close_range.size(), 3U);
}

TEST(AprilTagImagePipeline, ExpandsAndClampsTrackingRoi) {
    const cv::Rect roi = expandedImageRoi(
        cv::Rect2f(560.0F, 410.0F, 60.0F, 50.0F), 2.0,
        cv::Size(640, 480), 80);

    EXPECT_EQ(roi, cv::Rect(530, 385, 110, 95));
}

TEST(AprilTagImagePipeline, DetectsLowContrastAndRestoresRoiCoordinates) {
    cv::Mat low_contrast(80, 100, CV_8UC1, cv::Scalar(120));
    cv::Mat high_contrast(80, 100, CV_8UC1, cv::Scalar(0));
    high_contrast.colRange(50, 100).setTo(255);
    EXPECT_TRUE(imageNeedsContrastEnhancement(low_contrast, 12.0));
    EXPECT_FALSE(imageNeedsContrastEnhancement(high_contrast, 12.0));

    AprilTagDetection local;
    local.found = true;
    local.center = cv::Point2f(20.0F, 30.0F);
    local.bbox = cv::Rect2f(10.0F, 15.0F, 20.0F, 30.0F);
    local.corners = {
        {10.0F, 15.0F}, {30.0F, 15.0F},
        {30.0F, 45.0F}, {10.0F, 45.0F}};

    std::vector<AprilTagDetection> translated{local};
    translateAprilTagDetections({25, 40}, translated);

    EXPECT_EQ(translated.front().center, cv::Point2f(45.0F, 70.0F));
    EXPECT_EQ(translated.front().bbox.x, 35.0F);
    EXPECT_EQ(translated.front().bbox.y, 55.0F);
    EXPECT_EQ(translated.front().corners.front(), cv::Point2f(35.0F, 55.0F));
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
