#include <gtest/gtest.h>

#include <opencv2/aruco.hpp>
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
    EXPECT_EQ(detection.id, 0);
    EXPECT_NEAR(detection.center.x, 159.5, 1.0);
    EXPECT_NEAR(detection.center.y, 159.5, 1.0);
    EXPECT_GT(detection.confidence, 0.9);
}

TEST(AprilTagDetector, RejectsAnotherId) {
    AprilTagDetector detector(0, 8.0);

    EXPECT_FALSE(detector.detect(markerCanvas(1)).found);
}

}  // namespace
}  // namespace d_task_uav_control

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
