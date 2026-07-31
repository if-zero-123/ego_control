#include <gtest/gtest.h>

#include "d_task_uav_control/detection_source_selector.h"

namespace d_task_uav_control {
namespace {

TEST(DetectionSourceSelector, PrefersFreshDetectedAprilTag) {
    EXPECT_EQ(
        selectDetectionSource(
            10.0, 0.35, true, true, true, 9.90, true, true, 9.95),
        DetectionSource::APRILTAG);
}

TEST(DetectionSourceSelector, KeepsYoloDuringCruiseEvenWhenTagIsVisible) {
    EXPECT_EQ(
        selectDetectionSource(
            10.0, 0.35, false, true, true, 9.90, true, true, 9.95),
        DetectionSource::YOLO);
}

TEST(DetectionSourceSelector, FallsBackToYoloWhenTagIsMissing) {
    EXPECT_EQ(
        selectDetectionSource(
            10.0, 0.35, true, true, true, 9.90, true, false, 9.95),
        DetectionSource::YOLO);
}

TEST(DetectionSourceSelector, UsesFreshNotFoundSourceForLiveness) {
    EXPECT_EQ(
        selectDetectionSource(
            10.0, 0.35, true, true, false, 9.80, true, false, 9.95),
        DetectionSource::APRILTAG);
}

TEST(DetectionSourceSelector, RejectsTwoStaleSources) {
    EXPECT_EQ(
        selectDetectionSource(
            10.0, 0.35, true, true, true, 9.0, true, true, 9.5),
        DetectionSource::NONE);
}

}  // namespace
}  // namespace d_task_uav_control

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
