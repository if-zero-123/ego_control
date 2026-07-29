#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "d_task_uav_control/pixel_projector.h"
#include "d_task_uav_control/platform_tracker.h"

namespace d_task_uav_control {

TEST(PixelProjector, ProjectsNadirCenterOntoPlatformPlane) {
    CameraIntrinsics intrinsics{100.0, 100.0, 320.0, 240.0};
    Eigen::Matrix3d body_r_camera;
    body_r_camera << 0.0, -1.0, 0.0,
                    -1.0, 0.0, 0.0,
                     0.0, 0.0, -1.0;
    PixelProjector projector(intrinsics, body_r_camera, Eigen::Vector3d::Zero());
    Eigen::Vector3d result;

    const bool valid = projector.project(
        320.0, 240.0,
        Eigen::Vector3d(2.0, 3.0, 1.3),
        Eigen::Quaterniond::Identity(),
        0.3,
        result);

    EXPECT_TRUE(valid);
    EXPECT_NEAR(result.x(), 2.0, 1e-9);
    EXPECT_NEAR(result.y(), 3.0, 1e-9);
    EXPECT_NEAR(result.z(), 0.3, 1e-9);
}

TEST(PlatformTracker, LocksVisualOffsetAndUsesCarFallback) {
    TrackerConfig config;
    config.max_visual_residual_m = 1.0;
    config.source_timeout_s = 0.3;
    config.prediction_timeout_s = 1.0;
    PlatformTracker tracker(config);

    tracker.updateCar(CarMeasurement{0.0, 1.0, 2.0, 0.0, 0.2, 0.0, 1.0});
    ASSERT_TRUE(tracker.updateVision(VisionMeasurement{0.0, 1.2, 2.1, 0.9}));
    EXPECT_TRUE(tracker.hasLockedVisualOffset());

    tracker.updateCar(CarMeasurement{0.4, 1.08, 2.0, 0.0, 0.2, 0.0, 1.0});
    const PlatformState state = tracker.stateAt(0.4);

    EXPECT_TRUE(state.valid);
    EXPECT_EQ(state.mode, FilterMode::PREDICTED);
    EXPECT_NEAR(state.x, 1.28, 0.12);
    EXPECT_NEAR(state.y, 2.1, 0.12);
    EXPECT_NEAR(state.vx, 0.2, 0.12);
}

TEST(PlatformTracker, RejectsVisualOutlierAndEventuallyBecomesStale) {
    TrackerConfig config;
    config.max_visual_residual_m = 0.5;
    config.source_timeout_s = 0.3;
    config.prediction_timeout_s = 1.0;
    PlatformTracker tracker(config);
    tracker.updateCar(CarMeasurement{0.0, 1.0, 2.0, 0.0, 0.0, 0.0, 1.0});

    EXPECT_FALSE(tracker.updateVision(VisionMeasurement{0.1, 5.0, 6.0, 0.9}));
    EXPECT_EQ(tracker.stateAt(0.2).mode, FilterMode::PREDICTED);
    EXPECT_EQ(tracker.stateAt(1.2).mode, FilterMode::STALE);
    EXPECT_FALSE(tracker.stateAt(1.2).valid);
}

}  // namespace d_task_uav_control

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
