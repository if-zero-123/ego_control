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

TEST(PlatformTracker, DefaultTrackerIgnoresCarOnlyMeasurement) {
    PlatformTracker tracker;

    tracker.updateCar(CarMeasurement{
        0.0, -8.0, 4.0, 0.0, -1.0, 0.5, 1.0});
    const PlatformState state = tracker.stateAt(0.0);

    EXPECT_FALSE(state.valid);
    EXPECT_EQ(state.mode, FilterMode::INVALID);
}

TEST(PlatformTracker, CarMeasurementCannotPullVisualStateBackward) {
    PlatformTracker tracker;
    ASSERT_TRUE(tracker.updateVision(
        VisionMeasurement{0.0, 1.0, 2.0, 1.0}));
    const PlatformState visual_state = tracker.stateAt(0.0);

    tracker.updateCar(CarMeasurement{
        0.0, -10.0, -10.0, 0.0, -2.0, -2.0, 1.0});
    const PlatformState after_car = tracker.stateAt(0.0);

    ASSERT_TRUE(after_car.valid);
    EXPECT_NEAR(after_car.x, visual_state.x, 1e-9);
    EXPECT_NEAR(after_car.y, visual_state.y, 1e-9);
    EXPECT_NEAR(after_car.vx, visual_state.vx, 1e-9);
    EXPECT_NEAR(after_car.vy, visual_state.vy, 1e-9);
}

TEST(PlatformTracker, KeepsOnlyVisualPredictionForOneSecond) {
    TrackerConfig config;
    config.source_timeout_s = 0.30;
    config.prediction_timeout_s = 1.0;
    PlatformTracker tracker(config);

    ASSERT_TRUE(tracker.updateVision(
        VisionMeasurement{0.0, 1.0, 2.0, 1.0}));
    ASSERT_TRUE(tracker.updateVision(
        VisionMeasurement{0.2, 1.1, 2.0, 1.0}));
    tracker.updateCar(CarMeasurement{
        0.4, -10.0, -10.0, 0.0, -2.0, -2.0, 1.0});

    const PlatformState short_prediction = tracker.stateAt(1.1);
    EXPECT_TRUE(short_prediction.valid);
    EXPECT_EQ(short_prediction.mode, FilterMode::PREDICTED);

    const PlatformState expired_prediction = tracker.stateAt(1.21);
    EXPECT_FALSE(expired_prediction.valid);
    EXPECT_EQ(expired_prediction.mode, FilterMode::STALE);
}

TEST(PlatformTracker, LocksVisualOffsetAndUsesCarFallback) {
    TrackerConfig config;
    config.use_car_measurements = true;
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
    config.use_car_measurements = true;
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

TEST(PlatformTracker, ReinitializesVisualStateAfterStale) {
    TrackerConfig config;
    config.source_timeout_s = 0.30;
    config.prediction_timeout_s = 0.50;
    config.max_visual_residual_m = 0.20;
    PlatformTracker tracker(config);

    ASSERT_TRUE(tracker.updateVision(
        VisionMeasurement{0.0, 1.0, 2.0, 1.0}));
    EXPECT_EQ(tracker.stateAt(1.0).mode, FilterMode::STALE);

    ASSERT_TRUE(tracker.updateVision(
        VisionMeasurement{1.1, 5.0, -3.0, 1.0}));
    const PlatformState reacquired = tracker.stateAt(1.1);
    EXPECT_TRUE(reacquired.valid);
    EXPECT_EQ(reacquired.mode, FilterMode::MEASURED);
    EXPECT_NEAR(reacquired.x, 5.0, 1e-9);
    EXPECT_NEAR(reacquired.y, -3.0, 1e-9);
}

TEST(PlatformTracker, RotatesAndTranslatesCarPositionVelocityAndHeading) {
    TrackerConfig config;
    config.use_car_measurements = true;
    config.frame_offset_x_m = 10.0;
    config.frame_offset_y_m = -2.0;
    config.frame_yaw_offset_rad = 1.5707963267948966;
    config.platform_offset_body_x_m = 1.0;
    PlatformTracker tracker(config);

    tracker.updateCar(CarMeasurement{
        0.0, 2.0, 3.0, 0.0, 1.0, 0.0, 1.0});
    const PlatformState state = tracker.stateAt(0.0);

    // Base: R(90deg)*(2,3)+(10,-2)=(7,0). The 1 m platform
    // offset follows the transformed car yaw, giving platform=(7,1).
    EXPECT_TRUE(state.valid);
    EXPECT_NEAR(state.x, 7.0, 1e-9);
    EXPECT_NEAR(state.y, 1.0, 1e-9);
    EXPECT_NEAR(state.vx, 0.0, 1e-9);
    EXPECT_NEAR(state.vy, 1.0, 1e-9);
}

}  // namespace d_task_uav_control

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
