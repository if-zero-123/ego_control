#include <gtest/gtest.h>

#include "d_task_uav_control/pixel_servo.h"

namespace d_task_uav_control {
namespace {

TEST(PixelServo, MapsDefaultImageAxesToBodyVelocity) {
    PixelServoConfig config;
    config.filter_alpha = 1.0;
    config.gain_mps = 0.4;
    PixelServo servo(config);

    servo.update(PixelMeasurement{1.0, 480.0, 240.0, 640, 480, 1.0});
    const PixelServoState state = servo.stateAt(1.0, 0.0);

    ASSERT_TRUE(state.valid);
    EXPECT_NEAR(state.error_u, 0.5, 1e-9);
    EXPECT_NEAR(state.error_v, 0.0, 1e-9);
    EXPECT_NEAR(state.body_vx, 0.0, 1e-9);
    EXPECT_NEAR(state.body_vy, -0.2, 1e-9);
    EXPECT_NEAR(state.world_vx, 0.0, 1e-9);
    EXPECT_NEAR(state.world_vy, -0.2, 1e-9);
}

TEST(PixelServo, RotatesForwardCorrectionIntoWorldFrame) {
    PixelServoConfig config;
    config.filter_alpha = 1.0;
    config.gain_mps = 0.4;
    PixelServo servo(config);

    servo.update(PixelMeasurement{1.0, 320.0, 120.0, 640, 480, 1.0});
    const PixelServoState state = servo.stateAt(1.0, 1.5707963267948966);

    ASSERT_TRUE(state.valid);
    EXPECT_NEAR(state.body_vx, 0.2, 1e-9);
    EXPECT_NEAR(state.body_vy, 0.0, 1e-9);
    EXPECT_NEAR(state.world_vx, 0.0, 1e-9);
    EXPECT_NEAR(state.world_vy, 0.2, 1e-9);
}

TEST(PixelServo, RejectsExpiredVisualMeasurement) {
    PixelServoConfig config;
    config.filter_alpha = 1.0;
    config.source_timeout_s = 0.30;
    PixelServo servo(config);

    servo.update(PixelMeasurement{1.0, 320.0, 240.0, 640, 480, 1.0});

    EXPECT_TRUE(servo.stateAt(1.29, 0.0).valid);
    EXPECT_FALSE(servo.stateAt(1.31, 0.0).valid);
}

TEST(PixelServo, RejectsOutOfImageAndFutureMeasurements) {
    PixelServo servo;

    EXPECT_FALSE(servo.update(PixelMeasurement{1.0, -1.0, 240.0, 640, 480, 1.0}));
    EXPECT_FALSE(servo.update(PixelMeasurement{1.0, 320.0, 481.0, 640, 480, 1.0}));
    ASSERT_TRUE(servo.update(PixelMeasurement{1.0, 320.0, 240.0, 640, 480, 1.0}));

    EXPECT_FALSE(servo.stateAt(0.9, 0.0).valid);
}

TEST(PixelServo, RejectsInvalidAxisMapping) {
    PixelServoConfig config;
    config.body_x_from_u = 0.0;
    config.body_x_from_v = 0.0;
    config.body_y_from_u = 0.0;
    config.body_y_from_v = 0.0;

    EXPECT_FALSE(pixelServoConfigValid(config));
}

}  // namespace
}  // namespace d_task_uav_control

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
