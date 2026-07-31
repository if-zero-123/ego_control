#include <gtest/gtest.h>

#include <cmath>

#include "d_task_uav_control/mission_controller.h"

namespace d_task_uav_control {
namespace {

MissionControllerConfig fastConfig() {
    MissionControllerConfig config;
    config.hover_time_s = 0.0;
    config.vision_lock_time_s = 0.0;
    config.follow_stable_time_s = 0.0;
    config.phase_stable_time_s = 0.0;
    config.release_settle_time_s = 0.0;
    config.platform_hold_time_s = 5.2;
    config.cruise_height_m = 1.5;
    config.drop_height_m = 0.3;
    config.drop_apriltag_distance_m = 0.3;
    config.high_descent_height_m = 0.8;
    config.low_descent_height_m = 0.3;
    config.xy_tolerance_m = 0.12;
    config.relative_speed_tolerance_mps = 0.20;
    config.height_tolerance_m = 0.06;
    config.follow_max_accel_mps2 = 0.0;
    return config;
}

MissionInput nominalInput() {
    MissionInput input;
    input.now_s = 10.0;
    input.uav_valid = true;
    input.uav_x = 1.0;
    input.uav_y = 2.0;
    input.uav_z = 1.5;
    input.bridge_state = "HOVER";
    input.control_mode = 1;
    input.platform_valid = true;
    input.platform_vision_detected = true;
    input.pixel_valid = true;
    input.pixel_aligned = true;
    input.apriltag_range_valid = true;
    input.apriltag_center_tag_visible = true;
    input.apriltag_plane_distance_m = 0.8;
    input.platform_x = 1.0;
    input.platform_y = 2.0;
    input.platform_z = 0.3;
    input.descent_allowed = true;
    input.distance_to_d_m = 5.0;
    return input;
}

void prepareAndStart(MissionController& controller, MissionMode mode) {
    ASSERT_TRUE(controller.configure("mission-1", mode));
    ASSERT_TRUE(controller.markPositioningReady(
        "mission-1", HomePosition{0.0, 0.0, 0.0, 0.0}));
    ASSERT_TRUE(controller.start("mission-1", mode, "car_button", 10.0));
}

void advanceToFollow(MissionController& controller, MissionInput& input) {
    for (int index = 0; index < 12
         && controller.state() != MissionState::FOLLOW_CAR; ++index) {
        if (controller.state() == MissionState::MOVE_TO_SEARCH_START) {
            input.uav_x = 0.746;
            input.uav_y = -0.379;
            input.uav_z = 1.5;
        }
        input.now_s += 0.01;
        controller.update(input);
    }
    ASSERT_EQ(controller.state(), MissionState::FOLLOW_CAR);
    input.uav_x = input.platform_x;
    input.uav_y = input.platform_y;
}

void advanceToLock(MissionController& controller, MissionInput& input) {
    for (int index = 0; index < 12
         && controller.state() != MissionState::LOCK_CAR; ++index) {
        if (controller.state() == MissionState::MOVE_TO_SEARCH_START) {
            input.uav_x = 0.746;
            input.uav_y = -0.379;
            input.uav_z = 1.5;
        }
        input.now_s += 0.01;
        controller.update(input);
    }
    ASSERT_EQ(controller.state(), MissionState::LOCK_CAR);
}

TEST(MissionController, DoesNotStartBeforePositioningReady) {
    MissionController controller(fastConfig());
    ASSERT_TRUE(controller.configure("mission-1", MissionMode::DROP));

    EXPECT_FALSE(controller.start(
        "mission-1", MissionMode::DROP, "car_button", 10.0));
    EXPECT_EQ(controller.state(), MissionState::POSITIONING_INIT);
}

TEST(MissionController, RejectsUnknownStartReason) {
    MissionController controller(fastConfig());
    ASSERT_TRUE(controller.configure("mission-1", MissionMode::DROP));
    ASSERT_TRUE(controller.markPositioningReady(
        "mission-1", HomePosition{0.0, 0.0, 0.0, 0.0}));

    EXPECT_FALSE(controller.start(
        "mission-1", MissionMode::DROP, "ground_command", 10.0));
    EXPECT_EQ(controller.state(), MissionState::WAIT_START);
}

TEST(MissionController, AcceptsGroundWebStartRelayedByCar) {
    MissionController controller(fastConfig());
    ASSERT_TRUE(controller.configure("mission-1", MissionMode::DROP));
    ASSERT_TRUE(controller.markPositioningReady(
        "mission-1", HomePosition{0.0, 0.0, 0.0, 0.0}));

    EXPECT_TRUE(controller.start(
        "mission-1", MissionMode::DROP, "ground_web", 10.0));
    EXPECT_EQ(controller.state(), MissionState::TAKEOFF);
}

TEST(MissionController, RequestsConfiguredHeightRelativeToHome) {
    MissionController controller(fastConfig());
    ASSERT_TRUE(controller.configure("mission-1", MissionMode::DROP));
    ASSERT_TRUE(controller.markPositioningReady(
        "mission-1", HomePosition{0.0, 0.0, 0.2, 0.0}));
    ASSERT_TRUE(controller.start(
        "mission-1", MissionMode::DROP, "car_button", 10.0));
    MissionInput input = nominalInput();
    input.bridge_state = "IDLE";

    const MissionCommand command = controller.update(input);

    ASSERT_TRUE(command.request_takeoff);
    EXPECT_DOUBLE_EQ(command.target_z, 1.7);
}

TEST(MissionController, DropMovesToFixedSearchStartAfterHover) {
    MissionController controller(fastConfig());
    prepareAndStart(controller, MissionMode::DROP);
    MissionInput input = nominalInput();
    input.platform_valid = false;
    input.platform_vision_detected = false;
    input.pixel_valid = false;

    controller.update(input);
    input.now_s += 0.01;
    controller.update(input);
    input.now_s += 0.01;
    const MissionCommand command = controller.update(input);

    ASSERT_TRUE(command.setpoint_valid);
    EXPECT_DOUBLE_EQ(command.target_x, 0.746);
    EXPECT_DOUBLE_EQ(command.target_y, -0.379);
    EXPECT_DOUBLE_EQ(command.target_z, 1.5);
    EXPECT_DOUBLE_EQ(command.target_vx, 0.0);
    EXPECT_DOUBLE_EQ(command.target_vy, 0.0);
}

TEST(MissionController, DropSearchesOnePointFiveMetresAlongPositiveX) {
    MissionController controller(fastConfig());
    prepareAndStart(controller, MissionMode::DROP);
    MissionInput input = nominalInput();
    input.platform_valid = false;
    input.platform_vision_detected = false;
    input.pixel_valid = false;

    controller.update(input);
    input.now_s += 0.01;
    controller.update(input);
    ASSERT_EQ(controller.state(), MissionState::MOVE_TO_SEARCH_START);

    input.uav_x = 0.746;
    input.uav_y = -0.379;
    input.uav_z = 1.5;
    input.now_s += 0.01;
    controller.update(input);
    ASSERT_EQ(controller.state(), MissionState::FORWARD_SEARCH);

    input.now_s += 0.01;
    const MissionCommand command = controller.update(input);
    ASSERT_TRUE(command.setpoint_valid);
    EXPECT_DOUBLE_EQ(command.target_x, 2.246);
    EXPECT_DOUBLE_EQ(command.target_y, -0.379);
    EXPECT_DOUBLE_EQ(command.target_z, 1.5);
}

TEST(MissionController, DropLocksPlatformDetectedDuringForwardSearch) {
    MissionController controller(fastConfig());
    prepareAndStart(controller, MissionMode::DROP);
    MissionInput input = nominalInput();

    controller.update(input);
    input.now_s += 0.01;
    controller.update(input);
    input.uav_x = 0.746;
    input.uav_y = -0.379;
    input.uav_z = 1.5;
    input.now_s += 0.01;
    controller.update(input);
    ASSERT_EQ(controller.state(), MissionState::FORWARD_SEARCH);

    input.uav_x = 1.2;
    input.platform_x = 1.2;
    input.platform_y = -0.379;
    input.now_s += 0.01;
    controller.update(input);

    EXPECT_EQ(controller.state(), MissionState::LOCK_CAR);
}

TEST(MissionController, DropReturnsHomeWhenSearchEndpointHasNoDetection) {
    MissionController controller(fastConfig());
    prepareAndStart(controller, MissionMode::DROP);
    MissionInput input = nominalInput();
    input.platform_valid = false;
    input.platform_vision_detected = false;
    input.pixel_valid = false;

    controller.update(input);
    input.now_s += 0.01;
    controller.update(input);
    input.uav_x = 0.746;
    input.uav_y = -0.379;
    input.uav_z = 1.5;
    input.now_s += 0.01;
    controller.update(input);
    ASSERT_EQ(controller.state(), MissionState::FORWARD_SEARCH);

    input.uav_x = 2.246;
    input.now_s += 0.01;
    const MissionCommand command = controller.update(input);

    EXPECT_EQ(controller.state(), MissionState::RETURN_HOME);
    EXPECT_EQ(command.fault_code, 2104);
    EXPECT_EQ(command.fault_text, "platform_not_found_on_search_path");
}

TEST(MissionController, DropReleasesAtCruiseHeightWithOuterTags) {
    MissionController controller(fastConfig());
    prepareAndStart(controller, MissionMode::DROP);
    MissionInput input = nominalInput();
    advanceToFollow(controller, input);

    input.uav_z = fastConfig().cruise_height_m;
    input.apriltag_range_valid = false;
    input.apriltag_center_tag_visible = false;
    input.now_s += 0.1;
    controller.update(input);
    ASSERT_EQ(controller.state(), MissionState::RELEASE);

    input.now_s += 0.01;
    const MissionCommand command = controller.update(input);
    EXPECT_TRUE(command.release_payload);
    EXPECT_LT(input.distance_to_d_m, 10.0);
}

TEST(MissionController, DropDoesNotReleaseWithoutPlatformAlignment) {
    MissionController controller(fastConfig());
    prepareAndStart(controller, MissionMode::DROP);
    MissionInput input = nominalInput();
    advanceToFollow(controller, input);

    input.platform_x += 0.20;
    input.now_s += 0.1;
    const MissionCommand command = controller.update(input);

    EXPECT_EQ(controller.state(), MissionState::FOLLOW_CAR);
    ASSERT_TRUE(command.setpoint_valid);
    EXPECT_DOUBLE_EQ(command.target_vz, 0.0);
    EXPECT_FALSE(command.release_payload);
}

TEST(MissionController, DropDoesNotReleaseBeforeCruiseHeight) {
    MissionController controller(fastConfig());
    prepareAndStart(controller, MissionMode::DROP);
    MissionInput input = nominalInput();
    advanceToFollow(controller, input);

    input.uav_z = 1.20;
    input.now_s += 0.1;
    const MissionCommand command = controller.update(input);

    EXPECT_EQ(controller.state(), MissionState::FOLLOW_CAR);
    EXPECT_FALSE(command.release_payload);
}

TEST(MissionController, DropDoesNotRequireCenterTagOrRange) {
    MissionController controller(fastConfig());
    prepareAndStart(controller, MissionMode::DROP);
    MissionInput input = nominalInput();
    advanceToFollow(controller, input);

    input.uav_z = fastConfig().cruise_height_m;
    input.apriltag_range_valid = false;
    input.apriltag_center_tag_visible = false;
    input.now_s += 0.1;
    controller.update(input);

    EXPECT_EQ(controller.state(), MissionState::RELEASE);
}

TEST(MissionController, DropKeepsCruiseHeightBeforeRelease) {
    MissionController controller(fastConfig());
    prepareAndStart(controller, MissionMode::DROP);
    MissionInput input = nominalInput();
    advanceToFollow(controller, input);

    input.uav_z = fastConfig().cruise_height_m;
    input.apriltag_center_tag_visible = false;
    input.apriltag_range_valid = false;
    input.now_s += 0.1;
    const MissionCommand command = controller.update(input);

    EXPECT_EQ(controller.state(), MissionState::RELEASE);
    EXPECT_DOUBLE_EQ(command.target_z, input.uav_z);
    EXPECT_DOUBLE_EQ(command.target_vz, 0.0);
    EXPECT_FALSE(command.release_payload);
}

TEST(MissionController, DropAlignmentStabilityUsesCruiseHeight) {
    MissionControllerConfig config = fastConfig();
    config.follow_stable_time_s = 0.50;
    MissionController controller(config);
    prepareAndStart(controller, MissionMode::DROP);
    MissionInput input = nominalInput();
    advanceToFollow(controller, input);

    input.uav_z = config.cruise_height_m;
    input.apriltag_center_tag_visible = false;
    input.apriltag_range_valid = false;
    input.now_s += 0.1;
    MissionCommand command = controller.update(input);

    ASSERT_TRUE(command.setpoint_valid);
    EXPECT_DOUBLE_EQ(command.target_z, input.uav_z);
    EXPECT_DOUBLE_EQ(command.target_vz, 0.0);
    EXPECT_EQ(controller.state(), MissionState::FOLLOW_CAR);

    input.now_s += 0.49;
    command = controller.update(input);
    EXPECT_DOUBLE_EQ(command.target_z, input.uav_z);
    EXPECT_DOUBLE_EQ(command.target_vz, 0.0);
    EXPECT_EQ(controller.state(), MissionState::FOLLOW_CAR);

    input.now_s += 0.01;
    controller.update(input);
    EXPECT_EQ(controller.state(), MissionState::RELEASE);
}

TEST(MissionController, StaleCarPoseFreezesDynamicDescentAltitude) {
    MissionController controller(fastConfig());
    prepareAndStart(controller, MissionMode::DYNAMIC_LANDING);
    MissionInput input = nominalInput();
    advanceToFollow(controller, input);
    controller.update(input);
    ASSERT_EQ(controller.state(), MissionState::DESCEND_HIGH);

    input.uav_z = 1.35;
    input.descent_allowed = false;
    input.safety_hold = true;
    input.now_s += 0.1;
    const MissionCommand command = controller.update(input);

    ASSERT_TRUE(command.setpoint_valid);
    EXPECT_DOUBLE_EQ(command.target_z, input.uav_z);
    EXPECT_DOUBLE_EQ(command.target_vz, 0.0);
    EXPECT_EQ(controller.state(), MissionState::DESCEND_HIGH);
}

TEST(MissionController, PixelLossFreezesDynamicDescentImmediately) {
    MissionController controller(fastConfig());
    prepareAndStart(controller, MissionMode::DYNAMIC_LANDING);
    MissionInput input = nominalInput();
    advanceToFollow(controller, input);
    controller.update(input);
    ASSERT_EQ(controller.state(), MissionState::DESCEND_HIGH);

    input.pixel_valid = false;
    input.now_s += 0.1;
    const MissionCommand command = controller.update(input);

    ASSERT_TRUE(command.setpoint_valid);
    EXPECT_DOUBLE_EQ(command.target_x, input.uav_x);
    EXPECT_DOUBLE_EQ(command.target_y, input.uav_y);
    EXPECT_DOUBLE_EQ(command.target_z, input.uav_z);
    EXPECT_DOUBLE_EQ(command.target_vz, 0.0);
    EXPECT_EQ(controller.state(), MissionState::DESCEND_HIGH);
}

TEST(MissionController, LowDescentRequiresCenterTagBeforeFinalLanding) {
    MissionController controller(fastConfig());
    prepareAndStart(controller, MissionMode::DYNAMIC_LANDING);
    MissionInput input = nominalInput();
    advanceToFollow(controller, input);
    controller.update(input);
    ASSERT_EQ(controller.state(), MissionState::DESCEND_HIGH);
    input.uav_z = input.platform_z + fastConfig().high_descent_height_m;
    input.now_s += 0.1;
    controller.update(input);
    ASSERT_EQ(controller.state(), MissionState::DESCEND_LOW);

    input.uav_z = input.platform_z + fastConfig().low_descent_height_m;
    input.apriltag_center_tag_visible = false;
    input.now_s += 0.1;
    controller.update(input);

    EXPECT_EQ(controller.state(), MissionState::DESCEND_LOW);
}

TEST(MissionController, PredictsMovingPlatformAfterFinalCenterTagLoss) {
    MissionController controller(fastConfig());
    prepareAndStart(controller, MissionMode::DYNAMIC_LANDING);
    MissionInput input = nominalInput();
    advanceToFollow(controller, input);
    controller.update(input);
    input.uav_z = input.platform_z + fastConfig().high_descent_height_m;
    input.now_s += 0.1;
    controller.update(input);
    input.uav_z = input.platform_z + fastConfig().low_descent_height_m;
    input.now_s += 0.1;
    controller.update(input);
    ASSERT_EQ(controller.state(), MissionState::LAND_ON_PLATFORM);

    input.platform_vx = 0.22;
    input.platform_vy = -0.08;
    input.pixel_valid = false;
    input.pixel_aligned = false;
    input.now_s += 0.1;
    const MissionCommand command = controller.update(input);

    ASSERT_TRUE(command.setpoint_valid);
    EXPECT_TRUE(command.request_platform_land);
    EXPECT_NEAR(command.target_vx, input.platform_vx, 1e-9);
    EXPECT_NEAR(command.target_vy, input.platform_vy, 1e-9);
    EXPECT_LT(command.target_vz, 0.0);
    EXPECT_EQ(controller.state(), MissionState::LAND_ON_PLATFORM);
}

TEST(MissionController, CancelsBlindLandingWhenCarPoseBecomesStale) {
    MissionController controller(fastConfig());
    prepareAndStart(controller, MissionMode::DYNAMIC_LANDING);
    MissionInput input = nominalInput();
    advanceToFollow(controller, input);
    controller.update(input);
    input.uav_z = input.platform_z + fastConfig().high_descent_height_m;
    input.now_s += 0.1;
    controller.update(input);
    input.uav_z = input.platform_z + fastConfig().low_descent_height_m;
    input.now_s += 0.1;
    controller.update(input);
    ASSERT_EQ(controller.state(), MissionState::LAND_ON_PLATFORM);

    input.bridge_state = "PLATFORM_LANDING";
    input.pixel_valid = false;
    input.descent_allowed = false;
    input.safety_hold = true;
    input.now_s += 0.1;
    const MissionCommand command = controller.update(input);

    EXPECT_TRUE(command.request_platform_cancel);
    EXPECT_EQ(controller.state(), MissionState::CLIMB_TO_CRUISE);
}

TEST(MissionController, CancelsBlindLandingAtPredictionTimeout) {
    MissionControllerConfig config = fastConfig();
    config.landing_prediction_timeout_s = 0.50;
    MissionController controller(config);
    prepareAndStart(controller, MissionMode::DYNAMIC_LANDING);
    MissionInput input = nominalInput();
    advanceToFollow(controller, input);
    controller.update(input);
    input.uav_z = input.platform_z + config.high_descent_height_m;
    input.now_s += 0.1;
    controller.update(input);
    input.uav_z = input.platform_z + config.low_descent_height_m;
    input.now_s += 0.1;
    controller.update(input);
    ASSERT_EQ(controller.state(), MissionState::LAND_ON_PLATFORM);

    input.bridge_state = "PLATFORM_LANDING";
    input.pixel_valid = false;
    input.now_s += 0.1;
    controller.update(input);
    input.now_s += 0.51;
    const MissionCommand command = controller.update(input);

    EXPECT_TRUE(command.request_platform_cancel);
    EXPECT_EQ(controller.state(), MissionState::CLIMB_TO_CRUISE);
}

TEST(MissionController, PixelFollowAddsCorrectionToCarVelocity) {
    MissionController controller(fastConfig());
    prepareAndStart(controller, MissionMode::DYNAMIC_LANDING);
    MissionInput input = nominalInput();
    advanceToFollow(controller, input);
    input.platform_vx = 0.20;
    input.platform_vy = -0.10;
    input.pixel_world_vx = 0.10;
    input.pixel_world_vy = 0.05;

    const MissionCommand command = controller.update(input);

    ASSERT_TRUE(command.setpoint_valid);
    // 0.20 car feed-forward + 0.25*0.20 velocity D + 0.10 visual trim.
    // The lead position error remains inside the 0.04 m deadband.
    EXPECT_NEAR(command.target_vx, 0.35, 1e-9);
    EXPECT_NEAR(command.target_vy, -0.075, 1e-9);
    EXPECT_NEAR(command.target_x, input.uav_x, 1e-9);
    EXPECT_NEAR(command.target_y, input.uav_y, 1e-9);
}

TEST(MissionController, LockUsesPixelCorrectionBeforePixelAlignment) {
    MissionController controller(fastConfig());
    prepareAndStart(controller, MissionMode::DYNAMIC_LANDING);
    MissionInput input = nominalInput();
    input.pixel_aligned = false;
    input.pixel_world_vx = 0.10;
    input.pixel_world_vy = -0.05;
    advanceToLock(controller, input);

    const MissionCommand command = controller.update(input);

    ASSERT_TRUE(command.setpoint_valid);
    EXPECT_NEAR(command.target_vx, 0.10, 1e-9);
    EXPECT_NEAR(command.target_vy, -0.05, 1e-9);
    EXPECT_EQ(controller.state(), MissionState::LOCK_CAR);
}

TEST(MissionController, DynamicSearchUsesCoordinatePdBeforeAprilTagIsVisible) {
    MissionControllerConfig config = fastConfig();
    config.follow_position_deadband_m = 0.0;
    config.follow_max_correction_mps = 10.0;
    config.follow_max_total_speed_mps = 10.0;
    MissionController controller(config);
    prepareAndStart(controller, MissionMode::DYNAMIC_LANDING);
    MissionInput input = nominalInput();
    input.pixel_valid = false;
    input.pixel_aligned = false;
    input.platform_x = 1.50;
    input.platform_vx = 0.20;

    MissionCommand command;
    for (int index = 0; index < 5; ++index) {
        input.now_s += 0.01;
        command = controller.update(input);
    }

    ASSERT_EQ(controller.state(), MissionState::SEARCH_CAR);
    ASSERT_TRUE(command.setpoint_valid);
    // lead error = 0.50 + 0.20*0.15 = 0.53
    // v = 0.20 + 0.80*0.53 + 0.25*(0.20 - 0.0) = 0.674
    EXPECT_NEAR(command.target_vx, 0.674, 1e-9);
    EXPECT_NEAR(command.target_vy, 0.0, 1e-9);
    EXPECT_NEAR(command.target_x, input.uav_x, 1e-9);
    EXPECT_NEAR(command.target_y, input.uav_y, 1e-9);
    EXPECT_NEAR(command.target_z, config.cruise_height_m, 1e-9);
}

TEST(MissionController, FollowPdClampsTotalHorizontalSpeed) {
    MissionControllerConfig config = fastConfig();
    config.follow_position_deadband_m = 0.0;
    config.follow_max_correction_mps = 0.30;
    config.follow_max_total_speed_mps = 0.50;
    MissionController controller(config);
    prepareAndStart(controller, MissionMode::DYNAMIC_LANDING);
    MissionInput input = nominalInput();
    input.pixel_valid = false;
    input.pixel_aligned = false;
    input.platform_x = 4.0;
    input.platform_vx = 0.40;

    MissionCommand command;
    for (int index = 0; index < 5; ++index) {
        input.now_s += 0.01;
        command = controller.update(input);
    }

    ASSERT_TRUE(command.setpoint_valid);
    EXPECT_NEAR(std::hypot(command.target_vx, command.target_vy), 0.50, 1e-9);
}

TEST(MissionController, FollowPdLimitsAccelerationBetweenCommands) {
    MissionControllerConfig config = fastConfig();
    config.follow_position_deadband_m = 0.0;
    config.follow_max_correction_mps = 10.0;
    config.follow_max_total_speed_mps = 10.0;
    config.follow_max_accel_mps2 = 0.50;
    MissionController controller(config);
    prepareAndStart(controller, MissionMode::DYNAMIC_LANDING);
    MissionInput input = nominalInput();
    input.pixel_valid = false;
    input.pixel_aligned = false;

    MissionCommand command;
    for (int index = 0; index < 5; ++index) {
        input.now_s += 0.01;
        command = controller.update(input);
    }
    ASSERT_NEAR(command.target_vx, 0.0, 1e-9);

    input.platform_x = 3.0;
    input.now_s += 0.10;
    command = controller.update(input);

    ASSERT_TRUE(command.setpoint_valid);
    EXPECT_NEAR(std::hypot(command.target_vx, command.target_vy), 0.05, 1e-9);
}

TEST(MissionController, DropWaitsForCoordinateAndVelocityAlignment) {
    MissionController controller(fastConfig());
    prepareAndStart(controller, MissionMode::DROP);
    MissionInput input = nominalInput();
    advanceToFollow(controller, input);
    input.platform_x += 0.30;

    controller.update(input);

    EXPECT_EQ(controller.state(), MissionState::FOLLOW_CAR);
}

TEST(MissionController, ForceDropDoesNotBypassPixelLock) {
    MissionController controller(fastConfig());
    prepareAndStart(controller, MissionMode::DROP);
    MissionInput input = nominalInput();
    advanceToFollow(controller, input);
    input.pixel_valid = false;
    input.pixel_aligned = false;
    input.distance_to_d_m = 1.0;

    controller.update(input);

    EXPECT_EQ(controller.state(), MissionState::FOLLOW_CAR);
}

TEST(MissionController, ReleaseKeepsPixelCorrectionWhileVisible) {
    MissionController controller(fastConfig());
    prepareAndStart(controller, MissionMode::DROP);
    MissionInput input = nominalInput();
    advanceToFollow(controller, input);
    controller.update(input);
    ASSERT_EQ(controller.state(), MissionState::RELEASE);
    input.pixel_world_vx = 0.10;
    input.pixel_world_vy = 0.05;
    input.now_s += 0.01;

    const MissionCommand command = controller.update(input);

    EXPECT_NEAR(command.target_vx, 0.10, 1e-9);
    EXPECT_NEAR(command.target_vy, 0.05, 1e-9);
}

TEST(MissionController, HoldsOnPlatformForFullFivePointTwoSeconds) {
    MissionController controller(fastConfig());
    prepareAndStart(controller, MissionMode::DYNAMIC_LANDING);
    MissionInput input = nominalInput();
    advanceToFollow(controller, input);
    controller.update(input);
    ASSERT_EQ(controller.state(), MissionState::DESCEND_HIGH);

    input.uav_z = input.platform_z + fastConfig().high_descent_height_m;
    input.now_s += 0.1;
    controller.update(input);
    ASSERT_EQ(controller.state(), MissionState::DESCEND_LOW);
    input.uav_z = input.platform_z + fastConfig().low_descent_height_m;
    input.now_s += 0.1;
    controller.update(input);
    ASSERT_EQ(controller.state(), MissionState::LAND_ON_PLATFORM);

    input.bridge_state = "PLATFORM_LANDED";
    input.now_s = 20.0;
    controller.update(input);
    ASSERT_EQ(controller.state(), MissionState::PLATFORM_HOLD);

    input.now_s = 25.19;
    controller.update(input);
    EXPECT_EQ(controller.state(), MissionState::PLATFORM_HOLD);
    input.now_s = 25.20;
    controller.update(input);
    EXPECT_EQ(controller.state(), MissionState::PLATFORM_TAKEOFF);
    input.now_s += 0.01;
    EXPECT_TRUE(controller.update(input).request_platform_takeoff);
}

TEST(MissionController, CancelsContactAndRetriesDynamicLandingOnce) {
    MissionControllerConfig config = fastConfig();
    config.platform_contact_timeout_s = 1.0;
    MissionController controller(config);
    prepareAndStart(controller, MissionMode::DYNAMIC_LANDING);
    MissionInput input = nominalInput();
    advanceToFollow(controller, input);
    controller.update(input);
    input.uav_z = input.platform_z + config.high_descent_height_m;
    input.now_s += 0.1;
    controller.update(input);
    input.uav_z = input.platform_z + config.low_descent_height_m;
    input.now_s += 0.1;
    controller.update(input);
    ASSERT_EQ(controller.state(), MissionState::LAND_ON_PLATFORM);

    input.bridge_state = "PLATFORM_LANDING";
    input.now_s += 1.01;
    const MissionCommand command = controller.update(input);

    EXPECT_TRUE(command.request_platform_cancel);
    EXPECT_EQ(controller.retryCount(), 1);
    EXPECT_EQ(controller.state(), MissionState::CLIMB_TO_CRUISE);
    input.bridge_state = "HOVER";
    input.uav_z = config.cruise_height_m;
    input.now_s += 0.1;
    controller.update(input);
    EXPECT_EQ(controller.state(), MissionState::SEARCH_CAR);
}

TEST(MissionController, AirbornePlatformTakeoffTimeoutContinuesAbortReturn) {
    MissionControllerConfig config = fastConfig();
    config.platform_takeoff_timeout_s = 1.0;
    MissionController controller(config);
    prepareAndStart(controller, MissionMode::DYNAMIC_LANDING);
    MissionInput input = nominalInput();
    advanceToFollow(controller, input);
    controller.update(input);
    input.uav_z = input.platform_z + config.high_descent_height_m;
    input.now_s += 0.1;
    controller.update(input);
    input.uav_z = input.platform_z + config.low_descent_height_m;
    input.now_s += 0.1;
    controller.update(input);
    input.bridge_state = "PLATFORM_LANDED";
    input.now_s += 0.1;
    controller.update(input);
    ASSERT_EQ(controller.state(), MissionState::PLATFORM_HOLD);
    input.now_s += config.platform_hold_time_s;
    controller.update(input);
    ASSERT_EQ(controller.state(), MissionState::PLATFORM_TAKEOFF);

    input.bridge_state = "PLATFORM_TAKEOFF";
    input.now_s += config.platform_takeoff_timeout_s + 0.01;
    const MissionCommand command = controller.update(input);

    EXPECT_EQ(controller.state(), MissionState::CLIMB_TO_CRUISE);
    EXPECT_FALSE(command.abort);
    EXPECT_EQ(command.fault_code, 2204);
}

}  // namespace
}  // namespace d_task_uav_control

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
