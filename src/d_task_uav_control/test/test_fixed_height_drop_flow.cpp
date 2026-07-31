#include <gtest/gtest.h>

#include "d_task_uav_control/fixed_height_drop_flow.h"

namespace d_task_uav_control {
namespace {

FixedHeightDropFlow readyFlow(const FixedHeightDropFlowConfig& config) {
    FixedHeightDropFlow flow(config);
    flow.configure();
    EXPECT_TRUE(flow.markPositioningReady());
    EXPECT_TRUE(flow.start(1.0));
    return flow;
}

void advanceToFollow(FixedHeightDropFlow& flow,
                     FixedHeightDropFlowInput& input) {
    input.now_s = 1.1;
    input.takeoff_complete = true;
    flow.update(input);
    ASSERT_EQ(flow.state(), FixedHeightDropState::MOVE_TO_SEARCH_START);
    input.takeoff_complete = false;
    input.offset_reached = true;
    input.now_s = 1.2;
    flow.update(input);
    ASSERT_EQ(flow.state(), FixedHeightDropState::FORWARD_SEARCH);
    input.offset_reached = false;
    input.tag_detected = true;
    input.now_s = 1.3;
    flow.update(input);
    ASSERT_EQ(flow.state(), FixedHeightDropState::FOLLOW_CAR);
    input.tag_detected = false;
}

TEST(FixedHeightDropFlow, CompletesEventDrivenDropReturnAndLanding) {
    FixedHeightDropFlowConfig config;
    config.alignment_stable_s = 1.0;
    config.release_duration_s = 5.0;
    config.release_settle_s = 0.25;
    config.home_stable_s = 0.5;
    FixedHeightDropFlow flow = readyFlow(config);
    FixedHeightDropFlowInput input;
    advanceToFollow(flow, input);

    input.aligned = true;
    input.now_s = 2.0;
    EXPECT_FALSE(flow.update(input).trigger_payload);
    input.now_s = 3.0;
    const FixedHeightDropFlowOutput release = flow.update(input);
    EXPECT_TRUE(release.trigger_payload);
    EXPECT_EQ(flow.state(), FixedHeightDropState::RELEASE);

    input.aligned = false;
    input.now_s = 8.24;
    flow.update(input);
    EXPECT_EQ(flow.state(), FixedHeightDropState::RELEASE);
    input.now_s = 8.25;
    flow.update(input);
    EXPECT_EQ(flow.state(), FixedHeightDropState::RETURN_HOME);

    input.home_reached = true;
    input.now_s = 9.0;
    flow.update(input);
    input.now_s = 9.5;
    flow.update(input);
    EXPECT_EQ(flow.state(), FixedHeightDropState::LAND_HOME);

    input.home_reached = false;
    input.landed = true;
    input.now_s = 9.6;
    const FixedHeightDropFlowOutput complete = flow.update(input);
    EXPECT_TRUE(complete.terminal);
    EXPECT_FALSE(complete.aborted);
    EXPECT_EQ(flow.state(), FixedHeightDropState::COMPLETE);
}

TEST(FixedHeightDropFlow, AlignmentLossRestartsStableWindow) {
    FixedHeightDropFlowConfig config;
    config.alignment_stable_s = 1.0;
    FixedHeightDropFlow flow = readyFlow(config);
    FixedHeightDropFlowInput input;
    advanceToFollow(flow, input);

    input.aligned = true;
    input.now_s = 2.0;
    flow.update(input);
    input.aligned = false;
    input.now_s = 2.8;
    flow.update(input);
    input.aligned = true;
    input.now_s = 3.0;
    flow.update(input);
    input.now_s = 3.9;
    flow.update(input);
    EXPECT_EQ(flow.state(), FixedHeightDropState::FOLLOW_CAR);
    input.now_s = 4.0;
    EXPECT_TRUE(flow.update(input).trigger_payload);
    EXPECT_EQ(flow.state(), FixedHeightDropState::RELEASE);
}

TEST(FixedHeightDropFlow, MissingTagAtSearchEndReturnsAndAborts) {
    FixedHeightDropFlowConfig config;
    config.home_stable_s = 0.0;
    FixedHeightDropFlow flow = readyFlow(config);
    FixedHeightDropFlowInput input;
    input.now_s = 1.1;
    input.takeoff_complete = true;
    flow.update(input);
    input.takeoff_complete = false;
    input.offset_reached = true;
    input.now_s = 1.2;
    flow.update(input);
    input.offset_reached = false;
    input.search_endpoint_reached = true;
    input.now_s = 1.3;
    const FixedHeightDropFlowOutput failed = flow.update(input);

    EXPECT_EQ(flow.state(), FixedHeightDropState::RETURN_HOME);
    EXPECT_EQ(failed.fault_code, 2104);
    EXPECT_STREQ(failed.fault_text, "tag_not_found_on_search_path");
    EXPECT_TRUE(flow.finalAbort());

    input.search_endpoint_reached = false;
    input.home_reached = true;
    input.now_s = 1.4;
    flow.update(input);
    EXPECT_EQ(flow.state(), FixedHeightDropState::LAND_HOME);
    input.landed = true;
    input.now_s = 1.5;
    const FixedHeightDropFlowOutput terminal = flow.update(input);
    EXPECT_TRUE(terminal.terminal);
    EXPECT_TRUE(terminal.aborted);
    EXPECT_EQ(flow.state(), FixedHeightDropState::ABORT);
}

TEST(FixedHeightDropFlow, AbortWhileAirborneReturnsHome) {
    FixedHeightDropFlowConfig config;
    FixedHeightDropFlow flow = readyFlow(config);
    FixedHeightDropFlowInput input;
    advanceToFollow(flow, input);
    input.abort_requested = true;
    input.now_s = 2.0;

    const FixedHeightDropFlowOutput output = flow.update(input);

    EXPECT_EQ(flow.state(), FixedHeightDropState::RETURN_HOME);
    EXPECT_TRUE(flow.finalAbort());
    EXPECT_EQ(output.fault_code, 2303);
}

TEST(FixedHeightDropFlow, SameFlowCanBeConfiguredAgainAfterTerminal) {
    FixedHeightDropFlow flow;
    flow.configure();
    EXPECT_EQ(flow.state(), FixedHeightDropState::POSITIONING_INIT);
    EXPECT_TRUE(flow.markPositioningReady());
    EXPECT_TRUE(flow.start(1.0));
    flow.configure();
    EXPECT_EQ(flow.state(), FixedHeightDropState::POSITIONING_INIT);
    EXPECT_FALSE(flow.finalAbort());
}

}  // namespace
}  // namespace d_task_uav_control

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
