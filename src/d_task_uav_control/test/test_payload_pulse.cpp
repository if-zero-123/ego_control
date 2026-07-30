#include <gtest/gtest.h>

#include "d_task_uav_control/payload_pulse.h"

namespace d_task_uav_control {
namespace {

TEST(PayloadPulse, HoldsReleaseForConfiguredDurationThenReturnsNeutral) {
    PayloadPulse pulse(5.0);

    EXPECT_TRUE(pulse.trigger(10.0));
    EXPECT_EQ(pulse.update(10.0), PayloadPulseCommand::RELEASE);
    EXPECT_EQ(pulse.update(14.999), PayloadPulseCommand::RELEASE);
    EXPECT_EQ(pulse.update(15.0), PayloadPulseCommand::NEUTRAL);
    EXPECT_EQ(pulse.update(15.1), PayloadPulseCommand::NONE);
}

TEST(PayloadPulse, IsSingleShotAndResetReturnsNeutral) {
    PayloadPulse pulse(5.0);

    EXPECT_TRUE(pulse.trigger(10.0));
    EXPECT_FALSE(pulse.trigger(11.0));
    EXPECT_EQ(pulse.reset(), PayloadPulseCommand::NEUTRAL);
    EXPECT_EQ(pulse.update(11.0), PayloadPulseCommand::NONE);
    EXPECT_TRUE(pulse.trigger(12.0));
}

}  // namespace
}  // namespace d_task_uav_control

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
