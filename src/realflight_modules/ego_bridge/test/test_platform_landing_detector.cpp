#include <gtest/gtest.h>

#include "ego_bridge/platform_landing_detector.h"

TEST(PlatformLandingDetector, RequiresContinuousContactCondition) {
    ego_bridge::PlatformLandingDetector detector(0.08, 1.0);

    EXPECT_FALSE(detector.update(0.0, 0.01));
    EXPECT_FALSE(detector.update(0.7, 0.01));
    EXPECT_TRUE(detector.update(1.0, 0.01));
}

TEST(PlatformLandingDetector, ResetsWhenVerticalMotionReturns) {
    ego_bridge::PlatformLandingDetector detector(0.08, 1.0);

    EXPECT_FALSE(detector.update(0.0, 0.01));
    EXPECT_FALSE(detector.update(0.8, 0.20));
    EXPECT_FALSE(detector.update(1.4, 0.01));
    EXPECT_TRUE(detector.update(2.4, 0.01));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
