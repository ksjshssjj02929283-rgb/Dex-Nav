#include <gtest/gtest.h>

#include "rgbd_processing_state.hpp"

namespace dexe_sensor_process {

TEST(FrameDivisionState, EnabledProcessesEveryOtherFrameStartingWithFirst) {
    FrameDivisionState state(true);

    EXPECT_TRUE(state.shouldProcess());
    EXPECT_FALSE(state.shouldProcess());
    EXPECT_TRUE(state.shouldProcess());
    EXPECT_FALSE(state.shouldProcess());
}

TEST(FrameDivisionState, DisabledProcessesEveryFrame) {
    FrameDivisionState state(false);

    EXPECT_TRUE(state.shouldProcess());
    EXPECT_TRUE(state.shouldProcess());
    EXPECT_TRUE(state.shouldProcess());
}

TEST(SourceStampIntervalTracker, FirstValidStampHasUnknownInterval) {
    SourceStampIntervalTracker tracker;

    EXPECT_FLOAT_EQ(tracker.update(1'000'000'000LL), 0.0F);
}

TEST(SourceStampIntervalTracker, UsesAdjacentSuccessfulSourceStamps) {
    SourceStampIntervalTracker tracker;

    EXPECT_FLOAT_EQ(tracker.update(1'000'000'000LL), 0.0F);
    EXPECT_NEAR(tracker.update(1'133'333'333LL), 0.133333333F, 1e-6F);
    EXPECT_NEAR(tracker.update(1'266'666'666LL), 0.133333333F, 1e-6F);
}

TEST(SourceStampIntervalTracker, ZeroStampDoesNotCreateArtificialLongInterval) {
    SourceStampIntervalTracker tracker;

    EXPECT_FLOAT_EQ(tracker.update(0), 0.0F);
    EXPECT_FLOAT_EQ(tracker.update(2'000'000'000LL), 0.0F);
    EXPECT_NEAR(tracker.update(2'100'000'000LL), 0.1F, 1e-6F);
}

TEST(SourceStampIntervalTracker, NonMonotonicStampRestartsBaseline) {
    SourceStampIntervalTracker tracker;

    EXPECT_FLOAT_EQ(tracker.update(3'000'000'000LL), 0.0F);
    EXPECT_FLOAT_EQ(tracker.update(2'000'000'000LL), 0.0F);
    EXPECT_NEAR(tracker.update(2'200'000'000LL), 0.2F, 1e-6F);
}

}  // namespace dexe_sensor_process
