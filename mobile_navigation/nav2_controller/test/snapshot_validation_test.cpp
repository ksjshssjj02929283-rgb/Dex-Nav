// Copyright 2026 DEXE
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <limits>
#include "nav2_controller/snapshot_validation.hpp"
using nav2_controller::snapshot::validScan;
using nav2_controller::snapshot::usableResult;
TEST(Snapshot, AcceptsStillValidPreviousPlanWhileNextPlanRuns)
{
  EXPECT_TRUE(usableResult(2, 3, 2, 3, true, .2, .3));
}
TEST(Snapshot, RejectsLatePreviousSessionAndPath)
{
  EXPECT_FALSE(usableResult(1, 3, 2, 3, true, .01, .3));
  EXPECT_FALSE(usableResult(2, 2, 2, 3, true, .01, .3));
}
TEST(Snapshot, FailureAndExpiryStopImmediately)
{
  EXPECT_FALSE(usableResult(2, 3, 2, 3, false, .01, .3));
  EXPECT_FALSE(usableResult(2, 3, 2, 3, true, .31, .3));
  EXPECT_FALSE(usableResult(2, 3, 2, 3, true, -1, .3));
}
TEST(Sensors, ClearRaysAreValid)
{
  EXPECT_TRUE(validScan({std::numeric_limits<float>::infinity()}, .1, 5, .01, true, 2, 1, .1, .5));
}
TEST(Sensors, EmptyAndAllInvalidStop)
{
  EXPECT_FALSE(validScan({}, .1, 5, .01, true, 2, 1, .1, .5));
  EXPECT_FALSE(validScan({std::numeric_limits<float>::quiet_NaN(), -1, 0}, .1, 5, .01, true, 2, 1, .1, .5));
}
TEST(Sensors, ReplayedBackwardsFutureAndStaleStampsStop)
{
  EXPECT_FALSE(validScan({1}, .1, 5, .01, true, 2, 2, .1, .5));
  EXPECT_FALSE(validScan({1}, .1, 5, .01, true, 1, 2, .1, .5));
  EXPECT_FALSE(validScan({1}, .1, 5, .01, true, 3, 2, -.1, .5));
  EXPECT_FALSE(validScan({1}, .1, 5, .01, true, 3, 2, .6, .5));
}
TEST(Sensors, InvalidGeometryAndMissingFrameStop)
{
  EXPECT_FALSE(validScan({1}, 5, .1, .01, true, 2, 1, .1, .5));
  EXPECT_FALSE(validScan({1}, .1, 5, 0, true, 2, 1, .1, .5));
  EXPECT_FALSE(validScan({1}, .1, 5, .01, false, 2, 1, .1, .5));
}
