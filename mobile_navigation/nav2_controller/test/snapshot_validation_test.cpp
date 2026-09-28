// Copyright 2026 DEXE
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "nav2_controller/snapshot_validation.hpp"
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
