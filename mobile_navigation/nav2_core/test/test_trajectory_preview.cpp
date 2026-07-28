// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <vector>

#include "nav2_core/trajectory_preview.hpp"

namespace
{

using nav2_core::TrajectoryPreviewPoint;

std::vector<TrajectoryPreviewPoint> makeTimedDecline()
{
  return {
    {0.00, 0.0, 0.60, -0.10, 0.00},
    {0.02, 0.0, 0.55, -0.20, 0.04},
    {0.06, 0.0, 0.45, -0.40, 0.12},
    {0.10, 0.0, 0.35, -0.40, 0.20}};
}

TEST(TrajectoryPreview, TimeAlignedProjectionInterpolatesVelocityAndAcceleration)
{
  const auto points = makeTimedDecline();
  ASSERT_TRUE(nav2_core::hasValidRelativeTime(points));

  nav2_core::TrajectoryPreview preview;
  ASSERT_TRUE(nav2_core::sampleTrajectoryPreview(
      points, 0.01, 0.0, true, 0.08, 0.1, preview));

  EXPECT_EQ(preview.mode, nav2_core::TrajectoryPreviewMode::TIME_ALIGNED);
  EXPECT_NEAR(preview.relative_time, 0.10, 1e-9);
  EXPECT_NEAR(preview.velocity, 0.475, 1e-9);
  EXPECT_NEAR(preview.acceleration, -0.35, 1e-9);
}

TEST(TrajectoryPreview, InvalidOrNonMonotonicTimeUsesSpatialFallback)
{
  auto points = makeTimedDecline();
  points[2].relative_time = 0.02;
  ASSERT_FALSE(nav2_core::hasValidRelativeTime(points));

  nav2_core::TrajectoryPreview preview;
  ASSERT_TRUE(nav2_core::sampleTrajectoryPreview(
      points, 0.0, 0.0, false, 0.1, 0.1, preview));

  EXPECT_EQ(preview.mode, nav2_core::TrajectoryPreviewMode::SPATIAL_FALLBACK);
  EXPECT_NEAR(preview.velocity, 0.55, 1e-9);
  EXPECT_NEAR(preview.acceleration, -0.20, 1e-9);

  for (auto & point : points) {
    point.relative_time = 0.0;
  }
  EXPECT_FALSE(nav2_core::hasValidRelativeTime(points));
}

}  // namespace
