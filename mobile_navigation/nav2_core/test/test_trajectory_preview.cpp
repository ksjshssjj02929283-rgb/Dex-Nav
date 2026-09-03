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
  ASSERT_TRUE(
    nav2_core::sampleTrajectoryPreview(
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
  ASSERT_TRUE(
    nav2_core::sampleTrajectoryPreview(
      points, 0.0, 0.0, false, 0.1, 0.1, preview));

  EXPECT_EQ(preview.mode, nav2_core::TrajectoryPreviewMode::SPATIAL_FALLBACK);
  EXPECT_NEAR(preview.velocity, 0.55, 1e-9);
  // 采样点是首个前方点（target==1）：到达它的那一段是状态戳段（含实测量），
  // 不得用作前馈，因此仍取该点自身的段加速度。
  EXPECT_NEAR(preview.acceleration, -0.20, 1e-9);

  for (auto & point : points) {
    point.relative_time = 0.0;
  }
  EXPECT_FALSE(nav2_core::hasValidRelativeTime(points));
}

// 空间回退取的是窗口内速度极大点。极大点"离开该点"的段加速度必然是制动值，
// 把它当成该点的加速度交出去，消费侧拿到的 (v,a) 自相矛盾（"保持峰值速度的
// 同时最大减速度制动"）——静止起步时会被速度环的符号界永久钳在零指令。
// 与峰值速度自洽的是"到达峰值"那一段，即加速段。
TEST(TrajectoryPreview, SpatialPeakPairsAccelerationOfArrivingSegment)
{
  const std::vector<TrajectoryPreviewPoint> points = {
    {0.00, 0.0, 0.000, 0.5, 0.0},
    {0.02, 0.0, 0.050, 0.5, 0.0},
    {0.04, 0.0, 0.119, -0.4, 0.0},   // T 型峰值：离开该点即制动段
    {0.06, 0.0, 0.060, -0.4, 0.0}};

  nav2_core::TrajectoryPreview preview;
  ASSERT_TRUE(
    nav2_core::sampleTrajectoryPreview(
      points, 0.0, 0.0, false, 0.1, 0.1, preview));

  EXPECT_EQ(preview.mode, nav2_core::TrajectoryPreviewMode::SPATIAL_FALLBACK);
  EXPECT_NEAR(preview.velocity, 0.119, 1e-9);
  EXPECT_NEAR(preview.acceleration, 0.5, 1e-9);
}

}  // namespace
