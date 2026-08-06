// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "nav2_route_planner/costmap_aware_layer.hpp"

using nav2_route_planner::CostmapAwareLayer;

namespace
{
geometry_msgs::msg::PoseStamped pose(double x, double y)
{
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.w = 1.0;
  return p;
}
}  // namespace

TEST(CostmapAvoidanceSwitchTest, DisabledByDefaultAndBlocksThetaStarEntry)
{
  CostmapAwareLayer layer({}, nullptr);
  CostmapAwareLayer::Params params;
  layer.configure(params);

  EXPECT_FALSE(layer.isEnabled());
  EXPECT_TRUE(layer.isEndpointSafe(pose(0.0, 0.0)));

  const auto result = layer.planWithThetaStar(pose(0.0, 0.0), pose(1.0, 0.0));
  EXPECT_FALSE(result.has_value());
  EXPECT_NE(
    layer.describeLastThetaStarFailure().find("disabled"),
    std::string::npos);
}

TEST(CostmapAvoidanceSwitchTest, RuntimeEnableRestoresFailClosedSafety)
{
  CostmapAwareLayer layer({}, nullptr);
  CostmapAwareLayer::Params params;
  layer.configure(params);

  params.enable_route_costmap_avoidance = true;
  layer.updateParams(params);

  EXPECT_TRUE(layer.isEnabled());
  EXPECT_FALSE(layer.isEndpointSafe(pose(0.0, 0.0)));

  const auto result = layer.planWithThetaStar(pose(0.0, 0.0), pose(1.0, 0.0));
  EXPECT_FALSE(result.has_value());
  EXPECT_NE(
    layer.describeLastThetaStarFailure().find("unavailable"),
    std::string::npos);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
