// Copyright (c) 2026 lovebc
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/esdf_layer.hpp"

namespace nav2_costmap_2d
{

class EsdfLayerTester : public ::testing::Test
{
public:
  std::vector<float> computeDistanceTransform(const Costmap2D & costmap)
  {
    return layer_.computeDistanceTransform(costmap);
  }

  unsigned char distanceToCost(double distance_m)
  {
    return layer_.distanceToCost(distance_m);
  }

  void configureDistanceCost(
    double inscribed_radius,
    double influence_radius,
    double cost_scaling_factor,
    bool use_exponential_decay)
  {
    layer_.inscribed_radius_ = inscribed_radius;
    layer_.influence_radius_ = influence_radius;
    layer_.cost_scaling_factor_ = cost_scaling_factor;
    layer_.use_exponential_decay_ = use_exponential_decay;
  }

  void setTreatUnknownAsObstacle(bool enabled)
  {
    layer_.treat_unknown_as_obstacle_ = enabled;
  }

  float infDistanceCells() const
  {
    return EsdfLayer::kInfDistanceCells;
  }

private:
  EsdfLayer layer_;
};

TEST_F(EsdfLayerTester, ComputesSquaredDistanceFromLethalSite)
{
  Costmap2D costmap(5, 5, 1.0, 0.0, 0.0, FREE_SPACE);
  costmap.setCost(2, 2, LETHAL_OBSTACLE);

  const auto distances = computeDistanceTransform(costmap);

  EXPECT_FLOAT_EQ(distances[costmap.getIndex(2, 2)], 0.0F);
  EXPECT_FLOAT_EQ(distances[costmap.getIndex(3, 2)], 1.0F);
  EXPECT_FLOAT_EQ(distances[costmap.getIndex(4, 2)], 4.0F);
  EXPECT_FLOAT_EQ(distances[costmap.getIndex(3, 3)], 2.0F);
  EXPECT_FLOAT_EQ(distances[costmap.getIndex(0, 0)], 8.0F);
}

TEST_F(EsdfLayerTester, CanTreatUnknownAsDistanceSite)
{
  Costmap2D costmap(3, 3, 1.0, 0.0, 0.0, FREE_SPACE);
  costmap.setCost(1, 1, NO_INFORMATION);

  auto distances = computeDistanceTransform(costmap);
  EXPECT_GE(distances[costmap.getIndex(1, 1)], infDistanceCells());

  setTreatUnknownAsObstacle(true);
  distances = computeDistanceTransform(costmap);
  EXPECT_FLOAT_EQ(distances[costmap.getIndex(1, 1)], 0.0F);
  EXPECT_FLOAT_EQ(distances[costmap.getIndex(2, 1)], 1.0F);
}

TEST_F(EsdfLayerTester, MapsDistanceToContinuousCost)
{
  configureDistanceCost(0.5, 2.0, 1.0, false);

  EXPECT_EQ(distanceToCost(0.4), INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_EQ(distanceToCost(2.0), FREE_SPACE);

  const unsigned char close_cost = distanceToCost(0.75);
  const unsigned char far_cost = distanceToCost(1.5);
  EXPECT_GT(close_cost, far_cost);
  EXPECT_GT(far_cost, FREE_SPACE);
  EXPECT_LT(close_cost, INSCRIBED_INFLATED_OBSTACLE);
}

}  // namespace nav2_costmap_2d
