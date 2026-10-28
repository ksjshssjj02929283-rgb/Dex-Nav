#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "nav2_frenet_ilqr_controller/frenet_costs.hpp"
#include "nav2_frenet_ilqr_controller/frenet_policies.hpp"
#include "pluginlib/class_loader.hpp"

namespace
{

bool contains(const std::vector<std::string> & values, const std::string & value)
{
  return std::find(values.begin(), values.end(), value) != values.end();
}

}  // namespace

TEST(FrenetPlugins, PolicyPluginsAreDiscoverable) {
  pluginlib::ClassLoader<nav2_frenet_ilqr_controller::policies::RclcppNodePolicy> loader(
    "nav2_frenet_ilqr_controller", "nav2_frenet_ilqr_controller::policies::RclcppNodePolicy");
  const auto classes = loader.getDeclaredClasses();
  EXPECT_TRUE(contains(classes, "nav2_frenet_ilqr_controller::policies::ObstaclePolicy"));
  EXPECT_TRUE(contains(classes, "nav2_frenet_ilqr_controller::policies::ConstraintsPolicy"));
  EXPECT_TRUE(contains(classes, "nav2_frenet_ilqr_controller::policies::AccelerationPolicy"));
  EXPECT_NO_THROW(
    loader.createSharedInstance(
      "nav2_frenet_ilqr_controller::policies::ObstaclePolicy"));
}

TEST(FrenetPlugins, CostPluginsAreDiscoverable) {
  pluginlib::ClassLoader<nav2_frenet_ilqr_controller::costs::RclcppNodeCost> loader(
    "nav2_frenet_ilqr_controller", "nav2_frenet_ilqr_controller::costs::RclcppNodeCost");
  const auto classes = loader.getDeclaredClasses();
  EXPECT_TRUE(contains(classes, "nav2_frenet_ilqr_controller::costs::LateralDistanceCost"));
  EXPECT_TRUE(contains(classes, "nav2_frenet_ilqr_controller::costs::LongitudinalVelocityCost"));
  EXPECT_TRUE(contains(classes, "nav2_frenet_ilqr_controller::costs::LongtitutalVelocityCost"));
  EXPECT_TRUE(contains(classes, "nav2_frenet_ilqr_controller::costs::LatLonBalanceCost"));
  EXPECT_TRUE(contains(classes, "nav2_frenet_ilqr_controller::costs::ObstacleCost"));
  EXPECT_NO_THROW(
    loader.createSharedInstance(
      "nav2_frenet_ilqr_controller::costs::LateralDistanceCost"));
}
