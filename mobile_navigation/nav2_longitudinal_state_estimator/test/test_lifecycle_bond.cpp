// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <memory>

#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_longitudinal_state_estimator/longitudinal_state_estimator_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

class LongitudinalStateEstimatorNodeShim
  : public nav2_longitudinal_state_estimator::LongitudinalStateEstimatorNode
{
public:
  bool hasBond() const
  {
    return static_cast<bool>(bond_);
  }
};

class RclcppFixture
{
public:
  RclcppFixture()
  {
    rclcpp::init(0, nullptr);
  }

  ~RclcppFixture()
  {
    rclcpp::shutdown();
  }
};

RclcppFixture g_rclcpp_fixture;

TEST(LongitudinalStateEstimatorLifecycle, CreatesAndDestroysNav2Bond)
{
  auto node = std::make_shared<LongitudinalStateEstimatorNodeShim>();

  EXPECT_EQ(
    node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_FALSE(node->hasBond());

  EXPECT_EQ(
    node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_TRUE(node->hasBond());
  EXPECT_GE(node->count_publishers("bond"), 1u);

  EXPECT_EQ(
    node->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_FALSE(node->hasBond());

  EXPECT_EQ(
    node->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

}  // namespace
