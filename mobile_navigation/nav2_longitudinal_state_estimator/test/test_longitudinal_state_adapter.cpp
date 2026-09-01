// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "nav2_longitudinal_state_estimator/longitudinal_state_adapter.hpp"
#include "rclcpp/duration.hpp"

namespace
{

rclcpp::Time timeAt(double seconds)
{
  return rclcpp::Time(30, 0, RCL_ROS_TIME) + rclcpp::Duration::from_seconds(seconds);
}

TEST(LongitudinalStateAdapterTest, RequiresFreshSynchronizedSamples)
{
  nav2_longitudinal_state_estimator::LongitudinalStateAdapter adapter;
  for (int index = 0; index < 5; ++index) {
    const auto stamp = timeAt(0.01 * index);
    adapter.addVelocity(0.2, 1e-6, stamp);
    adapter.addAcceleration(0.1, 1e-8, stamp);
  }
  const auto estimate = adapter.estimate(timeAt(0.05));
  ASSERT_TRUE(estimate.valid);
  EXPECT_TRUE(estimate.velocity_valid);
  EXPECT_TRUE(estimate.acceleration_valid);
  EXPECT_NEAR(estimate.velocity, 0.2, 1e-12);
  EXPECT_NEAR(estimate.acceleration, 0.1, 1e-12);
  EXPECT_NEAR(estimate.velocity_std, 0.02, 1e-12);
  EXPECT_NEAR(estimate.acceleration_std, 0.15, 1e-12);
}

TEST(LongitudinalStateAdapterTest, ClampsAccelerationWithoutHidingRawValue)
{
  auto config = nav2_longitudinal_state_estimator::LongitudinalStateAdapter::Config();
  config.min_velocity_samples = 1;
  config.min_acceleration_samples = 1;
  nav2_longitudinal_state_estimator::LongitudinalStateAdapter adapter(config);
  adapter.addVelocity(0.3, 0.0004, timeAt(0.0));
  adapter.addAcceleration(-1.2, 0.0001, timeAt(0.0));
  const auto estimate = adapter.estimate(timeAt(0.01));
  ASSERT_TRUE(estimate.valid);
  EXPECT_NEAR(estimate.acceleration_raw, -1.2, 1e-12);
  EXPECT_NEAR(estimate.acceleration, -0.6, 1e-12);
  EXPECT_EQ(estimate.acceleration_clamps, 1u);
}

TEST(LongitudinalStateAdapterTest, RejectsStaleOrUnsynchronizedPairs)
{
  auto config = nav2_longitudinal_state_estimator::LongitudinalStateAdapter::Config();
  config.min_velocity_samples = 1;
  config.min_acceleration_samples = 1;
  nav2_longitudinal_state_estimator::LongitudinalStateAdapter adapter(config);
  adapter.addVelocity(0.1, 0.0004, timeAt(0.0));
  adapter.addAcceleration(0.0, 0.01, timeAt(0.03));
  EXPECT_FALSE(adapter.estimate(timeAt(0.04)).valid);

  adapter.reset();
  adapter.addVelocity(0.1, 0.0004, timeAt(0.0));
  adapter.addAcceleration(0.0, 0.01, timeAt(0.0));
  EXPECT_FALSE(adapter.estimate(timeAt(0.10)).valid);
}

}  // namespace
