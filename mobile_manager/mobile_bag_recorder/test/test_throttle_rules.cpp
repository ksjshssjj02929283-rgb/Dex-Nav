// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "mobile_bag_recorder/recorder_config.hpp"

using mobile_bag_recorder::RecorderConfig;

namespace
{
rclcpp::Logger testLogger()
{
  return rclcpp::get_logger("test_throttle_rules");
}
}  // namespace

TEST(ThrottleRules, ParsesRateIntoInterval)
{
  const auto intervals = RecorderConfig::parseThrottleRules(
    {"/local_costmap/costmap:2.0", "/feedback/odom:10.0"}, testLogger());

  ASSERT_EQ(intervals.size(), 2u);
  EXPECT_EQ(intervals.at("/local_costmap/costmap"), 500000000);
  EXPECT_EQ(intervals.at("/feedback/odom"), 100000000);
}

TEST(ThrottleRules, KeepsTopicNameWhenItContainsColon)
{
  // 话题名里出现冒号并不合法，但按最后一个冒号切分意味着即使出现也不会截断话题名
  const auto intervals = RecorderConfig::parseThrottleRules({"/ns:odd/topic:4.0"}, testLogger());

  ASSERT_EQ(intervals.size(), 1u);
  EXPECT_EQ(intervals.at("/ns:odd/topic"), 250000000);
}

TEST(ThrottleRules, SkipsMalformedEntriesWithoutDroppingValidOnes)
{
  // 一条笔误不该废掉整份配置：非法条目跳过并告警，合法条目照常生效
  const auto intervals = RecorderConfig::parseThrottleRules(
    {"/no_rate", "/bad:abc", "/zero:0", "/negative:-1", ":5.0", "/good:5.0"}, testLogger());

  ASSERT_EQ(intervals.size(), 1u);
  EXPECT_EQ(intervals.at("/good"), 200000000);
}

TEST(ThrottleRules, EmptyRulesMeanNoThrottling)
{
  EXPECT_TRUE(RecorderConfig::parseThrottleRules({}, testLogger()).empty());
}
