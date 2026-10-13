// Copyright (c) 2026 Dexforce
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
#include "nav2_collision_monitor/range_filter.hpp"

using nav2_collision_monitor::RangeFilter;

TEST(RangeFilterTest, TestSteadyReadings)
{
  RangeFilter filter(3, 0.30);
  bool is_outlier = false;

  EXPECT_NEAR(filter.update(1.2f, is_outlier), 1.2f, 1e-4);
  EXPECT_FALSE(is_outlier);

  EXPECT_NEAR(filter.update(1.2f, is_outlier), 1.2f, 1e-4);
  EXPECT_FALSE(is_outlier);

  EXPECT_NEAR(filter.update(1.2f, is_outlier), 1.2f, 1e-4);
  EXPECT_FALSE(is_outlier);
}

TEST(RangeFilterTest, TestSpikeOutlierRejection)
{
  RangeFilter filter(3, 0.30);
  bool is_outlier = false;

  // 1. 初始化稳定背景距离 1.5m
  filter.update(1.5f, is_outlier);
  filter.update(1.5f, is_outlier);

  // 2. 模拟外壳假回波/突发尖峰脉冲：0.08m
  float filtered = filter.update(0.08f, is_outlier);
  // 中值滤波应剔除 0.08m，输出仍为 1.5m
  EXPECT_NEAR(filtered, 1.5f, 1e-4);
  // 突变量 |0.08 - 1.5| = 1.42m > 0.30m，应标记为离群点
  EXPECT_TRUE(is_outlier);

  // 3. 读数恢复正常 1.5m
  filtered = filter.update(1.5f, is_outlier);
  EXPECT_NEAR(filtered, 1.5f, 1e-4);
  EXPECT_FALSE(is_outlier);
}

TEST(RangeFilterTest, TestContinuousObstacleApproach)
{
  RangeFilter filter(3, 0.30);
  bool is_outlier = false;

  // 真实障碍物持续靠近
  EXPECT_NEAR(filter.update(0.35f, is_outlier), 0.35f, 1e-4);
  EXPECT_NEAR(filter.update(0.30f, is_outlier), 0.35f, 1e-4);  // [0.35, 0.30] 索引 1 为 0.35
  EXPECT_NEAR(filter.update(0.25f, is_outlier), 0.30f, 1e-4);  // [0.35, 0.30, 0.25] 的中位数是 0.30
  EXPECT_NEAR(filter.update(0.25f, is_outlier), 0.25f, 1e-4);  // [0.30, 0.25, 0.25] 的中位数是 0.25
}

TEST(RangeFilterTest, TestReset)
{
  RangeFilter filter(3, 0.30);
  bool is_outlier = false;

  filter.update(0.20f, is_outlier);
  filter.update(0.20f, is_outlier);
  filter.reset();

  // reset 后第一个点独立成为窗口新起点
  EXPECT_NEAR(filter.update(1.0f, is_outlier), 1.0f, 1e-4);
  EXPECT_FALSE(is_outlier);
}
