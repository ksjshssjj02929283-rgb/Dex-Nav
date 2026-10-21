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
  // 设置 low_pass_alpha = 1.0 测试纯中值滤波行为
  RangeFilter filter(3, 0.30, 1.0);
  bool is_outlier = false;

  // 真实障碍物持续靠近
  EXPECT_NEAR(filter.update(0.35f, is_outlier), 0.35f, 1e-4);
  EXPECT_NEAR(filter.update(0.30f, is_outlier), 0.35f, 1e-4);  // [0.35, 0.30] 索引 1 为 0.35
  EXPECT_NEAR(filter.update(0.25f, is_outlier), 0.30f, 1e-4);  // [0.35, 0.30, 0.25] 的中位数是 0.30
  EXPECT_NEAR(filter.update(0.25f, is_outlier), 0.25f, 1e-4);  // [0.30, 0.25, 0.25] 的中位数是 0.25
}

TEST(RangeFilterTest, TestLowPassFilterDelay)
{
  // 验证中值滤波后的一阶低通平滑特性与 1-2 周期延迟
  // low_pass_alpha = 0.5 时，等效群延迟为 1.0 周期，2 周期内完成 75% 响应
  RangeFilter filter(1, 0.0, 0.5);
  bool is_outlier = false;

  // 初始稳态 2.0m
  EXPECT_NEAR(filter.update(2.0f, is_outlier), 2.0f, 1e-4);

  // 发生阶跃到 1.0m
  // 周期 1: 50% 响应到达 1.5m (0.5 * 1.0 + 0.5 * 2.0 = 1.5)
  EXPECT_NEAR(filter.update(1.0f, is_outlier), 1.5f, 1e-4);

  // 周期 2: 75% 响应到达 1.25m (0.5 * 1.0 + 0.5 * 1.5 = 1.25)
  EXPECT_NEAR(filter.update(1.0f, is_outlier), 1.25f, 1e-4);

  // 周期 3: 87.5% 响应到达 1.125m
  EXPECT_NEAR(filter.update(1.0f, is_outlier), 1.125f, 1e-4);
}

TEST(RangeFilterTest, TestCombinedMedianAndLowPassFilter)
{
  RangeFilter filter(3, 0.30, 0.5);
  bool is_outlier = false;

  // 第 1 帧: 0.35m -> median=0.35, lpf=0.35
  EXPECT_NEAR(filter.update(0.35f, is_outlier), 0.35f, 1e-4);

  // 第 2 帧: 0.30m -> median=0.35, lpf=0.5*0.35 + 0.5*0.35 = 0.35
  EXPECT_NEAR(filter.update(0.30f, is_outlier), 0.35f, 1e-4);

  // 第 3 帧: 0.25m -> median=0.30, lpf=0.5*0.30 + 0.5*0.35 = 0.325
  EXPECT_NEAR(filter.update(0.25f, is_outlier), 0.325f, 1e-4);

  // 第 4 帧: 0.25m -> median=0.25, lpf=0.5*0.25 + 0.5*0.325 = 0.2875
  EXPECT_NEAR(filter.update(0.25f, is_outlier), 0.2875f, 1e-4);
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

// 9.3.5 UT-FILTER-05: 3 窗口滑动中值滤波在阶跃信号下的收敛速度对比断言
TEST(RangeFilterTest, TestStepConvergenceWindow3VsWindow5)
{
  RangeFilter filter3(3, 0.50, 1.0);  // 3 窗口纯中值
  RangeFilter filter5(5, 0.50, 1.0);  // 5 窗口纯中值
  bool is_outlier = false;

  // 1. 初始背景 1.5m 稳定读数
  for (int i = 0; i < 5; ++i) {
    filter3.update(1.5f, is_outlier);
    filter5.update(1.5f, is_outlier);
  }

  // 2. 障碍物以阶跃方式出现（1.5m 突变至 0.3m）
  // 第 1 帧:
  EXPECT_NEAR(filter3.update(0.3f, is_outlier), 1.5f, 1e-4);
  EXPECT_NEAR(filter5.update(0.3f, is_outlier), 1.5f, 1e-4);

  // 第 2 帧: 3 窗口在第 2 帧即完成中值收敛（1.5, 0.3, 0.3 -> 0.3m）
  EXPECT_NEAR(filter3.update(0.3f, is_outlier), 0.3f, 1e-4);
  // 5 窗口在第 2 帧仍然输出 1.5m（1.5, 1.5, 1.5, 0.3, 0.3 -> 1.5m）
  EXPECT_NEAR(filter5.update(0.3f, is_outlier), 1.5f, 1e-4);

  // 第 3 帧: 5 窗口在第 3 帧才收敛为 0.3m
  EXPECT_NEAR(filter5.update(0.3f, is_outlier), 0.3f, 1e-4);
}
