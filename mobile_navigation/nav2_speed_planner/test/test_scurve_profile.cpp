// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// S 型速度曲线（Ruckig）纯算法单测。
// 直接测试 SCurveSpeedPlanner::generateRuckigProfile（仅依赖 ruckig，无 ROS）。
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "nav2_speed_planner/scurve_speed_planner.hpp"

using nav2_speed_planner::SCurveSpeedPlanner;

namespace
{
std::vector<double> uniformArc(size_t n, double step)
{
  std::vector<double> s(n, 0.0);
  for (size_t i = 1; i < n; ++i) {
    s[i] = s[i - 1] + step;
  }
  return s;
}

bool allFinite(const std::vector<double> & v)
{
  for (double x : v) {
    if (!std::isfinite(x)) {return false;}
  }
  return true;
}

// 通用边界断言：jerk≤j、加速度∈[-d_max,a_max]、不超包络、停车、时间单调、有限。
void checkBounds(
  const std::vector<double> & env, const std::vector<double> & v,
  const std::vector<double> & a, const std::vector<double> & t,
  double a_max, double d_max, double j)
{
  ASSERT_EQ(v.size(), env.size());
  EXPECT_TRUE(allFinite(v));
  EXPECT_TRUE(allFinite(a));
  EXPECT_TRUE(allFinite(t));
  EXPECT_NEAR(v.back(), 0.0, 1e-6);                    // 停车
  for (size_t i = 0; i < v.size(); ++i) {
    EXPECT_GE(v[i], -1e-9);
    EXPECT_LE(v[i], env[i] + 1e-3);                    // 不超包络
    EXPECT_LE(a[i], a_max + 0.02);                     // 加速度上界
    EXPECT_GE(a[i], -d_max - 0.02);                    // 减速度下界（非对称）
  }
  for (size_t i = 1; i < v.size(); ++i) {
    EXPECT_GT(t[i], t[i - 1] - 1e-9);                  // 时间单调
  }
  double max_jerk = 0.0;
  for (size_t i = 2; i < a.size(); ++i) {
    const double dt = t[i] - t[i - 1];
    if (dt > 1e-6) {max_jerk = std::max(max_jerk, std::fabs(a[i] - a[i - 1]) / dt);}
  }
  EXPECT_LE(max_jerk, j + 0.05);                       // 严格 jerk 限制（Ruckig 保证）
}
}  // namespace

TEST(SCurveRuckig, RestToCruiseToStop)
{
  const size_t n = 200;
  std::vector<double> env(n, 0.6);
  const auto s = uniformArc(n, 0.1);
  std::vector<double> v, a, t;
  SCurveSpeedPlanner::generateRuckigProfile(env, s, 0.0, 0.35, 0.15, 0.5, 0.05, v, a, t);
  checkBounds(env, v, a, t, 0.35, 0.15, 0.5);
  double vmax = 0.0;
  for (double x : v) {
    vmax = std::max(vmax, x);
  }
  EXPECT_NEAR(vmax, 0.6, 0.05);                        // 中段达巡航
}

TEST(SCurveRuckig, EnvelopeDip)
{
  const size_t n = 200;
  std::vector<double> env(n, 0.6);
  for (size_t i = 90; i < 110; ++i) {
    env[i] = 0.15;
  }
  const auto s = uniformArc(n, 0.1);
  std::vector<double> v, a, t;
  SCurveSpeedPlanner::generateRuckigProfile(env, s, 0.0, 0.35, 0.15, 0.5, 0.05, v, a, t);
  checkBounds(env, v, a, t, 0.35, 0.15, 0.5);
}

TEST(SCurveRuckig, StartWithVelocity)
{
  const size_t n = 200;
  std::vector<double> env(n, 0.6);
  const auto s = uniformArc(n, 0.1);
  std::vector<double> v, a, t;
  SCurveSpeedPlanner::generateRuckigProfile(env, s, 0.4, 0.35, 0.15, 0.5, 0.05, v, a, t);
  checkBounds(env, v, a, t, 0.35, 0.15, 0.5);
}

TEST(SCurveRuckig, ObstacleHardStop)
{
  const size_t n = 200;
  std::vector<double> env(n, 0.6);
  // 障碍：后半段置零
  for (size_t i = 120; i < n; ++i) {
    env[i] = 0.0;
  }
  const auto s = uniformArc(n, 0.1);
  std::vector<double> v, a, t;
  SCurveSpeedPlanner::generateRuckigProfile(env, s, 0.0, 0.35, 0.15, 0.5, 0.05, v, a, t);
  checkBounds(env, v, a, t, 0.35, 0.15, 0.5);
}

TEST(SCurveRuckig, Degenerate)
{
  std::vector<double> v, a, t;
  SCurveSpeedPlanner::generateRuckigProfile({}, {}, 0.5, 0.35, 0.15, 0.5, 0.05, v, a, t);
  EXPECT_TRUE(v.empty());
  SCurveSpeedPlanner::generateRuckigProfile({0.4}, {0.0}, 1.0, 0.35, 0.15, 0.5, 0.05, v, a, t);
  ASSERT_EQ(v.size(), 1u);
  EXPECT_NEAR(v[0], 0.4, 1e-6);
}

TEST(SCurveRuckig, ShortTerminalPath)
{
  const size_t n = 11;
  std::vector<double> env(n, 0.6);
  env[n - 3] = 0.05;
  env[n - 2] = 0.05;
  env[n - 1] = 0.05;
  const auto s = uniformArc(n, 0.0184);
  std::vector<double> v, a, t;

  SCurveSpeedPlanner::generateRuckigProfile(
    env, s, 0.0, 0.5, 0.4, 0.5, 0.05, v, a, t, 0.05);

  ASSERT_EQ(v.size(), n);
  EXPECT_TRUE(allFinite(v));
  EXPECT_TRUE(allFinite(a));
  EXPECT_TRUE(allFinite(t));
  EXPECT_NEAR(v.back(), 0.05, 1e-6);
  for (size_t i = 0; i < n; ++i) {
    EXPECT_GE(v[i], -1e-9);
    EXPECT_LE(v[i], env[i] + 1e-3);
  }
}
