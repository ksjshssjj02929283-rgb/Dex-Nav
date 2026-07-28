// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// S 型速度曲线（Ruckig）纯算法单测。
// 直接测试 SCurveSpeedPlanner::generateRuckigProfile（仅依赖 ruckig，无 ROS）。
#include <gtest/gtest.h>

#include <algorithm>
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
  SCurveSpeedPlanner::generateRuckigProfile(
    env, s, 0.0, 0.0, 0.35, 0.15, 0.5, 0.05, v, a, t);
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
  SCurveSpeedPlanner::generateRuckigProfile(
    env, s, 0.0, 0.0, 0.35, 0.15, 0.5, 0.05, v, a, t);
  checkBounds(env, v, a, t, 0.35, 0.15, 0.5);
}

TEST(SCurveRuckig, StartWithVelocity)
{
  const size_t n = 200;
  std::vector<double> env(n, 0.6);
  const auto s = uniformArc(n, 0.1);
  std::vector<double> v, a, t;
  SCurveSpeedPlanner::generateRuckigProfile(
    env, s, 0.4, 0.0, 0.35, 0.15, 0.5, 0.05, v, a, t);
  checkBounds(env, v, a, t, 0.35, 0.15, 0.5);
}

TEST(SCurveRuckig, PreservesAndClampsInitialAcceleration)
{
  const size_t n = 200;
  const std::vector<double> env(n, 0.6);
  const auto s = uniformArc(n, 0.05);
  std::vector<double> v, a, t;

  SCurveSpeedPlanner::generateRuckigProfile(
    env, s, 0.2, 0.25, 0.35, 0.15, 0.5, 0.05, v, a, t);
  ASSERT_FALSE(a.empty());
  EXPECT_NEAR(a.front(), 0.25, 1e-9);

  SCurveSpeedPlanner::generateRuckigProfile(
    env, s, 0.2, 1.0, 0.35, 0.15, 0.5, 0.05, v, a, t);
  ASSERT_FALSE(a.empty());
  EXPECT_NEAR(a.front(), 0.35, 1e-9);

  SCurveSpeedPlanner::generateRuckigProfile(
    env, s, 0.2, -1.0, 0.35, 0.15, 0.5, 0.05, v, a, t);
  ASSERT_FALSE(a.empty());
  EXPECT_NEAR(a.front(), -0.15, 1e-9);
}

TEST(SCurveRuckig, RepeatedReplanningKeepsAccelerationState)
{
  const size_t n = 300;
  const std::vector<double> env(n, 0.6);
  const auto s = uniformArc(n, 0.02);
  std::vector<double> v, a, t;
  double v_start = 0.1;
  double a_start = 0.2;

  for (int cycle = 0; cycle < 8; ++cycle) {
    SCurveSpeedPlanner::generateRuckigProfile(
      env, s, v_start, a_start, 0.35, 0.15, 0.5, 0.05, v, a, t);
    ASSERT_FALSE(a.empty());
    EXPECT_NEAR(a.front(), a_start, 1e-9);
    EXPECT_GT(a.front(), 0.05);

    auto sample = std::find_if(t.begin(), t.end(), [](double value) {return value >= 0.1;});
    ASSERT_NE(sample, t.end());
    const size_t index = static_cast<size_t>(std::distance(t.begin(), sample));
    v_start = v[index];
    a_start = a[index];
  }
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
  SCurveSpeedPlanner::generateRuckigProfile(
    env, s, 0.0, 0.0, 0.35, 0.15, 0.5, 0.05, v, a, t);
  checkBounds(env, v, a, t, 0.35, 0.15, 0.5);
}

TEST(SCurveRuckig, Degenerate)
{
  std::vector<double> v, a, t;
  SCurveSpeedPlanner::generateRuckigProfile(
    {}, {}, 0.5, 0.0, 0.35, 0.15, 0.5, 0.05, v, a, t);
  EXPECT_TRUE(v.empty());
  SCurveSpeedPlanner::generateRuckigProfile(
    {0.4}, {0.0}, 1.0, 0.0, 0.35, 0.15, 0.5, 0.05, v, a, t);
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
    env, s, 0.0, 0.0, 0.5, 0.4, 0.5, 0.05, v, a, t, 0.05);

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

TEST(SCurveRuckig, InfeasibleStopZoneUsesReachableJerkLimitedBraking)
{
  const std::vector<double> env{0.6, 0.05, 0.05, 0.05};
  const auto s = uniformArc(env.size(), 0.02);
  std::vector<double> v, a, t;

  SCurveSpeedPlanner::generateRuckigProfile(
    env, s, 0.325, 0.0, 0.5, 0.4, 0.5, 0.05, v, a, t, 0.05);

  ASSERT_EQ(v.size(), env.size());
  ASSERT_EQ(a.size(), env.size());
  ASSERT_EQ(t.size(), env.size());
  EXPECT_NEAR(v.front(), 0.325, 1e-9);
  EXPECT_NEAR(a.front(), 0.0, 1e-9);
  EXPECT_GT(v[1], 0.05) << "an unreachable future target must not be envelope-clamped";
  for (size_t i = 0; i < v.size(); ++i) {
    EXPECT_TRUE(std::isfinite(v[i]));
    EXPECT_TRUE(std::isfinite(a[i]));
    EXPECT_TRUE(std::isfinite(t[i]));
    EXPECT_GE(v[i], 0.05 - 1e-6);
    EXPECT_GE(a[i], -0.4 - 1e-6);
    EXPECT_LE(a[i], 0.5 + 1e-6);
    if (i == 0) {
      continue;
    }
    EXPECT_GT(t[i], t[i - 1]);
    const double implied_decel =
      (v[i - 1] * v[i - 1] - v[i] * v[i]) / (2.0 * (s[i] - s[i - 1]));
    EXPECT_LE(implied_decel, 0.4 + 0.01);
    const double jerk = std::fabs(a[i] - a[i - 1]) / (t[i] - t[i - 1]);
    EXPECT_LE(jerk, 0.5 + 0.05);
  }
}

TEST(SCurveRuckig, TerminalFallbackDoesNotUndershootOrRebound)
{
  const size_t n = 101;
  std::vector<double> env(n, 0.05);
  env.front() = 0.6;
  const auto s = uniformArc(n, 0.02);
  std::vector<double> v, a, t;

  const bool terminal_infeasible = SCurveSpeedPlanner::generateRuckigProfile(
    env, s, 0.6, 0.0, 0.5, 0.4, 0.5, 0.05, v, a, t, 0.05);

  EXPECT_TRUE(terminal_infeasible);
  ASSERT_EQ(v.size(), n);
  ASSERT_EQ(a.size(), n);
  ASSERT_EQ(t.size(), n);
  for (size_t i = 0; i < n; ++i) {
    EXPECT_GE(v[i], 0.05 - 1e-6);
    EXPECT_LE(a[i], 1e-6) << "terminal fallback must not accelerate back up";
    if (i > 0) {
      EXPECT_LE(v[i], v[i - 1] + 1e-6);
      EXPECT_GT(t[i], t[i - 1]);
    }
  }
  EXPECT_NEAR(v.back(), 0.05, 1e-6);
  EXPECT_NEAR(a.back(), 0.0, 1e-6);
}

TEST(SCurveRuckig, LateNegativeAccelerationReleasesWithoutRebound)
{
  const size_t n = 51;
  std::vector<double> env(n, 0.05);
  env.front() = 0.6;
  const auto s = uniformArc(n, 0.01);
  std::vector<double> v, a, t;

  const bool terminal_infeasible = SCurveSpeedPlanner::generateRuckigProfile(
    env, s, 0.0583, -0.108, 0.5, 0.4, 0.5, 0.05, v, a, t, 0.05);

  EXPECT_TRUE(terminal_infeasible);
  ASSERT_EQ(v.size(), n);
  double min_velocity = v.front();
  for (size_t i = 0; i < n; ++i) {
    min_velocity = std::min(min_velocity, v[i]);
    EXPECT_LE(a[i], 1e-6) << "an unavoidable release must not add a rebound";
    if (i > 0) {
      EXPECT_LE(v[i], v[i - 1] + 1e-6);
    }
  }
  EXPECT_NEAR(min_velocity, 0.046636, 1e-3);
  EXPECT_NEAR(v.back(), min_velocity, 1e-6);
  EXPECT_NEAR(a.back(), 0.0, 1e-6);
}
