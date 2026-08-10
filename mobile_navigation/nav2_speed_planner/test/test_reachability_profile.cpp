// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 纯可达性滤波后端单测。
//
// 这些用例钉的是**该后端与求解型后端的结构性差别**：每一步都是取严或取松的
// 单调运算，故不存在无解分支。若将来有人重新引入硬路点求解，本组用例会红。
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "nav2_speed_planner/backends/reachability_speed_planner.hpp"

namespace
{

using nav2_speed_planner::ConstraintSource;
using nav2_speed_planner::ReachabilitySpeedPlanner;
using nav2_speed_planner::TrajPointWork;

constexpr double kStep = 0.02;
constexpr double kAccel = 0.5;
constexpr double kDecel = -0.4;
constexpr double kMaxSpeed = 0.6;
constexpr double kEndSpeed = 0.05;
constexpr double kStopDist = 0.075;
constexpr double kJerk = 0.5;
constexpr double kControlDt = 0.1;   // controller_frequency: 10.0

/// 暴露受保护成员，并直接驱动 Phase B（不经 costmap / ROS）。
class Backend : public ReachabilitySpeedPlanner
{
public:
  Backend()
  {
    plugin_name_ = "test";
    max_speed_ = kMaxSpeed;
    min_speed_ = 0.05;
    accel_ = kAccel;
    decel_ = kDecel;
    end_speed_ = kEndSpeed;
    terminal_creep_distance_ = kStopDist;
    max_jerk_ = kJerk;
    control_dt_ = kControlDt;
  }
  using ReachabilitySpeedPlanner::generateProfile;
};

/// 直线路径 + 逐点包络上界。
std::vector<TrajPointWork> makeWork(const std::vector<double> & env)
{
  std::vector<TrajPointWork> w(env.size());
  for (size_t i = 0; i < env.size(); ++i) {
    w[i].pose.pose.position.x = static_cast<double>(i) * kStep;
    w[i].pose.pose.orientation.w = 1.0;
    w[i].velocity_upper_bound = env[i];
    w[i].constraint_source = ConstraintSource::kMaxSpeed;
  }
  return w;
}

std::vector<double> flatEnv(size_t n, double v) {return std::vector<double>(n, v);}

double maxAbsAccel(const std::vector<TrajPointWork> & w)
{
  double m = 0.0;
  for (const auto & p : w) {m = std::max(m, std::fabs(p.accel));}
  return m;
}

/// 加速度列必须等于速度列对时间的导数——两者若来自不同剖面（例如速度被
/// 事后钳位而加速度没有），下游拿到的 (v, a) 自相矛盾。
double maxVaInconsistency(const std::vector<TrajPointWork> & w)
{
  double m = 0.0;
  for (size_t i = 0; i + 1 < w.size(); ++i) {
    const double dt = w[i + 1].rel_time - w[i].rel_time;
    if (dt < 1e-9) {continue;}
    const double dv = (w[i + 1].velocity_upper_bound - w[i].velocity_upper_bound) / dt;
    m = std::max(m, std::fabs(w[i].accel - dv));
  }
  return m;
}

}  // namespace

// 直路：应爬到巡航上界并在末端收敛。
TEST(ReachabilityProfile, ReachesCruiseOnStraightPath)
{
  auto w = makeWork(flatEnv(250, kMaxSpeed));
  Backend b;
  b.generateProfile(w, 0.0, 0.0, kMaxSpeed, false);

  double peak = 0.0;
  for (const auto & p : w) {peak = std::max(peak, p.velocity_upper_bound);}
  EXPECT_NEAR(peak, kMaxSpeed, 1e-6);
  EXPECT_LE(maxAbsAccel(w), std::max(kAccel, std::fabs(kDecel)) + 1e-6);
}

// 首点必须是**实测速度**：机器人此刻就是这个速度，计划说别的既不可执行、
// 也让消费侧无从判断该跟哪个。
TEST(ReachabilityProfile, FirstPointIsTheMeasuredVelocity)
{
  for (double v0 : {0.0, 0.2, 0.45}) {
    auto w = makeWork(flatEnv(250, kMaxSpeed));
    Backend b;
    b.generateProfile(w, v0, 0.0, kMaxSpeed, false);
    EXPECT_NEAR(w.front().velocity_upper_bound, v0, 1e-9) << "v0=" << v0;
  }
}

// 起点即深谷 + 其后不远处另一处限速：求解型后端在此塌成末速，
// 滤波必须照常爬出去。
TEST(ReachabilityProfile, ClimbsOutOfAValleyAtPathStart)
{
  std::vector<double> env(250, kMaxSpeed);
  for (size_t i = 0; i < 20; ++i) {env[i] = 0.10;}          // 起点深谷
  for (size_t i = 45; i < 145; ++i) {env[i] = 0.5713;}      // 井后不远的平缓限速
  auto w = makeWork(env);
  Backend b;
  b.generateProfile(w, 0.0, 0.0, kMaxSpeed, false);

  double peak = 0.0;
  for (const auto & p : w) {peak = std::max(peak, p.velocity_upper_bound);}
  EXPECT_GT(peak, 0.5) << "起点谷不应使整条剖面塌陷";
  EXPECT_LE(maxAbsAccel(w), std::max(kAccel, std::fabs(kDecel)) + 1e-6);
}

// 包络平台段内不得鼓包：上界是约束不是路点，剖面应贴着走而不是"赶时间"。
TEST(ReachabilityProfile, PlateauStaysFlat)
{
  std::vector<double> env(300, kMaxSpeed);
  for (size_t i = 100; i < 200; ++i) {env[i] = 0.25;}
  auto w = makeWork(env);
  Backend b;
  b.generateProfile(w, 0.0, 0.0, kMaxSpeed, false);

  double lo = 1e9, hi = -1e9;
  for (size_t i = 120; i < 180; ++i) {           // 取平台芯部，避开进出坡道
    lo = std::min(lo, w[i].velocity_upper_bound);
    hi = std::max(hi, w[i].velocity_upper_bound);
  }
  EXPECT_LT(hi - lo, 1e-6) << "平台段出现鼓包：hi=" << hi << " lo=" << lo;
}

// (v, a) 必须自洽——加速度列是速度列的时间导数，不是另一条剖面的残留。
TEST(ReachabilityProfile, VelocityAndAccelerationAreConsistent)
{
  std::vector<double> env(300, kMaxSpeed);
  for (size_t i = 100; i < 200; ++i) {env[i] = 0.25;}
  auto w = makeWork(env);
  Backend b;
  b.generateProfile(w, 0.0, 0.0, kMaxSpeed, false);
  EXPECT_LT(maxVaInconsistency(w), 1e-3);
}

// 初值高于起点包络（限速骤降 / 障碍突现）：不得要求瞬间掉速。
// 输出应以最大减速度收敛，那些点会暂时高于包络——如实反映不可行，
// 而不是产生一个执行不了的阶跃。
TEST(ReachabilityProfile, InitialVelocityAboveEnvelopeConvergesAtPhysicalLimit)
{
  std::vector<double> env(250, kMaxSpeed);
  for (size_t i = 0; i < 60; ++i) {env[i] = 0.10;}
  auto w = makeWork(env);
  Backend b;
  b.generateProfile(w, 0.55, 0.0, kMaxSpeed, false);

  EXPECT_NEAR(w.front().velocity_upper_bound, 0.55, 1e-9);
  EXPECT_LE(maxAbsAccel(w), std::max(kAccel, std::fabs(kDecel)) + 1e-6)
    << "不得为了贴合包络而产生超限减速度";
  // 单调下降直到贴上包络
  EXPECT_LT(w[30].velocity_upper_bound, w[0].velocity_upper_bound);
}

// 无解分支不存在：包络里塞进各种病态形状，都必须出一条有限、非空的剖面。
TEST(ReachabilityProfile, NeverFailsOnPathologicalEnvelopes)
{
  const std::vector<std::vector<double>> cases = {
    flatEnv(3, kMaxSpeed),                                    // 极短
    flatEnv(250, 0.05),                                       // 全程贴末速
    [] {std::vector<double> e(250, kMaxSpeed);
      for (size_t i = 0; i < 250; i += 2) {e[i] = 0.08;}      // 逐点交替深谷
      return e;}(),
    [] {std::vector<double> e(250, kMaxSpeed);
      for (size_t i = 120; i < 123; ++i) {e[i] = 0.0;}        // 中段零速带
      return e;}(),
  };
  for (size_t k = 0; k < cases.size(); ++k) {
    auto w = makeWork(cases[k]);
    Backend b;
    b.generateProfile(w, 0.3, 0.0, kMaxSpeed, false);
    ASSERT_FALSE(w.empty()) << "case " << k;
    for (const auto & p : w) {
      EXPECT_TRUE(std::isfinite(p.velocity_upper_bound)) << "case " << k;
      EXPECT_TRUE(std::isfinite(p.accel)) << "case " << k;
      EXPECT_TRUE(std::isfinite(p.rel_time)) << "case " << k;
      EXPECT_GE(p.velocity_upper_bound, -1e-9) << "case " << k;
    }
    for (size_t i = 1; i < w.size(); ++i) {
      EXPECT_GE(w[i].rel_time, w[i - 1].rel_time - 1e-9) << "case " << k;
    }
  }
}

// 剖面**首点**的加速度是"此刻执行的那一个"，它每个控制周期都会被下一版计划
// 替换。seed → 首点这一次变化，执行侧只有一个控制周期去完成，与该点所在路径段
// 要走多久无关——故必须受 `j·control_dt` 约束，而不是段通过时长 `Δs/v`。
//
// 静止起步时 `Δs/v → ∞` 使段时长约束形同虚设：实机实测首点加速度在一个控制
// 周期内从 0.309 直接跳到 0.500（jerk 1.90，限值 0.5），且这是整条剖面上唯一
// 超限处。本用例扫初速与初加速度的组合，钉住"首点变化率不超过 j·control_dt"。
TEST(ReachabilityProfile, FirstPointAccelRespectsControlPeriodJerkLimit)
{
  for (double v0 : {0.0, 0.006, 0.05, 0.3}) {
    for (double a0 : {0.0, 0.1, 0.309}) {
      auto w = makeWork(flatEnv(250, kMaxSpeed));
      Backend b;
      b.generateProfile(w, v0, a0, kMaxSpeed, false);
      const double step = std::fabs(w.front().accel - a0);
      EXPECT_LE(step, kJerk * kControlDt + 1e-6)
        << "v0=" << v0 << " a0=" << a0
        << " 首点 a=" << w.front().accel << "（seed " << a0 << "）"
        << " 单周期变化 " << step << " > j·control_dt=" << kJerk * kControlDt;
    }
  }
}

// 逐周期推进（receding horizon）：把上一版计划的首点加速度当作下一版的种子，
// 连跑多个周期。离线场景库只跑单次规划，结构上看不见跨周期的加速度台阶——
// 实机那条 jerk 1.90 正是这样漏掉的。
TEST(ReachabilityProfile, AccelRampAcrossCyclesStaysJerkLimited)
{
  // 执行模型：**沿计划前进并取该处的计划速度**（完美跟踪）。
  // 不能用 `v += a·control_dt` 的前向欧拉——一个控制周期机器人走 5~6 cm、
  // 跨近 3 个 2 cm 路径段，而规划器的 `landing` 只按一段算；两边视野不同，
  // 测出来的"跨周期 jerk"是模型错配，不是规划器缺陷。⑤ 实际也是沿计划取速度。
  auto planVelocityAt = [](const std::vector<TrajPointWork> & w, double dist) {
      double s = 0.0;
      for (size_t i = 1; i < w.size(); ++i) {
        const double ds = std::hypot(
          w[i].pose.pose.position.x - w[i - 1].pose.pose.position.x,
          w[i].pose.pose.position.y - w[i - 1].pose.pose.position.y);
        if (s + ds >= dist) {
          const double t = ds > 1e-9 ? (dist - s) / ds : 0.0;
          return w[i - 1].velocity_upper_bound +
                 t * (w[i].velocity_upper_bound - w[i - 1].velocity_upper_bound);
        }
        s += ds;
      }
      return w.back().velocity_upper_bound;
    };

  double v = 0.0, a = 0.0, worst_jerk = 0.0, peak_a = 0.0;
  int cycles_to_peak = -1;
  for (int cycle = 0; cycle < 60; ++cycle) {
    auto w = makeWork(flatEnv(250, kMaxSpeed));
    Backend b;
    b.generateProfile(w, v, a, kMaxSpeed, false);
    const double a_next = w.front().accel;
    worst_jerk = std::max(worst_jerk, std::fabs(a_next - a) / kControlDt);
    if (a_next > peak_a) {
      peak_a = a_next;
      if (peak_a >= kAccel - 1e-6 && cycles_to_peak < 0) {cycles_to_peak = cycle;}
    }
    v = planVelocityAt(w, v * kControlDt + 0.5 * a_next * kControlDt * kControlDt);
    a = a_next;
  }
  EXPECT_LE(worst_jerk, kJerk + 1e-3) << "跨周期 max|jerk|=" << worst_jerk;
  // 约束收紧不得把起步变成爬行：加速度仍须爬满，且用时恰为 accel/jerk。
  EXPECT_NEAR(peak_a, kAccel, 1e-6) << "加速度未爬满";
  EXPECT_EQ(cycles_to_peak + 1, static_cast<int>(std::lround(kAccel / kJerk / kControlDt)))
    << "爬满用了 " << cycles_to_peak + 1 << " 个周期，应为 accel/jerk/control_dt";
  // 末态：已到巡航，加速度正确收回零（而不是停在上限）。
  EXPECT_NEAR(v, kMaxSpeed, 2e-3);
  EXPECT_NEAR(a, 0.0, 2e-3);
}

// 安全置零（阻塞）绝对优先：起点被置零时不得被初速抬起来。
TEST(ReachabilityProfile, BlockedStartIsNotLiftedByInitialVelocity)
{
  auto w = makeWork(flatEnv(250, kMaxSpeed));
  for (auto & p : w) {p.velocity_upper_bound = 0.0;}
  Backend b;
  b.generateProfile(w, 0.5, 0.0, 0.0, false);
  EXPECT_NEAR(w.front().velocity_upper_bound, 0.0, 1e-9);
}
