// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <algorithm>

#include "gtest/gtest.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_pid_controller/pid_controller.hpp"

// 速度环单测：直调 LinearSpeedControl（不依赖 ROS 运行时）。
// station PID 增益置零，target 由 min_target_speed 直接给定（符号跟随 e_dist），
// 便于逐场景控制速度环的跟踪目标。
class SpeedLoopPID : public nav2_pid_controller::PIDController
{
public:
  SpeedLoopPID()
  {
    d_t_ = 0.05;
    max_v_ = 1.0;
    min_v_ = 0.0;
    max_v_inc_ = 0.5;
    switch_speed_ = 0.0;
    desired_linear_vel_ = 0.5;
    default_desired_linear_vel_ = 0.15;
    speed_cmd_reseed_error_ = 0.3;

    nav2_controller_common::PidConf station;
    nav2_controller_common::PidConf speed;
    speed.kp = 2.0;
    speed.ki = 0.5;
    speed.integrator_enable = true;
    speed.integrator_saturation_level = 0.5;
    high_speed_station_pid_conf_ = station;
    low_speed_station_pid_conf_ = station;
    reverse_station_pid_conf_ = station;
    high_speed_pid_conf_ = speed;
    low_speed_pid_conf_ = speed;
    reverse_speed_pid_conf_ = speed;
    station_nav2_pid_controller_.Init(station);
    speed_nav2_pid_controller_.Init(speed);
  }

  double run(double measured, double e_dist, double max_v, double target, double ff = 0.0)
  {
    geometry_msgs::msg::Twist tw;
    tw.linear.x = measured;
    return LinearSpeedControl(tw, e_dist, max_v, target, 0.0, ff);
  }

  void setDesired(double v) {desired_linear_vel_ = v;}
  void setDefaultDesired(double v) {default_desired_linear_vel_ = v;}
  void invalidateIntegrator() {last_speed_cmd_valid_ = false;}
  double lastCmd() const {return last_speed_cmd_;}
  int speedIntegratorSaturation() const
  {
    return speed_nav2_pid_controller_.IntegratorSaturationStatus();
  }
};

// 指令积分器：滞后 plant 下指令能主动下探到目标（不被实测拖着走），
// 且前向指令不下穿零。
TEST(PidSpeedLoop, commandIntegratorActivelyBrakes)
{
  SpeedLoopPID pid;
  double measured = 0.5;
  double cmd = measured;
  double min_cmd_minus_measured = 1e9;
  int reach_cycle = -1;
  for (int i = 0; i < 60; ++i) {
    cmd = pid.run(measured, 1.0, 1.0, 0.2);
    min_cmd_minus_measured = std::min(min_cmd_minus_measured, cmd - measured);
    EXPECT_GE(cmd, -1e-9);
    if (reach_cycle < 0 && cmd <= 0.2 + 1e-6) {
      reach_cycle = i;
    }
    measured += (cmd - measured) * 0.1;  // 一阶滞后 plant
  }
  // 指令显著低于实测 = 主动制动权（锚定实测的旧结构只能低一个步长 0.025）
  EXPECT_LT(min_cmd_minus_measured, -0.05);
  // 指令按 max_v_inc 坡道下行：(0.5-0.2)/(0.5*0.05)=12 周期量级即应到达目标，
  // 不受 plant 时间常数拖累
  ASSERT_GE(reach_cycle, 0);
  EXPECT_LE(reach_cycle, 15);
  EXPECT_NEAR(measured, 0.2, 0.05);
}

// 前馈与瞬时速度误差反号时不得被方向守卫改写：
// 目标略高于实测但减速前馈已到，输出应下降而非加速。
TEST(PidSpeedLoop, feedforwardNotVetoedBySign)
{
  SpeedLoopPID pid;
  const double cmd = pid.run(0.4, 1.0, 1.0, 0.42, -0.5);
  EXPECT_LT(cmd, 0.4);
}

// 满停（限幅链全零）清速度环积分：饱和的积分器在停车后必须归零。
TEST(PidSpeedLoop, fullStopClearsIntegral)
{
  SpeedLoopPID pid;
  for (int i = 0; i < 100; ++i) {
    pid.run(0.0, 1.0, 1.0, 0.9);
  }
  EXPECT_EQ(pid.speedIntegratorSaturation(), 1);

  pid.setDesired(0.0);
  pid.setDefaultDesired(0.0);
  const double stop_cmd = pid.run(0.3, 1.0, 0.0, 0.0);
  EXPECT_EQ(stop_cmd, 0.0);
  EXPECT_EQ(pid.speedIntegratorSaturation(), 0);
  EXPECT_EQ(pid.lastCmd(), 0.0);
}

// 符号感知输出界：前向目标是上限（持续正前馈不超目标）、
// 下界为零（持续负前馈不穿零倒转）；倒退目标对称。
TEST(PidSpeedLoop, signAwareOutputBounds)
{
  SpeedLoopPID up;
  double measured = 0.1;
  double max_cmd = -1e9;
  for (int i = 0; i < 40; ++i) {
    const double cmd = up.run(measured, 1.0, 1.0, 0.2, 0.5);
    max_cmd = std::max(max_cmd, cmd);
    measured = cmd;  // 完美 plant
  }
  EXPECT_LE(max_cmd, 0.2 + 1e-9);

  SpeedLoopPID down;
  measured = 0.15;
  double min_cmd = 1e9;
  for (int i = 0; i < 40; ++i) {
    const double cmd = down.run(measured, 1.0, 1.0, 0.2, -0.5);
    min_cmd = std::min(min_cmd, cmd);
    measured = cmd;
  }
  EXPECT_GE(min_cmd, -1e-9);

  SpeedLoopPID rev;
  measured = 0.0;
  double rev_min = 1e9, rev_max = -1e9;
  for (int i = 0; i < 40; ++i) {
    const double cmd = rev.run(measured, -1.0, 1.0, 0.1);
    rev_min = std::min(rev_min, cmd);
    rev_max = std::max(rev_max, cmd);
    measured = cmd;
  }
  EXPECT_GE(rev_min, -0.1 - 1e-9);
  EXPECT_LE(rev_max, 1e-9);
}

// reseed：首拍从实测播种（指令连续），指令与实测脱节超阈值时对齐实测。
TEST(PidSpeedLoop, reseedOnDivergence)
{
  SpeedLoopPID pid;
  const double first = pid.run(0.45, 1.0, 1.0, 0.5);
  EXPECT_GE(first, 0.45 - 1e-9);

  for (int i = 0; i < 5; ++i) {
    pid.run(0.5, 1.0, 1.0, 0.5);
  }
  EXPECT_NEAR(pid.lastCmd(), 0.5, 0.05);

  const double after_jump = pid.run(0.05, 1.0, 1.0, 0.5);
  EXPECT_LT(after_jump, 0.15);
}

// approach 阶段单测：目标锁定 / 朝向-速度平滑耦合 / 旋转量安全网。
// 不依赖 TF/costmap——getClosestWaypointID 直接吃 TrajPoint 向量，
// headingSpeedScale/updateRotationWatchdog 是纯状态方法。
class ApproachStagePID : public nav2_pid_controller::PIDController
{
public:
  static std::vector<TrajPoint> makeStraightTraj(double length, double step = 0.02)
  {
    std::vector<TrajPoint> traj;
    const int n = static_cast<int>(length / step) + 1;
    for (int i = 0; i < n; ++i) {
      TrajPoint p;
      p.pose.pose.position.x = i * step;
      p.pose.pose.position.y = 0.0;
      p.pose.pose.orientation.w = 1.0;
      traj.push_back(p);
    }
    return traj;
  }
  int closestWaypointWrapper(
    const std::vector<TrajPoint> & traj,
    const geometry_msgs::msg::PoseStamped & pose,
    double ignore_dist, bool ignore_first)
  {
    return getClosestWaypointID(traj, pose, ignore_dist, ignore_first);
  }
  static double headingScale(double err_theta) {return headingSpeedScale(err_theta);}
  void resetWatchdog(double theta_rob) {resetRotationWatchdog(theta_rob);}
  bool tickWatchdog(double theta_rob) {return updateRotationWatchdog(theta_rob);}
  void setRotationBudget(double rad) {approach_rotation_budget_ = rad;}
};

namespace
{
geometry_msgs::msg::PoseStamped makePose(double x, double y)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.orientation.w = 1.0;
  return ps;
}

double wrapAngle(double a) {return std::atan2(std::sin(a), std::cos(a));}
}  // namespace

// 目标锁定：距终点 < final_target_lock_distance_ 时直接锁定终点索引，
// 不再随横向误差在候选点间跳变。
TEST(ApproachStage, targetLockPinsToGoalWithinLockDistance)
{
  ApproachStagePID pid;
  const auto traj = ApproachStagePID::makeStraightTraj(1.0);
  const auto pose = makePose(0.85, 0.15);  // 距终点(1,0) hypot(0.15,0.15)=0.212m

  const int locked = pid.closestWaypointWrapper(traj, pose, 0.3, true);
  EXPECT_EQ(locked, static_cast<int>(traj.size()) - 1);

  const int unlocked = pid.closestWaypointWrapper(traj, pose, 0.1, true);
  EXPECT_NE(unlocked, static_cast<int>(traj.size()) - 1);
}

// 朝向-速度平滑耦合：连续、无跳变，err_theta=0 时不缩放，
// |err_theta|>=90° 时缩为 0。
TEST(ApproachStage, headingSpeedScaleIsSmoothAndBounded)
{
  EXPECT_DOUBLE_EQ(ApproachStagePID::headingScale(0.0), 1.0);
  EXPECT_NEAR(ApproachStagePID::headingScale(M_PI / 4.0), 0.70710678, 1e-6);
  EXPECT_NEAR(ApproachStagePID::headingScale(M_PI / 2.0), 0.0, 1e-9);
  EXPECT_EQ(ApproachStagePID::headingScale(M_PI), 0.0);
  // 偶函数：正负误差同幅度缩放一致（不引入方向偏置）
  EXPECT_DOUBLE_EQ(ApproachStagePID::headingScale(1.0), ApproachStagePID::headingScale(-1.0));
  // 连续性：相邻小角度增量下缩放变化平滑，无阶跃
  double prev = ApproachStagePID::headingScale(0.0);
  for (int i = 1; i <= 90; ++i) {
    const double cur = ApproachStagePID::headingScale(i * M_PI / 180.0);
    EXPECT_LT(std::fabs(cur - prev), 0.02);  // 每度变化 < 0.02，无跳变
    prev = cur;
  }
}

// 旋转量安全网：持续单向旋转累计超预算后触发；未超时不触发。
TEST(ApproachStage, rotationWatchdogTripsOnSustainedSpin)
{
  ApproachStagePID pid;
  pid.setRotationBudget(200.0 * M_PI / 180.0);
  pid.resetWatchdog(wrapAngle(0.0));

  bool tripped = false;
  int trip_step = -1;
  for (int i = 1; i <= 60; ++i) {
    tripped = pid.tickWatchdog(wrapAngle(i * 0.1));  // 持续 +0.1rad/步，跨越 ±pi 卷绕
    if (tripped) {trip_step = i; break;}
  }
  ASSERT_TRUE(tripped);
  // 200deg/0.1rad ≈ 35 步触发，允许小范围浮动
  EXPECT_GE(trip_step, 33);
  EXPECT_LE(trip_step, 37);
}

// 旋转量安全网：未持续单向旋转（收敛后停止）不应误触发。
TEST(ApproachStage, rotationWatchdogNotTrippedOnConvergence)
{
  ApproachStagePID pid;
  pid.setRotationBudget(200.0 * M_PI / 180.0);
  pid.resetWatchdog(wrapAngle(1.5));  // 初始误差 ~86°

  // 单调收敛到 0，总旋转量 = 初始误差 < 180° < 200° 预算
  bool tripped = false;
  for (int i = 0; i <= 30; ++i) {
    const double theta = 1.5 * (1.0 - static_cast<double>(i) / 30.0);
    tripped = pid.tickWatchdog(wrapAngle(theta)) || tripped;
  }
  EXPECT_FALSE(tripped);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
