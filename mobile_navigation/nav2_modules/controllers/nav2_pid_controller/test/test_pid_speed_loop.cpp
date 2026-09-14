// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <cmath>
#include <cstdint>

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
    max_linear_accel_ = 0.5;
    max_linear_decel_ = 0.5;
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
  void setNegativeFeedforwardGuard(double error, double max_target_speed)
  {
    negative_ff_guard_error_ = error;
    negative_ff_max_target_speed_ = max_target_speed;
  }
  void setOverspeedUndercut(double gain, double max_undercut)
  {
    negative_ff_overspeed_undercut_gain_ = gain;
    negative_ff_max_target_undercut_ = max_undercut;
  }
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

// 末端切入PID时，规划速度既是上限也是本周期跟踪参考。即使位置环按
// kp*distance只给出0.25 m/s，也不能把0.45 m/s规划参考一拍钳成0.25。
TEST(PidSpeedLoop, plannedSpeedReferenceIsNotReducedByPositionLoop)
{
  SpeedLoopPID pid;
  const double cmd = pid.run(0.47, 0.25, 0.45, 0.45, -0.2);
  EXPECT_GT(cmd, 0.40);
  EXPECT_LE(cmd, 0.45);
}

// 反死锁（R49）：车已静止、目标要求前进，但前馈是负值（采样点配到了制动段）。
// 此时 last_speed_cmd_ + ff·dt < 0 会被符号界钳回 0，而 0 又成为下一周期的
// 指令基准——逐周期自复现，机器人永远起不来（实机末端 6cm 不可达）。
// 前馈描述的工况与被跟踪的目标速度不一致时，本周期必须退化为纯反馈。
TEST(PidSpeedLoop, standstillWithBrakingFeedforwardStillStarts)
{
  SpeedLoopPID pid;
  double cmd = 0.0;
  for (int i = 0; i < 5; ++i) {
    cmd = pid.run(0.0, 0.0669, 0.119115, 0.119115, -0.4);
    if (cmd > 0.0) {
      break;
    }
  }
  EXPECT_GT(cmd, 0.0);

  // 但"已在跑 + 负前馈"仍必须正常制动，守卫不得把制动权限也一起吃掉。
  SpeedLoopPID braking;
  EXPECT_LT(braking.run(0.4, 1.0, 1.0, 0.42, -0.5), 0.4);
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

// NEGATIVE_FF_CONSISTENCY_GUARD_BEGIN
TEST(PidSpeedLoop, terminalGuardPreventsFurtherBrakingBelowNonzeroTarget)
{
  SpeedLoopPID guarded;
  guarded.setNegativeFeedforwardGuard(0.01, 0.10);
  guarded.run(0.01, 1.0, 0.01, 0.01, 0.0);  // 播种上一拍cmd=实测
  const double guarded_cmd = guarded.run(0.01, 1.0, 0.035, 0.035, -0.20);

  SpeedLoopPID unguarded;
  // max_target_speed低于本用例目标，相当于关闭整个末段保护范围。
  unguarded.setNegativeFeedforwardGuard(1.0, 0.001);
  unguarded.run(0.01, 1.0, 0.01, 0.01, 0.0);
  const double unguarded_cmd = unguarded.run(0.01, 1.0, 0.035, 0.035, -0.20);

  EXPECT_GT(guarded_cmd, unguarded_cmd + 0.005);
  EXPECT_GE(guarded_cmd, 0.01 - 1e-6);
}

// 当前指令仍在目标上方时，强负前馈最多把本拍制动到目标，不得直接穿过
// 目标形成“规划0.05、cmd先到0.01”的下探。
TEST(PidSpeedLoop, terminalGuardStopsAtCurrentNonzeroTarget)
{
  SpeedLoopPID pid;
  pid.setNegativeFeedforwardGuard(0.01, 0.10);
  pid.run(0.08, 1.0, 0.08, 0.08, 0.0);
  const double cmd = pid.run(0.08, 1.0, 0.05, 0.05, -0.60);
  EXPECT_NEAR(cmd, 0.05, 1e-9);
}

// 复现164655：cmd已经落后于0.072目标，但odom尚未响应、仍约等于目标。
// 旧保护要等odom也落后才触发；一拍预测保护必须立即阻止cmd继续向零下降。
TEST(PidSpeedLoop, terminalGuardDoesNotWaitForMeasuredVelocityToLag)
{
  SpeedLoopPID pid;
  pid.setNegativeFeedforwardGuard(0.01, 0.10);
  const double previous_cmd = pid.run(0.052, 1.0, 0.052, 0.052, 0.0);
  const double guarded_cmd = pid.run(0.072, 1.0, 0.072, 0.072, -0.372);
  EXPECT_NEAR(previous_cmd, 0.052, 1e-9);
  EXPECT_GE(guarded_cmd, previous_cmd - 1e-9);
}

// 复现174118 L3：目标约0.0505但odom仍约0.125。此时必须允许有限低于
// 目标补刹，同时由0.03m/s最大下探量保证不会重新掉到0.01附近。
TEST(PidSpeedLoop, terminalGuardAllowsBoundedBrakingForMeasuredOverspeed)
{
  SpeedLoopPID pid;
  pid.setNegativeFeedforwardGuard(0.01, 0.15);
  pid.setOverspeedUndercut(0.50, 0.03);
  pid.run(0.05, 1.0, 0.05, 0.05, 0.0);
  const double cmd = pid.run(0.125, 1.0, 0.0505, 0.0505, -0.153);

  EXPECT_LT(cmd, 0.0505);
  EXPECT_GE(cmd, 0.0205 - 1e-9);
}

// 复现实机L3->L1：空间消费目标停在约0.08，cmd/odom已经落后，负前馈仍为
// -0.148。守卫不能只把合成加速度钳到0并冻结低cmd，而应撤销矛盾前馈，保留
// 正向速度反馈，让机器人重新获得空间进度并自然消费到后续0.05目标。
TEST(PidSpeedLoop, terminalGuardRetainsPositiveRecoveryFeedbackAtObservedTarget)
{
  SpeedLoopPID pid;
  pid.setNegativeFeedforwardGuard(0.01, 0.10);
  const double previous_cmd = pid.run(0.03, 1.0, 0.03, 0.03, 0.0);
  const double recovered_cmd = pid.run(0.04, 1.0, 0.0805, 0.0805, -0.148);

  EXPECT_NEAR(previous_cmd, 0.03, 1e-9);
  EXPECT_GT(recovered_cmd, previous_cmd);
  EXPECT_LE(recovered_cmd, 0.0805 + 1e-9);
}

TEST(PidSpeedLoop, zeroTargetKeepsFullStopAuthority)
{
  SpeedLoopPID pid;
  pid.setNegativeFeedforwardGuard(0.0, 0.10);
  EXPECT_DOUBLE_EQ(pid.run(0.05, 1.0, 0.0, 0.0, -0.50), 0.0);
}

TEST(PidSpeedLoop, highSpeedBrakingIsUnchanged)
{
  SpeedLoopPID pid;
  pid.setNegativeFeedforwardGuard(0.01, 0.10);
  pid.run(0.18, 1.0, 0.18, 0.18, 0.0);
  EXPECT_LT(pid.run(0.18, 1.0, 0.20, 0.20, -0.20), 0.18);
}
// NEGATIVE_FF_CONSISTENCY_GUARD_END

// 角速度环单测：最终角度采用机器人当前的固定目标纯PID控制，
// 不依赖轨迹生成器或墙钟时间。
class AngularLoopPID : public nav2_pid_controller::PIDController
{
public:
  AngularLoopPID()
  {
    d_t_ = 0.05;
    max_w_ = 1.57;
    min_w_ = 0.0;
    max_w_inc_ = 1.0;
    angular_error_deadband_ = 0.008;
    angular_cmd_reseed_error_ = 0.3;
    nav2_controller_common::PidConf station;
    station.kp = 1.0;
    angular_station_pid_conf_ = station;
    angular_station_nav2_pid_controller_.Init(station);
  }

  double run(double measured_w, double err_theta, bool final_rotation = true)
  {
    geometry_msgs::msg::Twist tw;
    tw.angular.z = measured_w;
    return AngularSpeedControl(tw, err_theta, final_rotation);
  }

  double lastCmd() const {return last_angular_cmd_;}
};

TEST(AngularLoopPID_Test, finalYawPurePidIsRateLimitedAndOneSided)
{
  AngularLoopPID pid;
  double error = M_PI_2;
  double measured = 0.0;
  double max_command = 0.0;
  for (int i = 0; i < 300; ++i) {
    const double cmd = pid.run(measured, error, true);
    EXPECT_GE(cmd, -1e-6) << "fixed target must not reverse before capture";
    EXPECT_LE(std::fabs(cmd), 1.57 + 1e-6);
    max_command = std::max(max_command, std::fabs(cmd));
    measured = cmd;
    error -= measured * 0.05;
    if (std::fabs(error) <= 0.008 && std::fabs(cmd) <= 0.008) {
      break;
    }
  }
  EXPECT_GT(max_command, 0.45);
  EXPECT_NEAR(measured, 0.0, 0.02);
}

TEST(AngularLoopPID_Test, finalYawPurePidMirrorsSigns)
{
  for (double sign : {1.0, -1.0}) {
    AngularLoopPID pid;
    double error = sign * 0.3;
    double measured = 0.0;
    for (int i = 0; i < 200; ++i) {
      const double cmd = pid.run(measured, error, true);
      EXPECT_GE(cmd * sign, -1e-6);
      measured = cmd;
      error -= measured * 0.05;
      if (std::fabs(error) <= 0.008 && std::fabs(cmd) <= 0.008) {
        break;
      }
    }
    EXPECT_NEAR(measured, 0.0, 0.02);
  }
}

// 新任务重置单测：直接播种所有三路PID、阶段和上一周期指令基准，验证
// ControllerServer调用resetEndLocalization后不会把旧任务状态带入新目标。
class TaskResetPID : public nav2_pid_controller::PIDController
{
public:
  TaskResetPID()
  {
    nav2_controller_common::PidConf config;
    config.ki = 1.0;
    config.integrator_enable = true;
    config.integrator_saturation_level = 0.1;
    station_nav2_pid_controller_.Init(config);
    speed_nav2_pid_controller_.Init(config);
    angular_station_nav2_pid_controller_.Init(config);
  }

  void seedPreviousTaskState()
  {
    curr_controller_stage_ = ControllerStage::ROTATING_TO_GOAL;
    prev_controller_stage_ = ControllerStage::APPROACHING;
    theta_errors_.push_back(0.2);
    longitude_errors_.push_back(0.1);
    station_nav2_pid_controller_.Control(1.0, 1.0);
    speed_nav2_pid_controller_.Control(1.0, 1.0);
    angular_station_nav2_pid_controller_.Control(1.0, 1.0);
    last_speed_cmd_ = 0.25;
    last_speed_cmd_valid_ = true;
    last_angular_cmd_ = -0.4;
    last_angular_cmd_valid_ = true;
    last_trajectory_tracking_target_valid_ = true;
    last_trajectory_tracking_target_v_ = 0.2;
    last_trajectory_tracking_target_a_ = -0.1;
  }

  bool stagesAreFinished() const
  {
    return curr_controller_stage_ == ControllerStage::FINISH &&
           prev_controller_stage_ == ControllerStage::FINISH;
  }

  bool errorWindowsAreEmpty() const
  {
    return theta_errors_.empty() && longitude_errors_.empty();
  }

  bool commandSeedsAreCleared() const
  {
    return !last_speed_cmd_valid_ && !last_angular_cmd_valid_ &&
           last_speed_cmd_ == 0.0 && last_angular_cmd_ == 0.0;
  }

  bool trajectorySeedIsCleared() const
  {
    return !last_trajectory_tracking_target_valid_ &&
           last_trajectory_tracking_target_v_ == 0.0 &&
           last_trajectory_tracking_target_a_ == 0.0;
  }

  bool pidIntegralsAreSaturated() const
  {
    return station_nav2_pid_controller_.IntegratorSaturationStatus() != 0 &&
           speed_nav2_pid_controller_.IntegratorSaturationStatus() != 0 &&
           angular_station_nav2_pid_controller_.IntegratorSaturationStatus() != 0;
  }

  bool pidIntegralsAreCleared() const
  {
    return station_nav2_pid_controller_.IntegratorSaturationStatus() == 0 &&
           speed_nav2_pid_controller_.IntegratorSaturationStatus() == 0 &&
           angular_station_nav2_pid_controller_.IntegratorSaturationStatus() == 0;
  }

  void enableEndLocalizationLock(double switch_distance_m, uint64_t goal_generation)
  {
    nav2_pid_controller::EndLocalizationLockConfig config;
    config.switch_distance_m = switch_distance_m;
    end_localization_lock_.configure(config);
    resetEndLocalization(goal_generation);
  }

  nav2_core::EndLocalizationSnapshot selectPositionReference(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & trajectory_fallback_goal,
    geometry_msgs::msg::PoseStamped & effective_goal)
  {
    return selectEndLocalizationPositionReference(
      robot_pose, trajectory_fallback_goal, effective_goal);
  }
};

TEST(PidTaskReset, NewGoalClearsIntegralsStagesAndCommandSeeds)
{
  constexpr uint64_t kNewGoalGeneration = 7;
  TaskResetPID pid;
  pid.seedPreviousTaskState();
  ASSERT_TRUE(pid.pidIntegralsAreSaturated());

  pid.resetEndLocalization(kNewGoalGeneration);

  EXPECT_TRUE(pid.pidIntegralsAreCleared());
  EXPECT_TRUE(pid.stagesAreFinished());
  EXPECT_TRUE(pid.errorWindowsAreEmpty());
  EXPECT_TRUE(pid.commandSeedsAreCleared());
  EXPECT_TRUE(pid.trajectorySeedIsCleared());
  EXPECT_EQ(pid.getEndLocalizationSnapshot().goal_generation, kNewGoalGeneration);
}

TEST(PidEndLocalization, MissingAuthoritativeGoalCannotLatchTrajectoryEndpoint)
{
  constexpr uint64_t kGoalGeneration = 3;
  constexpr double kSwitchDistanceM = 0.05;
  TaskResetPID pid;
  pid.enableEndLocalizationLock(kSwitchDistanceM, kGoalGeneration);

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "odom";
  robot_pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::PoseStamped trajectory_endpoint = robot_pose;
  trajectory_endpoint.pose.position.x = 1.0;
  robot_pose.pose.position.x = trajectory_endpoint.pose.position.x;

  geometry_msgs::msg::PoseStamped control_goal;
  const auto snapshot = pid.selectPositionReference(
    robot_pose, trajectory_endpoint, control_goal);

  // TF失败时轨迹终点仍可供本周期控制，但锁存器必须保持无权威目标状态。
  EXPECT_DOUBLE_EQ(control_goal.pose.position.x, trajectory_endpoint.pose.position.x);
  EXPECT_FALSE(snapshot.live_goal_valid);
  EXPECT_FALSE(snapshot.effective_goal_valid);
  EXPECT_FALSE(snapshot.position_goal_latched);
  EXPECT_EQ(snapshot.goal_generation, kGoalGeneration);
}

// approach 阶段单测：目标锁定 / 朝向-速度平滑耦合 / 旋转量安全网。
// 不依赖 TF/costmap——getClosestWaypointID 直接吃 TrajPoint 向量，
// headingSpeedScale 是纯函数。
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
      p.x = p.pose.pose.position.x;
      p.y = p.pose.pose.position.y;
      traj.push_back(p);
    }
    return traj;
  }
  // 前段巡航速度 high_speed，从 length*decel_start_frac 处开始线性降到终点
  // low_speed——模拟④收尾减速剖面（终点是蠕行值，不是巡航值）。
  static std::vector<TrajPoint> makeDecliningSpeedTraj(
    double length, double high_speed, double low_speed,
    double decel_start_frac, double step = 0.02)
  {
    std::vector<TrajPoint> traj;
    const int n = static_cast<int>(length / step) + 1;
    const double decel_start_x = length * decel_start_frac;
    for (int i = 0; i < n; ++i) {
      TrajPoint p;
      const double x = i * step;
      p.pose.pose.position.x = x;
      p.pose.pose.position.y = 0.0;
      p.pose.pose.orientation.w = 1.0;
      p.x = x;
      p.y = 0.0;
      if (x <= decel_start_x) {
        p.speed_limit = high_speed;
      } else {
        const double ratio = (x - decel_start_x) / (length - decel_start_x);
        p.speed_limit = high_speed - (high_speed - low_speed) * std::min(1.0, ratio);
      }
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
  bool nearestSpeedWrapper(
    const geometry_msgs::msg::PoseStamped & pose, double & speed, double & accel)
  {
    return nearestTrajectorySpeed(pose, speed, accel);
  }
  void setControllerTraj(const std::vector<TrajPoint> & traj)
  {
    controller_traj_ = traj;
    trajectory_time_valid_ = nav2_core::hasValidRelativeTime(controller_traj_);
  }
  void setTrajSpeedLookahead(double v) {traj_speed_lookahead_ = v;}
  void setTrajTimeLookahead(double v) {traj_time_lookahead_ = v;}
  static double headingScale(double err_theta) {return headingSpeedScale(err_theta);}
  static bool inPositionWindow(
    double lon, double lat, double distance, double lon_tol, double lat_tol, double margin)
  {
    return isWithinPositionWindow(lon, lat, distance, lon_tol, lat_tol, margin);
  }
  static double retryHeading(
    double current_yaw, double goal_yaw, double goal_dx, double goal_dy,
    double lateral_tolerance, bool & reverse, bool & holds_goal_yaw)
  {
    return selectPositionRetryHeading(
      current_yaw, goal_yaw, goal_dx, goal_dy,
      lateral_tolerance, reverse, holds_goal_yaw);
  }
  void beginRetryCaptureTest()
  {
    curr_controller_stage_ = ControllerStage::RETRY_TRANSLATE;
    retry_position_capture_cycles_ = 3;
    retry_position_capture_count_ = 0;
  }
  bool confirmRetryPosition(bool in_tolerance, double velocity = 0.0)
  {
    return confirmPositionRetryCapture(in_tolerance, 0.0, velocity, 0.01);
  }
  bool retryStillActive() const
  {
    return curr_controller_stage_ == ControllerStage::RETRY_TRANSLATE;
  }
  bool rotatingToGoal() const
  {
    return curr_controller_stage_ == ControllerStage::ROTATING_TO_GOAL;
  }
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

// 2 cm 是进入最终位置保持的真实精度；2 cm 滞回只扩大退出窗口到 4 cm，
// 不能反过来把“到点精度”放宽成 4 cm。
TEST(ApproachStage, positionHoldUsesTwoCentimeterEntryAndFourCentimeterExit)
{
  EXPECT_TRUE(ApproachStagePID::inPositionWindow(0.019, 0.0, 0.019, 0.02, 0.02, 0.0));
  EXPECT_FALSE(ApproachStagePID::inPositionWindow(0.021, 0.0, 0.021, 0.02, 0.02, 0.0));

  EXPECT_TRUE(ApproachStagePID::inPositionWindow(0.039, 0.0, 0.039, 0.02, 0.02, 0.02));
  EXPECT_FALSE(ApproachStagePID::inPositionWindow(0.041, 0.0, 0.041, 0.02, 0.02, 0.02));
}

// L1复现：目标位于车后约179deg，但反向行驶只需约1deg转角。位置补偿
// 应选择低速倒退，而不是前转180deg、补位置、再转180deg回最终姿态。
TEST(ApproachStage, positionRetryChoosesReverseWhenGoalIsJustBehind)
{
  bool reverse = false;
  bool holds_goal_yaw = false;
  const double current_yaw = -2.0 * M_PI / 180.0;
  const double goal_bearing = -179.0 * M_PI / 180.0;
  const double heading = ApproachStagePID::retryHeading(
    current_yaw, 0.0,
    std::cos(goal_bearing), std::sin(goal_bearing), 0.02,
    reverse, holds_goal_yaw);
  double error = heading - current_yaw;
  while (error > M_PI) {error -= 2.0 * M_PI;}
  while (error < -M_PI) {error += 2.0 * M_PI;}

  EXPECT_TRUE(reverse);
  EXPECT_TRUE(holds_goal_yaw);
  EXPECT_LT(std::fabs(error), 5.0 * M_PI / 180.0);
}

TEST(ApproachStage, positionRetryKeepsForwardWhenGoalIsAhead)
{
  bool reverse = true;
  bool holds_goal_yaw = false;
  const double heading = ApproachStagePID::retryHeading(
    0.1, 0.0, 1.0, 0.0, 0.02, reverse, holds_goal_yaw);
  EXPECT_FALSE(reverse);
  EXPECT_TRUE(holds_goal_yaw);
  EXPECT_NEAR(heading, 0.0, 1e-9);
}

// 复现172229的L3：最终朝向180deg，目标还在全局+x方向约2.4cm，
// 即位于机器人最终朝向的正后方。必须保持180deg直接倒退，不能根据厘米级
// atan2重新生成160--180deg之间不断变化的补偿角。
TEST(ApproachStage, finalYawRetryKeepsGoalYawForSmallLateralError)
{
  bool reverse = false;
  bool holds_goal_yaw = false;
  const double heading = ApproachStagePID::retryHeading(
    M_PI, M_PI, 0.024, -0.0004, 0.02, reverse, holds_goal_yaw);
  EXPECT_TRUE(reverse);
  EXPECT_TRUE(holds_goal_yaw);
  EXPECT_NEAR(std::fabs(heading), M_PI, 1e-9);
}

TEST(ApproachStage, lateralMissFallsBackToPointBearingHeading)
{
  bool reverse = false;
  bool holds_goal_yaw = true;
  const double heading = ApproachStagePID::retryHeading(
    M_PI, M_PI, 0.01, 0.03, 0.02, reverse, holds_goal_yaw);
  EXPECT_FALSE(holds_goal_yaw);
  EXPECT_TRUE(std::isfinite(heading));
}

TEST(ApproachStage, positionRetryRequiresThreeStableCaptureCycles)
{
  ApproachStagePID pid;
  pid.beginRetryCaptureTest();

  EXPECT_TRUE(pid.confirmRetryPosition(true));
  EXPECT_TRUE(pid.retryStillActive());
  // A single out-of-tolerance sample resets, rather than immediately toggling
  // between translation and final rotation.
  EXPECT_FALSE(pid.confirmRetryPosition(false));
  EXPECT_TRUE(pid.retryStillActive());
  EXPECT_TRUE(pid.confirmRetryPosition(true));
  EXPECT_TRUE(pid.confirmRetryPosition(true));
  EXPECT_TRUE(pid.retryStillActive());
  EXPECT_TRUE(pid.confirmRetryPosition(true));
  EXPECT_TRUE(pid.rotatingToGoal());
}

TEST(ApproachStage, singlePointTerminalTrajectoryIsAValidGoal)
{
  ApproachStagePID pid;
  const auto traj = ApproachStagePID::makeStraightTraj(0.0);

  ASSERT_EQ(traj.size(), 1u);
  EXPECT_EQ(pid.closestWaypointWrapper(traj, makePose(0.0, 0.0), 0.3, true), 0);
}

// 速度参考与瞄准目标解耦：瞄准目标在 lock 距离内锁定到终点索引（不变），
// 但速度参考不得跟着锁到终点的蠕行值——应反映最近点附近仍在下降的连续曲线。
TEST(ApproachStage, speedDecoupledFromLockedTargetIndex)
{
  ApproachStagePID pid;
  pid.setTrajSpeedLookahead(0.1);
  const auto traj = ApproachStagePID::makeDecliningSpeedTraj(1.0, 0.5, 0.02, 0.7);
  pid.setControllerTraj(traj);
  const auto pose = makePose(0.85, 0.15);  // 距终点(1,0) hypot(0.15,0.15)=0.212m < lock 0.3

  const int locked = pid.closestWaypointWrapper(traj, pose, 0.3, true);
  ASSERT_EQ(locked, static_cast<int>(traj.size()) - 1);  // 瞄准目标确实锁到终点
  ASSERT_NEAR(traj.back().speed_limit, 0.02, 1e-9);       // 终点是蠕行值

  double speed = 0.0, accel = 0.0;
  ASSERT_TRUE(pid.nearestSpeedWrapper(pose, speed, accel));
  EXPECT_GT(speed, 0.1);   // 明显高于终点蠕行值——没有跟着瞄准目标锁到 0.02
  EXPECT_LT(speed, 0.5);   // 明显低于巡航值——确实反映了减速段的连续下降
}

// 速度取窗口内极大点，加速度取"到达该点"的那一段——极大点之后的段必然是
// 制动值，配到峰值速度上就是自相矛盾的 (v,a)，静止起步会被速度环符号界钳死。
TEST(ApproachStage, accelerationComesFromSegmentArrivingAtPreviewPoint)
{
  ApproachStagePID pid;
  pid.setTrajSpeedLookahead(0.11);
  auto traj = ApproachStagePID::makeStraightTraj(0.16, 0.05);
  ASSERT_EQ(traj.size(), 4u);
  traj[0].speed_limit = 0.0;
  traj[0].accel = 0.0;
  traj[1].speed_limit = 0.1;
  traj[1].accel = 0.1;
  traj[2].speed_limit = 0.25;
  traj[2].accel = 0.35;
  traj[3].speed_limit = 0.2;
  traj[3].accel = -0.1;
  pid.setControllerTraj(traj);

  double speed = 0.0, accel = 0.0;
  ASSERT_TRUE(pid.nearestSpeedWrapper(makePose(0.0, 0.0), speed, accel));
  EXPECT_NEAR(speed, 0.25, 1e-9);
  EXPECT_NEAR(accel, 0.1, 1e-9);   // traj[1]→traj[2] 这一段，不是 traj[2] 之后
}

TEST(ApproachStage, trajectorySpeedUsesTimeAlignedInterpolation)
{
  ApproachStagePID pid;
  pid.setTrajSpeedLookahead(0.1);
  pid.setTrajTimeLookahead(0.08);
  auto traj = ApproachStagePID::makeStraightTraj(0.10, 0.02);
  ASSERT_EQ(traj.size(), 6u);
  traj.resize(4);
  traj[0].speed_limit = 0.60;
  traj[0].accel = -0.10;
  traj[0].relative_time = 0.00;
  traj[1].speed_limit = 0.55;
  traj[1].accel = -0.20;
  traj[1].relative_time = 0.04;
  traj[2].speed_limit = 0.45;
  traj[2].accel = -0.40;
  traj[2].relative_time = 0.12;
  traj[3].speed_limit = 0.35;
  traj[3].accel = -0.40;
  traj[3].relative_time = 0.20;
  pid.setControllerTraj(traj);

  double velocity = 0.0;
  double acceleration = 0.0;
  ASSERT_TRUE(pid.nearestSpeedWrapper(makePose(0.01, 0.0), velocity, acceleration));
  EXPECT_NEAR(velocity, 0.475, 1e-9);
  EXPECT_NEAR(acceleration, -0.35, 1e-9);
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

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
