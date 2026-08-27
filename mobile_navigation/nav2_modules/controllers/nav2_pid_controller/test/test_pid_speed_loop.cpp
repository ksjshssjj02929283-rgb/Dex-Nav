// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>

#include "gtest/gtest.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
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

  double runTarget(double measured, double target, double limit, double ff = 0.0)
  {
    geometry_msgs::msg::Twist tw;
    tw.linear.x = measured;
    return TrackTargetSpeed(tw, target, limit, ff);
  }

  void setDesired(double v) {desired_linear_vel_ = v;}
  void setDefaultDesired(double v) {default_desired_linear_vel_ = v;}
  void setReverse(bool enabled) {reverse_enabled_ = enabled;}
  void invalidateIntegrator() {last_speed_cmd_valid_ = false;}
  double lastCmd() const {return last_speed_cmd_;}
  int speedIntegratorSaturation() const
  {
    return speed_nav2_pid_controller_.IntegratorSaturationStatus();
  }
};

// 公共速度内环必须是机械搬迁：位置PID输出被min_target_speed固定后，
// LinearSpeedControl与直接调用TrackTargetSpeed应得到完全相同的速度命令。
TEST(PidSpeedLoop, extractedTargetTrackerPreservesExistingNumerics)
{
  constexpr double kTolerance = 1e-12;

  {
    SpeedLoopPID original_path;
    SpeedLoopPID extracted_path;
    const double original = original_path.run(0.1, 1.0, 0.6, 0.3, 0.2);
    const double extracted = extracted_path.runTarget(0.1, 0.3, 0.6, 0.2);
    EXPECT_NEAR(original, extracted, kTolerance);
  }

  {
    SpeedLoopPID original_path;
    SpeedLoopPID extracted_path;
    const double original = original_path.run(0.4, 1.0, 0.8, 0.42, -0.5);
    const double extracted = extracted_path.runTarget(0.4, 0.42, 0.8, -0.5);
    EXPECT_NEAR(original, extracted, kTolerance);
  }

  {
    SpeedLoopPID original_path;
    SpeedLoopPID extracted_path;
    original_path.setReverse(true);
    extracted_path.setReverse(true);
    const double original = original_path.run(0.0, -1.0, 0.2, 0.1);
    const double extracted = extracted_path.runTarget(0.0, -0.1, 0.2);
    EXPECT_NEAR(original, extracted, kTolerance);
  }
}

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

// 角速度环单测：直调 AngularSpeedControl（不依赖 ROS 运行时）。
// final_rotation=true 分支为 kp*err_theta 纯 P，target 由 err_theta 直接给定，
// 便于验证变化率限幅（镜像速度环 SpeedLoopPID 的测试结构）。
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

// 变化率限幅：朝向误差远超 max_w/kp（目标饱和到 max_w）时，首拍指令不得
// 直接砸到目标，须按 max_w_inc_ 逐周期爬升；多周期完美 plant 反馈后收敛到目标。
TEST(AngularLoopPID_Test, rateLimitedOnLargeStepError)
{
  AngularLoopPID pid;
  const double cmd1 = pid.run(0.0, 2.0, true);  // target=clamp(1.0*2.0, ..., 1.57)=1.57(饱和)
  EXPECT_GT(cmd1, 0.0);
  EXPECT_LT(cmd1, 0.1);  // 远低于饱和目标 1.57——证明没有单周期直接跳变

  double cmd = cmd1;
  double measured = cmd1;
  for (int i = 0; i < 60; ++i) {
    cmd = pid.run(measured, 2.0, true);
    measured = cmd;  // 完美 plant
  }
  EXPECT_NEAR(cmd, 1.57, 0.05);
}

// 符号感知输出界：target_w 是跟踪参考也是上/下限，持续同向反馈不越过目标。
TEST(AngularLoopPID_Test, signAwareOutputBoundsAngular)
{
  AngularLoopPID up;
  double measured = 0.0;
  double max_cmd = -1e9;
  for (int i = 0; i < 60; ++i) {
    const double cmd = up.run(measured, 0.3, true);  // target=0.3（未饱和）
    max_cmd = std::max(max_cmd, cmd);
    measured = cmd;
  }
  EXPECT_LE(max_cmd, 0.3 + 1e-6);

  AngularLoopPID down;
  measured = 0.0;
  double min_cmd = 1e9;
  for (int i = 0; i < 60; ++i) {
    const double cmd = down.run(measured, -0.3, true);
    min_cmd = std::min(min_cmd, cmd);
    measured = cmd;
  }
  EXPECT_GE(min_cmd, -0.3 - 1e-6);
}

// reseed：指令与实测脱节超阈值时对齐实测（同速度环 reseedOnDivergence）。
TEST(AngularLoopPID_Test, reseedOnAngularDivergence)
{
  AngularLoopPID pid;
  for (int i = 0; i < 5; ++i) {
    pid.run(0.5, 0.5, true);
  }
  EXPECT_NEAR(pid.lastCmd(), 0.5, 0.05);

  const double after_jump = pid.run(0.0, 0.5, true);  // 实测骤降到 0，脱节超 0.3 阈值
  EXPECT_LT(after_jump, 0.15);  // 从 0 重新播种爬升，而非贴着旧指令 0.5
}

// deadband 内清零并把指令积分器播种为 0：退出 deadband 后从 0 爬坡，
// 不会贴着 deadband 前的陈旧指令继续（同速度环满停清积分的用意）。
TEST(AngularLoopPID_Test, deadbandZeroesAndReseeds)
{
  AngularLoopPID pid;
  double measured = 0.0;
  for (int i = 0; i < 10; ++i) {
    measured = pid.run(measured, 1.0, true);  // 完美 plant，先爬升出一个非零指令基准
  }
  ASSERT_GT(pid.lastCmd(), 0.3);

  const double cmd = pid.run(measured, 0.001, true);  // 落入 deadband(0.008)
  EXPECT_EQ(cmd, 0.0);

  const double after = pid.run(0.0, 1.0, true);  // 退出 deadband，实测已归零
  EXPECT_LT(after, 0.1);  // 从 0 重新爬坡，不是贴着 deadband 前的旧指令
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
};

// 用真实computeVelocityCommands链路验证规划零速优先级和连续性回执顺序。
// reverse_enabled让目标位于base_link后方时航向误差仍为0，从而单独覆盖
// “err_x<0会启用倒回权限，但traj_v=0必须先停车”的边界。
class ApproachingZeroSpeedPID : public nav2_pid_controller::PIDController
{
public:
  ApproachingZeroSpeedPID()
  {
    constexpr double kControlPeriodS = 0.05;
    constexpr double kMaxLinearVelocityMps = 1.0;
    constexpr double kMaxLinearAccelerationMps2 = 0.5;
    constexpr double kFinalCorrectionVelocityMps = 0.05;
    constexpr double kFinalEntryToleranceM = 0.001;
    constexpr double kSpeedFeedbackGain = 2.0;

    d_t_ = kControlPeriodS;
    max_v_ = kMaxLinearVelocityMps;
    min_v_ = 0.0;
    max_v_inc_ = kMaxLinearAccelerationMps2;
    switch_speed_ = 0.0;
    final_rotation_max_v_ = kFinalCorrectionVelocityMps;
    long_precision_ = kFinalEntryToleranceM;
    lat_precision_ = kFinalEntryToleranceM;
    final_rotation_entry_distance_ = kFinalEntryToleranceM;
    final_rotation_exit_distance_ = kFinalEntryToleranceM;
    reverse_enabled_ = true;
    verbose_ = 0;

    nav2_controller_common::PidConf station;
    nav2_controller_common::PidConf speed;
    speed.kp = kSpeedFeedbackGain;
    high_speed_station_pid_conf_ = station;
    low_speed_station_pid_conf_ = station;
    reverse_station_pid_conf_ = station;
    high_speed_pid_conf_ = speed;
    low_speed_pid_conf_ = speed;
    reverse_speed_pid_conf_ = speed;
    angular_station_pid_conf_ = station;
    station_nav2_pid_controller_.Init(station);
    speed_nav2_pid_controller_.Init(speed);
    angular_station_nav2_pid_controller_.Init(station);

    global_frame_ = "map";
    base_frame_ = "base_link";
    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf_ = std::make_shared<tf2_ros::Buffer>(clock_);
    tf_->setUsingDedicatedThread(true);
    end_localization_lock_.configure(nav2_pid_controller::EndLocalizationLockConfig{});
    curr_controller_stage_ = ControllerStage::APPROACHING;
    prev_controller_stage_ = ControllerStage::APPROACHING;
  }

  bool configureZeroSpeedOvershootScenario(double trajectory_acceleration)
  {
    constexpr double kGoalXM = 1.0;
    constexpr double kRobotOvershootM = 0.01;

    geometry_msgs::msg::TransformStamped map_to_base;
    map_to_base.header.stamp = rclcpp::Time(0);
    map_to_base.header.frame_id = global_frame_;
    map_to_base.child_frame_id = base_frame_;
    map_to_base.transform.translation.x = kGoalXM + kRobotOvershootM;
    map_to_base.transform.rotation.w = 1.0;
    if (!tf_->setTransform(map_to_base, "pid_speed_loop_test", true)) {
      return false;
    }

    TrajPoint goal;
    goal.pose.header.frame_id = global_frame_;
    goal.pose.pose.position.x = kGoalXM;
    goal.pose.pose.orientation.w = 1.0;
    goal.x = kGoalXM;
    goal.y = 0.0;
    goal.speed_limit = 0.0;
    goal.accel = trajectory_acceleration;
    controller_traj_ = {goal};
    trajectory_time_valid_ = false;

    robot_pose_.header.frame_id = global_frame_;
    robot_pose_.pose.position.x = kGoalXM + kRobotOvershootM;
    robot_pose_.pose.orientation.w = 1.0;
    return true;
  }

  geometry_msgs::msg::TwistStamped run()
  {
    geometry_msgs::msg::Twist velocity;
    return computeVelocityCommands(robot_pose_, velocity, nullptr);
  }

  bool getContinuityTarget(double & velocity, double & acceleration)
  {
    return getLastTrajectoryTrackingTarget(velocity, acceleration);
  }

private:
  geometry_msgs::msg::PoseStamped robot_pose_;
};

// 最终转角切换单测：构造“锁存位置已满足进入门限，但实时位置因map修正落在
// 进入/退出迟滞区间”的场景，验证控制器按开关选择实时Map位置或Odom锁存位置。
class FinalRotationTransitionPID : public ApproachingZeroSpeedPID
{
public:
  bool configureLatchedEntryWithLiveOffset(bool should_keep_position_lock_in_final_yaw)
  {
    constexpr uint64_t kGoalGeneration = 9;
    constexpr double kRobotXM = 0.99;
    constexpr double kLatchedGoalXM = 1.0;
    constexpr double kLiveGoalXM = 1.015;
    constexpr double kGoalYawRad = 0.2;

    reverse_enabled_ = false;
    long_precision_ = 0.02;
    lat_precision_ = 0.02;
    final_rotation_entry_distance_ = 0.02;
    final_rotation_exit_distance_ = 0.03;
    final_rotation_linear_deadband_ = 0.01;

    nav2_pid_controller::EndLocalizationLockConfig config;
    config.switch_distance_m = 0.05;
    config.switch_angle_rad = 0.075;
    config.should_keep_position_lock_in_final_yaw = should_keep_position_lock_in_final_yaw;
    end_localization_lock_.configure(config);
    resetEndLocalization(kGoalGeneration);

    robot_pose_.header.frame_id = global_frame_;
    robot_pose_.pose.position.x = kRobotXM;
    robot_pose_.pose.orientation.w = 1.0;

    geometry_msgs::msg::TransformStamped map_to_base;
    map_to_base.header.stamp = rclcpp::Time(0);
    map_to_base.header.frame_id = global_frame_;
    map_to_base.child_frame_id = base_frame_;
    map_to_base.transform.translation.x = kRobotXM;
    map_to_base.transform.rotation.w = 1.0;
    if (!tf_->setTransform(map_to_base, "pid_final_rotation_test", true)) {
      return false;
    }

    geometry_msgs::msg::PoseStamped locked_goal;
    locked_goal.header.frame_id = global_frame_;
    locked_goal.pose.position.x = kLatchedGoalXM;
    locked_goal.pose.orientation.z = std::sin(kGoalYawRad * 0.5);
    locked_goal.pose.orientation.w = std::cos(kGoalYawRad * 0.5);
    setEndLocalizationGoal(locked_goal, kGoalGeneration);

    geometry_msgs::msg::PoseStamped effective_goal;
    const auto latched = selectEndLocalizationPositionReference(
      robot_pose_, locked_goal, effective_goal);
    if (!latched.position_goal_latched) {
      return false;
    }

    auto live_goal = locked_goal;
    live_goal.pose.position.x = kLiveGoalXM;
    setEndLocalizationGoal(live_goal, kGoalGeneration);

    TrajPoint endpoint;
    endpoint.pose = locked_goal;
    endpoint.x = kLatchedGoalXM;
    endpoint.y = 0.0;
    endpoint.speed_limit = final_rotation_max_v_;
    controller_traj_ = {endpoint};
    trajectory_time_valid_ = false;
    curr_controller_stage_ = ControllerStage::APPROACHING;
    prev_controller_stage_ = ControllerStage::APPROACHING;
    return true;
  }

  geometry_msgs::msg::TwistStamped runFinalRotationCycle()
  {
    geometry_msgs::msg::Twist velocity;
    return computeVelocityCommands(robot_pose_, velocity, nullptr);
  }

  bool isRotatingToGoal() const
  {
    return curr_controller_stage_ == ControllerStage::ROTATING_TO_GOAL;
  }

private:
  geometry_msgs::msg::PoseStamped robot_pose_;
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

TEST(ApproachStage, plannedZeroStopsBeforeOvershootReverseBoostAndUpdatesContinuity)
{
  constexpr double kTrajectoryAccelerationMps2 = -0.4;
  ApproachingZeroSpeedPID pid;
  ASSERT_TRUE(pid.configureZeroSpeedOvershootScenario(kTrajectoryAccelerationMps2));

  const auto command = pid.run();
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);

  double continuity_velocity = 1.0;
  double continuity_acceleration = 0.0;
  ASSERT_TRUE(pid.getContinuityTarget(continuity_velocity, continuity_acceleration));
  EXPECT_DOUBLE_EQ(continuity_velocity, 0.0);
  EXPECT_DOUBLE_EQ(continuity_acceleration, kTrajectoryAccelerationMps2);
}

TEST(ApproachStage, finalRotationEntersOnLatchedPositionThenTracksLivePosition)
{
  FinalRotationTransitionPID pid;
  ASSERT_TRUE(pid.configureLatchedEntryWithLiveOffset(false));

  const auto first_command = pid.runFinalRotationCycle();
  ASSERT_TRUE(pid.isRotatingToGoal());
  const auto first_snapshot = pid.getEndLocalizationSnapshot();
  EXPECT_FALSE(first_snapshot.position_goal_latched);
  EXPECT_EQ(first_snapshot.stage, nav2_core::EndLocalizationStage::MAP_YAW_COARSE);
  EXPECT_NEAR(first_snapshot.effective_goal.pose.position.x, 1.015, 1e-9);
  EXPECT_GT(first_command.twist.linear.x, 0.0);
  EXPECT_LE(first_command.twist.linear.x, 0.05);

  // 实时误差2.5cm位于2cm进入、3cm退出门限之间，下一周期应留在最终转角继续微调。
  const auto second_command = pid.runFinalRotationCycle();
  EXPECT_TRUE(pid.isRotatingToGoal());
  EXPECT_FALSE(pid.getEndLocalizationSnapshot().position_goal_latched);
  EXPECT_GT(second_command.twist.linear.x, 0.0);
  EXPECT_LE(second_command.twist.linear.x, 0.05);
}

TEST(ApproachStage, finalRotationKeepsLatchedPositionWhenConfigured)
{
  FinalRotationTransitionPID pid;
  ASSERT_TRUE(pid.configureLatchedEntryWithLiveOffset(true));

  pid.runFinalRotationCycle();
  ASSERT_TRUE(pid.isRotatingToGoal());
  const auto first_snapshot = pid.getEndLocalizationSnapshot();
  EXPECT_TRUE(first_snapshot.position_goal_latched);
  EXPECT_EQ(first_snapshot.stage, nav2_core::EndLocalizationStage::MAP_YAW_COARSE);
  EXPECT_NEAR(first_snapshot.live_goal.pose.position.x, 1.015, 1e-9);
  EXPECT_NEAR(first_snapshot.effective_goal.pose.position.x, 1.0, 1e-9);

  pid.runFinalRotationCycle();
  EXPECT_TRUE(pid.isRotatingToGoal());
  const auto second_snapshot = pid.getEndLocalizationSnapshot();
  EXPECT_TRUE(second_snapshot.position_goal_latched);
  EXPECT_NEAR(second_snapshot.effective_goal.pose.position.x, 1.0, 1e-9);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
