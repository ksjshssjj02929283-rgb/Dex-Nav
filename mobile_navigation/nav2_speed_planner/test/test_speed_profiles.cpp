// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// T/S 速度规划端到端剖面测试（不依赖 ROS 运行时：costmap 为空 → 障碍约束天然跳过）。
// 场景：直线路径 0.3 / 0.5 / 0.75 / 1.0 m，静止起步。
// 断言：剖面非全零（峰值 > 0）、首点后即有速度、蠕行区保持 end_speed、不超包络。
// 同时把每条剖面导出 CSV 到 /tmp/speed_plan_test/，供 plot_speed_profiles.py 画图。
#include <gtest/gtest.h>

#include <sys/stat.h>

#include <cmath>
#include <cstdio>
#include <deque>
#include <limits>
#include <string>
#include <vector>

#include "nav2_speed_planner/trapezoidal_speed_planner.hpp"
#include "nav2_speed_planner/scurve_speed_planner.hpp"
#include "nav2_core/trajectory_preview.hpp"

namespace
{

constexpr double kMaxSpeed = 1.0;
constexpr double kMinSpeed = 0.03;
constexpr double kAccel = 0.6;
constexpr double kDecel = -0.6;
constexpr double kEndSpeed = 0.03;
constexpr double kStopDistance = 0.2;
constexpr double kPointStep = 0.02;  // 2cm 点距（贴近实机路径密度）

nav_msgs::msg::Path makeStraightPath(double length)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  const int n = static_cast<int>(length / kPointStep) + 1;
  for (int i = 0; i < n; ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position.x = i * kPointStep;
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);
  }
  return path;
}

nav_msgs::msg::Path makeTwoPointPath(double length)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (double x : {0.0, length}) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position.x = x;
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);
  }
  return path;
}

nav_msgs::msg::Path makeStraightPathToExactGoal(double length)
{
  auto path = makeStraightPath(length);
  if (path.poses.back().pose.position.x < length - 1e-9) {
    auto goal = path.poses.back();
    goal.pose.position.x = length;
    path.poses.push_back(goal);
  }
  return path;
}

nav_msgs::msg::Path makePrunedStraightPath(double start_x, double length)
{
  auto path = makeStraightPath(length);
  for (auto & pose : path.poses) {
    pose.pose.position.x += start_x;
  }
  return path;
}

// 测试壳：绕开 ROS configure()，直接设置 protected 参数并调用 computeSpeedPlan。
template<typename PlannerT>
class TestablePlanner : public PlannerT
{
public:
  void setSharedParams()
  {
    this->plugin_name_ = "test";
    this->max_speed_ = kMaxSpeed;
    this->min_speed_ = kMinSpeed;
    this->accel_ = kAccel;
    this->decel_ = kDecel;
    this->end_speed_ = kEndSpeed;
    this->stop_distance_ = kStopDistance;
    this->collision_stop_distance_ = 0.2;
    // costmap_ros_ 为空 → isObstacleAt 恒 false → A1 跳过（离线纯算法测试）
  }
  void setTerminalParams(double end_speed, double stop_distance, double decel = kDecel)
  {
    this->end_speed_ = end_speed;
    this->stop_distance_ = stop_distance;
    this->decel_ = decel;
  }
  void setMotionLimits(double max_speed, double accel)
  {
    this->max_speed_ = max_speed;
    this->accel_ = accel;
  }
  nav_interfaces::msg::RobotTrajectory run(const nav_msgs::msg::Path & path, double v0)
  {
    nav2_core::SpeedPlanningBoundary boundary;
    boundary.velocity.linear.x = v0;
    return this->computeSpeedPlan(path, boundary);
  }
  nav_interfaces::msg::RobotTrajectory run(
    const nav_msgs::msg::Path & path, double v0, double a0)
  {
    nav2_core::SpeedPlanningBoundary boundary;
    boundary.velocity.linear.x = v0;
    boundary.acceleration = a0;
    boundary.acceleration_valid = true;
    return this->computeSpeedPlan(path, boundary);
  }
  nav_interfaces::msg::RobotTrajectory run(
    const nav_msgs::msg::Path & path, double v0, double a0,
    bool terminal_brake_latched)
  {
    nav2_core::SpeedPlanningBoundary boundary;
    boundary.velocity.linear.x = v0;
    boundary.acceleration = a0;
    boundary.acceleration_valid = true;
    boundary.terminal_brake_latched = terminal_brake_latched;
    return this->computeSpeedPlan(path, boundary);
  }
};

class TestableSCurve : public TestablePlanner<nav2_speed_planner::SCurveSpeedPlanner>
{
public:
  void setSCurveParams()
  {
    max_jerk_ = 0.5;
    control_dt_ = 0.05;
  }
  void setTerminalReserve(double execution_delay, double safety_margin)
  {
    terminal_execution_delay_ = execution_delay;
    terminal_safety_margin_ = safety_margin;
  }
};

void dumpCsv(
  const std::string & tag, double length,
  const nav_interfaces::msg::RobotTrajectory & traj)
{
  ::mkdir("/tmp/speed_plan_test", 0755);
  char fn[128];
  std::snprintf(fn, sizeof(fn), "/tmp/speed_plan_test/%s_%.2fm.csv", tag.c_str(), length);
  FILE * f = std::fopen(fn, "w");
  ASSERT_NE(f, nullptr);
  std::fprintf(f, "s,v,accel,rel_time\n");
  double s = 0.0;
  for (size_t i = 0; i < traj.points.size(); ++i) {
    if (i > 0) {
      const auto & a = traj.points[i - 1].pose.pose.position;
      const auto & b = traj.points[i].pose.pose.position;
      s += std::hypot(b.x - a.x, b.y - a.y);
    }
    std::fprintf(
      f, "%.4f,%.5f,%.5f,%.4f\n", s, traj.points[i].longitudinal_velocity,
      traj.points[i].acceleration,
      traj.points[i].relative_time.sec + traj.points[i].relative_time.nanosec * 1e-9);
  }
  std::fclose(f);
}

void checkProfile(
  const std::string & tag, double length,
  const nav_interfaces::msg::RobotTrajectory & traj)
{
  SCOPED_TRACE(tag + " len=" + std::to_string(length));
  ASSERT_GE(traj.points.size(), 2u);

  double peak = 0.0;
  for (const auto & p : traj.points) {
    peak = std::max(peak, p.longitudinal_velocity);
    // 不超共享上限
    EXPECT_LE(p.longitudinal_velocity, kMaxSpeed + 1e-6);
    EXPECT_GE(p.longitudinal_velocity, -1e-9);
  }
  // 核心断言：剖面非全零（全零 = 机器人不走）
  EXPECT_GT(peak, kEndSpeed - 1e-6) << "profile is (near) all-zero!";
  // 静止起步：首点=当前速度(0)，第二个点起必须有速度（否则消费侧无法起步）
  EXPECT_GT(traj.points[1].longitudinal_velocity, 1e-3) << "no speed right after start!";
  // 蠕行区：距终点 stop_distance 内保持 ≈ end_speed（不高于 end_speed + 容差）
  double s_total = 0.0;
  std::vector<double> arc(traj.points.size(), 0.0);
  for (size_t i = 1; i < traj.points.size(); ++i) {
    const auto & a = traj.points[i - 1].pose.pose.position;
    const auto & b = traj.points[i].pose.pose.position;
    s_total += std::hypot(b.x - a.x, b.y - a.y);
    arc[i] = s_total;
  }
  for (size_t i = 1; i < traj.points.size(); ++i) {
    if (s_total - arc[i] < kStopDistance - 1e-9) {
      EXPECT_LE(traj.points[i].longitudinal_velocity, kEndSpeed + 1e-6)
        << "creep zone exceeded at s=" << arc[i];
    }
  }
}

}  // namespace

class SpeedProfileDistances : public ::testing::TestWithParam<double> {};

TEST_P(SpeedProfileDistances, TrapezoidalNotAllZero)
{
  const double length = GetParam();
  TestablePlanner<nav2_speed_planner::TrapezoidalSpeedPlanner> planner;
  planner.setSharedParams();
  auto traj = planner.run(makeStraightPath(length), 0.0);
  dumpCsv("T", length, traj);
  checkProfile("T", length, traj);

  // 后向 pass 后 T 型同样无减速台阶（i=0 为状态戳/接缝，跳过）。
  for (size_t i = 1; i + 1 < traj.points.size(); ++i) {
    const double v0 = traj.points[i].longitudinal_velocity;
    const double v1 = traj.points[i + 1].longitudinal_velocity;
    if (v1 < v0) {
      const auto & a = traj.points[i].pose.pose.position;
      const auto & b = traj.points[i + 1].pose.pose.position;
      const double ds = std::hypot(b.x - a.x, b.y - a.y);
      const double implied_decel = (v0 * v0 - v1 * v1) / (2.0 * std::max(ds, 1e-6));
      EXPECT_LE(implied_decel, kDecel * -1.05)
        << "T decel cliff at index " << i << " (v " << v0 << " -> " << v1 << ")";
    }
  }
}

TEST_P(SpeedProfileDistances, SCurveNotAllZero)
{
  const double length = GetParam();
  TestableSCurve planner;
  planner.setSharedParams();
  planner.setSCurveParams();
  auto traj = planner.run(makeStraightPath(length), 0.0);
  dumpCsv("S", length, traj);
  checkProfile("S", length, traj);

  // S 型专属：无减速台阶——相邻点隐含减速度不得超能力（超了说明减速没在
  // 区界前完成、被包络钳位截断）。T 型静止快照的区界截断是 receding-horizon
  // 预期行为，不做此断言。
  for (size_t i = 0; i + 1 < traj.points.size(); ++i) {
    const double v0 = traj.points[i].longitudinal_velocity;
    const double v1 = traj.points[i + 1].longitudinal_velocity;
    if (v1 < v0) {
      const auto & a = traj.points[i].pose.pose.position;
      const auto & b = traj.points[i + 1].pose.pose.position;
      const double ds = std::hypot(b.x - a.x, b.y - a.y);
      const double implied_decel = (v0 * v0 - v1 * v1) / (2.0 * std::max(ds, 1e-6));
      EXPECT_LE(implied_decel, kDecel * -1.6)
        << "decel cliff at s-index " << i << " (v " << v0 << " -> " << v1 << ")";
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
  Distances, SpeedProfileDistances, ::testing::Values(0.3, 0.5, 0.75, 1.0, 3.0, 5.0));

TEST(SpeedProfileRecedingHorizon, TrapezoidalStartsAtMeasuredSpeedOnPrunedPath)
{
  TestablePlanner<nav2_speed_planner::TrapezoidalSpeedPlanner> planner;
  planner.setSharedParams();
  const auto traj = planner.run(makePrunedStraightPath(2.0, 3.0), 0.23);

  ASSERT_GE(traj.points.size(), 2u);
  EXPECT_NEAR(traj.points.front().pose.pose.position.x, 2.0, 1e-9);
  EXPECT_NEAR(traj.points.front().longitudinal_velocity, 0.23, 1e-9);
}

TEST(SpeedProfileRecedingHorizon, SCurveStartsAtMeasuredSpeedOnPrunedPath)
{
  TestableSCurve planner;
  planner.setSharedParams();
  planner.setSCurveParams();
  const auto traj = planner.run(makePrunedStraightPath(2.0, 3.0), 0.23);

  ASSERT_GE(traj.points.size(), 2u);
  EXPECT_NEAR(traj.points.front().pose.pose.position.x, 2.0, 1e-9);
  EXPECT_NEAR(traj.points.front().longitudinal_velocity, 0.23, 1e-9);
}

TEST(SpeedProfileTerminalPath, StopBoundaryIsInsertedAtExactDistance)
{
  TestablePlanner<nav2_speed_planner::TrapezoidalSpeedPlanner> planner;
  planner.setSharedParams();
  planner.setTerminalParams(0.05, 0.05, -0.4);
  const auto path = makeStraightPath(0.31);
  const auto traj = planner.run(path, 0.0);

  const auto boundary = std::find_if(
    traj.points.begin(), traj.points.end(), [](const auto & point) {
      return std::fabs(point.pose.pose.position.x - 0.25) < 1e-6;
    });
  ASSERT_NE(boundary, traj.points.end());
  EXPECT_EQ(boundary->region_type, "stop_zone");
  EXPECT_NEAR(boundary->longitudinal_velocity, 0.05, 1e-6);
  EXPECT_EQ(traj.points.back().pose, path.poses.back());
}

TEST(SpeedProfileTerminalPath, TrapezoidalMovingStartUsesReachableBraking)
{
  TestablePlanner<nav2_speed_planner::TrapezoidalSpeedPlanner> planner;
  planner.setSharedParams();
  planner.setTerminalParams(0.05, 0.05, -0.4);
  constexpr double kLength = 0.0225;
  constexpr double kStartSpeed = 0.325;
  const auto traj = planner.run(makeTwoPointPath(kLength), kStartSpeed, -0.4);

  ASSERT_EQ(traj.points.size(), 2u);
  EXPECT_NEAR(traj.points.front().longitudinal_velocity, kStartSpeed, 1e-9);
  EXPECT_GT(traj.points.back().longitudinal_velocity, 0.05);
  const double implied_decel =
    (kStartSpeed * kStartSpeed -
    traj.points.back().longitudinal_velocity * traj.points.back().longitudinal_velocity) /
    (2.0 * kLength);
  EXPECT_LE(implied_decel, 0.4 + 1e-6);
}

TEST(SpeedProfileTerminalPath, SCurveMovingStartUsesReachableBraking)
{
  TestableSCurve planner;
  planner.setSharedParams();
  planner.setSCurveParams();
  planner.setTerminalParams(0.05, 0.05, -0.4);
  constexpr double kLength = 0.0225;
  constexpr double kStartSpeed = 0.325;
  const auto traj = planner.run(makeTwoPointPath(kLength), kStartSpeed, -0.4);

  ASSERT_EQ(traj.points.size(), 2u);
  EXPECT_NEAR(traj.points.front().longitudinal_velocity, kStartSpeed, 1e-9);
  EXPECT_NEAR(traj.points.front().acceleration, -0.4, 1e-9);
  EXPECT_GT(traj.points.back().longitudinal_velocity, 0.05);
  EXPECT_GE(traj.points.back().acceleration, -0.4 - 1e-6);
  EXPECT_GT(
    traj.points.back().relative_time.sec +
    traj.points.back().relative_time.nanosec * 1e-9, 0.0);
  const double implied_decel =
    (kStartSpeed * kStartSpeed -
    traj.points.back().longitudinal_velocity * traj.points.back().longitudinal_velocity) /
    (2.0 * kLength);
  EXPECT_LE(implied_decel, 0.4 + 0.01);
  EXPECT_TRUE(planner.getLastPlanningDiagnostics().terminal_infeasible);
}

TEST(SpeedProfileTerminalPath, SCurveW1MovingStartDistanceMatrixIsReachable)
{
  TestableSCurve planner;
  planner.setSharedParams();
  planner.setSCurveParams();
  planner.setTerminalParams(0.05, 0.05, -0.4);

  for (double length : {0.02, 0.05, 0.08, 0.13}) {
    SCOPED_TRACE("remaining=" + std::to_string(length));
    const auto traj = planner.run(makeStraightPathToExactGoal(length), 0.325, -0.4);
    ASSERT_GE(traj.points.size(), 2u);
    EXPECT_NEAR(traj.points.front().longitudinal_velocity, 0.325, 1e-9);
    EXPECT_NEAR(traj.points.front().acceleration, -0.4, 1e-9);
    for (size_t i = 0; i + 1 < traj.points.size(); ++i) {
      const auto & p0 = traj.points[i];
      const auto & p1 = traj.points[i + 1];
      const double ds = p1.pose.pose.position.x - p0.pose.pose.position.x;
      ASSERT_GT(ds, 0.0);
      const double implied_decel =
        (p0.longitudinal_velocity * p0.longitudinal_velocity -
        p1.longitudinal_velocity * p1.longitudinal_velocity) / (2.0 * ds);
      EXPECT_LE(implied_decel, 0.4 + 0.01);
      EXPECT_GE(p1.acceleration, -0.4 - 1e-6);
      EXPECT_LE(p1.acceleration, 0.5 + 1e-6);
    }
  }
}

TEST(SpeedProfileRecedingHorizon, SCurveTenHertzTimePreviewReachesStopZoneSpeed)
{
  TestableSCurve planner;
  planner.setSharedParams();
  planner.setSCurveParams();
  planner.setMotionLimits(0.6, 0.5);
  planner.setTerminalParams(0.05, 0.05, -0.4);
  constexpr double kControllerDt = 0.1;
  constexpr double kGoal = 2.0;
  double position = 0.0;
  double velocity = 0.6;
  double acceleration = 0.0;
  bool entered_stop_zone = false;
  double terminal_min_velocity = std::numeric_limits<double>::infinity();

  for (int cycle = 0; cycle < 100 && position < kGoal - 0.005; ++cycle) {
    const double remaining = kGoal - position;
    const auto traj = planner.run(
      makeStraightPathToExactGoal(remaining), velocity, acceleration);
    ASSERT_GE(traj.points.size(), 2u);
    EXPECT_FALSE(planner.getLastPlanningDiagnostics().terminal_infeasible)
      << "normal long path became infeasible at cycle " << cycle <<
      " remaining=" << remaining << " input_v=" << velocity <<
      " input_a=" << acceleration;

    std::vector<nav2_core::TrajectoryPreviewPoint> preview_points;
    preview_points.reserve(traj.points.size());
    for (const auto & point : traj.points) {
      preview_points.push_back(
      {
        point.pose.pose.position.x, point.pose.pose.position.y,
        point.longitudinal_velocity, point.acceleration,
        nav2_core::durationToSeconds(point.relative_time)});
    }
    ASSERT_TRUE(nav2_core::hasValidRelativeTime(preview_points));
    nav2_core::TrajectoryPreview preview;
    ASSERT_TRUE(
      nav2_core::sampleTrajectoryPreview(
        preview_points, 0.0, 0.0, true, kControllerDt, 0.1, preview));

    const double next_position = position + 0.5 * (velocity + preview.velocity) * kControllerDt;
    if (!entered_stop_zone && next_position >= kGoal - 0.05) {
      entered_stop_zone = true;
      EXPECT_LE(preview.velocity, 0.06);
    }
    if (next_position >= kGoal - 0.15) {
      terminal_min_velocity = std::min(terminal_min_velocity, preview.velocity);
      EXPECT_GE(preview.velocity, 0.049)
        << "terminal velocity undershot stop_speed at cycle " << cycle <<
        " input_v=" << velocity << " input_a=" << acceleration <<
        " preview_a=" << preview.acceleration;
      if (preview.velocity <= 0.051) {
        EXPECT_LE(preview.acceleration, 1e-3)
          << "terminal profile must not rebound with positive acceleration";
      }
    }
    position = next_position;
    velocity = preview.velocity;
    acceleration = preview.acceleration;
  }
  EXPECT_TRUE(entered_stop_zone);
  EXPECT_GE(terminal_min_velocity, 0.049);
}

TEST(SpeedProfileTerminalReserve, ReachesStableCreepBeforeReservedBoundary)
{
  TestableSCurve planner;
  planner.setSharedParams();
  planner.setSCurveParams();
  planner.setMotionLimits(0.6, 0.5);
  planner.setTerminalParams(0.05, 0.075, -0.4);
  planner.setTerminalReserve(0.35, 0.05);

  constexpr double kGoal = 2.0;
  constexpr double kStartSpeed = 0.6;
  const auto traj = planner.run(
    makeStraightPathToExactGoal(kGoal), kStartSpeed, 0.0, false);
  const double reserve = nav2_speed_planner::SCurveSpeedPlanner::
    computeTerminalExecutionReserve(kStartSpeed, 0.0, 0.35, 0.05);
  const double reserved_boundary = kGoal - 0.075 - reserve;

  bool stable_creep_found = false;
  for (const auto & point : traj.points) {
    if (point.longitudinal_velocity <= 0.051 && std::fabs(point.acceleration) <= 1e-3) {
      stable_creep_found = true;
      EXPECT_LE(point.pose.pose.position.x, reserved_boundary + kPointStep + 1e-3)
        << "stable creep boundary may shift by at most one path sample";
      break;
    }
  }
  EXPECT_TRUE(stable_creep_found);
  const auto diagnostics = planner.getLastPlanningDiagnostics();
  EXPECT_NEAR(diagnostics.terminal_execution_reserve, reserve, 1e-9);
  EXPECT_FALSE(diagnostics.terminal_infeasible);
}

TEST(SpeedProfileTerminalReserve, ScalesWithVelocityAndPositiveAcceleration)
{
  using nav2_speed_planner::SCurveSpeedPlanner;
  EXPECT_NEAR(
    SCurveSpeedPlanner::computeTerminalExecutionReserve(0.6, 0.2, 0.35, 0.05),
    0.27225, 1e-9);
  EXPECT_NEAR(
    SCurveSpeedPlanner::computeTerminalExecutionReserve(0.6, -0.2, 0.35, 0.05),
    0.26, 1e-9);
  EXPECT_DOUBLE_EQ(
    SCurveSpeedPlanner::computeTerminalExecutionReserve(-0.1, 0.0, -1.0, -0.2),
    0.0);
}

TEST(SpeedProfileTerminalReserve, ColdStartLongPathDoesNotCollapseToEndSpeed)
{
  TestableSCurve planner;
  planner.setSharedParams();
  planner.setSCurveParams();
  planner.setMotionLimits(0.6, 0.5);
  planner.setTerminalParams(0.05, 0.075, -0.4);
  // Reproduce the field runtime where the new reserve parameters were not
  // loaded. A missing reserve must not turn a normal 3.24 m startup into a
  // terminal-infeasible all-creep profile.
  planner.setTerminalReserve(0.0, 0.0);

  const auto trajectory = planner.run(
    makeStraightPathToExactGoal(3.24), 0.0, 0.0, false);
  ASSERT_GE(trajectory.points.size(), 2u);
  const auto diagnostics = planner.getLastPlanningDiagnostics();
  EXPECT_FALSE(diagnostics.terminal_brake_required);
  EXPECT_FALSE(diagnostics.terminal_infeasible);

  double peak_velocity = 0.0;
  for (const auto & point : trajectory.points) {
    peak_velocity = std::max(peak_velocity, point.longitudinal_velocity);
  }
  EXPECT_GT(peak_velocity, 0.3)
    << "cold-start long path must contain a normal cruise/acceleration segment";
}

TEST(SpeedProfileRecedingHorizon, DynamicReserveHandlesDelayedLaggedPlant)
{
  TestableSCurve planner;
  planner.setSharedParams();
  planner.setSCurveParams();
  planner.setMotionLimits(0.6, 0.5);
  planner.setTerminalParams(0.05, 0.075, -0.4);
  planner.setTerminalReserve(0.35, 0.05);

  constexpr double kControllerDt = 0.1;
  constexpr double kGoal = 3.0;
  constexpr int kCommandDelaySteps = 2;
  // 200 ms transport delay + 150 ms first-order acceleration response models
  // the observed roughly 300-334 ms execution latency while preserving the
  // configured 0.4 m/s^2 steady-state braking authority.
  constexpr double kPlantTimeConstant = 0.15;
  double position = 0.0;
  double velocity = 0.6;
  double consumed_target_acceleration = 0.0;
  bool terminal_brake_latched = false;
  bool terminal_brake_seen = false;
  bool reached_creep_before_stop_zone = false;
  double actual_acceleration = 0.0;
  std::deque<double> delayed_accelerations(kCommandDelaySteps, 0.0);

  for (int cycle = 0; cycle < 120 && position < kGoal - 0.002; ++cycle) {
    const double remaining = kGoal - position;
    const auto traj = planner.run(
      makeStraightPathToExactGoal(remaining), velocity,
      consumed_target_acceleration, terminal_brake_latched);
    ASSERT_GE(traj.points.size(), 2u);
    const auto diagnostics = planner.getLastPlanningDiagnostics();
    if (diagnostics.terminal_brake_required) {
      terminal_brake_latched = true;
      terminal_brake_seen = true;
    }

    std::vector<nav2_core::TrajectoryPreviewPoint> preview_points;
    preview_points.reserve(traj.points.size());
    for (const auto & point : traj.points) {
      preview_points.push_back(
      {
        point.pose.pose.position.x, point.pose.pose.position.y,
        point.longitudinal_velocity, point.acceleration,
        nav2_core::durationToSeconds(point.relative_time)});
    }
    nav2_core::TrajectoryPreview preview;
    ASSERT_TRUE(
      nav2_core::sampleTrajectoryPreview(
        preview_points, 0.0, 0.0,
        nav2_core::hasValidRelativeTime(preview_points),
        kControllerDt, 0.1, preview));
    if (diagnostics.terminal_infeasible) {
      EXPECT_TRUE(terminal_brake_latched);
      EXPECT_LE(preview.acceleration, 1e-6)
        << "latched infeasible fallback must continue braking at cycle " << cycle;
    }
    consumed_target_acceleration = preview.acceleration;
    delayed_accelerations.push_back(preview.acceleration);
    const double delayed_acceleration = delayed_accelerations.front();
    delayed_accelerations.pop_front();
    actual_acceleration += std::clamp(
      (delayed_acceleration - actual_acceleration) /
      kPlantTimeConstant * kControllerDt, -0.4, 0.5);
    actual_acceleration = std::clamp(actual_acceleration, -0.4, 0.5);
    const double next_velocity = std::max(
      0.0, velocity + actual_acceleration * kControllerDt);
    const double next_position = position +
      0.5 * (velocity + next_velocity) * kControllerDt;
    if (next_velocity <= 0.06) {
      reached_creep_before_stop_zone = true;
      EXPECT_LE(next_position, kGoal - 0.075)
        << "delayed plant must reach creep before entering the stop zone";
      break;
    }
    if (next_position >= kGoal - 0.075) {
      ADD_FAILURE() << "entered stop zone at " << next_velocity << " m/s";
      break;
    }
    position = next_position;
    velocity = next_velocity;
  }

  EXPECT_TRUE(terminal_brake_seen);
  EXPECT_TRUE(reached_creep_before_stop_zone);
}

TEST(SpeedProfileTerminalReserve, LatchedBrakeDoesNotRearmCruiseOnLongerPath)
{
  TestableSCurve planner;
  planner.setSharedParams();
  planner.setSCurveParams();
  planner.setMotionLimits(0.6, 0.5);
  planner.setTerminalParams(0.05, 0.075, -0.4);
  planner.setTerminalReserve(0.35, 0.05);

  constexpr double kMeasuredVelocity = 0.4;
  const auto traj = planner.run(
    makeStraightPathToExactGoal(2.0), kMeasuredVelocity, -0.2, true);
  ASSERT_GE(traj.points.size(), 2u);
  for (const auto & point : traj.points) {
    EXPECT_LE(point.longitudinal_velocity, kMeasuredVelocity + 1e-6);
    EXPECT_LE(point.acceleration, 1e-6);
  }
  EXPECT_TRUE(planner.getLastPlanningDiagnostics().terminal_brake_required);
}

TEST(SpeedProfileTerminalPath, IdenticalTwoPointGoalPathIsPreservedAndStopped)
{
  TestablePlanner<nav2_speed_planner::TrapezoidalSpeedPlanner> planner;
  planner.setSharedParams();
  auto path = makeStraightPath(0.0);
  path.poses.push_back(path.poses.back());

  const auto traj = planner.run(path, 0.4);

  ASSERT_EQ(traj.points.size(), 2u);
  EXPECT_DOUBLE_EQ(traj.points.front().longitudinal_velocity, 0.0);
  EXPECT_DOUBLE_EQ(traj.points.back().longitudinal_velocity, 0.0);
}

TEST(SpeedProfileTerminalPath, PureRotationSequenceReachesControllerUnchanged)
{
  TestablePlanner<nav2_speed_planner::TrapezoidalSpeedPlanner> planner;
  planner.setSharedParams();
  auto path = makeStraightPath(0.0);
  path.poses.push_back(path.poses.back());
  path.poses.push_back(path.poses.back());
  path.poses[1].pose.orientation.z = std::sin(0.25);
  path.poses[1].pose.orientation.w = std::cos(0.25);
  path.poses[2].pose.orientation.z = std::sin(0.5);
  path.poses[2].pose.orientation.w = std::cos(0.5);

  const auto traj = planner.run(path, 0.4);

  ASSERT_EQ(traj.points.size(), path.poses.size());
  for (size_t i = 0; i < traj.points.size(); ++i) {
    EXPECT_EQ(traj.points[i].pose, path.poses[i]);
    EXPECT_DOUBLE_EQ(traj.points[i].longitudinal_velocity, 0.0);
  }
}

// ============ 障碍注入场景（覆盖实机"整条剖面全零"根因）============

// 起点在障碍内切区（机器人贴墙停放）：x <= 0.10 视为障碍。
class TWithStartObstacle
  : public TestablePlanner<nav2_speed_planner::TrapezoidalSpeedPlanner>
{
public:
  bool isObstacleAt(double x, double /*y*/) const override {return x <= 0.10;}
};

TEST(SpeedProfileObstacle, StartInObstacleIsExempted)
{
  TWithStartObstacle planner;
  planner.setSharedParams();
  auto traj = planner.run(makeStraightPath(1.0), 0.0);
  dumpCsv("T_start_obstacle", 1.0, traj);
  // 起点豁免（obstacle_start_exempt_distance=0.25 > 0.10）：剖面不得全零。
  double peak = 0.0;
  for (const auto & p : traj.points) {
    peak = std::max(peak, p.longitudinal_velocity);
  }
  EXPECT_GT(peak, 0.1) << "start-zone obstacle must NOT zero the whole profile";
  EXPECT_GT(traj.points[1].longitudinal_velocity, 1e-3);
}

// 中途障碍：x >= 0.70 视为障碍 → 0.70 之后置零 + 前方 0.2m 缓冲带置零 + 减速过渡。
class TWithMidObstacle
  : public TestablePlanner<nav2_speed_planner::TrapezoidalSpeedPlanner>
{
public:
  bool isObstacleAt(double x, double /*y*/) const override {return x >= 0.70;}
};

// 5m 路径、3m 处障碍：长路径 + 巡航段 + 中途障碍（T/S 各测一次）。
class TWithObstacleAt3m
  : public TestablePlanner<nav2_speed_planner::TrapezoidalSpeedPlanner>
{
public:
  bool isObstacleAt(double x, double /*y*/) const override {return x >= 3.0;}
};

class SWithObstacleAt3m : public TestableSCurve
{
public:
  bool isObstacleAt(double x, double /*y*/) const override {return x >= 3.0;}
};

template<typename PlannerT>
void checkObstacleAt3m(PlannerT & planner, const std::string & tag)
{
  auto traj = planner.run(makeStraightPath(5.0), 0.0);
  dumpCsv(tag, 5.0, traj);
  double peak = 0.0;
  for (const auto & p : traj.points) {
    const double x = p.pose.pose.position.x;
    peak = std::max(peak, p.longitudinal_velocity);
    if (x >= 2.80 - 1e-9) {  // 障碍(3.0) + 缓冲带(0.2)
      EXPECT_LT(p.longitudinal_velocity, 1e-6) << tag << " zero zone violated at x=" << x;
    }
  }
  // 长路径应到达巡航（accel 0.6 → 0.83m 加速距离，3m 内充分）
  EXPECT_GT(peak, 0.9) << tag << " should reach cruise before the obstacle";
  EXPECT_GT(traj.points[1].longitudinal_velocity, 1e-3);
}

TEST(SpeedProfileObstacle, ObstacleAt3mOn5mPathTrapezoidal)
{
  TWithObstacleAt3m planner;
  planner.setSharedParams();
  checkObstacleAt3m(planner, "T_obs3");
}

TEST(SpeedProfileObstacle, ObstacleAt3mOn5mPathSCurve)
{
  SWithObstacleAt3m planner;
  planner.setSharedParams();
  planner.setSCurveParams();
  checkObstacleAt3m(planner, "S_obs3");
}

// ============ 曲率场景：5m 路径、2m 处起 0.5m 弧段（半径 0.5m）============
//
// A3 曲率限速：radius < regulated_linear_scaling_min_radius(默认 1.2) 时
//   v_cap = max_speed · (1 - (min_rad - radius)/min_rad) = max_speed · radius/min_rad。
// 本场景 r=0.5 → v_cap = 1.0 · 0.5/1.2 ≈ 0.417 m/s。
// 验证点：弯内速度贴合 v_cap；进弯前有完整减速坡道（无台阶）；弯后恢复。

constexpr double kCurveStart = 2.0;
constexpr double kCurveLen = 0.5;
constexpr double kCurveRadius = 0.5;
constexpr double kMinRad = 1.2;  // regulated_linear_scaling_min_radius 默认值
constexpr double kCurveSpeed = kMaxSpeed * kCurveRadius / kMinRad;

nav_msgs::msg::Path makePathWithArcAt2m(double length)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  double x = 0.0, y = 0.0, yaw = 0.0;
  const int n = static_cast<int>(length / kPointStep) + 1;
  for (int i = 0; i < n; ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.orientation.z = std::sin(yaw / 2.0);
    ps.pose.orientation.w = std::cos(yaw / 2.0);
    path.poses.push_back(ps);
    const double s_next = (i + 1) * kPointStep;
    if (s_next > kCurveStart && s_next <= kCurveStart + kCurveLen) {
      yaw += kPointStep / kCurveRadius;  // 弧段内恒曲率转向
    }
    x += kPointStep * std::cos(yaw);
    y += kPointStep * std::sin(yaw);
  }
  return path;
}

void checkCurveProfile(
  const std::string & tag, const nav_interfaces::msg::RobotTrajectory & traj,
  double cliff_factor)
{
  SCOPED_TRACE(tag);
  ASSERT_GE(traj.points.size(), 2u);
  std::vector<double> arc(traj.points.size(), 0.0);
  for (size_t i = 1; i < traj.points.size(); ++i) {
    const auto & a = traj.points[i - 1].pose.pose.position;
    const auto & b = traj.points[i].pose.pose.position;
    arc[i] = arc[i - 1] + std::hypot(b.x - a.x, b.y - a.y);
  }

  double peak_before = 0.0, peak_after = 0.0;
  for (size_t i = 0; i < traj.points.size(); ++i) {
    const double v = traj.points[i].longitudinal_velocity;
    // 弯内速度贴合曲率限速（±0.15m 余量避开 Menger 三点窗的边缘混合段）。
    if (arc[i] >= kCurveStart + 0.15 && arc[i] <= kCurveStart + kCurveLen - 0.15) {
      EXPECT_NEAR(v, kCurveSpeed, 0.03) << "curve speed off at s=" << arc[i];
    }
    if (arc[i] < kCurveStart - 0.3) {
      peak_before = std::max(peak_before, v);
    }
    if (arc[i] > kCurveStart + kCurveLen + 0.3 && arc[i] < 4.0) {
      peak_after = std::max(peak_after, v);
    }
  }
  // 弯前应有明显加速（T 达巡航；S 受 jerk 限制加减速衔接，放宽阈值）。
  EXPECT_GT(peak_before, 0.6) << "should speed up before the curve";
  // 弯后不被弯道残留压制。
  EXPECT_GT(peak_after, kCurveSpeed + 0.1) << "should speed up again after the curve";

  // 进弯坡道：全程无减速台阶（i=0 为当前速度状态戳，跳过）。
  for (size_t i = 1; i + 1 < traj.points.size(); ++i) {
    const double v0 = traj.points[i].longitudinal_velocity;
    const double v1 = traj.points[i + 1].longitudinal_velocity;
    if (v1 < v0) {
      const double ds = std::max(arc[i + 1] - arc[i], 1e-6);
      const double implied_decel = (v0 * v0 - v1 * v1) / (2.0 * ds);
      EXPECT_LE(implied_decel, kDecel * -cliff_factor)
        << "decel cliff at s=" << arc[i] << " (v " << v0 << " -> " << v1 << ")";
    }
  }
}

TEST(SpeedProfileCurve, ArcAt2mOn5mPathTrapezoidal)
{
  TestablePlanner<nav2_speed_planner::TrapezoidalSpeedPlanner> planner;
  planner.setSharedParams();
  auto traj = planner.run(makePathWithArcAt2m(5.0), 0.0);
  dumpCsv("T_curve2", 5.0, traj);
  checkProfile("T_curve2", 5.0, traj);
  checkCurveProfile("T_curve2", traj, 1.05);

  // 前向 pass 后：出弯必须有再加速坡道——加速方向同样无台阶
  // （隐含加速度 ≤ 1.05×accel，含 i=0 起步，前向传播保证可行）。
  std::vector<double> arc(traj.points.size(), 0.0);
  for (size_t i = 1; i < traj.points.size(); ++i) {
    const auto & a = traj.points[i - 1].pose.pose.position;
    const auto & b = traj.points[i].pose.pose.position;
    arc[i] = arc[i - 1] + std::hypot(b.x - a.x, b.y - a.y);
  }
  for (size_t i = 0; i + 1 < traj.points.size(); ++i) {
    const double v0 = traj.points[i].longitudinal_velocity;
    const double v1 = traj.points[i + 1].longitudinal_velocity;
    if (v1 > v0) {
      const double ds = std::max(arc[i + 1] - arc[i], 1e-6);
      const double implied_accel = (v1 * v1 - v0 * v0) / (2.0 * ds);
      EXPECT_LE(implied_accel, kAccel * 1.05)
        << "T accel cliff at s=" << arc[i] << " (v " << v0 << " -> " << v1 << ")";
    }
  }
}

TEST(SpeedProfileCurve, ArcAt2mOn5mPathSCurve)
{
  TestableSCurve planner;
  planner.setSharedParams();
  planner.setSCurveParams();
  auto traj = planner.run(makePathWithArcAt2m(5.0), 0.0);
  dumpCsv("S_curve2", 5.0, traj);
  checkProfile("S_curve2", 5.0, traj);
  checkCurveProfile("S_curve2", traj, 1.6);
}

TEST(SpeedProfileObstacle, MidPathObstacleStopsWithBuffer)
{
  TWithMidObstacle planner;
  planner.setSharedParams();
  auto traj = planner.run(makeStraightPath(1.0), 0.0);
  dumpCsv("T_mid_obstacle", 1.0, traj);
  double peak = 0.0;
  for (const auto & p : traj.points) {
    const double x = p.pose.pose.position.x;
    peak = std::max(peak, p.longitudinal_velocity);
    if (x >= 0.70 - 1e-9) {
      EXPECT_LT(p.longitudinal_velocity, 1e-6) << "obstacle zone must be zero at x=" << x;
    }
    if (x >= 0.52 && x < 0.70) {  // 缓冲带（0.70-0.2，留点距余量）
      EXPECT_LT(p.longitudinal_velocity, 1e-6) << "buffer zone must be zero at x=" << x;
    }
  }
  EXPECT_GT(peak, 0.1) << "pre-obstacle segment should still move";
}
