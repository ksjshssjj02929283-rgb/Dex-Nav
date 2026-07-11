// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 离线速度剖面生成器（不依赖 ROS 运行时：costmap 为空/子类注入障碍）。
// 覆盖场景：直线距离族（静止起步）、运动中起步、弯道（2m 处 0.5m 弧段）、
// 起点障碍豁免、中途障碍、5m@3m 障碍、硬停（end_speed=0）。
// 每场景 T/S 双 planner 各导一份 CSV（s,v,accel,jerk,rel_time,region），
// 供 plot_speed_profiles.py / gen_speed_plan_charts.py 出图。
//
// 用法：ros2 run nav2_speed_planner speed_profile_gen [输出目录]
//   缺省输出 $HOME/.dexe/navigation/speed_planner/csv/

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <sys/stat.h>
#include <vector>

#include "nav2_speed_planner/trapezoidal_speed_planner.hpp"
#include "nav2_speed_planner/scurve_speed_planner.hpp"

namespace
{

constexpr double kMaxSpeed = 1.0;
constexpr double kMinSpeed = 0.03;
constexpr double kAccel = 0.6;
constexpr double kDecel = -0.6;
constexpr double kEndSpeed = 0.03;
constexpr double kStopDistance = 0.2;
constexpr double kPointStep = 0.02;

constexpr double kCurveStart = 2.0;
constexpr double kCurveLen = 0.5;
constexpr double kCurveRadius = 0.5;

std::string g_out_dir;

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
      yaw += kPointStep / kCurveRadius;
    }
    x += kPointStep * std::cos(yaw);
    y += kPointStep * std::sin(yaw);
  }
  return path;
}

template<typename PlannerT>
class TestablePlanner : public PlannerT
{
public:
  void setSharedParams()
  {
    this->plugin_name_ = "offline";
    this->max_speed_ = kMaxSpeed;
    this->min_speed_ = kMinSpeed;
    this->accel_ = kAccel;
    this->decel_ = kDecel;
    this->end_speed_ = kEndSpeed;
    this->stop_distance_ = kStopDistance;
    this->collision_stop_distance_ = 0.2;
  }
  void setEndSpeed(double v) {this->end_speed_ = v;}
  nav_interfaces::msg::RobotTrajectory run(const nav_msgs::msg::Path & path, double v0)
  {
    geometry_msgs::msg::Twist tw;
    tw.linear.x = v0;
    return this->computeSpeedPlan(path, tw);
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
};

template<typename BaseT>
class WithObstacleBand : public BaseT
{
public:
  double obs_from_{1e9};
  double obs_to_{1e9};
  bool isObstacleAt(double x, double /*y*/) const override
  {
    return x >= obs_from_ && x <= obs_to_;
  }
};

void dumpCsv(
  const std::string & tag, double length,
  const nav_interfaces::msg::RobotTrajectory & traj)
{
  char fn[512];
  std::snprintf(
    fn, sizeof(fn), "%s/%s_%.2fm.csv", g_out_dir.c_str(), tag.c_str(), length);
  FILE * f = std::fopen(fn, "w");
  if (f == nullptr) {
    std::fprintf(stderr, "cannot open %s\n", fn);
    std::exit(1);
  }
  std::fprintf(f, "s,v,accel,jerk,rel_time,region\n");
  double s = 0.0;
  for (size_t i = 0; i < traj.points.size(); ++i) {
    if (i > 0) {
      const auto & a = traj.points[i - 1].pose.pose.position;
      const auto & b = traj.points[i].pose.pose.position;
      s += std::hypot(b.x - a.x, b.y - a.y);
    }
    const auto & p = traj.points[i];
    std::fprintf(
      f, "%.4f,%.5f,%.5f,%.5f,%.4f,%s\n", s, p.longitudinal_velocity,
      p.acceleration, p.jerk,
      p.relative_time.sec + p.relative_time.nanosec * 1e-9,
      p.region_type.c_str());
  }
  std::fclose(f);
  std::printf("  %s\n", fn);
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc > 1) {
    g_out_dir = argv[1];
  } else {
    const char * home = std::getenv("HOME");
    g_out_dir = std::string(home ? home : ".") + "/.dexe/navigation/speed_planner/csv";
  }
  // 逐级建目录
  std::string acc;
  for (size_t i = 0; i < g_out_dir.size(); ++i) {
    acc += g_out_dir[i];
    if (g_out_dir[i] == '/' || i + 1 == g_out_dir.size()) {
      ::mkdir(acc.c_str(), 0755);
    }
  }
  std::printf("速度剖面 CSV 输出目录：%s\n", g_out_dir.c_str());

  // ── 直线距离族（静止起步，T/S）──
  for (double len : {0.3, 0.5, 1.0, 2.5, 5.0}) {
    TestablePlanner<nav2_speed_planner::TrapezoidalSpeedPlanner> t;
    t.setSharedParams();
    dumpCsv("T", len, t.run(makeStraightPath(len), 0.0));

    TestableSCurve s;
    s.setSharedParams();
    s.setSCurveParams();
    dumpCsv("S", len, s.run(makeStraightPath(len), 0.0));
  }

  // ── 运动中起步（receding-horizon 状态戳形态，2.5m）──
  for (double v0 : {0.3, 0.6, 1.0}) {
    TestablePlanner<nav2_speed_planner::TrapezoidalSpeedPlanner> t;
    t.setSharedParams();
    char tag[32];
    std::snprintf(tag, sizeof(tag), "T_moving_v%.1f", v0);
    dumpCsv(tag, 2.5, t.run(makeStraightPath(2.5), v0));

    TestableSCurve s;
    s.setSharedParams();
    s.setSCurveParams();
    std::snprintf(tag, sizeof(tag), "S_moving_v%.1f", v0);
    dumpCsv(tag, 2.5, s.run(makeStraightPath(2.5), v0));
  }

  // ── 弯道（5m 路径 2m 处 0.5m 弧段 r=0.5，T/S）──
  {
    TestablePlanner<nav2_speed_planner::TrapezoidalSpeedPlanner> t;
    t.setSharedParams();
    dumpCsv("T_curve2", 5.0, t.run(makePathWithArcAt2m(5.0), 0.0));

    TestableSCurve s;
    s.setSharedParams();
    s.setSCurveParams();
    dumpCsv("S_curve2", 5.0, s.run(makePathWithArcAt2m(5.0), 0.0));
  }

  // ── 障碍场景 ──
  {
    // 起点在障碍内切区（豁免起步）
    WithObstacleBand<TestablePlanner<nav2_speed_planner::TrapezoidalSpeedPlanner>> t;
    t.setSharedParams();
    t.obs_from_ = 0.0;
    t.obs_to_ = 0.10;
    dumpCsv("T_start_obstacle", 1.0, t.run(makeStraightPath(1.0), 0.0));
  }
  {
    // 1m 路径中途障碍 x≥0.7
    WithObstacleBand<TestablePlanner<nav2_speed_planner::TrapezoidalSpeedPlanner>> t;
    t.setSharedParams();
    t.obs_from_ = 0.7;
    t.obs_to_ = 1.0;
    dumpCsv("T_mid_obstacle", 1.0, t.run(makeStraightPath(1.0), 0.0));
  }
  for (int variant = 0; variant < 2; ++variant) {
    // 5m 路径 3m 处障碍（T/S）
    if (variant == 0) {
      WithObstacleBand<TestablePlanner<nav2_speed_planner::TrapezoidalSpeedPlanner>> t;
      t.setSharedParams();
      t.obs_from_ = 3.0;
      t.obs_to_ = 5.0;
      dumpCsv("T_obs3", 5.0, t.run(makeStraightPath(5.0), 0.0));
    } else {
      WithObstacleBand<TestableSCurve> s;
      s.setSharedParams();
      s.setSCurveParams();
      s.obs_from_ = 3.0;
      s.obs_to_ = 5.0;
      dumpCsv("S_obs3", 5.0, s.run(makeStraightPath(5.0), 0.0));
    }
  }

  // ── 硬停（end_speed=0，1m）──
  {
    TestablePlanner<nav2_speed_planner::TrapezoidalSpeedPlanner> t;
    t.setSharedParams();
    t.setEndSpeed(0.0);
    dumpCsv("T_hardstop", 1.0, t.run(makeStraightPath(1.0), 0.0));

    TestableSCurve s;
    s.setSharedParams();
    s.setSCurveParams();
    s.setEndSpeed(0.0);
    dumpCsv("S_hardstop", 1.0, s.run(makeStraightPath(1.0), 0.0));
  }

  std::printf("完成。\n");
  return 0;
}
