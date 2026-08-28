// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 离线速度剖面验证——**执行外壳**（路径构造 + 规划器注入 + 跑一个场景）。
// 不含场景数据（见 speed_scenarios.hpp），不含校验逻辑（见 speed_checks.hpp）。

#ifndef TOOLS__SPEED_HARNESS_HPP_
#define TOOLS__SPEED_HARNESS_HPP_

#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>

#include "nav2_speed_planner/backends/trapezoidal_speed_planner.hpp"
#include "nav2_speed_planner/backends/scurve_speed_planner.hpp"
#include "nav2_speed_planner/backends/reachability_speed_planner.hpp"
#include "speed_scenarios.hpp"

namespace speed_offline
{

inline nav_msgs::msg::Path makePath(
  double length, const std::vector<Arc> & arcs, PathKind kind,
  double lateral_offset = 0.0, double dense_spacing = 0.0)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  auto pose_at = [](double x, double y, double yaw) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "map";
      ps.pose.position.x = x;
      ps.pose.position.y = y;
      ps.pose.orientation.z = std::sin(yaw / 2.0);
      ps.pose.orientation.w = std::cos(yaw / 2.0);
      return ps;
    };

  switch (kind) {
    case PathKind::Empty:
      return path;
    case PathKind::SinglePoint:
      path.poses.push_back(pose_at(0.0, 0.0, 0.0));
      return path;
    case PathKind::TwoPoints:
      path.poses.push_back(pose_at(0.0, 0.0, 0.0));
      path.poses.push_back(pose_at(kPointStep, 0.0, 0.0));
      return path;
    case PathKind::AllCoincident:
      for (int i = 0; i < 10; ++i) {path.poses.push_back(pose_at(0.0, 0.0, 0.0));}
      return path;
    case PathKind::Teleport:
      for (int i = 0; i < 50; ++i) {
        path.poses.push_back(pose_at(i * kPointStep + (i > 25 ? 10.0 : 0.0), 0.0, 0.0));
      }
      return path;
    default:
      break;
  }

  double x = 0.0, y = 0.0, yaw = 0.0;
  const int n = static_cast<int>(length / kPointStep) + 1;
  const double dir = (kind == PathKind::Backward) ? -1.0 : 1.0;
  for (int i = 0; i < n; ++i) {
    // 直线用 i*kPointStep（原 makeStraightPath 的写法），弯道用累加
    // （原 makePathWithArcAt2m 的写法）。两式尾位不同——累加式还会被编译器
    // 按 -ffp-contract=fast 收成 FMA——直线场景下 x 会差 1 ULP，正好在障碍带
    // 边界 `x >= from` 上翻转，使减速斜坡整体错位一个采样点。合并生成器时
    // 必须保留这个区分，否则历史 CSV 对不上（实测 T_obs3 偏移 0.02m）。
    const bool straight = arcs.empty();
    auto ps = pose_at(
      dir * (straight ? i * kPointStep : x), straight ? 0.0 : y, yaw);
    if (i == n / 2) {
      if (kind == PathKind::NaNPose) {
        ps.pose.position.x = std::nan("");
      } else if (kind == PathKind::InfPose) {
        ps.pose.position.x = std::numeric_limits<double>::infinity();
      }
    }
    if (kind == PathKind::ZeroQuaternion) {
      ps.pose.orientation.x = ps.pose.orientation.y = 0.0;
      ps.pose.orientation.z = ps.pose.orientation.w = 0.0;
    }
    // 首点 = 机器人状态戳；偏离路网时它与第二点之间形成横向折角
    if (kind == PathKind::OffRouteStart && i == 0) {
      ps.pose.position.y += lateral_offset;
    }
    path.poses.push_back(ps);
    // 弯道中段插入密集点簇：在两个正常点之间补若干 dense_spacing 间距的点，
    // 复现实机全局路径的间距不均（2.56~70.5 mm）。
    if (kind == PathKind::NonUniform && dense_spacing > 1e-9) {
      const double s_here = i * kPointStep;
      if (s_here > kCurveStart + 0.1 && s_here < kCurveStart + 0.3) {
        const int extra = static_cast<int>(kPointStep / dense_spacing) - 1;
        for (int e = 1; e <= extra && e < 40; ++e) {
          auto mid = ps;
          const double f = static_cast<double>(e) / static_cast<double>(extra + 1);
          mid.pose.position.x += f * kPointStep * std::cos(yaw);
          mid.pose.position.y += f * kPointStep * std::sin(yaw);
          path.poses.push_back(mid);
        }
      }
    }
    if (kind == PathKind::WithDuplicates && i % 10 == 0) {
      path.poses.push_back(path.poses.back());   // 插入重复点
    }

    // ⚠ 弯道触发沿用原式 (i+1)*kPointStep——换成累加会改变弯道起止格点，
    //   进而改变整条弯道几何（实测 T_curve2 243 行变化）。
    const double s_next = (i + 1) * kPointStep;
    for (const auto & a : arcs) {
      if (s_next > a.start && s_next <= a.start + a.len) {
        yaw += kPointStep / a.radius;
        break;
      }
    }
    x += kPointStep * std::cos(yaw);
    y += kPointStep * std::sin(yaw);
  }
  return path;
}

// 障碍带以**路径 x 坐标**定义（与 checkInvariants 的判定坐标必须一致）。
template<typename PlannerT>
class Harness : public PlannerT
{
public:
  std::vector<Band> bands_;
  std::vector<Zone> zones_;
  bool available_{true};

  void setSharedParams(const Params & q = Params{})
  {
    this->plugin_name_ = "offline";
    this->max_speed_ = q.max_speed;
    this->min_speed_ = q.min_speed;
    this->accel_ = q.accel;
    this->decel_ = q.decel;
    this->end_speed_ = q.end_speed;
    this->terminal_speed_cap_ = q.end_speed;
    this->terminal_creep_distance_ = q.stop_distance;
    this->obstacle_safety_margin_ = q.obstacle_safety_margin;
    this->obstacle_stop_margin_ = q.obstacle_stop_margin;
    if (q.min_curvature_radius > 0.0) {
      this->regulated_linear_scaling_min_radius_ = q.min_curvature_radius;
      this->min_curvature_speed_ = q.min_curvature_speed;
      this->max_lateral_accel_ = q.max_lateral_accel;
    }
  }
  void setEndSpeed(double v)
  {
    this->end_speed_ = v;
    this->terminal_speed_cap_ = v;
  }
  void setPolicy(nav2_speed_planner::ConstraintFailurePolicy p)
  {
    this->obstacle_unavailable_policy_ = p;
  }

  // 注入式预言机**确实可用**——无 costmap 不代表无法评估障碍。若不声明，
  // 步 1 的可用性门 + 默认 FAIL_CLOSED 会把每个场景的轨迹都清空
  // （曾真实发生：24 个 CSV 全剩表头，程序仍打印成功）。
  // available_=false 是**刻意**制造不可评估，用于 F_ 系列失败策略场景。
  bool isObstacleQueryAvailable() const override {return available_;}

  // 经生产接缝供给逐点限速——这是 M11 区域限速今天唯一能离线验证的路径。
  float pointSpeedHint(
    const nav_msgs::msg::Path & path, size_t index) const override
  {
    if (zones_.empty() || index >= path.poses.size()) {return 0.0f;}
    const double x = path.poses[index].pose.position.x;
    float hint = 0.0f;
    for (const auto & z : zones_) {
      if (x >= z.from && x <= z.to) {
        hint = (hint > 0.0f) ?
          std::min(hint, static_cast<float>(z.speed)) :
          static_cast<float>(z.speed);
      }
    }
    return hint;
  }

  nav2_speed_planner::PointObstacleStatus checkObstacleAt(
    double x, double /*y*/, double /*theta*/) const override
  {
    for (const auto & b : bands_) {
      if (x >= b.from && x <= b.to) {
        return nav2_speed_planner::PointObstacleStatus::OBSTACLE;
      }
    }
    return nav2_speed_planner::PointObstacleStatus::FREE;
  }
};

using HarnessT = Harness<nav2_speed_planner::TrapezoidalSpeedPlanner>;

class HarnessR : public Harness<nav2_speed_planner::ReachabilitySpeedPlanner>
{
public:
  void setJerk(double j) {max_jerk_ = j;}
};

class HarnessS : public Harness<nav2_speed_planner::SCurveSpeedPlanner>
{
public:
  void setSCurveParams(const Params & q = Params{})
  {
    max_jerk_ = q.max_jerk;
    control_dt_ = q.control_dt;
  }
  void setTerminalExecutionDelay(double delay)
  {
    terminal_execution_delay_ = delay;
  }
};

/// 最近一次 runScenario 的规划诊断。用于回答"不可行到底卡在哪一步"——
/// 7 个失败站点里只有 1 个是求解器真的报错，其余是本项目自己的判据
/// （duration budget 启发式、步数预算、手写的稳定末段生成）。
inline nav2_core::SpeedPlanningDiagnostics & lastDiagnostics()
{
  static nav2_core::SpeedPlanningDiagnostics d;
  return d;
}

inline nav_interfaces::msg::RobotTrajectory runScenario(
  const Scenario & sc, const nav_msgs::msg::Path & path)
{
  nav2_core::InitialMotionState boundary;
  boundary.velocity.linear.x = sc.v0;
  boundary.terminal_brake_latched = sc.terminal_brake_latched;

  if (sc.backend == Backend::Reachability) {
    HarnessR p;
    p.setSharedParams(sc.params);
    p.setJerk(sc.params.max_jerk);
    if (sc.override_end_speed) {p.setEndSpeed(sc.end_speed);}
    p.setPolicy(sc.unavailable_policy);
    p.bands_ = sc.obstacles;
    p.zones_ = sc.zones;
    p.available_ = sc.obstacle_available;
    if (sc.external_limit > 0.0) {p.setSpeedLimit(sc.external_limit, false);}
    auto traj = p.computeSpeedPlan(path, boundary);
    lastDiagnostics() = p.getLastPlanningDiagnostics();
    return traj;
  }
  if (sc.backend == Backend::Trapezoidal) {
    HarnessT p;
    p.setSharedParams(sc.params);
    if (sc.override_end_speed) {p.setEndSpeed(sc.end_speed);}
    p.setPolicy(sc.unavailable_policy);
    p.bands_ = sc.obstacles;
    p.zones_ = sc.zones;
    p.available_ = sc.obstacle_available;
    if (sc.external_limit > 0.0) {p.setSpeedLimit(sc.external_limit, false);}
    auto traj = p.computeSpeedPlan(path, boundary);
    lastDiagnostics() = p.getLastPlanningDiagnostics();
    return traj;
  }
  HarnessS p;
  p.setSharedParams(sc.params);
  p.setSCurveParams(sc.params);
  p.setTerminalExecutionDelay(sc.terminal_execution_delay);
  if (sc.override_end_speed) {p.setEndSpeed(sc.end_speed);}
  p.setPolicy(sc.unavailable_policy);
  p.bands_ = sc.obstacles;
  p.zones_ = sc.zones;
  p.available_ = sc.obstacle_available;
  if (sc.external_limit > 0.0) {p.setSpeedLimit(sc.external_limit, false);}
  auto traj = p.computeSpeedPlan(path, boundary);
  lastDiagnostics() = p.getLastPlanningDiagnostics();
  return traj;
}

inline nav_interfaces::msg::RobotTrajectory runScenario(const Scenario & sc)
{
  return runScenario(
    sc, makePath(
      sc.length, sc.arcs, sc.path_kind, sc.lateral_offset, sc.dense_spacing));
}

}  // namespace speed_offline

#endif  // TOOLS__SPEED_HARNESS_HPP_
