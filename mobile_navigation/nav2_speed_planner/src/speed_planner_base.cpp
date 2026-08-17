// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 速度规划器抽象基类实现：Phase A 约束包络（障碍 → 巡航 → 曲率 → 末端蠕行，
// 逐点 min 叠加，安全置零绝对优先）+ Phase B 纯虚 hook。
// base 不含任何具体速度剖面算法（恒加速度剖面在 Trapezoidal，jerk 限制剖面在 SCurve）。
#include "nav2_speed_planner/speed_planner_base.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"

namespace nav2_speed_planner
{

#include "constraints/planner_internal.hpp"

using namespace nav2_speed_planner::internal;  // NOLINT(build/namespaces)

// ============================ 生命周期 ============================

void SpeedPlannerBase::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  // R47：与 RPP 同源的 footprint 碰撞查询；costmap 指针在 footprintCostAt 每次调用时用
  // setCostmap 刷新，构造期传空指针即可（RPP 也是同一时机以 costmap_ 构造）。
  collision_checker_ = std::make_unique<nav2_costmap_2d::
      FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(
    costmap_ros_ ? costmap_ros_->getCostmap() : nullptr);

  auto node = parent.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_speed", rclcpp::ParameterValue(0.6));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_speed", rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".accel", rclcpp::ParameterValue(0.35));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".decel", rclcpp::ParameterValue(-0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_stencil_arc", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".regulated_linear_scaling_min_radius", rclcpp::ParameterValue(1.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_curvature_speed", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".constant_velocity_dist_threshold", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".curve_lead_in_distance", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".curve_lead_out_distance", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".enable_constant_velocity_while_turning", rclcpp::ParameterValue(false));

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".enable_curvature_constraint", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lateral_accel", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".end_speed", rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_creep_distance", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".obstacle_safety_margin", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".obstacle_stop_margin", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".obstacle_cost_threshold", rclcpp::ParameterValue(254));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".obstacle_unavailable_policy", rclcpp::ParameterValue(
      std::string("fail_closed")));

  node->get_parameter(plugin_name_ + ".max_speed", max_speed_);
  node->get_parameter(plugin_name_ + ".min_speed", min_speed_);
  node->get_parameter(plugin_name_ + ".accel", accel_);
  node->get_parameter(plugin_name_ + ".decel", decel_);
  node->get_parameter(plugin_name_ + ".curvature_stencil_arc", curvature_stencil_arc_);
  node->get_parameter(
    plugin_name_ + ".regulated_linear_scaling_min_radius", regulated_linear_scaling_min_radius_);
  node->get_parameter(plugin_name_ + ".min_curvature_speed", min_curvature_speed_);
  node->get_parameter(
    plugin_name_ + ".constant_velocity_dist_threshold", constant_velocity_dist_threshold_);
  node->get_parameter(plugin_name_ + ".curve_lead_in_distance", curve_lead_in_distance_);
  node->get_parameter(plugin_name_ + ".curve_lead_out_distance", curve_lead_out_distance_);
  node->get_parameter(
    plugin_name_ + ".enable_constant_velocity_while_turning",
    enable_constant_velocity_while_turning_);
  node->get_parameter(
    plugin_name_ + ".enable_curvature_constraint", enable_curvature_constraint_);
  node->get_parameter(plugin_name_ + ".max_lateral_accel", max_lateral_accel_);
  max_lateral_accel_ = std::max(max_lateral_accel_, 1e-3);
  node->get_parameter(plugin_name_ + ".end_speed", end_speed_);
  end_speed_ = std::max(end_speed_, 0.0);
  node->get_parameter(plugin_name_ + ".terminal_creep_distance", terminal_creep_distance_);
  node->get_parameter(plugin_name_ + ".obstacle_safety_margin", obstacle_safety_margin_);
  obstacle_safety_margin_ = std::max(obstacle_safety_margin_, 0.0);
  node->get_parameter(plugin_name_ + ".obstacle_stop_margin", obstacle_stop_margin_);
  obstacle_stop_margin_ = std::max(obstacle_stop_margin_, 0.0);
  node->get_parameter(plugin_name_ + ".obstacle_cost_threshold", obstacle_cost_threshold_);
  obstacle_cost_threshold_ = std::clamp(obstacle_cost_threshold_, 1, 254);
  std::string obstacle_unavailable_policy = "fail_closed";
  node->get_parameter(plugin_name_ + ".obstacle_unavailable_policy", obstacle_unavailable_policy);
  if (obstacle_unavailable_policy == "fail_open") {
    obstacle_unavailable_policy_ = ConstraintFailurePolicy::FAIL_OPEN;
    RCLCPP_WARN(
      logger_,
      "[%s] obstacle_unavailable_policy=fail_open: when TF or costmap is unavailable the "
      "robot will keep cruising WITHOUT obstacle constraints. Intended for simulation or "
      "explicit grayscale only.", plugin_name_.c_str());
  } else {
    obstacle_unavailable_policy_ = ConstraintFailurePolicy::FAIL_CLOSED;
    if (obstacle_unavailable_policy != "fail_closed") {
      RCLCPP_WARN(
        logger_, "[%s] unknown obstacle_unavailable_policy '%s'; falling back to fail_closed.",
        plugin_name_.c_str(), obstacle_unavailable_policy.c_str());
    }
  }

  // 参数非法只告警不 abort（避免拖垮整个 nav 栈）。
  if (max_speed_ <= min_speed_) {
    RCLCPP_WARN(
      logger_, "[%s] max_speed (%.3f) should be > min_speed (%.3f).",
      plugin_name_.c_str(), max_speed_, min_speed_);
  }
  if (accel_ <= 0.0) {
    RCLCPP_WARN(logger_, "[%s] accel (%.3f) should be > 0.", plugin_name_.c_str(), accel_);
  }
  if (decel_ >= 0.0) {
    RCLCPP_WARN(logger_, "[%s] decel (%.3f) should be < 0.", plugin_name_.c_str(), decel_);
  }

  RCLCPP_INFO(logger_, "Configured SpeedPlanner plugin: %s", plugin_name_.c_str());
}


void SpeedPlannerBase::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up SpeedPlanner plugin: %s", plugin_name_.c_str());
  tf_.reset();
  costmap_ros_.reset();
}


void SpeedPlannerBase::activate()
{
  RCLCPP_INFO(logger_, "Activating SpeedPlanner plugin: %s", plugin_name_.c_str());
}


void SpeedPlannerBase::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating SpeedPlanner plugin: %s", plugin_name_.c_str());
}


void SpeedPlannerBase::setPlan(const nav_msgs::msg::Path & path)
{
  current_plan_ = path;
}


void SpeedPlannerBase::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

// ============================ 模板方法入口 ============================


nav_interfaces::msg::RobotTrajectory SpeedPlannerBase::computeSpeedPlan(
  const nav_msgs::msg::Path & path,
  const nav2_core::InitialMotionState & boundary)
{
  last_planning_diagnostics_ = nav2_core::SpeedPlanningDiagnostics();
  using TimingClock = std::chrono::steady_clock;
  constexpr double kSlowSpeedPlannerWarnMs = 20.0;
  const auto total_start = TimingClock::now();
  double input_ms = 0.0;
  double initialization_ms = 0.0;
  double obstacle_ms = 0.0;
  double cruise_ms = 0.0;
  double curvature_ms = 0.0;
  double end_constraint_ms = 0.0;
  double profile_ms = 0.0;
  double stats_ms = 0.0;
  double output_ms = 0.0;
  double path_length = 0.0;
  size_t work_points = 0;
  last_obstacle_timing_ = ObstacleConstraintTiming();

  nav_interfaces::msg::RobotTrajectory traj;
  traj.header = path.header;
  path_frame_ = path.header.frame_id;

  const auto log_timing = [&](const char * result) {
      const double total_ms = std::chrono::duration<double, std::milli>(
        TimingClock::now() - total_start).count();
      const double measured_ms =
        input_ms + initialization_ms + obstacle_ms + cruise_ms + curvature_ms +
        end_constraint_ms + profile_ms + stats_ms + output_ms;
      const double unaccounted_ms = std::max(0.0, total_ms - measured_ms);
      const std::string costmap_frame = costmap_ros_ ? costmap_ros_->getGlobalFrameID() : "";
      if (total_ms >= kSlowSpeedPlannerWarnMs && clock_) {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 1000,
          "[%s] speed planner timing (ms): result=%s total=%.3f input=%.3f init=%.3f "
          "obstacle=%.3f (tf=%.3f scan=%.3f apply=%.3f) cruise=%.3f curvature=%.3f "
          "end=%.3f profile=%.3f stats=%.3f output=%.3f unaccounted=%.3f "
          "tf_required=%s frames=%s->%s points=%zu->%zu->%zu length=%.3f",
          plugin_name_.c_str(), result, total_ms, input_ms, initialization_ms, obstacle_ms,
          last_obstacle_timing_.tf_ms, last_obstacle_timing_.scan_ms,
          last_obstacle_timing_.apply_ms, cruise_ms, curvature_ms, end_constraint_ms,
          profile_ms, stats_ms, output_ms, unaccounted_ms,
          last_obstacle_timing_.transform_required ? "true" : "false",
          path_frame_.c_str(), costmap_frame.c_str(), path.poses.size(), work_points,
          traj.points.size(), path_length);
      } else if (clock_) {
        RCLCPP_DEBUG_THROTTLE(
          logger_, *clock_, 1000,
          "[%s] speed planner timing (ms): result=%s total=%.3f input=%.3f init=%.3f "
          "obstacle=%.3f (tf=%.3f scan=%.3f apply=%.3f) cruise=%.3f curvature=%.3f "
          "end=%.3f profile=%.3f stats=%.3f output=%.3f unaccounted=%.3f "
          "tf_required=%s frames=%s->%s points=%zu->%zu->%zu length=%.3f",
          plugin_name_.c_str(), result, total_ms, input_ms, initialization_ms, obstacle_ms,
          last_obstacle_timing_.tf_ms, last_obstacle_timing_.scan_ms,
          last_obstacle_timing_.apply_ms, cruise_ms, curvature_ms, end_constraint_ms,
          profile_ms, stats_ms, output_ms, unaccounted_ms,
          last_obstacle_timing_.transform_required ? "true" : "false",
          path_frame_.c_str(), costmap_frame.c_str(), path.poses.size(), work_points,
          traj.points.size(), path_length);
      }
    };

  // 构建内部工作结构。
  const auto input_start = TimingClock::now();
  std::vector<TrajPointWork> work;
  work.reserve(path.poses.size());
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto & ps = path.poses[i];
    TrajPointWork p;
    p.pose = ps;
    // nav_msgs/Path 无段语义字段：默认无提示。带语义 path 接入后（P0-2）
    // 由 pointSpeedHint 覆写供给；基类实现恒返回 0，行为与此前完全一致。
    p.speed_hint = pointSpeedHint(path, i);
    p.region_type = "";
    p.segment_id = "";
    // 仅合并几何与朝向均等价的相邻点。相同位置但不同朝向的点是终点
    // 原地转向轨迹，不得作为曲率噪声删除；合并时取后到者保住精确 goal。
    if (!work.empty()) {
      const auto & prev = work.back().pose.pose.position;
      const double dx = ps.pose.position.x - prev.x;
      const double dy = ps.pose.position.y - prev.y;
      if (dx * dx + dy * dy < kDuplicateDistanceSq &&
        orientationsEquivalent(ps.pose.orientation, work.back().pose.pose.orientation))
      {
        if (i + 1 == path.poses.size() && work.size() == 1) {
          // Keep both roles in a two-point terminal path even when start and
          // goal have identical geometry. Controllers may require both.
          work.push_back(std::move(p));
          continue;
        }
        work.back().pose = p.pose;
        continue;
      }
    }
    work.push_back(std::move(p));
  }

  path_length = computePathLength(work);
  work_points = work.size();
  input_ms = std::chrono::duration<double, std::milli>(
    TimingClock::now() - input_start).count();
  if (work.size() < 2 || path_length <= kGeometryEpsilon) {
    // 点数不足或纯原地转向：保留完整 pose 序列并禁止线速度。
    const auto output_start = TimingClock::now();
    for (const auto & p : work) {
      nav_interfaces::msg::TrajectoryPoint tp;
      tp.pose = p.pose;
      tp.longitudinal_velocity = 0.0;
      tp.path_kappa = 0.0;
      tp.speed_hint = p.speed_hint;
      tp.region_type = p.region_type;
      tp.segment_id = p.segment_id;
      traj.points.push_back(std::move(tp));
    }
    output_ms = std::chrono::duration<double, std::milli>(
      TimingClock::now() - output_start).count();
    log_timing("terminal_geometry");
    return traj;
  }

  // 初始限速取严：max_speed / path speed_hint / 外部限速。
  const auto initialization_start = TimingClock::now();
  double external_limit = std::numeric_limits<double>::max();
  if (speed_limit_ > 0.0) {
    external_limit = speed_limit_is_percentage_ ?
      (max_speed_ * speed_limit_ / 100.0) : speed_limit_;
  }
  for (auto & p : work) {
    double init_limit = max_speed_;
    ConstraintSource src = ConstraintSource::kMaxSpeed;
    if (p.speed_hint > 0.0f && static_cast<double>(p.speed_hint) < init_limit) {
      init_limit = static_cast<double>(p.speed_hint);
      src = ConstraintSource::kSpeedHint;
    }
    if (external_limit < init_limit) {
      init_limit = external_limit;
      src = ConstraintSource::kExternalLimit;
    }
    p.velocity_upper_bound = init_limit;
    p.constraint_source = src;
  }

  insertEndSpeedBoundary(work);
  work_points = work.size();

  points_interval_ = path_length / static_cast<double>(work.size() - 1);

  const double curr_vel = boundary.velocity.linear.x;
  const double curr_accel = boundary.acceleration_valid ? boundary.acceleration : 0.0;
  initialization_ms = std::chrono::duration<double, std::milli>(
    TimingClock::now() - initialization_start).count();

  // ====== Phase A 约束包络（逐点 min 叠加，安全置零绝对优先）======
  const auto obstacle_start = TimingClock::now();
  const ObstacleConstraintOutcome obstacle_outcome =
    produceObstacleConstraint(work, boundary.should_stop_for_path_obstacle);
  applyConstraint(work, obstacle_outcome.constraint);
  obstacle_ms = std::chrono::duration<double, std::milli>(
    TimingClock::now() - obstacle_start).count();
  const bool has_obstacle = obstacle_outcome.has_obstacle;

  // 调研 P0-3：障碍约束**无法评估**时按显式策略处置，而不是静默当作"无障碍"。
  // FAIL_CLOSED（生产默认）返回空轨迹 → runPlanningPipeline 判空 → 控制器发零速。
  if (!obstacle_outcome.evaluable) {
    if (obstacle_unavailable_policy_ == ConstraintFailurePolicy::FAIL_CLOSED) {
      // clock_ 仅在 ROS configure() 后可用；离线测试壳直接设参数、不走 configure。
      if (clock_) {
        RCLCPP_ERROR_THROTTLE(
          logger_, *clock_, 1000,
          "[%s] obstacle constraint unavailable (%s) and policy=FAIL_CLOSED: "
          "emitting empty trajectory so the controller stops this cycle.",
          plugin_name_.c_str(), obstacle_outcome.unavailable_reason.c_str());
      }
      traj.points.clear();
      log_timing("obstacle_constraint_unavailable_fail_closed");
      return traj;
    }
    if (clock_) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "[%s] obstacle constraint unavailable (%s) but policy=FAIL_OPEN: "
        "continuing WITHOUT any obstacle constraint this cycle.",
        plugin_name_.c_str(), obstacle_outcome.unavailable_reason.c_str());
    }
  }
  const auto cruise_start = TimingClock::now();
  // A2 巡航基准：纯查询，不产生约束。
  // 它读的是**障碍约束之后**的包络（跳过已置零的点），这个顺序依赖真实存在。
  // 结果只作为 A3 的限速基准与 Phase B 的巡航目标；此处不再回填包络——
  // 初始化时已 `min(max_speed, hint, external)`，Phase A 内无任何环节会抬高，
  // 再施加一次全局上界恒为空操作。
  const double cruise = computeProfileCruiseTarget(work);
  cruise_ms = std::chrono::duration<double, std::milli>(
    TimingClock::now() - cruise_start).count();
  const auto curvature_start = TimingClock::now();
  // 曲率是路径的几何属性，由编排层算一次并写入；约束只消费不重算。
  const std::vector<double> curvature_vec =
    calcTrajectoryCurvatureFrom3Points(work, curvature_stencil_arc_);
  for (size_t i = 0; i < work.size() && i < curvature_vec.size(); ++i) {
    work[i].curvature = curvature_vec[i];
  }
  // A3 横向加速度约束 `v ≤ √(a_lat_max/|κ|)`。**不再依赖 cruise**——旧公式
  // 以巡航速度为基准，是"A3 必须排在 A2 之后"的唯一来源；改正后四条约束
  // 在值语义上可自由组合（合并是逐点 min，可交换可结合）。
  applyConstraint(work, produceLateralAccelConstraint(work, curvature_vec));
  curvature_ms = std::chrono::duration<double, std::milli>(
    TimingClock::now() - curvature_start).count();
  const auto end_constraint_start = TimingClock::now();
  applyConstraint(work, produceTerminalConstraint(work));  // A4 末端蠕行区
  end_constraint_ms = std::chrono::duration<double, std::milli>(
    TimingClock::now() - end_constraint_start).count();

  // ── 包络归因自检 + 组成摘要 ──────────────────────────────────────
  //
  // 不变量：任何低于 max_speed_ 的点都必须有一条约束对它负责。若出现
  // kNone，说明存在一处**未经 lowerBound 的下压**——包络被谁改了将无法追溯，
  // 这正是此前"为什么这段速度是这个值"只能靠通读全部约束反推的原因。
  {
    size_t unattributed = 0;
    constexpr size_t kConstraintSourceCount =
      static_cast<size_t>(ConstraintSource::kCount);
    std::array<size_t, kConstraintSourceCount> by_source{};
    for (const auto & p : work) {
      if (p.velocity_upper_bound < max_speed_ - 1e-9) {
        const auto idx = static_cast<size_t>(p.constraint_source);
        if (idx < by_source.size()) {
          ++by_source[idx];
        }
        if (p.constraint_source == ConstraintSource::kNone) {
          ++unattributed;
        }
      }
    }
    if (unattributed > 0 && clock_) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000,
        "[%s] %zu/%zu envelope points are below max_speed with no attributed "
        "constraint — a lowering site is bypassing lowerBound().",
        plugin_name_.c_str(), unattributed, work.size());
    }
    RCLCPP_DEBUG(
      logger_,
      "[%s] envelope composition: max_speed=%zu hint=%zu external=%zu "
      "obstacle=%zu safety_margin=%zu stop_margin=%zu lateral_accel=%zu creep=%zu "
      "end_boundary=%zu "
      "unattributed=%zu",
      plugin_name_.c_str(),
      by_source[static_cast<size_t>(ConstraintSource::kMaxSpeed)],
      by_source[static_cast<size_t>(ConstraintSource::kSpeedHint)],
      by_source[static_cast<size_t>(ConstraintSource::kExternalLimit)],
      by_source[static_cast<size_t>(ConstraintSource::kObstacle)],
      by_source[static_cast<size_t>(ConstraintSource::kObstacleSafetyMargin)],
      by_source[static_cast<size_t>(ConstraintSource::kObstacleStopMargin)],
      by_source[static_cast<size_t>(ConstraintSource::kLateralAccel)],
      by_source[static_cast<size_t>(ConstraintSource::kTerminalCreep)],
      by_source[static_cast<size_t>(ConstraintSource::kEndBoundary)],
      unattributed);
  }

  // ====== Phase B hook：派生类生成运动学可行剖面 ======
  const auto profile_start = TimingClock::now();
  generateProfile(
    work, curr_vel, curr_accel, cruise, boundary.terminal_brake_latched);
  profile_ms = std::chrono::duration<double, std::milli>(
    TimingClock::now() - profile_start).count();

  // 可观测：巡航上界 vs 实际峰值（"能否到最大限速"直接看日志）。
  const auto stats_start = TimingClock::now();
  double peak = 0.0;
  for (const auto & p : work) {
    peak = std::max(peak, p.velocity_upper_bound);
  }
  RCLCPP_DEBUG(
    logger_,
    "[%s] speed profile: cruise_limit=%.3f peak=%.3f reached=%s obstacle=%s points=%zu "
    "scanned=%zu out_of_range=%zu",
    plugin_name_.c_str(), cruise, peak,
    peak + 1e-6 >= cruise ? "true" : "false",
    has_obstacle ? "true" : "false", work.size(),
    obstacle_outcome.scanned_points, obstacle_outcome.out_of_range_points);
  stats_ms = std::chrono::duration<double, std::milli>(
    TimingClock::now() - stats_start).count();

  // 回填 RobotTrajectory。
  const auto output_start = TimingClock::now();
  traj.points.reserve(work.size());
  for (const auto & p : work) {
    nav_interfaces::msg::TrajectoryPoint tp;
    tp.pose = p.pose;
    tp.longitudinal_velocity = p.velocity_upper_bound;
    tp.path_kappa = p.curvature;
    tp.acceleration = p.accel;
    tp.jerk = p.jerk;
    tp.relative_time = rclcpp::Duration::from_seconds(p.rel_time);
    tp.speed_hint = p.speed_hint;
    tp.region_type = p.region_type;
    tp.segment_id = p.segment_id;
    traj.points.push_back(std::move(tp));
  }
  output_ms = std::chrono::duration<double, std::milli>(
    TimingClock::now() - output_start).count();

  log_timing("success");
  return traj;
}


nav2_core::SpeedPlanningDiagnostics SpeedPlannerBase::getLastPlanningDiagnostics() const
{
  return last_planning_diagnostics_;
}

// ============================ Phase A 约束包络 ============================


double SpeedPlannerBase::computePathLength(const std::vector<TrajPointWork> & path)
{
  double length = 0.0;
  for (int i = 1; i < static_cast<int>(path.size()); ++i) {
    double dx = path.at(i).pose.pose.position.x - path.at(i - 1).pose.pose.position.x;
    double dy = path.at(i).pose.pose.position.y - path.at(i - 1).pose.pose.position.y;
    length += std::sqrt(dx * dx + dy * dy);
  }
  return length;
}


std::vector<double> SpeedPlannerBase::computeCumulativeArcLengths(
  const std::vector<TrajPointWork> & path)
{
  std::vector<double> arc_s(path.size(), 0.0);
  for (size_t i = 1; i < path.size(); ++i) {
    const double dx = path[i].pose.pose.position.x - path[i - 1].pose.pose.position.x;
    const double dy = path[i].pose.pose.position.y - path[i - 1].pose.pose.position.y;
    arc_s[i] = arc_s[i - 1] + std::hypot(dx, dy);
  }
  return arc_s;
}


}  // namespace nav2_speed_planner
