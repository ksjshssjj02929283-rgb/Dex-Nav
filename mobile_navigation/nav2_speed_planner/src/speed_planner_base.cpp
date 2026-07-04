// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 速度规划器抽象基类实现。
// computeSpeedPlan 为模板方法（path→work→assignVelocityProfile→RobotTrajectory）；
// assignConstantAccelProfile 是恒加速度参考内核（源自 dcm tcurve）：
// Phase A 约束包络（障碍 → 巡航 → 曲率，逐点 min 叠加，安全置零绝对优先）
// + Phase B 运动学可行化（加速段 → 减速段，从当前实际速度出发）。
#include "nav2_speed_planner/speed_planner_base.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_speed_planner
{

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

  auto node = parent.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // 参数声明（默认值同 dcm initialize）。
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_speed", rclcpp::ParameterValue(0.6));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_speed", rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".accel", rclcpp::ParameterValue(0.35));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".decel", rclcpp::ParameterValue(-0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".curve_compute_dist", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".decel_distance_before_curve", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".decel_distance_after_curve", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".regulated_linear_scaling_min_radius", rclcpp::ParameterValue(1.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_curvature_speed", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".constant_velocity_dist_threshold", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".enable_constant_velocity_while_turning", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".end_speed", rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".stop_distance", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".collision_stop_distance", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".obstacle_cost_threshold", rclcpp::ParameterValue(253));

  node->get_parameter(plugin_name_ + ".max_speed", max_speed_);
  node->get_parameter(plugin_name_ + ".min_speed", min_speed_);
  node->get_parameter(plugin_name_ + ".accel", accel_);
  node->get_parameter(plugin_name_ + ".decel", decel_);
  node->get_parameter(plugin_name_ + ".curve_compute_dist", curve_compute_dist_);
  node->get_parameter(plugin_name_ + ".decel_distance_before_curve", decel_distance_before_curve_);
  node->get_parameter(plugin_name_ + ".decel_distance_after_curve", decel_distance_after_curve_);
  node->get_parameter(
    plugin_name_ + ".regulated_linear_scaling_min_radius", regulated_linear_scaling_min_radius_);
  node->get_parameter(plugin_name_ + ".min_curvature_speed", min_curvature_speed_);
  node->get_parameter(
    plugin_name_ + ".constant_velocity_dist_threshold", constant_velocity_dist_threshold_);
  node->get_parameter(
    plugin_name_ + ".enable_constant_velocity_while_turning",
    enable_constant_velocity_while_turning_);
  node->get_parameter(plugin_name_ + ".end_speed", end_speed_);
  end_speed_ = std::max(end_speed_, 0.0);
  node->get_parameter(plugin_name_ + ".stop_distance", stop_distance_);
  node->get_parameter(plugin_name_ + ".collision_stop_distance", collision_stop_distance_);
  node->get_parameter(plugin_name_ + ".obstacle_cost_threshold", obstacle_cost_threshold_);
  obstacle_cost_threshold_ = std::clamp(obstacle_cost_threshold_, 1, 254);

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
  const geometry_msgs::msg::Twist & current_velocity)
{
  nav_interfaces::msg::RobotTrajectory traj;
  traj.header = path.header;

  // 构建内部工作结构（对齐 dcm dcm_msgs::Path.path_poses）。
  std::vector<TrajPointWork> work;
  work.reserve(path.poses.size());
  for (const auto & ps : path.poses) {
    TrajPointWork p;
    p.pose = ps;
    // nav_msgs/Path 无段语义字段：透传置默认（known_gap：未来由带语义 path 提供）。
    p.speed_hint = 0.0f;
    p.region_type = "";
    p.segment_id = "";
    // 相邻重复点去重（<0.1mm）：重复点破坏曲率三点拟合并使点距统计失真；
    // 姿态取后到者（保住终点朝向）。
    if (!work.empty()) {
      const auto & prev = work.back().pose.pose.position;
      const double dx = ps.pose.position.x - prev.x;
      const double dy = ps.pose.position.y - prev.y;
      if (dx * dx + dy * dy < 1e-8) {
        work.back().pose = p.pose;
        continue;
      }
    }
    work.push_back(std::move(p));
  }

  if (work.size() < 2) {
    // 点数不足，原样返回空速轨迹（对齐 dcm return false 行为）。
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
    return traj;
  }

  // 把外部限速 + path speed_hint 取严，作为各点初始 speed_limit。
  // 外部限速解析：百分比→max_speed_ 比例；绝对值直接用。speed_limit_<=0 视为不限速。
  double external_limit = std::numeric_limits<double>::max();
  if (speed_limit_ > 0.0) {
    external_limit = speed_limit_is_percentage_ ?
      (max_speed_ * speed_limit_ / 100.0) : speed_limit_;
  }
  for (auto & p : work) {
    double init_limit = max_speed_;
    if (p.speed_hint > 0.0f) {
      init_limit = std::min(init_limit, static_cast<double>(p.speed_hint));
    }
    init_limit = std::min(init_limit, external_limit);
    p.speed_limit = init_limit;
  }

  curr_path_length_ = computePathLength(work);
  points_interval_ = curr_path_length_ / static_cast<double>(work.size() - 1);

  const double curr_vel = current_velocity.linear.x;

  // —— Template Method hook：派生类填速度剖面（T 型 = 恒加速度内核；S 型 = jerk 限制）——
  assignVelocityProfile(work, curr_vel);

  // 回填到 RobotTrajectory：speed_limit→longitudinal_velocity，curvature→path_kappa，pose 透传。
  // accel / rel_time 若派生类填了则一并写，否则默认 0（恒加速度内核不填）。
  traj.points.reserve(work.size());
  for (const auto & p : work) {
    nav_interfaces::msg::TrajectoryPoint tp;
    tp.pose = p.pose;
    tp.longitudinal_velocity = p.speed_limit;
    tp.path_kappa = p.curvature;
    tp.acceleration = p.accel;
    tp.relative_time = rclcpp::Duration::from_seconds(p.rel_time);
    // 段语义透传（当前 nav_msgs/Path 无字段，置默认）。
    tp.speed_hint = p.speed_hint;
    tp.region_type = p.region_type;
    tp.segment_id = p.segment_id;
    traj.points.push_back(std::move(tp));
  }

  return traj;
}

// ============================ 恒加速度参考内核 ============================
// 两阶段：Phase A 约束包络（障碍/巡航/曲率，逐点 min 叠加，安全置零绝对优先）；
// Phase B 运动学可行化（加速段/减速段，从当前实际速度出发）。
// T 型直接调用全流程；S 型复用 Phase A 后交给 Ruckig。

void SpeedPlannerBase::assignConstantAccelProfile(
  std::vector<TrajPointWork> & work, double curr_vel)
{
  // ====== T 型 pipeline（两阶段：约束包络 → 运动学可行化）======
  // Phase A 约束包络（逐点 min 叠加，安全置零绝对优先）
  const bool has_obstacle = applyObstacleConstraint(work);   // A1 障碍 + 缓冲带置零
  const double speed_limit = getSpeedLimit(work, curr_vel);  // A2 理想巡航求解（纯查询）
  setPlanSpeedLimit(work, speed_limit);                      //    巡航回填
  applyCurvatureConstraint(work, speed_limit);               // A3 曲率约束

  // Phase B 运动学可行化（receding-horizon：从当前实际速度出发）
  computeAccelPlan(work, curr_vel, speed_limit);             // B1 加速段
  computeDecelPlan(work, curr_vel, speed_limit);             // B2 减速段

  // 可观测：巡航上界 vs 实际峰值（"能否到最大限速"直接看日志）。
  double peak = 0.0;
  for (const auto & p : work) {
    peak = std::max(peak, p.speed_limit);
  }
  RCLCPP_INFO(
    logger_,
    "[%s] speed profile: cruise_limit=%.3f peak=%.3f reached=%s obstacle=%s points=%zu",
    plugin_name_.c_str(), speed_limit, peak,
    peak + 1e-6 >= speed_limit ? "true" : "false",
    has_obstacle ? "true" : "false", work.size());
}

// ============================ 限速求解 ============================

void SpeedPlannerBase::setPlanSpeedLimit(
  std::vector<TrajPointWork> & path, const double speed_limit)
{
  // 双边钳制到 [min_speed_, max_speed_]；零速点（安全置零）跳过，不回填巡航速度。
  const double valid_speed_limit = std::clamp(speed_limit, min_speed_, max_speed_);

  for (size_t i = 0; i < path.size(); i++) {
    if (path[i].speed_limit < 1e-3) {
      continue;
    }
    path[i].speed_limit = valid_speed_limit;
  }
}

bool SpeedPlannerBase::applyObstacleConstraint(std::vector<TrajPointWork> & path)
{
  // A1 障碍约束（安全置零绝对优先）：首个障碍点及其后全部置零，
  // 障碍前 collision_stop_distance_ 缓冲带置零（index 0 为机器人当前位置，不动）。
  if (path.empty()) {
    return false;
  }
  size_t first_obstacle_index = path.size();
  for (size_t i = 0; i < path.size(); ++i) {
    if (isObstacleAt(path[i].pose.pose.position.x, path[i].pose.pose.position.y)) {
      first_obstacle_index = i;
      break;
    }
  }
  if (first_obstacle_index == path.size()) {
    return false;
  }
  for (size_t i = first_obstacle_index; i < path.size(); ++i) {
    path[i].speed_limit = 0.0;
  }
  const std::vector<double> arc_s = computeCumulativeArcLengths(path);
  const double buffer_start = arc_s[first_obstacle_index] - collision_stop_distance_;
  for (size_t j = first_obstacle_index; j-- > 1;) {
    if (arc_s[j] <= buffer_start) {
      break;
    }
    path[j].speed_limit = 0.0;
  }
  return true;
}

double SpeedPlannerBase::getSpeedLimit(
  const std::vector<TrajPointWork> & path, const double curr_vel)
{
  // A2 理想巡航上界（纯查询，无副作用）：回答"理想状态下当前路径能到达的最大速度"。
  // 障碍置零已由 applyObstacleConstraint（A1）先行，安全零速点不参与上界求解。
  double speed_limit = std::numeric_limits<double>::max();
  bool has_zero = false;
  for (size_t i = 0; i < path.size(); i++) {
    if (path[i].speed_limit < 1e-3) {
      has_zero = true;
      continue;
    }
    if (speed_limit > path[i].speed_limit) {
      speed_limit = path[i].speed_limit;
    }
  }
  if (!(speed_limit < std::numeric_limits<double>::max())) {
    // 全部点被安全置零（起点即阻塞）。
    return 0.0;
  }
  speed_limit = std::clamp(speed_limit, min_speed_, max_speed_);

  const double curr_min_speed = has_zero ? 0.0 : min_speed_;

  // 加减速所需距离 vs 有效路径长度（receding-horizon：基于当前速度）。
  const double accel_path_length = computeAccelLength(curr_vel, speed_limit);
  const double decel_path_length = computeDecelLength(curr_vel, curr_min_speed);

  const double path_length = computeValidSpeedPathLength(path);
  const double no_stop_length = std::max(path_length - stop_distance_, 0.0);
  if (accel_path_length + decel_path_length > no_stop_length) {
    // 不能到达最大速度，回退求解可达峰值。
    speed_limit = std::sqrt(
      (2 * accel_ * decel_ * no_stop_length + decel_ * curr_vel * curr_vel -
      accel_ * curr_min_speed * curr_min_speed) / (decel_ - accel_));
    return speed_limit;
  }
  // 可以到达最大速度。
  return speed_limit;
}

double SpeedPlannerBase::computeValidSpeedPathLength(const std::vector<TrajPointWork> & path)
{
  double length = 0.0;
  for (int i = 1; i < static_cast<int>(path.size()); ++i) {
    double dx = path.at(i).pose.pose.position.x - path.at(i - 1).pose.pose.position.x;
    double dy = path.at(i).pose.pose.position.y - path.at(i - 1).pose.pose.position.y;
    length += std::sqrt(dx * dx + dy * dy);

    // 第一个零速点也包含在路径长度中。
    if (path.at(i).speed_limit == 0.0) {
      break;
    }
  }
  return length;
}

// ============================ 加速段 ============================

void SpeedPlannerBase::computeAccelPlan(
  std::vector<TrajPointWork> & path, const double curr_vel, const double speed_limit)
{
  const double accel_path_length = computeAccelLength(curr_vel, speed_limit);
  // 安全置零绝对优先：起点被置零（阻塞）时不得抬升。
  if (path[0].speed_limit >= 1e-3) {
    path[0].speed_limit = curr_vel;
  }
  const std::vector<double> arc_s = computeCumulativeArcLengths(path);
  for (size_t i = 1; i < path.size(); i++) {
    // 当前路径点距离起点的沿路弧长。
    const double s_t = arc_s[i];
    if (s_t > accel_path_length) {
      break;
    }
    // 按当前速度 + 加速度，到达该点所需时间，再求速度。
    double t = (-curr_vel + std::sqrt(curr_vel * curr_vel + 2 * accel_ * s_t)) / accel_;
    double v = curr_vel + accel_ * t;
    if (v > speed_limit) {
      v = speed_limit;
    }
    // 只降不升：保留曲率/障碍等已有更严约束。
    path[i].speed_limit = std::min(path[i].speed_limit, v);
  }
}

// ============================ 曲率约束 ============================

void SpeedPlannerBase::applyCurvatureConstraint(
  std::vector<TrajPointWork> & path, const double ex_speed_limit)
{
  if (path.size() < 3) {
    return;
  }

  size_t idx_dist = static_cast<size_t>(std::round(curve_compute_dist_ / points_interval_));
  const std::vector<double> curvature_vec = calcTrajectoryCurvatureFrom3Points(path, idx_dist);

  std::vector<TrajPointWork> & output = path;

  // 对路径上每个点：写入曲率，并按小半径处限速。
  std::vector<int> filtered_points;
  for (size_t i = 0; i < output.size(); ++i) {
    output.at(i).curvature = curvature_vec.at(i);
    const double radius = std::fabs(1.0 / curvature_vec.at(i));
    const double & min_rad = regulated_linear_scaling_min_radius_;
    if (radius > min_rad) {
      continue;
    }
    double v_curvature_max = ex_speed_limit * (1.0 - (std::fabs(radius - min_rad) / min_rad));

    // 如果该点速度已经小于阈值，则不再要求其大于阈值。
    v_curvature_max = std::max(v_curvature_max, min_curvature_speed_);
    if (output.at(i).speed_limit < v_curvature_max) {
      continue;
    }

    // 将速度限幅在 v_curvature_max 之下。
    if (output.at(i).speed_limit > v_curvature_max) {
      output.at(i).speed_limit = v_curvature_max;
      filtered_points.push_back(static_cast<int>(i));
    }
  }

  // 转弯保持匀速：把相近的限速点聚成区间，区间内统一取最小速度。
  const double dist_threshold = constant_velocity_dist_threshold_;
  std::vector<std::tuple<size_t, size_t, double>> latacc_filtered_ranges;
  size_t start_index = 0;
  size_t end_index = 0;
  bool is_updated = false;
  double min_latacc_velocity = 0.0;
  for (size_t i = 0; i < filtered_points.size(); ++i) {
    const size_t index = static_cast<size_t>(filtered_points.at(i));

    if (is_updated == false) {
      start_index = index;
      end_index = index;
      min_latacc_velocity = output.at(index).speed_limit;
      is_updated = true;
      continue;
    }
    // 将 start_index 后 dist_threshold 内的点归入同一区间。
    double seg_dist = std::sqrt(
      std::pow(
        output.at(end_index).pose.pose.position.x - output.at(index).pose.pose.position.x, 2) +
      std::pow(
        output.at(end_index).pose.pose.position.y - output.at(index).pose.pose.position.y, 2));
    if (seg_dist < dist_threshold) {
      end_index = index;
      min_latacc_velocity = std::min(
        static_cast<double>(output.at(index).speed_limit), min_latacc_velocity);
    } else {
      latacc_filtered_ranges.emplace_back(start_index, end_index, min_latacc_velocity);
      start_index = index;
      end_index = index;
      min_latacc_velocity = output.at(index).speed_limit;
    }
  }
  if (is_updated) {
    latacc_filtered_ranges.emplace_back(start_index, end_index, min_latacc_velocity);
  }

  for (size_t i = 0; i < output.size(); ++i) {
    for (const auto & lat_acc_filtered_range : latacc_filtered_ranges) {
      const size_t s_idx = std::get<0>(lat_acc_filtered_range);
      const size_t e_idx = std::get<1>(lat_acc_filtered_range);
      const double min_v = std::get<2>(lat_acc_filtered_range);

      if (s_idx <= i && i <= e_idx && enable_constant_velocity_while_turning_) {
        // 只降不升：区间内安全置零点/更严约束不得被抬升到 min_v。
        output.at(i).speed_limit = std::min(output.at(i).speed_limit, min_v);
        break;
      }
    }
  }
}

// ============================ 减速段 ============================

void SpeedPlannerBase::computeDecelPlan(
  std::vector<TrajPointWork> & path, const double curr_vel, const double speed_limit)
{
  // 减速路径计算从后往前。

  const std::vector<double> arc_s = computeCumulativeArcLengths(path);

  // 末端速度：end_speed_ >= 1e-3 时末端 stop_distance_ 内保持 end_speed_ 蠕行；
  // end_speed_ < 1e-3 时无蠕行区，减速曲线在终点收敛到 0。写入只降不升。
  const double v_end = std::max(end_speed_, 0.0);
  if (stop_distance_ > 0 || v_end < 1e-3) {
    size_t first_no_stop_index = path.size() - 1;
    if (v_end >= 1e-3 && stop_distance_ > 0) {
      for (size_t i = path.size() - 1; i > 0; i--) {
        const double s_t = arc_s.back() - arc_s[i];
        if (s_t > stop_distance_) {
          first_no_stop_index = std::min(i + 1, path.size() - 1);
          break;
        }
        path[i].speed_limit = std::min(path[i].speed_limit, v_end);
      }
    } else {
      path[first_no_stop_index].speed_limit =
        std::min(path[first_no_stop_index].speed_limit, v_end);
    }
    double stop_decel_path_length = computeDecelLength(curr_vel, v_end);
    // index 0 是机器人当前速度，不参与设置。
    for (size_t i = first_no_stop_index; i > 0; i--) {
      const double s_t = arc_s[first_no_stop_index] - arc_s[i];
      double v = std::sqrt(v_end * v_end - 2 * decel_ * s_t);
      if (v > speed_limit) {
        v = speed_limit;
      }
      path[i].speed_limit = std::min(path[i].speed_limit, v);
      if (s_t > stop_decel_path_length) {
        break;
      }
    }
  }

  // 曲线前减速：以第一个小半径点作为目标速度点，向前减速（曲率已由 Phase A3 写入）。
  if ((curr_path_length_ - stop_distance_) > 0) {
    size_t first_curve_decel_index = path.size() - 1;
    bool has_curve_decel = false;
    for (size_t i = 0; i < path.size(); i++) {
      const double radius = std::fabs(1.0 / path[i].curvature);
      const double & min_rad = regulated_linear_scaling_min_radius_;
      if (radius > min_rad) {
        continue;
      }
      has_curve_decel = true;
      first_curve_decel_index = i;
      break;
    }
    if (has_curve_decel) {
      double curr_min_speed = min_curvature_speed_;
      double curve_decel_path_length = computeDecelLength(curr_vel, curr_min_speed);
      for (int i = static_cast<int>(first_curve_decel_index); i > 0; i--) {
        const double s_t = arc_s[first_curve_decel_index] - arc_s[i];
        double v = std::sqrt(curr_min_speed * curr_min_speed - 2 * decel_ * s_t);
        if (v > speed_limit) {
          v = speed_limit;
        }
        path[i].speed_limit = std::min(path[i].speed_limit, v);  // 只降不升
        if (s_t > curve_decel_path_length) {
          break;
        }
      }
    }
  }

  // 零速点前减速过渡：障碍与缓冲带已由 A1 置零，此处只做从首个零速点向前的减速曲线；
  // 末端硬停的零点复用同一原语（min 语义下与停止段结果一致）。
  size_t first_zero_index = path.size();
  for (size_t i = 1; i < path.size(); ++i) {
    if (path[i].speed_limit < 1e-3) {
      first_zero_index = i;
      break;
    }
  }
  if (first_zero_index < path.size()) {
    double curr_min_speed = 0.0;
    double collision_decel_path_length = computeDecelLength(curr_vel, curr_min_speed);

    for (size_t i = first_zero_index; i > 0; i--) {
      const double s_t = arc_s[first_zero_index] - arc_s[i];
      double v = std::sqrt(curr_min_speed * curr_min_speed - 2 * decel_ * s_t);
      if (v > speed_limit) {
        v = speed_limit;
      }
      path[i].speed_limit = std::min(path[i].speed_limit, v);  // 只降不升
      if (s_t > collision_decel_path_length) {
        break;
      }
    }
  }
}

// ============================ 加减速距离 ============================

double SpeedPlannerBase::computeAccelLength(const double curr_vel, const double speed_limit) const
{
  if (curr_vel >= speed_limit) {  // 不需要加速
    return 0.0;
  }
  double t = (speed_limit - curr_vel) / accel_;
  double s = curr_vel * t + 0.5 * accel_ * t * t;
  return s;
}

double SpeedPlannerBase::computeDecelLength(const double curr_vel, const double min_speed) const
{
  if (curr_vel <= min_speed) {  // 不需要减速
    return 0.0;
  }
  double t = (min_speed - curr_vel) / decel_;
  double s = curr_vel * t + 0.5 * decel_ * t * t;
  return s;
}

// ============================ 共享工具（移植自 dcm nav_util）============================

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

std::vector<double> SpeedPlannerBase::calcTrajectoryCurvatureFrom3Points(
  const std::vector<TrajPointWork> & path, size_t idx_dist)
{
  if (path.size() < 3) {
    return std::vector<double>(path.size(), 0.0);
  }

  // idx 不足时收缩 idx_dist。
  const auto max_idx_dist = static_cast<size_t>(std::floor((path.size() - 1) / 2.0));
  idx_dist = std::max(static_cast<size_t>(1), std::min(idx_dist, max_idx_dist));

  if (idx_dist < 1) {
    throw std::logic_error("idx_dist less than 1 is not expected");
  }

  // 三点圆拟合求曲率。
  std::vector<double> k_arr(path.size(), 0.0);
  for (size_t i = 1; i + 1 < path.size(); i++) {
    double curvature = 0.0;
    const auto p0 = path.at(i - std::min(idx_dist, i)).pose.pose;
    const auto p1 = path.at(i).pose.pose;
    const auto p2 = path.at(i + std::min(idx_dist, path.size() - 1 - i)).pose.pose;
    try {
      curvature = calcCurvature(p0, p1, p2);
    } catch (std::exception const & e) {
      if (clock_) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "%s", e.what());
      }
      if (i > 1) {
        curvature = k_arr.at(i - 1);  // 沿用前一点曲率
      } else {
        curvature = 0.0;
      }
    }
    k_arr.at(i) = curvature;
  }
  // 首尾点复制相邻点曲率。
  k_arr.at(0) = k_arr.at(1);
  k_arr.back() = k_arr.at(path.size() - 2);

  return k_arr;
}

double SpeedPlannerBase::calcCurvature(
  const geometry_msgs::msg::Pose & p1,
  const geometry_msgs::msg::Pose & p2,
  const geometry_msgs::msg::Pose & p3)
{
  // Menger 曲率：https://en.wikipedia.org/wiki/Menger_curvature
  auto dist = [](const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b) {
      double dx = a.position.x - b.position.x;
      double dy = a.position.y - b.position.y;
      return std::sqrt(dx * dx + dy * dy);
    };
  const double denominator = dist(p1, p2) * dist(p2, p3) * dist(p3, p1);
  if (std::fabs(denominator) < 1e-10) {
    throw std::runtime_error("points are too close for curvature calculation.");
  }
  return 2.0 *
         ((p2.position.x - p1.position.x) * (p3.position.y - p1.position.y) -
         (p2.position.y - p1.position.y) * (p3.position.x - p1.position.x)) / denominator;
}

bool SpeedPlannerBase::isObstacleAt(double x, double y) const
{
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_ ? costmap_ros_->getCostmap() : nullptr;
  if (costmap == nullptr) {
    return false;
  }
  unsigned int mx, my;
  if (!costmap->worldToMap(x, y, mx, my)) {
    return false;
  }
  return costmap->getCost(mx, my) >= static_cast<unsigned char>(obstacle_cost_threshold_);
}

}  // namespace nav2_speed_planner
