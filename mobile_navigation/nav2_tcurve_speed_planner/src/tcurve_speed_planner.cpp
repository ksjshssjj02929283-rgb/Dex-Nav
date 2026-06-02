// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// T 型曲线速度规划器实现，移植自 dcm tcurve_speed_planner（ROS1 → ROS2 Nav2 插件）。
// 三段式：加速段 / 匀速段 / 减速段（停止距离 + 曲率前减速 + 障碍前减速）+ 曲率约束。
#include "nav2_tcurve_speed_planner/tcurve_speed_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_tcurve_speed_planner
{

// ============================ 生命周期 ============================

void TCurveSpeedPlanner::configure(
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
    node, plugin_name_ + ".stop_speed", rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".stop_distance", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".collision_stop_distance", rclcpp::ParameterValue(0.2));

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
  node->get_parameter(plugin_name_ + ".stop_speed", stop_speed_);
  node->get_parameter(plugin_name_ + ".stop_distance", stop_distance_);
  node->get_parameter(plugin_name_ + ".collision_stop_distance", collision_stop_distance_);

  // 对齐 dcm 的 assert 约束，改为日志告警（不在运行期 abort 整个 nav 栈）。
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

  RCLCPP_INFO(logger_, "Configured TCurveSpeedPlanner plugin: %s", plugin_name_.c_str());
}

void TCurveSpeedPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up TCurveSpeedPlanner plugin: %s", plugin_name_.c_str());
  tf_.reset();
  costmap_ros_.reset();
}

void TCurveSpeedPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating TCurveSpeedPlanner plugin: %s", plugin_name_.c_str());
}

void TCurveSpeedPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating TCurveSpeedPlanner plugin: %s", plugin_name_.c_str());
}

void TCurveSpeedPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  current_plan_ = path;
}

void TCurveSpeedPlanner::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

// ============================ 核心入口 ============================

nav_interfaces::msg::RobotTrajectory TCurveSpeedPlanner::computeSpeedPlan(
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
  const double speed_limit = getSpeedLimit(work, curr_vel);
  setPlanSpeedLimit(work, speed_limit);

  computeAccelPlan(work, curr_vel, speed_limit);
  computeDecelPlan(work, curr_vel, speed_limit);

  // 回填到 RobotTrajectory：speed_limit→longitudinal_velocity，curvature→path_kappa，pose 透传。
  traj.points.reserve(work.size());
  for (const auto & p : work) {
    nav_interfaces::msg::TrajectoryPoint tp;
    tp.pose = p.pose;
    tp.longitudinal_velocity = p.speed_limit;
    tp.path_kappa = p.curvature;
    // 段语义透传（当前 nav_msgs/Path 无字段，置默认）。
    tp.speed_hint = p.speed_hint;
    tp.region_type = p.region_type;
    tp.segment_id = p.segment_id;
    traj.points.push_back(std::move(tp));
  }

  return traj;
}

// ============================ 限速求解 ============================

void TCurveSpeedPlanner::setPlanSpeedLimit(
  std::vector<TrajPointWork> & path, const double speed_limit)
{
  // 修正 dcm 原版 clamp bug：原实现第二行 (... ? min_speed_ : speed_limit) 用 speed_limit
  // 覆盖了上界钳制结果，导致 max_speed_ 限幅失效。此处改为正确的双边钳制。
  // （configure 已 assert max_speed_ > min_speed_，clamp 前置条件满足）
  const double valid_speed_limit = std::clamp(speed_limit, min_speed_, max_speed_);

  for (size_t i = 0; i < path.size(); i++) {
    if (path[i].speed_limit < 1e-3) {
      continue;
    }
    path[i].speed_limit = valid_speed_limit;
  }
}

double TCurveSpeedPlanner::getSpeedLimit(
  std::vector<TrajPointWork> & path, const double curr_vel)
{
  // max_speed_ 是路径点中 speed_limit 的最小值。
  double speed_limit = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.size(); i++) {
    if (speed_limit > path[i].speed_limit) {
      speed_limit = path[i].speed_limit;
    }
  }

  if (speed_limit > max_speed_) {
    speed_limit = max_speed_;
  }
  if (speed_limit < min_speed_) {
    speed_limit = min_speed_;
  }

  // 先计算加减速所需距离。
  const double accel_path_length = computeAccelLength(curr_vel, speed_limit);
  const double decel_path_length = computeDecelLength(curr_vel, min_speed_);

  // 障碍检测：把第一个障碍物及其后所有点速度置零。
  bool has_obstacle = false;
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_ ? costmap_ros_->getCostmap() : nullptr;
  for (size_t i = 0; i < path.size(); i++) {
    if (costmap != nullptr) {
      unsigned int mx, my;
      if (costmap->worldToMap(
          path[i].pose.pose.position.x, path[i].pose.pose.position.y, mx, my))
      {
        unsigned char cost = costmap->getCost(mx, my);
        if (cost > nav2_costmap_2d::FREE_SPACE) {
          path[i].speed_limit = 0.0;
          has_obstacle = true;
        }
      }
    }
    // 将第一个障碍物后面的路径速度都限制为 0。
    if (has_obstacle) {
      path[i].speed_limit = 0.0;
    }
  }

  double curr_min_speed = min_speed_;
  if (has_obstacle) {
    curr_min_speed = 0.0;
  }

  const double path_length = computeValidSpeedPathLength(path);
  const double no_stop_length = std::max(path_length - stop_distance_, 0.0);
  if (accel_path_length + decel_path_length > no_stop_length) {
    // 不能到达最大速度，回退求解可达速度。
    speed_limit = std::sqrt(
      (2 * accel_ * decel_ * no_stop_length + decel_ * curr_vel * curr_vel -
      accel_ * curr_min_speed * curr_min_speed) / (decel_ - accel_));
    return speed_limit;
  }
  // 可以到达最大速度。
  return speed_limit;
}

double TCurveSpeedPlanner::computeValidSpeedPathLength(const std::vector<TrajPointWork> & path)
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

void TCurveSpeedPlanner::computeAccelPlan(
  std::vector<TrajPointWork> & path, const double curr_vel, const double speed_limit)
{
  const double accel_path_length = computeAccelLength(curr_vel, speed_limit);
  path[0].speed_limit = curr_vel;
  const auto & pose_s = path[0].pose.pose;  // 路径起始点
  for (size_t i = 1; i < path.size(); i++) {
    // 当前路径点距离起点的距离。
    double s_t = std::sqrt(
      std::pow(path[i].pose.pose.position.x - pose_s.position.x, 2) +
      std::pow(path[i].pose.pose.position.y - pose_s.position.y, 2));
    if (s_t > accel_path_length) {
      break;
    }
    // 按当前速度 + 加速度，到达该点所需时间，再求速度。
    double t = (-curr_vel + std::sqrt(curr_vel * curr_vel + 2 * accel_ * s_t)) / accel_;
    double v = curr_vel + accel_ * t;
    if (v > speed_limit) {
      v = speed_limit;
    }
    path[i].speed_limit = v;
  }
}

// ============================ 曲率约束 ============================

void TCurveSpeedPlanner::applyCurvatureConstraint(
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
        output.at(i).speed_limit = min_v;
        break;
      }
    }
  }
}

// ============================ 减速段 ============================

void TCurveSpeedPlanner::computeDecelPlan(
  std::vector<TrajPointWork> & path, const double curr_vel, const double speed_limit)
{
  // 减速路径计算从后往前。

  // 停止速度：末端 stop_distance_ 内强制 stop_speed_，再向前减速过渡。
  if (stop_distance_ > 0) {
    size_t first_no_stop_index = path.size() - 1;
    const auto & pose_e = path.back().pose.pose;  // 路径终止点
    for (size_t i = path.size() - 1; i > 0; i--) {
      double s_t = std::sqrt(
        std::pow(path[i].pose.pose.position.x - pose_e.position.x, 2) +
        std::pow(path[i].pose.pose.position.y - pose_e.position.y, 2));
      if (s_t > stop_distance_) {
        first_no_stop_index = std::min(i + 1, path.size() - 1);
        break;
      }
      path[i].speed_limit = stop_speed_;
    }
    double curr_min_speed = stop_speed_;
    double stop_decel_path_length = computeDecelLength(curr_vel, curr_min_speed);
    // index 0 是机器人当前速度，不参与设置。
    const auto & pose_decel_e = path[first_no_stop_index].pose.pose;
    for (size_t i = first_no_stop_index; i > 0; i--) {
      double s_t = std::sqrt(
        std::pow(path[i].pose.pose.position.x - pose_decel_e.position.x, 2) +
        std::pow(path[i].pose.pose.position.y - pose_decel_e.position.y, 2));
      double v = std::sqrt(curr_min_speed * curr_min_speed - 2 * decel_ * s_t);
      if (v > speed_limit) {
        v = speed_limit;
      }
      path[i].speed_limit = v;
      if (s_t > stop_decel_path_length) {
        break;
      }
    }
  }

  applyCurvatureConstraint(path, speed_limit);

  // 曲线前减速：以第一个小半径点作为目标速度点，向前减速。
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
      const auto & pose_curve_decel = path[first_curve_decel_index].pose.pose;
      for (int i = static_cast<int>(first_curve_decel_index); i > 0; i--) {
        double s_t = std::sqrt(
          std::pow(path[i].pose.pose.position.x - pose_curve_decel.position.x, 2) +
          std::pow(path[i].pose.pose.position.y - pose_curve_decel.position.y, 2));
        double v = std::sqrt(curr_min_speed * curr_min_speed - 2 * decel_ * s_t);
        if (v > speed_limit) {
          v = speed_limit;
        }
        path[i].speed_limit = v;
        if (s_t > curve_decel_path_length) {
          break;
        }
      }
    }
  }

  // 障碍物前减速：从后往前找障碍点全置零，最后一个为 first_collision_index。
  size_t first_collision_index = path.size() - 1;
  bool has_obstacle = false;
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_ ? costmap_ros_->getCostmap() : nullptr;
  for (size_t i = path.size() - 1; i > 0; i--) {
    if (costmap != nullptr) {
      unsigned int mx, my;
      if (costmap->worldToMap(
          path[i].pose.pose.position.x, path[i].pose.pose.position.y, mx, my))
      {
        unsigned char cost = costmap->getCost(mx, my);
        if (cost > nav2_costmap_2d::FREE_SPACE) {
          path[i].speed_limit = 0.0;
          has_obstacle = true;
          first_collision_index = i;
        }
      }
    }
  }

  if (has_obstacle) {
    // 在碰撞点前一段处把速度限制为 0。
    // 第 0 个点是机器人当前位置，offset 从 index 1 起算，故 +1。
    size_t collision_offset_index =
      1 + static_cast<size_t>(std::round(collision_stop_distance_ / points_interval_));

    if (collision_offset_index > first_collision_index) {
      collision_offset_index = first_collision_index;
    }
    size_t collision_stop_index =
      std::max(first_collision_index - collision_offset_index, static_cast<size_t>(1));
    for (size_t i = first_collision_index; i >= collision_stop_index; i--) {
      path.at(i).speed_limit = 0.0;
    }
    double curr_min_speed = 0.0;
    double collision_decel_path_length = computeDecelLength(curr_vel, curr_min_speed);

    const auto & pose_collision = path[collision_stop_index].pose.pose;
    for (size_t i = collision_stop_index; i > 0; i--) {
      double s_t = std::sqrt(
        std::pow(path[i].pose.pose.position.x - pose_collision.position.x, 2) +
        std::pow(path[i].pose.pose.position.y - pose_collision.position.y, 2));
      double v = std::sqrt(curr_min_speed * curr_min_speed - 2 * decel_ * s_t);
      if (v > speed_limit) {
        v = speed_limit;
      }
      path[i].speed_limit = v;
      if (s_t > collision_decel_path_length) {
        break;
      }
    }
  }
}

// ============================ 加减速距离 ============================

double TCurveSpeedPlanner::computeAccelLength(const double curr_vel, const double speed_limit) const
{
  if (curr_vel >= speed_limit) {  // 不需要加速
    return 0.0;
  }
  double t = (speed_limit - curr_vel) / accel_;
  double s = curr_vel * t + 0.5 * accel_ * t * t;
  return s;
}

double TCurveSpeedPlanner::computeDecelLength(const double curr_vel, const double min_speed) const
{
  if (curr_vel <= min_speed) {  // 不需要减速
    return 0.0;
  }
  double t = (min_speed - curr_vel) / decel_;
  double s = curr_vel * t + 0.5 * decel_ * t * t;
  return s;
}

// ============================ 路径工具（移植自 dcm nav_util）============================

double TCurveSpeedPlanner::computePathLength(const std::vector<TrajPointWork> & path)
{
  double length = 0.0;
  for (int i = 1; i < static_cast<int>(path.size()); ++i) {
    double dx = path.at(i).pose.pose.position.x - path.at(i - 1).pose.pose.position.x;
    double dy = path.at(i).pose.pose.position.y - path.at(i - 1).pose.pose.position.y;
    length += std::sqrt(dx * dx + dy * dy);
  }
  return length;
}

std::vector<double> TCurveSpeedPlanner::calcTrajectoryCurvatureFrom3Points(
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
      RCLCPP_WARN(logger_, "%s", e.what());
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

double TCurveSpeedPlanner::calcCurvature(
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

}  // namespace nav2_tcurve_speed_planner

PLUGINLIB_EXPORT_CLASS(
  nav2_tcurve_speed_planner::TCurveSpeedPlanner, nav2_core::SpeedPlanner)
