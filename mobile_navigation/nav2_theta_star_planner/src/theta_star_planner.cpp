// Copyright 2020 Anshumaan Singh
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <filesystem>
#include <vector>
#include <memory>
#include <string>
#include <algorithm>
#include <cctype>
#include <cmath>
#include "nav2_theta_star_planner/theta_star_planner.hpp"
#include "nav2_theta_star_planner/theta_star.hpp"
#include "nav2_map_server/node_map_io.hpp"
#include "tf2/utils.h"

namespace nav2_theta_star_planner
{
namespace
{
constexpr size_t kMaxInterpolatedPoses = 50000;
constexpr double kEndpointPositionEpsilon = 1e-6;

nav_msgs::msg::Path coordsToPath(const std::vector<coordsW> & pts)
{
  nav_msgs::msg::Path pa;
  pa.poses.reserve(pts.size());
  for (const auto & pt : pts) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = pt.x;
    ps.pose.position.y = pt.y;
    pa.poses.push_back(ps);
  }
  return pa;
}

std::string toLowerCopy(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
  return value;
}

double shortestAngularDistance(double from, double to)
{
  double delta = to - from;
  while (delta > M_PI) {
    delta -= 2.0 * M_PI;
  }
  while (delta < -M_PI) {
    delta += 2.0 * M_PI;
  }
  return delta;
}

std::vector<double> computeCumulativePathLength(const nav_msgs::msg::Path & path)
{
  const size_t n = path.poses.size();
  std::vector<double> cum(n, 0.0);
  for (size_t i = 1; i < n; ++i) {
    const auto & p0 = path.poses[i - 1].pose.position;
    const auto & p1 = path.poses[i].pose.position;
    cum[i] = cum[i - 1] + std::hypot(p1.x - p0.x, p1.y - p0.y);
  }
  return cum;
}
}  // namespace

PathOrientationMode ThetaStarPlanner::parsePathOrientationMode(const std::string & mode)
{
  const std::string normalized = toLowerCopy(mode);
  if (normalized == "none") {
    return PathOrientationMode::NONE;
  }
  if (normalized == "start") {
    return PathOrientationMode::START;
  }
  if (normalized == "goal") {
    return PathOrientationMode::GOAL;
  }
  if (normalized == "tangent") {
    return PathOrientationMode::TANGENT;
  }
  // 默认 auto：起终点同 yaw 平移；不同则沿路径弧长插值
  return PathOrientationMode::AUTO;
}

SmoothPathMode ThetaStarPlanner::parseSmoothPathMode(const std::string & mode)
{
  const std::string normalized = toLowerCopy(mode);
  if (normalized == "l1_spline" || normalized == "l1" || normalized == "spline") {
    return SmoothPathMode::L1_SPLINE;
  }
  return SmoothPathMode::LEGACY;
}

void ThetaStarPlanner::assignPathOrientations(
  nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  const size_t plan_size = path.poses.size();
  if (plan_size == 0 || path_orientation_mode_ == PathOrientationMode::NONE) {
    return;
  }

  if (path_orientation_mode_ == PathOrientationMode::AUTO) {
    const double yaw_start = tf2::getYaw(start.pose.orientation);
    const double yaw_goal = tf2::getYaw(goal.pose.orientation);
    const double yaw_delta = shortestAngularDistance(yaw_start, yaw_goal);

    // 起终点朝向一致：全程同一 yaw，全向纯平移
    if (std::fabs(yaw_delta) < path_orientation_same_threshold_rad_) {
      const auto fixed_orientation =
        nav2_util::geometry_utils::orientationAroundZAxis(yaw_start);
      for (auto & pose_stamped : path.poses) {
        pose_stamped.pose.orientation = fixed_orientation;
      }
      return;
    }

    // 起终点朝向不同：按已走路径弧长比例，从 start_yaw 平滑过渡到 goal_yaw
    const std::vector<double> cum_dist = computeCumulativePathLength(path);
    const double total_length = cum_dist.back();

    for (size_t i = 0; i < plan_size; ++i) {
      double progress = 1.0;
      if (plan_size > 1 && total_length > 1e-6) {
        progress = cum_dist[i] / total_length;
      }
      const double yaw = yaw_start + progress * yaw_delta;
      path.poses[i].pose.orientation =
        nav2_util::geometry_utils::orientationAroundZAxis(yaw);
    }
    return;
  }

  if (path_orientation_mode_ == PathOrientationMode::START ||
    path_orientation_mode_ == PathOrientationMode::GOAL)
  {
    const auto & cruise_orientation =
      (path_orientation_mode_ == PathOrientationMode::START) ?
      start.pose.orientation :
      goal.pose.orientation;

    for (size_t i = 0; i < plan_size; ++i) {
      path.poses[i].pose.orientation = cruise_orientation;
    }
    return;
  }

  // TANGENT：通常使用相邻点切线作为 yaw。进入终点保持区后冻结为“来向 yaw”，
  // 避免 5 cm 栅格量化产生的最后一条极短斜线突然改变参考朝向。若整条新路径都比
  // 保持距离短（终点附近重规划的典型情况），直接沿用当前机器人 yaw；全向底盘仍可
  // 横移到精确 XY，最终 goal yaw 继续由末尾零位移 marker 独立处理。
  const std::vector<double> cum_dist = computeCumulativePathLength(path);
  const double total_length = cum_dist.back();
  const double hold_distance = std::max(0.0, tangent_terminal_yaw_hold_distance_);
  const double hold_start_distance = std::max(0.0, total_length - hold_distance);
  double terminal_hold_yaw = tf2::getYaw(start.pose.orientation);

  if (plan_size > 1 && total_length > hold_distance && hold_distance > 0.0) {
    // 找到跨过“终点保持区起点”的有效线段，把它的方向作为终端来向。
    for (size_t i = 0; i + 1 < plan_size; ++i) {
      if (cum_dist[i + 1] + kEndpointPositionEpsilon < hold_start_distance) {
        continue;
      }
      const auto & p0 = path.poses[i].pose.position;
      const auto & p1 = path.poses[i + 1].pose.position;
      if (std::hypot(p1.x - p0.x, p1.y - p0.y) > kEndpointPositionEpsilon) {
        terminal_hold_yaw = std::atan2(p1.y - p0.y, p1.x - p0.x);
        break;
      }
    }
  }

  for (size_t i = 0; i + 1 < plan_size; ++i) {
    const auto & p0 = path.poses[i].pose.position;
    const auto & p1 = path.poses[i + 1].pose.position;
    const double dx = p1.x - p0.x;
    const double dy = p1.y - p0.y;
    if (std::hypot(dx, dy) < 1e-6) {
      if (i > 0) {
        path.poses[i].pose.orientation = path.poses[i - 1].pose.orientation;
      }
      continue;
    }
    const bool in_terminal_hold =
      hold_distance > 0.0 && cum_dist[i] + kEndpointPositionEpsilon >= hold_start_distance;
    const double theta = in_terminal_hold ? terminal_hold_yaw : std::atan2(dy, dx);
    path.poses[i].pose.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(theta);
  }

  if (plan_size == 1) {
    path.poses[0].pose.orientation = start.pose.orientation;
  } else {
    path.poses.back().pose.orientation = path.poses[plan_size - 2].pose.orientation;
  }
}

void ThetaStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  planner_ = std::make_unique<theta_star::ThetaStar>();
  multi_layer_planner_ = std::make_unique<theta_star::MultiLayerNodePlanner>();
  parent_node_ = parent;
  auto node = parent_node_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  name_ = name;
  tf_ = tf;
  planner_->costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".how_many_corners", rclcpp::ParameterValue(8));

  node->get_parameter(name_ + ".how_many_corners", planner_->how_many_corners_);

  if (planner_->how_many_corners_ != 8 && planner_->how_many_corners_ != 4) {
    planner_->how_many_corners_ = 8;
    RCLCPP_WARN(logger_, "Your value for - .how_many_corners  was overridden, and is now set to 8");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".allow_unknown", planner_->allow_unknown_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".w_euc_cost", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + ".w_euc_cost", planner_->w_euc_cost_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".w_traversal_cost", rclcpp::ParameterValue(2.0));
  node->get_parameter(name_ + ".w_traversal_cost", planner_->w_traversal_cost_);

  planner_->w_heuristic_cost_ = planner_->w_euc_cost_ < 1.0 ? planner_->w_euc_cost_ : 1.0;

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".use_final_approach_orientation", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".use_final_approach_orientation", use_final_approach_orientation_);

  // ── 路径点 orientation 后处理 ────────────────────────────────────────────
  // mode=auto（默认）：
  //   - 起终点 yaw 相同 → 全程固定该 yaw（全向纯平移）
  //   - 起终点 yaw 不同 → 沿路径弧长从 start_yaw 线性插值到 goal_yaw
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".assign_path_orientations", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".assign_path_orientations", assign_path_orientations_);

  std::string path_orientation_mode_param{"auto"};
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".path_orientation_mode", rclcpp::ParameterValue(path_orientation_mode_param));
  node->get_parameter(name + ".path_orientation_mode", path_orientation_mode_param);
  path_orientation_mode_ = parsePathOrientationMode(path_orientation_mode_param);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".path_orientation_same_threshold_rad", rclcpp::ParameterValue(0.02));
  node->get_parameter(
    name + ".path_orientation_same_threshold_rad", path_orientation_same_threshold_rad_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".tangent_in_place_rotation_markers", rclcpp::ParameterValue(false));
  node->get_parameter(
    name + ".tangent_in_place_rotation_markers", tangent_in_place_rotation_markers_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".tangent_terminal_yaw_hold_distance", rclcpp::ParameterValue(0.20));
  node->get_parameter(
    name + ".tangent_terminal_yaw_hold_distance", tangent_terminal_yaw_hold_distance_);
  tangent_terminal_yaw_hold_distance_ = std::max(0.0, tangent_terminal_yaw_hold_distance_);

  if (!assign_path_orientations_) {
    path_orientation_mode_ = PathOrientationMode::NONE;
  }

  // ── B 层：舵轮连续性后处理（对齐 mobile_cmd_omni_region.cpp Two-Phase 逻辑）──
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".enforce_steering_continuity", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".enforce_steering_continuity", enforce_steering_continuity_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".steering_chassis_wheelbase", rclcpp::ParameterValue(0.46));
  node->get_parameter(name + ".steering_chassis_wheelbase", swerve_steering_config_.wheelbase);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".steering_chassis_track", rclcpp::ParameterValue(0.345));
  node->get_parameter(name + ".steering_chassis_track", swerve_steering_config_.track);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".steering_limit_rad", rclcpp::ParameterValue(M_PI_2));
  node->get_parameter(name + ".steering_limit_rad", swerve_steering_config_.steer_limit);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".steering_max_wheel_angle_step", rclcpp::ParameterValue(1.0));
  node->get_parameter(
    name + ".steering_max_wheel_angle_step", swerve_steering_config_.max_wheel_angle_step);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".steering_nominal_path_speed", rclcpp::ParameterValue(0.5));
  node->get_parameter(
    name + ".steering_nominal_path_speed", swerve_steering_config_.nominal_path_speed);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".steering_max_body_yaw_rate", rclcpp::ParameterValue(1.2));
  node->get_parameter(
    name + ".steering_max_body_yaw_rate", swerve_steering_config_.max_body_yaw_rate);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".steering_max_smoothing_passes", rclcpp::ParameterValue(3));
  node->get_parameter(
    name + ".steering_max_smoothing_passes", swerve_steering_config_.max_smoothing_passes);

  // ── 全向底盘几何路径平滑（无最小转弯半径约束）────────────────────────────
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_path", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".smooth_path", smooth_path_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_do_string_pull", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".smooth_do_string_pull", smooth_do_string_pull_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_do_gradient", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".smooth_do_gradient", smooth_do_gradient_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_max_iterations", rclcpp::ParameterValue(1000));
  node->get_parameter(name + ".smooth_max_iterations", path_smooth_config_.max_iterations);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_w_data", rclcpp::ParameterValue(0.2));
  node->get_parameter(name + ".smooth_w_data", path_smooth_config_.w_data);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_w_smooth", rclcpp::ParameterValue(0.3));
  node->get_parameter(name + ".smooth_w_smooth", path_smooth_config_.w_smooth);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_tolerance", rclcpp::ParameterValue(1e-10));
  node->get_parameter(name + ".smooth_tolerance", path_smooth_config_.tolerance);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_do_refinement", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".smooth_do_refinement", path_smooth_config_.do_refinement);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_refinement_num", rclcpp::ParameterValue(2));
  node->get_parameter(name + ".smooth_refinement_num", path_smooth_config_.refinement_num);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_string_pull_corner_angle_rad", rclcpp::ParameterValue(0.17));
  node->get_parameter(
    name + ".smooth_string_pull_corner_angle_rad",
    path_smooth_config_.string_pull_corner_angle_rad);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_do_curve_fit", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".smooth_do_curve_fit", smooth_do_curve_fit_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_log_timing", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".smooth_log_timing", smooth_log_timing_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_curve_sample_step_m", rclcpp::ParameterValue(0.05));
  node->get_parameter(name + ".smooth_curve_sample_step_m", curve_path_config_.sample_step_m);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_curve_fillet_max_leg_fraction", rclcpp::ParameterValue(0.35));
  node->get_parameter(
    name + ".smooth_curve_fillet_max_leg_fraction",
    curve_path_config_.fillet_max_leg_fraction);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_path_mode", rclcpp::ParameterValue(std::string("legacy")));
  std::string smooth_path_mode_param;
  node->get_parameter(name + ".smooth_path_mode", smooth_path_mode_param);
  smooth_path_mode_ = parseSmoothPathMode(smooth_path_mode_param);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_l1_clearance_radius_factor", rclcpp::ParameterValue(0.85));
  node->get_parameter(
    name + ".smooth_l1_clearance_radius_factor", smooth_l1_clearance_radius_factor_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_l1_corner_angle_rad", rclcpp::ParameterValue(0.25));
  node->get_parameter(name + ".smooth_l1_corner_angle_rad", smooth_l1_corner_angle_rad_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_l1_simplify_angle_rad", rclcpp::ParameterValue(0.10));
  node->get_parameter(name + ".smooth_l1_simplify_angle_rad", smooth_l1_simplify_angle_rad_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_l1_do_gradient", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".smooth_l1_do_gradient", smooth_l1_do_gradient_);

  curve_path_config_.corner_angle_rad = path_smooth_config_.string_pull_corner_angle_rad;

  multi_layer_planner_->costmap_ = costmap_ros->getCostmap();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_node_map", rclcpp::ParameterValue(false));
  node->get_parameter(name_ + ".use_node_map", use_node_map_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".node_map_yaml", rclcpp::ParameterValue(std::string("")));
  node->get_parameter(name_ + ".node_map_yaml", node_map_yaml_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".node_map_source_yaml", rclcpp::ParameterValue(std::string("")));
  std::string node_map_source_yaml;
  node->get_parameter(name_ + ".node_map_source_yaml", node_map_source_yaml);
  node_map_source_yaml = nav2_map_server::expandUserPath(node_map_source_yaml);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".local_map_yaml", rclcpp::ParameterValue(std::string("")));
  node->get_parameter(name_ + ".local_map_yaml", local_map_yaml_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".raw_map_yaml", rclcpp::ParameterValue(std::string("")));
  node->get_parameter(name_ + ".raw_map_yaml", raw_map_yaml_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".distance_map_yaml", rclcpp::ParameterValue(std::string("")));
  node->get_parameter(name_ + ".distance_map_yaml", distance_map_yaml_);

  node_map_yaml_ = nav2_map_server::expandUserPath(node_map_yaml_);
  local_map_yaml_ = nav2_map_server::expandUserPath(local_map_yaml_);
  raw_map_yaml_ = nav2_map_server::expandUserPath(raw_map_yaml_);
  distance_map_yaml_ = nav2_map_server::expandUserPath(distance_map_yaml_);

  if (node_map_yaml_.empty() && !node_map_source_yaml.empty()) {
    node_map_yaml_ = nav2_map_server::defaultGlobalNodeMapPath(node_map_source_yaml);
    RCLCPP_INFO(
      logger_, "node_map_yaml derived from map: %s", node_map_yaml_.c_str());
  }

  if (use_node_map_) {
    node_map_loaded_ = loadNodeMapResources();
    if (node_map_loaded_) {
      RCLCPP_INFO(logger_, "Node map planning ENABLED: %s", node_map_yaml_.c_str());
    } else {
      RCLCPP_ERROR(
        logger_,
        "Node map planning requested but load failed — will fall back to grid Theta*");
    }
  }

  RCLCPP_INFO(
    logger_,
    "Theta* path orientation: assign=%s mode=%s same_threshold=%.3f rad "
    "tangent_in_place_markers=%s terminal_yaw_hold=%.3f m. "
    "Omni: PathAngleCritic off + PathAlign use_path_orientations=true",
    assign_path_orientations_ ? "true" : "false",
    path_orientation_mode_param.c_str(),
    path_orientation_same_threshold_rad_,
    tangent_in_place_rotation_markers_ ? "true" : "false",
    tangent_terminal_yaw_hold_distance_);

  RCLCPP_INFO(
    logger_,
    "Theta* steering continuity (B): enforce=%s wheelbase=%.3f track=%.3f "
    "max_wheel_step=%.2f rad nominal_v=%.2f max_yaw_rate=%.2f",
    enforce_steering_continuity_ ? "true" : "false",
    swerve_steering_config_.wheelbase,
    swerve_steering_config_.track,
    swerve_steering_config_.max_wheel_angle_step,
    swerve_steering_config_.nominal_path_speed,
    swerve_steering_config_.max_body_yaw_rate);
}

bool ThetaStarPlanner::loadNodeMapResources()
{
  if (node_map_yaml_.empty()) {
    RCLCPP_ERROR(logger_, "use_node_map=true but node_map_yaml is empty");
    return false;
  }

  try {
    const auto preprocessed_dir = std::filesystem::path(node_map_yaml_).parent_path();

    if (local_map_yaml_.empty()) {
      local_map_yaml_ = (preprocessed_dir / "nodemap_local.yaml").string();
    }
    if (raw_map_yaml_.empty()) {
      raw_map_yaml_ = (preprocessed_dir / "nodemap_raw.yaml").string();
    }

    nav2_map_server::NodeMap global_map;
    nav2_map_server::NodeMap local_map;
    nav2_map_server::NodeMap raw_map;

    if (!nav2_map_server::loadNodeMap(node_map_yaml_, global_map)) {
      RCLCPP_ERROR(logger_, "Failed to load global node map: %s", node_map_yaml_.c_str());
      return false;
    }
    if (!nav2_map_server::loadNodeMap(local_map_yaml_, local_map)) {
      RCLCPP_WARN(logger_, "Local node map missing: %s", local_map_yaml_.c_str());
    }

    std::string dist_yaml = distance_map_yaml_;
    if (dist_yaml.empty()) {
      dist_yaml = (preprocessed_dir / "distance_map.yaml").string();
    }

    nav2_map_server::DistanceMap distance_map;
    if (!nav2_map_server::loadDistanceMap(dist_yaml, distance_map)) {
      RCLCPP_WARN(
        logger_,
        "Distance map missing (%s); node LOS clearance checks disabled",
        dist_yaml.c_str());
      distance_map = nav2_map_server::DistanceMap{};
    }

    multi_layer_planner_->global_map_ = std::move(global_map);
    multi_layer_planner_->local_map_ = std::move(local_map);
    multi_layer_planner_->raw_map_ = std::move(raw_map);
    multi_layer_planner_->distance_map_ = std::move(distance_map);
    multi_layer_planner_->costmap_ = planner_->costmap_;
    multi_layer_planner_->w_euc_cost_ = planner_->w_euc_cost_;
    multi_layer_planner_->w_traversal_cost_ = planner_->w_traversal_cost_;
    multi_layer_planner_->w_heuristic_cost_ = planner_->w_heuristic_cost_;
    multi_layer_planner_->allow_unknown_ = planner_->allow_unknown_;

    RCLCPP_INFO(
      logger_,
      "Node map planner: L1 global=%zu nodes (L2 disabled); final leg <= %.1fm is direct line",
      multi_layer_planner_->global_map_.nodes.size(),
      theta_star::MultiLayerNodePlanner::kDirectFinalLegMaxDistM);
    return true;
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      logger_, "Exception loading node map resources: %s", ex.what());
    return false;
  }
}

void ThetaStarPlanner::applySteeringContinuityConstraint(
  nav_msgs::msg::Path & path,
  double goal_yaw) const
{
  if (path.poses.size() < 2) {
    return;
  }

  const size_t adjusted = enforceSteeringContinuityOnPath(
    path, goal_yaw, swerve_steering_config_);

  RCLCPP_DEBUG(
    logger_,
    "Steering continuity post-process: adjusted %zu poses toward goal_yaw=%.3f",
    adjusted, goal_yaw);
}

void ThetaStarPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "CleaningUp plugin %s of type nav2_theta_star_planner", name_.c_str());
  planner_.reset();
  multi_layer_planner_.reset();
}

void ThetaStarPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s of type nav2_theta_star_planner", name_.c_str());
  RCLCPP_INFO(
    logger_,
    "Path post-process: smooth_path=%s mode=%s string_pull=%s gradient=%s fillet_geom=%s "
    "log_timing=%s [build=l1_spline_v3]",
    smooth_path_ ? "true" : "false",
    smooth_path_mode_ == SmoothPathMode::L1_SPLINE ? "l1_spline" : "legacy",
    smooth_do_string_pull_ ? "true" : "false",
    smooth_do_gradient_ ? "true" : "false",
    smooth_do_curve_fit_ ? "true" : "false",
    smooth_log_timing_ ? "true" : "false");
  if (use_node_map_ && !node_map_loaded_) {
    node_map_loaded_ = loadNodeMapResources();
    if (node_map_loaded_) {
      RCLCPP_INFO(logger_, "Node map loaded on activate: %s", node_map_yaml_.c_str());
    }
  }
  // Add callback for dynamic parameters
  auto node = parent_node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&ThetaStarPlanner::dynamicParametersCallback, this, std::placeholders::_1));
}

void ThetaStarPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type nav2_theta_star_planner", name_.c_str());
}

nav_msgs::msg::Path ThetaStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;
  auto start_time = std::chrono::steady_clock::now();

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(planner_->costmap_->getMutex()));

  // 起终点先转换到栅格，仅把栅格用于 Theta* 搜索；用户的连续坐标会在搜索后接回。
  unsigned int mx_start, my_start, mx_goal, my_goal;
  const bool start_in_map = planner_->costmap_->worldToMap(
    start.pose.position.x, start.pose.position.y, mx_start, my_start);
  const bool goal_in_map = planner_->costmap_->worldToMap(
    goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal);
  if (!start_in_map || !goal_in_map) {
    RCLCPP_WARN(
      logger_, "Failed to create path because start or goal is outside the costmap");
    return global_path;
  }

  // 同一栅格不再退化成一个栅格中心/起点：只要 XY 不同，就直接输出经过安全检查的
  // 亚栅格短线 [精确 start, 精确 goal]，再走下方统一的 yaw 与 marker 后处理。
  if (mx_start == mx_goal && my_start == my_goal) {
    if (!planner_->isSafe(mx_start, my_start)) {
      RCLCPP_WARN(
        logger_,
        "Failed to create a unique pose path because the pose is not in free space");
      return global_path;
    }
    global_path.header.stamp = clock_->now();
    global_path.header.frame_id = global_frame_;
    if (!isWorldSegmentSafe(start.pose.position, goal.pose.position)) {
      RCLCPP_WARN(logger_, "The exact same-cell start-to-goal connection is not collision free");
      return global_path;
    }

    geometry_msgs::msg::PoseStamped exact_start = start;
    geometry_msgs::msg::PoseStamped exact_goal = goal;
    exact_start.header = global_path.header;
    exact_goal.header = global_path.header;
    exact_start.pose.position.z = 0.0;
    exact_goal.pose.position.z = 0.0;
    global_path.poses.push_back(exact_start);

    const double exact_distance = std::hypot(
      goal.pose.position.x - start.pose.position.x,
      goal.pose.position.y - start.pose.position.y);
    if (exact_distance > kEndpointPositionEpsilon) {
      global_path.poses.push_back(exact_goal);
    } else {
      // 真正同一点沿用原有模式协议：AUTO 等模式保持单点，TANGENT/START 保留各自
      // 的显式 marker，使“只改变目标 yaw”的任务仍能进入控制器的原地旋转阶段。
      if (path_orientation_mode_ == PathOrientationMode::TANGENT &&
        tangent_in_place_rotation_markers_)
      {
        geometry_msgs::msg::PoseStamped start_marker = exact_start;
        geometry_msgs::msg::PoseStamped cruise_start = exact_start;
        geometry_msgs::msg::PoseStamped cruise_goal = exact_start;
        geometry_msgs::msg::PoseStamped goal_marker = exact_start;
        goal_marker.pose.orientation = goal.pose.orientation;
        global_path.poses = {start_marker, cruise_start, cruise_goal, goal_marker};
      } else if (path_orientation_mode_ == PathOrientationMode::START) {
        geometry_msgs::msg::PoseStamped goal_pose = exact_start;
        goal_pose.pose.orientation = goal.pose.orientation;
        global_path.poses = {exact_start, exact_start, exact_start, goal_pose};
      } else {
        global_path.poses.front().pose.orientation = goal.pose.orientation;
      }
      return global_path;
    }
  } else {
    planner_->setStartAndGoal(start, goal);
    getPlan(global_path, start, goal);
  }

  const size_t plan_size = global_path.poses.size();
  if (plan_size > 0) {
    // 1) 为中间/全程点写入 orientation（可配置）
    if (assign_path_orientations_ &&
      path_orientation_mode_ != PathOrientationMode::NONE)
    {
      assignPathOrientations(global_path, start, goal);
    }

    // 2) 最后用户 goal yaw 与途中 yaw 策略严格分离。零位移 marker 同时作为
    // 控制模式契约：TANGENT=首尾各一个、GOAL=起点一个、START=起点两个、
    // NONE=终点一个、AUTO=无。START 必须显式编码，否则控制器无法把它和
    // 允许边走边转的 AUTO 区分开。
    // controller 会先识别并移除 marker，不把重复点交给 MPPI 跟踪。
    const double final_path_yaw = tf2::getYaw(goal.pose.orientation);

    // 3) B 层：沿路径仿真舵轮 IK，平滑 yaw 曲线，避免底盘 Two-Phase 停车转舵
    // TANGENT phase boundaries are intentional yaw discontinuities, so they
    // must not be spread back into the translating part of the path.
    if (enforce_steering_continuity_ &&
      assign_path_orientations_ &&
      path_orientation_mode_ != PathOrientationMode::NONE &&
      path_orientation_mode_ != PathOrientationMode::TANGENT &&
      path_orientation_mode_ != PathOrientationMode::START &&
      path_orientation_mode_ != PathOrientationMode::GOAL)
    {
      applySteeringContinuityConstraint(global_path, final_path_yaw);
    }

    const auto final_goal_orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(final_path_yaw);
    // START 的固定朝向语义必须始终显式编码，不能依赖 tangent 专用开关；否则
    // 关闭 tangent marker 时 START 会再次退化成无法识别的 AUTO。
    if ((tangent_in_place_rotation_markers_ ||
      path_orientation_mode_ == PathOrientationMode::START) && plan_size >= 2)
    {
      if (path_orientation_mode_ == PathOrientationMode::TANGENT) {
        geometry_msgs::msg::PoseStamped start_marker = global_path.poses.front();
        start_marker.pose.orientation = start.pose.orientation;
        global_path.poses.insert(global_path.poses.begin(), start_marker);

        geometry_msgs::msg::PoseStamped goal_marker = global_path.poses.back();
        goal_marker.pose.orientation = final_goal_orientation;
        global_path.poses.push_back(goal_marker);
      } else if (path_orientation_mode_ == PathOrientationMode::GOAL) {
        // 仅起点重复表示 GOAL：controller 先原地对齐 goal yaw，再保持该 yaw 平移。
        geometry_msgs::msg::PoseStamped goal_yaw_start_marker = global_path.poses.front();
        goal_yaw_start_marker.pose.orientation = final_goal_orientation;
        global_path.poses.insert(global_path.poses.begin(), goal_yaw_start_marker);
        global_path.poses.back().pose.orientation = final_goal_orientation;
      } else if (path_orientation_mode_ == PathOrientationMode::NONE) {
        // 仅终点重复表示 NONE：途中完全不使用路径 yaw，终点仍保留用户 goal yaw。
        geometry_msgs::msg::PoseStamped none_mode_goal_marker = global_path.poses.back();
        none_mode_goal_marker.pose.orientation = final_goal_orientation;
        global_path.poses.push_back(none_mode_goal_marker);
      } else if (path_orientation_mode_ == PathOrientationMode::START) {
        // 两个起点 marker 是 START 的显式协议。不能再与 AUTO 共用“无 marker”，
        // 否则 MPPI 无从判断当前任务是否允许产生角速度。
        geometry_msgs::msg::PoseStamped start_marker = global_path.poses.front();
        global_path.poses.insert(global_path.poses.begin(), start_marker);
        global_path.poses.insert(global_path.poses.begin(), start_marker);
        global_path.poses.back().pose.orientation = final_goal_orientation;
      } else {
        // AUTO 无 marker；最后 yaw 只供统一 FINAL_YAW 使用。
        global_path.poses.back().pose.orientation = final_goal_orientation;
      }
    } else {
      global_path.poses.back().pose.orientation = final_goal_orientation;
    }
  }

  auto stop_time = std::chrono::steady_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::microseconds>(stop_time - start_time);
  const double total_ms = dur.count() / 1000.0;
  RCLCPP_DEBUG(logger_, "the time taken is : %i", static_cast<int>(dur.count()));
  RCLCPP_DEBUG(logger_, "the nodes_opened are:  %i", planner_->nodes_opened);
  if (smooth_log_timing_ && plan_size > 0) {
    RCLCPP_INFO(
      logger_,
      "createPlan total=%.2f ms (includes search + post-process; "
      "see post-process timing for baseline vs curve breakdown)",
      total_ms);
  }
  if (global_path.poses.empty()) {
    RCLCPP_ERROR(
      logger_,
      "createPlan returned empty path (start=%.2f,%.2f goal=%.2f,%.2f)",
      start.pose.position.x, start.pose.position.y,
      goal.pose.position.x, goal.pose.position.y);
  }
  return global_path;
}

void ThetaStarPlanner::getPlan(
  nav_msgs::msg::Path & global_path,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  std::vector<coordsW> path;
  bool goal_unreachable = false;

  multi_layer_planner_->costmap_ = planner_->costmap_;
  if (use_node_map_ && !node_map_loaded_) {
    node_map_loaded_ = loadNodeMapResources();
  }

  const bool use_nodes = use_node_map_ && node_map_loaded_ && multi_layer_planner_->hasMaps();
  if (use_nodes) {
    multi_layer_planner_->w_euc_cost_ = planner_->w_euc_cost_;
    multi_layer_planner_->w_traversal_cost_ = planner_->w_traversal_cost_;
    multi_layer_planner_->w_heuristic_cost_ = planner_->w_heuristic_cost_;
    multi_layer_planner_->allow_unknown_ = planner_->allow_unknown_;

    try {
      if (multi_layer_planner_->generatePath(start, goal, path, goal_unreachable)) {
        if (goal_unreachable) {
          RCLCPP_WARN(
            logger_,
            "Goal lies in obstacle on raw layer; path ends at L1 sparse node");
        }
        RCLCPP_INFO(
          logger_,
          "Node map plan: %zu graph waypoints (before interpolation)",
          path.size());
        buildGeometricPath(global_path, path);
        // goal_unreachable=true 表示多层规划器有意停在替代节点，不能再强接原始障碍终点。
        if (!global_path.poses.empty() &&
          !connectPathToExactEndpoints(global_path, start, goal, !goal_unreachable))
        {
          RCLCPP_ERROR(logger_, "Could not safely connect node-map path to exact endpoints");
          global_path.poses.clear();
        }
        if (global_path.poses.empty() && !path.empty()) {
          RCLCPP_ERROR(
            logger_,
            "Node path interpolation produced empty plan from %zu waypoints",
            path.size());
        }
        return;
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        logger_, "3-layer node planning exception: %s — falling back to grid Theta*",
        ex.what());
      path.clear();
    }
    RCLCPP_WARN(
      logger_,
      "3-layer node Theta* could not find a path — falling back to grid Theta*");
    path.clear();
  } else if (use_node_map_ && !node_map_loaded_) {
    RCLCPP_WARN_ONCE(logger_, "Falling back to grid Theta* (node map not loaded)");
  }

  if (planner_->isUnsafeToPlan()) {
    RCLCPP_ERROR(logger_, "Either of the start or goal pose are an obstacle! ");
    global_path.poses.clear();
    return;
  }
  if (!planner_->generatePath(path)) {
    RCLCPP_ERROR(logger_, "Could not generate path between the given poses");
    global_path.poses.clear();
    return;
  }

  buildGeometricPath(global_path, path);
  if (!global_path.poses.empty() &&
    !connectPathToExactEndpoints(global_path, start, goal))
  {
    RCLCPP_ERROR(logger_, "Could not safely connect Theta* path to exact endpoints");
    global_path.poses.clear();
  }
}

bool ThetaStarPlanner::isWorldSegmentSafe(
  const geometry_msgs::msg::Point & from,
  const geometry_msgs::msg::Point & to) const
{
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double distance = std::hypot(dx, dy);

  // costmap 已包含机器人外形膨胀。用 1/4 栅格分辨率采样短连接，可覆盖连接经过的
  // 每个栅格；沿用 Theta*::isSafe 的判定，避免端点修正绕过规划器的障碍规则。
  const double sample_step = std::max(0.001, planner_->costmap_->getResolution() * 0.25);
  const size_t sample_count = std::max<size_t>(
    1, static_cast<size_t>(std::ceil(distance / sample_step)));
  for (size_t i = 0; i <= sample_count; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(sample_count);
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!planner_->costmap_->worldToMap(from.x + ratio * dx, from.y + ratio * dy, mx, my) ||
      !planner_->isSafe(mx, my))
    {
      return false;
    }
  }
  return true;
}

bool ThetaStarPlanner::connectPathToExactEndpoints(
  nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  bool connect_goal) const
{
  if (path.poses.empty()) {
    return false;
  }

  geometry_msgs::msg::PoseStamped exact_start = start;
  exact_start.header = path.header;
  exact_start.pose.position.z = 0.0;
  const auto & grid_start = path.poses.front().pose.position;
  const double start_gap = std::hypot(
    exact_start.pose.position.x - grid_start.x,
    exact_start.pose.position.y - grid_start.y);
  if (start_gap <= kEndpointPositionEpsilon) {
    path.poses.front().pose.position = exact_start.pose.position;
  } else {
    if (!isWorldSegmentSafe(exact_start.pose.position, grid_start)) {
      return false;
    }
    // 首点必须是当前机器人连续坐标，MPPI 再从该点连续积分，不会“跳到”栅格中心。
    path.poses.insert(path.poses.begin(), exact_start);
  }

  if (!connect_goal) {
    return true;
  }

  geometry_msgs::msg::PoseStamped exact_goal = goal;
  exact_goal.header = path.header;
  exact_goal.pose.position.z = 0.0;
  const auto & grid_goal = path.poses.back().pose.position;
  const double goal_gap = std::hypot(
    exact_goal.pose.position.x - grid_goal.x,
    exact_goal.pose.position.y - grid_goal.y);
  if (goal_gap <= kEndpointPositionEpsilon) {
    path.poses.back().pose.position = exact_goal.pose.position;
  } else {
    if (!isWorldSegmentSafe(grid_goal, exact_goal.pose.position)) {
      return false;
    }
    // 末点恢复为用户下发的连续坐标，目标容差不再受 5 cm 栅格中心量化限制。
    path.poses.push_back(exact_goal);
  }
  return true;
}

void ThetaStarPlanner::buildGeometricPath(
  nav_msgs::msg::Path & global_path,
  const std::vector<coordsW> & raw_path)
{
  if (smooth_path_mode_ == SmoothPathMode::L1_SPLINE) {
    buildGeometricPathL1Spline(global_path, raw_path);
  } else {
    buildGeometricPathLegacy(global_path, raw_path);
  }
}

void ThetaStarPlanner::buildGeometricPathL1Spline(
  nav_msgs::msg::Path & global_path,
  const std::vector<coordsW> & raw_path)
{
  const double res = planner_->costmap_->getResolution();
  const nav2_map_server::DistanceMap * dist_map = nullptr;
  if (multi_layer_planner_ && !multi_layer_planner_->distance_map_.distance_m.empty()) {
    dist_map = &multi_layer_planner_->distance_map_;
  }

  const auto elapsedMs = [](const std::chrono::steady_clock::time_point & start) {
      return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    };
  const auto t0 = std::chrono::steady_clock::now();

  L1SplinePathConfig l1_cfg;
  l1_cfg.sample_step_m = curve_path_config_.sample_step_m > 0.0 ?
    curve_path_config_.sample_step_m : res;
  l1_cfg.corner_angle_rad = smooth_l1_corner_angle_rad_;
  l1_cfg.simplify_angle_rad = smooth_l1_simplify_angle_rad_;
  l1_cfg.fillet_max_leg_fraction = curve_path_config_.fillet_max_leg_fraction;
  l1_cfg.clearance_radius_factor = smooth_l1_clearance_radius_factor_;

  std::vector<coordsW> dense_pts;
  L1SplinePathStats l1_stats;
  L1SplinePathBuilder::build(raw_path, dist_map, l1_cfg, dense_pts, &l1_stats);

  global_path = coordsToPath(dense_pts);

  bool gradient_ok = true;
  if (smooth_l1_do_gradient_ && global_path.poses.size() > 2) {
    gradient_ok = OmniPathSmoother::gradientSmooth(
      global_path, planner_->costmap_, path_smooth_config_, planner_->allow_unknown_);
  }

  if (smooth_log_timing_) {
    RCLCPP_INFO(
      logger_,
      "L1 spline path: graph=%zu simplified=%zu → dense=%zu | fillet seen=%zu applied=%zu "
      "rejected=%zu | min_clear=%.3fm | gradient=%s | %.2f ms",
      l1_stats.graph_waypoints,
      l1_stats.simplified_waypoints,
      l1_stats.dense_poses,
      l1_stats.fillet_corners_seen,
      l1_stats.fillet_corners_applied,
      l1_stats.fillet_corners_rejected,
      l1_stats.min_corner_clearance_m,
      smooth_l1_do_gradient_ ? (gradient_ok ? "ok" : "partial") : "off",
      elapsedMs(t0));
  }

  global_path.header.stamp = clock_->now();
  global_path.header.frame_id = global_frame_;
}

void ThetaStarPlanner::buildGeometricPathLegacy(
  nav_msgs::msg::Path & global_path,
  const std::vector<coordsW> & raw_path)
{
  std::vector<coordsW> waypoints = raw_path;
  const bool do_string_pull =
    (smooth_path_ && smooth_do_string_pull_) ||
    (smooth_do_curve_fit_ && smooth_do_string_pull_);
  if (do_string_pull && waypoints.size() >= 2) {
    OmniPathSmoother::stringPull(
      waypoints, planner_->costmap_, planner_->allow_unknown_, path_smooth_config_);
  }

  const double res = planner_->costmap_->getResolution();
  curve_path_config_.corner_angle_rad = path_smooth_config_.string_pull_corner_angle_rad;
  if (curve_path_config_.sample_step_m <= 0.0) {
    curve_path_config_.sample_step_m = res;
  }

  const auto elapsedMs = [](const std::chrono::steady_clock::time_point & start) {
      return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    };

  const auto t0 = std::chrono::steady_clock::now();
  CurveFitStats fillet_stats;
  const size_t sparse_count = raw_path.size();
  if (smooth_do_curve_fit_ && waypoints.size() >= 2) {
    CurvePathSmoother::fitCurvesGeometric(
      waypoints, curve_path_config_, nullptr, 0.85, &fillet_stats);
    if (smooth_log_timing_ && fillet_stats.fillet_corners_seen > 0) {
      RCLCPP_INFO(
        logger_,
        "Corner fillet (geometry): seen=%zu applied=%zu rejected=%zu | waypoints %zu→%zu",
        fillet_stats.fillet_corners_seen,
        fillet_stats.fillet_corners_applied,
        fillet_stats.fillet_corners_rejected,
        sparse_count,
        waypoints.size());
    }
  }

  const bool waypoints_dense = waypoints.size() > sparse_count + sparse_count / 3;
  if (waypoints_dense) {
    global_path = coordsToPath(waypoints);
  } else {
    global_path = linearInterpolation(waypoints, res);
  }

  bool gradient_ok = true;
  if (smooth_path_ && smooth_do_gradient_ && global_path.poses.size() > 2) {
    gradient_ok = OmniPathSmoother::gradientSmooth(
      global_path, planner_->costmap_, path_smooth_config_, planner_->allow_unknown_);
  }

  if (smooth_log_timing_) {
    RCLCPP_INFO(
      logger_,
      "Path post-process: sparse=%zu dense=%zu poses=%zu | mode=legacy "
      "fillet=%s gradient=%s(%s) | %.2f ms",
      sparse_count,
      waypoints.size(),
      global_path.poses.size(),
      smooth_do_curve_fit_ ? "on" : "off",
      (smooth_path_ && smooth_do_gradient_) ? "on" : "off",
      gradient_ok ? "ok" : "partial",
      elapsedMs(t0));
  }

  global_path.header.stamp = clock_->now();
  global_path.header.frame_id = global_frame_;
}

nav_msgs::msg::Path ThetaStarPlanner::linearInterpolation(
  const std::vector<coordsW> & raw_path,
  const double & dist_bw_points)
{
  nav_msgs::msg::Path pa;

  if (raw_path.empty()) {
    return pa;
  }

  geometry_msgs::msg::PoseStamped p1;
  if (raw_path.size() == 1 || dist_bw_points <= 0.0) {
    p1.pose.position.x = raw_path[0].x;
    p1.pose.position.y = raw_path[0].y;
    pa.poses.push_back(p1);
    return pa;
  }

  for (unsigned int j = 0; j < raw_path.size() - 1; j++) {
    coordsW pt1 = raw_path[j];
    p1.pose.position.x = pt1.x;
    p1.pose.position.y = pt1.y;
    pa.poses.push_back(p1);

    coordsW pt2 = raw_path[j + 1];
    double distance = std::hypot(pt2.x - pt1.x, pt2.y - pt1.y);
    int loops = static_cast<int>(distance / dist_bw_points);
    double sin_alpha = (pt2.y - pt1.y) / distance;
    double cos_alpha = (pt2.x - pt1.x) / distance;
    for (int k = 1; k < loops; k++) {
      p1.pose.position.x = pt1.x + k * dist_bw_points * cos_alpha;
      p1.pose.position.y = pt1.y + k * dist_bw_points * sin_alpha;
      pa.poses.push_back(p1);
    }
  }

  p1.pose.position.x = raw_path.back().x;
  p1.pose.position.y = raw_path.back().y;
  pa.poses.push_back(p1);

  if (pa.poses.size() > kMaxInterpolatedPoses) {
    pa.poses.resize(kMaxInterpolatedPoses);
  }

  return pa;
}

rcl_interfaces::msg::SetParametersResult
ThetaStarPlanner::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == name_ + ".how_many_corners") {
        planner_->how_many_corners_ = parameter.as_int();
      } else if (name == name_ + ".steering_max_smoothing_passes") {
        swerve_steering_config_.max_smoothing_passes = parameter.as_int();
      } else if (name == name_ + ".smooth_max_iterations") {
        path_smooth_config_.max_iterations = parameter.as_int();
      } else if (name == name_ + ".smooth_refinement_num") {
        path_smooth_config_.refinement_num = parameter.as_int();
      }
    } else if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == name_ + ".w_euc_cost") {
        planner_->w_euc_cost_ = parameter.as_double();
      } else if (name == name_ + ".w_traversal_cost") {
        planner_->w_traversal_cost_ = parameter.as_double();
      } else if (name == name_ + ".path_orientation_same_threshold_rad") {
        path_orientation_same_threshold_rad_ = parameter.as_double();
      } else if (name == name_ + ".tangent_terminal_yaw_hold_distance") {
        tangent_terminal_yaw_hold_distance_ = std::max(0.0, parameter.as_double());
      } else if (name == name_ + ".steering_chassis_wheelbase") {
        swerve_steering_config_.wheelbase = parameter.as_double();
      } else if (name == name_ + ".steering_chassis_track") {
        swerve_steering_config_.track = parameter.as_double();
      } else if (name == name_ + ".steering_limit_rad") {
        swerve_steering_config_.steer_limit = parameter.as_double();
      } else if (name == name_ + ".steering_max_wheel_angle_step") {
        swerve_steering_config_.max_wheel_angle_step = parameter.as_double();
      } else if (name == name_ + ".steering_nominal_path_speed") {
        swerve_steering_config_.nominal_path_speed = parameter.as_double();
      } else if (name == name_ + ".steering_max_body_yaw_rate") {
        swerve_steering_config_.max_body_yaw_rate = parameter.as_double();
      } else if (name == name_ + ".smooth_w_data") {
        path_smooth_config_.w_data = parameter.as_double();
      } else if (name == name_ + ".smooth_w_smooth") {
        path_smooth_config_.w_smooth = parameter.as_double();
      } else if (name == name_ + ".smooth_tolerance") {
        path_smooth_config_.tolerance = parameter.as_double();
      } else if (name == name_ + ".smooth_string_pull_corner_angle_rad") {
        path_smooth_config_.string_pull_corner_angle_rad = parameter.as_double();
        curve_path_config_.corner_angle_rad = path_smooth_config_.string_pull_corner_angle_rad;
      } else if (name == name_ + ".smooth_curve_sample_step_m") {
        curve_path_config_.sample_step_m = parameter.as_double();
      } else if (name == name_ + ".smooth_curve_fillet_max_leg_fraction") {
        curve_path_config_.fillet_max_leg_fraction = parameter.as_double();
      } else if (name == name_ + ".smooth_l1_clearance_radius_factor") {
        smooth_l1_clearance_radius_factor_ = parameter.as_double();
      } else if (name == name_ + ".smooth_l1_corner_angle_rad") {
        smooth_l1_corner_angle_rad_ = parameter.as_double();
      } else if (name == name_ + ".smooth_l1_simplify_angle_rad") {
        smooth_l1_simplify_angle_rad_ = parameter.as_double();
      } else if (name == name_ + ".smooth_l1_do_gradient") {
        smooth_l1_do_gradient_ = parameter.as_bool();
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == name_ + ".smooth_path_mode") {
        smooth_path_mode_ = parseSmoothPathMode(parameter.as_string());
      } else if (name == name_ + ".path_orientation_mode") {
        path_orientation_mode_ = parsePathOrientationMode(parameter.as_string());
        if (!assign_path_orientations_) {
          path_orientation_mode_ = PathOrientationMode::NONE;
        }
      } else if (name == name_ + ".node_map_yaml") {
        node_map_yaml_ = nav2_map_server::expandUserPath(parameter.as_string());
        if (use_node_map_) {
          node_map_loaded_ = loadNodeMapResources();
        }
      } else if (name == name_ + ".distance_map_yaml") {
        distance_map_yaml_ = nav2_map_server::expandUserPath(parameter.as_string());
        if (use_node_map_) {
          node_map_loaded_ = loadNodeMapResources();
        }
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == name_ + ".use_final_approach_orientation") {
        use_final_approach_orientation_ = parameter.as_bool();
      } else if (name == name_ + ".allow_unknown") {
        planner_->allow_unknown_ = parameter.as_bool();
      } else if (name == name_ + ".assign_path_orientations") {
        assign_path_orientations_ = parameter.as_bool();
        if (!assign_path_orientations_) {
          path_orientation_mode_ = PathOrientationMode::NONE;
        }
      } else if (name == name_ + ".enforce_steering_continuity") {
        enforce_steering_continuity_ = parameter.as_bool();
      } else if (name == name_ + ".tangent_in_place_rotation_markers") {
        tangent_in_place_rotation_markers_ = parameter.as_bool();
      } else if (name == name_ + ".use_node_map") {
        use_node_map_ = parameter.as_bool();
        if (use_node_map_) {
          node_map_loaded_ = loadNodeMapResources();
        }
      } else if (name == name_ + ".smooth_path") {
        smooth_path_ = parameter.as_bool();
      } else if (name == name_ + ".smooth_do_string_pull") {
        smooth_do_string_pull_ = parameter.as_bool();
      } else if (name == name_ + ".smooth_do_gradient") {
        smooth_do_gradient_ = parameter.as_bool();
      } else if (name == name_ + ".smooth_do_refinement") {
        path_smooth_config_.do_refinement = parameter.as_bool();
      } else if (name == name_ + ".smooth_do_curve_fit") {
        smooth_do_curve_fit_ = parameter.as_bool();
      } else if (name == name_ + ".smooth_log_timing") {
        smooth_log_timing_ = parameter.as_bool();
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_theta_star_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_theta_star_planner::ThetaStarPlanner, nav2_core::GlobalPlanner)
