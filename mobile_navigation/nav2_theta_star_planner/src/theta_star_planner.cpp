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

}  // namespace

SmoothPathMode ThetaStarPlanner::parseSmoothPathMode(const std::string & mode)
{
  const std::string normalized = toLowerCopy(mode);
  if (normalized == "l1_spline" || normalized == "l1" || normalized == "spline") {
    return SmoothPathMode::L1_SPLINE;
  }
  return SmoothPathMode::LEGACY;
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

  // Geometry-only path smoothing (no minimum-turning-radius constraint).
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_path", rclcpp::ParameterValue(false));
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
    if (!nav2_map_server::loadNodeMap(raw_map_yaml_, raw_map)) {
      RCLCPP_WARN(logger_, "Raw node map missing: %s", raw_map_yaml_.c_str());
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

  // Corner case of start and goal beeing on the same cell
  unsigned int mx_start, my_start, mx_goal, my_goal;
  planner_->costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start);
  planner_->costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal);
  if (mx_start == mx_goal && my_start == my_goal) {
    if (!planner_->isSafe(mx_start, my_start)) {
      RCLCPP_WARN(
        logger_, "Failed to create a unique pose path because the pose is not in free space");
      return global_path;
    }
    global_path.header.stamp = clock_->now();
    global_path.header.frame_id = global_frame_;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = global_path.header;
    pose.pose.position.z = 0.0;

    pose.pose = start.pose;
    // if we have a different start and goal orientation, set the unique path pose to the goal
    // orientation, unless use_final_approach_orientation=true where we need it to be the start
    // orientation to avoid movement from the local planner
    if (start.pose.orientation != goal.pose.orientation && !use_final_approach_orientation_) {
      pose.pose.orientation = goal.pose.orientation;
    }
    global_path.poses.push_back(pose);
    return global_path;
  }

  planner_->setStartAndGoal(start, goal);
  RCLCPP_DEBUG(
    logger_, "Got the src and dst... (%i, %i) && (%i, %i)",
    planner_->src_.x, planner_->src_.y, planner_->dst_.x, planner_->dst_.y);
  getPlan(global_path, start, goal);
  // check if a plan is generated
  size_t plan_size = global_path.poses.size();
  if (plan_size > 0) {
    global_path.poses.back().pose.orientation = goal.pose.orientation;
  }

  // If use_final_approach_orientation=true, interpolate the last pose orientation from the
  // previous pose to set the orientation to the 'final approach' orientation of the robot so
  // it does not rotate.
  // And deal with corner case of plan of length 1
  if (use_final_approach_orientation_) {
    if (plan_size == 1) {
      global_path.poses.back().pose.orientation = start.pose.orientation;
    } else if (plan_size > 1) {
      double dx, dy, theta;
      auto last_pose = global_path.poses.back().pose.position;
      auto approach_pose = global_path.poses[plan_size - 2].pose.position;
      dx = last_pose.x - approach_pose.x;
      dy = last_pose.y - approach_pose.y;
      theta = atan2(dy, dx);
      global_path.poses.back().pose.orientation =
        nav2_util::geometry_utils::orientationAroundZAxis(theta);
    }
  }

  auto stop_time = std::chrono::steady_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::microseconds>(stop_time - start_time);
  RCLCPP_DEBUG(logger_, "the time taken is : %i", static_cast<int>(dur.count()));
  RCLCPP_DEBUG(logger_, "the nodes_opened are:  %i", planner_->nodes_opened);
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

  if (!smooth_path_ && !smooth_do_curve_fit_) {
    // Backward-compatible release behavior for the existing differential-drive setup.
    global_path = linearInterpolation(path, planner_->costmap_->getResolution());
    global_path.header.stamp = clock_->now();
    global_path.header.frame_id = global_frame_;
  } else {
    buildGeometricPath(global_path, path);
    if (!global_path.poses.empty() &&
      !connectPathToExactEndpoints(global_path, start, goal))
    {
      RCLCPP_ERROR(logger_, "Could not safely connect smoothed Theta* path to exact endpoints");
      global_path.poses.clear();
    }
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
    // 首点必须是当前机器人连续坐标，local controller 再从该点连续积分，不会“跳到”栅格中心。
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
    global_path = denseInterpolation(waypoints, res);
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

  geometry_msgs::msg::PoseStamped p1;
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

  return pa;
}

nav_msgs::msg::Path ThetaStarPlanner::denseInterpolation(
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
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == name_ + ".smooth_path_mode") {
        smooth_path_mode_ = parseSmoothPathMode(parameter.as_string());
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
      } else if (name == name_ + ".smooth_l1_do_gradient") {
        smooth_l1_do_gradient_ = parameter.as_bool();
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
