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

#include <vector>
#include <memory>
#include <string>
#include <algorithm>
#include <cctype>
#include <cmath>
#include "nav2_theta_star_planner/theta_star_planner.hpp"
#include "nav2_theta_star_planner/theta_star.hpp"
#include "tf2/utils.h"

namespace nav2_theta_star_planner
{
namespace
{
std::string toLowerCopy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
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

  // TANGENT：相邻点方向作为 yaw（差速车/需要贴路径切线时使用）
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
    const double theta = std::atan2(dy, dx);
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

  RCLCPP_INFO(
    logger_,
    "Theta* path orientation: assign=%s mode=%s same_threshold=%.3f rad. "
    "Omni: PathAngleCritic off + PathAlign use_path_orientations=true",
    assign_path_orientations_ ? "true" : "false",
    path_orientation_mode_param.c_str(),
    path_orientation_same_threshold_rad_);

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
}

void ThetaStarPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s of type nav2_theta_star_planner", name_.c_str());
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
      RCLCPP_WARN(logger_, "Failed to create a unique pose path because the pose is not in free space");
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
  getPlan(global_path);

  const size_t plan_size = global_path.poses.size();
  if (plan_size > 0) {
    // 1) 为中间/全程点写入 orientation（可配置）
    if (assign_path_orientations_ &&
      path_orientation_mode_ != PathOrientationMode::NONE)
    {
      assignPathOrientations(global_path, start, goal);
    }

    // 2) 确定路径末点目标 yaw
    double final_goal_yaw = tf2::getYaw(goal.pose.orientation);
    if (use_final_approach_orientation_) {
      if (plan_size == 1) {
        final_goal_yaw = tf2::getYaw(start.pose.orientation);
      } else {
        const auto & last_pose = global_path.poses.back().pose.position;
        const auto & approach_pose = global_path.poses[plan_size - 2].pose.position;
        final_goal_yaw = std::atan2(
          last_pose.y - approach_pose.y,
          last_pose.x - approach_pose.x);
      }
    }

    // 3) B 层：沿路径仿真舵轮 IK，平滑 yaw 曲线，避免底盘 Two-Phase 停车转舵
    if (enforce_steering_continuity_ &&
      assign_path_orientations_ &&
      path_orientation_mode_ != PathOrientationMode::NONE)
    {
      applySteeringContinuityConstraint(global_path, final_goal_yaw);
    }

    // 4) 末点 orientation 与 final_goal_yaw 对齐
    global_path.poses.back().pose.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(final_goal_yaw);
  }

  auto stop_time = std::chrono::steady_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::microseconds>(stop_time - start_time);
  RCLCPP_DEBUG(logger_, "the time taken is : %i", static_cast<int>(dur.count()));
  RCLCPP_DEBUG(logger_, "the nodes_opened are:  %i", planner_->nodes_opened);
  return global_path;
}

void ThetaStarPlanner::getPlan(nav_msgs::msg::Path & global_path)
{
  std::vector<coordsW> path;
  if (planner_->isUnsafeToPlan()) {
    RCLCPP_ERROR(logger_, "Either of the start or goal pose are an obstacle! ");
    global_path.poses.clear();
  } else if (planner_->generatePath(path)) {
    global_path = linearInterpolation(path, planner_->costmap_->getResolution());
  } else {
    RCLCPP_ERROR(logger_, "Could not generate path between the given poses");
    global_path.poses.clear();
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
      }
    } else if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == name_ + ".w_euc_cost") {
        planner_->w_euc_cost_ = parameter.as_double();
      } else if (name == name_ + ".w_traversal_cost") {
        planner_->w_traversal_cost_ = parameter.as_double();
      } else if (name == name_ + ".path_orientation_same_threshold_rad") {
        path_orientation_same_threshold_rad_ = parameter.as_double();
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
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == name_ + ".path_orientation_mode") {
        path_orientation_mode_ = parsePathOrientationMode(parameter.as_string());
        if (!assign_path_orientations_) {
          path_orientation_mode_ = PathOrientationMode::NONE;
        }
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_theta_star_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_theta_star_planner::ThetaStarPlanner, nav2_core::GlobalPlanner)
