// Copyright (C) 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0

#include "nav2_frenet_ilqr_controller/frenet_local_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace nav2_frenet_ilqr_controller
{
namespace
{

constexpr double kEpsilon = 1e-6;

double poseYaw(const geometry_msgs::msg::PoseStamped & pose)
{
  return tf2::getYaw(pose.pose.orientation);
}

std::string defaultPolicyType(const std::string & alias)
{
  return "nav2_frenet_ilqr_controller::policies::" + alias;
}

std::string defaultCostType(const std::string & alias)
{
  return "nav2_frenet_ilqr_controller::costs::" + alias;
}

double smoothStep(double value)
{
  const double u = std::clamp(value, 0.0, 1.0);
  return u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
}

}  // namespace

void FrenetLocalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("FrenetLocalPlanner: parent node expired");
  }

  node_ = parent;
  plugin_name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  costmap_ = costmap_ros_ ? costmap_ros_->getCostmap() : nullptr;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  using nav2_util::declare_parameter_if_not_declared;
  const std::string frenet = plugin_name_ + ".frenet_trajectory_planner.";

  auto readDoubleWithFallbacks =
    [&](const std::string & parameter,
      const std::vector<std::string> & fallback_parameters,
      double default_value) {
      const bool canonical_was_set = node->has_parameter(parameter);
      declare_parameter_if_not_declared(node, parameter, rclcpp::ParameterValue(default_value));
      for (const auto & fallback_parameter : fallback_parameters) {
        declare_parameter_if_not_declared(
          node, fallback_parameter, rclcpp::ParameterValue(
            default_value));
      }
      double value = default_value;
      node->get_parameter(parameter, value);
      if (!canonical_was_set) {
        for (const auto & fallback_parameter : fallback_parameters) {
          double fallback_value = default_value;
          node->get_parameter(fallback_parameter, fallback_value);
          if (std::abs(fallback_value - default_value) > kEpsilon) {
            value = fallback_value;
            break;
          }
        }
      }
      return value;
    };

  auto readDoubleWithLegacy =
    [&](const std::string & parameter, const std::string & legacy_parameter,
      double default_value) {
      return readDoubleWithFallbacks(parameter, {legacy_parameter}, default_value);
    };

  frenet_config_.min_lateral_distance =
    readDoubleWithLegacy(
    frenet + "min_lateral_distance", plugin_name_ + ".lateral_sample_min",
    -1.5);
  frenet_config_.max_lateral_distance =
    readDoubleWithLegacy(
    frenet + "max_lateral_distance", plugin_name_ + ".lateral_sample_max",
    1.5);
  frenet_config_.step_lateral_distance =
    readDoubleWithLegacy(
    frenet + "step_lateral_distance", plugin_name_ + ".lateral_sample_step",
    0.25);
  frenet_config_.min_longitudinal_velocity =
    readDoubleWithFallbacks(
    frenet + "min_longitudinal_velocity",
    {frenet + "min_longtitutal_velocity", plugin_name_ + ".min_longitudinal_velocity"}, 0.0);
  frenet_config_.max_longitudinal_velocity =
    readDoubleWithFallbacks(
    frenet + "max_longitudinal_velocity",
    {frenet + "max_longtitutal_velocity", plugin_name_ + ".max_longitudinal_velocity"}, 0.8);
  frenet_config_.step_longitudinal_velocity =
    readDoubleWithFallbacks(
    frenet + "step_longitudinal_velocity",
    {frenet + "step_longtitutal_velocity", plugin_name_ + ".step_longitudinal_velocity"}, 0.1);
  frenet_config_.min_time_interval =
    readDoubleWithLegacy(frenet + "min_time_interval", plugin_name_ + ".min_time_interval", 1.0);
  frenet_config_.max_time_interval =
    readDoubleWithLegacy(frenet + "max_time_interval", plugin_name_ + ".max_time_interval", 4.0);
  frenet_config_.step_time_interval =
    readDoubleWithLegacy(frenet + "step_time_interval", plugin_name_ + ".step_time_interval", 0.25);
  frenet_config_.min_lateral_time_interval =
    readDoubleWithLegacy(frenet + "min_lateral_time_interval", plugin_name_ + ".min_lateral_time_interval", 0.0);
  frenet_config_.max_lateral_time_interval =
    readDoubleWithLegacy(frenet + "max_lateral_time_interval", plugin_name_ + ".max_lateral_time_interval", 0.0);
  frenet_config_.step_lateral_time_interval =
    readDoubleWithLegacy(frenet + "step_lateral_time_interval", plugin_name_ + ".step_lateral_time_interval", 0.0);
  frenet_config_.dt =
    readDoubleWithLegacy(
    plugin_name_ + ".time_discretization",
    plugin_name_ + ".time_discretization", 0.05);

  declare_parameter_if_not_declared(
    node, frenet + "max_state_in_trajectory", rclcpp::ParameterValue(
      120));
  node->get_parameter(frenet + "max_state_in_trajectory", frenet_config_.max_state_in_trajectory);
  frenet_config_.max_state_in_trajectory = std::max(2, frenet_config_.max_state_in_trajectory);

  waypoint_resolution_ =
    readDoubleWithLegacy(
    plugin_name_ + ".waypoint_resolution", plugin_name_ + ".path_step_s",
    0.25);
  projection_search_distance_ = readDoubleWithLegacy(
    plugin_name_ + ".projection_search_distance",
    plugin_name_ + ".projection_search_distance", 5.0);
  max_backtrack_distance_ =
    readDoubleWithLegacy(
    plugin_name_ + ".max_backtrack_distance",
    plugin_name_ + ".max_backtrack_distance", 0.5);
  transform_tolerance_ =
    readDoubleWithLegacy(
    plugin_name_ + ".transform_tolerance",
    plugin_name_ + ".transform_tolerance", 0.2);
  tail_rejoin_distance_ =
    readDoubleWithLegacy(
    plugin_name_ + ".tail_rejoin_distance",
    plugin_name_ + ".tail_rejoin_distance", 1.5);

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".visualize_candidate_trajectories",
    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_visualized_candidates", rclcpp::ParameterValue(
      80));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".candidate_paths_topic",
    rclcpp::ParameterValue(std::string("/frenet_rpp_controller/candidate_paths")));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rollouts_topic",
    rclcpp::ParameterValue(std::string("/controller_server/frenet_rollouts")));
  node->get_parameter(
    plugin_name_ + ".visualize_candidate_trajectories",
    visualize_candidate_trajectories_);
  node->get_parameter(plugin_name_ + ".max_visualized_candidates", max_visualized_candidates_);
  node->get_parameter(plugin_name_ + ".candidate_paths_topic", candidate_paths_topic_);
  node->get_parameter(plugin_name_ + ".rollouts_topic", rollouts_topic_);

  frenet_config_.step_lateral_distance = std::max(0.01, frenet_config_.step_lateral_distance);
  frenet_config_.step_longitudinal_velocity = std::max(
    0.01,
    frenet_config_.step_longitudinal_velocity);
  frenet_config_.step_time_interval = std::max(0.01, frenet_config_.step_time_interval);
  frenet_config_.dt = std::max(0.01, frenet_config_.dt);
  waypoint_resolution_ = std::max(0.05, waypoint_resolution_);
  projection_search_distance_ = std::max(0.5, projection_search_distance_);
  max_backtrack_distance_ = std::max(0.0, max_backtrack_distance_);
  transform_tolerance_ = std::max(0.0, transform_tolerance_);
  tail_rejoin_distance_ = std::max(0.0, tail_rejoin_distance_);
  max_visualized_candidates_ = std::max(0, max_visualized_candidates_);

  if (frenet_config_.max_lateral_distance < frenet_config_.min_lateral_distance) {
    std::swap(frenet_config_.max_lateral_distance, frenet_config_.min_lateral_distance);
  }
  if (frenet_config_.max_longitudinal_velocity < frenet_config_.min_longitudinal_velocity) {
    std::swap(frenet_config_.max_longitudinal_velocity, frenet_config_.min_longitudinal_velocity);
  }
  if (frenet_config_.max_time_interval < frenet_config_.min_time_interval) {
    std::swap(frenet_config_.max_time_interval, frenet_config_.min_time_interval);
  }

  frenet_planner_ = std::make_unique<frenet_planner::FrenetTrajectoryPlanner>(frenet_config_);
  loadPolicies(node);
  loadCosts(node);

  if (visualize_candidate_trajectories_ && !candidate_paths_topic_.empty()) {
    candidate_paths_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      candidate_paths_topic_, rclcpp::SystemDefaultsQoS());
  }
  if (visualize_candidate_trajectories_ && !rollouts_topic_.empty()) {
    rollouts_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      rollouts_topic_, rclcpp::SystemDefaultsQoS());
  }

  RCLCPP_INFO(
    logger_,
    "[nav2_core::LocalPlanner] Upstream-aligned Frenet planner '%s': lateral=[%.2f, %.2f] "
    "step=%.2f, longitudinal=[%.2f, %.2f] step=%.2f, lon_time=[%.2f, %.2f] step=%.2f, "
    "lat_time=[%.2f, %.2f] step=%.2f, states=%d dt=%.3f, policies=%zu, costs=%zu, iLQR=disabled",
    plugin_name_.c_str(), frenet_config_.min_lateral_distance, frenet_config_.max_lateral_distance,
    frenet_config_.step_lateral_distance, frenet_config_.min_longitudinal_velocity,
    frenet_config_.max_longitudinal_velocity, frenet_config_.step_longitudinal_velocity,
    frenet_config_.min_time_interval, frenet_config_.max_time_interval, frenet_config_.step_time_interval,
    frenet_config_.min_lateral_time_interval, frenet_config_.max_lateral_time_interval, frenet_config_.step_lateral_time_interval,
    frenet_config_.max_state_in_trajectory, frenet_config_.dt, policies_.size(), costs_.size());
}

void FrenetLocalPlanner::cleanup()
{
  std::lock_guard<std::mutex> lock(mtx_);
  raw_global_plan_ = nav_msgs::msg::Path();
  cumulative_arc_.clear();
  last_projection_index_ = -1;
  policies_.clear();
  costs_.clear();
  frenet_planner_.reset();
  policy_loader_.reset();
  cost_loader_.reset();
  candidate_paths_pub_.reset();
  rollouts_pub_.reset();
}

void FrenetLocalPlanner::activate()
{
  if (candidate_paths_pub_) {
    candidate_paths_pub_->on_activate();
  }
  if (rollouts_pub_) {
    rollouts_pub_->on_activate();
  }
}

void FrenetLocalPlanner::deactivate()
{
  if (candidate_paths_pub_) {
    candidate_paths_pub_->on_deactivate();
  }
  if (rollouts_pub_) {
    rollouts_pub_->on_deactivate();
  }
}

void FrenetLocalPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mtx_);
  raw_global_plan_ = path;
  last_projection_index_ = -1;
  rebuildArcLengths();
}

void FrenetLocalPlanner::rebuildArcLengths()
{
  cumulative_arc_.clear();
  if (raw_global_plan_.poses.empty()) {
    return;
  }
  cumulative_arc_.resize(raw_global_plan_.poses.size(), 0.0);
  for (std::size_t index = 1U; index < raw_global_plan_.poses.size(); ++index) {
    const auto & previous = raw_global_plan_.poses[index - 1U].pose.position;
    const auto & current = raw_global_plan_.poses[index].pose.position;
    cumulative_arc_[index] =
      cumulative_arc_[index - 1U] + std::hypot(current.x - previous.x, current.y - previous.y);
  }
  if (raw_global_plan_.header.frame_id.empty()) {
    raw_global_plan_.header.frame_id = raw_global_plan_.poses.front().header.frame_id;
  }
}

void FrenetLocalPlanner::loadPolicies(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  std::vector<std::string> aliases;
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".policy_plugins",
    rclcpp::ParameterValue(std::vector<std::string>{"ObstaclePolicy", "ConstraintsPolicy"}));
  node->get_parameter(plugin_name_ + ".policy_plugins", aliases);
  if (aliases.empty()) {
    RCLCPP_WARN(logger_, "No Frenet policy plugins configured; candidates are not safety-filtered");
    return;
  }

  policy_loader_ = std::make_unique<pluginlib::ClassLoader<policies::RclcppNodePolicy>>(
    "nav2_frenet_ilqr_controller", "nav2_frenet_ilqr_controller::policies::RclcppNodePolicy");
  for (const auto & alias : aliases) {
    const std::string parameter_prefix = plugin_name_ + "." + alias;
    nav2_util::declare_parameter_if_not_declared(
      node, parameter_prefix + ".plugin",
      rclcpp::ParameterValue(defaultPolicyType(alias)));
    std::string plugin_type;
    node->get_parameter(parameter_prefix + ".plugin", plugin_type);
    try {
      auto policy = policy_loader_->createSharedInstance(plugin_type);
      policy->initialize(parameter_prefix, node_, costmap_ros_);
      policies_.push_back(policy);
      frenet_planner_->addPolicy(policy);
      RCLCPP_INFO(
        logger_, "Loaded Frenet policy '%s' type='%s'", alias.c_str(),
        plugin_type.c_str());
    } catch (const pluginlib::PluginlibException & exception) {
      throw std::runtime_error(
              "Failed to load Frenet policy '" + alias + "' type='" + plugin_type +
              "': " + exception.what());
    }
  }
}

void FrenetLocalPlanner::loadCosts(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  std::vector<std::string> aliases;
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".cost_checker_plugins",
    rclcpp::ParameterValue(
      std::vector<std::string>{"LateralDistanceCostChecker", "LongitudinalVelocityCostChecker",
        "LatLonBalanceCostChecker"}));
  node->get_parameter(plugin_name_ + ".cost_checker_plugins", aliases);
  if (aliases.empty()) {
    RCLCPP_WARN(logger_, "No Frenet cost plugins configured; selection uses zero cost");
    return;
  }

  cost_loader_ = std::make_unique<pluginlib::ClassLoader<costs::RclcppNodeCost>>(
    "nav2_frenet_ilqr_controller", "nav2_frenet_ilqr_controller::costs::RclcppNodeCost");
  const auto default_type_for_alias = [](const std::string & alias) {
      if (alias == "LateralDistanceCostChecker") {
        return std::string("nav2_frenet_ilqr_controller::costs::LateralDistanceCost");
      }
      if (alias == "LongitudinalVelocityCostChecker") {
        return std::string("nav2_frenet_ilqr_controller::costs::LongitudinalVelocityCost");
      }
      if (alias == "LongtitutalVelocityCostChecker") {
        return std::string("nav2_frenet_ilqr_controller::costs::LongtitutalVelocityCost");
      }
      if (alias == "LatLonBalanceCostChecker") {
        return std::string("nav2_frenet_ilqr_controller::costs::LatLonBalanceCost");
      }
      if (alias == "ObstacleCostChecker") {
        return std::string("nav2_frenet_ilqr_controller::costs::ObstacleCost");
      }
      return defaultCostType(alias);
    };

  for (const auto & alias : aliases) {
    const std::string parameter_prefix = plugin_name_ + "." + alias;
    nav2_util::declare_parameter_if_not_declared(
      node, parameter_prefix + ".plugin",
      rclcpp::ParameterValue(default_type_for_alias(alias)));
    std::string plugin_type;
    node->get_parameter(parameter_prefix + ".plugin", plugin_type);
    try {
      auto cost = cost_loader_->createSharedInstance(plugin_type);
      cost->initialize(parameter_prefix, node_, costmap_ros_);
      costs_.push_back(cost);
      frenet_planner_->addCost(cost);
      RCLCPP_INFO(logger_, "Loaded Frenet cost '%s' type='%s'", alias.c_str(), plugin_type.c_str());
    } catch (const pluginlib::PluginlibException & exception) {
      throw std::runtime_error(
              "Failed to load Frenet cost '" + alias + "' type='" + plugin_type +
              "': " + exception.what());
    }
  }
}

FrenetLocalPlanner::PreparedPlan FrenetLocalPlanner::preparePlanInCostmapFrame(
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  PreparedPlan prepared;
  prepared.path = raw_global_plan_;
  prepared.pose = robot_pose;
  prepared.cumulative_arc = cumulative_arc_;
  if (prepared.path.poses.empty()) {
    return prepared;
  }

  std::string source_frame = prepared.path.header.frame_id;
  if (source_frame.empty()) {
    source_frame = prepared.path.poses.front().header.frame_id;
  }
  std::string target_frame = costmap_ros_ ? costmap_ros_->getGlobalFrameID() : source_frame;
  if (target_frame.empty()) {
    target_frame = source_frame;
  }
  if (source_frame.empty()) {
    source_frame = target_frame;
  }
  prepared.path.header.frame_id = target_frame;

  for (auto & pose : prepared.path.poses) {
    if (pose.header.frame_id.empty()) {
      pose.header.frame_id = source_frame;
    }
  }
  if (prepared.pose.header.frame_id.empty()) {
    prepared.pose.header.frame_id = source_frame;
  }

  if (!tf_ || (source_frame == target_frame && prepared.pose.header.frame_id == target_frame)) {
    prepared.path.header.frame_id = target_frame;
    prepared.pose.header.frame_id = target_frame;
    return prepared;
  }

  try {
    const auto timeout = rclcpp::Duration::from_seconds(transform_tolerance_);
    geometry_msgs::msg::TransformStamped transform;
    if (source_frame != target_frame) {
      transform = tf_->lookupTransform(target_frame, source_frame, rclcpp::Time(0), timeout);
      for (std::size_t index = 0U; index < prepared.path.poses.size(); ++index) {
        geometry_msgs::msg::PoseStamped transformed;
        tf2::doTransform(prepared.path.poses[index], transformed, transform);
        transformed.header.frame_id = target_frame;
        prepared.path.poses[index] = std::move(transformed);
      }
      prepared.transformed = true;
    }
    if (prepared.pose.header.frame_id != target_frame) {
      geometry_msgs::msg::PoseStamped transformed;
      tf2::doTransform(prepared.pose, transformed, transform);
      transformed.header.frame_id = target_frame;
      prepared.pose = std::move(transformed);
      prepared.transformed = true;
    }
    prepared.path.header.frame_id = target_frame;
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "Frenet planner cannot transform plan '%s' -> costmap '%s': %s; "
      "using plan-frame coordinates",
      source_frame.c_str(), target_frame.c_str(), exception.what());
    prepared.path = raw_global_plan_;
    prepared.pose = robot_pose;
    prepared.cumulative_arc = cumulative_arc_;
    prepared.transformed = false;
  }
  return prepared;
}

FrenetLocalPlanner::Projection FrenetLocalPlanner::projectOntoPath(
  const nav_msgs::msg::Path & path,
  const std::vector<double> & cumulative_arc,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  double lower_arc,
  double upper_arc) const
{
  Projection projection;
  if (path.poses.size() < 2U || cumulative_arc.size() != path.poses.size()) {
    return projection;
  }

  const Eigen::Vector2d robot(robot_pose.pose.position.x, robot_pose.pose.position.y);
  const double lower = std::clamp(lower_arc, 0.0, cumulative_arc.back());
  const double upper = std::clamp(upper_arc, lower, cumulative_arc.back());

  double best_distance_squared = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index + 1U < path.poses.size(); ++index) {
    if (cumulative_arc[index + 1U] + kEpsilon<lower || cumulative_arc[index] - kEpsilon> upper) {
      continue;
    }
    const auto & first = path.poses[index].pose.position;
    const auto & second = path.poses[index + 1U].pose.position;
    const Eigen::Vector2d start(first.x, first.y);
    const Eigen::Vector2d delta(second.x - first.x, second.y - first.y);
    const double length = delta.norm();
    if (length < kEpsilon) {
      continue;
    }
    const Eigen::Vector2d tangent = delta / length;
    const double ratio = std::clamp((robot - start).dot(tangent) / length, 0.0, 1.0);
    const Eigen::Vector2d point = start + ratio * delta;
    const double distance_squared = (point - robot).squaredNorm();
    if (distance_squared < best_distance_squared) {
      best_distance_squared = distance_squared;
      projection.valid = true;
      projection.segment_index = index;
      projection.ratio = ratio;
      projection.arc_length = cumulative_arc[index] + ratio * length;
      projection.yaw = std::atan2(delta.y(), delta.x());
      projection.point = point;
      projection.lateral_offset = tangent.x() * (robot.y() - point.y()) - tangent.y() *
        (robot.x() - point.x());
    }
  }
  return projection;
}

std::vector<frenet_planner::CartesianPoint> FrenetLocalPlanner::buildFrenetWaypoints(
  const nav_msgs::msg::Path & path, const Projection & projection) const
{
  std::vector<frenet_planner::CartesianPoint> waypoints;
  if (!projection.valid || path.poses.empty()) {
    return waypoints;
  }
  waypoints.push_back(projection.point);

  const auto appendIfFarEnough = [&](const geometry_msgs::msg::Point & point) {
      const Eigen::Vector2d candidate(point.x, point.y);
      if ((candidate - waypoints.back()).norm() >= waypoint_resolution_) {
        waypoints.push_back(candidate);
      }
    };

  for (std::size_t index = projection.segment_index + 1U; index < path.poses.size(); ++index) {
    appendIfFarEnough(path.poses[index].pose.position);
  }
  const auto & last = path.poses.back().pose.position;
  const Eigen::Vector2d final_point(last.x, last.y);
  if ((final_point - waypoints.back()).norm() > 1e-4) {
    waypoints.push_back(final_point);
  }
  return waypoints;
}

frenet_planner::CartesianState FrenetLocalPlanner::makeCartesianState(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity) const
{
  const double yaw = poseYaw(pose);
  const double vx = velocity.linear.x * std::cos(yaw) - velocity.linear.y * std::sin(yaw);
  const double vy = velocity.linear.x * std::sin(yaw) + velocity.linear.y * std::cos(yaw);
  frenet_planner::CartesianState state = frenet_planner::CartesianState::Zero();
  state[0] = pose.pose.position.x;
  state[1] = vx;
  state[3] = pose.pose.position.y;
  state[4] = vy;
  state[6] = yaw;
  return state;
}

void FrenetLocalPlanner::setPoseFromCartesian(
  const frenet_planner::CartesianState & state,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  geometry_msgs::msg::PoseStamped & pose) const
{
  pose.header.frame_id = frame_id;
  pose.header.stamp = stamp;
  pose.pose.position.x = state[0];
  pose.pose.position.y = state[3];
  pose.pose.position.z = 0.0;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, state[6]);
  pose.pose.orientation = tf2::toMsg(quaternion);
}

void FrenetLocalPlanner::appendPathTail(
  const frenet_planner::FrenetTrajectoryPlanner::PlanResult & result,
  const nav_msgs::msg::Path & path,
  const std::vector<double> & cumulative_arc,
  const Projection & projection,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  nav_msgs::msg::Path & local_path) const
{
  if (result.frenet.empty() || path.poses.size() < 2U ||
    cumulative_arc.size() != path.poses.size())
  {
    return;
  }

  const double final_local_s = result.frenet.back()[0];
  const double final_lateral = result.frenet.back()[3];
  const double absolute_s = projection.arc_length + final_local_s;
  if (absolute_s >= cumulative_arc.back() - 1e-3) {
    return;
  }

  std::size_t tail_index = static_cast<std::size_t>(
    std::lower_bound(
      cumulative_arc.begin(), cumulative_arc.end(),
      absolute_s) - cumulative_arc.begin());
  while (tail_index < cumulative_arc.size() && cumulative_arc[tail_index] <= absolute_s + 1e-3) {
    ++tail_index;
  }
  if (tail_index >= cumulative_arc.size()) {
    return;
  }

  std::size_t previous_index = tail_index > 0U ? tail_index - 1U : 0U;
  std::size_t next_index = std::min(tail_index + 1U, path.poses.size() - 1U);
  double held_lateral = final_lateral;
  for (std::size_t index = tail_index; index < path.poses.size(); ++index) {
    const auto & reference = path.poses[index].pose.position;
    const auto & previous = path.poses[previous_index].pose.position;
    const auto & next = path.poses[next_index].pose.position;
    Eigen::Vector2d tangent(next.x - previous.x, next.y - previous.y);
    if (tangent.norm() < kEpsilon) {
      tangent = Eigen::Vector2d(1.0, 0.0);
    } else {
      tangent.normalize();
    }
    const Eigen::Vector2d normal(-tangent.y(), tangent.x());
    const double arc_offset = cumulative_arc[index] - absolute_s;
    const double blend = tail_rejoin_distance_ <= kEpsilon ? 1.0 : smoothStep(
      arc_offset / tail_rejoin_distance_);
    double lateral = final_lateral * (1.0 - blend);

    // 若参考线该处或其紧邻前方仍存在障碍物，禁止过早向中心线回并，保持横向避障通道
    if (costmap_ && std::abs(final_lateral) > 1e-4) {
      unsigned int r_mx = 0U, r_my = 0U;
      if (costmap_->worldToMap(reference.x, reference.y, r_mx, r_my)) {
        const unsigned char rc = costmap_->getCost(r_mx, r_my);
        if (rc >= 80 && rc != nav2_costmap_2d::NO_INFORMATION) {
          lateral = final_lateral;
        }
      }
    }

    Eigen::Vector2d candidate(reference.x, reference.y);
    candidate += lateral * normal;
    bool collision = false;
    if (costmap_) {
      unsigned int map_x = 0U;
      unsigned int map_y = 0U;
      if (costmap_->worldToMap(candidate.x(), candidate.y(), map_x, map_y)) {
        const unsigned char cost = costmap_->getCost(map_x, map_y);
        collision = cost != nav2_costmap_2d::NO_INFORMATION &&
          static_cast<int>(cost) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
      }
    }
    if (collision && std::abs(held_lateral) > 1e-4) {
      lateral = held_lateral;
      candidate = Eigen::Vector2d(reference.x, reference.y) + lateral * normal;
      unsigned int map_x = 0U;
      unsigned int map_y = 0U;
      if (costmap_->worldToMap(candidate.x(), candidate.y(), map_x, map_y)) {
        const unsigned char cost = costmap_->getCost(map_x, map_y);
        collision = cost != nav2_costmap_2d::NO_INFORMATION &&
          static_cast<int>(cost) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
      }
    } else {
      held_lateral = lateral;
    }

    if (collision) {
      break;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp = stamp;
    pose.pose.position.x = candidate.x();
    pose.pose.position.y = candidate.y();
    pose.pose.position.z = 0.0;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, std::atan2(tangent.y(), tangent.x()));
    pose.pose.orientation = tf2::toMsg(quaternion);

    if (!local_path.poses.empty()) {
      const auto & last = local_path.poses.back().pose.position;
      if (std::hypot(pose.pose.position.x - last.x, pose.pose.position.y - last.y) < 1e-4) {
        continue;
      }
    }
    local_path.poses.push_back(std::move(pose));
    previous_index = index;
    next_index = std::min(index + 1U, path.poses.size() - 1U);
  }

  if (!local_path.poses.empty()) {
    local_path.poses.back().pose.orientation = path.poses.back().pose.orientation;
  }
}

void FrenetLocalPlanner::publishCandidates(
  const frenet_planner::DebugInfo & debug_info,
  const std::string & frame_id,
  const rclcpp::Time & stamp) const
{
  if (!visualize_candidate_trajectories_ || max_visualized_candidates_ <= 0) {
    return;
  }
  const bool publish_candidates =
    candidate_paths_pub_ && candidate_paths_pub_->is_activated() &&
    (candidate_paths_pub_->get_subscription_count() > 0U ||
    candidate_paths_pub_->get_intra_process_subscription_count() > 0U);
  const bool publish_rollouts =
    rollouts_pub_ && rollouts_pub_->is_activated() &&
    (rollouts_pub_->get_subscription_count() > 0U ||
    rollouts_pub_->get_intra_process_subscription_count() > 0U);
  if (!publish_candidates && !publish_rollouts) {
    return;
  }
  visualization_msgs::msg::MarkerArray marker_array;
  double best_cost = std::numeric_limits<double>::infinity();
  for (const auto & entry : debug_info.cartesian_trajectories) {
    if (entry.second >= 0.0) {
      best_cost = std::min(best_cost, entry.second);
    }
  }

  int marker_id = 0;
  for (const auto & entry : debug_info.cartesian_trajectories) {
    if (marker_id >= max_visualized_candidates_) {
      break;
    }
    const auto & trajectory = entry.first;
    if (trajectory.size() < 2U) {
      continue;
    }
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "frenet_upstream_rollouts";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = entry.second >= 0.0 &&
      std::abs(entry.second - best_cost) < 1e-6 ? 0.05 : 0.025;
    marker.color.a = 0.8F;
    if (entry.second < 0.0) {
      marker.color.r = 0.94F;
      marker.color.g = 0.27F;
      marker.color.b = 0.27F;
    } else if (std::abs(entry.second - best_cost) < 1e-6) {
      marker.color.r = 0.06F;
      marker.color.g = 0.73F;
      marker.color.b = 0.51F;
    } else {
      marker.color.r = 0.96F;
      marker.color.g = 0.62F;
      marker.color.b = 0.04F;
    }
    const std::size_t stride = std::max<std::size_t>(1U, trajectory.size() / 100U);
    for (std::size_t index = 0U; index < trajectory.size(); index += stride) {
      geometry_msgs::msg::Point point;
      point.x = trajectory[index][0];
      point.y = trajectory[index][3];
      point.z = 0.05;
      marker.points.push_back(point);
    }
    marker_array.markers.push_back(std::move(marker));
  }

  if (publish_candidates) {
    candidate_paths_pub_->publish(marker_array);
  }
  if (publish_rollouts) {
    rollouts_pub_->publish(marker_array);
  }
}

bool FrenetLocalPlanner::computeLocalPlan(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav_msgs::msg::Path & local_path)
{
  std::lock_guard<std::mutex> lock(mtx_);
  local_path = nav_msgs::msg::Path();
  const rclcpp::Time stamp = clock_->now();
  if (!frenet_planner_ || raw_global_plan_.poses.size() < 2U ||
    cumulative_arc_.size() != raw_global_plan_.poses.size())
  {
    return false;
  }

  PreparedPlan prepared = preparePlanInCostmapFrame(pose);
  if (prepared.path.poses.size() < 2U ||
    prepared.cumulative_arc.size() != prepared.path.poses.size())
  {
    return false;
  }

  double lower_arc = 0.0;
  double upper_arc = prepared.cumulative_arc.back();
  if (last_projection_index_ >= 0 &&
    static_cast<std::size_t>(last_projection_index_) < prepared.cumulative_arc.size())
  {
    const double previous_arc =
      prepared.cumulative_arc[static_cast<std::size_t>(last_projection_index_)];
    lower_arc = std::max(0.0, previous_arc - max_backtrack_distance_);
    upper_arc =
      std::min(prepared.cumulative_arc.back(), previous_arc + projection_search_distance_);
  }
  Projection projection =
    projectOntoPath(prepared.path, prepared.cumulative_arc, prepared.pose, lower_arc, upper_arc);
  if (!projection.valid) {
    projection =
      projectOntoPath(
      prepared.path, prepared.cumulative_arc, prepared.pose, 0.0,
      prepared.cumulative_arc.back());
  }
  if (!projection.valid) {
    return false;
  }
  last_projection_index_ = static_cast<int>(projection.segment_index);

  const auto waypoints = buildFrenetWaypoints(prepared.path, projection);
  if (waypoints.size() < 2U) {
    if (!prepared.path.poses.empty()) {
      local_path.header.frame_id = prepared.path.header.frame_id;
      local_path.header.stamp = stamp;
      local_path.poses.push_back(prepared.path.poses.back());
      return true;
    }
    return false;
  }

  // 终点保护：若距终点剩余弧长 < 0.25m，直接输出直通局部路径，交由高精度 GoalChecker 停靠
  const double remaining_arc = prepared.cumulative_arc.back() - projection.arc_length;
  if (remaining_arc < 0.25) {
    local_path.header.frame_id = prepared.path.header.frame_id;
    local_path.header.stamp = stamp;
    local_path.poses.reserve(prepared.path.poses.size() - projection.segment_index + 1U);
    geometry_msgs::msg::PoseStamped proj_pose = prepared.pose;
    proj_pose.pose.position.x = projection.point.x();
    proj_pose.pose.position.y = projection.point.y();
    local_path.poses.push_back(std::move(proj_pose));
    for (std::size_t i = projection.segment_index + 1U; i < prepared.path.poses.size(); ++i) {
      local_path.poses.push_back(prepared.path.poses[i]);
    }
    return true;
  }

  const auto robot_state = makeCartesianState(prepared.pose, velocity);
  auto debug_info = std::make_shared<frenet_planner::DebugInfo>();
  const auto result = frenet_planner_->planByWaypoint(robot_state, waypoints, debug_info);
  if (result.cartesian.size() < 2U) {
    int start_cost = -1;
    unsigned int start_mx = 0U;
    unsigned int start_my = 0U;
    if (costmap_ && costmap_->worldToMap(robot_state[0], robot_state[3], start_mx, start_my)) {
      start_cost = static_cast<int>(costmap_->getCost(start_mx, start_my));
    }

    std::ostringstream reject_counts;
    if (debug_info->policy_reject_counts.empty()) {
      reject_counts << "0";
    } else {
      for (std::size_t i = 0U; i < debug_info->policy_reject_counts.size(); ++i) {
        if (i > 0U) {
          reject_counts << ",";
        }
        reject_counts << debug_info->policy_reject_counts[i];
      }
    }

    std::ostringstream first_reject;
    if (debug_info->has_first_reject) {
      first_reject << "first_reject_policy=" << debug_info->first_reject_policy;
      if (debug_info->first_reject_violation.valid) {
        first_reject << ", violation(x=" << debug_info->first_reject_violation.x
                     << ", y=" << debug_info->first_reject_violation.y
                     << ", cost=" << debug_info->first_reject_violation.cost
                     << ", speed=" << debug_info->first_reject_violation.speed
                     << ", acc=" << debug_info->first_reject_violation.acceleration
                     << ", non_finite=" << (debug_info->first_reject_violation.non_finite ? "true" : "false")
                     << ")";
      }
    } else {
      first_reject << "first_reject=none";
    }

    std::ostringstream diagnostic;
    diagnostic << "projection_s=" << projection.arc_length <<
      ", d=" << projection.lateral_offset <<
      ", segment=" << projection.segment_index <<
      ", raw_poses=" << raw_global_plan_.poses.size() <<
      ", prepared_poses=" << prepared.path.poses.size() <<
      ", path_arc=" << prepared.cumulative_arc.back() <<
      ", waypoints=" << waypoints.size() <<
      ", transformed=" << (prepared.transformed ? "true" : "false") <<
      ", raw_frame='" << raw_global_plan_.header.frame_id << "'" <<
      ", costmap_frame='" << prepared.path.header.frame_id << "'" <<
      ", robot_xy=(" << robot_state[0] << "," << robot_state[3] << ")" <<
      ", start_cost=" << start_cost <<
      ", robot_frenet_s=" << debug_info->robot_frenet_s <<
      ", first_candidate_s=" << debug_info->first_candidate_s <<
      ", segment_count=" << debug_info->segment_count <<
      ", frenet_total_length=" << debug_info->frenet_total_length <<
      ", first_cartesian_size=" << debug_info->first_cartesian_size <<
      ", candidates=" << debug_info->candidate_count <<
      ", short=" << debug_info->short_candidate_count <<
      ", invalid_cartesian=" << debug_info->invalid_cartesian_count <<
      ", feasible=" << debug_info->feasible_candidate_count <<
      ", nonfinite_cost=" << debug_info->nonfinite_cost_count <<
      ", policy_rejects=[" << reject_counts.str() << "]" <<
      ", " << first_reject.str();
    const std::string diagnostic_text = diagnostic.str();
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "Frenet planner found no feasible trajectory: %s", diagnostic_text.c_str());
    publishCandidates(*debug_info, prepared.path.header.frame_id, stamp);
    return false;
  }

  local_path.header.frame_id = prepared.path.header.frame_id;
  local_path.header.stamp = stamp;

  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 500,
    "Frenet selected best trajectory: len=%zu, final_s=%.2f, final_d=%.2f, feasible=%zu/%zu",
    result.cartesian.size(),
    result.frenet.empty() ? 0.0 : result.frenet.back()[0],
    result.frenet.empty() ? 0.0 : result.frenet.back()[3],
    debug_info->feasible_candidate_count,
    debug_info->candidate_count);
  local_path.poses.reserve(result.cartesian.size() + prepared.path.poses.size());
  for (std::size_t index = 0U; index < result.cartesian.size(); ++index) {
    geometry_msgs::msg::PoseStamped trajectory_pose;
    const rclcpp::Time pose_stamp =
      stamp + rclcpp::Duration::from_seconds(static_cast<double>(index) * frenet_config_.dt);
    setPoseFromCartesian(
      result.cartesian[index], prepared.path.header.frame_id, pose_stamp,
      trajectory_pose);
    local_path.poses.push_back(std::move(trajectory_pose));
  }

  appendPathTail(
    result, prepared.path, prepared.cumulative_arc, projection, prepared.path.header.frame_id,
    stamp,
    local_path);
  publishCandidates(*debug_info, prepared.path.header.frame_id, stamp);
  return local_path.poses.size() >= 1U;
}

}  // namespace nav2_frenet_ilqr_controller

PLUGINLIB_EXPORT_CLASS(nav2_frenet_ilqr_controller::FrenetLocalPlanner, nav2_core::LocalPlanner)
