// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include "nav2_path_pruning_local_planner/path_pruning_local_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_path_pruning_local_planner
{

namespace
{
constexpr double kDistanceEpsilonSq = 1e-8;
constexpr double kArcEpsilon = 1e-9;

bool orientationsEquivalent(
  const geometry_msgs::msg::Quaternion & lhs,
  const geometry_msgs::msg::Quaternion & rhs)
{
  const double direct_distance_sq =
    std::pow(lhs.x - rhs.x, 2) + std::pow(lhs.y - rhs.y, 2) +
    std::pow(lhs.z - rhs.z, 2) + std::pow(lhs.w - rhs.w, 2);
  const double negated_distance_sq =
    std::pow(lhs.x + rhs.x, 2) + std::pow(lhs.y + rhs.y, 2) +
    std::pow(lhs.z + rhs.z, 2) + std::pow(lhs.w + rhs.w, 2);
  return std::min(direct_distance_sq, negated_distance_sq) <= kArcEpsilon;
}
}  // namespace

void PathPruningLocalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS>/*costmap_ros*/)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("PathPruningLocalPlanner parent node expired");
  }

  node_ = parent;
  tf_ = std::move(tf);
  plugin_name_ = std::move(name);
  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_search_distance", rclcpp::ParameterValue(5.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_backtrack_distance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.2));
  node->get_parameter(plugin_name_ + ".max_search_distance", max_search_distance_);
  node->get_parameter(plugin_name_ + ".max_backtrack_distance", max_backtrack_distance_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);

  max_search_distance_ = std::max(max_search_distance_, 0.01);
  max_backtrack_distance_ = std::max(max_backtrack_distance_, 0.0);
  transform_tolerance_ = std::max(transform_tolerance_, 0.0);

  RCLCPP_INFO(
    logger_,
    "Configured path-pruning local planner '%s': search=%.2f m backtrack=%.2f m",
    plugin_name_.c_str(), max_search_distance_, max_backtrack_distance_);
}

void PathPruningLocalPlanner::cleanup()
{
  std::lock_guard<std::mutex> lock(mutex_);
  global_plan_ = nav_msgs::msg::Path();
  cumulative_arc_.clear();
  progress_initialized_ = false;
  progress_arc_ = 0.0;
}

void PathPruningLocalPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating path-pruning local planner: %s", plugin_name_.c_str());
}

void PathPruningLocalPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating path-pruning local planner: %s", plugin_name_.c_str());
}

void PathPruningLocalPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  global_plan_ = path;
  plan_frame_ = path.header.frame_id;
  if (plan_frame_.empty() && !path.poses.empty()) {
    plan_frame_ = path.poses.front().header.frame_id;
    global_plan_.header.frame_id = plan_frame_;
  }
  rebuildArcLengths();
  progress_initialized_ = false;
  progress_arc_ = 0.0;
}

bool PathPruningLocalPlanner::computeLocalPlan(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav_msgs::msg::Path & local_path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  local_path = nav_msgs::msg::Path();

  if (global_plan_.poses.empty()) {
    RCLCPP_ERROR(logger_, "Cannot prune an empty global plan");
    return false;
  }
  if (plan_frame_.empty() || pose.header.frame_id.empty()) {
    RCLCPP_ERROR(logger_, "Path and robot pose must both have a frame_id");
    return false;
  }

  geometry_msgs::msg::PoseStamped robot_in_plan;
  try {
    if (pose.header.frame_id == plan_frame_) {
      robot_in_plan = pose;
    } else {
      if (!tf_) {
        RCLCPP_ERROR(logger_, "No TF buffer available to transform robot pose");
        return false;
      }
      tf_->transform(
        pose, robot_in_plan, plan_frame_,
        tf2::durationFromSec(transform_tolerance_));
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      logger_, "Cannot transform robot pose from '%s' to path frame '%s': %s",
      pose.header.frame_id.c_str(), plan_frame_.c_str(), ex.what());
    return false;
  }

  local_path.header = global_plan_.header;
  local_path.header.frame_id = plan_frame_;
  if (global_plan_.poses.size() == 1) {
    local_path.poses.push_back(global_plan_.poses.back());
    progress_initialized_ = true;
    progress_arc_ = 0.0;
    return true;
  }
  if (cumulative_arc_.back() <= kArcEpsilon) {
    // A zero-length plan can still carry a terminal orientation sequence. It is
    // already local to the robot, so preserve every pose for the controller.
    local_path.poses = global_plan_.poses;
    progress_initialized_ = true;
    progress_arc_ = 0.0;
    return true;
  }

  const double total_arc = cumulative_arc_.back();
  const double lower_arc = progress_initialized_ ?
    std::max(0.0, progress_arc_ - max_backtrack_distance_) : 0.0;
  const double upper_arc = progress_initialized_ ?
    std::min(total_arc, progress_arc_ + max_search_distance_) : total_arc;
  const Projection projection = projectOntoPlan(robot_in_plan, lower_arc, upper_arc);
  if (!projection.valid) {
    RCLCPP_ERROR(logger_, "No valid segment found in path-pruning search window");
    return false;
  }

  progress_arc_ = progress_initialized_ ?
    std::max(progress_arc_, projection.arc_length) : projection.arc_length;
  progress_initialized_ = true;

  size_t start_segment = 0;
  auto projected_start = poseAtArcLength(progress_arc_, start_segment);
  projected_start.header.frame_id = plan_frame_;
  projected_start.header.stamp = robot_in_plan.header.stamp;
  local_path.poses.push_back(projected_start);

  for (size_t i = start_segment + 1; i < global_plan_.poses.size(); ++i) {
    const auto & candidate = global_plan_.poses[i];
    const auto & previous = local_path.poses.back().pose.position;
    const double dx = candidate.pose.position.x - previous.x;
    const double dy = candidate.pose.position.y - previous.y;
    if (dx * dx + dy * dy < kDistanceEpsilonSq) {
      const auto & previous_pose = local_path.poses.back().pose;
      const bool same_pose =
        std::fabs(candidate.pose.position.z - previous_pose.position.z) < kArcEpsilon &&
        orientationsEquivalent(candidate.pose.orientation, previous_pose.orientation);
      if (!same_pose ||
        (i + 1 == global_plan_.poses.size() && local_path.poses.size() == 1))
      {
        // Same-position poses may encode terminal rotation. A two-pose
        // projected-start/goal path also has distinct state and goal roles.
        local_path.poses.push_back(candidate);
        continue;
      }
      if (i + 1 == global_plan_.poses.size()) {
        local_path.poses.back() = candidate;
      }
      continue;
    }
    local_path.poses.push_back(candidate);
  }

  const auto & true_goal = global_plan_.poses.back();
  if (local_path.poses.back() != true_goal) {
    local_path.poses.push_back(true_goal);
  }
  return !local_path.poses.empty();
}

void PathPruningLocalPlanner::rebuildArcLengths()
{
  cumulative_arc_.assign(global_plan_.poses.size(), 0.0);
  for (size_t i = 1; i < global_plan_.poses.size(); ++i) {
    const auto & a = global_plan_.poses[i - 1].pose.position;
    const auto & b = global_plan_.poses[i].pose.position;
    cumulative_arc_[i] = cumulative_arc_[i - 1] + std::hypot(b.x - a.x, b.y - a.y);
  }
}

PathPruningLocalPlanner::Projection PathPruningLocalPlanner::projectOntoPlan(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  double lower_arc, double upper_arc) const
{
  Projection best;
  best.distance_sq = std::numeric_limits<double>::max();
  const double px = robot_pose.pose.position.x;
  const double py = robot_pose.pose.position.y;

  for (size_t i = 0; i + 1 < global_plan_.poses.size(); ++i) {
    const double segment_start_arc = cumulative_arc_[i];
    const double segment_end_arc = cumulative_arc_[i + 1];
    const double segment_length = segment_end_arc - segment_start_arc;
    if (segment_length <= kArcEpsilon || segment_end_arc < lower_arc - kArcEpsilon ||
      segment_start_arc > upper_arc + kArcEpsilon)
    {
      continue;
    }

    const auto & a = global_plan_.poses[i].pose.position;
    const auto & b = global_plan_.poses[i + 1].pose.position;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_sq = dx * dx + dy * dy;
    const double min_ratio = std::clamp(
      (lower_arc - segment_start_arc) / segment_length, 0.0, 1.0);
    const double max_ratio = std::clamp(
      (upper_arc - segment_start_arc) / segment_length, 0.0, 1.0);
    const double raw_ratio = ((px - a.x) * dx + (py - a.y) * dy) / length_sq;
    const double ratio = std::clamp(raw_ratio, min_ratio, max_ratio);
    const double projected_x = a.x + ratio * dx;
    const double projected_y = a.y + ratio * dy;
    const double distance_sq =
      (px - projected_x) * (px - projected_x) +
      (py - projected_y) * (py - projected_y);
    const double arc_length = segment_start_arc + ratio * segment_length;

    if (!best.valid || distance_sq < best.distance_sq - kArcEpsilon ||
      (std::fabs(distance_sq - best.distance_sq) <= kArcEpsilon &&
      arc_length < best.arc_length))
    {
      best.valid = true;
      best.segment_index = i;
      best.ratio = ratio;
      best.arc_length = arc_length;
      best.distance_sq = distance_sq;
    }
  }
  return best;
}

geometry_msgs::msg::PoseStamped PathPruningLocalPlanner::poseAtArcLength(
  double arc_length, size_t & segment_index) const
{
  if (arc_length >= cumulative_arc_.back() - kArcEpsilon) {
    segment_index = global_plan_.poses.size() - 2;
    return global_plan_.poses.back();
  }

  auto upper = std::upper_bound(cumulative_arc_.begin(), cumulative_arc_.end(), arc_length);
  segment_index = static_cast<size_t>(std::distance(cumulative_arc_.begin(), upper) - 1);
  segment_index = std::min(segment_index, global_plan_.poses.size() - 2);
  const double segment_length = cumulative_arc_[segment_index + 1] - cumulative_arc_[segment_index];
  const double ratio = segment_length > kArcEpsilon ?
    (arc_length - cumulative_arc_[segment_index]) / segment_length : 0.0;
  return interpolatePose(segment_index, std::clamp(ratio, 0.0, 1.0));
}

geometry_msgs::msg::PoseStamped PathPruningLocalPlanner::interpolatePose(
  size_t segment_index, double ratio) const
{
  const auto & a = global_plan_.poses[segment_index];
  const auto & b = global_plan_.poses[segment_index + 1];
  geometry_msgs::msg::PoseStamped result = a;
  result.header.frame_id = plan_frame_;
  result.pose.position.x = a.pose.position.x + ratio * (b.pose.position.x - a.pose.position.x);
  result.pose.position.y = a.pose.position.y + ratio * (b.pose.position.y - a.pose.position.y);
  result.pose.position.z = a.pose.position.z + ratio * (b.pose.position.z - a.pose.position.z);

  tf2::Quaternion qa;
  tf2::Quaternion qb;
  tf2::fromMsg(a.pose.orientation, qa);
  tf2::fromMsg(b.pose.orientation, qb);
  if (qa.length2() > kArcEpsilon && qb.length2() > kArcEpsilon) {
    qa.normalize();
    qb.normalize();
    result.pose.orientation = tf2::toMsg(qa.slerp(qb, ratio).normalized());
  }
  return result;
}

}  // namespace nav2_path_pruning_local_planner

PLUGINLIB_EXPORT_CLASS(
  nav2_path_pruning_local_planner::PathPruningLocalPlanner,
  nav2_core::LocalPlanner)
