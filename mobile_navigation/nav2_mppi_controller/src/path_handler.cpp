// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Dexory
// Copyright (c) 2023 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_mppi_controller/tools/path_handler.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // tf2::doTransform(PoseStamped) 特化
#include "tf2/utils.h"  // tf2::getYaw
#include "angles/angles.h"

#include <cmath>
#include <iterator>
#include <stdexcept>

namespace mppi
{

void PathHandler::initialize(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap,
  std::shared_ptr<tf2_ros::Buffer> buffer, ParametersHandler * param_handler)
{
  name_ = name;
  costmap_ = costmap;
  tf_buffer_ = buffer;
  auto node = parent.lock();
  logger_ = node->get_logger();
  parameters_handler_ = param_handler;

  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(max_robot_pose_search_dist_, "max_robot_pose_search_dist", getMaxCostmapDist());
  getParam(prune_distance_, "prune_distance", 1.5);
  getParam(transform_tolerance_, "transform_tolerance", 0.1);
  getParam(enforce_path_inversion_, "enforce_path_inversion", false);
  if (enforce_path_inversion_) {
    getParam(inversion_xy_tolerance_, "inversion_xy_tolerance", 0.2);
    getParam(inversion_yaw_tolerance, "inversion_yaw_tolerance", 0.4);
    inversion_locale_ = 0u;
  }
}

std::pair<nav_msgs::msg::Path, PathIterator>
PathHandler::getGlobalPlanConsideringBoundsInCostmapFrame(
  const geometry_msgs::msg::PoseStamped & global_pose)
{
  using nav2_util::geometry_utils::euclidean_distance;

  auto begin = global_plan_up_to_inversion_.poses.begin();

  // Limit the search for the closest pose up to max_robot_pose_search_dist on the path
  auto closest_pose_upper_bound =
    nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_up_to_inversion_.poses.begin(), global_plan_up_to_inversion_.poses.end(),
    max_robot_pose_search_dist_);

  // Find closest point to the robot
  auto closest_point = nav2_util::geometry_utils::min_by(
    begin, closest_pose_upper_bound,
    [&global_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(global_pose, ps);
    });

  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header.frame_id = costmap_->getGlobalFrameID();
  transformed_plan.header.stamp = global_pose.header.stamp;

  auto pruned_plan_end =
    nav2_util::geometry_utils::first_after_integrated_distance(
    closest_point, global_plan_up_to_inversion_.poses.end(), prune_distance_);

  // ⭐ 性能优化：循环外一次性查 plan_frame→costmap_frame 的 TF，循环内用 doTransform 纯矩阵
  // 批量变换，避免对窗口内每个路径点都走 tf_buffer_->transform 查询（原逐点查询 ~10ms）
  const std::string costmap_frame = costmap_->getGlobalFrameID();
  const std::string plan_frame = global_plan_.header.frame_id;
  const bool same_frame = (plan_frame == costmap_frame);
  geometry_msgs::msg::TransformStamped plan_to_costmap_tf;
  if (!same_frame) {
    try {
      plan_to_costmap_tf = tf_buffer_->lookupTransform(
        costmap_frame, plan_frame, global_pose.header.stamp,
        tf2::durationFromSec(transform_tolerance_));
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(
        logger_,
        "Exception in lookupTransform: %s. TF lookup failed; cannot transform global path into costmap frame.",
        ex.what());
      return {transformed_plan, closest_point};
    }
  }

  unsigned int mx, my;
  // Find the furthest relevent pose on the path to consider within costmap
  // bounds, transforming each into the costmap frame in the same loop.
  for (auto global_plan_pose = closest_point; global_plan_pose != pruned_plan_end;
    ++global_plan_pose)
  {
    geometry_msgs::msg::PoseStamped costmap_plan_pose;
    if (same_frame) {
      costmap_plan_pose = *global_plan_pose;
    } else {
      geometry_msgs::msg::PoseStamped in_pose = *global_plan_pose;
      in_pose.header.frame_id = plan_frame;
      in_pose.header.stamp = global_pose.header.stamp;
      tf2::doTransform(in_pose, costmap_plan_pose, plan_to_costmap_tf);  // 纯矩阵，不查 buffer
    }
    costmap_plan_pose.header.frame_id = costmap_frame;
    costmap_plan_pose.header.stamp = global_pose.header.stamp;

    // Check if pose is inside the costmap
    if (!costmap_->getCostmap()->worldToMap(
        costmap_plan_pose.pose.position.x, costmap_plan_pose.pose.position.y, mx, my))
    {
      return {transformed_plan, closest_point};
    }

    // Filling the transformed plan to return with the transformed pose
    transformed_plan.poses.push_back(costmap_plan_pose);
  }

  return {transformed_plan, closest_point};
}

geometry_msgs::msg::PoseStamped PathHandler::transformToGlobalPlanFrame(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_up_to_inversion_.poses.empty()) {
    throw std::runtime_error(
            "Received plan with zero length. Cannot transform an empty global path for local control.");
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_up_to_inversion_.header.frame_id, pose, robot_pose)) {
    throw std::runtime_error(
            "Unable to transform robot pose into global plan's frame. Check TF.");
  }

  return robot_pose;
}

nav_msgs::msg::Path PathHandler::transformPath(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  // Find relevent bounds of path to use
  geometry_msgs::msg::PoseStamped global_pose =
    transformToGlobalPlanFrame(robot_pose);
  updateGoalDistance(global_pose);
  auto [transformed_plan, lower_bound] = getGlobalPlanConsideringBoundsInCostmapFrame(global_pose);
  updateHeadingPathAngle(global_pose, lower_bound);
  const double pruned_distance =
    getIntegratedDistance(global_plan_up_to_inversion_.poses.begin(), lower_bound);
  last_path_progress_ = path_progress_offset_ + pruned_distance;
  path_progress_offset_ += pruned_distance;
  RCLCPP_DEBUG(
    logger_,
    "MPPI path state: path_progress=%.3f goal_distance=%.3f pruned_distance=%.3f "
    "remaining_global_plan=%zu transformed_plan=%zu. "
    "These values decide whether currently in the start, middle, or final stage.",
    last_path_progress_, last_goal_distance_, pruned_distance,
    global_plan_up_to_inversion_.poses.size(), transformed_plan.poses.size());

  prunePlan(global_plan_up_to_inversion_, lower_bound);

  if (enforce_path_inversion_ && inversion_locale_ != 0u) {
    if (isWithinInversionTolerances(global_pose)) {
      prunePlan(global_plan_, global_plan_.poses.begin() + inversion_locale_);
      global_plan_up_to_inversion_ = global_plan_;
      inversion_locale_ = utils::removePosesAfterFirstInversion(global_plan_up_to_inversion_);
      path_progress_offset_ = 0.0;
      last_path_progress_ = 0.0;
      RCLCPP_INFO(
        logger_,
        "MPPI path inversion reached, reset path progress. "
        "Path reversing point passed; three-stage progress restarts from the current sub-path.");
    }
  }

  if (transformed_plan.poses.empty()) {
    throw std::runtime_error(
            "Resulting plan has 0 poses in it. Path empty after transform to local costmap; global path may be empty, outside costmap, or pruned to no valid points.");
  }

  return transformed_plan;
}

bool PathHandler::transformPose(
  const std::string & frame, const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_buffer_->transform(
      in_pose, out_pose, frame,
      tf2::durationFromSec(transform_tolerance_));
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      logger_,
      "Exception in transformPose: %s. TF transform failed; MPPI cannot transform robot pose or path to target frame.",
      ex.what());
  }
  return false;
}

double PathHandler::getMaxCostmapDist()
{
  const auto & costmap = costmap_->getCostmap();
  return static_cast<double>(std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY())) *
         costmap->getResolution() * 0.50;
}

void PathHandler::setPath(const nav_msgs::msg::Path & plan)
{
  global_plan_ = plan;
  global_plan_up_to_inversion_ = global_plan_;
  path_progress_offset_ = 0.0;
  last_path_progress_ = 0.0;
  last_goal_distance_ = 0.0;
  last_heading_path_angle_ = 0.0;
  RCLCPP_INFO(
    logger_,
    "MPPI new global plan received: poses=%zu. "
    "Reset three-stage path_progress and goal_distance stats.",
    global_plan_.poses.size());
  if (enforce_path_inversion_) {
    inversion_locale_ = utils::removePosesAfterFirstInversion(global_plan_up_to_inversion_);
  }
}

nav_msgs::msg::Path & PathHandler::getPath() {return global_plan_;}

double PathHandler::getLastGoalDistance() const
{
  return last_goal_distance_;
}

double PathHandler::getLastPathProgress() const
{
  return last_path_progress_;
}

double PathHandler::getLastHeadingPathAngle() const
{
  return last_heading_path_angle_;
}

void PathHandler::updateHeadingPathAngle(
  const geometry_msgs::msg::PoseStamped & global_pose, const PathIterator & closest)
{
  auto & poses = global_plan_up_to_inversion_.poses;
  if (poses.empty() || closest == poses.end()) {
    last_heading_path_angle_ = 0.0;
    return;
  }
  // 路径在最近点的切线方向：取最近点到前方第 3 个点的方向（更稳，避免单点噪声）
  const std::ptrdiff_t remaining = std::distance(closest, poses.end());
  const PathIterator ahead = closest + (remaining > 3 ? 3 : (remaining - 1));
  double path_yaw;
  if (ahead != closest) {
    path_yaw = std::atan2(
      ahead->pose.position.y - closest->pose.position.y,
      ahead->pose.position.x - closest->pose.position.x);
  } else {
    path_yaw = tf2::getYaw(closest->pose.orientation);
  }
  const double robot_yaw = tf2::getYaw(global_pose.pose.orientation);
  last_heading_path_angle_ = std::fabs(angles::shortest_angular_distance(robot_yaw, path_yaw));
}

void PathHandler::updateGoalDistance(const geometry_msgs::msg::PoseStamped & global_pose)
{
  if (global_plan_.poses.empty()) {
    throw std::runtime_error(
            "Received plan with zero length. Cannot compute robot-to-goal distance from an empty path.");
  }

  const auto & goal_pose = global_plan_.poses.back();
  last_goal_distance_ = std::hypot(
    global_pose.pose.position.x - goal_pose.pose.position.x,
    global_pose.pose.position.y - goal_pose.pose.position.y);
}

double PathHandler::getIntegratedDistance(PathIterator begin, PathIterator end) const
{
  double distance = 0.0;
  if (begin == end) {
    return distance;
  }

  auto previous = begin;
  for (auto current = begin + 1; current <= end; ++current) {
    distance += std::hypot(
      current->pose.position.x - previous->pose.position.x,
      current->pose.position.y - previous->pose.position.y);
    previous = current;
  }

  return distance;
}

void PathHandler::prunePlan(nav_msgs::msg::Path & plan, const PathIterator end)
{
  plan.poses.erase(plan.poses.begin(), end);
}

bool PathHandler::isWithinInversionTolerances(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  // Keep full path if we are within tolerance of the inversion pose
  const auto last_pose = global_plan_up_to_inversion_.poses.back();
  float distance = hypotf(
    robot_pose.pose.position.x - last_pose.pose.position.x,
    robot_pose.pose.position.y - last_pose.pose.position.y);

  float angle_distance = angles::shortest_angular_distance(
    tf2::getYaw(robot_pose.pose.orientation),
    tf2::getYaw(last_pose.pose.orientation));

  return distance <= inversion_xy_tolerance_ && fabs(angle_distance) <= inversion_yaw_tolerance;
}

}  // namespace mppi
