// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 实现 nav2_core::LocalPlanner。
//   - configure() + activate()/deactivate()/cleanup()
//   - 参数加载用 PlanningParams::declareAndLoad(node, name)
//   - setPlan(nav_msgs::msg::Path)
//   - computeLocalPlan：robot pose 由入参 pose 提供；当前速度由入参 velocity 提供
//   - transformGlobalPlan：tf 用 shared_ptr<tf2_ros::Buffer>，异常类型 tf2::TransformException
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_lex_local_planner/lex_local_planner.hpp"
#include "nav2_lex_local_planner/utility.hpp"

namespace nav2_lex_local_planner
{

LexLocalPlanner::LexLocalPlanner()
: costmap_(nullptr)
{
}

LexLocalPlanner::~LexLocalPlanner()
{
}

void LexLocalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  global_frame_ = costmap_ros_->getGlobalFrameID();
  robot_base_frame_ = costmap_ros_->getBaseFrameID();

  // Step: 1. 加载参数（ROS2 declare/get 替代 ROS1 NodeHandle::param）
  planning_params_ = std::make_shared<PlanningParams>();
  planning_params_->declareAndLoad(node, name_);
  planning_params_->print(logger_);

  // Step: 2. 构造规划核心（LexPlanner 用 nav2_costmap_2d::Costmap2D*）
  rollout_generator_ = std::make_unique<RolloutGenerator>(planning_params_);
  lex_planner_ = std::make_unique<LexPlanner>(costmap_, planning_params_);

  RCLCPP_INFO(logger_, "nav2_lex_local_planner plugin configured: %s", name_.c_str());
}

void LexLocalPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up nav2_lex_local_planner: %s", name_.c_str());
  rollout_generator_.reset();
  lex_planner_.reset();
  planning_params_.reset();
}

void LexLocalPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating nav2_lex_local_planner: %s", name_.c_str());
}

void LexLocalPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating nav2_lex_local_planner: %s", name_.c_str());
}

void LexLocalPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path.poses;
}

bool LexLocalPlanner::computeLocalPlan(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav_msgs::msg::Path & local_path)
{
  std::lock_guard<std::mutex> lock(mtx_);

  local_path.poses.clear();
  local_path.header.frame_id = global_frame_;
  local_path.header.stamp = clock_->now();

  if (global_plan_.empty()) {
    RCLCPP_ERROR(logger_, "Global plan is empty, cannot create local plan");
    return false;
  }

  // 起终点重合：直接透传全局路径
  if (global_plan_.size() >= 2 &&
      nav2_util::geometry_utils::euclidean_distance(global_plan_.front(), global_plan_.back()) < 0.1)
  {
    local_path.poses = global_plan_;
    return true;
  }

  const geometry_msgs::msg::PoseStamped & robot_pose = pose;

  std::vector<geometry_msgs::msg::PoseStamped> transformed_plan;
  int goal_idx;
  geometry_msgs::msg::TransformStamped tf_plan_to_global;

  if (!transformGlobalPlan(global_plan_, robot_pose, global_frame_,
                           planning_params_->microPlanDistance,
                           transformed_plan, &goal_idx, &tf_plan_to_global))
  {
    RCLCPP_WARN(logger_, "Could not transform the routing plan to the frame of the global plan");
    return false;
  }

  geometry_msgs::msg::PoseStamped global_goal;
  tf2::doTransform(global_plan_.back(), global_goal, tf_plan_to_global);

  if (transformed_plan.size() == 0) {
    local_path.poses.push_back(global_goal);
    return true;
  }

  if (transformed_plan.size() >= 2 &&
      nav2_util::geometry_utils::euclidean_distance(transformed_plan.front(), transformed_plan.back()) < 0.1)
  {
    local_path.poses = transformed_plan;
    local_path.poses.back() = global_goal;
    return true;
  }

  global_waypoints_.clear();
  nav2_lex_local_planner::poseVec2LocalLane(transformed_plan, global_waypoints_);

  nav2_lex_local_planner::WayPoint current_pose;
  nav2_lex_local_planner::poseStampedMsgToWayPoint(robot_pose, current_pose);

  double curr_speed = std::hypot(velocity.linear.x, velocity.linear.y);

  std::vector<std::vector<nav2_lex_local_planner::WayPoint>> alternative_paths;
  rollout_generator_->Run(current_pose, curr_speed, global_waypoints_, alternative_paths);

  std::vector<nav2_lex_local_planner::WayPoint> final_path;
  lex_planner_->findPath(alternative_paths, final_path);

  if (final_path.empty()) {
    RCLCPP_WARN(logger_, "The lex planner cannot get a valid path.");
    return false;
  }

  std::vector<geometry_msgs::msg::PoseStamped> local_poses;
  nav2_lex_local_planner::localLane2PoseVec(final_path, local_poses, global_frame_);
  local_path.poses = local_poses;

  if (goal_idx >= static_cast<int>(global_plan_.size()) - 1) {
    local_path.poses.back() = global_goal;
  }

  return final_path.size() > 0;
}

bool LexLocalPlanner::transformGlobalPlan(
  const std::vector<geometry_msgs::msg::PoseStamped> & global_plan,
  const geometry_msgs::msg::PoseStamped & global_pose,
  const std::string & global_frame, double max_plan_length,
  std::vector<geometry_msgs::msg::PoseStamped> & transformed_plan,
  int * current_goal_idx, geometry_msgs::msg::TransformStamped * tf_plan_to_global) const
{
  transformed_plan.clear();

  try {
    if (global_plan.empty()) {
      RCLCPP_ERROR(logger_, "Received plan with zero length");
      if (current_goal_idx) *current_goal_idx = 0;
      return false;
    }

    const geometry_msgs::msg::PoseStamped & plan_pose = global_plan[0];

    // get plan_to_global_transform from plan frame to global_frame
    geometry_msgs::msg::TransformStamped plan_to_global_transform = tf_->lookupTransform(
      global_frame, tf2::TimePointZero, plan_pose.header.frame_id,
      tf2_ros::fromMsg(plan_pose.header.stamp),
      plan_pose.header.frame_id, tf2::durationFromSec(0.5));

    // let's get the pose of the robot in the frame of the plan
    geometry_msgs::msg::PoseStamped robot_pose;
    tf_->transform(global_pose, robot_pose, plan_pose.header.frame_id);

    transformed_plan.push_back(robot_pose);

    int i = 0;
    double sq_dist = 1e10;

    // we need to loop to a point on the plan that is within a certain distance of the robot
    bool robot_reached = false;
    for (int j = 0; j < static_cast<int>(global_plan.size()); ++j) {
      double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
      double new_sq_dist = x_diff * x_diff + y_diff * y_diff;

      if (robot_reached && new_sq_dist > sq_dist)
        break;

      if (new_sq_dist < sq_dist) {  // find closest distance
        sq_dist = new_sq_dist;
        i = j;
        if (sq_dist < 0.05)        // 2.5 cm to the robot; take the immediate local minima
          robot_reached = true;
      }
    }

    geometry_msgs::msg::PoseStamped newer_pose;

    double plan_length = 0;  // check cumulative Euclidean distance along the plan

    // now we'll transform until points are outside of our distance threshold
    while (i < static_cast<int>(global_plan.size()) &&
           (max_plan_length <= 0 || plan_length <= max_plan_length))
    {
      const geometry_msgs::msg::PoseStamped & pose = global_plan[i];
      tf2::doTransform(pose, newer_pose, plan_to_global_transform);

      transformed_plan.push_back(newer_pose);

      // calculate distance to previous pose
      if (i > 0 && max_plan_length > 0)
        plan_length += distance2points(global_plan[i - 1].pose.position, global_plan[i].pose.position);

      ++i;
    }

    // if we are really close to the goal and the goal is not yet reached the resulting
    // transformed plan can be empty. In that case we explicitly inject the global goal.
    if (transformed_plan.empty()) {
      tf2::doTransform(global_plan.back(), newer_pose, plan_to_global_transform);

      transformed_plan.push_back(newer_pose);

      if (current_goal_idx) *current_goal_idx = static_cast<int>(global_plan.size()) - 1;
    } else {
      // subtract 1, since i was increased once before leaving the loop
      if (current_goal_idx) *current_goal_idx = i - 1;
    }

    // Return the transformation from the global plan to the global planning frame if desired
    if (tf_plan_to_global) {
      *tf_plan_to_global = plan_to_global_transform;
    }
  } catch (tf2::LookupException & ex) {
    RCLCPP_ERROR(logger_, "No Transform available Error: %s\n", ex.what());
    return false;
  } catch (tf2::ConnectivityException & ex) {
    RCLCPP_ERROR(logger_, "Connectivity Error: %s\n", ex.what());
    return false;
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(logger_, "Extrapolation Error: %s\n", ex.what());
    if (global_plan.size() > 0) {
      RCLCPP_ERROR(
        logger_, "Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(),
        static_cast<unsigned int>(global_plan.size()), global_plan[0].header.frame_id.c_str());
    }
    return false;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Transform Error: %s\n", ex.what());
    return false;
  }

  return true;
}

}  // namespace nav2_lex_local_planner

// register this planner as a nav2_core::LocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(nav2_lex_local_planner::LexLocalPlanner, nav2_core::LocalPlanner)
