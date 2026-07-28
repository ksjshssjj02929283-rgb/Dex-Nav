// Copyright (c) 2021 Samsung Research
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

#include <cmath>
#include <vector>
#include <string>
#include <set>
#include <memory>
#include <limits>
#include "nav2_bt_navigator/navigators/navigate_through_poses.hpp"

namespace nav2_bt_navigator
{

namespace
{

double getYaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

bool
NavigateThroughPosesNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
{
  start_time_ = rclcpp::Time(0);
  auto node = parent_node.lock();

  if (!node->has_parameter("goals_blackboard_id")) {
    node->declare_parameter("goals_blackboard_id", std::string("goals"));
  }

  goals_blackboard_id_ = node->get_parameter("goals_blackboard_id").as_string();

  if (!node->has_parameter("path_blackboard_id")) {
    node->declare_parameter("path_blackboard_id", std::string("path"));
  }

  path_blackboard_id_ = node->get_parameter("path_blackboard_id").as_string();

  // Odometry smoother object for getting current speed
  odom_smoother_ = odom_smoother;

  return true;
}

std::string
NavigateThroughPosesNavigator::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();

  if (!node->has_parameter("default_nav_through_poses_bt_xml")) {
    std::string pkg_share_dir =
      ament_index_cpp::get_package_share_directory("nav2_bt_navigator");
    node->declare_parameter<std::string>(
      "default_nav_through_poses_bt_xml",
      pkg_share_dir +
      "/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml");
  }

  node->get_parameter("default_nav_through_poses_bt_xml", default_bt_xml_filename);

  return default_bt_xml_filename;
}

bool
NavigateThroughPosesNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  auto bt_xml_filename = goal->behavior_tree;

  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(
      logger_, "Error loading XML file: %s. Navigation canceled.",
      bt_xml_filename.c_str());
    return false;
  }

  initializeGoalPoses(goal);

  return true;
}

void
NavigateThroughPosesNavigator::goalCompleted(
  typename ActionT::Result::SharedPtr /*result*/,
  const nav2_behavior_tree::BtStatus final_bt_status)
{
  if (final_bt_status != nav2_behavior_tree::BtStatus::SUCCEEDED) {
    return;
  }

  const auto goal = bt_action_server_->getCurrentGoal();
  if (!goal || goal->poses.empty()) {
    RCLCPP_WARN(logger_, "Multi-pose navigation succeeded without final goal metadata");
    return;
  }

  const auto & final_goal = goal->poses.back();
  const auto & pose = final_goal.pose;
  const double goal_yaw = getYaw(pose.orientation);
  const std::string goal_uuid =
    rclcpp_action::to_string(bt_action_server_->getCurrentGoalId());

  RCLCPP_INFO(
    logger_,
    "Multi-pose navigation reached: uuid=%s, frame=%s, goal=(%.3f,%.3f,%.4f), poses=%zu",
    goal_uuid.c_str(), final_goal.header.frame_id.c_str(),
    pose.position.x, pose.position.y, goal_yaw, goal->poses.size());
}

void
NavigateThroughPosesNavigator::onLoop()
{
  using namespace nav2_util::geometry_utils;  // NOLINT

  // action server feedback (pose, duration of task,
  // number of recoveries, and distance remaining to goal, etc)
  auto feedback_msg = std::make_shared<ActionT::Feedback>();

  auto blackboard = bt_action_server_->getBlackboard();

  Goals goal_poses;
  blackboard->get<Goals>(goals_blackboard_id_, goal_poses);

  if (goal_poses.size() == 0) {
    bt_action_server_->publishFeedback(feedback_msg);
    return;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  nav2_util::getCurrentPose(
    current_pose, *feedback_utils_.tf,
    feedback_utils_.global_frame, feedback_utils_.robot_frame,
    feedback_utils_.transform_tolerance);

  try {
    // Get current path points
    nav_msgs::msg::Path current_path;
    blackboard->get<nav_msgs::msg::Path>(path_blackboard_id_, current_path);

    // Find the closest pose to current pose on global path
    auto find_closest_pose_idx =
      [&current_pose, &current_path]() {
        size_t closest_pose_idx = 0;
        double curr_min_dist = std::numeric_limits<double>::max();
        for (size_t curr_idx = 0; curr_idx < current_path.poses.size(); ++curr_idx) {
          double curr_dist = nav2_util::geometry_utils::euclidean_distance(
            current_pose, current_path.poses[curr_idx]);
          if (curr_dist < curr_min_dist) {
            curr_min_dist = curr_dist;
            closest_pose_idx = curr_idx;
          }
        }
        return closest_pose_idx;
      };

    // Calculate distance on the path
    double distance_remaining =
      nav2_util::geometry_utils::calculate_path_length(current_path, find_closest_pose_idx());

    // Default value for time remaining
    rclcpp::Duration estimated_time_remaining = rclcpp::Duration::from_seconds(0.0);

    // Get current speed
    geometry_msgs::msg::Twist current_odom = odom_smoother_->getTwist();
    double current_linear_speed = std::hypot(current_odom.linear.x, current_odom.linear.y);

    // Calculate estimated time taken to goal if speed is higher than 1cm/s
    // and at least 10cm to go
    if ((std::abs(current_linear_speed) > 0.01) && (distance_remaining > 0.1)) {
      estimated_time_remaining =
        rclcpp::Duration::from_seconds(distance_remaining / std::abs(current_linear_speed));
    }

    feedback_msg->distance_remaining = distance_remaining;
    feedback_msg->estimated_time_remaining = estimated_time_remaining;
  } catch (...) {
    // Ignore
  }

  int recovery_count = 0;
  blackboard->get<int>("number_recoveries", recovery_count);
  feedback_msg->number_of_recoveries = recovery_count;
  feedback_msg->current_pose = current_pose;
  feedback_msg->navigation_time = clock_->now() - start_time_;
  feedback_msg->number_of_poses_remaining = goal_poses.size();

  bt_action_server_->publishFeedback(feedback_msg);
}

void
NavigateThroughPosesNavigator::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(logger_, "Received goal preemption request");

  if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() ||
    (goal->behavior_tree.empty() &&
    bt_action_server_->getCurrentBTFilename() == bt_action_server_->getDefaultBTFilename()))
  {
    // 新目标覆盖旧目标时，先停止旧行为树，确保旧 FollowPath 不再继续执行。
    bt_action_server_->haltTree();

    // 接受新目标时，SimpleActionServer 会将旧目标置为 ABORTED。
    const auto pending_goal = bt_action_server_->acceptPendingGoal();
    if (!pending_goal) {
      RCLCPP_ERROR(logger_, "Multi-pose navigation preemption failed: pending goal unavailable");
      return;
    }

    // 清空旧目标生成的路径，下一次行为树 tick 将立即为新目标重新规划。
    auto blackboard = bt_action_server_->getBlackboard();
    blackboard->set<nav_msgs::msg::Path>(path_blackboard_id_, nav_msgs::msg::Path());
    initializeGoalPoses(pending_goal);

    RCLCPP_INFO(logger_, "Multi-pose navigation goal preempted");
  } else {
    RCLCPP_WARN(
      logger_,
      "Navigation preemption rejected: behavior tree changed");
    bt_action_server_->terminatePendingGoal();
  }
}

void
NavigateThroughPosesNavigator::initializeGoalPoses(ActionT::Goal::ConstSharedPtr goal)
{
  if (goal->poses.size() > 0) {
    const auto & last_goal_pose = goal->poses.back().pose;
    const double goal_yaw = getYaw(last_goal_pose.orientation);

    RCLCPP_INFO(
      logger_,
      "Multi-pose navigation started: poses=%zu, goal=(%.2f,%.2f,%.4f)",
      goal->poses.size(), last_goal_pose.position.x, last_goal_pose.position.y,
      goal_yaw);
  }

  // Reset state for new action feedback
  start_time_ = clock_->now();
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<int>("number_recoveries", 0);  // NOLINT

  // Update the goal pose on the blackboard
  blackboard->set<Goals>(goals_blackboard_id_, goal->poses);
}

}  // namespace nav2_bt_navigator
