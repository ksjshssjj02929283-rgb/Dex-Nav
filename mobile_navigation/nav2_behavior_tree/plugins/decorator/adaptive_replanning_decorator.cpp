// Copyright (c) 2026 kuawei
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
#include <cinttypes>
#include <limits>
#include <memory>
#include <string>

#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

#include "behaviortree_cpp_v3/decorator_node.h"
#include "nav2_behavior_tree/plugins/decorator/adaptive_replanning_decorator.hpp"

namespace nav2_behavior_tree
{

namespace
{

constexpr double kGoalPositionChangeToleranceM = 1e-3;
constexpr double kGoalOrientationChangeTolerance = 1e-3;
constexpr int64_t kPlanningLogThrottleMs = 1000;
constexpr int64_t kWarningLogThrottleMs = 5000;

}  // namespace

AdaptiveReplanning::AdaptiveReplanning(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  has_valid_plan_(false),
  has_last_goal_(false),
  planning_in_progress_(false),
  global_frame_("map"),
  robot_base_frame_("base_link")
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  global_replan_context_ = getOrCreateGlobalReplanContext(config().blackboard);

  node_->get_parameter("transform_tolerance", transform_tolerance_);

  std::string mode_value{"disabled"};
  getInput("recovery_replan_mode", mode_value);
  if (!parseGlobalReplanMode(mode_value, recovery_replan_mode_)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "AdaptiveReplanning: Invalid recovery_replan_mode='%s'; using disabled.",
      mode_value.c_str());
    recovery_replan_mode_ = GlobalReplanMode::DISABLED;
  }
  global_replan_context_->setMode(recovery_replan_mode_);
  RCLCPP_INFO(
    node_->get_logger(), "AdaptiveReplanning: Recovery replan mode=%s.",
    nav2_behavior_tree::toString(recovery_replan_mode_));
}

void AdaptiveReplanning::beginPlanningAttempt()
{
  planning_in_progress_ = true;
  active_replan_request_ = recovery_replan_mode_ == GlobalReplanMode::ACTIVE ?
    global_replan_context_->latestPendingRequest() : std::nullopt;

  if (active_replan_request_) {
    const auto & request = *active_replan_request_;
    RCLCPP_INFO(
      node_->get_logger(),
      "Global replan started: id=%" PRIu64 ", reason=%s.",
      request.token.request_id,
      nav2_behavior_tree::toString(request.reason));
  }
}

BT::NodeStatus AdaptiveReplanning::tickPlanningChild()
{
  const BT::NodeStatus child_state = child_node_->executeTick();
  if (child_state == BT::NodeStatus::RUNNING) {
    return child_state;
  }

  planning_in_progress_ = false;

  if (child_state == BT::NodeStatus::SUCCESS) {
    has_valid_plan_ = true;
    last_planning_time_ = std::chrono::steady_clock::now();

    if (active_replan_request_ &&
      global_replan_context_->complete(active_replan_request_->token))
    {
      nav_msgs::msg::Path path;
      getInput("path", path);
      RCLCPP_INFO(
        node_->get_logger(),
        "Global replan completed: id=%" PRIu64 ", poses=%zu.",
        active_replan_request_->token.request_id,
        path.poses.size());
    }
  } else if (child_state == BT::NodeStatus::FAILURE && active_replan_request_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Global replan remains pending: id=%" PRIu64 ", planner failed.",
      active_replan_request_->token.request_id);
  }

  active_replan_request_.reset();
  return child_state;
}

void AdaptiveReplanning::halt()
{
  BT::DecoratorNode::halt();

  // Behavior Tree 在任务结束、取消或抢占时会 halt；下次执行必须重新生成首条路径。
  has_valid_plan_ = false;
  has_last_goal_ = false;
  planning_in_progress_ = false;
  active_replan_request_.reset();
}

BT::NodeStatus AdaptiveReplanning::tick()
{
  // 1. Get current goal and check if goal has changed
  geometry_msgs::msg::PoseStamped current_goal;
  if (!getInput("goal", current_goal)) {
    if (!planning_in_progress_) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), kWarningLogThrottleMs,
        "AdaptiveReplanning: Goal input unavailable; planning without gating.");
      beginPlanningAttempt();
    }
    return tickPlanningChild();
  }

  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);

  double distance_threshold = kDefaultDistanceThresholdM;
  getInput("distance_threshold", distance_threshold);
  if (!std::isfinite(distance_threshold) || distance_threshold < 0.0) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), kWarningLogThrottleMs,
      "AdaptiveReplanning: Invalid distance_threshold=%.3f; using %.3f m.",
      distance_threshold, kDefaultDistanceThresholdM);
    distance_threshold = kDefaultDistanceThresholdM;
  }

  double max_deviation = kDefaultMaxDeviationM;
  getInput("max_deviation", max_deviation);
  if (!std::isfinite(max_deviation) || max_deviation < 0.0) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), kWarningLogThrottleMs,
      "AdaptiveReplanning: Invalid max_deviation=%.3f; using %.3f m.",
      max_deviation, kDefaultMaxDeviationM);
    max_deviation = kDefaultMaxDeviationM;
  }

  double hz = kDefaultReplanningFrequencyHz;
  getInput("hz", hz);
  if (!std::isfinite(hz) || hz <= 0.0) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), kWarningLogThrottleMs,
      "AdaptiveReplanning: Invalid hz=%.3f; using %.3f Hz.",
      hz, kDefaultReplanningFrequencyHz);
    hz = kDefaultReplanningFrequencyHz;
  }

  // Check if target goal has changed
  bool goal_changed = false;
  if (!has_last_goal_) {
    goal_changed = true;
  } else {
    double dx = current_goal.pose.position.x - last_goal_.pose.position.x;
    double dy = current_goal.pose.position.y - last_goal_.pose.position.y;
    double dz = current_goal.pose.position.z - last_goal_.pose.position.z;
    double dqx = current_goal.pose.orientation.x - last_goal_.pose.orientation.x;
    double dqy = current_goal.pose.orientation.y - last_goal_.pose.orientation.y;
    double dqz = current_goal.pose.orientation.z - last_goal_.pose.orientation.z;
    double dqw = current_goal.pose.orientation.w - last_goal_.pose.orientation.w;

    if (std::abs(dx) > kGoalPositionChangeToleranceM ||
      std::abs(dy) > kGoalPositionChangeToleranceM ||
      std::abs(dz) > kGoalPositionChangeToleranceM ||
      std::abs(dqx) > kGoalOrientationChangeTolerance ||
      std::abs(dqy) > kGoalOrientationChangeTolerance ||
      std::abs(dqz) > kGoalOrientationChangeTolerance ||
      std::abs(dqw) > kGoalOrientationChangeTolerance ||
      current_goal.header.frame_id != last_goal_.header.frame_id)
    {
      goal_changed = true;
    }
  }

  if (goal_changed) {
    // 正在运行的规划仍绑定旧目标，先取消后再以新目标启动首次规划。
    if (planning_in_progress_) {
      haltChild();
      planning_in_progress_ = false;
      active_replan_request_.reset();
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "AdaptiveReplanning: Goal changed; previous plan invalidated.");
    has_valid_plan_ = false;
    last_goal_ = current_goal;
    has_last_goal_ = true;
  }

  // 一次规划进入 RUNNING 后必须持续 tick，不能被近目标或频率门控中途截断。
  if (planning_in_progress_) {
    return tickPlanningChild();
  }

  // 2. Get current robot pose
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), kWarningLogThrottleMs,
      "AdaptiveReplanning: Robot pose unavailable; planning without pose gating.");
    beginPlanningAttempt();
    return tickPlanningChild();
  }

  // 3. Compute distance to goal
  double dist = nav2_util::geometry_utils::euclidean_distance(
    current_pose.pose, current_goal.pose);

  // 4. Check if robot has deviated from path (if path is available and it's not the first planning)
  bool deviated = false;
  nav_msgs::msg::Path path;
  double min_dist = 0.0;
  if (has_valid_plan_ && getInput("path", path) && !path.poses.empty()) {
    min_dist = std::numeric_limits<double>::max();
    for (const auto & pose : path.poses) {
      double d = nav2_util::geometry_utils::euclidean_distance(
        current_pose.pose, pose.pose);
      if (d < min_dist) {
        min_dist = d;
      }
    }
    if (min_dist > max_deviation) {
      deviated = true;
    }
  }

  // 5. 恢复请求优先于近目标和频率门控，确保失败后一定能刷新旧路径。
  const double period_sec = 1.0 / hz;
  const bool has_recovery_request =
    recovery_replan_mode_ == GlobalReplanMode::ACTIVE &&
    static_cast<bool>(global_replan_context_->latestPendingRequest());

  if (!has_valid_plan_) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), kPlanningLogThrottleMs,
      "AdaptiveReplanning: Planning initial path (dist_to_goal=%.3f m).", dist);
  } else if (has_recovery_request) {
    // 请求详情由beginPlanningAttempt()统一记录。
  } else if (dist <= distance_threshold) {
    return BT::NodeStatus::SUCCESS;
  } else if (deviated) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), kPlanningLogThrottleMs,
      "AdaptiveReplanning: Path deviation requires replanning "
      "(deviation=%.3f m, limit=%.3f m, dist_to_goal=%.3f m).",
      min_dist, max_deviation, dist);
  } else {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed_sec =
      std::chrono::duration<double>(now - last_planning_time_).count();
    if (elapsed_sec < period_sec) {
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), kPlanningLogThrottleMs,
      "AdaptiveReplanning: Replanning period elapsed "
      "(elapsed=%.3f s, period=%.3f s, dist_to_goal=%.3f m).",
      elapsed_sec, period_sec, dist);
  }

  beginPlanningAttempt();
  return tickPlanningChild();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::AdaptiveReplanning>("AdaptiveReplanning");
}
