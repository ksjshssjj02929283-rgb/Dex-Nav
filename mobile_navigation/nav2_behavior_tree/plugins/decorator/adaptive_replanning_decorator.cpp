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

#include <algorithm>
#include <chrono>
#include <cmath>
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

AdaptiveReplanning::AdaptiveReplanning(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  is_first_planning_(true),
  last_planning_failed_(false),
  has_last_goal_(false),
  has_planned_once_(false),
  global_frame_("map"),
  robot_base_frame_("base_link")
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  config().blackboard->get<BtTickTiming *>("bt_tick_timing", tick_timing_);

  node_->get_parameter("transform_tolerance", transform_tolerance_);
}

BT::NodeStatus AdaptiveReplanning::tick()
{
  using SteadyClock = std::chrono::steady_clock;

  const bool timing_enabled = tick_timing_ != nullptr;
  const auto tick_start = timing_enabled ? SteadyClock::now() : SteadyClock::time_point{};
  auto stage_start = tick_start;
  BtAdaptiveReplanningTiming timing;
  timing.valid = timing_enabled;
  const auto finish_stage = [&](double & stage_ms) {
      if (!timing_enabled) {
        return;
      }
      const auto stage_end = SteadyClock::now();
      stage_ms += std::chrono::duration<double, std::milli>(stage_end - stage_start).count();
      stage_start = stage_end;
    };
  const auto begin_stage = [&]() {
      if (timing_enabled) {
        stage_start = SteadyClock::now();
      }
    };
  const auto finish_timing = [&]() {
      if (!timing_enabled) {
        return;
      }
      timing.total_ms = std::chrono::duration<double, std::milli>(
        SteadyClock::now() - tick_start).count();
      const double known_ms =
        timing.ports_ms + timing.goal_update_ms + timing.tf_ms + timing.path_copy_ms +
        timing.path_scan_ms + timing.decision_ms + timing.child_ms;
      timing.other_ms = std::max(0.0, timing.total_ms - known_ms);
      if (!tick_timing_->adaptive.valid ||
        timing.total_ms > tick_timing_->adaptive.total_ms)
      {
        // 同一轮可能因控制节点重试而多次执行，保留其中最慢的一次。
        tick_timing_->adaptive = timing;
      }
    };
  const auto execute_child = [&]() {
      // 规划日志和状态维护不属于子树执行，重新取边界后再统计child。
      begin_stage();
      const BT::NodeStatus child_state = child_node_->executeTick();
      finish_stage(timing.child_ms);
      return child_state;
    };

  // 1. Get current goal and check if goal has changed
  geometry_msgs::msg::PoseStamped current_goal;
  if (!getInput("goal", current_goal)) {
    finish_stage(timing.ports_ms);
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "AdaptiveReplanning: Failed to get 'goal' from input port.");
    const BT::NodeStatus child_state = execute_child();
    finish_timing();
    return child_state;
  }

  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);

  double distance_threshold = 0.5;
  getInput("distance_threshold", distance_threshold);

  double max_deviation = 0.5;
  getInput("max_deviation", max_deviation);

  double hz = 0.02;
  getInput("hz", hz);
  finish_stage(timing.ports_ms);

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

    if (std::abs(dx) > 1e-3 || std::abs(dy) > 1e-3 || std::abs(dz) > 1e-3 ||
      std::abs(dqx) > 1e-3 || std::abs(dqy) > 1e-3 || std::abs(dqz) > 1e-3 ||
      std::abs(dqw) > 1e-3 ||
      current_goal.header.frame_id != last_goal_.header.frame_id)
    {
      goal_changed = true;
    }
  }

  if (goal_changed) {
    RCLCPP_INFO(
      node_->get_logger(),
      "AdaptiveReplanning: New navigation goal received. Resetting planning status.");
    is_first_planning_ = true;
    last_planning_failed_ = false;
    last_goal_ = current_goal;
    has_last_goal_ = true;
    has_planned_once_ = false;
  }
  finish_stage(timing.goal_update_ms);

  // 2. Get current robot pose
  geometry_msgs::msg::PoseStamped current_pose;
  const bool current_pose_available = nav2_util::getCurrentPose(
    current_pose, *tf_, global_frame_, robot_base_frame_,
    transform_tolerance_);
  finish_stage(timing.tf_ms);
  if (!current_pose_available) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "AdaptiveReplanning: Current robot pose is not available. Proceeding with planning.");
    const BT::NodeStatus child_state = execute_child();
    if (child_state == BT::NodeStatus::SUCCESS) {
      is_first_planning_ = false;
      last_planning_failed_ = false;
      has_planned_once_ = true;
      last_planning_time_ = std::chrono::steady_clock::now();
    } else if (child_state == BT::NodeStatus::FAILURE) {
      last_planning_failed_ = true;
    }
    finish_timing();
    return child_state;
  }

  // 3. Check if robot has deviated from path (if path is available and it's not the first planning)
  bool deviated = false;
  nav_msgs::msg::Path path;
  double min_dist = 0.0;
  bool path_available = false;
  if (!is_first_planning_) {
    path_available = static_cast<bool>(getInput("path", path));
    timing.path_poses = path.poses.size();
  }
  finish_stage(timing.path_copy_ms);
  if (path_available && !path.poses.empty()) {
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
  finish_stage(timing.path_scan_ms);

  // 4. Decision:
  // - If it's the first planning, we must plan.
  // - If robot has deviated, we must plan.
  // - If distance to goal <= threshold, we skip replanning.
  // - If distance > threshold: we apply the rate control (hz) using absolute time.
  bool should_plan = false;
  std::string plan_reason = "";
  double elapsed_sec = 0.0;
  double period = 1.0 / hz;
  const double dist = nav2_util::geometry_utils::euclidean_distance(
    current_pose.pose, current_goal.pose);

  if (is_first_planning_) {
    should_plan = true;
    plan_reason = "first_planning";
  } else if (dist <= distance_threshold) {
    should_plan = false;
    finish_stage(timing.decision_ms);
    finish_timing();
    return BT::NodeStatus::SUCCESS;
  } else if (deviated) {
    should_plan = true;
    plan_reason = "path_deviation";
  } else {
    // Apply frequency check
    if (!has_planned_once_) {
      should_plan = true;
      plan_reason = "has_not_planned_once_yet";
    } else {
      auto now = std::chrono::steady_clock::now();
      elapsed_sec = std::chrono::duration<double>(now - last_planning_time_).count();
      if (elapsed_sec >= period) {
        should_plan = true;
        plan_reason = "rate_limit_elapsed";
      } else {
        should_plan = false;
      }
    }
  }
  finish_stage(timing.decision_ms);

  if (should_plan) {
    if (plan_reason == "first_planning" || plan_reason == "has_not_planned_once_yet") {
      RCLCPP_INFO(
        node_->get_logger(),
        "AdaptiveReplanning: Global replanning triggered: no valid initial path "
        "(reason=%s, dist_to_goal=%.3f m).",
        plan_reason.c_str(), dist);
    } else if (plan_reason == "path_deviation") {
      RCLCPP_INFO(
        node_->get_logger(),
        "AdaptiveReplanning: Global replanning triggered: path deviation exceeded limit "
        "(deviation=%.3f m > limit=%.3f m, dist_to_goal=%.3f m).",
        min_dist, max_deviation, dist);
    } else if (plan_reason == "rate_limit_elapsed") {
      RCLCPP_INFO(
        node_->get_logger(),
        "AdaptiveReplanning: Global replanning triggered: replanning period elapsed "
        "(elapsed=%.3f s >= period=%.3f s, hz=%.3f, dist_to_goal=%.3f m).",
        elapsed_sec, period, hz, dist);
    } else {
      RCLCPP_INFO(
        node_->get_logger(),
        "AdaptiveReplanning: Global replanning triggered (reason=%s, dist_to_goal=%.3f m).",
        plan_reason.c_str(), dist);
    }

    const BT::NodeStatus child_state = execute_child();

    if (child_state == BT::NodeStatus::SUCCESS) {
      is_first_planning_ = false;
      last_planning_failed_ = false;
      has_planned_once_ = true;
      last_planning_time_ = std::chrono::steady_clock::now();
    } else if (child_state == BT::NodeStatus::FAILURE) {
      last_planning_failed_ = true;
      // "First planning" means that no valid initial path has been produced yet.
      // Keep the flag set so recovery retries are not suppressed by the near-goal gate.
    }
    finish_timing();
    return child_state;
  }

  finish_timing();
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::AdaptiveReplanning>("AdaptiveReplanning");
}
