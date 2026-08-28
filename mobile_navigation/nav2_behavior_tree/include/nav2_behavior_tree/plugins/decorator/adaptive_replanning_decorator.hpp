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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__ADAPTIVE_REPLANNING_DECORATOR_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__ADAPTIVE_REPLANNING_DECORATOR_HPP_

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

#include "behaviortree_cpp_v3/decorator_node.h"
#include "nav2_behavior_tree/global_replan_context.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::DecoratorNode that prevents replanning when the robot is close to the goal,
 * except for the first planning attempt, and regulates planning frequency except when deviated.
 */
class AdaptiveReplanning : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::AdaptiveReplanning
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  AdaptiveReplanning(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  void halt() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to check distance"),
      BT::InputPort<double>(
        "distance_threshold", kDefaultDistanceThresholdM,
        "Distance threshold to skip replanning"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame"),
      BT::InputPort<nav_msgs::msg::Path>("path", "Global path to check deviation from"),
      BT::InputPort<double>(
        "max_deviation", kDefaultMaxDeviationM,
        "Max allowed deviation from path"),
      BT::InputPort<double>("hz", kDefaultReplanningFrequencyHz, "Replanning frequency"),
      BT::InputPort<std::string>(
        "recovery_replan_mode", std::string("disabled"),
        "Recovery replan mode: disabled, observe, or active")
    };
  }

private:
  static constexpr double kDefaultDistanceThresholdM = 0.3;
  static constexpr double kDefaultMaxDeviationM = 0.5;
  static constexpr double kDefaultReplanningFrequencyHz = 0.333;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  // 规划开始时锁存当前请求，确保运行中的旧规划不能确认之后产生的新请求。
  void beginPlanningAttempt();
  BT::NodeStatus tickPlanningChild();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<GlobalReplanContext> global_replan_context_;
  GlobalReplanMode recovery_replan_mode_{GlobalReplanMode::DISABLED};
  double transform_tolerance_;

  bool has_valid_plan_;
  geometry_msgs::msg::PoseStamped last_goal_;
  bool has_last_goal_;

  std::chrono::steady_clock::time_point last_planning_time_;

  bool planning_in_progress_;
  std::optional<GlobalReplanRequest> active_replan_request_;

  std::string global_frame_;
  std::string robot_base_frame_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__ADAPTIVE_REPLANNING_DECORATOR_HPP_
