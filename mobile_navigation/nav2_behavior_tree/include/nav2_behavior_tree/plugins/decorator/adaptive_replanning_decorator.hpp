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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
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
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        "goals", "Destinations to check final distance"),
      BT::InputPort<double>(
        "distance_threshold", kDefaultDistanceThresholdM,
        "Distance threshold to skip replanning"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame"),
      BT::InputPort<nav_msgs::msg::Path>("path", "Global path to check deviation from"),
      BT::InputPort<bool>(
        "enable_global_replanning", true,
        "Enable periodic, deviation, obstacle, and recovery-triggered global replanning"),
      BT::InputPort<double>(
        "max_deviation", kDefaultMaxDeviationM,
        "Max allowed deviation from path"),
      BT::InputPort<bool>(
        "enable_deviation_replanning", true,
        "Whether path deviation can trigger replanning"),
      BT::InputPort<double>("hz", kDefaultReplanningFrequencyHz, "Replanning frequency"),
      BT::InputPort<double>(
        "path_obstacle_check_hz", kDefaultPathObstacleCheckHz,
        "Global-path obstacle check frequency"),
      BT::InputPort<int>(
        "invalid_path_confirmations", kDefaultInvalidPathConfirmations,
        "Consecutive blocked checks before replanning"),
      BT::InputPort<int>(
        "min_blocked_path_poses", kDefaultMinBlockedPathPoses,
        "Minimum blocked path poses treated as a large obstacle"),
      BT::InputPort<double>(
        "obstacle_replan_cooldown", kDefaultObstacleReplanCooldownS,
        "Seconds to suspend path-validity checks after a successful obstacle-triggered replan"),
      BT::InputPort<std::string>(
        "recovery_replan_mode", std::string("disabled"),
        "Recovery replan mode: disabled, observe, or active")
    };
  }

private:
  static constexpr double kDefaultDistanceThresholdM = 0.3;
  static constexpr double kDefaultMaxDeviationM = 0.5;
  static constexpr double kDefaultReplanningFrequencyHz = 0.02;
  static constexpr double kDefaultPathObstacleCheckHz = 2.0;
  static constexpr int kDefaultInvalidPathConfirmations = 4;
  static constexpr int kDefaultMinBlockedPathPoses = 3;
  static constexpr double kDefaultObstacleReplanCooldownS = 2.5;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  // 规划开始时锁存当前请求，确保运行中的旧规划不能确认之后产生的新请求。
  void beginPlanningAttempt();
  BT::NodeStatus tickPlanningChild();
  // 从顶层 Navigator 写入的黑板数据同步当前导航代次。
  void syncPrepareSession();
  // 将 controller_server 的后退完成事件接入现有全局重规划上下文。
  void consumePrepareReplanRequest();
  void cancelPendingPrepareReplan();
  // 轮询 planner_server 的全局路径有效性服务；持续的大障碍阻塞会锁存一次重规划。
  void pollPathValidity(const nav_msgs::msg::Path & path, bool has_path);
  void resetPathValidityState();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<GlobalReplanContext> global_replan_context_;
  bool global_replanning_enabled_{true};
  GlobalReplanMode recovery_replan_mode_{GlobalReplanMode::DISABLED};
  double transform_tolerance_;

  bool has_valid_plan_;
  geometry_msgs::msg::PoseStamped last_goal_;
  bool has_last_goal_;

  std::chrono::steady_clock::time_point last_planning_time_;

  bool planning_in_progress_;
  std::optional<GlobalReplanRequest> active_replan_request_;

  rclcpp::CallbackGroup::SharedPtr prepare_replan_callback_group_;
  rclcpp::executors::SingleThreadedExecutor prepare_replan_callback_executor_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr prepare_replan_request_sub_;
  std::atomic<uint64_t> latest_prepare_replan_request_id_{0};
  uint64_t prepare_session_id_{0};
  uint64_t last_consumed_prepare_replan_request_id_{0};
  std::optional<GlobalReplanToken> pending_prepare_replan_;

  rclcpp::Client<nav2_msgs::srv::IsPathValid>::SharedPtr path_validity_client_;
  rclcpp::CallbackGroup::SharedPtr path_validity_callback_group_;
  rclcpp::executors::SingleThreadedExecutor path_validity_executor_;
  rclcpp::Client<nav2_msgs::srv::IsPathValid>::SharedFuture pending_path_validity_;
  bool path_validity_request_pending_{false};
  bool path_blocked_latched_{false};
  bool planning_for_blocked_path_{false};
  int invalid_path_count_{0};
  std::chrono::steady_clock::time_point last_path_check_time_;
  std::chrono::steady_clock::time_point path_validation_cooldown_until_;

  std::string global_frame_;
  std::string robot_base_frame_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__ADAPTIVE_REPLANNING_DECORATOR_HPP_
