// Copyright (c) 2019 Samsung Research America
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

#ifndef NAV2_PLANNER__PLANNER_SERVER_HPP_
#define NAV2_PLANNER__PLANNER_SERVER_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <mutex>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_core/smoother.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"

namespace nav2_planner
{
/**
 * @class nav2_planner::PlannerServer
 * @brief An action server implements the behavior tree's ComputePathToPose
 * interface and hosts various plugins of different algorithms to compute plans.
 */
class PlannerServer : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for nav2_planner::PlannerServer
   * @param options Additional options to control creation of the node.
   */
  explicit PlannerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief A destructor for nav2_planner::PlannerServer
   */
  ~PlannerServer();

  using PlannerMap = std::unordered_map<std::string, nav2_core::GlobalPlanner::Ptr>;
  using SmootherMap = std::unordered_map<std::string, nav2_core::Smoother::Ptr>;

  /**
   * @brief Method to get plan from the desired plugin
   * @param start starting pose
   * @param goal goal request
   * @return Path
   */
  nav_msgs::msg::Path getPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & planner_id);

protected:
  /**
   * @brief Configure member variables and initializes planner
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Reset member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  using ActionToPose = nav2_msgs::action::ComputePathToPose;
  using ActionThroughPoses = nav2_msgs::action::ComputePathThroughPoses;
  using ActionServerToPose = nav2_util::SimpleActionServer<ActionToPose>;
  using ActionServerThroughPoses = nav2_util::SimpleActionServer<ActionThroughPoses>;

  /**
   * @brief Check if an action server is valid / active
   * @param action_server Action server to test
   * @return SUCCESS or FAILURE
   */
  template<typename T>
  bool isServerInactive(std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server);

  /**
   * @brief Check if an action server has a cancellation request pending
   * @param action_server Action server to test
   * @return SUCCESS or FAILURE
   */
  template<typename T>
  bool isCancelRequested(std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server);

  /**
   * @brief Wait for costmap to be valid with updated sensor data or repopulate after a
   * clearing recovery. Blocks until true without timeout.
   */
  void waitForCostmap();

  /**
   * @brief Check if an action server has a preemption request and replaces the goal
   * with the new preemption goal.
   * @param action_server Action server to get updated goal if required
   * @param goal Goal to overwrite
   */
  template<typename T>
  void getPreemptedGoalIfRequested(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
    typename std::shared_ptr<const typename T::Goal> goal);

  /**
   * @brief Get the starting pose from costmap or message, if valid
   * @param action_server Action server to terminate if required
   * @param goal Goal to find start from
   * @param start The starting pose to use
   * @return bool If successful in finding a valid starting pose
   */
  template<typename T>
  bool getStartPose(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
    typename std::shared_ptr<const typename T::Goal> goal,
    geometry_msgs::msg::PoseStamped & start);

  /**
   * @brief Transform start and goal poses into the costmap
   * global frame for path planning plugins to utilize
   * @param action_server Action server to terminate if required
   * @param start The starting pose to transform
   * @param goal Goal pose to transform
   * @return bool If successful in transforming poses
   */
  template<typename T>
  bool transformPosesToGlobalFrame(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
    geometry_msgs::msg::PoseStamped & curr_start,
    geometry_msgs::msg::PoseStamped & curr_goal);

  /**
   * @brief Validate that the path contains a meaningful path
   * @param action_server Action server to terminate if required
   * @param goal Goal Current goal
   * @param path Current path
   * @param planner_id The planner ID used to generate the path
   * @return bool If path is valid
   */
  template<typename T>
  bool validatePath(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
    const geometry_msgs::msg::PoseStamped & curr_goal,
    const nav_msgs::msg::Path & path,
    const std::string & planner_id);

  // Our action server implements the ComputePathToPose action
  std::unique_ptr<ActionServerToPose> action_server_pose_;
  std::unique_ptr<ActionServerThroughPoses> action_server_poses_;

  /**
   * @brief The action server callback which calls planner to get the path
   * ComputePathToPose
   */
  void computePlan();

  /**
   * @brief The action server callback which calls planner to get the path
   * ComputePathThroughPoses
   */
  void computePlanThroughPoses();

  /**
   * @brief The service callback to determine if the path is still valid
   * @param request to the service
   * @param response from the service
   */
  void isPathValid(
    const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
    std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response);

  /**
   * @brief Publish a path for visualization purposes
   * @param path Reference to Global Path
   */
  void publishPlan(const nav_msgs::msg::Path & path);

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;

  // Planner
  PlannerMap planners_;
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> gp_loader_;

  // ── 路径平滑统一后处理（getPlan 出口，goal_line_interp 之后；current_* 口径）──
  // 平滑责任归"生产折线的那一层"：global 侧折线源 = Theta* 拐点 / 直线接管接缝；
  // ③ local planner（优化型）输出天然平滑，不做二次平滑（doc 17）。
  SmootherMap smoothers_;
  pluginlib::ClassLoader<nav2_core::Smoother> smoother_loader_;
  std::vector<std::string> smoother_ids_;
  std::vector<std::string> smoother_types_;
  std::string smoother_ids_concat_;
  std::string current_smoother_;
  bool smoother_enable_{false};
  double smoother_max_time_{0.1};
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> smoother_costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> smoother_footprint_sub_;

  /// 平滑统一后处理：current_smoother 对 path 执行 smooth()，异常/碰撞由插件
  /// 内部防护，此处兜底捕获并保留未平滑路径。
  void applyPathSmoothing(nav_msgs::msg::Path & path);
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> planner_ids_;
  std::vector<std::string> planner_types_;
  double max_planner_duration_;
  std::string planner_ids_concat_;
  // The currently active planner key used as fallback when planner_id is
  // empty or not found. Switchable at runtime via the "current_planner" param.
  std::string current_planner_;

  // When true, the planner_id sent by callers (e.g. BT's hard-coded
  // "GridBased") is IGNORED and getPlan always dispatches to current_planner_.
  // This is the "hot-swap" mode: register all candidate planners under their
  // real names (RouteGridPlanner, SmacGridBased, ...), set
  // override_planner_id=true, and flip current_planner at runtime.
  // When false (default): nav2 standard behavior — caller-supplied
  // planner_id wins; current_planner_ only kicks in as a fallback.
  bool override_planner_id_{false};

  // Free-space planner to retry with when the route planner reports no route
  // graph (map has no road network). Empty = disabled. "route_fallback_planner".
  std::string route_fallback_planner_;

  // Clock
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  /**
   * @brief 末端直线插值统一后处理（原 smac fork 定制逻辑上收，对所有 planner 生效）：
   *   - 成功路径：尾点距 goal ≤ tail_distance 时，截断该窗口内尾段（如朝向对齐
   *     弯钩等末端机动），直线插值补到 goal——朝向对齐交给控制器末端旋转
   *   - 失败兜底：规划为空且 start 距 goal ≤ tail_distance 时，直线插值 start→goal
   *     （近距目标首次规划的保护：短距机动超出运动学 planner 求解能力时必然失败）
   *   两分支均要求 goal 处 footprint 无碰撞；位置线性插值 + yaw 线性混合。
   */
  void applyGoalLineInterpolation(
    nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

  /// goal 处机器人 footprint 无碰撞检查（全局 costmap，LETHAL 阈值）。
  bool isGoalFootprintFree(const geometry_msgs::msg::PoseStamped & goal);

  /// 从 from 直线插值到 goal 并追加到 path（跳过 from 本身，防重复点）。
  void appendGoalLine(
    nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & from,
    const geometry_msgs::msg::PoseStamped & goal);

  // 末端直线插值参数（开关与参数暴露到 YAML）。
  // tail_distance 同时用于两个分支：成功路径的尾段替换窗口 + 失败兜底的
  // 判定半径——两者语义相同（goal 附近多大范围内允许直线接管），单参数管理。
  bool goal_line_interp_enable_{false};
  double goal_line_interp_tail_distance_{0.3};
  double goal_line_interp_step_{0.02};

  // Global Costmap
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
  nav2_costmap_2d::Costmap2D * costmap_;

  // Publishers for the path
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;

  // Service to deterime if the path is valid
  rclcpp::Service<nav2_msgs::srv::IsPathValid>::SharedPtr is_path_valid_service_;
};

}  // namespace nav2_planner

#endif  // NAV2_PLANNER__PLANNER_SERVER_HPP_
