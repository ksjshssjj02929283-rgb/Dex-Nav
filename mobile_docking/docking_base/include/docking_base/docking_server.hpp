// Copyright (c) 2024 Open Navigation LLC
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

#ifndef DOCKING_BASE__DOCKING_SERVER_HPP_
#define DOCKING_BASE__DOCKING_SERVER_HPP_

#include <vector>
#include <memory>
#include <string>
#include <mutex>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "docking_base/utils.hpp"
#include "docking_base/types.hpp"
#include "docking_base/navigator.hpp"
#include "docking_base_core/docking_planner.hpp"
#include "docking_base_core/docking_interface.hpp"
#include "docking_base_core/perception_interface.hpp"
#include "tf2_ros/transform_listener.h"

namespace docking_base
{
/**
 * @class docking_base::DockingServer
 * @brief An action server which implements charger docking node for AMRs
 */
class DockingServer : public nav2_util::LifecycleNode
{
public:
  using DockingActionServer = nav2_util::SimpleActionServer<DockRobot>;
  using UndockingActionServer = nav2_util::SimpleActionServer<UndockRobot>;

  /**
   * @brief A constructor for docking_base::DockingServer
   * @param options Additional options to control creation of the node.
   */
  explicit DockingServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief A destructor for docking_base::DockingServer
   */
  ~DockingServer() = default;

  /**
   * @brief Publish feedback from a docking action.
   * @param state Current state - should be one of those defined in message.
   */
  void publishDockingFeedback(uint16_t state);

  /**
   * @brief 由 dock 位姿 + staging offset 参数算 staging 位姿（DEC-014：替代 ChargingDock::getStagingPose）。
   *        staging = dock_pose 沿 dock x 轴偏移 staging_x_offset + 朝向加 staging_yaw_offset。
   */
  geometry_msgs::msg::PoseStamped computeStagingPose(
    const geometry_msgs::msg::PoseStamped & dock_pose);

  /**
   * @brief Get the robot pose (aka base_frame pose) in another frame.
   * @param frame The frame_id to get the robot pose in.
   * @returns Computed robot pose, throws TF2 error if failure.
   */
  virtual geometry_msgs::msg::PoseStamped getRobotPoseInFrame(const std::string & frame);

  /**
   * @brief Gets a preempted goal if immediately requested
   * @param Goal goal to check or replace if required with preemption
   * @param action_server Action server to check for preemptions on
   * @return SUCCESS or FAILURE
   */
  template<typename ActionT>
  void getPreemptedGoalIfRequested(
    typename std::shared_ptr<const typename ActionT::Goal> goal,
    const std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server);

  /**
   * @brief Checks and logs warning if action canceled
   * @param action_server Action server to check for cancellation on
   * @param name Name of action to put in warning message
   * @return True if action has been cancelled
   */
  template<typename ActionT>
  bool checkAndWarnIfCancelled(
    std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server,
    const std::string & name);

  /**
   * @brief Checks and logs warning if action preempted
   * @param action_server Action server to check for preemption on
   * @param name Name of action to put in warning message
   * @return True if action has been preempted
   */
  template<typename ActionT>
  bool checkAndWarnIfPreempted(
    std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server,
    const std::string & name);

  /**
   * @brief Configure member variables
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

  /**
   * @brief Publish zero velocity at terminal condition
   */
  void publishZeroVelocity();

protected:
  /**
   * @brief Main action callback method to complete docking request
   */
  void dockRobot();

  /**
   * @brief Main action callback method to complete undocking request
   */
  void undockRobot();

  // ===== ROS1 范式：三类执行插件 + DockingInterface 任务驱动（双线程）=====

  /**
   * @brief 加载 planner / controller / goal_checker 三类 pluginlib map（按参数 id 列表）。
   *        controller/goal_checker 用 nav2_core，planner 用 docking_base_core::DockingPlanner。
   */
  void loadPluginMaps();

  /**
   * @brief 预加载 task_plugins / perception_plugins map（与三类执行 map 对称，on_configure 调）。
   *        task 实例按 id（charging/exit…）预加载并 configure；executeDock 按 dock_type 取复用。
   */
  void loadTaskMaps();

  /**
   * @brief 按 dock_type 从预加载 task map 取 DockingInterface 实例（任务从 action goal 传入）。
   * @param dock_type 对接桩类型（来自 DockRobot/UndockRobot goal）
   * @param docking   true=对接任务(<dock_type>.dock_task)；false=出桩任务(<dock_type>.undock_task)
   * @return 预加载的任务实例（复用，调用方负责 Reset+Init）；找不到返回 nullptr
   */
  docking_base_core::DockingInterface::Ptr resolveTask(
    const std::string & dock_type, bool docking);

  /**
   * @brief 按 dock_type 从预加载 perception map 取 PerceptionInterface（可为空=该任务无感知）。
   */
  docking_base_core::PerceptionInterface::Ptr resolvePerception(const std::string & dock_type);

  /**
   * @brief 非抛出版机器人位姿获取（控制/规划线程用）。
   */
  bool getRobotPoseSafe(const std::string & frame, geometry_msgs::msg::PoseStamped & out);

  /**
   * @brief ROS1 式双线程执行一个对接任务：
   *        规划线程（位姿喂入 + 感知 + UpdateTaskStage 推阶段）+
   *        控制线程（按阶段指令包选 planner/controller/goal_checker，computeVelocityCommands 出 cmd_vel）。
   * @return 任务 SUCCESS 为 true；FAILED/超时/取消为 false
   */
  template<typename ActionServerT>
  bool runDockingTask(
    docking_base_core::DockingInterface::Ptr task,
    docking_base_core::PerceptionInterface::Ptr perception,
    std::unique_ptr<ActionServerT> & action_server,
    const std::string & action_name, double timeout_s);

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  // Mutex for dynamic parameters and dock database
  std::shared_ptr<std::mutex> mutex_;

  // Frequency to run control loops
  double controller_frequency_;
  // Timeout for initially detecting the charge dock
  double initial_perception_timeout_;
  // Timeout after making contact with dock for charging to start
  // If this is exceeded, the robot returns to the staging pose and retries
  double wait_charge_timeout_;
  // Timeout to approach into the dock and reset its approach is retrying
  double dock_approach_timeout_;
  // Timeout to rotate to the dock
  double rotate_to_dock_timeout_;
  // When undocking, these are the tolerances for arriving at the staging pose
  double undock_linear_tolerance_, undock_angular_tolerance_;
  // Maximum number of times the robot will return to staging pose and retry docking
  int max_retries_, num_retries_;
  // This is the root frame of the robot - typically "base_link"
  std::string base_frame_;
  // This is our fixed frame for controlling - typically "odom"
  std::string fixed_frame_;
  // Does the robot drive backwards onto the dock? Default is forwards
  bool dock_backwards_;
  // Whether to rotate to the dock before docking
  bool rotate_to_dock_;
  // The tolerance to the dock's staging pose not requiring navigation
  double dock_prestaging_tolerance_;
  // Angular tolerance to exit the rotation loop when rotate_to_dock is enabled
  double rotation_angular_tolerance_;
  // staging 位姿偏移（DEC-014：替代 ChargingDock::getStagingPose）
  double staging_x_offset_;
  double staging_yaw_offset_;

  // This is a class member so it can be accessed in publish feedback
  rclcpp::Time action_start_time_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;

  std::unique_ptr<DockingActionServer> docking_action_server_;
  std::unique_ptr<UndockingActionServer> undocking_action_server_;

  std::unique_ptr<Navigator> navigator_;
  std::string curr_dock_type_;

  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

  // ===== ROS1 范式：三类执行插件 + 任务/感知插件 + 最小 costmap =====
  // planner / controller / goal_checker 三类 pluginlib map（按阶段指令包选用）
  pluginlib::ClassLoader<docking_base_core::DockingPlanner> planner_loader_;
  pluginlib::ClassLoader<nav2_core::Controller> controller_loader_;
  pluginlib::ClassLoader<nav2_core::GoalChecker> goal_checker_loader_;
  std::unordered_map<std::string, docking_base_core::DockingPlanner::Ptr> planners_;
  std::unordered_map<std::string, nav2_core::Controller::Ptr> controllers_;
  std::unordered_map<std::string, nav2_core::GoalChecker::Ptr> goal_checkers_;
  std::vector<std::string> planner_ids_, controller_ids_, goal_checker_ids_;

  // 任务 / 感知插件：on_configure 预加载成 map（与三类执行 map 对称），按 dock_type 选
  pluginlib::ClassLoader<docking_base_core::DockingInterface> task_loader_;
  pluginlib::ClassLoader<docking_base_core::PerceptionInterface> perception_loader_;
  std::unordered_map<std::string, docking_base_core::DockingInterface::Ptr> tasks_;
  std::unordered_map<std::string, docking_base_core::PerceptionInterface::Ptr> perceptions_;
  std::vector<std::string> task_ids_, perception_ids_;

  // 最小 costmap（仅供 controller/goal_checker 取 frame；不避障）
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;

  double plan_frequency_;          // 规划线程频率
  double docking_task_timeout_;    // 单次对接任务超时（s）
};

}  // namespace docking_base

#endif  // DOCKING_BASE__DOCKING_SERVER_HPP_
