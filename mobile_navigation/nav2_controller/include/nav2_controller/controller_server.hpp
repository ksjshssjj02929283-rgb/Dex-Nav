// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_CONTROLLER__CONTROLLER_SERVER_HPP_
#define NAV2_CONTROLLER__CONTROLLER_SERVER_HPP_

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <mutex>

#include "nav2_core/controller.hpp"
#include "nav2_core/progress_checker.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.h"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav_2d_utils/odom_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_controller
{

class ProgressChecker;
/**
 * @class nav2_controller::ControllerServer
 * @brief This class hosts variety of plugins of different algorithms to
 * complete control tasks from the exposed FollowPath action server.
 */
class ControllerServer : public nav2_util::LifecycleNode
{
public:
  using ControllerMap = std::unordered_map<std::string, nav2_core::Controller::Ptr>;
  using GoalCheckerMap = std::unordered_map<std::string, nav2_core::GoalChecker::Ptr>;

  /**
   * @brief Constructor for nav2_controller::ControllerServer
   * @param options Additional options to control creation of the node.
   */
  explicit ControllerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief Destructor for nav2_controller::ControllerServer
   */
  ~ControllerServer();

protected:
  /**
   * @brief Configures controller parameters and member variables
   *
   * Configures controller plugin and costmap; Initialize odom subscriber,
   * velocity publisher and follow path action server.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   * @throw pluginlib::PluginlibException When failed to initialize controller
   * plugin
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates member variables
   *
   * Activates controller, costmap, velocity publisher and follow path action
   * server
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates member variables
   *
   * Deactivates follow path action server, controller, costmap and velocity
   * publisher. Before calling deactivate state, velocity is being set to zero.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Calls clean up states and resets member variables.
   *
   * Controller and costmap clean up state is called, and resets rest of the
   * variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in Shutdown state
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  using Action = nav2_msgs::action::FollowPath;
  using ActionServer = nav2_util::SimpleActionServer<Action>;

  // Our action server implements the FollowPath action
  std::unique_ptr<ActionServer> action_server_;

  /**
   * @brief FollowPath action server callback. Handles action server updates and
   * spins server until goal is reached
   *
   * Provides global path to controller received from action client. Twist
   * velocities for the robot are calculated and published using controller at
   * the specified rate till the goal is reached.
   * @throw nav2_core::PlannerException
   */
  void computeControl();

  /**
   * @brief Find the valid controller ID name for the given request
   *
   * @param c_name The requested controller name
   * @param name Reference to the name to use for control if any valid available
   * @return bool Whether it found a valid controller to use
   */
  bool findControllerId(const std::string & c_name, std::string & name);

  /**
   * @brief Find the valid goal checker ID name for the specified parameter
   *
   * @param c_name The goal checker name
   * @param name Reference to the name to use for goal checking if any valid available
   * @return bool Whether it found a valid goal checker to use
   */
  bool findGoalCheckerId(const std::string & c_name, std::string & name);

  /**
   * @brief Assigns path to controller
   * @param path Path received from action server
   */
  void setPlannerPath(const nav_msgs::msg::Path & path);
  /**
   * @brief Calculates velocity and publishes to "cmd_vel" topic
   */
  void computeAndPublishVelocity();
  /**
   * @brief Reset the near-goal progress checker baseline
   */
  void resetNearGoalProgress();
  /**
   * @brief Check if the robot is close enough to use relaxed near-goal progress rules
   * @param pose Current robot pose
   * @return true if the robot is within the near-goal progress distance
   */
  bool isNearGoal(const geometry_msgs::msg::PoseStamped & pose);
  /**
   * @brief Check relaxed progress while close to the goal
   * @param pose Current robot pose
   * @return true if progress is acceptable
   */
  bool checkNearGoalProgress(const geometry_msgs::msg::PoseStamped & pose);
  /**
   * @brief Reset the near-goal lateral-error stagnation checker
   */
  void resetNearGoalLateralStall();
  /**
   * @brief 按当前 goal_checker 的接受容差判断近目标误差
   * @param pose 当前机器人位姿
   * @param mark_goal_reached_on_success 满足时是否标记为兜底到位成功
   * @param check_yaw 是否同时检查角度容差；预检查位置窗口时可只看位置
   * @return true 表示当前误差已满足接受容差
   */
  bool isWithinGoalCheckerTolerance(
    const geometry_msgs::msg::PoseStamped & pose,
    bool mark_goal_reached_on_success,
    bool check_yaw = true);
  /**
   * @brief Check the current odometry against the active goal checker's stop tolerances
   * @return true if translational and rotational velocities are both stopped
   */
  bool isRobotStoppedForGoalChecker();
  /**
   * @brief 判断新 path 是否仍指向当前目标点
   * @param path 待更新的全局路径
   * @return true 表示新 path 终点与当前目标点基本一致
   */
  bool isSamePathGoal(const nav_msgs::msg::Path & path);
  /**
   * @brief Calls setPlannerPath method with an updated path received from
   * action server
   */
  void updateGlobalPath();
  /**
   * @brief Calls velocity publisher to publish the velocity on "cmd_vel" topic
   * @param velocity Twist velocity to be published
   */
  void publishVelocity(const geometry_msgs::msg::TwistStamped & velocity);
  /**
   * @brief Calls velocity publisher to publish zero velocity
   */
  void publishZeroVelocity();
  /**
   * @brief Checks if goal is reached
   * @return true or false
   */
  bool isGoalReached();
  /**
   * @brief Obtain current pose of the robot
   * @param pose To store current pose of the robot
   * @return true if able to obtain current pose of the robot, else false
   */
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief get the thresholded velocity
   * @param velocity The current velocity from odometry
   * @param threshold The minimum velocity to return non-zero
   * @return double velocity value
   */
  double getThresholdedVelocity(double velocity, double threshold)
  {
    return (std::abs(velocity) > threshold) ? velocity : 0.0;
  }

  /**
   * @brief get the thresholded Twist
   * @param Twist The current Twist from odometry
   * @return Twist Twist after thresholds applied
   */
  nav_2d_msgs::msg::Twist2D getThresholdedTwist(const nav_2d_msgs::msg::Twist2D & twist)
  {
    nav_2d_msgs::msg::Twist2D twist_thresh;
    twist_thresh.x = getThresholdedVelocity(twist.x, min_x_velocity_threshold_);
    twist_thresh.y = getThresholdedVelocity(twist.y, min_y_velocity_threshold_);
    twist_thresh.theta = getThresholdedVelocity(twist.theta, min_theta_velocity_threshold_);
    return twist_thresh;
  }

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;

  // The controller needs a costmap node
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;

  // Publishers and subscribers
  std::unique_ptr<nav_2d_utils::OdomSubscriber> odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    goal_error_publisher_;
  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_sub_;

  // Progress Checker Plugin
  pluginlib::ClassLoader<nav2_core::ProgressChecker> progress_checker_loader_;
  nav2_core::ProgressChecker::Ptr progress_checker_;
  std::string default_progress_checker_id_;
  std::string default_progress_checker_type_;
  std::string progress_checker_id_;
  std::string progress_checker_type_;

  // Goal Checker Plugin
  pluginlib::ClassLoader<nav2_core::GoalChecker> goal_checker_loader_;
  GoalCheckerMap goal_checkers_;
  std::vector<std::string> default_goal_checker_ids_;
  std::vector<std::string> default_goal_checker_types_;
  std::vector<std::string> goal_checker_ids_;
  std::vector<std::string> goal_checker_types_;
  std::string goal_checker_ids_concat_, default_goal_checker_, current_goal_checker_;

  // Controller Plugins
  pluginlib::ClassLoader<nav2_core::Controller> lp_loader_;
  ControllerMap controllers_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> controller_ids_;
  std::vector<std::string> controller_types_;
  std::string controller_ids_concat_, default_controller_, current_controller_;

  double controller_frequency_;
  double min_x_velocity_threshold_;
  double min_y_velocity_threshold_;
  double min_theta_velocity_threshold_;

  double failure_tolerance_;

  bool near_goal_progress_enabled_{true};
  bool near_goal_hold_position_when_pose_reached_{false};
  bool near_goal_pose_hold_active_{false};
  // near-goal hold 进入门控：位姿满足容差且底盘实际速度已足够低时才允许 ControllerServer 接管，
  // 避免在底盘仍有较大惯性时抢占 MPPI 的动态减速过程
  double near_goal_hold_enter_vx_{0.01};
  double near_goal_hold_enter_wz_{0.03};
  // near-goal hold 释放迟滞：停稳后误差需超过更宽松的阈值才允许释放，避免边界抖动反复抢占
  double near_goal_hold_release_lon_tolerance_{0.04};
  double near_goal_hold_release_lat_tolerance_{0.04};
  double near_goal_hold_release_yaw_tolerance_{0.04};
  // hold 释放宽限计数：误差持续超出 accept 容差但未超 release 容差时，累计若干周期后释放，
  // 避免在 0.03~0.04 区间永久卡死（GoalChecker 不成功且 hold 不释放）
  size_t near_goal_hold_outside_accept_count_{0};
  size_t near_goal_hold_outside_accept_cycles_{3};
  double near_goal_progress_distance_{0.25};
  double near_goal_progress_required_movement_radius_{0.01};
  double near_goal_progress_required_movement_angle_{0.005};
  rclcpp::Duration near_goal_progress_time_allowance_{
    rclcpp::Duration::from_seconds(8.0)};
  bool near_goal_lateral_stall_enabled_{false};
  double near_goal_lateral_stall_lon_tolerance_{0.02};
  double near_goal_lateral_stall_yaw_tolerance_{0.05};
  double near_goal_lateral_stall_lat_tolerance_{0.02};
  double near_goal_lateral_stall_min_improvement_{0.005};
  rclcpp::Duration near_goal_lateral_stall_time_allowance_{
    rclcpp::Duration::from_seconds(3.0)};
  int near_goal_lateral_stall_entry_count_{0};
  bool near_goal_lateral_stall_monitoring_{false};
  double near_goal_lateral_stall_baseline_error_{0.0};
  double near_goal_lateral_stall_best_error_{0.0};
  rclcpp::Time near_goal_lateral_stall_start_time_;
  bool near_goal_baseline_pose_set_{false};
  bool in_near_goal_progress_mode_{false};
  bool near_goal_progress_acceptance_reached_{false};
  geometry_msgs::msg::PoseStamped near_goal_baseline_pose_;
  rclcpp::Time near_goal_baseline_time_;

  // Whether we've published the single controller warning yet
  geometry_msgs::msg::PoseStamped end_pose_;

  // Last time the controller generated a valid command
  rclcpp::Time last_valid_cmd_time_;

  // Current path container
  nav_msgs::msg::Path current_path_;

private:
  /**
    * @brief Callback for speed limiting messages
    * @param msg Shared pointer to nav2_msgs::msg::SpeedLimit
    */
  void speedLimitCallback(const nav2_msgs::msg::SpeedLimit::SharedPtr msg);
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__CONTROLLER_SERVER_HPP_
