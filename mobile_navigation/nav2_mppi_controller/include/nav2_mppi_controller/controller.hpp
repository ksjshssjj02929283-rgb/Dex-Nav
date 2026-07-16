// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#ifndef NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_
#define NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_

#include <string>
#include <memory>
#include <optional>

#include "nav2_mppi_controller/tools/path_handler.hpp"
#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/tools/trajectory_visualizer.hpp"
#include "nav2_mppi_controller/models/constraints.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_mppi_controller
{

using namespace mppi;  // NOLINT

/**
 * @class mppi::MPPIController
 * @brief Main plugin controller for MPPI Controller
 */
class MPPIController : public nav2_core::Controller
{
public:
  /**
    * @brief Constructor for mppi::MPPIController
    */
  MPPIController() = default;

  /**
    * @brief Configure controller on bringup
    * @param parent WeakPtr to node
    * @param name Name of plugin
    * @param tf TF buffer to use
    * @param costmap_ros Costmap2DROS object of environment
    */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
    * @brief Cleanup resources
    */
  void cleanup() override;

  /**
    * @brief Activate controller
    */
  void activate() override;

  /**
    * @brief Deactivate controller
    */
  void deactivate() override;

  /**
    * @brief Reset the controller state between tasks
    */
  void reset();

  /**
    * @brief Main method to compute velocities using the optimizer
    * @param robot_pose Robot pose
    * @param robot_speed Robot speed
    * @param goal_checker Pointer to the goal checker for awareness if completed task
    */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    nav2_core::GoalChecker * goal_checker) override;

  /**
    * @brief Set new reference path to track
    * @param path Path to track
    */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
    * @brief Set new speed limit from callback
    * @param speed_limit Speed limit to use
    * @param percentage Bool if the speed limit is absolute or relative
    */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  enum class TangentPhase
  {
    DISABLED = 0,
    START_STOP,
    START_YAW_ALIGN,
    START_TANGENT_ALIGN,
    START_SETTLE,
    TRACK,
    GOAL_STOP,
    GOAL_YAW_ALIGN,
    COMPLETE
  };

  /**
    * @brief Visualize trajectories
    * @param transformed_plan Transformed input plan
    */
  void visualize(nav_msgs::msg::Path transformed_plan);

  /** Detect and remove Theta* tangent endpoint rotation markers. */
  bool configureTangentPlan(
    const nav_msgs::msg::Path & path, nav_msgs::msg::Path & cruise_path);

  /** Return a rotate/stop command while a tangent phase owns control. */
  std::optional<geometry_msgs::msg::TwistStamped> computeTangentPhaseCommand(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed);

  geometry_msgs::msg::TwistStamped makeStoppedCommand(
    const geometry_msgs::msg::PoseStamped & robot_pose) const;

  geometry_msgs::msg::TwistStamped makeRotationCommand(
    double yaw_error,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed);

  void ensureRotationIsCollisionFree(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::TwistStamped & command,
    double yaw_error) const;

  static const char * tangentPhaseName(TangentPhase phase);

  std::string name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::unique_ptr<ParametersHandler> parameters_handler_;
  Optimizer optimizer_;
  PathHandler path_handler_;
  TrajectoryVisualizer trajectory_visualizer_;
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  tangent_collision_checker_;

  bool visualize_;

  double reset_period_;
  // Last time computeVelocityCommands was called
  rclcpp::Time last_time_called_;

  // Opt-in three-phase control for Theta* TANGENT marker paths. Ordinary
  // paths never enter this state machine and retain the original MPPI flow.
  bool tangent_phase_control_enabled_{false};
  TangentPhase tangent_phase_{TangentPhase::DISABLED};
  geometry_msgs::msg::Pose tangent_start_pose_;
  geometry_msgs::msg::Pose tangent_cruise_start_pose_;
  geometry_msgs::msg::Pose tangent_goal_pose_;
  double tangent_rotate_kp_{1.5};
  double tangent_rotate_max_speed_{0.5};
  double tangent_rotate_min_speed_{0.08};
  double tangent_rotate_max_accel_{0.7};
  double tangent_start_yaw_tolerance_{0.03};
  double tangent_goal_yaw_tolerance_{0.03};
  double tangent_goal_xy_tolerance_{0.05};
  double tangent_stopped_linear_velocity_{0.03};
  double tangent_stopped_angular_velocity_{0.03};
  double tangent_marker_position_tolerance_{1e-4};
  double tangent_rotation_simulate_ahead_time_{0.5};
  double control_duration_{0.1};
  std::string tangent_command_frame_;
};

}  // namespace nav2_mppi_controller

#endif  // NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_
