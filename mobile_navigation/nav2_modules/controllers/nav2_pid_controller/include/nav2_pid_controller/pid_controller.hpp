// Copyright (c) 2023 Yang Haodong, Guo Zhanyu, Wu Maojia (original ROS1 dcm_navigation)
// Copyright (c) 2026 dexforce (ROS2 / nav2_core port)
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

#ifndef PID_CONTROLLER__PID_CONTROLLER_HPP_
#define PID_CONTROLLER__PID_CONTROLLER_HPP_

#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_interfaces/msg/robot_trajectory.hpp"

#include "tf2_ros/buffer.h"

#include "nav2_controller_common/pid_controller.h"

namespace nav2_pid_controller
{

/**
 * @class nav2_pid_controller::PIDController
 * @brief A nav2_core::Controller plugin implementing a PID local controller.
 *
 * Linear velocity: in APPROACHING the speed loop tracks the planned speed of
 * the closest trajectory point (sign from the longitudinal error) — decel
 * tails / curvature slowdowns / obstacle zeros are encoded per-point by the
 * speed planner; in the final-rotation stage a small signed compensation speed
 * is generated from the position error. A high/low-speed gain set is switched
 * at switch_speed_, plus a dedicated reverse gain set. Angular velocity is
 * controlled by a single station PID. A three-stage state machine
 * (APPROACHING -> ROTATING_TO_GOAL) governs the behavior. Trajectories (with
 * per-point longitudinal_velocity / path_kappa) are consumed through the
 * nav2_core setTrajectory() extension; plain geometric paths are accepted
 * through setPlan() with a default desired speed.
 */
class PIDController : public nav2_core::Controller
{
public:
  PIDController() = default;
  ~PIDController() override = default;

  /**
   * @brief Configure controller on bringup
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller state
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
   * @brief Set the geometric global plan (no speed information; uses default 0).
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Set the time-space trajectory (L5 SpeedPlanner output). Consumes
   *        per-point longitudinal_velocity as desired speed and path_kappa as
   *        curvature, caching geometry + speed for computeVelocityCommands.
   * @param trajectory The robot trajectory
   */
  void setTrajectory(const nav_interfaces::msg::RobotTrajectory & trajectory) override;

  /**
   * @brief Compute the best command given the current pose and velocity
   * @param pose Current robot pose (in costmap global frame)
   * @param velocity Current robot velocity (in base frame)
   * @param goal_checker Goal checker for this task
   * @return Best command for the robot to drive
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief Limit the maximum linear speed of the robot.
   * @param speed_limit absolute (m/s) or percentage of max speed
   * @param percentage true if speed_limit is a percentage
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  /**
   * @brief Check if the goal pose has been achieved
   * @return True if achieved, false otherwise
   */
  bool isGoalReached();

  /**
   * @brief Enable / disable reverse (backward) driving.
   * @param enabled true to drive in reverse
   */
  void setReverseEnabled(const bool enabled) {reverse_enabled_ = enabled;}

protected:
  /**
   * @brief Single cached trajectory point: geometry + segment speed/curvature.
   */
  struct TrajPoint
  {
    geometry_msgs::msg::PoseStamped pose;
    double speed_limit = 0.0;  // desired longitudinal velocity (m/s)
    double curvature = 0.0;    // path_kappa
    double accel = 0.0;        // 规划加速度，供速度环前馈（几何路径回退时恒 0）
  };

  enum ControllerStage
  {
    FINISH,
    APPROACHING,
    ROTATING_TO_GOAL
  };

  inline void regularizeAngle(double & angle)
  {
    angle = angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
  }

  double CalculateMean(const std::deque<double> & data);
  double CalculateVariance(const std::deque<double> & data, double mean);

  /**
   * @brief Find the closest waypoint id on the cached trajectory.
   *        Ported from nav_util::getClosestWaypointID.
   */
  int getClosestWaypointID(
    const std::vector<TrajPoint> & traj,
    const geometry_msgs::msg::PoseStamped & pose,
    double ignore_dist = 0.0,
    bool ignore_first = false);

  /**
   * @brief Erase trajectory points behind the robot (within dist_behind_robot).
   */
  bool pruneGlobalPlan(
    const geometry_msgs::msg::PoseStamped & global_pose,
    std::vector<TrajPoint> & traj,
    double dist_behind_robot);

  bool shouldRotateToPath(
    const geometry_msgs::msg::PoseStamped & carrot_pose,
    double & angle_to_path);

  void rotateToHeading(
    double & linear_vel, double & angular_vel,
    const double & angle_to_path,
    const geometry_msgs::msg::Twist & curr_vel);

  /**
   * @brief Linear control: position error generates a signed target speed,
   *        then the speed PID tracks that target.
   * @param velocity current robot velocity (base frame)
   * @param e_dist longitudinal position error in base frame
   * @return linear velocity command
   */
  double LinearSpeedControl(
    const geometry_msgs::msg::Twist & velocity,
    double e_dist,
    double max_linear_speed = std::numeric_limits<double>::quiet_NaN(),
    double min_target_speed = 0.0,
    double position_deadband = 0.0,
    double feedforward_accel = 0.0);

  /**
   * @brief Angular PID control.
   * @param velocity current robot velocity (base frame)
   * @param e_theta heading error
   * @return angular velocity command
   */
  double AngularSpeedControl(
    const geometry_msgs::msg::Twist & velocity,
    double e_theta,
    bool final_rotation = false);

  /**
   * @brief Human-readable name for controller stage logging.
   */
  const char * controllerStageName(ControllerStage stage) const;

  /**
   * @brief Build the cached trajectory from a geometric path (speed = 0),
   *        transforming each pose into the costmap global frame.
   */
  bool cachePathAsTrajectory(const nav_msgs::msg::Path & path);

  // ---- ROS2 plumbing ----
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("PIDController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string global_frame_;
  std::string base_frame_;

  // ---- plan / trajectory cache ----
  std::vector<TrajPoint> controller_traj_;
  std::mutex plan_mutex_;
  int plan_index_ = 0;
  geometry_msgs::msg::PoseStamped target_ps_;
  geometry_msgs::msg::PoseStamped current_ps_;

  double goal_x_ = 0.0, goal_y_ = 0.0, goal_yaw_ = 0.0;

  // ---- tolerances / limits ----
  double long_precision_ = 0.02, lat_precision_ = 0.04, o_precision_ = 0.5;
  double angular_error_deadband_ = 0.008;
  double d_t_ = 0.05;  // control time step (1 / controller_frequency)

  double max_v_ = 0.5, min_v_ = 0.0, max_v_inc_ = 0.5;  // linear velocity
  double max_w_ = 1.57, min_w_ = 0.0, max_w_inc_ = 1.57;  // angular velocity

  double k_theta_ = 0.5;            // heading-weighting factor
  double switch_speed_ = 0.1;       // high/low speed switch threshold
  double default_desired_linear_vel_ = 0.15; // fallback speed for geometric paths
  double desired_linear_vel_ = 0.0; // per-point desired velocity
  double final_rotation_max_v_ = 0.05;
  double final_rotation_linear_deadband_ = 0.005;
  double acc_max_ = 0.5;

  // upper speed bound from setSpeedLimit (taken min with trajectory speed)
  double speed_limit_ = std::numeric_limits<double>::infinity();

  bool reverse_enabled_ = false;

  // ---- state machine ----
  ControllerStage curr_controller_stage_ = FINISH;
  ControllerStage prev_controller_stage_ = FINISH;

  std::deque<double> theta_errors_;
  std::deque<double> longitude_errors_;

  double last_linear_position_cmd_ = 0.0;
  double last_linear_target_v_ = 0.0;
  double last_linear_speed_error_ = 0.0;
  double last_linear_accel_cmd_ = 0.0;
  double last_linear_accel_limited_ = 0.0;
  double last_linear_limit_ = 0.0;
  double last_linear_min_target_v_ = 0.0;
  double last_linear_deadband_ = 0.0;

  // ---- linear PID controllers ----
  nav2_controller_common::PIDController station_nav2_pid_controller_;
  nav2_controller_common::PIDController speed_nav2_pid_controller_;

  nav2_controller_common::PidConf high_speed_station_pid_conf_;
  nav2_controller_common::PidConf low_speed_station_pid_conf_;
  nav2_controller_common::PidConf high_speed_pid_conf_;
  nav2_controller_common::PidConf low_speed_pid_conf_;
  nav2_controller_common::PidConf reverse_station_pid_conf_;
  nav2_controller_common::PidConf reverse_speed_pid_conf_;

  // ---- angular PID controller ----
  nav2_controller_common::PIDController angular_station_nav2_pid_controller_;
  nav2_controller_common::PidConf angular_station_pid_conf_;
};

}  // namespace nav2_pid_controller

#endif  // PID_CONTROLLER__PID_CONTROLLER_HPP_
