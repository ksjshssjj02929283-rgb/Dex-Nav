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

#include <array>
#include <cstddef>
#include <memory>
#include <string>

#include "nav2_mppi_controller/tools/path_handler.hpp"
#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/tools/trajectory_visualizer.hpp"
#include "nav2_mppi_controller/models/constraints.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace nav2_mppi_controller
{

using namespace mppi;  // NOLINT

/**
 * @class mppi::MPPIController
 * @brief Main plugin controller for MPPI Controller
 */
// 三阶段枚举（消灭 magic index 0/1/2）：数组顺序 = 枚举值
enum Stage : size_t { STAGE_ALIGN = 0, STAGE_TRACK = 1, STAGE_APPROACH = 2 };

// approach 内部子阶段：先修正目标坐标系位置误差，再完成最终朝向。
enum ApproachSubstage : size_t {
  APPROACH_POSITIONING = 0,
  APPROACH_FINAL_YAW = 1
};

// 进入 approach 时按当时的最终航向误差锁定位置调整策略，避免过程中来回切换。
enum ApproachMode : size_t {
  APPROACH_NORMAL = 0,
  APPROACH_POSITION_FIRST = 1
};

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
  /**
    * @brief Visualize trajectories
    * @param transformed_plan Transformed input plan
    */
  void visualize(nav_msgs::msg::Path transformed_plan);

  void configureStages();

  // 根据路径进度和到目标距离选择三阶段 profile，并把速度/行为代价平滑应用到 optimizer。
  void applyProfileForPathState(
    double path_progress, double goal_distance, double heading_path_angle,
    double goal_longitudinal_error, double goal_lateral_error, double goal_yaw_error);

  void updateApproachMode(size_t previous_stage, size_t selected_stage, double goal_yaw_error);

  void updateApproachSubstage(double goal_longitudinal_error, double goal_lateral_error);

  void applyApproachSubstageProfile();

  size_t selectStage(double goal_distance, double heading_path_angle) const;

  std::string stageName(size_t profile) const;

  std::string approachSubstageName() const;

  std::string approachModeName() const;

  MppiProfileSettings interpolateProfiles(
    const MppiProfileSettings & from,
    const MppiProfileSettings & to,
    double ratio) const;

  void updateProfileTransition();

  void publishProfileDebug(double path_progress, double goal_distance);

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
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>>
  profile_debug_publisher_;

  bool visualize_;
  std::array<MppiProfileSettings, 3> stages_;
  MppiProfileSettings applied_profile_;
  MppiProfileSettings transition_start_profile_;
  MppiProfileSettings transition_target_profile_;
  rclcpp::Time profile_transition_start_time_;
  double profile_start_distance_{0.8};
  double approach_distance_{1.0};
  double profile_transition_duration_{0.0};
  double approach_hysteresis_{0.05};
  double align_enter_angle_{0.52};   // 对齐段进入阈值(rad,~30°)
  double align_exit_angle_{0.31};    // 对齐段退出阈值(rad,~18°,迟滞)
  double final_yaw_enter_lon_tolerance_{0.015};
  double final_yaw_enter_lat_tolerance_{0.015};
  double final_yaw_exit_lon_tolerance_{0.02};
  double final_yaw_exit_lat_tolerance_{0.02};
  double position_first_yaw_threshold_{2.094};
  float positioning_goal_angle_weight_{10.0f};
  float final_yaw_vx_max_{0.03f};
  float final_yaw_wz_max_{0.4f};
  size_t active_stage_{0};
  size_t approach_substage_{APPROACH_POSITIONING};
  size_t approach_mode_{APPROACH_NORMAL};
  bool approach_mode_locked_{false};
  bool profile_transition_active_{false};

  double reset_period_;
  // Last time computeVelocityCommands was called
  rclcpp::Time last_time_called_;

  // ⭐ costmap 异步解耦：控制线程读快照副本，try_lock 刷新，永不阻塞等 costmap 更新锁
  nav2_costmap_2d::Costmap2D costmap_snapshot_;
  bool costmap_snapshot_initialized_{false};
};

}  // namespace nav2_mppi_controller

#endif  // NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_
