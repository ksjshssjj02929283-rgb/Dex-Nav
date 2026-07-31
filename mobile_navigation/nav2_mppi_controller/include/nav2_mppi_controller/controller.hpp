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
#include <cstdint>
#include <memory>
#include <string>

#include "nav2_mppi_controller/tools/path_handler.hpp"
#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/tools/trajectory_visualizer.hpp"
#include "nav2_mppi_controller/models/constraints.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_controller/near_goal_odom_context.hpp"
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
// MPPI控制阶段使用强类型枚举，业务逻辑禁止使用数字表示状态。
enum class Stage : size_t
{
  ALIGN,
  TRACK,
  APPROACH
};

// approach 内部子阶段：先修正目标坐标系位置误差，再完成最终朝向。
enum class ApproachSubstage : size_t
{
  POSITIONING,
  FINAL_YAW
};

// 进入 approach 时按当时的最终航向误差锁定位置调整策略，避免过程中来回切换。
enum class ApproachMode : size_t
{
  NORMAL,
  POSITION_FIRST
};

class MPPIController
  : public nav2_core::Controller,
    public nav2_controller::NearGoalOdomContextConsumer
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
    * @brief 消费速度规划轨迹（④ 产出）：每个控制周期取最近轨迹点的规划速度，
    *        作为采样器的前向 vx 上限（叠加在三阶段 profile 约束之上）。
    * @param trajectory 带逐点速度的时空轨迹
    */
  void setTrajectory(const nav_interfaces::msg::RobotTrajectory & trajectory) override;

  /**
    * @brief Set new speed limit from callback
    * @param speed_limit Speed limit to use
    * @param percentage Bool if the speed limit is absolute or relative
    */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  // 注入ControllerServer维护的只读末端状态，不在MPPI内推进状态机。
  void setNearGoalOdomContext(
    std::shared_ptr<const nav2_controller::NearGoalOdomSnapshotProvider> provider,
    uint64_t expected_generation) override;

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

  void updateApproachMode(
    Stage previous_stage, Stage selected_stage, double goal_longitudinal_error,
    double goal_lateral_error, double goal_yaw_error);

  void updateApproachSubstage(double goal_longitudinal_error, double goal_lateral_error);

  void applyApproachSubstageProfile();

  void smoothFinalYawCommand(
    geometry_msgs::msg::TwistStamped & cmd,
    double goal_yaw_error,
    const geometry_msgs::msg::Twist & robot_speed);

  // 控制指令经历HOLD/恢复后清空末端滤波与制动状态，下一帧从实际速度重新初始化。
  void resetFinalYawCommandState();

  // 新目标到来时清除上一目标的阶段和Profile，确保首周期从ALIGN开始。
  void resetForNewGoal();

  Stage selectStage(double goal_distance, double heading_path_angle) const;

  const char * stageName(Stage stage) const;

  const char * approachSubstageName() const;

  const char * approachModeName() const;

  // 返回末端航向控制状态名称，避免日志输出难维护的数字状态。
  const char * finalYawControlStateName() const;

  // 将具名控制阶段映射到内部profile存储位置，业务逻辑不直接使用数组下标。
  size_t profileStorageIndex(Stage stage) const;

  MppiProfileSettings & profileFor(Stage stage);

  const MppiProfileSettings & profileFor(Stage stage) const;

  // 仅用于兼容既有Float64调试Topic，控制逻辑和日志禁止使用这些数字编码。
  double profileDebugStageCode() const;

  double profileDebugSubstageCode() const;

  double profileDebugModeCode() const;

  MppiProfileSettings interpolateProfiles(
    const MppiProfileSettings & from,
    const MppiProfileSettings & to,
    double ratio) const;

  void updateProfileTransition();

  void publishProfileDebug(double path_progress, double goal_distance);

  /// 最近轨迹点的规划速度（机器人位姿先转到轨迹坐标系）；无轨迹/TF 失败返回 -1（不设帽）。
  double nearestTrajectorySpeed(const geometry_msgs::msg::PoseStamped & robot_pose) const;

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
  double track_stage_min_duration_{1.2};
  double align_force_enter_angle_{1.2};
  double final_yaw_enter_lon_tolerance_{0.015};
  double final_yaw_enter_lat_tolerance_{0.015};
  double final_yaw_exit_lon_tolerance_{0.02};
  double final_yaw_exit_lat_tolerance_{0.02};
  // final_yaw 退出连续违反计数：位置误差连续 N 个周期超退出阈值才真正退出，避免原地转向的厘米级漂移
  // 导致 final_yaw <-> positioning 反复切换
  size_t final_yaw_exit_violation_count_{0};
  size_t final_yaw_exit_violation_threshold_{3};
  double position_first_yaw_threshold_{2.094};
  double position_first_lateral_error_threshold_{0.05};
  double position_first_lateral_ratio_{1.0};
  float positioning_goal_angle_weight_{10.0f};
  float final_yaw_vx_max_{0.03f};
  float final_yaw_wz_max_{0.4f};
  float final_yaw_cmd_wz_slew_rate_{1.2f};
  float final_yaw_cmd_brake_accel_{0.7f};
  float final_yaw_cmd_stop_yaw_tolerance_{0.0f};
  float final_yaw_cmd_direction_deadband_{0.05f};
  float final_yaw_cmd_actual_decel_{0.4f};  // 底盘实际角减速度估计，用于动态制动距离计算
  float final_yaw_cmd_delay_{0.15f};        // 控制周期+平滑器+底盘通信延迟，用于动态制动距离
  float final_yaw_cmd_brake_safety_margin_{0.01f};  // 动态制动触发安全余量
  // 动态急刹最低触发速度：低于此速度不触发急刹，交给线性曲线平滑降速，避免低速停-启卡顿
  float final_yaw_dynamic_brake_min_wz_{0.10f};
  // CREEP 低速精调阶段：动态制动结束后用此范围角速度完成最后修正，不再触发急刹
  float final_yaw_creep_wz_min_{0.04f};
  float final_yaw_creep_wz_max_{0.06f};
  float final_yaw_internal_tolerance_{0.02f};  // CREEP 进入 STOP 的内部容差，比 GoalChecker 接受容差严格，预留停止余量
  // TRACKING -> CREEP 直接入口：低速接近目标时不必走 BRAKING，直接进入 CREEP 低速精调
  float final_yaw_creep_enter_yaw_error_{0.06f};      // 误差<=此值且速度低时进入 CREEP
  float final_yaw_creep_enter_max_actual_wz_{0.10f};  // 实际角速度<此值才允许直接进 CREEP
  // STOP 状态退出阈值：比 internal_tolerance 宽松，比 direction_deadband 小，避免 STOP 后小幅漂移不修正
  float final_yaw_stop_exit_tolerance_{0.04f};
  bool final_yaw_braking_active_{false};    // 动态制动锁定状态，触发后持续输出零直到底盘停稳
  // 包络追赶制动：actual_wz 超出 safe_wz 时临时请求零速快速降速，
  // 但不锁定到完全停稳，actual 回到包络内立即恢复连续控制（区别于 BRAKING）
  bool final_yaw_envelope_braking_active_{false};
  float final_yaw_envelope_brake_enter_margin_{0.02f};  // actual > safe_wz + 此值才触发
  float final_yaw_envelope_brake_exit_margin_{0.005f};  // actual <= safe_wz + 此值才释放
  // final_yaw 内部状态机：TRACKING(正常刹车曲线) -> BRAKING(动态急刹) -> CREEP(低速精调) -> STOP
  // CREEP 阶段不会再触发动态急刹，避免"急停-微调-再急停"循环
  enum class FinalYawControlState : size_t
  {
    TRACKING,
    BRAKING,
    CREEP,
    STOP
  };
  FinalYawControlState final_yaw_control_state_{FinalYawControlState::TRACKING};
  Stage active_stage_{Stage::ALIGN};
  ApproachSubstage approach_substage_{ApproachSubstage::POSITIONING};
  ApproachMode approach_mode_{ApproachMode::NORMAL};
  bool approach_mode_locked_{false};
  bool profile_transition_active_{false};
  rclcpp::Time active_stage_enter_time_;
  bool final_yaw_cmd_filter_active_{false};
  bool final_yaw_direction_lock_active_{false};
  float final_yaw_locked_direction_{0.0f};
  float last_final_yaw_cmd_wz_{0.0f};
  rclcpp::Time last_final_yaw_cmd_time_;

  double reset_period_;
  // Last time computeVelocityCommands was called
  rclcpp::Time last_time_called_;

  // ⭐ costmap 异步解耦：控制线程读快照副本，try_lock 刷新，永不阻塞等 costmap 更新锁
  nav2_costmap_2d::Costmap2D costmap_snapshot_;
  bool costmap_snapshot_initialized_{false};
  // 速度规划轨迹缓存（④ 每控制周期分发；前瞻窗内最大规划速度 → vx 采样上限）
  nav_interfaces::msg::RobotTrajectory reference_trajectory_;
  // 前瞻窗长度（m）：帽允许领先计划加速坡的距离；窗内至少含最近点的下一点（防死锁）
  double traj_speed_cap_lookahead_{0.2};

  std::shared_ptr<const nav2_controller::NearGoalOdomSnapshotProvider>
    near_goal_odom_provider_;
  uint64_t expected_goal_generation_{0};
  uint64_t last_near_goal_control_revision_{0};
  nav2_controller::NearGoalOdomStage last_near_goal_odom_stage_{
    nav2_controller::NearGoalOdomStage::INACTIVE};

};

}  // namespace nav2_mppi_controller

#endif  // NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_
