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
#include <optional>
#include <string>

#include "nav2_mppi_controller/tools/path_handler.hpp"
#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/tools/trajectory_visualizer.hpp"
#include "nav2_mppi_controller/models/constraints.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
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
enum ApproachSubstage : size_t
{
  APPROACH_POSITIONING = 0,
  APPROACH_FINAL_YAW = 1
};

// 进入 approach 时按当时的最终航向误差锁定位置调整策略，避免过程中来回切换。
enum ApproachMode : size_t
{
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
  // 规划器通过首尾零位移 marker 显式传递 orientation 模式。START 必须与 AUTO
  // 分开，否则控制器无法保证 START 平移阶段绝不输出角速度。
  enum class PathControlMode
  {
    AUTO = 0,
    START,
    TANGENT,
    GOAL,
    NONE
  };

  // 历史类型名保留 TangentPhase 以减少部署接口变化，实际由 TANGENT/GOAL 共用：
  // TANGENT 对齐出发切线，GOAL 对齐用户 goal yaw；START/AUTO/NONE 不进入该状态机。
  enum class TangentPhase
  {
    DISABLED = 0,
    START_STOP,
    START_TANGENT_ALIGN,
    START_SETTLE,
    TRACK
  };

  /**
    * @brief Visualize trajectories
    * @param transformed_plan Transformed input plan
    */
  void visualize(nav_msgs::msg::Path transformed_plan);

  /** 识别并移除 Theta* orientation mode marker，返回路径的显式控制模式。 */
  PathControlMode configurePlanForControl(
    const nav_msgs::msg::Path & path, nav_msgs::msg::Path & tracking_path);

  /** 只有 tangent 和 goal 需要起点旋转；start/auto/none 直接进入 FOLLOW。 */
  bool shouldRunStartControl(PathControlMode mode) const;

  /** 判断全局规划更新是否仍属于当前导航目标；恢复重试期间不得重复起点对齐。 */
  bool isSameNavigationGoal(const geometry_msgs::msg::Pose & goal) const;

  /**
   * @brief 在 controller 坐标系的路径上按固定物理前视提取并锁存一次性起点 yaw
   * @return 路径非空且成功得到目标 yaw 时为 true
   */
  bool configureStartPoseInControllerFrame(const nav_msgs::msg::Path & transformed_path);

  /** 起点阶段拥有控制权时返回停车/旋转命令；TRACK 时返回空。 */
  std::optional<geometry_msgs::msg::TwistStamped> computeTangentStartCommand(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed);

  geometry_msgs::msg::TwistStamped makeStoppedCommand() const;

  geometry_msgs::msg::TwistStamped makeRotationCommand(
    double yaw_error,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    double yaw_tolerance);

  void ensureRotationIsCollisionFree(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::TwistStamped & command,
    double yaw_error) const;

  static const char * tangentPhaseName(TangentPhase phase);

  static const char * pathControlModeName(PathControlMode mode);

  void configureStages();

  // tangent 起点对齐完成后（其他模式直接进入），根据到目标距离选择 TRACK/APPROACH。
  void applyProfileForPathState(
    double path_progress, double goal_distance,
    double goal_longitudinal_error, double goal_lateral_error, double goal_yaw_error);

  void updateApproachMode(
    size_t previous_stage, size_t selected_stage, double goal_longitudinal_error,
    double goal_lateral_error, double goal_yaw_error);

  void updateApproachSubstage(double goal_longitudinal_error, double goal_lateral_error);

  void applyApproachSubstageProfile();

  /** 按剩余距离连续收紧平移速度上限，避免在 approach_distance 处阶跃刹车。 */
  void applyApproachBrakingEnvelope(double goal_distance);

  /**
   * @brief 位置进入严格 2 cm 后，停止平移并直接完成最终 yaw
   *
   * 该阶段不再运行第二套 MPPI profile，避免目标角 critic 与位置 critic 争夺控制权。
   */
  geometry_msgs::msg::TwistStamped computeFinalGoalYawCommand(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    double goal_yaw_error);

  /** START/GOAL 的平移语义是固定朝向，禁止 MPPI 采样噪声产生边走边转。 */
  void enforceFixedCruiseYaw(geometry_msgs::msg::TwistStamped & cmd) const;

  /**
   * @brief 末段把“方向正确但底盘无法执行”的小平移命令抬到可执行合速度
   *
   * 只缩放 vx/vy 向量，不改变 MPPI 给出的方向；零命令、背离目标的命令和
   * 速度约束不足时不会被强制启动。
   */
  void enforceTerminalExecutableTranslation(
    geometry_msgs::msg::TwistStamped & cmd,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double goal_x, double goal_y, double goal_distance);

  /** 纯数学内核，供控制器和单元测试共用。返回 true 表示命令被抬升。 */
  static bool liftTerminalTranslationCommand(
    geometry_msgs::msg::TwistStamped & cmd,
    double goal_body_x, double goal_body_y,
    float minimum_speed, float command_deadband, float minimum_goal_alignment,
    const models::ControlConstraints & constraints);

  void smoothFinalYawCommand(
    geometry_msgs::msg::TwistStamped & cmd,
    double goal_yaw_error,
    const geometry_msgs::msg::Twist & robot_speed);

  size_t selectStage(double goal_distance) const;

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
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  tangent_collision_checker_;
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
  // 单套 MPPI 终点制动包络使用的平移减速度。
  double approach_braking_deceleration_{0.5};
  // 离 2 cm 门槛只剩很小距离时保留的最低速度上限，避免限幅降为零后停在门外。
  double terminal_min_translation_speed_{0.05};
  // 只在终点附近启用可执行速度抬升，远处完全尊重 MPPI 原始速度规划。
  double terminal_translation_assist_distance_{0.30};
  // 小于该值视为 MPPI 明确停车/数值噪声，不得强行启动底盘。
  double terminal_translation_command_deadband_{0.005};
  // 命令与目标方向夹角余弦下限；默认 0.5 即要求夹角小于 60°。
  double terminal_translation_min_goal_alignment_{0.5};
  double profile_transition_duration_{0.0};
  double approach_hysteresis_{0.05};
  double final_yaw_enter_lon_tolerance_{0.015};
  double final_yaw_enter_lat_tolerance_{0.015};
  double final_yaw_exit_lon_tolerance_{0.02};
  double final_yaw_exit_lat_tolerance_{0.02};
  double position_first_yaw_threshold_{2.094};
  double position_first_lateral_error_threshold_{0.05};
  double position_first_lateral_ratio_{1.0};
  // positioning 阶段默认完全关闭最终朝向；只有位置进入硬性容差后才由 final_yaw 接管。
  float positioning_goal_angle_weight_{0.0f};
  float final_yaw_vx_max_{0.03f};
  float final_yaw_vy_max_{0.03f};
  float final_yaw_wz_max_{0.4f};
  bool final_yaw_lock_translation_{true};
  float final_yaw_cmd_wz_slew_rate_{1.2f};
  float final_yaw_cmd_brake_accel_{0.7f};
  float final_yaw_cmd_stop_yaw_tolerance_{0.0f};
  float final_yaw_cmd_direction_deadband_{0.05f};
  size_t active_stage_{STAGE_TRACK};
  size_t approach_substage_{APPROACH_POSITIONING};
  size_t approach_mode_{APPROACH_NORMAL};
  bool approach_mode_locked_{false};
  bool profile_transition_active_{false};
  rclcpp::Time active_stage_enter_time_;
  bool final_yaw_cmd_filter_active_{false};
  bool final_yaw_direction_lock_active_{false};
  // 终点旋转开始前只等待一次底盘停稳；开始后不能再把正常角速度误判成未停稳。
  bool final_yaw_rotation_started_{false};
  float final_yaw_locked_direction_{0.0f};
  float last_final_yaw_cmd_wz_{0.0f};
  rclcpp::Time last_final_yaw_cmd_time_;

  double reset_period_;
  // Last time computeVelocityCommands was called
  rclcpp::Time last_time_called_;

  // 只有带 Theta* tangent marker 的路径才启用该起点状态机。
  bool tangent_phase_control_enabled_{false};
  PathControlMode current_path_control_mode_{PathControlMode::AUTO};
  TangentPhase tangent_phase_{TangentPhase::DISABLED};
  geometry_msgs::msg::Pose tangent_cruise_start_pose_;
  // setPlan() 时还没有当前机器人姿态，不能安全做 map->odom 变换；新任务首个控制
  // 周期从已变换路径提取 yaw 后清零，同目标重规划不得重新打开该标志。
  bool start_yaw_target_pending_{false};
  // 所有模式共用的当前导航目标，仅用于区分“同目标重规划”和“真正更换目标”。
  geometry_msgs::msg::Pose tangent_goal_pose_;
  bool navigation_goal_initialized_{false};
  double tangent_rotate_kp_{1.5};
  double tangent_rotate_max_speed_{0.5};
  double tangent_rotate_min_speed_{0.08};
  double tangent_rotate_max_accel_{0.7};
  double tangent_start_yaw_tolerance_{0.03};
  // 以下两个容差只用于识别“同一目标的重规划”，不再用于终点停车/旋转接管。
  double tangent_goal_yaw_tolerance_{0.03};
  double tangent_goal_xy_tolerance_{0.05};
  double tangent_stopped_linear_velocity_{0.03};
  double tangent_stopped_angular_velocity_{0.03};
  double tangent_marker_position_tolerance_{1e-4};
  // tangent 不能直接用相邻点计算起步方向；规划栅格噪声会让近点方向失真。
  double start_yaw_lookahead_distance_{0.30};
  double tangent_rotation_simulate_ahead_time_{0.5};
  double control_duration_{0.1};
  std::string tangent_command_frame_;
  // 起点旋转限加速度必须基于上一周期命令，而不是可能仍为零的里程计速度。
  double last_start_rotation_command_wz_{0.0};
  bool start_rotation_command_initialized_{false};

  // ⭐ costmap 异步解耦：控制线程读快照副本，try_lock 刷新，永不阻塞等 costmap 更新锁
  nav2_costmap_2d::Costmap2D costmap_snapshot_;
  bool costmap_snapshot_initialized_{false};
};

}  // namespace nav2_mppi_controller

#endif  // NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_
