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
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int64.hpp"

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
   * @brief TANGENT 起点旋转完成后，将首个有效 MPPI 命令稳定接入巡航
   *
   * 底盘尚在零速对舵时保持首个轮向；只有检测到车体已经真实移动后，才在固定
   * 时间内逐步放开后续 MPPI 命令。其他模式及终点确定性控制不调用。
   */
  void smoothTangentTrackEntryCommand(
    geometry_msgs::msg::TwistStamped & cmd,
    const geometry_msgs::msg::Twist & robot_speed);

  /** TANGENT TRACK 接入的纯数学混合内核，供控制器和单元测试共用。 */
  static void blendTangentTrackEntryCommand(
    geometry_msgs::msg::Twist & command,
    const geometry_msgs::msg::Twist & entry_reference,
    double blend_ratio);

  /** AUTO 专用：限制相邻命令对应的舵轮目标角变化，不修改其他 orientation mode。 */
  bool enforceAutoSteeringContinuity(
    geometry_msgs::msg::TwistStamped & cmd,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    const geometry_msgs::msg::Twist * collision_fallback = nullptr,
    bool always_check_collision = false);

  /**
   * @brief AUTO 专用：将 MPPI 平移速度方向改写为规划路径的世界坐标前视方向
   *
   * 只保留 MPPI 给出的合速度大小；方向由已变换路径确定。调用者在舵轮可行性投影后
   * 对最终命令统一做一次短时轨迹检查，避免重复碰撞检查拖慢控制周期。
   */
  bool alignAutoTranslationWithPath(
    geometry_msgs::msg::TwistStamped & cmd,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const nav_msgs::msg::Path & transformed_plan);

  /** AUTO 路径方向改写的纯数学内核，供控制器和单元测试共用。 */
  static bool makeAutoPathAlignedTranslation(
    geometry_msgs::msg::Twist & command,
    const geometry_msgs::msg::Pose & robot_pose,
    const nav_msgs::msg::Path & transformed_plan,
    double lookahead_distance, double command_deadband);

  /**
   * @brief 限制 AUTO 世界路径方向的单周期变化
   *
   * 普通重规划只按 max_step 渐进转向；超过 emergency_reset_angle 视为规划器
   * 明确要求紧急大幅改道，立即采用新方向并交给后续碰撞检查。
   * @return 世界方向被渐进限制时为 true
   */
  static bool limitAutoPathDirectionChange(
    geometry_msgs::msg::Twist & command,
    double robot_yaw, double previous_world_yaw,
    bool history_initialized, double max_step,
    double emergency_reset_angle, double & selected_world_yaw);

  /** 用局部代价地图向前验证 AUTO 限幅后的常速度命令；失败时保留 MPPI 原命令。 */
  bool isAutoCommandCollisionFree(
    const geometry_msgs::msg::Twist & command,
    const geometry_msgs::msg::PoseStamped & robot_pose);

  /**
   * @brief AUTO 世界方向不变的舵轮运动学投影纯数学内核
   *
   * 路径层已经确定世界平移方向；本函数只允许缩放该方向上的速度大小并收紧 wz，
   * 禁止在上一命令方向和当前方向之间插值出一条新的世界曲线。
   * @return 命令因角速度/舵角连续性约束被修改时为 true
   */
  static bool projectAutoCommandForSteering(
    geometry_msgs::msg::Twist & command,
    const geometry_msgs::msg::Twist & previous_command,
    bool history_initialized,
    double previous_robot_yaw, double current_robot_yaw,
    double chassis_length, double chassis_width,
    double max_angular_speed, double max_wheel_steer_step);

  /**
   * @brief AUTO 巡航合速度低通，只改变速度大小并严格保留当前世界路径方向
   *
   * 仅在普通 MPPI 巡航分支调用；终点确定性控制、显式零速以及其他模式不使用。
   * @return 合速度因连续性约束被修改时为 true
   */
  static bool filterAutoLinearSpeedMagnitude(
    geometry_msgs::msg::Twist & command,
    const geometry_msgs::msg::Twist & previous_command,
    bool history_initialized, double filter_alpha,
    double maximum_acceleration_step, double maximum_deceleration_step);

  /**
   * @brief AUTO 角速度的低通、加速度及 jerk 连续化纯数学内核
   *
   * acceleration_state 跨周期保存上一角加速度；函数只更新传入的临时状态，
   * 调用者应在碰撞检查通过后再提交，避免不可执行命令污染滤波历史。
   */
  static double filterAutoAngularVelocity(
    double target_wz, double previous_wz, bool history_initialized,
    double filter_alpha, double maximum_acceleration,
    double maximum_jerk, double dt, double & acceleration_state);

  /**
   * @brief 按整次导航任务的单调进度重写 AUTO 路径 yaw
   *
   * 同目标重规划只替换 XY 几何，不再从当前机器人 yaw 重新压缩剩余转角。
   */
  static void applyPersistentAutoYawProfile(
    nav_msgs::msg::Path & path,
    double task_start_yaw, double task_goal_yaw,
    double task_progress, double initial_path_length,
    double completion_distance_before_goal);

  /** 终点近区启用纵横目标误差，tangent yaw 保持区额外抑制无效角速度。 */
  void applyTerminalGoalAxisWeights(double goal_distance);

  /**
   * @brief 末段生成底盘可连续执行的平移命令
   *
   * 用作确定性短距控制未接管时的 MPPI 后处理：只放大方向正确的 MPPI 向量，
   * 零命令、背离目标的命令和速度约束不足时不会被强制启动。
   */
  void enforceTerminalExecutableTranslation(
    geometry_msgs::msg::TwistStamped & cmd,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double goal_x, double goal_y, double goal_distance);

  /**
   * @brief 在 MPPI 之前生成 tangent/start 终点近区的连续位置命令
   *
   * 只有整条短直达段 footprint 检查通过才接管，否则返回空并退回 MPPI。
   */
  std::optional<geometry_msgs::msg::TwistStamped> computeTerminalPositionCommand(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    double goal_x, double goal_y, double goal_yaw, double goal_distance);

  /** TANGENT/START/AUTO 使用全向短距位置控制；AUTO 仍允许独立旋转。 */
  static bool usesDeterministicTerminalPositionControl(PathControlMode mode);

  /** 纯数学内核：按目标方向和连续距离反馈生成平移命令。 */
  static bool makeDeterministicTerminalTranslation(
    geometry_msgs::msg::TwistStamped & cmd,
    double goal_body_x, double goal_body_y, double goal_distance,
    double position_tolerance, float minimum_speed, float maximum_speed,
    float proportional_gain,
    const models::ControlConstraints & constraints);

  /** 检查从当前位姿直达位置容差边界的短平移是否保持 footprint 无碰撞。 */
  bool isDirectTerminalTranslationCollisionFree(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double goal_x, double goal_y, double goal_distance,
    double position_tolerance);

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

  void publishProfileDebug(
    double path_progress, double goal_distance,
    double transform_ms = -1.0, double costmap_lock_ms = -1.0,
    double optimize_ms = -1.0);

  /** 向底盘发布带周期刷新的 AUTO 标志；底盘只接受新鲜标志，避免导航结束后残留。 */
  void publishAutoModeState(bool active);

  /**
   * @brief 向底盘发布带周期刷新的 TANGENT 标志
   *
   * 底盘据此只为 TANGENT 锁存协调换向目标，START/GOAL/NONE 不会进入该逻辑。
   */
  void publishTangentModeState(bool active);

  /** 只标记 AUTO 是否已进入原有确定性终点段，用于隔离途中舵轮预测。 */
  void publishAutoTerminalState(bool active);

  /**
   * @brief 发布真正的 AUTO 导航任务编号
   *
   * 同一目标的 Theta* 周期重规划保持编号不变；只有 setPlan() 判断为新目标时
   * 才递增。底盘据此为每次 AUTO 重新做一次整段舵轮分支预选，不能再依赖
   * nav_auto_mode 的上升沿，因为连续发送多个 AUTO 目标时该模式会始终为 true。
   */
  void publishAutoTaskId();

  /**
   * @brief 向底盘发布 AUTO 整次任务剩余 yaw
   *
   * 目标 yaw 必须先从全局 plan frame 变换到 controller/costmap frame，再与
   * 同坐标系 robot_pose 相减。底盘只把结果用于任务起步时的舵轮分支预选，
   * 不参与终点控制，也不会修改 TANGENT/START/GOAL。
   */
  void publishAutoYawRemaining(const geometry_msgs::msg::PoseStamped & robot_pose);

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
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>>
  auto_mode_publisher_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>>
  tangent_mode_publisher_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>>
  auto_terminal_publisher_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt64>>
  auto_task_id_publisher_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>>
  auto_yaw_remaining_publisher_;

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
  // 历史参数名保留 tangent_ 前缀；当前供 TANGENT/START 共用短距位置控制。
  double tangent_terminal_position_kp_{0.8};
  double tangent_terminal_position_max_speed_{0.20};
  // 与 Theta* 的 tangent 终点 yaw 保持距离一致；区内期望角速度为零。
  double tangent_terminal_yaw_hold_distance_{0.20};
  // AUTO 边走边转时的舵轮可执行性约束；不降低底盘最终安全保护阈值。
  double auto_chassis_length_{0.460};
  double auto_chassis_width_{0.345};
  // AUTO 角速度与线速度彻底解耦；最大角速度和角加速度独立限制。
  double auto_max_angular_speed_{0.45};
  double auto_max_angular_acceleration_{0.7};
  // 角速度目标先低通，再由跨周期角加速度状态施加 jerk 限制，消除 ±固定步长锯齿。
  double auto_angular_speed_filter_alpha_{0.45};
  double auto_max_angular_jerk_{3.0};
  // AUTO 普通巡航只对合速度大小做连续化；方向仍由世界路径前视决定。
  double auto_linear_speed_filter_alpha_{0.35};
  double auto_max_linear_acceleration_{0.6};
  double auto_max_linear_deceleration_{0.8};
  double auto_max_wheel_steer_step_{0.08};
  double auto_steering_filter_simulate_ahead_time_{0.5};
  // 从已变换路径上选取前视点，跳过 Theta* 起点接入产生的厘米级短线段。
  double auto_path_direction_lookahead_distance_{0.70};
  double auto_path_direction_command_deadband_{0.005};
  // 同目标重规划时限制世界路径方向变化；超过紧急角阈值则立即采用新规划方向。
  double auto_path_direction_max_rate_{0.8};
  double auto_path_direction_emergency_reset_angle_{1.047};
  double auto_terminal_yaw_kp_{1.2};
  // 路径足够长时，在终点前该距离完成主要 yaw，避免把旋转堆到最后几厘米。
  double auto_yaw_completion_distance_before_goal_{0.20};
  bool auto_command_filter_initialized_{false};
  geometry_msgs::msg::Twist last_auto_filtered_command_;
  double last_auto_filter_robot_yaw_{0.0};
  double last_auto_filtered_wz_acceleration_{0.0};
  // 方向历史只在最终命令通过碰撞检查后提交；同目标重规划不会清零。
  bool auto_path_world_yaw_initialized_{false};
  double last_auto_path_world_yaw_{0.0};
  bool auto_path_world_yaw_pending_{false};
  double pending_auto_path_world_yaw_{0.0};
  // AUTO 的 yaw 参考属于整次导航任务；同目标重规划不得重置以下锁存量。
  bool auto_yaw_task_initialized_{false};
  double auto_task_start_yaw_{0.0};
  double auto_task_goal_yaw_{0.0};
  double auto_task_initial_path_length_{0.0};
  double auto_task_progress_{0.0};
  // 从 1 开始单调递增；只表示真正的新 AUTO 目标，不随同目标重规划变化。
  std::uint64_t auto_task_id_{0};
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
  // TANGENT 起点对齐后的巡航接入：底盘零速对舵期间锁住首个有效 MPPI 轮向，
  // 车辆真实移动后才逐渐放开新命令，避免等待阶段追逐每次采样变化。
  double tangent_track_entry_blend_duration_{0.8};
  double tangent_track_entry_motion_threshold_{0.03};
  double tangent_track_entry_command_deadband_{0.01};
  bool tangent_track_entry_active_{false};
  bool tangent_track_entry_reference_initialized_{false};
  geometry_msgs::msg::Twist tangent_track_entry_reference_;
  double tangent_track_entry_moving_time_{0.0};
  rclcpp::Time tangent_track_entry_last_time_;
  // 起点旋转限加速度必须基于上一周期命令，而不是可能仍为零的里程计速度。
  double last_start_rotation_command_wz_{0.0};
  bool start_rotation_command_initialized_{false};

  // ⭐ costmap 异步解耦：控制线程读快照副本，try_lock 刷新，永不阻塞等 costmap 更新锁
  nav2_costmap_2d::Costmap2D costmap_snapshot_;
  bool costmap_snapshot_initialized_{false};
};

}  // namespace nav2_mppi_controller

#endif  // NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_
