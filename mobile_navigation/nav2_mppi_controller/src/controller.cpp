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

#include <stdint.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include "nav2_mppi_controller/controller.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2/utils.h"  // tf2::getYaw（到点精度 log）

// #define BENCHMARK_TESTING

namespace nav2_mppi_controller
{
namespace
{

double shortestAngularDistance(double from, double to)
{
  const double delta = to - from;
  return std::atan2(std::sin(delta), std::cos(delta));
}

double poseDistance(const geometry_msgs::msg::Pose & lhs, const geometry_msgs::msg::Pose & rhs)
{
  return std::hypot(
    lhs.position.x - rhs.position.x,
    lhs.position.y - rhs.position.y);
}

}  // namespace

void MPPIController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  name_ = name;
  parameters_handler_ = std::make_unique<ParametersHandler>(parent);

  auto node = parent_.lock();
  clock_ = node->get_clock();
  last_time_called_ = clock_->now();
  // Get high-level controller parameters
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(visualize_, "visualize", false);
  getParam(reset_period_, "reset_period", 1.0);
  active_stage_ = STAGE_TRACK;
  configureStages();
  active_stage_enter_time_ = clock_->now();

  // 起点停车旋转用于带 marker 的 TANGENT/GOAL。参数名保留 tangent_ 前缀兼容
  // 已部署配置；START/AUTO/NONE 保持各自途中 yaw 语义，不在起点强制转身。
  getParam(tangent_phase_control_enabled_, "tangent_phase_control_enabled", false);
  getParam(tangent_rotate_kp_, "tangent_rotate_kp", 1.5);
  getParam(tangent_rotate_max_speed_, "tangent_rotate_max_speed", 0.5);
  getParam(tangent_rotate_min_speed_, "tangent_rotate_min_speed", 0.08);
  getParam(tangent_rotate_max_accel_, "tangent_rotate_max_accel", 0.7);
  getParam(tangent_start_yaw_tolerance_, "tangent_start_yaw_tolerance", 0.03);
  getParam(tangent_goal_yaw_tolerance_, "tangent_goal_yaw_tolerance", 0.03);
  getParam(tangent_goal_xy_tolerance_, "tangent_goal_xy_tolerance", 0.05);
  getParam(
    tangent_stopped_linear_velocity_, "tangent_stopped_linear_velocity", 0.03);
  getParam(
    tangent_stopped_angular_velocity_, "tangent_stopped_angular_velocity", 0.03);
  getParam(
    tangent_marker_position_tolerance_, "tangent_marker_position_tolerance", 1e-4);
  getParam(
    start_yaw_lookahead_distance_, "start_yaw_lookahead_distance", 0.30);
  start_yaw_lookahead_distance_ = std::max(
    start_yaw_lookahead_distance_, tangent_marker_position_tolerance_);
  getParam(
    tangent_rotation_simulate_ahead_time_,
    "tangent_rotation_simulate_ahead_time", 0.5);

  double controller_frequency = 10.0;
  node->get_parameter("controller_frequency", controller_frequency);
  control_duration_ = 1.0 / std::max(controller_frequency, 1.0);
  tangent_command_frame_ = costmap_ros_->getBaseFrameID();
  tangent_collision_checker_ = std::make_unique<
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(
    costmap_ros_->getCostmap());

  // Configure composed objects
  optimizer_.initialize(parent_, name_, costmap_ros_, parameters_handler_.get());
  applied_profile_ = stages_[active_stage_];
  optimizer_.applyProfile(applied_profile_);
  parameters_handler_->addPostCallback(
    [this]() {
      optimizer_.applyProfile(applied_profile_);
    });
  path_handler_.initialize(parent_, name_, costmap_ros_, tf_buffer_, parameters_handler_.get());
  trajectory_visualizer_.on_configure(
    parent_, name_,
    costmap_ros_->getGlobalFrameID(), parameters_handler_.get());
  profile_debug_publisher_ =
    node->create_publisher<std_msgs::msg::Float64MultiArray>("/mppi/profile_debug", 10);

  RCLCPP_INFO(
    logger_,
    "Configured MPPI Controller: %s，tangent/goal 起点对齐=%s，之后运行 TRACK/APPROACH",
    name_.c_str(), tangent_phase_control_enabled_ ? "启用" : "关闭");
}

void MPPIController::cleanup()
{
  optimizer_.shutdown();
  trajectory_visualizer_.on_cleanup();
  tangent_collision_checker_.reset();
  tangent_phase_ = TangentPhase::DISABLED;
  current_path_control_mode_ = PathControlMode::AUTO;
  navigation_goal_initialized_ = false;
  start_yaw_target_pending_ = false;
  profile_debug_publisher_.reset();
  parameters_handler_.reset();
  RCLCPP_INFO(logger_, "Cleaned up MPPI Controller: %s", name_.c_str());
}

void MPPIController::activate()
{
  trajectory_visualizer_.on_activate();
  if (profile_debug_publisher_) {
    profile_debug_publisher_->on_activate();
  }
  parameters_handler_->start();
  RCLCPP_INFO(logger_, "Activated MPPI Controller: %s", name_.c_str());
}

void MPPIController::deactivate()
{
  if (profile_debug_publisher_) {
    profile_debug_publisher_->on_deactivate();
  }
  trajectory_visualizer_.on_deactivate();
  RCLCPP_INFO(logger_, "Deactivated MPPI Controller: %s", name_.c_str());
}

void MPPIController::reset()
{
  optimizer_.reset();
  approach_substage_ = APPROACH_POSITIONING;
  approach_mode_ = APPROACH_NORMAL;
  approach_mode_locked_ = false;
  active_stage_enter_time_ = clock_->now();
  final_yaw_cmd_filter_active_ = false;
  final_yaw_direction_lock_active_ = false;
  final_yaw_rotation_started_ = false;
  final_yaw_locked_direction_ = 0.0f;
  last_final_yaw_cmd_wz_ = 0.0f;
  last_start_rotation_command_wz_ = 0.0;
  start_rotation_command_initialized_ = false;
}

// geometry_msgs::msg::TwistStamped MPPIController::computeVelocityCommands(
//   const geometry_msgs::msg::PoseStamped & robot_pose,
//   const geometry_msgs::msg::Twist & robot_speed,
//   nav2_core::GoalChecker * goal_checker)
// {
// #ifdef BENCHMARK_TESTING
//   auto start = std::chrono::system_clock::now();
// #endif

//   if (clock_->now() - last_time_called_ > rclcpp::Duration::from_seconds(reset_period_)) {
//     reset();
//   }
//   last_time_called_ = clock_->now();

//   std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
//   nav_msgs::msg::Path transformed_plan = path_handler_.transformPath(robot_pose);

//   nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
//   std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));
//   auto start_mppi = std::chrono::system_clock::now();
//   geometry_msgs::msg::TwistStamped cmd =
//     optimizer_.evalControl(robot_pose, robot_speed, transformed_plan, goal_checker);
//   auto end_mppi = std::chrono::system_clock::now();
//   auto duration_mppi =
//     std::chrono::duration_cast<std::chrono::milliseconds>(end_mppi - start_mppi).count();
//   RCLCPP_INFO(logger_, "Control evalControl time: %ld [ms]", duration_mppi);
// #ifdef BENCHMARK_TESTING
//   auto end = std::chrono::system_clock::now();
//   auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
//   RCLCPP_INFO(logger_, "Control loop execution time: %ld [ms]", duration);
// #endif

//   if (visualize_) {
//     visualize(std::move(transformed_plan));
//   }

//   return cmd;
// }
geometry_msgs::msg::TwistStamped MPPIController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * goal_checker)
{
  constexpr double kSlowTimingWarnMs = 100.0;  // 100.0
  auto t_total_start = std::chrono::high_resolution_clock::now();

  if (clock_->now() - last_time_called_ > rclcpp::Duration::from_seconds(reset_period_)) {
    reset();
  }
  last_time_called_ = clock_->now();

  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());

  auto t_transform_start = std::chrono::high_resolution_clock::now();
  nav_msgs::msg::Path transformed_plan = path_handler_.transformPath(robot_pose);

  // TANGENT 需要出发切线 yaw，GOAL 需要用户 goal yaw；两者都必须从已经变换到
  // controller/costmap 坐标系的路径提取。START/AUTO/NONE 的 pending 始终为 false。
  if (start_yaw_target_pending_) {
    if (!configureStartPoseInControllerFrame(transformed_plan)) {
      throw nav2_core::PlannerException(
              "tangent 起点对齐无法从 controller 坐标系路径提取 yaw");
    }
    start_yaw_target_pending_ = false;
    RCLCPP_INFO(
      logger_,
      "新导航目标起点 yaw 已锁存：mode=%s frame=%s yaw=%.3f rad，tangent 前视=%.3f m",
      pathControlModeName(current_path_control_mode_),
      transformed_plan.header.frame_id.c_str(),
      tf2::getYaw(tangent_cruise_start_pose_.orientation),
      start_yaw_lookahead_distance_);
  }

  // tangent/goal 起点停车旋转阶段直接接管；其他模式或完成后继续 TRACK/APPROACH。
  if (auto tangent_command = computeTangentStartCommand(robot_pose, robot_speed)) {
    return *tangent_command;
  }

  const double goal_distance = path_handler_.getLastGoalDistance();
  const double path_progress = path_handler_.getLastPathProgress();

  // 提前捕获目标位姿（用于 approach 子阶段和到点精度 log；plan 后续可能被 move 走）
  double key_goal_x = 0.0, key_goal_y = 0.0, key_goal_yaw = 0.0;
  double key_lon = 0.0, key_lat = 0.0, key_yaw = 0.0;
  const bool key_has_goal = !transformed_plan.poses.empty();
  if (key_has_goal) {
    const auto & gp = transformed_plan.poses.back().pose;
    key_goal_x = gp.position.x;
    key_goal_y = gp.position.y;
    key_goal_yaw = tf2::getYaw(gp.orientation);
    const double dx = robot_pose.pose.position.x - key_goal_x;
    const double dy = robot_pose.pose.position.y - key_goal_y;
    const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    key_lon = std::cos(key_goal_yaw) * dx + std::sin(key_goal_yaw) * dy;
    key_lat = -std::sin(key_goal_yaw) * dx + std::cos(key_goal_yaw) * dy;
    const double yaw_error = robot_yaw - key_goal_yaw;
    key_yaw = std::atan2(std::sin(yaw_error), std::cos(yaw_error));
  }
  // 终点控制只保留一个硬门槛：纵向、横向误差都进入 2 cm 后，停止平移并
  // 单独处理最终 yaw。门槛外始终使用同一套 TRACK MPPI，不再在 1 m 处切换
  // APPROACH profile，也不让目标 yaw critic 提前与位置控制争夺控制权。
  // 进入 final_yaw 使用更严格的 enter 容差；已经开始旋转后允许位置在硬性
  // 2 cm exit 容差内轻微波动，避免定位噪声让 positioning/final_yaw 来回切换。
  // exit 仍不超过 2 cm，因此不会放宽最终到点标准。
  const bool final_yaw_was_active =
    active_stage_ == STAGE_APPROACH &&
    approach_substage_ == APPROACH_FINAL_YAW;
  const double final_lon_tolerance = final_yaw_was_active ?
    final_yaw_exit_lon_tolerance_ : final_yaw_enter_lon_tolerance_;
  const double final_lat_tolerance = final_yaw_was_active ?
    final_yaw_exit_lat_tolerance_ : final_yaw_enter_lat_tolerance_;
  const bool position_within_final_tolerance =
    key_has_goal &&
    std::abs(key_lon) <= final_lon_tolerance &&
    std::abs(key_lat) <= final_lat_tolerance;
  if (position_within_final_tolerance) {
    if (
      active_stage_ != STAGE_APPROACH ||
      approach_substage_ != APPROACH_FINAL_YAW)
    {
      active_stage_ = STAGE_APPROACH;
      approach_substage_ = APPROACH_FINAL_YAW;
      profile_transition_active_ = false;
      final_yaw_rotation_started_ = false;
      last_start_rotation_command_wz_ = 0.0;
      start_rotation_command_initialized_ = false;
      RCLCPP_INFO(
        logger_,
        "终点位置已进入严格容差，切换到独立 final_yaw：|lon|=%.3f m |lat|=%.3f m",
        std::abs(key_lon), std::abs(key_lat));
    }

    auto command = computeFinalGoalYawCommand(robot_pose, robot_speed, key_yaw);
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 1000,
      "[MPPI] stage=final_yaw mode=%s err[lon=%.3f lat=%.3f yaw=%.3f] "
      "cmd[vx=%.2f vy=%.2f wz=%.2f] act[vx=%.2f vy=%.2f wz=%.2f]",
      pathControlModeName(current_path_control_mode_), key_lon, key_lat, key_yaw,
      command.twist.linear.x, command.twist.linear.y, command.twist.angular.z,
      robot_speed.linear.x, robot_speed.linear.y, robot_speed.angular.z);
    return command;
  }

  if (active_stage_ == STAGE_APPROACH) {
    // 旋转过程中若任一位置轴漂出 2 cm，立即恢复同一套位置 MPPI。
    RCLCPP_INFO(
      logger_,
      "final_yaw 期间位置漂出容差，恢复连续位置靠近：|lon|=%.3f m |lat|=%.3f m",
      std::abs(key_lon), std::abs(key_lat));
    final_yaw_rotation_started_ = false;
    start_rotation_command_initialized_ = false;
  }
  active_stage_ = STAGE_TRACK;
  approach_substage_ = APPROACH_POSITIONING;
  applyProfileForPathState(
    path_progress, goal_distance, key_lon, key_lat, key_yaw);
  auto t_transform_end = std::chrono::high_resolution_clock::now();
  double transform_ms =
    std::chrono::duration<double, std::milli>(t_transform_end - t_transform_start).count();

  nav2_costmap_2d::Costmap2D * live_costmap = costmap_ros_->getCostmap();

  // ⭐ costmap 异步解耦：try_lock 拷快照，拿不到锁就复用上一帧快照，控制线程永不阻塞等更新
  auto t_lock_start = std::chrono::high_resolution_clock::now();
  auto * costmap_mutex = live_costmap->getMutex();
  if (!costmap_snapshot_initialized_) {
    // 首帧必须有有效快照：阻塞拷一次
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lk(*costmap_mutex);
    costmap_snapshot_ = *live_costmap;
    costmap_snapshot_initialized_ = true;
  } else if (costmap_mutex->try_lock()) {
    costmap_snapshot_ = *live_costmap;   // 拿到锁就刷新快照（亚毫秒深拷）
    costmap_mutex->unlock();
  }  // 拿不到锁（costmap 更新进行中）：复用上一帧快照，不阻塞
  auto t_lock_end = std::chrono::high_resolution_clock::now();
  double wait_lock_ms =
    std::chrono::duration<double, std::milli>(t_lock_end - t_lock_start).count();

  // critic 评分指向快照，evalControl 全程不持 live costmap 锁
  optimizer_.setCostmapSnapshot(&costmap_snapshot_);

  auto t_optimize_start = std::chrono::high_resolution_clock::now();
  geometry_msgs::msg::TwistStamped cmd =
    optimizer_.evalControl(robot_pose, robot_speed, transformed_plan, goal_checker);
  enforceTerminalExecutableTranslation(
    cmd, robot_pose, key_goal_x, key_goal_y, goal_distance);
  enforceFixedCruiseYaw(cmd);
  publishProfileDebug(path_progress, goal_distance);
  auto t_optimize_end = std::chrono::high_resolution_clock::now();
  double optimize_ms =
    std::chrono::duration<double, std::milli>(t_optimize_end - t_optimize_start).count();

  double visualize_ms = 0.0;
  if (visualize_) {
    auto t_visualize_start = std::chrono::high_resolution_clock::now();
    visualize(std::move(transformed_plan));
    auto t_visualize_end = std::chrono::high_resolution_clock::now();
    visualize_ms =
      std::chrono::duration<double, std::milli>(t_visualize_end - t_visualize_start).count();
  }

  auto t_total_end = std::chrono::high_resolution_clock::now();
  double total_ms =
    std::chrono::duration<double, std::milli>(t_total_end - t_total_start).count();

  if (
    total_ms >= kSlowTimingWarnMs || transform_ms >= kSlowTimingWarnMs ||
    wait_lock_ms >= kSlowTimingWarnMs || optimize_ms >= kSlowTimingWarnMs ||
    visualize_ms >= kSlowTimingWarnMs)
  {
    RCLCPP_WARN(
      logger_,
      "MPPI timing (ms): transform=%.3f wait_costmap_lock=%.3f optimize=%.3f visualize=%.3f "
      "total=%.3f. Control cycle too long; use the transform/lock-wait/optimize/visualize fields to locate the bottleneck.",
      transform_ms, wait_lock_ms, optimize_ms, visualize_ms, total_ms);
  }

  // ── 关键运行 log（节流 1Hz）：阶段 / 到目标距离 / 到点精度(lon/lat/yaw,目标系) / 本周期耗时 / 命令 vs 实际 ──
  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 1000,
    "[MPPI] stage=%s mode=%s substage=%s goal_dist=%.3fm "
    "err[lon=%.3f lat=%.3f yaw=%.3f] "
    "cycle=%.1fms(opt=%.1f) cmd[vx=%.2f vy=%.2f wz=%.2f] "
    "act[vx=%.2f vy=%.2f wz=%.2f]",
    stageName(active_stage_).c_str(), approachModeName().c_str(),
    approachSubstageName().c_str(), goal_distance,
    key_lon, key_lat, key_yaw, total_ms, optimize_ms,
    cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.angular.z,
    robot_speed.linear.x, robot_speed.linear.y, robot_speed.angular.z);

  return cmd;
}

void MPPIController::publishProfileDebug(double path_progress, double goal_distance)
{
  if (!profile_debug_publisher_ || !profile_debug_publisher_->is_activated()) {
    return;
  }

  const auto & applied_velocity = applied_profile_.velocity;
  const auto & active_constraints = optimizer_.getActiveConstraints();
  auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
  msg->data = {
    static_cast<double>(active_stage_ + 1),
    path_progress,
    goal_distance,
    profile_transition_active_ ? 1.0 : 0.0,
    static_cast<double>(applied_velocity.vx_min),
    static_cast<double>(applied_velocity.vx_max),
    static_cast<double>(applied_velocity.vy_max),
    static_cast<double>(applied_velocity.wz_max),
    static_cast<double>(active_constraints.vx_min),
    static_cast<double>(active_constraints.vx_max),
    static_cast<double>(active_constraints.vy),
    static_cast<double>(active_constraints.wz),
    optimizer_.isSpeedLimitActive() ? 1.0 : 0.0,
    optimizer_.getSpeedLimit(),
    optimizer_.isSpeedLimitPercentage() ? 1.0 : 0.0,
    profile_start_distance_,
    approach_distance_,
    profile_transition_duration_,
    approach_hysteresis_,
    active_stage_ == STAGE_APPROACH ? static_cast<double>(approach_substage_ + 1) : 0.0,
    final_yaw_enter_lon_tolerance_,
    final_yaw_enter_lat_tolerance_,
    final_yaw_exit_lon_tolerance_,
    final_yaw_exit_lat_tolerance_,
    position_first_yaw_threshold_,
    active_stage_ == STAGE_APPROACH ? static_cast<double>(approach_mode_ + 1) : 0.0,
  };
  profile_debug_publisher_->publish(std::move(msg));
}

void MPPIController::configureStages()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  // 三段速度先统一用全局默认（不再 per-stage 配置）；需差异化时在 yaml 加
  // stage_align/track/approach 的 vx_max/vx_min/vy_max/wz_max 覆盖即可。
  float g_vx_max = 0.7f, g_vx_min = -0.5f, g_vy_max = 0.5f, g_wz_max = 0.75f;
  float g_vx_std = 0.2f, g_vy_std = 0.2f, g_wz_std = 0.4f;
  getParam(g_vx_max, "vx_max", 0.7, ParameterType::Static);
  getParam(g_vx_min, "vx_min", -0.5, ParameterType::Static);
  getParam(g_vy_max, "vy_max", 0.5, ParameterType::Static);
  getParam(g_wz_max, "wz_max", 0.75, ParameterType::Static);
  getParam(g_vx_std, "vx_std", 0.2, ParameterType::Static);
  getParam(g_vy_std, "vy_std", 0.2, ParameterType::Static);
  getParam(g_wz_std, "wz_std", 0.4, ParameterType::Static);
  bool global_path_align_use_orientations = false;
  bool global_path_align_ignore_goal_orientation = true;
  getParam(
    global_path_align_use_orientations, "PathAlignCritic.use_path_orientations",
    false, ParameterType::Static);
  getParam(
    global_path_align_ignore_goal_orientation,
    "PathAlignCritic.ignore_goal_orientation", true, ParameterType::Static);
  for (auto & stage : stages_) {
    stage.velocity = {g_vx_max, g_vx_min, g_vy_max, g_wz_max};
    stage.sampling_std = {g_vx_std, g_vy_std, g_wz_std};
  }

  // 运行时恢复为单套 TRACK MPPI。ALIGN/APPROACH 数组项仅保留参数 ABI 兼容，
  // 不再参与终点位置控制；最终 yaw 由严格 2 cm 门控后的独立旋转命令完成。
  stages_[0].critics.goal = {0.0f};
  stages_[0].critics.goal_lateral = {0.0f};
  stages_[0].critics.goal_longitudinal = {0.0f};
  stages_[0].critics.goal_angle = {0.0f};
  stages_[0].critics.path_angle = {7.0f, 3, 0.7f, true};
  stages_[0].critics.path_align = {
    8.0f, 8, 3, 0.08f, global_path_align_use_orientations,
    global_path_align_ignore_goal_orientation};
  stages_[0].critics.path_follow = {7.0f, 7};
  stages_[0].critics.prefer_forward = {6.0f};
  stages_[0].critics.linear_velocity = {0.0f};   // 对齐段关闭速度软约束（不惩罚后退机动）
  stages_[0].critics.obstacles = {1.1f};
  stages_[0].critics.cost = {3.81f};
  stages_[0].critics.angular_velocity = {0.0f};

  stages_[1].critics.goal = {8.0f};
  stages_[1].critics.goal_lateral = {0.0f};
  stages_[1].critics.goal_longitudinal = {0.0f};
  stages_[1].critics.goal_angle = {0.0f};
  stages_[1].critics.path_angle = {0.0f, 3, 0.9f, true};
  stages_[1].critics.path_align = {
    14.0f, 1, 3, 0.05f, global_path_align_use_orientations,
    global_path_align_ignore_goal_orientation};
  stages_[1].critics.path_follow = {5.0f, 5};
  stages_[1].critics.prefer_forward = {0.0f};
  stages_[1].critics.linear_velocity = {0.0f};
  stages_[1].critics.obstacles = {1.1f};
  stages_[1].critics.cost = {3.81f};
  stages_[1].critics.angular_velocity = {0.0f};

  stages_[2].critics.goal = {4.0f};
  stages_[2].critics.goal_lateral = {40.0f};
  stages_[2].critics.goal_longitudinal = {25.0f};
  stages_[2].critics.goal_angle = {8.0f};
  stages_[2].critics.path_angle = {1.0f, 2, 1.0f, true};
  // 终点定位阶段只贴合路径位置；最终朝向必须等 lon/lat 都进入硬容差后再处理。
  stages_[2].critics.path_align = {
    2.0f, 15, 3, 0.05f, false, global_path_align_ignore_goal_orientation};
  stages_[2].critics.path_follow = {1.0f, 5};
  stages_[2].critics.prefer_forward = {1.0f};
  stages_[2].critics.linear_velocity = {8.0f};
  stages_[2].critics.obstacles = {0.0f};   // 趋近段软斥力关（保致命）
  stages_[2].critics.cost = {0.0f};
  stages_[2].critics.angular_velocity = {15.0f};

  // profile_start_distance 已废弃（对齐段改按角度切，不再按距起点距离）；成员保留仅供 debug 兼容
  getParam(approach_distance_, "approach_distance", 1.0, ParameterType::Static);
  getParam(
    approach_braking_deceleration_, "approach_braking_deceleration",
    0.5, ParameterType::Static);
  getParam(
    terminal_min_translation_speed_, "terminal_min_translation_speed",
    0.05, ParameterType::Static);
  getParam(
    terminal_translation_assist_distance_, "terminal_translation_assist_distance",
    0.30, ParameterType::Static);
  getParam(
    terminal_translation_command_deadband_, "terminal_translation_command_deadband",
    0.005, ParameterType::Static);
  getParam(
    terminal_translation_min_goal_alignment_, "terminal_translation_min_goal_alignment",
    0.5, ParameterType::Static);
  getParam(profile_transition_duration_, "profile_transition_duration", 1.0, ParameterType::Static);
  getParam(approach_hysteresis_, "approach_hysteresis", 0.05, ParameterType::Static);
  approach_hysteresis_ = std::max(0.0, approach_hysteresis_);
  approach_braking_deceleration_ = std::max(0.0, approach_braking_deceleration_);
  terminal_min_translation_speed_ = std::max(0.0, terminal_min_translation_speed_);
  terminal_translation_assist_distance_ = std::max(
    0.0, terminal_translation_assist_distance_);
  terminal_translation_command_deadband_ = std::max(
    0.0, terminal_translation_command_deadband_);
  terminal_translation_min_goal_alignment_ = std::clamp(
    terminal_translation_min_goal_alignment_, -1.0, 1.0);
  getParam(
    final_yaw_enter_lon_tolerance_, "approach_final_yaw_enter_lon_tolerance",
    0.015, ParameterType::Static);
  getParam(
    final_yaw_enter_lat_tolerance_, "approach_final_yaw_enter_lat_tolerance",
    0.015, ParameterType::Static);
  getParam(
    final_yaw_exit_lon_tolerance_, "approach_final_yaw_exit_lon_tolerance",
    0.02, ParameterType::Static);
  getParam(
    final_yaw_exit_lat_tolerance_, "approach_final_yaw_exit_lat_tolerance",
    0.02, ParameterType::Static);
  getParam(
    position_first_yaw_threshold_, "approach_position_first_yaw_threshold",
    2.094, ParameterType::Static);
  getParam(
    position_first_lateral_error_threshold_,
    "approach_position_first_lateral_error_threshold", 0.05, ParameterType::Static);
  getParam(
    position_first_lateral_ratio_,
    "approach_position_first_lateral_ratio", 1.0, ParameterType::Static);
  getParam(
    positioning_goal_angle_weight_, "approach_positioning_goal_angle_weight",
    0.0, ParameterType::Static);
  getParam(
    final_yaw_vx_max_, "approach_final_yaw_vx_max", 0.03, ParameterType::Static);
  getParam(
    final_yaw_vy_max_, "approach_final_yaw_vy_max", 0.03, ParameterType::Static);
  getParam(
    final_yaw_wz_max_, "approach_final_yaw_wz_max", 0.4, ParameterType::Static);
  getParam(
    final_yaw_lock_translation_, "approach_final_yaw_lock_translation",
    true, ParameterType::Static);
  getParam(
    final_yaw_cmd_wz_slew_rate_, "approach_final_yaw_cmd_wz_slew_rate",
    1.2, ParameterType::Static);
  getParam(
    final_yaw_cmd_brake_accel_, "approach_final_yaw_cmd_brake_accel",
    0.7, ParameterType::Static);
  getParam(
    final_yaw_cmd_stop_yaw_tolerance_, "approach_final_yaw_cmd_stop_yaw_tolerance",
    0.0, ParameterType::Static);
  getParam(
    final_yaw_cmd_direction_deadband_, "approach_final_yaw_cmd_direction_deadband",
    0.05, ParameterType::Static);
  final_yaw_enter_lon_tolerance_ = std::max(0.0, final_yaw_enter_lon_tolerance_);
  final_yaw_enter_lat_tolerance_ = std::max(0.0, final_yaw_enter_lat_tolerance_);
  final_yaw_exit_lon_tolerance_ = std::max(
    final_yaw_enter_lon_tolerance_, final_yaw_exit_lon_tolerance_);
  final_yaw_exit_lat_tolerance_ = std::max(
    final_yaw_enter_lat_tolerance_, final_yaw_exit_lat_tolerance_);
  position_first_yaw_threshold_ = std::clamp(position_first_yaw_threshold_, 0.0, M_PI);
  position_first_lateral_error_threshold_ =
    std::max(0.0, position_first_lateral_error_threshold_);
  position_first_lateral_ratio_ = std::max(0.0, position_first_lateral_ratio_);
  positioning_goal_angle_weight_ = std::max(0.0f, positioning_goal_angle_weight_);
  final_yaw_vx_max_ = std::max(0.0f, final_yaw_vx_max_);
  final_yaw_vy_max_ = std::max(0.0f, final_yaw_vy_max_);
  final_yaw_wz_max_ = std::max(0.0f, final_yaw_wz_max_);
  final_yaw_cmd_wz_slew_rate_ = std::max(0.0f, final_yaw_cmd_wz_slew_rate_);
  final_yaw_cmd_brake_accel_ = std::max(0.0f, final_yaw_cmd_brake_accel_);
  final_yaw_cmd_stop_yaw_tolerance_ = std::max(0.0f, final_yaw_cmd_stop_yaw_tolerance_);
  final_yaw_cmd_direction_deadband_ = std::max(0.0f, final_yaw_cmd_direction_deadband_);

  for (size_t i = 0; i < stages_.size(); ++i) {
    auto & profile = stages_[i];
    auto & profile_critics = profile.critics;
    static const char * kStageKeys[3] = {"stage_align", "stage_track", "stage_approach"};
    const std::string prefix = std::string(kStageKeys[i]) + ".";

    getParam(
      profile.velocity.vx_max, prefix + "vx_max",
      profile.velocity.vx_max, ParameterType::Static);
    getParam(
      profile.velocity.vx_min, prefix + "vx_min",
      profile.velocity.vx_min, ParameterType::Static);
    getParam(
      profile.velocity.vy_max, prefix + "vy_max",
      profile.velocity.vy_max, ParameterType::Static);
    getParam(
      profile.velocity.wz_max, prefix + "wz_max",
      profile.velocity.wz_max, ParameterType::Static);
    getParam(
      profile.sampling_std.vx, prefix + "vx_std",
      profile.sampling_std.vx, ParameterType::Static);
    getParam(
      profile.sampling_std.vy, prefix + "vy_std",
      profile.sampling_std.vy, ParameterType::Static);
    getParam(
      profile.sampling_std.wz, prefix + "wz_std",
      profile.sampling_std.wz, ParameterType::Static);

    getParam(
      profile_critics.goal.weight, prefix + "GoalCritic.cost_weight",
      profile_critics.goal.weight, ParameterType::Static);
    getParam(
      profile_critics.goal_lateral.weight, prefix + "GoalLateralCritic.cost_weight",
      profile_critics.goal_lateral.weight, ParameterType::Static);
    getParam(
      profile_critics.goal_longitudinal.weight,
      prefix + "GoalLongitudinalCritic.cost_weight",
      profile_critics.goal_longitudinal.weight, ParameterType::Static);
    getParam(
      profile_critics.goal_angle.weight, prefix + "GoalAngleCritic.cost_weight",
      profile_critics.goal_angle.weight, ParameterType::Static);
    getParam(
      profile_critics.path_angle.weight, prefix + "PathAngleCritic.cost_weight",
      profile_critics.path_angle.weight, ParameterType::Static);
    getParam(
      profile_critics.path_angle.offset_from_furthest,
      prefix + "PathAngleCritic.offset_from_furthest",
      profile_critics.path_angle.offset_from_furthest, ParameterType::Static);
    getParam(
      profile_critics.path_angle.max_angle_to_furthest,
      prefix + "PathAngleCritic.max_angle_to_furthest",
      profile_critics.path_angle.max_angle_to_furthest, ParameterType::Static);
    getParam(
      profile_critics.path_angle.forward_preference,
      prefix + "PathAngleCritic.forward_preference",
      profile_critics.path_angle.forward_preference, ParameterType::Static);
    getParam(
      profile_critics.path_follow.weight, prefix + "PathFollowCritic.cost_weight",
      profile_critics.path_follow.weight, ParameterType::Static);
    getParam(
      profile_critics.path_follow.offset_from_furthest,
      prefix + "PathFollowCritic.offset_from_furthest",
      profile_critics.path_follow.offset_from_furthest, ParameterType::Static);
    getParam(
      profile_critics.path_align.weight, prefix + "PathAlignCritic.cost_weight",
      profile_critics.path_align.weight, ParameterType::Static);
    getParam(
      profile_critics.path_align.offset_from_furthest,
      prefix + "PathAlignCritic.offset_from_furthest",
      profile_critics.path_align.offset_from_furthest, ParameterType::Static);
    getParam(
      profile_critics.path_align.trajectory_point_step,
      prefix + "PathAlignCritic.trajectory_point_step",
      profile_critics.path_align.trajectory_point_step, ParameterType::Static);
    getParam(
      profile_critics.path_align.max_path_occupancy_ratio,
      prefix + "PathAlignCritic.max_path_occupancy_ratio",
      profile_critics.path_align.max_path_occupancy_ratio, ParameterType::Static);
    getParam(
      profile_critics.path_align.use_path_orientations,
      prefix + "PathAlignCritic.use_path_orientations",
      profile_critics.path_align.use_path_orientations, ParameterType::Static);
    getParam(
      profile_critics.path_align.ignore_goal_orientation,
      prefix + "PathAlignCritic.ignore_goal_orientation",
      profile_critics.path_align.ignore_goal_orientation, ParameterType::Static);
    getParam(
      profile_critics.prefer_forward.weight, prefix + "PreferForwardCritic.cost_weight",
      profile_critics.prefer_forward.weight, ParameterType::Static);
    getParam(
      profile_critics.linear_velocity.weight,
      prefix + "LinearVelocityCritic.cost_weight",
      profile_critics.linear_velocity.weight, ParameterType::Static);
    getParam(
      profile_critics.angular_velocity.track_goal_yaw,
      prefix + "AngularVelocityCritic.track_goal_yaw",
      profile_critics.angular_velocity.track_goal_yaw, ParameterType::Static);
  }

  // ── 权重矩阵：stage_weights.<Critic> = [align, track, approach]，各 critic 权重统一调参入口 ──
  // 覆盖各段该 critic 权重；缺省=当前值（矩阵缺失则不变，安全）。非权重 sub-param 仍由 stage_* 块管。
  auto loadWeightRow = [&](const std::string & critic, float & wa, float & wt, float & wp) {
      std::vector<double> def = {wa, wt, wp};
      std::vector<double> row = def;
      getParam(row, "stage_weights." + critic, def, ParameterType::Static);
      if (row.size() == 3) {
        wa = static_cast<float>(row[0]);
        wt = static_cast<float>(row[1]);
        wp = static_cast<float>(row[2]);
      } else {
        RCLCPP_WARN(
          logger_, "stage_weights.%s needs 3 values [align,track,approach]; ignored",
          critic.c_str());
      }
    };
  loadWeightRow(
    "GoalCritic", stages_[0].critics.goal.weight, stages_[1].critics.goal.weight,
    stages_[2].critics.goal.weight);
  loadWeightRow(
    "GoalLateralCritic", stages_[0].critics.goal_lateral.weight,
    stages_[1].critics.goal_lateral.weight, stages_[2].critics.goal_lateral.weight);
  loadWeightRow(
    "GoalLongitudinalCritic", stages_[0].critics.goal_longitudinal.weight,
    stages_[1].critics.goal_longitudinal.weight, stages_[2].critics.goal_longitudinal.weight);
  loadWeightRow(
    "GoalAngleCritic", stages_[0].critics.goal_angle.weight,
    stages_[1].critics.goal_angle.weight, stages_[2].critics.goal_angle.weight);
  loadWeightRow(
    "PathAngleCritic", stages_[0].critics.path_angle.weight,
    stages_[1].critics.path_angle.weight, stages_[2].critics.path_angle.weight);
  loadWeightRow(
    "PathAlignCritic", stages_[0].critics.path_align.weight,
    stages_[1].critics.path_align.weight, stages_[2].critics.path_align.weight);
  loadWeightRow(
    "PathFollowCritic", stages_[0].critics.path_follow.weight,
    stages_[1].critics.path_follow.weight, stages_[2].critics.path_follow.weight);
  loadWeightRow(
    "PreferForwardCritic", stages_[0].critics.prefer_forward.weight,
    stages_[1].critics.prefer_forward.weight, stages_[2].critics.prefer_forward.weight);
  loadWeightRow(
    "LinearVelocityCritic", stages_[0].critics.linear_velocity.weight,
    stages_[1].critics.linear_velocity.weight, stages_[2].critics.linear_velocity.weight);
  loadWeightRow(
    "ObstaclesCritic", stages_[0].critics.obstacles.repulsion_weight,
    stages_[1].critics.obstacles.repulsion_weight, stages_[2].critics.obstacles.repulsion_weight);
  loadWeightRow(
    "CostCritic", stages_[0].critics.cost.weight,
    stages_[1].critics.cost.weight, stages_[2].critics.cost.weight);
  loadWeightRow(
    "AngularVelocityCritic", stages_[0].critics.angular_velocity.weight,
    stages_[1].critics.angular_velocity.weight, stages_[2].critics.angular_velocity.weight);

  for (size_t i = 0; i < stages_.size(); ++i) {
    const auto & profile = stages_[i];
    RCLCPP_INFO(
      logger_,
      "MPPI profile_%zu velocity: vx=[%.3f, %.3f] vy=%.3f wz=%.3f. "
      "Goal(w=%.3f) GoalLateral(w=%.3f) GoalLongitudinal(w=%.3f) "
      "GoalAngle(w=%.3f) PathAngle(w=%.3f off=%d max_angle=%.3f forward=%d) "
      "PathAlign(w=%.3f off=%d step=%d occ=%.3f yaw=%d) PathFollow(w=%.3f off=%d) "
      "PreferForward(w=%.3f). weight=0 disables this cost for the stage.",
      i + 1, profile.velocity.vx_min, profile.velocity.vx_max,
      profile.velocity.vy_max, profile.velocity.wz_max,
      profile.critics.goal.weight,
      profile.critics.goal_lateral.weight,
      profile.critics.goal_longitudinal.weight,
      profile.critics.goal_angle.weight,
      profile.critics.path_angle.weight, profile.critics.path_angle.offset_from_furthest,
      profile.critics.path_angle.max_angle_to_furthest,
      profile.critics.path_angle.forward_preference,
      profile.critics.path_align.weight, profile.critics.path_align.offset_from_furthest,
      profile.critics.path_align.trajectory_point_step,
      profile.critics.path_align.max_path_occupancy_ratio,
      profile.critics.path_align.use_path_orientations,
      profile.critics.path_follow.weight, profile.critics.path_follow.offset_from_furthest,
      profile.critics.prefer_forward.weight);
  }

  applied_profile_ = stages_[active_stage_];
  transition_start_profile_ = applied_profile_;
  transition_target_profile_ = applied_profile_;
}

void MPPIController::applyProfileForPathState(
  double path_progress, double goal_distance,
  double goal_longitudinal_error, double goal_lateral_error, double goal_yaw_error)
{
  (void)path_progress;
  (void)goal_longitudinal_error;
  (void)goal_lateral_error;
  (void)goal_yaw_error;

  // 门槛外每个周期都从同一个 TRACK profile 派生，不保留阶段插值状态。
  // 这样不会在某个固定距离突然更换 critic、采样方差或速度上限。
  active_stage_ = STAGE_TRACK;
  approach_mode_ = APPROACH_NORMAL;
  approach_mode_locked_ = false;
  profile_transition_active_ = false;
  applied_profile_ = stages_[STAGE_TRACK];

  // 最终 goal yaw 在位置进入 2 cm 之前完全关闭。途中 yaw 只来自各模式自己的
  // 路径 orientation；NONE 则连路径 yaw 也不读取。
  applied_profile_.critics.goal_angle.weight = 0.0f;
  applied_profile_.critics.angular_velocity.weight = 0.0f;
  applied_profile_.critics.angular_velocity.track_goal_yaw = false;
  applied_profile_.critics.path_angle.weight = 0.0f;
  applied_profile_.critics.path_align.use_path_orientations =
    current_path_control_mode_ != PathControlMode::NONE &&
    stages_[STAGE_TRACK].critics.path_align.use_path_orientations;

  if (
    current_path_control_mode_ == PathControlMode::START ||
    current_path_control_mode_ == PathControlMode::GOAL)
  {
    // 不仅在发布前把 wz 清零，也要让 MPPI 的预测模型从一开始就只采样纯平移。
    // 否则优化器可能按“预测中会旋转”的轨迹选择 vx/vy，最后再清掉 wz 会造成
    // 预测与实际命令不一致，表现为路径跟踪发飘或一顿一顿。
    applied_profile_.velocity.wz_max = 0.0f;
    applied_profile_.sampling_std.wz = 0.0f;
  }

  // 只连续收紧同一 profile 的平移速度约束，不再切换到低速 APPROACH profile。
  applyApproachBrakingEnvelope(goal_distance);
  applied_profile_.critics.linear_velocity.speed_max = std::max(
    std::abs(applied_profile_.velocity.vx_min),
    std::abs(applied_profile_.velocity.vx_max));
  optimizer_.applyProfile(applied_profile_);
}

void MPPIController::updateApproachMode(
  size_t previous_stage, size_t selected_stage, double goal_longitudinal_error,
  double goal_lateral_error, double goal_yaw_error)
{
  if (selected_stage != STAGE_APPROACH) {
    approach_mode_ = APPROACH_NORMAL;
    approach_mode_locked_ = false;
    return;
  }
  if (previous_stage == STAGE_APPROACH && approach_mode_locked_) {
    return;
  }

  approach_substage_ = APPROACH_POSITIONING;
  const double abs_lon = std::abs(goal_longitudinal_error);
  const double abs_lat = std::abs(goal_lateral_error);
  const bool large_yaw_change =
    std::abs(goal_yaw_error) >= position_first_yaw_threshold_;
  const bool lateral_dominant_goal =
    abs_lat >= position_first_lateral_error_threshold_ &&
    abs_lat >= position_first_lateral_ratio_ * std::max(abs_lon, 1e-6);
  approach_mode_ = large_yaw_change || lateral_dominant_goal ?
    APPROACH_POSITION_FIRST : APPROACH_NORMAL;
  approach_mode_locked_ = true;
  const char * mode_name =
    approach_mode_ == APPROACH_POSITION_FIRST ? "position_first" : "normal";
  RCLCPP_INFO(
    logger_,
    "MPPI approach mode locked: %s, entry |lon|=%.3f m |lat|=%.3f m "
    "|yaw_error|=%.3f rad, yaw_threshold=%.3f rad, lateral_threshold=%.3f m, "
    "lateral_ratio=%.3f, reason=%s. The mode remains locked until leaving approach.",
    mode_name, abs_lon, abs_lat, std::abs(goal_yaw_error), position_first_yaw_threshold_,
    position_first_lateral_error_threshold_, position_first_lateral_ratio_,
    large_yaw_change ? "large_yaw" :
    (lateral_dominant_goal ? "lateral_dominant" : "normal_geometry"));
}

void MPPIController::updateApproachSubstage(
  double goal_longitudinal_error, double goal_lateral_error)
{
  if (active_stage_ != STAGE_APPROACH) {
    approach_substage_ = APPROACH_POSITIONING;
    return;
  }

  const double abs_lon = std::abs(goal_longitudinal_error);
  const double abs_lat = std::abs(goal_lateral_error);
  const size_t previous_substage = approach_substage_;
  if (approach_substage_ == APPROACH_FINAL_YAW) {
    if (abs_lon > final_yaw_exit_lon_tolerance_ ||
      abs_lat > final_yaw_exit_lat_tolerance_)
    {
      approach_substage_ = APPROACH_POSITIONING;
    }
  } else if (
    abs_lon <= final_yaw_enter_lon_tolerance_ &&
    abs_lat <= final_yaw_enter_lat_tolerance_)
  {
    approach_substage_ = APPROACH_FINAL_YAW;
  }

  if (previous_substage != approach_substage_) {
    RCLCPP_INFO(
      logger_, "MPPI approach substage switched: %s -> %s, |lon|=%.3f, |lat|=%.3f",
      previous_substage == APPROACH_FINAL_YAW ? "final_yaw" : "positioning",
      approachSubstageName().c_str(), abs_lon, abs_lat);
  }
}

void MPPIController::applyApproachSubstageProfile()
{
  if (active_stage_ != STAGE_APPROACH) {
    return;
  }

  // APPROACH 全程让 PathAlign 只评价位置。否则 tangent 最后一个路径点的用户
  // goal yaw 会通过 PathAlign 绕过 GoalAngle 门控，导致尚未到 2 cm 就停车转舵。
  applied_profile_.critics.path_align.use_path_orientations = false;

  if (approach_substage_ == APPROACH_POSITIONING) {
    // 定位阶段只负责把目标坐标系的 lon/lat 都压进硬性位置容差。此前 GoalAngle
    // 在距目标几十厘米时已经生效，短路径约束一减弱就会输出“小平移、大角速度”，
    // 进而让底盘误入纯旋转模式。现在无论进入 approach 时的几何类型如何，都把
    // 最终 yaw 延后到 APPROACH_FINAL_YAW；同时保留角速度惩罚，抑制无意义转身。
    applied_profile_.critics.goal_angle.weight = std::min(
      applied_profile_.critics.goal_angle.weight, positioning_goal_angle_weight_);
    applied_profile_.critics.angular_velocity.weight =
      stages_[STAGE_APPROACH].critics.angular_velocity.weight;
    // AngularVelocityCritic 原实现会按最终 yaw 主动生成期望 wz。定位阶段改成纯
    // 角速度阻尼，只把 wz 压向 0；否则即使 GoalAngle=0 仍会提前停车转向。
    applied_profile_.critics.angular_velocity.track_goal_yaw = false;
    return;
  }

  // 最终转向近似原地完成：保留位置类 critic 防漂移，关闭会鼓励平移的软代价。
  applied_profile_.critics.angular_velocity.track_goal_yaw = true;
  applied_profile_.velocity.vx_max =
    std::min(applied_profile_.velocity.vx_max, final_yaw_vx_max_);
  applied_profile_.velocity.vx_min =
    std::max(applied_profile_.velocity.vx_min, -final_yaw_vx_max_);
  applied_profile_.velocity.vy_max =
    std::min(applied_profile_.velocity.vy_max, final_yaw_vy_max_);
  applied_profile_.velocity.wz_max =
    std::min(applied_profile_.velocity.wz_max, final_yaw_wz_max_);
  applied_profile_.critics.path_angle.weight = 0.0f;
  applied_profile_.critics.linear_velocity.weight = 0.0f;
  applied_profile_.critics.prefer_forward.weight = 0.0f;
}

void MPPIController::applyApproachBrakingEnvelope(double goal_distance)
{
  if (!std::isfinite(goal_distance) || approach_braking_deceleration_ <= 0.0) {
    return;
  }

  // 直接对“距离严格位置门槛还剩多少”应用 v²=2as。整个过程中 critic 和采样
  // profile 都不变，因此不会再在 1 m 或其它固定距离突然刹车。门槛外保留一个
  // 很小的速度上限下限，防止数值限幅先降到 0 而机器人停在 2 cm 外。
  const double position_tolerance = std::max(
    final_yaw_enter_lon_tolerance_, final_yaw_enter_lat_tolerance_);
  const double braking_distance = std::max(0.0, goal_distance - position_tolerance);
  const float kinematic_limit = static_cast<float>(std::max(
      terminal_min_translation_speed_,
      std::sqrt(2.0 * approach_braking_deceleration_ * braking_distance)));

  const auto & track = stages_[STAGE_TRACK].velocity;
  const float forward_limit = std::min(std::abs(track.vx_max), kinematic_limit);
  const float reverse_limit = std::min(std::abs(track.vx_min), kinematic_limit);
  const float lateral_limit = std::min(std::abs(track.vy_max), kinematic_limit);

  applied_profile_.velocity.vx_max = std::min(
    applied_profile_.velocity.vx_max, forward_limit);
  applied_profile_.velocity.vx_min = std::max(
    applied_profile_.velocity.vx_min, -reverse_limit);
  applied_profile_.velocity.vy_max = std::min(
    applied_profile_.velocity.vy_max, lateral_limit);
}

geometry_msgs::msg::TwistStamped MPPIController::computeFinalGoalYawCommand(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  double goal_yaw_error)
{
  const double linear_speed = std::hypot(robot_speed.linear.x, robot_speed.linear.y);
  if (!final_yaw_rotation_started_) {
    if (
      linear_speed > tangent_stopped_linear_velocity_ ||
      std::abs(robot_speed.angular.z) > tangent_stopped_angular_velocity_)
    {
      // 只在刚进入 final_yaw 时等待一次停稳，隔离上一条平移/旋转动作。
      // 不能在后续每个周期都检查角速度，否则机器人一旦正常转起来
      // 就会被误判成“未停稳”，形成转动—停止—再转动的脉冲。
      start_rotation_command_initialized_ = false;
      return makeStoppedCommand();
    }

    // 底盘已停稳，锁存本次终点旋转。直到 yaw 达标、位置漂出 2 cm
    // 或导航任务复位前，正常的实测角速度都不再打断旋转。
    final_yaw_rotation_started_ = true;
    start_rotation_command_initialized_ = false;
  }

  const double yaw_tolerance = std::max(
    0.0f, final_yaw_cmd_stop_yaw_tolerance_);
  if (std::abs(goal_yaw_error) <= yaw_tolerance) {
    final_yaw_rotation_started_ = false;
    start_rotation_command_initialized_ = false;
    return makeStoppedCommand();
  }

  // key_yaw 的符号是 current-goal，而 makeRotationCommand 需要 current->goal。
  return makeRotationCommand(
    -goal_yaw_error, robot_pose, robot_speed, yaw_tolerance);
}

void MPPIController::enforceFixedCruiseYaw(geometry_msgs::msg::TwistStamped & cmd) const
{
  if (
    current_path_control_mode_ == PathControlMode::START ||
    current_path_control_mode_ == PathControlMode::GOAL)
  {
    // START：保持接收任务时的当前 yaw 纯平移；GOAL：起点对齐 goal yaw 后纯平移。
    // 两种模式都不允许 MPPI 的 wz 采样噪声把“固定朝向”变成边走边转。
    cmd.twist.angular.z = 0.0;
  }
}

void MPPIController::enforceTerminalExecutableTranslation(
  geometry_msgs::msg::TwistStamped & cmd,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  double goal_x, double goal_y, double goal_distance)
{
  const double position_tolerance = std::max(
    final_yaw_enter_lon_tolerance_, final_yaw_enter_lat_tolerance_);
  if (
    !std::isfinite(goal_distance) ||
    goal_distance <= position_tolerance ||
    goal_distance > terminal_translation_assist_distance_)
  {
    return;
  }

  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  const double dx = goal_x - robot_pose.pose.position.x;
  const double dy = goal_y - robot_pose.pose.position.y;
  // cmd_vel 使用机器人本体坐标系；把目标向量从 controller/costmap 坐标系旋转到
  // 本体坐标系后再判断方向，不能直接拿全局 dx/dy 与 vx/vy 做点积。
  const double goal_body_x = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
  const double goal_body_y = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;

  const double original_vx = cmd.twist.linear.x;
  const double original_vy = cmd.twist.linear.y;
  if (liftTerminalTranslationCommand(
      cmd, goal_body_x, goal_body_y,
      static_cast<float>(terminal_min_translation_speed_),
      static_cast<float>(terminal_translation_command_deadband_),
      static_cast<float>(terminal_translation_min_goal_alignment_),
      optimizer_.getActiveConstraints()))
  {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 1000,
      "终点低速抬升：goal_dist=%.3f m，原命令=(%.3f, %.3f) m/s，"
      "可执行命令=(%.3f, %.3f) m/s",
      goal_distance, original_vx, original_vy,
      cmd.twist.linear.x, cmd.twist.linear.y);
  }
}

bool MPPIController::liftTerminalTranslationCommand(
  geometry_msgs::msg::TwistStamped & cmd,
  double goal_body_x, double goal_body_y,
  float minimum_speed, float command_deadband, float minimum_goal_alignment,
  const models::ControlConstraints & constraints)
{
  const double vx = cmd.twist.linear.x;
  const double vy = cmd.twist.linear.y;
  const double command_speed = std::hypot(vx, vy);
  const double goal_distance = std::hypot(goal_body_x, goal_body_y);
  const double safe_minimum_speed = std::max(0.0f, minimum_speed);
  const double safe_command_deadband = std::max(0.0f, command_deadband);

  // 零命令可能来自碰撞约束或 MPPI 明确停车，不能为了克服静摩擦而强行启动。
  // 已经达到可执行速度时也无需改写。
  if (
    command_speed <= safe_command_deadband ||
    command_speed >= safe_minimum_speed ||
    goal_distance <= 1e-6)
  {
    return false;
  }

  const double goal_alignment =
    (vx * goal_body_x + vy * goal_body_y) / (command_speed * goal_distance);
  if (goal_alignment < std::clamp(
      static_cast<double>(minimum_goal_alignment), -1.0, 1.0))
  {
    // 只放大 MPPI 已经认可且确实朝向目标的方向。横移避障、倒车腾挪或随机抖动
    // 保持原值，避免末段辅助破坏原有安全决策。
    return false;
  }

  const double unit_x = vx / command_speed;
  const double unit_y = vy / command_speed;
  double executable_speed = safe_minimum_speed;

  // 抬升后的向量仍必须落在 Optimizer 当前有效约束内；这里包含外部 speed limit，
  // 因此终点辅助不会越过限速区或上层临时限速。
  if (std::abs(unit_x) > 1e-6) {
    const double x_limit = unit_x > 0.0 ?
      std::max(0.0f, constraints.vx_max) :
      std::max(0.0f, -constraints.vx_min);
    executable_speed = std::min(executable_speed, x_limit / std::abs(unit_x));
  }
  if (std::abs(unit_y) > 1e-6) {
    const double y_limit = std::max(0.0f, constraints.vy);
    executable_speed = std::min(executable_speed, y_limit / std::abs(unit_y));
  }

  if (executable_speed <= command_speed + 1e-6) {
    return false;
  }

  cmd.twist.linear.x = unit_x * executable_speed;
  cmd.twist.linear.y = unit_y * executable_speed;
  return true;
}

void MPPIController::smoothFinalYawCommand(
  geometry_msgs::msg::TwistStamped & cmd,
  double goal_yaw_error,
  const geometry_msgs::msg::Twist & robot_speed)
{
  if (
    active_stage_ != STAGE_APPROACH ||
    approach_substage_ != APPROACH_FINAL_YAW ||
    final_yaw_cmd_wz_slew_rate_ <= 0.0f)
  {
    final_yaw_cmd_filter_active_ = false;
    final_yaw_direction_lock_active_ = false;
    final_yaw_locked_direction_ = 0.0f;
    return;
  }

  // 最终朝向阶段由位置迟滞负责守住目标点：位置一旦超出 exit 容差，下一周期
  // 会自动退回 positioning。默认在本阶段锁住 vx/vy，避免 VelocityDeadbandCritic
  // 或采样噪声输出毫米级平移，触发舵轮反复换向；需要边转边微调时可关闭该参数。
  if (final_yaw_lock_translation_) {
    cmd.twist.linear.x = 0.0;
    cmd.twist.linear.y = 0.0;
  }

  const rclcpp::Time now = clock_->now();
  double dt = 0.1;
  if (final_yaw_cmd_filter_active_) {
    dt = (now - last_final_yaw_cmd_time_).seconds();
    if (dt <= 0.0 || dt > 0.5) {
      dt = 0.1;
    }
  } else {
    last_final_yaw_cmd_wz_ = static_cast<float>(robot_speed.angular.z);
    final_yaw_cmd_filter_active_ = true;
  }

  const float wz_limit = std::max(0.0f, applied_profile_.velocity.wz_max);
  const float abs_yaw_error = static_cast<float>(std::abs(goal_yaw_error));
  const float stop_yaw_tolerance = std::max(0.0f, final_yaw_cmd_stop_yaw_tolerance_);
  float target_wz = static_cast<float>(cmd.twist.angular.z);
  if (
    stop_yaw_tolerance > 0.0f &&
    abs_yaw_error <= stop_yaw_tolerance)
  {
    target_wz = 0.0f;
    final_yaw_direction_lock_active_ = false;
    final_yaw_locked_direction_ = 0.0f;
  } else if (abs_yaw_error > final_yaw_cmd_direction_deadband_) {
    // final_yaw 阶段只做目标朝向收敛。航向误差还明显大于死区时，锁定正确旋转
    // 方向，避免 yaw_error 在 ±pi 边界从 +3.13 跳到 -3.13 后立刻反向。
    //
    // 角速度不再“一直顶到 wz_max 再突然停”，而是按剩余角度和可用角加速度
    // 计算刹车曲线：误差大时允许接近 wz_max，越接近目标角度越提前降速，减少
    // 冲过目标姿态后的来回修正。
    if (!final_yaw_direction_lock_active_) {
      // goal_yaw_error 的符号语义是 current - goal。要让误差收敛到 0，
      // 角速度方向应与误差符号相反：误差为负需要正转，误差为正需要负转。
      final_yaw_locked_direction_ = goal_yaw_error > 0.0 ? -1.0f : 1.0f;
      final_yaw_direction_lock_active_ = true;
      RCLCPP_INFO(
        logger_,
        "MPPI final_yaw direction locked: direction=%+.0f, yaw_error=%.3f rad. "
        "The direction will be kept until |yaw_error| <= %.3f rad.",
        final_yaw_locked_direction_, goal_yaw_error, final_yaw_cmd_direction_deadband_);
    }
    float target_wz_abs = wz_limit;
    if (final_yaw_cmd_brake_accel_ > 0.0f) {
      const float braking_yaw_error = std::max(0.0f, abs_yaw_error - stop_yaw_tolerance);
      target_wz_abs = std::min(
        wz_limit,
        std::sqrt(2.0f * final_yaw_cmd_brake_accel_ * braking_yaw_error));
    }
    target_wz = final_yaw_locked_direction_ * target_wz_abs;
  } else {
    final_yaw_direction_lock_active_ = false;
    final_yaw_locked_direction_ = 0.0f;
    // 接近目标方向时，不强行顶满角速度，只禁止明显反向的小抖动。
    if (goal_yaw_error > 0.0) {
      target_wz = std::min(0.0f, target_wz);
    } else {
      target_wz = std::max(0.0f, target_wz);
    }
  }

  const float max_delta = final_yaw_cmd_wz_slew_rate_ * static_cast<float>(dt);
  const float filtered_wz = std::clamp(
    target_wz,
    last_final_yaw_cmd_wz_ - max_delta,
    last_final_yaw_cmd_wz_ + max_delta);
  cmd.twist.angular.z = std::clamp(filtered_wz, -wz_limit, wz_limit);

  last_final_yaw_cmd_wz_ = static_cast<float>(cmd.twist.angular.z);
  last_final_yaw_cmd_time_ = now;
}

size_t MPPIController::selectStage(double goal_distance) const
{
  (void)goal_distance;
  // 位置控制不再按距离切换 profile；只有 2 cm 内的最终 yaw 由控制循环直接接管。
  return STAGE_TRACK;
}

std::string MPPIController::stageName(size_t profile) const
{
  switch (profile) {
    case 0:
      return "align";
    case 1:
      return "track";
    case 2:
      return "approach";
    default:
      return "unknown";
  }
}

std::string MPPIController::approachSubstageName() const
{
  if (active_stage_ != STAGE_APPROACH) {
    return "inactive";
  }
  return approach_substage_ == APPROACH_FINAL_YAW ? "final_yaw" : "positioning";
}

std::string MPPIController::approachModeName() const
{
  if (active_stage_ != STAGE_APPROACH) {
    return "inactive";
  }
  return approach_mode_ == APPROACH_POSITION_FIRST ? "position_first" : "normal";
}

MppiProfileSettings MPPIController::interpolateProfiles(
  const MppiProfileSettings & from,
  const MppiProfileSettings & to,
  double ratio) const
{
  ratio = std::max(0.0, std::min(1.0, ratio));

  const bool entering_goal_profile =
    to.critics.goal_lateral.weight > from.critics.goal_lateral.weight &&
    to.critics.path_align.weight < from.critics.path_align.weight;
  const double velocity_ratio = ratio;
  // 进入末段时 goal 慢启动、path 慢退场，避免切换初期突然丢失路径牵引。
  const double goal_ratio = entering_goal_profile ? ratio * ratio : ratio;
  const double path_ratio = entering_goal_profile ? ratio * ratio : ratio;

  const auto lerpFloatWithRatio = [](float lhs, float rhs, double blend_ratio) {
      return static_cast<float>(lhs + (rhs - lhs) * blend_ratio);
    };
  const auto lerpFloat = [ratio, &lerpFloatWithRatio](float lhs, float rhs) {
      return lerpFloatWithRatio(lhs, rhs, ratio);
    };
  const auto lerpInt = [ratio](int lhs, int rhs) {
      return static_cast<int>(std::lround(lhs + (rhs - lhs) * ratio));
    };

  MppiProfileSettings result = to;
  // 速度约束线性过渡，目标类 critic 在进入末段时更晚接管。
  result.velocity.vx_max =
    lerpFloatWithRatio(from.velocity.vx_max, to.velocity.vx_max, velocity_ratio);
  result.velocity.vx_min =
    lerpFloatWithRatio(from.velocity.vx_min, to.velocity.vx_min, velocity_ratio);
  result.velocity.vy_max =
    lerpFloatWithRatio(from.velocity.vy_max, to.velocity.vy_max, velocity_ratio);
  result.velocity.wz_max =
    lerpFloatWithRatio(from.velocity.wz_max, to.velocity.wz_max, velocity_ratio);
  result.sampling_std.vx =
    lerpFloat(from.sampling_std.vx, to.sampling_std.vx);
  result.sampling_std.vy =
    lerpFloat(from.sampling_std.vy, to.sampling_std.vy);
  result.sampling_std.wz =
    lerpFloat(from.sampling_std.wz, to.sampling_std.wz);

  result.critics.goal.weight =
    lerpFloatWithRatio(from.critics.goal.weight, to.critics.goal.weight, goal_ratio);

  result.critics.goal_lateral.weight =
    lerpFloatWithRatio(
    from.critics.goal_lateral.weight, to.critics.goal_lateral.weight, goal_ratio);

  result.critics.goal_longitudinal.weight =
    lerpFloatWithRatio(
    from.critics.goal_longitudinal.weight, to.critics.goal_longitudinal.weight, goal_ratio);

  result.critics.goal_angle.weight =
    lerpFloatWithRatio(
    from.critics.goal_angle.weight, to.critics.goal_angle.weight, goal_ratio);

  result.critics.path_angle.weight =
    lerpFloatWithRatio(
    from.critics.path_angle.weight, to.critics.path_angle.weight, path_ratio);
  result.critics.path_angle.offset_from_furthest = std::max(
    0, lerpInt(
      from.critics.path_angle.offset_from_furthest,
      to.critics.path_angle.offset_from_furthest));
  result.critics.path_angle.max_angle_to_furthest = lerpFloat(
    from.critics.path_angle.max_angle_to_furthest,
    to.critics.path_angle.max_angle_to_furthest);
  result.critics.path_angle.forward_preference =
    ratio < 0.5 ? from.critics.path_angle.forward_preference :
    to.critics.path_angle.forward_preference;

  result.critics.path_follow.weight =
    lerpFloatWithRatio(
    from.critics.path_follow.weight, to.critics.path_follow.weight, path_ratio);
  result.critics.path_follow.offset_from_furthest = std::max(
    0, lerpInt(
      from.critics.path_follow.offset_from_furthest,
      to.critics.path_follow.offset_from_furthest));

  result.critics.path_align.weight =
    lerpFloatWithRatio(
    from.critics.path_align.weight, to.critics.path_align.weight, path_ratio);
  result.critics.path_align.offset_from_furthest = std::max(
    0, lerpInt(
      from.critics.path_align.offset_from_furthest,
      to.critics.path_align.offset_from_furthest));
  result.critics.path_align.trajectory_point_step = std::max(
    1, lerpInt(
      from.critics.path_align.trajectory_point_step,
      to.critics.path_align.trajectory_point_step));
  result.critics.path_align.max_path_occupancy_ratio = lerpFloat(
    from.critics.path_align.max_path_occupancy_ratio,
    to.critics.path_align.max_path_occupancy_ratio);

  result.critics.prefer_forward.weight =
    lerpFloatWithRatio(
    from.critics.prefer_forward.weight, to.critics.prefer_forward.weight, path_ratio);

  return result;
}

void MPPIController::updateProfileTransition()
{
  if (!profile_transition_active_) {
    return;
  }

  const double elapsed =
    (clock_->now() - profile_transition_start_time_).seconds();
  const double ratio = profile_transition_duration_ <= 0.0 ?
    1.0 : elapsed / profile_transition_duration_;

  if (ratio >= 1.0) {
    applied_profile_ = transition_target_profile_;
    profile_transition_active_ = false;
  } else {
    applied_profile_ = interpolateProfiles(
      transition_start_profile_, transition_target_profile_, ratio);
  }
}

void MPPIController::visualize(nav_msgs::msg::Path transformed_plan)
{
  trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(), "Candidate Trajectories");
  trajectory_visualizer_.add(optimizer_.getOptimizedTrajectory(), "Optimal Trajectory");
  trajectory_visualizer_.visualize(std::move(transformed_plan));
}

MPPIController::PathControlMode MPPIController::configurePlanForControl(
  const nav_msgs::msg::Path & path, nav_msgs::msg::Path & tracking_path)
{
  tracking_path = path;
  if (path.poses.size() < 2) {
    return PathControlMode::AUTO;
  }

  const bool has_start_marker = poseDistance(
    path.poses.front().pose, path.poses[1].pose) <= tangent_marker_position_tolerance_;
  const bool has_second_start_marker =
    path.poses.size() >= 3 &&
    poseDistance(
    path.poses[1].pose, path.poses[2].pose) <= tangent_marker_position_tolerance_;
  const bool has_goal_marker = poseDistance(
    path.poses[path.poses.size() - 2].pose,
    path.poses.back().pose) <= tangent_marker_position_tolerance_;
  const auto & final_goal = path.poses.back().pose;

  if (has_start_marker && has_second_start_marker) {
    // START：两个连续起点 marker。先识别它，避免被单 marker 的 GOAL 规则吞掉。
    tracking_path.poses.erase(tracking_path.poses.begin(), tracking_path.poses.begin() + 2);
    if (!tracking_path.poses.empty()) {
      tracking_path.poses.back().pose.orientation = final_goal.orientation;
    }
    return PathControlMode::START;
  }

  if (has_start_marker && has_goal_marker && path.poses.size() >= 4) {
    // TANGENT：首尾 marker；起点对齐出发切线，途中跟随切线，终点再处理 goal yaw。
    tracking_path.poses.erase(tracking_path.poses.begin());
    tracking_path.poses.pop_back();
    tracking_path.poses.back().pose.orientation = final_goal.orientation;
    return PathControlMode::TANGENT;
  }

  if (has_start_marker) {
    // GOAL：只有起点 marker；起点先对齐 goal yaw，之后保持该 yaw 平移。
    tracking_path.poses.erase(tracking_path.poses.begin());
    return PathControlMode::GOAL;
  }

  if (has_goal_marker) {
    // NONE：只有终点 marker；移除 marker 后恢复最终 goal yaw 供统一 FINAL_YAW 使用，
    // TRACK 阶段会按 current_path_control_mode_ 完全关闭 PathAlign orientation。
    tracking_path.poses.pop_back();
    tracking_path.poses.back().pose.orientation = final_goal.orientation;
    return PathControlMode::NONE;
  }

  // AUTO 无 marker；它是唯一允许按路径弧长边走边改变 yaw 的普通模式。
  return PathControlMode::AUTO;
}

bool MPPIController::shouldRunStartControl(PathControlMode mode) const
{
  return tangent_phase_control_enabled_ &&
         (mode == PathControlMode::TANGENT || mode == PathControlMode::GOAL);
}

const char * MPPIController::pathControlModeName(PathControlMode mode)
{
  switch (mode) {
    case PathControlMode::TANGENT: return "tangent";
    case PathControlMode::GOAL: return "goal";
    case PathControlMode::NONE: return "none";
    case PathControlMode::START: return "start";
    case PathControlMode::AUTO: return "auto";
  }
  return "unknown";
}

bool MPPIController::isSameNavigationGoal(const geometry_msgs::msg::Pose & goal) const
{
  return navigation_goal_initialized_ &&
         poseDistance(tangent_goal_pose_, goal) <= tangent_goal_xy_tolerance_ &&
         std::abs(shortestAngularDistance(
           tf2::getYaw(tangent_goal_pose_.orientation),
           tf2::getYaw(goal.orientation))) <= tangent_goal_yaw_tolerance_;
}

bool MPPIController::configureStartPoseInControllerFrame(
  const nav_msgs::msg::Path & transformed_path)
{
  if (transformed_path.poses.empty()) {
    return false;
  }

  tangent_cruise_start_pose_ = transformed_path.poses.front().pose;
  if (current_path_control_mode_ == PathControlMode::GOAL) {
    // GOAL 的起点目标不是路径几何切线，而是用户指定的最终 yaw。路径已变换到
    // controller 坐标系，因此直接使用末点 orientation，不混用 map/odom 角度。
    tangent_cruise_start_pose_.orientation =
      transformed_path.poses.back().pose.orientation;
    return true;
  }

  // TANGENT 才从路径几何按固定弧长前视计算出发切线。
  const auto & start = transformed_path.poses.front().pose.position;
  double integrated_distance = 0.0;
  size_t fallback_index = 0;

  for (size_t i = 1; i < transformed_path.poses.size(); ++i) {
    const auto & previous = transformed_path.poses[i - 1].pose.position;
    const auto & current = transformed_path.poses[i].pose.position;
    integrated_distance += std::hypot(
      current.x - previous.x, current.y - previous.y);

    const double dx = current.x - start.x;
    const double dy = current.y - start.y;
    if (std::hypot(dx, dy) <= tangent_marker_position_tolerance_) {
      continue;
    }

    fallback_index = i;
    if (integrated_distance < start_yaw_lookahead_distance_) {
      continue;
    }

    const double start_yaw = std::atan2(dy, dx);
    tangent_cruise_start_pose_.orientation.x = 0.0;
    tangent_cruise_start_pose_.orientation.y = 0.0;
    tangent_cruise_start_pose_.orientation.z = std::sin(start_yaw * 0.5);
    tangent_cruise_start_pose_.orientation.w = std::cos(start_yaw * 0.5);
    return true;
  }

  // 目标距离短于前视距离时，使用路径上最远的有效点；仍然避免退回相邻栅格点。
  if (fallback_index > 0) {
    const auto & fallback = transformed_path.poses[fallback_index].pose.position;
    const double start_yaw = std::atan2(fallback.y - start.y, fallback.x - start.x);
    tangent_cruise_start_pose_.orientation.x = 0.0;
    tangent_cruise_start_pose_.orientation.y = 0.0;
    tangent_cruise_start_pose_.orientation.z = std::sin(start_yaw * 0.5);
    tangent_cruise_start_pose_.orientation.w = std::cos(start_yaw * 0.5);
  }
  return true;
}

std::optional<geometry_msgs::msg::TwistStamped>
MPPIController::computeTangentStartCommand(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed)
{
  if (
    tangent_phase_ == TangentPhase::DISABLED ||
    tangent_phase_ == TangentPhase::TRACK)
  {
    return std::nullopt;
  }

  auto transition = [this](TangentPhase next) {
      if (tangent_phase_ != next) {
        RCLCPP_INFO(
          logger_, "tangent 起点阶段：%s -> %s",
          tangentPhaseName(tangent_phase_), tangentPhaseName(next));
        tangent_phase_ = next;
      }
    };

  const double linear_speed = std::hypot(robot_speed.linear.x, robot_speed.linear.y);
  const double angular_speed = std::abs(robot_speed.angular.z);
  const double current_yaw = tf2::getYaw(robot_pose.pose.orientation);

  // 一次控制周期内允许连续跨过无需等待的状态，减少无意义的零速周期。
  for (int transition_count = 0; transition_count < 4; ++transition_count) {
    switch (tangent_phase_) {
      case TangentPhase::START_STOP:
        if (
          linear_speed > tangent_stopped_linear_velocity_ ||
          angular_speed > tangent_stopped_angular_velocity_)
        {
          return makeStoppedCommand();
        }
        last_start_rotation_command_wz_ = 0.0;
        start_rotation_command_initialized_ = true;
        transition(TangentPhase::START_TANGENT_ALIGN);
        break;

      case TangentPhase::START_TANGENT_ALIGN: {
          const double yaw_error = shortestAngularDistance(
            current_yaw, tf2::getYaw(tangent_cruise_start_pose_.orientation));
          if (std::abs(yaw_error) > tangent_start_yaw_tolerance_) {
            return makeRotationCommand(
              yaw_error, robot_pose, robot_speed, tangent_start_yaw_tolerance_);
          }
          last_start_rotation_command_wz_ = 0.0;
          start_rotation_command_initialized_ = false;
          transition(TangentPhase::START_SETTLE);
          break;
        }

      case TangentPhase::START_SETTLE:
        if (
          linear_speed > tangent_stopped_linear_velocity_ ||
          angular_speed > tangent_stopped_angular_velocity_)
        {
          return makeStoppedCommand();
        }
        // 起点已经对齐并停稳，之后只进入 TRACK/APPROACH。局部路径变化由
        // 全向 vx/vy/wz 连续跟踪，不再存在运行中的 ALIGN 状态切换。
        active_stage_ = STAGE_TRACK;
        active_stage_enter_time_ = clock_->now();
        profile_transition_active_ = false;
        applied_profile_ = stages_[STAGE_TRACK];
        transition(TangentPhase::TRACK);
        // 明确发布一个零速周期作为起点状态机与 MPPI 的边界；下一周期开始 TRACK。
        return makeStoppedCommand();

      case TangentPhase::TRACK:
      case TangentPhase::DISABLED:
        return std::nullopt;
    }
  }

  return makeStoppedCommand();
}

geometry_msgs::msg::TwistStamped MPPIController::makeStoppedCommand() const
{
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = clock_->now();
  command.header.frame_id = tangent_command_frame_;
  return command;
}

geometry_msgs::msg::TwistStamped MPPIController::makeRotationCommand(
  double yaw_error,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  double yaw_tolerance)
{
  geometry_msgs::msg::TwistStamped command = makeStoppedCommand();
  const double abs_yaw_error = std::abs(yaw_error);
  double desired_abs = std::min(
    tangent_rotate_max_speed_, tangent_rotate_kp_ * abs_yaw_error);

  // 按“剩余误差 - 允许容差”计算可停车角速度，提前进入制动，不再一直保持
  // min_speed 冲过目标后反向找 yaw。若已经在减速，则允许命令降到物理死区以下；
  // 只有机器人确实停住且仍在容差外时，才给一次最小速度用于克服静摩擦。
  if (tangent_rotate_max_accel_ > 0.0) {
    const double braking_angle = std::max(
      0.0, abs_yaw_error - std::max(0.0, yaw_tolerance));
    desired_abs = std::min(
      desired_abs, std::sqrt(2.0 * tangent_rotate_max_accel_ * braking_angle));
  }
  if (
    desired_abs > 0.0 && desired_abs < tangent_rotate_min_speed_ &&
    std::abs(robot_speed.angular.z) <= tangent_stopped_angular_velocity_)
  {
    desired_abs = tangent_rotate_min_speed_;
  }
  const double desired = std::copysign(desired_abs, yaw_error);

  const double max_delta = tangent_rotate_max_accel_ * control_duration_;
  if (!start_rotation_command_initialized_) {
    // START_STOP 已保证机器人停稳；测试或异常直接进入本函数时才用实测速度接续。
    last_start_rotation_command_wz_ = robot_speed.angular.z;
    start_rotation_command_initialized_ = true;
  }
  command.twist.angular.z = std::clamp(
    desired,
    last_start_rotation_command_wz_ - max_delta,
    last_start_rotation_command_wz_ + max_delta);
  last_start_rotation_command_wz_ = command.twist.angular.z;
  ensureRotationIsCollisionFree(robot_pose, command, yaw_error);
  return command;
}

void MPPIController::ensureRotationIsCollisionFree(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::TwistStamped & command,
  double yaw_error) const
{
  const double angular_speed = std::abs(command.twist.angular.z);
  if (!tangent_collision_checker_ || angular_speed < 1e-6) {
    return;
  }

  const double rotation_time = std::abs(yaw_error) / angular_speed;
  const double simulate_time = std::min(
    tangent_rotation_simulate_ahead_time_, rotation_time);
  const double initial_yaw = tf2::getYaw(robot_pose.pose.orientation);

  auto * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
  for (
    double time = control_duration_;
    time <= simulate_time + 1e-9;
    time += control_duration_)
  {
    const double yaw = initial_yaw + command.twist.angular.z * time;
    const double footprint_cost = tangent_collision_checker_->footprintCostAtPose(
      robot_pose.pose.position.x, robot_pose.pose.position.y, yaw,
      costmap_ros_->getRobotFootprint());
    if (
      (footprint_cost == static_cast<double>(nav2_costmap_2d::NO_INFORMATION) &&
      costmap_ros_->getLayeredCostmap()->isTrackingUnknown()) ||
      footprint_cost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE))
    {
      throw nav2_core::PlannerException("起点原地对齐存在碰撞风险");
    }
  }
}

const char * MPPIController::tangentPhaseName(TangentPhase phase)
{
  switch (phase) {
    case TangentPhase::DISABLED: return "disabled";
    case TangentPhase::START_STOP: return "start_stop";
    case TangentPhase::START_TANGENT_ALIGN: return "start_yaw_align";
    case TangentPhase::START_SETTLE: return "start_settle";
    case TangentPhase::TRACK: return "track";
  }
  return "unknown";
}

void MPPIController::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    tangent_phase_ = TangentPhase::DISABLED;
    current_path_control_mode_ = PathControlMode::AUTO;
    start_yaw_target_pending_ = false;
    // 显式空路径代表当前控制任务已经被清空；之后即便目标坐标相同，也应作为
    // 新任务若为 tangent 会重新执行起点对齐；恢复重试只更新非空 plan，不走这里。
    navigation_goal_initialized_ = false;
    path_handler_.setPath(path);
    return;
  }

  nav_msgs::msg::Path tracking_path;
  const PathControlMode path_control_mode = configurePlanForControl(path, tracking_path);

  const auto & final_goal = path.poses.back().pose;
  // FollowPath 失败后的 Wait/Clear/BackUp 会让控制循环暂停数秒，但仍属于同一个
  // 导航目标。不能再用“超过 reset_period 没调用控制器”推断任务已结束，否则每次
  // 恢复都会重新进入起点 yaw 对齐。任务结束由显式空路径/生命周期清理复位。
  const bool same_task =
    isSameNavigationGoal(final_goal) && path_control_mode == current_path_control_mode_;
  tangent_goal_pose_ = final_goal;
  navigation_goal_initialized_ = true;
  current_path_control_mode_ = path_control_mode;

  if (!same_task) {
    const bool run_start_control = shouldRunStartControl(path_control_mode);
    tangent_phase_ = run_start_control ?
      TangentPhase::START_STOP : TangentPhase::TRACK;
    start_yaw_target_pending_ = run_start_control;
    active_stage_ = STAGE_TRACK;
    approach_substage_ = APPROACH_POSITIONING;
    approach_mode_ = APPROACH_NORMAL;
    approach_mode_locked_ = false;
    final_yaw_rotation_started_ = false;
    last_start_rotation_command_wz_ = 0.0;
    start_rotation_command_initialized_ = false;
    profile_transition_active_ = false;
    applied_profile_ = stages_[STAGE_TRACK];
    optimizer_.reset();
    optimizer_.applyProfile(applied_profile_);
    active_stage_enter_time_ = clock_->now();
    RCLCPP_INFO(
      logger_,
      "新导航目标：orientation_mode=%s，起点对齐=%s；goal_yaw=%.3f",
      pathControlModeName(path_control_mode),
      run_start_control ? "执行" : "跳过，直接 FOLLOW",
      tf2::getYaw(tangent_goal_pose_.orientation));
  } else {
    RCLCPP_DEBUG(
      logger_, "同目标重规划：只替换 FOLLOW 路径，保留起点状态 %s",
      tangentPhaseName(tangent_phase_));
  }

  path_handler_.setPath(tracking_path);
}

void MPPIController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  optimizer_.setSpeedLimit(speed_limit, percentage);
}

}  // namespace nav2_mppi_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_mppi_controller::MPPIController, nav2_core::Controller)
