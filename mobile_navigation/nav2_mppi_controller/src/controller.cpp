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
#include <limits>
#include <string>
#include "nav2_mppi_controller/controller.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "tf2/utils.h"  // tf2::getYaw（到点精度 log）
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// #define BENCHMARK_TESTING

namespace nav2_mppi_controller
{

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
  getParam(traj_speed_cap_lookahead_, "trajectory_speed_cap_lookahead", 0.2);
  configureStages();
  active_stage_enter_time_ = clock_->now();

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

  RCLCPP_INFO(logger_, "Configured MPPI Controller: %s", name_.c_str());
}

void MPPIController::cleanup()
{
  optimizer_.shutdown();
  trajectory_visualizer_.on_cleanup();
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
  final_yaw_locked_direction_ = 0.0f;
  last_final_yaw_cmd_wz_ = 0.0f;
  final_yaw_braking_active_ = false;
  final_yaw_envelope_braking_active_ = false;
  final_yaw_control_state_ = FINAL_YAW_TRACKING;
  final_yaw_exit_violation_count_ = 0;
  reference_trajectory_.points.clear();
  optimizer_.setTrajectorySpeedCap(-1.0);
}

void MPPIController::setTrajectory(const nav_interfaces::msg::RobotTrajectory & trajectory)
{
  reference_trajectory_ = trajectory;
}

double MPPIController::nearestTrajectorySpeed(
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  if (reference_trajectory_.points.empty()) {
    return -1.0;
  }
  std::string traj_frame = reference_trajectory_.header.frame_id;
  if (traj_frame.empty()) {
    traj_frame = reference_trajectory_.points.front().pose.header.frame_id;
  }
  geometry_msgs::msg::PoseStamped pose_in_traj = robot_pose;
  if (!traj_frame.empty() && robot_pose.header.frame_id != traj_frame) {
    try {
      geometry_msgs::msg::PoseStamped query = robot_pose;
      query.header.stamp = rclcpp::Time(0);
      tf_buffer_->transform(query, pose_in_traj, traj_frame, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException &) {
      return -1.0;  // TF 失败：本周期不设帽，回退 profile 约束
    }
  }
  size_t nearest_i = 0;
  double best_d2 = std::numeric_limits<double>::max();
  const auto & pts = reference_trajectory_.points;
  for (size_t i = 0; i < pts.size(); ++i) {
    const double dx = pts[i].pose.pose.position.x - pose_in_traj.pose.position.x;
    const double dy = pts[i].pose.pose.position.y - pose_in_traj.pose.position.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      nearest_i = i;
    }
  }

  // 帽 = 前瞻窗 [最近点, +trajectory_speed_cap_lookahead] 内的最大规划速度，
  // 不能取最近点本身——剖面每周期从当前实际速度重规划（receding-horizon），
  // "当前位置的计划速度"恒等于当前速度，静止时会形成"帽=0 → 动不了 → 速度恒 0"
  // 的自引用死锁。窗的含义：允许 MPPI 领先计划加速坡多远——窗越小越贴近计划的
  // 加速形状，但必须至少包含最近点的下一个点（点距稀疏时防死锁复发）；
  // 减速段剖面递减，窗内最大值=当前位置计划速度，不受窗大小影响（不提前刹）。
  double cap = std::fabs(pts[nearest_i].longitudinal_velocity);
  double s = 0.0;
  for (size_t j = nearest_i + 1; j < pts.size(); ++j) {
    const double dx = pts[j].pose.pose.position.x - pts[j - 1].pose.pose.position.x;
    const double dy = pts[j].pose.pose.position.y - pts[j - 1].pose.pose.position.y;
    s += std::hypot(dx, dy);
    cap = std::max(cap, std::fabs(pts[j].longitudinal_velocity));
    if (s >= traj_speed_cap_lookahead_ && j > nearest_i + 1) {
      break;
    }
  }
  return cap;
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
  auto now = clock_->now();

  if (now - last_time_called_ > rclcpp::Duration::from_seconds(reset_period_)) {
    reset();
  }
  last_time_called_ = now;

  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());

  auto t_transform_start = std::chrono::high_resolution_clock::now();
  nav_msgs::msg::Path transformed_plan = path_handler_.transformPath(robot_pose);
  const double goal_distance = path_handler_.getLastGoalDistance();
  const double path_progress = path_handler_.getLastPathProgress();
  const double heading_path_angle = path_handler_.getLastHeadingPathAngle();

  // 提前捕获目标位姿（用于 approach 子阶段和到点精度 log；plan 后续可能被 move 走）
  // ④ 轨迹参考速度：最近轨迹点的规划速度作为本周期 vx 采样上限（叠加在 profile 之上）。
  // 减速尾段/曲率限速/障碍零速由速度规划剖面逐点编码；无轨迹/TF 失败时不设帽。
  optimizer_.setTrajectorySpeedCap(nearestTrajectorySpeed(robot_pose));

  // 提前捕获目标位姿（用于到点精度 log；transformed_plan 后续可能被 visualize move 走）
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
  applyProfileForPathState(
    path_progress, goal_distance, heading_path_angle, key_lon, key_lat, key_yaw);
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
  smoothFinalYawCommand(cmd, key_yaw, robot_speed);
  // final_yaw 阶段在输出端强制线速度归零，不依赖 optimizer 的速度约束
  // 避免 optimizer 旧控制序列残留、profile 过渡未完全应用等导致的小幅 vx
  if (
    active_stage_ == STAGE_APPROACH &&
    approach_substage_ == APPROACH_FINAL_YAW)
  {
    cmd.twist.linear.x = 0.0;
    cmd.twist.linear.y = 0.0;
  }
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

  // 关键运行日志（1 Hz）：控制阶段、目标误差、周期耗时以及命令/实际速度。
  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 1000,
    "[MPPI] stage=%s mode=%s substage=%s goal_dist=%.3fm "
    "err[lon=%.3f lat=%.3f yaw=%.3f] "
    "cycle=%.1fms(opt=%.1f) cmd[vx=%.2f wz=%.2f] act[vx=%.2f wz=%.2f]",
    stageName(active_stage_).c_str(), approachModeName().c_str(),
    approachSubstageName().c_str(), goal_distance,
    key_lon, key_lat, key_yaw, total_ms, optimize_ms,
    cmd.twist.linear.x, cmd.twist.angular.z,
    robot_speed.linear.x, robot_speed.angular.z);

  return cmd;
}

void MPPIController::publishProfileDebug(double path_progress, double goal_distance)
{
  if (profile_debug_publisher_->get_subscription_count() == 0) {
    return;
  }

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
  getParam(g_vx_max, "vx_max", 0.7, ParameterType::Static);
  getParam(g_vx_min, "vx_min", -0.5, ParameterType::Static);
  getParam(g_vy_max, "vy_max", 0.5, ParameterType::Static);
  getParam(g_wz_max, "wz_max", 0.75, ParameterType::Static);
  for (auto & stage : stages_) {
    stage.velocity = {g_vx_max, g_vx_min, g_vy_max, g_wz_max};
  }

  // 单阶段 MPPI 已删除：行为类 Goal/Path/PreferForward 代价只允许由 profile_1/2/3 管理。
  // 这里的默认值只作为缺省兜底，不再从 FollowPath 全局行为 critic 中继承。
  stages_[0].critics.goal = {0.0f};
  stages_[0].critics.goal_lateral = {0.0f};
  stages_[0].critics.goal_longitudinal = {0.0f};
  stages_[0].critics.goal_angle = {0.0f};
  stages_[0].critics.path_angle = {7.0f, 3, 0.7f, true};
  stages_[0].critics.path_align = {8.0f, 8, 3, 0.08f};
  stages_[0].critics.path_follow = {7.0f, 7};
  stages_[0].critics.prefer_forward = {6.0f};
  stages_[0].critics.linear_velocity = {0.0f};   // 对齐段关闭速度软约束（不惩罚后退机动）
  stages_[0].critics.obstacles = {1.1f};
  stages_[0].critics.cost = {3.81f};
  stages_[0].critics.angular_velocity = {0.0f};

  stages_[1].critics.goal = {0.0f};
  stages_[1].critics.goal_lateral = {0.0f};
  stages_[1].critics.goal_longitudinal = {0.0f};
  stages_[1].critics.goal_angle = {0.0f};
  stages_[1].critics.path_angle = {5.0f, 3, 0.9f, true};
  stages_[1].critics.path_align = {20.0f, 15, 2, 0.05f};
  stages_[1].critics.path_follow = {4.0f, 5};
  stages_[1].critics.prefer_forward = {5.0f};
  stages_[1].critics.linear_velocity = {8.0f};
  stages_[1].critics.obstacles = {1.1f};
  stages_[1].critics.cost = {3.81f};
  stages_[1].critics.angular_velocity = {0.0f};

  stages_[2].critics.goal = {4.0f};
  stages_[2].critics.goal_lateral = {40.0f};
  stages_[2].critics.goal_longitudinal = {25.0f};
  stages_[2].critics.goal_angle = {8.0f};
  stages_[2].critics.path_angle = {1.0f, 2, 1.0f, true};
  stages_[2].critics.path_align = {2.0f, 15, 3, 0.05f};
  stages_[2].critics.path_follow = {1.0f, 5};
  stages_[2].critics.prefer_forward = {1.0f};
  stages_[2].critics.linear_velocity = {8.0f};
  stages_[2].critics.obstacles = {0.0f};   // 趋近段软斥力关（保致命）
  stages_[2].critics.cost = {0.0f};
  stages_[2].critics.angular_velocity = {15.0f};

  // profile_start_distance 已废弃（对齐段改按角度切，不再按距起点距离）；成员保留仅供 debug 兼容
  getParam(approach_distance_, "approach_distance", 1.0, ParameterType::Static);
  getParam(profile_transition_duration_, "profile_transition_duration", 1.0, ParameterType::Static);
  getParam(approach_hysteresis_, "approach_hysteresis", 0.05, ParameterType::Static);
  approach_hysteresis_ = std::max(0.0, approach_hysteresis_);
  getParam(align_enter_angle_, "align_enter_angle", 0.52, ParameterType::Static);  // ~30°
  getParam(align_exit_angle_, "align_exit_angle", 0.31, ParameterType::Static);    // ~18° 迟滞
  align_exit_angle_ = std::min(align_exit_angle_, align_enter_angle_);
  getParam(
    track_stage_min_duration_, "track_stage_min_duration",
    1.2, ParameterType::Static);
  getParam(
    align_force_enter_angle_, "align_force_enter_angle",
    1.2, ParameterType::Static);
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
  // final_yaw 退出连续违反周期数：ROS 2 整数参数底层是 int64，用 int 中转避免 size_t 类型问题
  int final_yaw_exit_violation_threshold_param = 3;
  getParam(
    final_yaw_exit_violation_threshold_param, "approach_final_yaw_exit_violation_threshold",
    3, ParameterType::Static);
  final_yaw_exit_violation_threshold_ =
    static_cast<size_t>(std::max(1, final_yaw_exit_violation_threshold_param));
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
    10.0, ParameterType::Static);
  getParam(
    final_yaw_vx_max_, "approach_final_yaw_vx_max", 0.03, ParameterType::Static);
  getParam(
    final_yaw_wz_max_, "approach_final_yaw_wz_max", 0.4, ParameterType::Static);
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
  getParam(
    final_yaw_cmd_actual_decel_, "approach_final_yaw_cmd_actual_decel",
    0.4, ParameterType::Static);
  final_yaw_cmd_actual_decel_ = std::max(0.01f, final_yaw_cmd_actual_decel_);
  getParam(
    final_yaw_cmd_delay_, "approach_final_yaw_cmd_delay",
    0.15, ParameterType::Static);
  final_yaw_cmd_delay_ = std::max(0.0f, final_yaw_cmd_delay_);
  getParam(
    final_yaw_cmd_brake_safety_margin_, "approach_final_yaw_cmd_brake_safety_margin",
    0.01, ParameterType::Static);
  final_yaw_cmd_brake_safety_margin_ = std::max(0.0f, final_yaw_cmd_brake_safety_margin_);
  getParam(
    final_yaw_dynamic_brake_min_wz_, "approach_final_yaw_dynamic_brake_min_wz",
    0.10, ParameterType::Static);
  final_yaw_dynamic_brake_min_wz_ = std::max(0.0f, final_yaw_dynamic_brake_min_wz_);
  getParam(
    final_yaw_creep_wz_min_, "approach_final_yaw_creep_wz_min",
    0.04, ParameterType::Static);
  final_yaw_creep_wz_min_ = std::max(0.0f, final_yaw_creep_wz_min_);
  getParam(
    final_yaw_creep_wz_max_, "approach_final_yaw_creep_wz_max",
    0.06, ParameterType::Static);
  final_yaw_creep_wz_max_ = std::max(final_yaw_creep_wz_min_, final_yaw_creep_wz_max_);
  getParam(
    final_yaw_internal_tolerance_, "approach_final_yaw_internal_tolerance",
    0.02, ParameterType::Static);
  final_yaw_internal_tolerance_ = std::max(0.0f, final_yaw_internal_tolerance_);
  getParam(
    final_yaw_creep_enter_yaw_error_, "approach_final_yaw_creep_enter_yaw_error",
    0.06, ParameterType::Static);
  final_yaw_creep_enter_yaw_error_ =
    std::max(final_yaw_internal_tolerance_, final_yaw_creep_enter_yaw_error_);
  getParam(
    final_yaw_creep_enter_max_actual_wz_, "approach_final_yaw_creep_enter_max_actual_wz",
    0.10, ParameterType::Static);
  final_yaw_creep_enter_max_actual_wz_ = std::max(0.0f, final_yaw_creep_enter_max_actual_wz_);
  getParam(
    final_yaw_stop_exit_tolerance_, "approach_final_yaw_stop_exit_tolerance",
    0.04, ParameterType::Static);
  final_yaw_stop_exit_tolerance_ =
    std::max(final_yaw_internal_tolerance_, final_yaw_stop_exit_tolerance_);
  getParam(
    final_yaw_envelope_brake_enter_margin_, "approach_final_yaw_envelope_brake_enter_margin",
    0.02, ParameterType::Static);
  final_yaw_envelope_brake_enter_margin_ = std::max(0.0f, final_yaw_envelope_brake_enter_margin_);
  getParam(
    final_yaw_envelope_brake_exit_margin_, "approach_final_yaw_envelope_brake_exit_margin",
    0.005, ParameterType::Static);
  final_yaw_envelope_brake_exit_margin_ = std::max(0.0f, final_yaw_envelope_brake_exit_margin_);
  final_yaw_enter_lon_tolerance_ = std::max(0.0, final_yaw_enter_lon_tolerance_);
  final_yaw_enter_lat_tolerance_ = std::max(0.0, final_yaw_enter_lat_tolerance_);
  final_yaw_exit_lon_tolerance_ = std::max(
    final_yaw_enter_lon_tolerance_, final_yaw_exit_lon_tolerance_);
  final_yaw_exit_lat_tolerance_ = std::max(
    final_yaw_enter_lat_tolerance_, final_yaw_exit_lat_tolerance_);
  track_stage_min_duration_ = std::max(0.0, track_stage_min_duration_);
  align_force_enter_angle_ = std::max(align_enter_angle_, align_force_enter_angle_);
  position_first_yaw_threshold_ = std::clamp(position_first_yaw_threshold_, 0.0, M_PI);
  position_first_lateral_error_threshold_ =
    std::max(0.0, position_first_lateral_error_threshold_);
  position_first_lateral_ratio_ = std::max(0.0, position_first_lateral_ratio_);
  positioning_goal_angle_weight_ = std::max(0.0f, positioning_goal_angle_weight_);
  final_yaw_vx_max_ = std::max(0.0f, final_yaw_vx_max_);
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
      profile_critics.prefer_forward.weight, prefix + "PreferForwardCritic.cost_weight",
      profile_critics.prefer_forward.weight, ParameterType::Static);
    getParam(
      profile_critics.linear_velocity.weight,
      prefix + "LinearVelocityCritic.cost_weight",
      profile_critics.linear_velocity.weight, ParameterType::Static);
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
          logger_, "stage_weights.%s needs 3 values [align,track,approach]; ignored", critic.c_str());
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
      "PathAlign(w=%.3f off=%d step=%d occ=%.3f) PathFollow(w=%.3f off=%d) "
      "PreferForward(w=%.3f). weight=0 disables this cost for the stage.",
      i + 1, profile.velocity.vx_min, profile.velocity.vx_max,
      profile.velocity.vy_max, profile.velocity.wz_max,
      profile.critics.goal.weight,
      profile.critics.goal_lateral.weight,
      profile.critics.goal_longitudinal.weight,
      profile.critics.goal_angle.weight,
      profile.critics.path_angle.weight, profile.critics.path_angle.offset_from_furthest,
      profile.critics.path_angle.max_angle_to_furthest, profile.critics.path_angle.forward_preference,
      profile.critics.path_align.weight, profile.critics.path_align.offset_from_furthest,
      profile.critics.path_align.trajectory_point_step, profile.critics.path_align.max_path_occupancy_ratio,
      profile.critics.path_follow.weight, profile.critics.path_follow.offset_from_furthest,
      profile.critics.prefer_forward.weight);
  }

  applied_profile_ = stages_[active_stage_];
  transition_start_profile_ = applied_profile_;
  transition_target_profile_ = applied_profile_;
}

void MPPIController::applyProfileForPathState(
  double path_progress, double goal_distance, double heading_path_angle,
  double goal_longitudinal_error, double goal_lateral_error, double goal_yaw_error)
{
  size_t selected_profile = selectStage(goal_distance, heading_path_angle);
  if (
    active_stage_ == STAGE_TRACK &&
    selected_profile == STAGE_ALIGN &&
    track_stage_min_duration_ > 0.0)
  {
    const double track_elapsed = (clock_->now() - active_stage_enter_time_).seconds();
    if (track_elapsed < track_stage_min_duration_ &&
      heading_path_angle < align_force_enter_angle_)
    {
      selected_profile = STAGE_TRACK;
    }
  }
  if (selected_profile == STAGE_APPROACH &&
    (active_stage_ != STAGE_APPROACH || !approach_mode_locked_))
  {
    updateApproachMode(
      active_stage_, selected_profile, goal_longitudinal_error,
      goal_lateral_error, goal_yaw_error);
  } else if (selected_profile != STAGE_APPROACH) {
    approach_mode_ = APPROACH_NORMAL;
    approach_mode_locked_ = false;
  }

  if (selected_profile != active_stage_) {
    const size_t previous_profile = active_stage_;
    active_stage_ = selected_profile;
    active_stage_enter_time_ = clock_->now();
    if (profile_transition_duration_ > 0.0) {
      transition_start_profile_ = applied_profile_;
      transition_target_profile_ = stages_[active_stage_];
      profile_transition_start_time_ = clock_->now();
      profile_transition_active_ = true;
    } else {
      applied_profile_ = stages_[active_stage_];
    }

    const auto & profile = stages_[active_stage_];
    RCLCPP_INFO(
      logger_,
      "MPPI profile switched: %zu -> %zu, reason=%s, path_progress=%.3f, "
      "goal_distance=%.3f, velocity vx=[%.3f, %.3f] vy=%.3f wz=%.3f. "
      "transition_duration=%.3f. "
      "Stage switched; new velocity constraints and critic params apply to the Optimizer over the transition duration.",
      previous_profile + 1, active_stage_ + 1, stageName(active_stage_).c_str(),
      path_progress, goal_distance, profile.velocity.vx_min, profile.velocity.vx_max,
      profile.velocity.vy_max, profile.velocity.wz_max, profile_transition_duration_);
  } else if (!profile_transition_active_) {
    // 每周期从阶段原始 profile 重新派生，避免上周期的 approach 子阶段覆盖残留。
    applied_profile_ = stages_[active_stage_];
  }

  updateProfileTransition();
  if (active_stage_ == STAGE_APPROACH) {
    // 进入最后 25 cm 时速度上限必须立即收紧。critic 权重仍可平滑过渡，但若速度约束也
    // 用 1 s 插值，短距离场景可能在限速完全生效前已经越过目标。
    applied_profile_.velocity.vx_max = std::min(
      applied_profile_.velocity.vx_max, stages_[STAGE_APPROACH].velocity.vx_max);
    applied_profile_.velocity.vx_min = std::max(
      applied_profile_.velocity.vx_min, stages_[STAGE_APPROACH].velocity.vx_min);
  }
  updateApproachSubstage(goal_longitudinal_error, goal_lateral_error);
  applyApproachSubstageProfile();
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
    // final_yaw 退出需连续 N 个周期位置误差超退出阈值，避免原地转向的厘米级漂移
    // 导致 final_yaw <-> positioning 反复切换（第一份日志主因）
    const bool position_far_outside =
      abs_lon > final_yaw_exit_lon_tolerance_ ||
      abs_lat > final_yaw_exit_lat_tolerance_;
    if (position_far_outside) {
      ++final_yaw_exit_violation_count_;
    } else {
      final_yaw_exit_violation_count_ = 0;
    }
    if (final_yaw_exit_violation_count_ >= final_yaw_exit_violation_threshold_) {
      approach_substage_ = APPROACH_POSITIONING;
      final_yaw_exit_violation_count_ = 0;
    }
  } else if (
    abs_lon <= final_yaw_enter_lon_tolerance_ &&
    abs_lat <= final_yaw_enter_lat_tolerance_)
  {
    approach_substage_ = APPROACH_FINAL_YAW;
  }

  if (previous_substage != approach_substage_) {
    // 每次进入 final_yaw 子阶段，重置内部状态机到 TRACKING
    if (approach_substage_ == APPROACH_FINAL_YAW) {
      final_yaw_control_state_ = FINAL_YAW_TRACKING;
      final_yaw_braking_active_ = false;
      final_yaw_exit_violation_count_ = 0;
    }
    RCLCPP_INFO(
      logger_, "MPPI approach substage switched: %s -> %s, |lon|=%.3f, |lat|=%.3f, "
      "exit_violation_count=%zu",
      previous_substage == APPROACH_FINAL_YAW ? "final_yaw" : "positioning",
      approachSubstageName().c_str(), abs_lon, abs_lat,
      final_yaw_exit_violation_count_);
  }
}

void MPPIController::applyApproachSubstageProfile()
{
  if (active_stage_ != STAGE_APPROACH) {
    return;
  }

  if (approach_substage_ == APPROACH_POSITIONING) {
    if (approach_mode_ == APPROACH_POSITION_FIRST) {
      // 大角度到点场景先修位置：暂时减弱最终朝向和角速度跟踪，给差速底盘留出平移纠偏能力。
      applied_profile_.critics.goal_angle.weight = std::min(
        applied_profile_.critics.goal_angle.weight, positioning_goal_angle_weight_);
      applied_profile_.critics.angular_velocity.weight = 0.0f;
    } else {
      // 小角度进入时不让 1 s 的阶段过渡延迟朝向接管，否则高速转动可能在权重升起前
      // 已越过目标方向。这里只立即启用朝向相关 critic，其余位置/路径权重仍平滑过渡。
      applied_profile_.critics.goal_angle.weight =
        stages_[STAGE_APPROACH].critics.goal_angle.weight;
      applied_profile_.critics.angular_velocity.weight =
        stages_[STAGE_APPROACH].critics.angular_velocity.weight;
      // GoalAngle 的最短方向惩罚会在越过目标方向后立即要求反向修正，避免位置/路径
      // 代价把机器人继续带成一整圈。
    }
    return;
  }

  // 最终转向近似原地完成：保留位置类 critic 防漂移，关闭会鼓励平移的软代价。
  // final_yaw 阶段线速度直接设为 0，避免原地转向时正负 vx 切换导致位置漂移触发子阶段切换
  // （第一份日志中 lon 从 0.005 漂到 0.035 退出 final_yaw 的主因）
  applied_profile_.velocity.vx_max = std::min(applied_profile_.velocity.vx_max, final_yaw_vx_max_);
  applied_profile_.velocity.vx_min = std::max(applied_profile_.velocity.vx_min, -final_yaw_vx_max_);
  applied_profile_.velocity.wz_max = std::min(applied_profile_.velocity.wz_max, final_yaw_wz_max_);
  applied_profile_.critics.path_angle.weight = 0.0f;
  applied_profile_.critics.linear_velocity.weight = 0.0f;
  applied_profile_.critics.prefer_forward.weight = 0.0f;
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
    final_yaw_braking_active_ = false;
    final_yaw_envelope_braking_active_ = false;
    return;
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
  // 方向死区不能小于停止容差，否则会在停止容差内还在反复切换方向锁
  const float direction_deadband =
    std::max(stop_yaw_tolerance, final_yaw_cmd_direction_deadband_);

  // 期望旋转方向：goal_yaw_error = current - goal，误差为正需要负转，为负需要正转
  const float desired_direction =
    goal_yaw_error > 0.0 ? -1.0f :
    goal_yaw_error < 0.0 ? 1.0f : 0.0f;

  float target_wz = 0.0f;
  // 本周期是否强制零输出（绕过斜率限制）
  bool force_zero_output = false;
  // 是否需要保持零速直到底盘实际停稳（用于 stop_tolerance / crossed_goal 场景）
  bool hold_zero_until_stopped = false;
  // 安全速度包络相关变量（在 else 分支内赋值，供日志使用）
  float safe_wz_limit = 0.0f;
  float effective_delay = 0.0f;

  // ========================================================================
  // 动态制动改造：同方向正常旋转用安全速度包络连续降速，不完全停车；
  // 只有反方向/越过目标/进 stop_tolerance 才完全停车（hold_zero_until_stopped）
  // ========================================================================

  // 底盘实际角速度状态
  const float actual_wz = static_cast<float>(robot_speed.angular.z);
  const float actual_wz_abs = std::abs(actual_wz);
  constexpr float kChassisStoppedWz = 0.015f;
  const bool chassis_stopped = actual_wz_abs <= kChassisStoppedWz;
  const bool chassis_still_moving = !chassis_stopped;

  // ===== crossed_goal 检测（使用上一周期方向锁快照）=====
  // 必须在更新本周期方向锁之前判断，否则正常越过目标时误差必然 < direction_deadband，
  // 方向锁已被清除，crossed_goal 永远为 false
  const bool previous_direction_lock_active = final_yaw_direction_lock_active_;
  const float previous_locked_direction = final_yaw_locked_direction_;
  const bool crossed_goal =
    previous_direction_lock_active &&
    abs_yaw_error < 0.5f &&
    previous_locked_direction * static_cast<float>(goal_yaw_error) > 0.0f;

  // ===== 方向锁判断（在 crossed_goal 之后更新）=====
  // 先确定本周期有效控制方向 command_direction，避免 ±π 边界跳变导致急停误判
  float command_direction = desired_direction;
  if (!crossed_goal) {
    if (abs_yaw_error > direction_deadband) {
      if (!final_yaw_direction_lock_active_) {
        final_yaw_locked_direction_ = desired_direction;
        final_yaw_direction_lock_active_ = true;
        RCLCPP_INFO(
          logger_,
          "MPPI final_yaw direction locked: direction=%+.0f, yaw_error=%.3f rad. "
          "The direction will be kept until |yaw_error| <= %.3f rad.",
          final_yaw_locked_direction_, goal_yaw_error, direction_deadband);
      }
      command_direction = final_yaw_locked_direction_;
    } else {
      final_yaw_direction_lock_active_ = false;
      final_yaw_locked_direction_ = 0.0f;
    }
  } else {
    // 越过目标：清除方向锁，本周期完全停车，下周期重新定向
    final_yaw_direction_lock_active_ = false;
    final_yaw_locked_direction_ = 0.0f;
  }

  // 是否必须完全停车：反方向旋转 / 越过目标 / 进入 stop_tolerance
  // 注意：用方向锁后的 command_direction 判断，不能用 desired_direction
  // 否则 ±π 边界跳变会让 desired_direction 瞬间反向，误判为 wrong_direction
  const bool actual_wrong_direction =
    chassis_still_moving && command_direction * actual_wz < 0.0f;

  if (abs_yaw_error <= stop_yaw_tolerance) {
    // 已进入停止容差，立即停车；若底盘未停稳则保持零速锁定
    target_wz = 0.0f;
    force_zero_output = true;
    hold_zero_until_stopped = chassis_still_moving;
    final_yaw_direction_lock_active_ = false;
    final_yaw_locked_direction_ = 0.0f;
  } else if (crossed_goal) {
    // 已经越过目标方向，先制动消除惯性，避免继续沿错误方向旋转
    target_wz = 0.0f;
    force_zero_output = true;
    hold_zero_until_stopped = chassis_still_moving;
    RCLCPP_INFO(
      logger_,
      "MPPI final_yaw crossed goal: prev_locked_dir=%+.0f, yaw_error=%.3f rad. "
      "Holding zero until the chassis stops.",
      previous_locked_direction, goal_yaw_error);
  } else if (actual_wrong_direction) {
    // 底盘实际方向与有效控制方向相反：必须先完全停车再重新定向
    target_wz = 0.0f;
    force_zero_output = true;
    hold_zero_until_stopped = chassis_still_moving;
    final_yaw_direction_lock_active_ = false;
    final_yaw_locked_direction_ = 0.0f;
    RCLCPP_INFO(
      logger_,
      "MPPI final_yaw actual wrong direction: cmd_dir=%+.0f, actual_wz=%.3f. "
      "Holding zero until the chassis stops.",
      command_direction, actual_wz);
  } else {
    // ===== 正常同方向旋转：用安全速度包络连续降速，不完全停车 =====

    // 动态延迟：取配置值和本周期实测 dt 的较大值，钳在 [0.05, 0.18]
    // 控制周期偶发达到 150~180ms 时，安全包络会自动收紧，避免超调
    const float measured_cycle = static_cast<float>(dt);
    effective_delay = std::clamp(
      std::max(final_yaw_cmd_delay_, measured_cycle), 0.05f, 0.18f);

    // 安全速度包络：按剩余角度和实际减速度计算当前允许的最大角速度
    // v = sqrt(2 * a * s_eff)，s_eff 已扣除延迟期间继续转的角度和安全余量
    // 这是绝对上限，任何阶段都不能突破，否则会超调
    safe_wz_limit = wz_limit;
    if (final_yaw_cmd_actual_decel_ > 0.0f) {
      const float remaining_yaw = std::max(
        0.0f,
        abs_yaw_error - final_yaw_internal_tolerance_ - final_yaw_cmd_brake_safety_margin_);
      const float effective_remaining_yaw = std::max(
        0.0f, remaining_yaw - actual_wz_abs * effective_delay);
      safe_wz_limit = std::sqrt(
        2.0f * final_yaw_cmd_actual_decel_ * effective_remaining_yaw);
    }
    float target_wz_abs = std::min(wz_limit, safe_wz_limit);

    // CREEP 低速区：误差较小时用 [creep_min, creep_max] 范围的低速精调
    // CREEP 是"希望使用的速度"，安全包络是"不能超过的速度"，取 min
    // 这样越接近目标，安全包络越小，CREEP 不会突破安全包络导致超调
    if (
      abs_yaw_error > final_yaw_internal_tolerance_ &&
      abs_yaw_error <= final_yaw_creep_enter_yaw_error_)
    {
      const float remaining_error =
        std::max(0.0f, abs_yaw_error - final_yaw_internal_tolerance_);
      float creep_wz_abs = 1.5f * remaining_error;
      creep_wz_abs = std::clamp(
        creep_wz_abs, final_yaw_creep_wz_min_, final_yaw_creep_wz_max_);
      target_wz_abs = std::min(target_wz_abs, creep_wz_abs);
    }

    target_wz = command_direction * target_wz_abs;

    // 包络追赶制动：actual_wz 超出 safe_wz 时临时请求零速快速降速
    // 与 BRAKING 不同：不锁定到完全停稳，actual 回到包络内立即恢复连续控制
    // 解决"指令满足安全包络但底盘实际速度仍超出"的偶发超调
    const bool actual_above_envelope =
      actual_wz_abs > safe_wz_limit + final_yaw_envelope_brake_enter_margin_;
    if (!final_yaw_envelope_braking_active_ && actual_above_envelope) {
      final_yaw_envelope_braking_active_ = true;
      RCLCPP_INFO(
        logger_,
        "MPPI final_yaw envelope braking entered: actual=%.3f > safe=%.3f + margin=%.3f. "
        "Temp zero until actual back inside envelope.",
        actual_wz_abs, safe_wz_limit, final_yaw_envelope_brake_enter_margin_);
    }
    if (final_yaw_envelope_braking_active_) {
      const bool actual_back_inside_envelope =
        actual_wz_abs <= safe_wz_limit + final_yaw_envelope_brake_exit_margin_;
      if (actual_back_inside_envelope) {
        final_yaw_envelope_braking_active_ = false;
        RCLCPP_INFO(
          logger_,
          "MPPI final_yaw envelope braking exited: actual=%.3f <= safe=%.3f + margin=%.3f. "
          "Resume continuous control.",
          actual_wz_abs, safe_wz_limit, final_yaw_envelope_brake_exit_margin_);
      } else {
        // 仍在包络外：本周期强制零速降速，但不设 braking_active（不等完全停稳）
        target_wz = 0.0f;
        force_zero_output = true;
      }
    }
  }

  // stop_tolerance / crossed_goal / wrong_direction 场景下要求保持零速直到底盘停稳
  // 同步设置状态机为 BRAKING，保持日志状态与实际行为一致
  if (hold_zero_until_stopped) {
    final_yaw_braking_active_ = true;
    final_yaw_control_state_ = FINAL_YAW_BRAKING;
  }

  // BRAKING 状态：完全停车阶段，持续输出零直到底盘停稳
  if (final_yaw_braking_active_) {
    target_wz = 0.0f;
    force_zero_output = true;
    if (chassis_stopped) {
      final_yaw_braking_active_ = false;
      if (abs_yaw_error <= final_yaw_internal_tolerance_) {
        final_yaw_control_state_ = FINAL_YAW_STOP;
        RCLCPP_INFO(
          logger_,
          "MPPI final_yaw BRAKING->STOP: yaw_error=%.3f <= internal_tol=%.3f, chassis stopped.",
          goal_yaw_error, final_yaw_internal_tolerance_);
      } else {
        // 停稳后仍未到位：状态回 TRACKING，下一周期重新走安全速度包络/CREEP
        final_yaw_control_state_ = FINAL_YAW_TRACKING;
        RCLCPP_INFO(
          logger_,
          "MPPI final_yaw BRAKING->TRACKING: yaw_error=%.3f > internal_tol=%.3f, "
          "resuming continuous correction.",
          goal_yaw_error, final_yaw_internal_tolerance_);
      }
    }
  }

  // STOP 状态：已到位，持续输出零，等待 ControllerServer/GoalChecker 确认
  if (final_yaw_control_state_ == FINAL_YAW_STOP && !final_yaw_braking_active_) {
    target_wz = 0.0f;
    force_zero_output = true;
    // 误差超出 stop_exit_tolerance 后退出 STOP，统一回 TRACKING
    // CREEP 不再作为独立状态，只是 TRACKING 状态下小误差区的速度计算逻辑
    if (abs_yaw_error > final_yaw_stop_exit_tolerance_) {
      final_yaw_control_state_ = FINAL_YAW_TRACKING;
      RCLCPP_INFO(
        logger_,
        "MPPI final_yaw STOP->TRACKING: yaw_error=%.3f exceeded exit_tol=%.3f.",
        goal_yaw_error, final_yaw_stop_exit_tolerance_);
    }
  }

  // 斜率限制：加速慢、减速快；方向反转时本周期先刹到零
  const float accel_slew_rate = final_yaw_cmd_wz_slew_rate_;
  // 减速斜率至少与刹车曲线假设的减速度一致，否则斜率限制会让刹车比规划晚
  const float decel_slew_rate = std::max(
    2.0f * final_yaw_cmd_wz_slew_rate_,
    final_yaw_cmd_brake_accel_);

  const bool direction_reversal =
    target_wz * last_final_yaw_cmd_wz_ < 0.0f;
  // 实际底盘仍在旋转（未停稳，>0.015）且目标方向与实际方向相反时也必须先刹到零
  // 避免在底盘未停稳时直接发反向指令
  const bool actual_direction_reversal =
    chassis_still_moving && target_wz * actual_wz < 0.0f;
  const bool reducing_magnitude =
    std::abs(target_wz) < std::abs(last_final_yaw_cmd_wz_);
  const bool is_braking =
    direction_reversal || actual_direction_reversal ||
    reducing_magnitude || final_yaw_braking_active_;

  // 制动锁定、实际方向反转、指令方向反转时都先把目标钳到零，避免直接反向冲击
  float filter_target_wz = target_wz;
  if (
    final_yaw_braking_active_ ||
    actual_direction_reversal)
  {
    filter_target_wz = 0.0f;
  } else if (
    direction_reversal &&
    std::abs(last_final_yaw_cmd_wz_) > 1e-3f)
  {
    filter_target_wz = 0.0f;
  }

  // 任何强制零输出场景（stop_tolerance / crossed_goal / 动态制动）都直接请求零速，
  // 绕过本函数的斜率限制。动态停止距离已按底盘实测减速度规划，
  // 后续速度平滑器和底盘负责实际减速。
  float filtered_wz;
  if (force_zero_output || final_yaw_braking_active_) {
    filtered_wz = 0.0f;
  } else {
    const float selected_slew_rate = is_braking ? decel_slew_rate : accel_slew_rate;
    const float max_delta = selected_slew_rate * static_cast<float>(dt);
    filtered_wz =
      last_final_yaw_cmd_wz_ +
      std::clamp(
        filter_target_wz - last_final_yaw_cmd_wz_,
        -max_delta,
        max_delta);
  }

  // 安全包络硬钳制：减速时绝不能突破本周期安全目标
  // 加速可以慢，但减速时 |filtered| 不得 > |target|，否则安全包络形同虚设
  // 这是最关键的一道防线：斜率限制只控制变化率，不控制绝对值上限
  if (!force_zero_output && !final_yaw_braking_active_) {
    if (std::abs(target_wz) <= 1e-6f) {
      filtered_wz = 0.0f;
    } else if (filtered_wz * target_wz <= 0.0f) {
      // 过滤后方向与目标方向不同（包括反方向），先归零
      filtered_wz = 0.0f;
    } else {
      // 同方向：|filtered| 不得超过 |target|
      filtered_wz = std::copysign(
        std::min(std::abs(filtered_wz), std::abs(target_wz)),
        target_wz);
    }
  }

  cmd.twist.angular.z = std::clamp(filtered_wz, -wz_limit, wz_limit);

  // 节流日志：final_yaw 阶段每周期的目标/实际/安全包络/状态机，便于排查超调和卡顿
  // 实机必须确认：|filtered| <= |target| <= safe_wz，任何周期都不能违反
  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 500,
    "final_yaw control: error=%+.4f last=%+.3f target=%+.3f filtered=%+.3f actual=%+.3f "
    "safe_wz=%.3f delay=%.3f dt=%.3f braking=%d state=%zu",
    goal_yaw_error, last_final_yaw_cmd_wz_, target_wz,
    static_cast<float>(cmd.twist.angular.z),
    actual_wz, safe_wz_limit, effective_delay,
    static_cast<float>(dt), final_yaw_braking_active_,
    final_yaw_control_state_);

  last_final_yaw_cmd_wz_ = static_cast<float>(cmd.twist.angular.z);
  last_final_yaw_cmd_time_ = now;
}

size_t MPPIController::selectStage(double goal_distance, double heading_path_angle) const
{
  // 按语义切换，优先级：approach(趋近,距离) > align(对齐,角度) > track(跟踪,默认)，各带迟滞。
  if (active_stage_ == STAGE_APPROACH &&
    goal_distance <= approach_distance_ + approach_hysteresis_)
  {
    return STAGE_APPROACH;  // 趋近段距离迟滞
  }
  if (goal_distance <= approach_distance_) {
    return STAGE_APPROACH;
  }
  if (active_stage_ == STAGE_ALIGN && heading_path_angle > align_exit_angle_) {
    return STAGE_ALIGN;     // 对齐段角度迟滞（退出用更小阈值）
  }
  if (heading_path_angle > align_enter_angle_) {
    return STAGE_ALIGN;
  }
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
  trajectory_visualizer_.addWeighted(
    optimizer_.getGeneratedTrajectories(), optimizer_.getTrajectoryWeights(),
    "High Weight Trajectories");
  trajectory_visualizer_.add(optimizer_.getOptimizedTrajectory(), "Optimal Trajectory");
  trajectory_visualizer_.visualize(std::move(transformed_plan));
}

void MPPIController::setPlan(const nav_msgs::msg::Path & path)
{
  path_handler_.setPath(path);
}

void MPPIController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  optimizer_.setSpeedLimit(speed_limit, percentage);
}

}  // namespace nav2_mppi_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_mppi_controller::MPPIController, nav2_core::Controller)
