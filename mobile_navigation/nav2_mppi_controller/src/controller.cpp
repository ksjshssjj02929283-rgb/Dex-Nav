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
  configureProfiles();

  // Configure composed objects
  optimizer_.initialize(parent_, name_, costmap_ros_, parameters_handler_.get());
  if (profile_enabled_) {
    applied_profile_ = profiles_[active_profile_];
    optimizer_.applyProfile(applied_profile_);
  }
  parameters_handler_->addPostCallback(
    [this]() {
      if (profile_enabled_) {
        optimizer_.applyProfile(applied_profile_);
      }
    });
  path_handler_.initialize(parent_, name_, costmap_ros_, tf_buffer_, parameters_handler_.get());
  trajectory_visualizer_.on_configure(
    parent_, name_,
    costmap_ros_->getGlobalFrameID(), parameters_handler_.get());

  RCLCPP_INFO(logger_, "Configured MPPI Controller: %s", name_.c_str());
}

void MPPIController::cleanup()
{
  optimizer_.shutdown();
  trajectory_visualizer_.on_cleanup();
  parameters_handler_.reset();
  RCLCPP_INFO(logger_, "Cleaned up MPPI Controller: %s", name_.c_str());
}

void MPPIController::activate()
{
  trajectory_visualizer_.on_activate();
  parameters_handler_->start();
  RCLCPP_INFO(logger_, "Activated MPPI Controller: %s", name_.c_str());
}

void MPPIController::deactivate()
{
  trajectory_visualizer_.on_deactivate();
  RCLCPP_INFO(logger_, "Deactivated MPPI Controller: %s", name_.c_str());
}

void MPPIController::reset()
{
  optimizer_.reset();
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
  const double goal_distance = path_handler_.getLastGoalDistance();
  const double path_progress = path_handler_.getLastPathProgress();
  applyProfileForPathState(path_progress, goal_distance);
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
    visualize_ms >= kSlowTimingWarnMs || 1)
  {
    RCLCPP_WARN(
      logger_,
      "MPPI timing (ms): transform=%.3f wait_costmap_lock=%.3f optimize=%.3f visualize=%.3f "
      "total=%.3f. 控制周期耗时过长，可根据 transform/锁等待/优化/可视化字段定位卡顿环节。",
      transform_ms, wait_lock_ms, optimize_ms, visualize_ms, total_ms);
  }

  return cmd;
}


void MPPIController::configureProfiles()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  double vx_max = 0.5;
  double vx_min = -0.35;
  double vy_max = 0.5;
  double wz_max = 1.9;
  getParam(vx_max, "vx_max", 0.5, ParameterType::Static);
  getParam(vx_min, "vx_min", -0.35, ParameterType::Static);
  getParam(vy_max, "vy_max", 0.5, ParameterType::Static);
  getParam(wz_max, "wz_max", 1.9, ParameterType::Static);

  MppiProfileSettings default_profile;
  default_profile.velocity = {
    static_cast<float>(vx_max),
    static_cast<float>(vx_min),
    static_cast<float>(vy_max),
    static_cast<float>(wz_max)};

  auto & critics = default_profile.critics;
  getParam(critics.goal.weight, "GoalCritic.cost_weight", 5.0, ParameterType::Static);
  getParam(
    critics.goal.threshold_to_consider, "GoalCritic.threshold_to_consider", 1.4,
    ParameterType::Static);
  getParam(
    critics.goal_lateral.weight, "GoalLateralCritic.cost_weight", 0.0,
    ParameterType::Static);
  getParam(
    critics.goal_lateral.threshold_to_consider, "GoalLateralCritic.threshold_to_consider", 1.5,
    ParameterType::Static);
  getParam(
    critics.goal_lateral.last_n_points, "GoalLateralCritic.last_n_points", 8,
    ParameterType::Static);
  getParam(
    critics.goal_longitudinal.weight, "GoalLongitudinalCritic.cost_weight", 0.0,
    ParameterType::Static);
  getParam(
    critics.goal_longitudinal.threshold_to_consider,
    "GoalLongitudinalCritic.threshold_to_consider", 1.5, ParameterType::Static);
  getParam(
    critics.goal_longitudinal.last_n_points, "GoalLongitudinalCritic.last_n_points", 8,
    ParameterType::Static);
  getParam(
    critics.goal_angle.weight, "GoalAngleCritic.cost_weight", 3.0, ParameterType::Static);
  getParam(
    critics.goal_angle.threshold_to_consider, "GoalAngleCritic.threshold_to_consider", 0.5,
    ParameterType::Static);
  getParam(
    critics.path_angle.weight, "PathAngleCritic.cost_weight", 2.0, ParameterType::Static);
  getParam(
    critics.path_angle.threshold_to_consider, "PathAngleCritic.threshold_to_consider", 0.5,
    ParameterType::Static);
  getParam(
    critics.path_angle.offset_from_furthest, "PathAngleCritic.offset_from_furthest", 4,
    ParameterType::Static);
  getParam(
    critics.path_angle.max_angle_to_furthest, "PathAngleCritic.max_angle_to_furthest", 1.2,
    ParameterType::Static);
  getParam(
    critics.path_angle.forward_preference, "PathAngleCritic.forward_preference", true,
    ParameterType::Static);
  getParam(
    critics.path_follow.weight, "PathFollowCritic.cost_weight", 5.0, ParameterType::Static);
  getParam(
    critics.path_follow.threshold_to_consider, "PathFollowCritic.threshold_to_consider", 1.4,
    ParameterType::Static);
  getParam(
    critics.path_follow.offset_from_furthest, "PathFollowCritic.offset_from_furthest", 6,
    ParameterType::Static);
  getParam(
    critics.path_align.weight, "PathAlignCritic.cost_weight", 10.0, ParameterType::Static);
  getParam(
    critics.path_align.threshold_to_consider, "PathAlignCritic.threshold_to_consider", 0.5,
    ParameterType::Static);
  getParam(
    critics.path_align.offset_from_furthest, "PathAlignCritic.offset_from_furthest", 20,
    ParameterType::Static);
  getParam(
    critics.path_align.trajectory_point_step, "PathAlignCritic.trajectory_point_step", 4,
    ParameterType::Static);
  getParam(
    critics.path_align.max_path_occupancy_ratio, "PathAlignCritic.max_path_occupancy_ratio", 0.07,
    ParameterType::Static);
  getParam(
    critics.prefer_forward.weight, "PreferForwardCritic.cost_weight", 5.0,
    ParameterType::Static);
  getParam(
    critics.prefer_forward.threshold_to_consider,
    "PreferForwardCritic.threshold_to_consider", 0.5, ParameterType::Static);

  profiles_.fill(default_profile);

  getParam(profile_enabled_, "profile_enabled", true, ParameterType::Static);
  getParam(profile_start_distance_, "profile_start_distance", 0.8, ParameterType::Static);
  getParam(profile_goal_distance_, "profile_goal_distance", 1.0, ParameterType::Static);
  getParam(profile_transition_duration_, "profile_transition_duration", 0.0, ParameterType::Static);

  for (size_t i = 0; i < profiles_.size(); ++i) {
    auto & profile = profiles_[i];
    auto & profile_critics = profile.critics;
    const std::string prefix = "profile_" + std::to_string(i + 1) + ".";

    getParam(
      profile.velocity.vx_max, prefix + "vx_max",
      default_profile.velocity.vx_max, ParameterType::Static);
    getParam(
      profile.velocity.vx_min, prefix + "vx_min",
      default_profile.velocity.vx_min, ParameterType::Static);
    getParam(
      profile.velocity.vy_max, prefix + "vy_max",
      default_profile.velocity.vy_max, ParameterType::Static);
    getParam(
      profile.velocity.wz_max, prefix + "wz_max",
      default_profile.velocity.wz_max, ParameterType::Static);

    getParam(
      profile_critics.goal.weight, prefix + "GoalCritic.cost_weight",
      default_profile.critics.goal.weight, ParameterType::Static);
    getParam(
      profile_critics.goal.threshold_to_consider,
      prefix + "GoalCritic.threshold_to_consider",
      default_profile.critics.goal.threshold_to_consider, ParameterType::Static);
    getParam(
      profile_critics.goal_lateral.weight, prefix + "GoalLateralCritic.cost_weight",
      default_profile.critics.goal_lateral.weight, ParameterType::Static);
    getParam(
      profile_critics.goal_lateral.threshold_to_consider,
      prefix + "GoalLateralCritic.threshold_to_consider",
      default_profile.critics.goal_lateral.threshold_to_consider, ParameterType::Static);
    getParam(
      profile_critics.goal_lateral.last_n_points,
      prefix + "GoalLateralCritic.last_n_points",
      default_profile.critics.goal_lateral.last_n_points, ParameterType::Static);
    getParam(
      profile_critics.goal_longitudinal.weight,
      prefix + "GoalLongitudinalCritic.cost_weight",
      default_profile.critics.goal_longitudinal.weight, ParameterType::Static);
    getParam(
      profile_critics.goal_longitudinal.threshold_to_consider,
      prefix + "GoalLongitudinalCritic.threshold_to_consider",
      default_profile.critics.goal_longitudinal.threshold_to_consider, ParameterType::Static);
    getParam(
      profile_critics.goal_longitudinal.last_n_points,
      prefix + "GoalLongitudinalCritic.last_n_points",
      default_profile.critics.goal_longitudinal.last_n_points, ParameterType::Static);
    getParam(
      profile_critics.goal_angle.weight, prefix + "GoalAngleCritic.cost_weight",
      default_profile.critics.goal_angle.weight, ParameterType::Static);
    getParam(
      profile_critics.goal_angle.threshold_to_consider,
      prefix + "GoalAngleCritic.threshold_to_consider",
      default_profile.critics.goal_angle.threshold_to_consider, ParameterType::Static);
    getParam(
      profile_critics.path_angle.weight, prefix + "PathAngleCritic.cost_weight",
      default_profile.critics.path_angle.weight, ParameterType::Static);
    getParam(
      profile_critics.path_angle.threshold_to_consider,
      prefix + "PathAngleCritic.threshold_to_consider",
      default_profile.critics.path_angle.threshold_to_consider, ParameterType::Static);
    getParam(
      profile_critics.path_angle.offset_from_furthest,
      prefix + "PathAngleCritic.offset_from_furthest",
      default_profile.critics.path_angle.offset_from_furthest, ParameterType::Static);
    getParam(
      profile_critics.path_angle.max_angle_to_furthest,
      prefix + "PathAngleCritic.max_angle_to_furthest",
      default_profile.critics.path_angle.max_angle_to_furthest, ParameterType::Static);
    getParam(
      profile_critics.path_angle.forward_preference,
      prefix + "PathAngleCritic.forward_preference",
      default_profile.critics.path_angle.forward_preference, ParameterType::Static);
    getParam(
      profile_critics.path_follow.weight, prefix + "PathFollowCritic.cost_weight",
      default_profile.critics.path_follow.weight, ParameterType::Static);
    getParam(
      profile_critics.path_follow.threshold_to_consider,
      prefix + "PathFollowCritic.threshold_to_consider",
      default_profile.critics.path_follow.threshold_to_consider, ParameterType::Static);
    getParam(
      profile_critics.path_follow.offset_from_furthest,
      prefix + "PathFollowCritic.offset_from_furthest",
      default_profile.critics.path_follow.offset_from_furthest, ParameterType::Static);
    getParam(
      profile_critics.path_align.weight, prefix + "PathAlignCritic.cost_weight",
      default_profile.critics.path_align.weight, ParameterType::Static);
    getParam(
      profile_critics.path_align.threshold_to_consider,
      prefix + "PathAlignCritic.threshold_to_consider",
      default_profile.critics.path_align.threshold_to_consider, ParameterType::Static);
    getParam(
      profile_critics.path_align.offset_from_furthest,
      prefix + "PathAlignCritic.offset_from_furthest",
      default_profile.critics.path_align.offset_from_furthest, ParameterType::Static);
    getParam(
      profile_critics.path_align.trajectory_point_step,
      prefix + "PathAlignCritic.trajectory_point_step",
      default_profile.critics.path_align.trajectory_point_step, ParameterType::Static);
    getParam(
      profile_critics.path_align.max_path_occupancy_ratio,
      prefix + "PathAlignCritic.max_path_occupancy_ratio",
      default_profile.critics.path_align.max_path_occupancy_ratio, ParameterType::Static);
    getParam(
      profile_critics.prefer_forward.weight, prefix + "PreferForwardCritic.cost_weight",
      default_profile.critics.prefer_forward.weight, ParameterType::Static);
    getParam(
      profile_critics.prefer_forward.threshold_to_consider,
      prefix + "PreferForwardCritic.threshold_to_consider",
      default_profile.critics.prefer_forward.threshold_to_consider, ParameterType::Static);
  }

  const auto sameFloat = [](float lhs, float rhs) {
      constexpr float kEps = 1e-5f;
      return std::abs(lhs - rhs) < kEps;
    };
  const auto profileMatchesDefault = [&](const MppiProfileSettings & profile) {
      return
        sameFloat(profile.velocity.vx_max, default_profile.velocity.vx_max) &&
        sameFloat(profile.velocity.vx_min, default_profile.velocity.vx_min) &&
        sameFloat(profile.velocity.vy_max, default_profile.velocity.vy_max) &&
        sameFloat(profile.velocity.wz_max, default_profile.velocity.wz_max) &&
        sameFloat(profile.critics.goal.weight, default_profile.critics.goal.weight) &&
        sameFloat(
          profile.critics.goal.threshold_to_consider,
          default_profile.critics.goal.threshold_to_consider) &&
        sameFloat(
          profile.critics.goal_lateral.weight,
          default_profile.critics.goal_lateral.weight) &&
        sameFloat(
          profile.critics.goal_lateral.threshold_to_consider,
          default_profile.critics.goal_lateral.threshold_to_consider) &&
        profile.critics.goal_lateral.last_n_points ==
        default_profile.critics.goal_lateral.last_n_points &&
        sameFloat(
          profile.critics.goal_longitudinal.weight,
          default_profile.critics.goal_longitudinal.weight) &&
        sameFloat(
          profile.critics.goal_longitudinal.threshold_to_consider,
          default_profile.critics.goal_longitudinal.threshold_to_consider) &&
        profile.critics.goal_longitudinal.last_n_points ==
        default_profile.critics.goal_longitudinal.last_n_points &&
        sameFloat(profile.critics.goal_angle.weight, default_profile.critics.goal_angle.weight) &&
        sameFloat(
          profile.critics.goal_angle.threshold_to_consider,
          default_profile.critics.goal_angle.threshold_to_consider) &&
        sameFloat(profile.critics.path_angle.weight, default_profile.critics.path_angle.weight) &&
        sameFloat(
          profile.critics.path_angle.threshold_to_consider,
          default_profile.critics.path_angle.threshold_to_consider) &&
        profile.critics.path_angle.offset_from_furthest ==
        default_profile.critics.path_angle.offset_from_furthest &&
        sameFloat(
          profile.critics.path_angle.max_angle_to_furthest,
          default_profile.critics.path_angle.max_angle_to_furthest) &&
        profile.critics.path_angle.forward_preference ==
        default_profile.critics.path_angle.forward_preference &&
        sameFloat(profile.critics.path_follow.weight, default_profile.critics.path_follow.weight) &&
        sameFloat(
          profile.critics.path_follow.threshold_to_consider,
          default_profile.critics.path_follow.threshold_to_consider) &&
        profile.critics.path_follow.offset_from_furthest ==
        default_profile.critics.path_follow.offset_from_furthest &&
        sameFloat(profile.critics.path_align.weight, default_profile.critics.path_align.weight) &&
        sameFloat(
          profile.critics.path_align.threshold_to_consider,
          default_profile.critics.path_align.threshold_to_consider) &&
        profile.critics.path_align.offset_from_furthest ==
        default_profile.critics.path_align.offset_from_furthest &&
        profile.critics.path_align.trajectory_point_step ==
        default_profile.critics.path_align.trajectory_point_step &&
        sameFloat(
          profile.critics.path_align.max_path_occupancy_ratio,
          default_profile.critics.path_align.max_path_occupancy_ratio) &&
        sameFloat(
          profile.critics.prefer_forward.weight,
          default_profile.critics.prefer_forward.weight) &&
        sameFloat(
          profile.critics.prefer_forward.threshold_to_consider,
          default_profile.critics.prefer_forward.threshold_to_consider);
    };
  const bool all_profiles_match_default =
    profileMatchesDefault(profiles_[0]) &&
    profileMatchesDefault(profiles_[1]) &&
    profileMatchesDefault(profiles_[2]);

  RCLCPP_INFO(
    logger_,
    "MPPI profiles: enabled=%d start_distance=%.3f goal_distance=%.3f. "
    "transition_duration=%.3f. "
    "enabled=1 时启用三阶段切换；当前默认开启。path_progress 小于 start_distance 为起步段，"
    "goal_distance 小于等于 profile_goal_distance 为末段。",
    profile_enabled_, profile_start_distance_, profile_goal_distance_,
    profile_transition_duration_);

  if (profile_enabled_) {
    for (size_t i = 0; i < profiles_.size(); ++i) {
      const auto & profile = profiles_[i];
      RCLCPP_INFO(
        logger_,
        "MPPI profile_%zu velocity: vx=[%.3f, %.3f] vy=%.3f wz=%.3f. "
        "Goal(w=%.3f th=%.3f) GoalLateral(w=%.3f th=%.3f points=%d) "
        "GoalLongitudinal(w=%.3f th=%.3f points=%d) "
        "GoalAngle(w=%.3f th=%.3f) "
        "PathAngle(w=%.3f th=%.3f off=%d max_angle=%.3f forward=%d) "
        "PathAlign(w=%.3f th=%.3f off=%d step=%d occ=%.3f) "
        "PathFollow(w=%.3f th=%.3f off=%d) PreferForward(w=%.3f th=%.3f). "
        "这是该阶段会覆盖到 MPPI 的速度约束和关键 critic 参数。",
        i + 1, profile.velocity.vx_min, profile.velocity.vx_max,
        profile.velocity.vy_max, profile.velocity.wz_max,
        profile.critics.goal.weight, profile.critics.goal.threshold_to_consider,
        profile.critics.goal_lateral.weight,
        profile.critics.goal_lateral.threshold_to_consider,
        profile.critics.goal_lateral.last_n_points,
        profile.critics.goal_longitudinal.weight,
        profile.critics.goal_longitudinal.threshold_to_consider,
        profile.critics.goal_longitudinal.last_n_points,
        profile.critics.goal_angle.weight, profile.critics.goal_angle.threshold_to_consider,
        profile.critics.path_angle.weight, profile.critics.path_angle.threshold_to_consider,
        profile.critics.path_angle.offset_from_furthest,
        profile.critics.path_angle.max_angle_to_furthest,
        profile.critics.path_angle.forward_preference,
        profile.critics.path_align.weight, profile.critics.path_align.threshold_to_consider,
        profile.critics.path_align.offset_from_furthest,
        profile.critics.path_align.trajectory_point_step,
        profile.critics.path_align.max_path_occupancy_ratio,
        profile.critics.path_follow.weight, profile.critics.path_follow.threshold_to_consider,
        profile.critics.path_follow.offset_from_furthest,
        profile.critics.prefer_forward.weight,
        profile.critics.prefer_forward.threshold_to_consider);
    }

    if (all_profiles_match_default) {
      RCLCPP_WARN(
        logger_,
        "MPPI three-stage profile is enabled, but profile_1/profile_2/profile_3 are all equal to "
        "the global FollowPath parameters. 当前会打印阶段切换日志，但三个阶段的控制行为相同；"
        "如需测试差异化三段式效果，需要在 YAML 中补充 profile_1/profile_2/profile_3 覆盖参数。");
    }
  }

  applied_profile_ = profiles_[active_profile_];
  transition_start_profile_ = applied_profile_;
  transition_target_profile_ = applied_profile_;
}

void MPPIController::applyProfileForPathState(double path_progress, double goal_distance)
{
  if (!profile_enabled_) {
    return;
  }

  const size_t selected_profile = selectProfile(path_progress, goal_distance);
  const auto & selected = profiles_[selected_profile];
  RCLCPP_DEBUG(
    logger_,
    "MPPI profile state: active=%zu selected=%zu path_progress=%.3f goal_distance=%.3f. "
    "selected_reason=%s selected_vx=[%.3f, %.3f] selected_wz=%.3f. "
    "三阶段判断输入，若 active 与 selected 不同则即将切换。",
    active_profile_ + 1, selected_profile + 1, path_progress, goal_distance,
    getProfileReason(selected_profile).c_str(),
    selected.velocity.vx_min, selected.velocity.vx_max, selected.velocity.wz_max);

  if (selected_profile == active_profile_) {
    updateProfileTransition();
    return;
  }

  const size_t previous_profile = active_profile_;
  active_profile_ = selected_profile;
  if (profile_transition_duration_ > 0.0) {
    transition_start_profile_ = applied_profile_;
    transition_target_profile_ = profiles_[active_profile_];
    profile_transition_start_time_ = clock_->now();
    profile_transition_active_ = true;
    updateProfileTransition();
  } else {
    applied_profile_ = profiles_[active_profile_];
    optimizer_.applyProfile(applied_profile_);
  }

  const auto & profile = profiles_[active_profile_];
  RCLCPP_INFO(
    logger_,
    "MPPI profile switched: %zu -> %zu, reason=%s, path_progress=%.3f, "
    "goal_distance=%.3f, velocity vx=[%.3f, %.3f] vy=%.3f wz=%.3f. "
    "transition_duration=%.3f. "
    "三阶段发生切换，新的速度约束和 critic 参数会按过渡时间应用到 Optimizer。",
    previous_profile + 1, active_profile_ + 1, getProfileReason(active_profile_).c_str(),
    path_progress, goal_distance, profile.velocity.vx_min, profile.velocity.vx_max,
    profile.velocity.vy_max, profile.velocity.wz_max, profile_transition_duration_);
}

size_t MPPIController::selectProfile(double path_progress, double goal_distance) const
{
  if (goal_distance <= profile_goal_distance_) {
    return 2;
  }

  if (path_progress < profile_start_distance_) {
    return 0;
  }

  return 1;
}

std::string MPPIController::getProfileReason(size_t profile) const
{
  switch (profile) {
    case 0:
      return "start_segment/起步段";
    case 1:
      return "middle_segment/中间段";
    case 2:
      return "goal_segment/末段";
    default:
      return "unknown";
  }
}

MppiProfileSettings MPPIController::interpolateProfiles(
  const MppiProfileSettings & from,
  const MppiProfileSettings & to,
  double ratio) const
{
  ratio = std::max(0.0, std::min(1.0, ratio));

  const auto lerpFloat = [ratio](float lhs, float rhs) {
      return static_cast<float>(lhs + (rhs - lhs) * ratio);
    };
  const auto lerpInt = [ratio](int lhs, int rhs) {
      return static_cast<int>(std::lround(lhs + (rhs - lhs) * ratio));
    };

  MppiProfileSettings result = to;
  result.velocity.vx_max = lerpFloat(from.velocity.vx_max, to.velocity.vx_max);
  result.velocity.vx_min = lerpFloat(from.velocity.vx_min, to.velocity.vx_min);
  result.velocity.vy_max = lerpFloat(from.velocity.vy_max, to.velocity.vy_max);
  result.velocity.wz_max = lerpFloat(from.velocity.wz_max, to.velocity.wz_max);

  result.critics.goal.weight =
    lerpFloat(from.critics.goal.weight, to.critics.goal.weight);
  result.critics.goal.threshold_to_consider =
    lerpFloat(from.critics.goal.threshold_to_consider, to.critics.goal.threshold_to_consider);

  result.critics.goal_lateral.weight =
    lerpFloat(from.critics.goal_lateral.weight, to.critics.goal_lateral.weight);
  result.critics.goal_lateral.threshold_to_consider = lerpFloat(
    from.critics.goal_lateral.threshold_to_consider,
    to.critics.goal_lateral.threshold_to_consider);
  result.critics.goal_lateral.last_n_points = std::max(
    1, lerpInt(
      from.critics.goal_lateral.last_n_points,
      to.critics.goal_lateral.last_n_points));

  result.critics.goal_longitudinal.weight =
    lerpFloat(from.critics.goal_longitudinal.weight, to.critics.goal_longitudinal.weight);
  result.critics.goal_longitudinal.threshold_to_consider = lerpFloat(
    from.critics.goal_longitudinal.threshold_to_consider,
    to.critics.goal_longitudinal.threshold_to_consider);
  result.critics.goal_longitudinal.last_n_points = std::max(
    1, lerpInt(
      from.critics.goal_longitudinal.last_n_points,
      to.critics.goal_longitudinal.last_n_points));

  result.critics.goal_angle.weight =
    lerpFloat(from.critics.goal_angle.weight, to.critics.goal_angle.weight);
  result.critics.goal_angle.threshold_to_consider = lerpFloat(
    from.critics.goal_angle.threshold_to_consider,
    to.critics.goal_angle.threshold_to_consider);

  result.critics.path_angle.weight =
    lerpFloat(from.critics.path_angle.weight, to.critics.path_angle.weight);
  result.critics.path_angle.threshold_to_consider = lerpFloat(
    from.critics.path_angle.threshold_to_consider,
    to.critics.path_angle.threshold_to_consider);
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
    lerpFloat(from.critics.path_follow.weight, to.critics.path_follow.weight);
  result.critics.path_follow.threshold_to_consider = lerpFloat(
    from.critics.path_follow.threshold_to_consider,
    to.critics.path_follow.threshold_to_consider);
  result.critics.path_follow.offset_from_furthest = std::max(
    0, lerpInt(
      from.critics.path_follow.offset_from_furthest,
      to.critics.path_follow.offset_from_furthest));

  result.critics.path_align.weight =
    lerpFloat(from.critics.path_align.weight, to.critics.path_align.weight);
  result.critics.path_align.threshold_to_consider = lerpFloat(
    from.critics.path_align.threshold_to_consider,
    to.critics.path_align.threshold_to_consider);
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
    lerpFloat(from.critics.prefer_forward.weight, to.critics.prefer_forward.weight);
  result.critics.prefer_forward.threshold_to_consider = lerpFloat(
    from.critics.prefer_forward.threshold_to_consider,
    to.critics.prefer_forward.threshold_to_consider);

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

  optimizer_.applyProfile(applied_profile_);
}

void MPPIController::visualize(nav_msgs::msg::Path transformed_plan)
{
  trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(), "Candidate Trajectories");
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
