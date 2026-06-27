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
  configureStages();

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
  const double heading_path_angle = path_handler_.getLastHeadingPathAngle();
  applyProfileForPathState(path_progress, goal_distance, heading_path_angle);
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
      "total=%.3f. 控制周期耗时过长，可根据 transform/锁等待/优化/可视化字段定位卡顿环节。",
      transform_ms, wait_lock_ms, optimize_ms, visualize_ms, total_ms);
  }

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
  };
  profile_debug_publisher_->publish(std::move(msg));
}

void MPPIController::configureStages()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  // 单阶段 MPPI 已删除：行为类 Goal/Path/PreferForward 代价只允许由 profile_1/2/3 管理。
  // 这里的默认值只作为缺省兜底，不再从 FollowPath 全局行为 critic 中继承。
  stages_[0].velocity = {0.4f, -0.2f, 0.5f, 0.9f};
  stages_[0].critics.goal = {0.0f, 0.0f};
  stages_[0].critics.goal_lateral = {0.0f, 0.0f, 8};
  stages_[0].critics.goal_longitudinal = {0.0f, 0.0f, 8};
  stages_[0].critics.goal_angle = {0.0f, 0.0f};
  stages_[0].critics.path_angle = {7.0f, 0.3f, 3, 0.7f, true};
  stages_[0].critics.path_align = {8.0f, 0.3f, 8, 3, 0.08f};
  stages_[0].critics.path_follow = {7.0f, 1.2f, 7};
  stages_[0].critics.prefer_forward = {6.0f, 0.25f};
  stages_[0].critics.desired_velocity = {0.0f};   // 对齐段关闭速度软约束（不惩罚后退机动）
  stages_[0].critics.obstacles = {1.1f};
  stages_[0].critics.cost = {3.81f};

  stages_[1].velocity = {0.7f, -0.5f, 0.5f, 0.75f};
  stages_[1].critics.goal = {0.0f, 0.0f};
  stages_[1].critics.goal_lateral = {0.0f, 0.0f, 8};
  stages_[1].critics.goal_longitudinal = {0.0f, 0.0f, 8};
  stages_[1].critics.goal_angle = {0.0f, 0.0f};
  stages_[1].critics.path_angle = {5.0f, 0.5f, 3, 0.9f, true};
  stages_[1].critics.path_align = {20.0f, 0.5f, 15, 2, 0.05f};
  stages_[1].critics.path_follow = {4.0f, 0.5f, 5};
  stages_[1].critics.prefer_forward = {5.0f, 0.25f};
  stages_[1].critics.desired_velocity = {8.0f};
  stages_[1].critics.obstacles = {1.1f};
  stages_[1].critics.cost = {3.81f};

  stages_[2].velocity = {0.7f, 0.0f, 0.5f, 0.30f};  // 末段速度交给 DesiredVelocityCritic 软约束；硬上限保持 0.7、禁倒车
  stages_[2].critics.goal = {4.0f, 1.0f};
  stages_[2].critics.goal_lateral = {40.0f, 1.0f, 10};
  stages_[2].critics.goal_longitudinal = {25.0f, 0.8f, 10};
  stages_[2].critics.goal_angle = {8.0f, 0.2f};
  stages_[2].critics.path_angle = {1.0f, 0.5f, 2, 1.0f, true};
  stages_[2].critics.path_align = {2.0f, 0.5f, 15, 3, 0.05f};
  stages_[2].critics.path_follow = {1.0f, 0.5f, 5};
  stages_[2].critics.prefer_forward = {1.0f, 1.0f};
  stages_[2].critics.desired_velocity = {8.0f};
  stages_[2].critics.obstacles = {0.0f};   // 趋近段软斥力关（保致命）
  stages_[2].critics.cost = {0.0f};

  // profile_start_distance 已废弃（对齐段改按角度切，不再按距起点距离）；成员保留仅供 debug 兼容
  getParam(approach_distance_, "approach_distance", 1.0, ParameterType::Static);
  getParam(profile_transition_duration_, "profile_transition_duration", 1.0, ParameterType::Static);
  getParam(approach_hysteresis_, "approach_hysteresis", 0.05, ParameterType::Static);
  approach_hysteresis_ = std::max(0.0, approach_hysteresis_);
  getParam(align_enter_angle_, "align_enter_angle", 0.52, ParameterType::Static);  // ~30°
  getParam(align_exit_angle_, "align_exit_angle", 0.31, ParameterType::Static);    // ~18° 迟滞
  align_exit_angle_ = std::min(align_exit_angle_, align_enter_angle_);

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
      profile_critics.goal.threshold_to_consider,
      prefix + "GoalCritic.threshold_to_consider",
      profile_critics.goal.threshold_to_consider, ParameterType::Static);
    getParam(
      profile_critics.goal_lateral.weight, prefix + "GoalLateralCritic.cost_weight",
      profile_critics.goal_lateral.weight, ParameterType::Static);
    getParam(
      profile_critics.goal_lateral.threshold_to_consider,
      prefix + "GoalLateralCritic.threshold_to_consider",
      profile_critics.goal_lateral.threshold_to_consider, ParameterType::Static);
    getParam(
      profile_critics.goal_lateral.last_n_points,
      prefix + "GoalLateralCritic.last_n_points",
      profile_critics.goal_lateral.last_n_points, ParameterType::Static);
    getParam(
      profile_critics.goal_longitudinal.weight,
      prefix + "GoalLongitudinalCritic.cost_weight",
      profile_critics.goal_longitudinal.weight, ParameterType::Static);
    getParam(
      profile_critics.goal_longitudinal.threshold_to_consider,
      prefix + "GoalLongitudinalCritic.threshold_to_consider",
      profile_critics.goal_longitudinal.threshold_to_consider, ParameterType::Static);
    getParam(
      profile_critics.goal_longitudinal.last_n_points,
      prefix + "GoalLongitudinalCritic.last_n_points",
      profile_critics.goal_longitudinal.last_n_points, ParameterType::Static);
    getParam(
      profile_critics.goal_angle.weight, prefix + "GoalAngleCritic.cost_weight",
      profile_critics.goal_angle.weight, ParameterType::Static);
    getParam(
      profile_critics.goal_angle.threshold_to_consider,
      prefix + "GoalAngleCritic.threshold_to_consider",
      profile_critics.goal_angle.threshold_to_consider, ParameterType::Static);
    getParam(
      profile_critics.path_angle.weight, prefix + "PathAngleCritic.cost_weight",
      profile_critics.path_angle.weight, ParameterType::Static);
    getParam(
      profile_critics.path_angle.threshold_to_consider,
      prefix + "PathAngleCritic.threshold_to_consider",
      profile_critics.path_angle.threshold_to_consider, ParameterType::Static);
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
      profile_critics.path_follow.threshold_to_consider,
      prefix + "PathFollowCritic.threshold_to_consider",
      profile_critics.path_follow.threshold_to_consider, ParameterType::Static);
    getParam(
      profile_critics.path_follow.offset_from_furthest,
      prefix + "PathFollowCritic.offset_from_furthest",
      profile_critics.path_follow.offset_from_furthest, ParameterType::Static);
    getParam(
      profile_critics.path_align.weight, prefix + "PathAlignCritic.cost_weight",
      profile_critics.path_align.weight, ParameterType::Static);
    getParam(
      profile_critics.path_align.threshold_to_consider,
      prefix + "PathAlignCritic.threshold_to_consider",
      profile_critics.path_align.threshold_to_consider, ParameterType::Static);
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
      profile_critics.prefer_forward.threshold_to_consider,
      prefix + "PreferForwardCritic.threshold_to_consider",
      profile_critics.prefer_forward.threshold_to_consider, ParameterType::Static);
    getParam(
      profile_critics.desired_velocity.weight,
      prefix + "DesiredVelocityCritic.cost_weight",
      profile_critics.desired_velocity.weight, ParameterType::Static);
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
          logger_, "stage_weights.%s 需 3 个值[align,track,approach]，已忽略", critic.c_str());
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
    "DesiredVelocityCritic", stages_[0].critics.desired_velocity.weight,
    stages_[1].critics.desired_velocity.weight, stages_[2].critics.desired_velocity.weight);
  loadWeightRow(
    "ObstaclesCritic", stages_[0].critics.obstacles.repulsion_weight,
    stages_[1].critics.obstacles.repulsion_weight, stages_[2].critics.obstacles.repulsion_weight);
  loadWeightRow(
    "CostCritic", stages_[0].critics.cost.weight,
    stages_[1].critics.cost.weight, stages_[2].critics.cost.weight);

  RCLCPP_INFO(
    logger_,
    "MPPI three-stage profiles: start_distance=%.3f goal_distance=%.3f. "
    "transition_duration=%.3f switch_hysteresis=%.3f. "
    "单阶段 MPPI 已删除：profile_1/2/3 分别完整管理行为代价。"
    "path_progress 小于 start_distance 为起步段，goal_distance 小于等于 "
    "profile_goal_distance 为末段；迟滞带用于避免边界附近反复切换。",
    profile_start_distance_, approach_distance_,
    profile_transition_duration_, approach_hysteresis_);

  for (size_t i = 0; i < stages_.size(); ++i) {
    const auto & profile = stages_[i];
    RCLCPP_INFO(
      logger_,
      "MPPI profile_%zu velocity: vx=[%.3f, %.3f] vy=%.3f wz=%.3f. "
      "Goal(w=%.3f th=%.3f) GoalLateral(w=%.3f th=%.3f points=%d) "
      "GoalLongitudinal(w=%.3f th=%.3f points=%d) "
      "GoalAngle(w=%.3f th=%.3f) "
      "PathAngle(w=%.3f th=%.3f off=%d max_angle=%.3f forward=%d) "
      "PathAlign(w=%.3f th=%.3f off=%d step=%d occ=%.3f) "
      "PathFollow(w=%.3f th=%.3f off=%d) PreferForward(w=%.3f th=%.3f). "
      "这是该阶段完整接管的速度约束和行为类 critic 参数；weight=0 表示本阶段关闭该代价。",
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

  applied_profile_ = stages_[active_stage_];
  transition_start_profile_ = applied_profile_;
  transition_target_profile_ = applied_profile_;
}

void MPPIController::applyProfileForPathState(
  double path_progress, double goal_distance, double heading_path_angle)
{
  const size_t selected_profile = selectStage(goal_distance, heading_path_angle);
  const auto & selected = stages_[selected_profile];
  RCLCPP_DEBUG(
    logger_,
    "MPPI profile state: active=%zu selected=%zu path_progress=%.3f goal_distance=%.3f heading_angle=%.3f. "
    "selected_reason=%s selected_vx=[%.3f, %.3f] selected_wz=%.3f. "
    "三阶段判断输入，若 active 与 selected 不同则即将切换。",
    active_stage_ + 1, selected_profile + 1, path_progress, goal_distance, heading_path_angle,
    stageName(selected_profile).c_str(),
    selected.velocity.vx_min, selected.velocity.vx_max, selected.velocity.wz_max);

  if (selected_profile == active_stage_) {
    updateProfileTransition();
    return;
  }

  const size_t previous_profile = active_stage_;
  active_stage_ = selected_profile;
  if (profile_transition_duration_ > 0.0) {
    transition_start_profile_ = applied_profile_;
    transition_target_profile_ = stages_[active_stage_];
    profile_transition_start_time_ = clock_->now();
    profile_transition_active_ = true;
    updateProfileTransition();
  } else {
    applied_profile_ = stages_[active_stage_];
    optimizer_.applyProfile(applied_profile_);
  }

  const auto & profile = stages_[active_stage_];
  RCLCPP_INFO(
    logger_,
    "MPPI profile switched: %zu -> %zu, reason=%s, path_progress=%.3f, "
    "goal_distance=%.3f, velocity vx=[%.3f, %.3f] vy=%.3f wz=%.3f. "
    "transition_duration=%.3f. "
    "三阶段发生切换，新的速度约束和 critic 参数会按过渡时间应用到 Optimizer。",
    previous_profile + 1, active_stage_ + 1, stageName(active_stage_).c_str(),
    path_progress, goal_distance, profile.velocity.vx_min, profile.velocity.vx_max,
    profile.velocity.vy_max, profile.velocity.wz_max, profile_transition_duration_);
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
      return "align/对齐段";
    case 1:
      return "track/跟踪段";
    case 2:
      return "approach/趋近段";
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
  const auto keepGoalGateOpen = [](float lhs, float rhs) {
      // 目标类 critic: threshold 是启用范围，先打开门；实际牵引强度由 goal_ratio 慢启动。
      return std::max(lhs, rhs);
    };
  const auto keepPathGateOpen = [](float lhs, float rhs) {
      // 路径类/前进偏好 critic: 距离小于 threshold 时关闭，取较小值可避免切换边界过早退出。
      return std::min(lhs, rhs);
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
  result.critics.goal.threshold_to_consider =
    keepGoalGateOpen(from.critics.goal.threshold_to_consider, to.critics.goal.threshold_to_consider);

  result.critics.goal_lateral.weight =
    lerpFloatWithRatio(
    from.critics.goal_lateral.weight, to.critics.goal_lateral.weight, goal_ratio);
  result.critics.goal_lateral.threshold_to_consider = keepGoalGateOpen(
    from.critics.goal_lateral.threshold_to_consider,
    to.critics.goal_lateral.threshold_to_consider);
  result.critics.goal_lateral.last_n_points = std::max(
    1, lerpInt(
      from.critics.goal_lateral.last_n_points,
      to.critics.goal_lateral.last_n_points));

  result.critics.goal_longitudinal.weight =
    lerpFloatWithRatio(
    from.critics.goal_longitudinal.weight, to.critics.goal_longitudinal.weight, goal_ratio);
  result.critics.goal_longitudinal.threshold_to_consider = keepGoalGateOpen(
    from.critics.goal_longitudinal.threshold_to_consider,
    to.critics.goal_longitudinal.threshold_to_consider);
  result.critics.goal_longitudinal.last_n_points = std::max(
    1, lerpInt(
      from.critics.goal_longitudinal.last_n_points,
      to.critics.goal_longitudinal.last_n_points));

  result.critics.goal_angle.weight =
    lerpFloatWithRatio(
    from.critics.goal_angle.weight, to.critics.goal_angle.weight, goal_ratio);
  result.critics.goal_angle.threshold_to_consider = keepGoalGateOpen(
    from.critics.goal_angle.threshold_to_consider,
    to.critics.goal_angle.threshold_to_consider);

  result.critics.path_angle.weight =
    lerpFloatWithRatio(
    from.critics.path_angle.weight, to.critics.path_angle.weight, path_ratio);
  result.critics.path_angle.threshold_to_consider = keepPathGateOpen(
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
    lerpFloatWithRatio(
    from.critics.path_follow.weight, to.critics.path_follow.weight, path_ratio);
  result.critics.path_follow.threshold_to_consider = keepPathGateOpen(
    from.critics.path_follow.threshold_to_consider,
    to.critics.path_follow.threshold_to_consider);
  result.critics.path_follow.offset_from_furthest = std::max(
    0, lerpInt(
      from.critics.path_follow.offset_from_furthest,
      to.critics.path_follow.offset_from_furthest));

  result.critics.path_align.weight =
    lerpFloatWithRatio(
    from.critics.path_align.weight, to.critics.path_align.weight, path_ratio);
  result.critics.path_align.threshold_to_consider = keepPathGateOpen(
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
    lerpFloatWithRatio(
    from.critics.prefer_forward.weight, to.critics.prefer_forward.weight, path_ratio);
  result.critics.prefer_forward.threshold_to_consider = keepPathGateOpen(
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
