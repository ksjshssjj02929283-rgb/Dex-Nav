// Copyright (c) 2019 Intel Corporation
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

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "angles/angles.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_controller/controller_server.hpp"
#include "tf2/exceptions.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{

namespace
{

constexpr int kEndLocalizationWarningThrottleMs = 1000;

const char * speedPlanningTerminalBranchName(
  nav2_core::SpeedPlanningTerminalBranch branch)
{
  switch (branch) {
    case nav2_core::SpeedPlanningTerminalBranch::PATH_OBSTACLE_STOP:
      return "obstacle";
    case nav2_core::SpeedPlanningTerminalBranch::GOAL_TERMINAL:
      return "terminal";
    case nav2_core::SpeedPlanningTerminalBranch::NONE:
    default:
      return "none";
  }
}

}  // namespace

ControllerServer::ControllerServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("controller_server", "", options),
  progress_checker_loader_("nav2_core", "nav2_core::ProgressChecker"),
  default_progress_checker_id_{"progress_checker"},
  default_progress_checker_type_{"nav2_controller::SimpleProgressChecker"},
  goal_checker_loader_("nav2_core", "nav2_core::GoalChecker"),
  default_goal_checker_ids_{
    "simple_goal_checker",
    "stopped_goal_checker",
    "position_goal_checker",
    "rotation_goal_checker",
    "longitudinal_goal_checker",
    "high_precision_goal_checker"},
  default_goal_checker_types_{
    "nav2_simple_goal_checker::SimpleGoalChecker",
    "nav2_stopped_goal_checker::StoppedGoalChecker",
    "nav2_position_goal_checker::PositionGoalChecker",
    "nav2_rotation_goal_checker::RotationGoalChecker",
    "nav2_longitudinal_goal_checker::LongitudinalGoalChecker",
    "nav2_high_precision_goal_checker::HighPrecisionGoalChecker"},
  lp_loader_("nav2_core", "nav2_core::Controller"),
  default_ids_{
    "MPPIController",
    "DWBController",
    "RPPController",
    "GracefulController",
    "RotationShimController",
    "PIDController",
    "LatLonController",
    "PureRotationController",
    "StraightController"},
  default_types_{
    "nav2_mppi_controller::MPPIController",
    "dwb_core::DWBLocalPlanner",
    "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    "nav2_graceful_controller::GracefulController",
    "nav2_rotation_shim_controller::RotationShimController",
    "nav2_pid_controller::PIDController",
    "nav2_lat_lon_controller::LatLonController",
    "nav2_pure_rotation_controller::PureRotationController",
    "nav2_straight_controller::StraightController"},
  local_planner_loader_("nav2_core", "nav2_core::LocalPlanner"),
  speed_planner_loader_("nav2_core", "nav2_core::SpeedPlanner")
{
  RCLCPP_INFO(get_logger(), "Controller server created.");

  declare_parameter("controller_frequency", 20.0);

  declare_parameter("progress_checker_plugin", default_progress_checker_id_);
  declare_parameter("goal_checker_plugins", default_goal_checker_ids_);
  declare_parameter("controller_plugins", default_ids_);
  declare_parameter("current_goal_checker", default_goal_checker_ids_.front());
  declare_parameter("current_controller", default_ids_.front());
  // ③ 局部规划：默认直通 global path，YAML 显式配置后每控制周期生效
  declare_parameter("local_planner_plugins", std::vector<std::string>());
  declare_parameter("current_local_planner", std::string(""));
  // ④ 速度规划：默认不启用（空列表 + 空选择器），YAML 显式配置后生效
  declare_parameter("speed_planner_plugins", std::vector<std::string>());
  declare_parameter("current_speed_planner", std::string(""));
  declare_parameter("initial_motion.max_age", 0.5);
  declare_parameter("initial_motion.max_spatial_error", 0.15);
  // P1-2（重构方案 §2.3）：|实测-计划采样速度| 在此内继承计划速度，
  // 否则重锚定到实测但仍保留加速度；旧名 max_velocity_error 的语义是
  // "超出即整体拒绝"，已改行为，改名避免和旧默认值（0.25）混淆。
  declare_parameter("initial_motion.replan_dev", 0.10);
  declare_parameter("initial_motion.terminal_overspeed_replan_dev", 0.02);
  declare_parameter("initial_motion.failsafe_deviation_cycles", 5);
  declare_parameter("initial_motion.goal_position_tolerance", 0.05);
  declare_parameter("initial_motion.goal_yaw_tolerance", 0.2);
  declare_parameter("initial_motion.branch_heading_tolerance", 0.7853981634);
  declare_parameter("initial_motion.stitching.horizon", 0.20);
  declare_parameter("initial_motion.stitching.min_duration", 0.10);
  declare_parameter("initial_motion.stitching.max_path_error", 0.05);
  declare_parameter("initial_motion.stitching.max_time_error", 0.15);
  declare_parameter(
    "initial_motion.longitudinal_state_topic", longitudinal_state_topic_);
  declare_parameter("initial_motion.longitudinal_state_max_age", 0.10);
  declare_parameter("initial_motion.state_limits.min_acceleration", -0.40);
  declare_parameter("initial_motion.state_limits.max_acceleration", 0.50);
  declare_parameter("initial_motion.planned_correction.enabled", true);
  declare_parameter("initial_motion.planned_correction.gain", 0.40);
  declare_parameter("initial_motion.planned_correction.innovation_limit", 0.30);
  declare_parameter("initial_motion.planned_correction.max_jerk", 0.30);
  declare_parameter("initial_motion.terminal_stall_recovery.enabled", true);
  declare_parameter("initial_motion.terminal_stall_recovery.duration", 0.5);
  declare_parameter("initial_motion.terminal_stall_recovery.stopped_velocity", 0.01);
  declare_parameter("initial_motion.terminal_stall_recovery.min_target_velocity", 0.03);
  declare_parameter("initial_motion.terminal_stall_recovery.max_target_acceleration", -0.03);
  declare_parameter("initial_motion.terminal_stall_recovery.max_progress_per_cycle", 0.005);
  declare_parameter("initial_motion.terminal_stall_recovery.min_remaining_distance", 0.05);
  // nav_monitor 任务回显：默认关闭，零开销
  declare_parameter("enable_nav_monitor_echo", false);
  declare_parameter("min_x_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_y_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_theta_velocity_threshold", rclcpp::ParameterValue(0.0001));

  declare_parameter("speed_limit_topic", rclcpp::ParameterValue("speed_limit"));
  declare_parameter("hard_stop_topic", rclcpp::ParameterValue("cmd_vel_hard_stop"));

  declare_parameter("failure_tolerance", rclcpp::ParameterValue(0.0));
  declare_parameter("staged_control.enabled", rclcpp::ParameterValue(true));
  declare_parameter("staged_control.track_controller", rclcpp::ParameterValue("MPPIController"));
  declare_parameter("staged_control.approach_controller", rclcpp::ParameterValue("PIDController"));
  declare_parameter("staged_control.approach_distance", rclcpp::ParameterValue(0.3));
  declare_parameter(
    "staged_control.align_heading_tolerance", rclcpp::ParameterValue(0.5235987756));
  declare_parameter("staged_control.align_angular_kp", rclcpp::ParameterValue(1.5));
  declare_parameter("staged_control.align_max_angular_velocity", rclcpp::ParameterValue(0.5));
  declare_parameter("staged_control.align_min_angular_velocity", rclcpp::ParameterValue(0.08));
  declare_parameter("staged_control.align_target_distance", rclcpp::ParameterValue(0.3));
  declare_parameter("staged_control.prepare.enabled", rclcpp::ParameterValue(true));
  declare_parameter(
    "staged_control.prepare.obstacle_check_distance", rclcpp::ParameterValue(0.2));
  declare_parameter(
    "staged_control.prepare.side_check_distance", rclcpp::ParameterValue(0.15));
  declare_parameter(
    "staged_control.prepare.obstacle_cost_threshold", rclcpp::ParameterValue(253));
  declare_parameter("staged_control.prepare.backup_speed", rclcpp::ParameterValue(-0.08));
  declare_parameter("staged_control.prepare.backup_duration", rclcpp::ParameterValue(0.6));
  declare_parameter(
    "staged_control.prepare.backup_use_distance", rclcpp::ParameterValue(false));
  declare_parameter("staged_control.prepare.backup_distance", rclcpp::ParameterValue(0.3));

  // The costmap node is used in the implementation of the controller
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "local_costmap", std::string{get_namespace()}, "local_costmap");
}

ControllerServer::~ControllerServer()
{
  progress_checker_.reset();
  goal_checkers_.clear();
  controllers_.clear();
  local_planners_.clear();
  costmap_thread_.reset();
}

nav2_util::CallbackReturn
ControllerServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  auto node = shared_from_this();

  RCLCPP_INFO(get_logger(), "Configuring controller server.");

  get_parameter("progress_checker_plugin", progress_checker_id_);
  if (progress_checker_id_ == default_progress_checker_id_) {
    nav2_util::declare_parameter_if_not_declared(
      node, default_progress_checker_id_ + ".plugin",
      rclcpp::ParameterValue(default_progress_checker_type_));
  }

  get_parameter("goal_checker_plugins", goal_checker_ids_);
  if (goal_checker_ids_ == default_goal_checker_ids_) {
    for (size_t i = 0; i < default_goal_checker_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_goal_checker_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_goal_checker_types_[i]));
    }
  }

  get_parameter("controller_plugins", controller_ids_);
  if (controller_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_types_[i]));
    }
    nav2_util::declare_parameter_if_not_declared(
      node, std::string("DWB.critics"),
      rclcpp::ParameterValue(
        std::vector<std::string>{
          "RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign",
          "PathAlign", "PathDist", "GoalDist"}));
    nav2_util::declare_parameter_if_not_declared(
      node, std::string("MPPI.critics"),
      rclcpp::ParameterValue(
        std::vector<std::string>{
          "ConstraintCritic", "CostCritic", "ObstaclesCritic", "GoalCritic",
          "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic", "PathAngleCritic",
          "PreferForwardCritic", "VelocityDeadbandCritic"}));
    nav2_util::declare_parameter_if_not_declared(
      node, std::string("RotationShim.primary_controller"),
      rclcpp::ParameterValue(
        std::string(
          "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController")));
  }

  controller_types_.resize(controller_ids_.size());
  goal_checker_types_.resize(goal_checker_ids_.size());

  get_parameter("controller_frequency", controller_frequency_);
  get_parameter("min_x_velocity_threshold", min_x_velocity_threshold_);
  get_parameter("min_y_velocity_threshold", min_y_velocity_threshold_);
  get_parameter("min_theta_velocity_threshold", min_theta_velocity_threshold_);
  RCLCPP_INFO(get_logger(), "Controller frequency: %.4f Hz.", controller_frequency_);

  std::string speed_limit_topic;
  get_parameter("speed_limit_topic", speed_limit_topic);
  get_parameter("failure_tolerance", failure_tolerance_);
  nav2_speed_planner::InitialMotionArbiter::Config speed_plan_state_config;
  get_parameter("initial_motion.max_age", speed_plan_state_config.max_age);
  get_parameter(
    "initial_motion.max_spatial_error",
    speed_plan_state_config.max_spatial_error);
  get_parameter(
    "initial_motion.replan_dev",
    speed_plan_state_config.replan_dev);
  get_parameter(
    "initial_motion.terminal_overspeed_replan_dev",
    speed_plan_state_config.terminal_overspeed_replan_dev);
  get_parameter(
    "initial_motion.failsafe_deviation_cycles",
    speed_plan_state_config.failsafe_deviation_cycles);
  get_parameter(
    "initial_motion.goal_position_tolerance",
    speed_plan_state_config.goal_position_tolerance);
  get_parameter(
    "initial_motion.goal_yaw_tolerance",
    speed_plan_state_config.goal_yaw_tolerance);
  get_parameter(
    "initial_motion.branch_heading_tolerance",
    speed_plan_state_config.branch_heading_tolerance);
  get_parameter(
    "initial_motion.stitching.horizon",
    speed_plan_state_config.stitching_horizon);
  get_parameter(
    "initial_motion.stitching.min_duration",
    speed_plan_state_config.stitching_min_duration);
  get_parameter(
    "initial_motion.stitching.max_time_error",
    speed_plan_state_config.stitching_max_time_error);
  get_parameter(
    "initial_motion.stitching.max_path_error",
    trajectory_stitching_max_path_error_);
  trajectory_stitching_max_path_error_ = std::max(
    0.0, trajectory_stitching_max_path_error_);
  initial_motion_arbiter_.configure(speed_plan_state_config);
  get_parameter("initial_motion.longitudinal_state_topic", longitudinal_state_topic_);
  get_parameter("initial_motion.longitudinal_state_max_age", longitudinal_state_max_age_);
  get_parameter("initial_motion.state_limits.min_acceleration", state_acceleration_min_);
  get_parameter("initial_motion.state_limits.max_acceleration", state_acceleration_max_);
  get_parameter(
    "initial_motion.planned_correction.enabled", planned_acceleration_correction_enabled_);
  get_parameter(
    "initial_motion.planned_correction.gain", planned_acceleration_correction_gain_);
  get_parameter(
    "initial_motion.planned_correction.innovation_limit",
    planned_acceleration_innovation_limit_);
  get_parameter(
    "initial_motion.planned_correction.max_jerk",
    planned_acceleration_correction_max_jerk_);
  get_parameter(
    "initial_motion.terminal_stall_recovery.enabled",
    terminal_stall_recovery_enabled_);
  get_parameter(
    "initial_motion.terminal_stall_recovery.duration",
    terminal_stall_recovery_duration_);
  get_parameter(
    "initial_motion.terminal_stall_recovery.stopped_velocity",
    terminal_stall_stopped_velocity_);
  get_parameter(
    "initial_motion.terminal_stall_recovery.min_target_velocity",
    terminal_stall_min_target_velocity_);
  get_parameter(
    "initial_motion.terminal_stall_recovery.max_target_acceleration",
    terminal_stall_max_target_acceleration_);
  get_parameter(
    "initial_motion.terminal_stall_recovery.max_progress_per_cycle",
    terminal_stall_max_progress_per_cycle_);
  get_parameter(
    "initial_motion.terminal_stall_recovery.min_remaining_distance",
    terminal_stall_min_remaining_distance_);
  terminal_stall_recovery_duration_ = std::max(0.0, terminal_stall_recovery_duration_);
  terminal_stall_stopped_velocity_ = std::max(0.0, terminal_stall_stopped_velocity_);
  terminal_stall_min_target_velocity_ = std::max(0.0, terminal_stall_min_target_velocity_);
  terminal_stall_max_progress_per_cycle_ = std::max(
    0.0, terminal_stall_max_progress_per_cycle_);
  terminal_stall_min_remaining_distance_ = std::max(
    0.0, terminal_stall_min_remaining_distance_);
  get_parameter("staged_control.enabled", staged_control_enabled_);
  get_parameter("staged_control.track_controller", track_controller_);
  get_parameter("staged_control.approach_controller", approach_controller_);
  get_parameter("staged_control.approach_distance", approach_distance_);
  get_parameter("staged_control.align_heading_tolerance", align_heading_tolerance_);
  get_parameter("staged_control.align_angular_kp", align_angular_kp_);
  get_parameter("staged_control.align_max_angular_velocity", align_max_angular_velocity_);
  get_parameter("staged_control.align_min_angular_velocity", align_min_angular_velocity_);
  get_parameter("staged_control.align_target_distance", align_target_distance_);
  get_parameter("staged_control.prepare.enabled", prepare_enabled_);
  get_parameter("staged_control.prepare.obstacle_check_distance", prepare_obstacle_check_distance_);
  get_parameter("staged_control.prepare.side_check_distance", prepare_side_check_distance_);
  get_parameter("staged_control.prepare.obstacle_cost_threshold", prepare_obstacle_cost_threshold_);
  get_parameter("staged_control.prepare.backup_speed", prepare_backup_speed_);
  get_parameter("staged_control.prepare.backup_duration", prepare_backup_duration_);
  get_parameter("staged_control.prepare.backup_use_distance", prepare_backup_use_distance_);
  get_parameter("staged_control.prepare.backup_distance", prepare_backup_distance_);

  costmap_ros_->configure();
  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

  try {
    progress_checker_type_ = nav2_util::get_plugin_type_param(node, progress_checker_id_);
    progress_checker_ = progress_checker_loader_.createUniqueInstance(progress_checker_type_);
    RCLCPP_INFO(
      get_logger(), "Progress checker loaded: id='%s', type='%s'.",
      progress_checker_id_.c_str(), progress_checker_type_.c_str());
    progress_checker_->initialize(node, progress_checker_id_);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      get_logger(),
      "Progress checker '%s' failed to load: %s",
      progress_checker_id_.c_str(), ex.what());
    return nav2_util::CallbackReturn::FAILURE;
  }

  for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
    try {
      goal_checker_types_[i] = nav2_util::get_plugin_type_param(node, goal_checker_ids_[i]);
      nav2_core::GoalChecker::Ptr goal_checker =
        goal_checker_loader_.createUniqueInstance(goal_checker_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Goal checker loaded: id='%s', type='%s'.",
        goal_checker_ids_[i].c_str(), goal_checker_types_[i].c_str());
      goal_checker->initialize(node, goal_checker_ids_[i], costmap_ros_);
      goal_checkers_.insert({goal_checker_ids_[i], goal_checker});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Goal checker '%s' failed to load: %s",
        goal_checker_ids_[i].c_str(), ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  goal_checker_ids_concat_.clear();
  for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
    goal_checker_ids_concat_ += goal_checker_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Goal checkers: %s", goal_checker_ids_concat_.c_str());

  get_parameter("current_goal_checker", default_goal_checker_);
  if (goal_checkers_.find(default_goal_checker_) == goal_checkers_.end()) {
    RCLCPP_FATAL(
      get_logger(),
      "current_goal_checker '%s' not found. Available: %s",
      default_goal_checker_.c_str(), goal_checker_ids_concat_.c_str());
    return nav2_util::CallbackReturn::FAILURE;
  }

  for (size_t i = 0; i != controller_ids_.size(); i++) {
    try {
      controller_types_[i] = nav2_util::get_plugin_type_param(node, controller_ids_[i]);
      nav2_core::Controller::Ptr controller =
        lp_loader_.createUniqueInstance(controller_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Controller loaded: id='%s', type='%s'.",
        controller_ids_[i].c_str(), controller_types_[i].c_str());
      controller->configure(
        node, controller_ids_[i],
        costmap_ros_->getTfBuffer(), costmap_ros_);
      controllers_.insert({controller_ids_[i], controller});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Controller '%s' failed to load: %s",
        controller_ids_[i].c_str(), ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  controller_ids_concat_.clear();
  for (size_t i = 0; i != controller_ids_.size(); i++) {
    controller_ids_concat_ += controller_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Controllers: %s", controller_ids_concat_.c_str());

  get_parameter("current_controller", default_controller_);
  if (controllers_.find(default_controller_) == controllers_.end()) {
    RCLCPP_FATAL(
      get_logger(),
      "current_controller '%s' not found. Available: %s",
      default_controller_.c_str(), controller_ids_concat_.c_str());
    return nav2_util::CallbackReturn::FAILURE;
  }

  get_parameter("local_planner_plugins", local_planner_ids_);
  local_planner_types_.resize(local_planner_ids_.size());
  for (size_t i = 0; i != local_planner_ids_.size(); ++i) {
    try {
      local_planner_types_[i] = nav2_util::get_plugin_type_param(node, local_planner_ids_[i]);
      nav2_core::LocalPlanner::Ptr local_planner =
        local_planner_loader_.createUniqueInstance(local_planner_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Local planner loaded: id='%s', type='%s'.",
        local_planner_ids_[i].c_str(), local_planner_types_[i].c_str());
      local_planner->configure(
        node, local_planner_ids_[i], costmap_ros_->getTfBuffer(), costmap_ros_);
      local_planners_.insert({local_planner_ids_[i], local_planner});
    } catch (const std::exception & ex) {
      RCLCPP_FATAL(
        get_logger(), "Local planner '%s' failed to load: %s",
        local_planner_ids_[i].c_str(), ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  local_planner_ids_concat_.clear();
  for (const auto & id : local_planner_ids_) {
    local_planner_ids_concat_ += id + std::string(" ");
  }

  get_parameter("current_local_planner", current_local_planner_);
  if (current_local_planner_.empty()) {
    RCLCPP_INFO(
      get_logger(),
      "Local planning off. Using the global path.");
  } else if (local_planners_.find(current_local_planner_) == local_planners_.end()) {
    RCLCPP_FATAL(
      get_logger(),
      "current_local_planner '%s' not found. Available: %s",
      current_local_planner_.c_str(), local_planner_ids_concat_.c_str());
    return nav2_util::CallbackReturn::FAILURE;
  } else {
    RCLCPP_INFO(
      get_logger(), "Local planners: %s Selected: '%s'.",
      local_planner_ids_concat_.c_str(), current_local_planner_.c_str());
  }

  get_parameter("speed_planner_plugins", speed_planner_ids_);
  speed_planner_types_.resize(speed_planner_ids_.size());
  for (size_t i = 0; i != speed_planner_ids_.size(); i++) {
    try {
      speed_planner_types_[i] = nav2_util::get_plugin_type_param(node, speed_planner_ids_[i]);
      nav2_core::SpeedPlanner::Ptr speed_planner =
        speed_planner_loader_.createUniqueInstance(speed_planner_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Speed planner loaded: id='%s', type='%s'.",
        speed_planner_ids_[i].c_str(), speed_planner_types_[i].c_str());
      speed_planner->configure(
        node, speed_planner_ids_[i],
        costmap_ros_->getTfBuffer(), costmap_ros_);
      speed_planners_.insert({speed_planner_ids_[i], speed_planner});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Speed planner '%s' failed to load: %s",
        speed_planner_ids_[i].c_str(), ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  speed_planner_ids_concat_.clear();
  for (size_t i = 0; i != speed_planner_ids_.size(); i++) {
    speed_planner_ids_concat_ += speed_planner_ids_[i] + std::string(" ");
  }

  get_parameter("current_speed_planner", current_speed_planner_);
  if (current_speed_planner_.empty()) {
    RCLCPP_INFO(
      get_logger(),
      "Speed planning off. Controllers use paths without speed data.");
  } else if (speed_planners_.find(current_speed_planner_) == speed_planners_.end()) {
    RCLCPP_FATAL(
      get_logger(),
      "current_speed_planner '%s' not found. Available: %s",
      current_speed_planner_.c_str(), speed_planner_ids_concat_.c_str());
    return nav2_util::CallbackReturn::FAILURE;
  } else {
    RCLCPP_INFO(
      get_logger(),
      "Speed planners: %s Selected: '%s'.",
      speed_planner_ids_concat_.c_str(), current_speed_planner_.c_str());
  }

  if (staged_control_enabled_) {
    if (controllers_.find(track_controller_) == controllers_.end()) {
      RCLCPP_FATAL(
        get_logger(),
        "staged_control.track_controller '%s' not found. Available: %s",
        track_controller_.c_str(), controller_ids_concat_.c_str());
      return nav2_util::CallbackReturn::FAILURE;
    }
    if (controllers_.find(approach_controller_) == controllers_.end()) {
      RCLCPP_FATAL(
        get_logger(),
        "staged_control.approach_controller '%s' not found. Available: %s",
        approach_controller_.c_str(), controller_ids_concat_.c_str());
      return nav2_util::CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(
      get_logger(),
      "Staged control on: prepare=%s, align_limit=%.3f rad, track='%s', "
      "approach='%s', approach_distance=%.3f m.",
      prepare_enabled_ ? "true" : "false",
      align_heading_tolerance_,
      track_controller_.c_str(),
      approach_controller_.c_str(),
      approach_distance_);
  }

  odom_sub_ = std::make_unique<nav_2d_utils::OdomSubscriber>(node);
  longitudinal_state_sub_ = create_subscription<
    nav2_longitudinal_state_estimator::msg::LongitudinalState>(
    longitudinal_state_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ControllerServer::longitudinalStateCallback, this, std::placeholders::_1));
  get_parameter("hard_stop_topic", hard_stop_topic_);
  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  hard_stop_publisher_ = create_publisher<std_msgs::msg::Empty>(
    hard_stop_topic_, rclcpp::QoS(1).reliable());
  goal_error_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/navigation_goal_error",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
  // 速度规划可视化：transient_local 让 RViz 晚订阅也能收到最新一帧
  speed_plan_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/navigation/speed_plan",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  speed_plan_diagnostics_publisher_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/navigation/speed_plan_diagnostics",
    rclcpp::QoS(rclcpp::KeepLast(100)).reliable().transient_local());

  get_parameter("enable_nav_monitor_echo", enable_nav_monitor_echo_);
  if (enable_nav_monitor_echo_) {
    global_path_echo_pub_ = create_publisher<nav_msgs::msg::Path>(
      "nav_monitor/current_global_path", rclcpp::SystemDefaultsQoS());
    phase_echo_pub_ = create_publisher<std_msgs::msg::String>(
      "nav_monitor/controller_phase", rclcpp::SystemDefaultsQoS());
    desired_vel_echo_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "nav_monitor/desired_velocity", rclcpp::SystemDefaultsQoS());
    controller_target_echo_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "nav_monitor/controller_target", rclcpp::SystemDefaultsQoS());
  }

  nav_status_pub_ = create_publisher<nav_interfaces::msg::NavigationStatus>(
    "navigation_status", 10);

  // Create the action server that we implement with our followPath method
  action_server_ = std::make_unique<ActionServer>(
    shared_from_this(),
    "follow_path",
    std::bind(&ControllerServer::computeControl, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);

  // Set subscribtion to the speed limiting topic
  speed_limit_sub_ = create_subscription<nav2_msgs::msg::SpeedLimit>(
    speed_limit_topic, rclcpp::QoS(10),
    std::bind(&ControllerServer::speedLimitCallback, this, std::placeholders::_1));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  costmap_ros_->activate();
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->activate();
  }
  for (auto & lp : local_planners_) {
    lp.second->activate();
  }
  for (auto & sp : speed_planners_) {
    sp.second->activate();
  }
  vel_publisher_->on_activate();
  hard_stop_publisher_->on_activate();
  goal_error_publisher_->on_activate();
  speed_plan_publisher_->on_activate();
  speed_plan_diagnostics_publisher_->on_activate();
  if (global_path_echo_pub_) {
    global_path_echo_pub_->on_activate();
  }
  if (phase_echo_pub_) {
    phase_echo_pub_->on_activate();
  }
  if (desired_vel_echo_pub_) {
    desired_vel_echo_pub_->on_activate();
  }
  if (controller_target_echo_pub_) {
    controller_target_echo_pub_->on_activate();
  }
  invalidateSpeedPlanningContinuity();
  speed_plan_cycle_id_ = 0;
  speed_plan_path_id_ = 0;
  speed_plan_cycle_ = SpeedPlanCycleContext();
  nav_status_pub_->on_activate();
  action_server_->activate();

  auto node = shared_from_this();
  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&ControllerServer::dynamicParametersCallback, this, _1));

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->deactivate();
  }
  for (auto & lp : local_planners_) {
    lp.second->deactivate();
  }
  for (auto & sp : speed_planners_) {
    sp.second->deactivate();
  }

  /*
   * The costmap is also a lifecycle node, so it may have already fired on_deactivate
   * via rcl preshutdown cb. Despite the rclcpp docs saying on_shutdown callbacks fire
   * in the order added, the preshutdown callbacks clearly don't per se, due to using an
   * unordered_set iteration. Once this issue is resolved, we can maybe make a stronger
   * ordering assumption: https://github.com/ros2/rclcpp/issues/2096
   */
  if (costmap_ros_->get_current_state().id() ==
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    costmap_ros_->deactivate();
  }

  publishHardStopVelocity();
  nav_status_pub_->on_deactivate();
  goal_error_publisher_->on_deactivate();
  speed_plan_publisher_->on_deactivate();
  speed_plan_diagnostics_publisher_->on_deactivate();
  if (global_path_echo_pub_) {
    global_path_echo_pub_->on_deactivate();
  }
  if (phase_echo_pub_) {
    phase_echo_pub_->on_deactivate();
  }
  if (desired_vel_echo_pub_) {
    desired_vel_echo_pub_->on_deactivate();
  }
  if (controller_target_echo_pub_) {
    controller_target_echo_pub_->on_deactivate();
  }
  vel_publisher_->on_deactivate();
  hard_stop_publisher_->on_deactivate();
  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // Cleanup the helper classes
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->cleanup();
  }
  controllers_.clear();

  for (auto & lp : local_planners_) {
    lp.second->cleanup();
  }
  local_planners_.clear();

  for (auto & sp : speed_planners_) {
    sp.second->cleanup();
  }
  speed_planners_.clear();

  goal_checkers_.clear();

  if (costmap_ros_->get_current_state().id() ==
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    costmap_ros_->cleanup();
  }

  // Release any allocated resources
  action_server_.reset();
  odom_sub_.reset();
  longitudinal_state_sub_.reset();
  {
    std::lock_guard<std::mutex> lock(longitudinal_state_mutex_);
    has_longitudinal_state_ = false;
    latest_longitudinal_state_ =
      nav2_longitudinal_state_estimator::msg::LongitudinalState();
  }
  planned_acceleration_correction_ = 0.0;
  planned_acceleration_correction_valid_ = false;
  planned_acceleration_correction_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  costmap_thread_.reset();
  vel_publisher_.reset();
  nav_status_pub_.reset();
  hard_stop_publisher_.reset();
  goal_error_publisher_.reset();
  speed_plan_publisher_.reset();
  speed_plan_diagnostics_publisher_.reset();
  global_path_echo_pub_.reset();
  phase_echo_pub_.reset();
  desired_vel_echo_pub_.reset();
  controller_target_echo_pub_.reset();
  invalidateSpeedPlanningContinuity();
  speed_limit_sub_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool ControllerServer::findControllerId(
  const std::string & c_name,
  std::string & current_controller)
{
  if (c_name.empty()) {
    if (controllers_.find(default_controller_) != controllers_.end()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No controller set. Using current_controller '%s'.",
        default_controller_.c_str());
      current_controller = default_controller_;
      return true;
    }
  }

  if (controllers_.find(c_name) == controllers_.end()) {
    if (controllers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No controller set. Using the only loaded controller: %s",
        controller_ids_concat_.c_str());
      current_controller = controllers_.begin()->first;
    } else {
      RCLCPP_ERROR(
        get_logger(), "Controller '%s' not found. Available: %s",
        c_name.c_str(), controller_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Controller selected: '%s'.", c_name.c_str());
    current_controller = c_name;
  }

  return true;
}

bool ControllerServer::findGoalCheckerId(
  const std::string & c_name,
  std::string & current_goal_checker)
{
  if (c_name.empty()) {
    if (goal_checkers_.find(default_goal_checker_) != goal_checkers_.end()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No goal checker set. Using current_goal_checker '%s'.",
        default_goal_checker_.c_str());
      current_goal_checker = default_goal_checker_;
      return true;
    }
  }

  if (goal_checkers_.find(c_name) == goal_checkers_.end()) {
    if (goal_checkers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No goal checker set. Using the only loaded goal checker: %s",
        goal_checker_ids_concat_.c_str());
      current_goal_checker = goal_checkers_.begin()->first;
    } else {
      RCLCPP_ERROR(
        get_logger(), "Goal checker '%s' not found. Available: %s",
        c_name.c_str(), goal_checker_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Goal checker selected: '%s'.", c_name.c_str());
    current_goal_checker = c_name;
  }

  return true;
}

void ControllerServer::computeControl()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  is_collision_stop_active_ = false;
  is_progress_check_paused_for_obstacle_stop_ = false;

  if (odom_sub_) {
    const auto start_twist = getThresholdedTwist(odom_sub_->getTwist());
    RCLCPP_INFO(
      get_logger(),
      "FollowPath started: odom_vx=%.3f m/s, odom_vy=%.3f m/s, odom_wz=%.3f rad/s.",
      start_twist.x, start_twist.y, start_twist.theta);
  } else {
    RCLCPP_INFO(get_logger(), "FollowPath started.");
  }

  publishNavigationStatus(nav_interfaces::msg::NavigationStatus::NAVIGATING);

  try {
    std::string c_name = action_server_->get_current_goal()->controller_id;
    std::string current_controller;
    if (findControllerId(c_name, current_controller)) {
      current_controller_ = current_controller;
    } else {
      resetEndLocalizationCapabilities(goal_generation_);
      action_server_->terminate_current();
      return;
    }

    std::string gc_name = action_server_->get_current_goal()->goal_checker_id;
    std::string current_goal_checker;
    if (findGoalCheckerId(gc_name, current_goal_checker)) {
      current_goal_checker_ = current_goal_checker;
    } else {
      resetEndLocalizationCapabilities(goal_generation_);
      action_server_->terminate_current();
      return;
    }

    setPlannerPath(
      action_server_->get_current_goal()->path,
      PathUpdateType::NEW_GOAL);
    progress_checker_->reset();
    // 阶段状态只以“到达目标”为重置基准（isGoalReached 分支），不在此处重置：
    // computeControl() 会在同一次导航内因 BT 恢复重试（前方受阻→异常→
    // RecoveryNode 重发 FollowPath）被多次重新进入，入口重置会导致阶段一
    // 反复重新武装、无限重试后退。“一次导航只走一次阶段一”只能靠到达信号
    // 判定——没有可靠信号能区分“恢复重试”与“真正开始新导航”。

    last_valid_cmd_time_ = now();
    rclcpp::WallRate loop_rate(controller_frequency_);
    // added by hz
    std::chrono::time_point<std::chrono::steady_clock> start;
    std::chrono::time_point<std::chrono::steady_clock> end;
    std::chrono::duration<double, std::milli> elapsed;
    // end
    while (rclcpp::ok()) {
      if (action_server_ == nullptr || !action_server_->is_server_active()) {
        RCLCPP_DEBUG(get_logger(), "Action server is not active. Control stopped.");
        resetEndLocalizationCapabilities(goal_generation_);
        return;
      }

      if (action_server_->is_cancel_requested()) {
        RCLCPP_INFO(get_logger(), "Goal canceled. Stopping robot.");
        publishHardStopVelocity();
        publishNavigationStatus(nav_interfaces::msg::NavigationStatus::IDLE);
        resetEndLocalizationCapabilities(goal_generation_);
        // The FollowPath result is the controller-side stop acknowledgement used
        // by BtActionNode cleanup. Publish zero before making that result visible.
        action_server_->terminate_all();
        return;
      }

      // Don't compute a trajectory until costmap is valid (after clear costmap)
      rclcpp::Rate r(100);
      while (!costmap_ros_->isCurrent()) {
        if (action_server_->is_cancel_requested()) {
          RCLCPP_INFO(
            get_logger(),
            "Goal canceled while waiting for costmap. Stopping robot.");
          publishHardStopVelocity();
          publishNavigationStatus(nav_interfaces::msg::NavigationStatus::IDLE);
          resetEndLocalizationCapabilities(goal_generation_);
          action_server_->terminate_all();
          return;
        }
        r.sleep();
      }
      start = std::chrono::steady_clock::now();
      updateGlobalPath();
      end = std::chrono::steady_clock::now();
      elapsed = end - start;
      if (elapsed.count() >= 100) {
        RCLCPP_WARN(
          get_logger(), "Slow path update: %.3f ms.",
          elapsed.count());
      }

      // ③④ 规划级：每个控制周期先把 global path 裁到机器人投影点，再从最新
      // odom 速度与最新 costmap 重算速度剖面。任何已配置规划阶段失败都零速闭环，
      // 不允许 controller 继续消费上周期缓存轨迹。
      bool planning_succeeded = false;
      try {
        planning_succeeded = runPlanningPipeline(current_path_);
      } catch (const std::exception &) {
        if (speed_plan_cycle_.active) {
          speed_plan_cycle_.planning_result = "planning_exception";
          publishSpeedPlanDiagnostics(
            false, false, std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(), "not_run");
        }
        throw;
      }
      if (!planning_succeeded) {
        publishZeroVelocity();
        if (!loop_rate.sleep()) {
          RCLCPP_WARN(
            get_logger(),
            "Control loop late after planning failure: target=%.4f Hz.",
            controller_frequency_);
        }
        continue;
      }

      start = std::chrono::steady_clock::now();
      computeAndPublishVelocity();
      end = std::chrono::steady_clock::now();
      elapsed = end - start;
      if (elapsed.count() >= 100) {
        RCLCPP_WARN(
          get_logger(), "Slow control update: %.3f ms.",
          elapsed.count());
      }

      if (isGoalReached()) {
        RCLCPP_INFO(get_logger(), "Goal reached.");
        resetStagedControlState();
        break;
      }

      end = std::chrono::steady_clock::now();
      elapsed = end - start;

      if (!loop_rate.sleep()) {
        RCLCPP_WARN(
          get_logger(), "Control loop late: target=%.4f Hz, control_time=%.3f ms.",
          controller_frequency_, elapsed.count());
      }
    }
  } catch (nav2_core::CollisionException & e) {
    RCLCPP_ERROR(this->get_logger(), "Collision detected! %s", e.what());
    publishHardStopVelocity();
    resetEndLocalizationCapabilities(goal_generation_);
    publishNavigationStatus(nav_interfaces::msg::NavigationStatus::FAILED, e.what());
    action_server_->terminate_current();
    publishNavigationStatus(nav_interfaces::msg::NavigationStatus::IDLE);
    return;
  } catch (nav2_core::PlannerException & e) {
    RCLCPP_ERROR(this->get_logger(), "Control failed: %s", e.what());
    publishHardStopVelocity();
    resetEndLocalizationCapabilities(goal_generation_);
    publishNavigationStatus(nav_interfaces::msg::NavigationStatus::FAILED, e.what());
    action_server_->terminate_current();
    publishNavigationStatus(nav_interfaces::msg::NavigationStatus::IDLE);
    return;
  }catch (std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Unexpected control error: %s", e.what());
    publishHardStopVelocity();
    resetEndLocalizationCapabilities(goal_generation_);
    publishNavigationStatus(nav_interfaces::msg::NavigationStatus::FAILED, e.what());
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    action_server_->terminate_current(result);
    publishNavigationStatus(nav_interfaces::msg::NavigationStatus::IDLE);
    return;
  }
  RCLCPP_DEBUG(get_logger(), "FollowPath succeeded.");

  publishZeroVelocity();

  // TODO(orduno) #861 Handle a pending preemption and set controller name
  publishNavigationStatus(nav_interfaces::msg::NavigationStatus::FINISHED);
  action_server_->succeeded_current();
  publishNavigationStatus(nav_interfaces::msg::NavigationStatus::IDLE);
}

void ControllerServer::setPlannerPath(
  const nav_msgs::msg::Path & path,
  PathUpdateType update_type)
{
  RCLCPP_DEBUG(
    get_logger(),
    "Set path: controller='%s'.", current_controller_.c_str());
  if (path.poses.empty()) {
    throw nav2_core::PlannerException("Path is empty. FollowPath cannot start.");
  }
  controllers_[current_controller_]->setPlan(path);
  if (staged_control_enabled_) {
    if (track_controller_ != current_controller_) {
      controllers_[track_controller_]->setPlan(path);
    }
    if (approach_controller_ != current_controller_ && approach_controller_ != track_controller_) {
      controllers_[approach_controller_]->setPlan(path);
    }
  }

  for (auto & lp : local_planners_) {
    lp.second->setPlan(path);
  }

  // ③④ 规划级不在此触发：setPlan 只更新插件输入与重置裁剪进度；局部路径和
  // 速度剖面在控制循环内每周期重算（见 computeControl）。

  commitPlannerPathState(path, update_type);
}

void ControllerServer::commitPlannerPathState(
  const nav_msgs::msg::Path & path,
  PathUpdateType update_type)
{
  if (update_type == PathUpdateType::NEW_GOAL) {
    // 只有新目标才换靶心。同目标时保持 end_pose_ 不动有两个作用：
    // ① 近目标重规划做 position-only 插值，path 终点本就有厘米级抖动，而
    //    GoalChecker 的接受窗同样是厘米级——靶心跟着抖，末端就是在追一个与
    //    容差同量级的动靶，稳定窗永远凑不满。
    // ② isSamePathGoal 拿新终点与 end_pose_ 比较。若 end_pose_ 每次都刷新，
    //    比较基准跟着漂，每次漂一点点都判"同目标"，目标可棘轮式走掉任意远
    //    而永不触发 NEW_GOAL。锚定后比较基准固定，累计漂移超容差才换目标。
    end_pose_ = path.poses.back();
    end_pose_.header.frame_id = path.header.frame_id;
    // The goal/path timestamp describes when planning completed, not the time at which every
    // following control cycle should query TF. end_pose_ is retained for the whole FollowPath
    // action, so a non-zero stamp quickly becomes older than map->odom and makes all
    // staged-control and goal-check transforms fail. Time zero explicitly requests the latest
    // available transform.
    end_pose_.header.stamp = rclcpp::Time(0);
    ++goal_generation_;
    resetEndLocalizationCapabilities(goal_generation_);
    goal_checkers_[current_goal_checker_]->reset();
    controller_phase_ = ControllerPhase::TRACK;
    terminal_brake_latched_ = false;
    terminal_stall_consecutive_cycles_ = 0;
    terminal_stall_recovery_count_ = 0;
    is_collision_stop_active_ = false;
    is_progress_check_paused_for_obstacle_stop_ = false;
  } else {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Same-goal path update: phase=%s, terminal_brake=%s, collision_stop=%s. State kept.",
      controllerPhaseName(controller_phase_), terminal_brake_latched_ ? "true" : "false",
      is_collision_stop_active_ ? "true" : "false");
  }

  RCLCPP_DEBUG(
    get_logger(), "Goal anchor: x=%.2f, y=%.2f.",
    end_pose_.pose.position.x, end_pose_.pose.position.y);

  current_path_ = path;
  ++speed_plan_path_id_;

  // nav_monitor 任务回显：这里是"一条 path 真正生效"语义上唯一的落点
  // （setPlannerPath 是首条路径与后续每次重规划的共同收敛点），复述一份供
  // nav2_nav_monitor 累计 planned_path_length/replanning_count
  if (enable_nav_monitor_echo_ && global_path_echo_pub_->is_activated()) {
    global_path_echo_pub_->publish(path);
  }
}

bool ControllerServer::computeLocalPath(
  const nav_msgs::msg::Path & global_path,
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav_msgs::msg::Path & local_path)
{
  local_path = global_path;
  if (current_local_planner_.empty()) {
    return true;
  }

  const auto local_planner = local_planners_.find(current_local_planner_);
  if (local_planner == local_planners_.end()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Local planner '%s' is not loaded. No command this cycle.",
      current_local_planner_.c_str());
    local_path = nav_msgs::msg::Path();
    return false;
  }

  local_path = nav_msgs::msg::Path();
  if (!local_planner->second->computeLocalPlan(pose, velocity, local_path) ||
    local_path.poses.empty())
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Local planner '%s' failed or gave an empty path. No command this cycle.",
      current_local_planner_.c_str());
    local_path = nav_msgs::msg::Path();
    return false;
  }
  return true;
}

nav2_core::InitialMotionState ControllerServer::buildInitialMotionState(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Twist & measured_velocity,
  bool should_stop_for_path_obstacle)
{
  pending_stitching_prefix_ = nav_interfaces::msg::RobotTrajectory();
  pending_stitching_boundary_ = nav2_core::InitialMotionState();
  nav2_longitudinal_state_estimator::msg::LongitudinalState measured_state;
  bool has_measured_state = false;
  {
    std::lock_guard<std::mutex> lock(longitudinal_state_mutex_);
    has_measured_state = has_longitudinal_state_;
    if (has_measured_state) {
      measured_state = latest_longitudinal_state_;
    }
  }
  double state_age = std::numeric_limits<double>::quiet_NaN();
  if (has_measured_state) {
    const rclcpp::Time state_stamp(measured_state.header.stamp, get_clock()->get_clock_type());
    state_age = (now() - state_stamp).seconds();
  }
  const bool state_fresh = has_measured_state && std::isfinite(state_age) &&
    state_age >= 0.0 && state_age <= longitudinal_state_max_age_;
  const bool velocity_usable = state_fresh && measured_state.velocity_valid &&
    std::isfinite(measured_state.velocity);
  const bool acceleration_usable = state_fresh && measured_state.valid &&
    measured_state.acceleration_valid && std::isfinite(measured_state.acceleration);

  // Let the arbiter compare the accepted trajectory against robot_localization velocity.
  // Normal stitching still inherits planned v/a; only an explicit mismatch re-anchors to
  // this measured velocity. Raw /feedback/odom remains the automatic fallback.
  geometry_msgs::msg::Twist arbiter_velocity = measured_velocity;
  if (velocity_usable) {
    arbiter_velocity.linear.x = measured_state.velocity;
  }
  auto decision = initial_motion_arbiter_.resolve(
    path, arbiter_velocity, now(), terminal_brake_latched_);
  const double planned_acceleration = decision.state.acceleration;
  const bool planned_acceleration_valid = decision.state.acceleration_valid &&
    std::isfinite(planned_acceleration);
  std::string acceleration_source = "planned_continuity";

  const bool longitudinal_tracking_phase =
    controller_phase_ == ControllerPhase::TRACK ||
    controller_phase_ == ControllerPhase::APPROACH;
  const bool use_planned_correction = planned_acceleration_correction_enabled_ &&
    planned_acceleration_valid && acceleration_usable && longitudinal_tracking_phase &&
    (!velocity_usable || measured_state.velocity >= -0.01);
  if (use_planned_correction) {
    const double measured_acceleration = std::clamp(
      measured_state.acceleration, state_acceleration_min_, state_acceleration_max_);
    const double innovation = std::clamp(
      measured_acceleration - planned_acceleration,
      -std::max(planned_acceleration_innovation_limit_, 0.0),
      std::max(planned_acceleration_innovation_limit_, 0.0));
    const double target_correction =
      std::clamp(planned_acceleration_correction_gain_, 0.0, 1.0) * innovation;

    // Only the small measurement correction is rate-limited. The Ruckig planned
    // acceleration remains the sole owner of the main cross-cycle jerk profile.
    if (!planned_acceleration_correction_valid_) {
      planned_acceleration_correction_ = target_correction;
      planned_acceleration_correction_valid_ = true;
    } else {
      const double dt = std::clamp(
        (now() - planned_acceleration_correction_stamp_).seconds(), 0.0, 0.5);
      const double max_delta = std::max(
        planned_acceleration_correction_max_jerk_, 0.0) * dt;
      planned_acceleration_correction_ += std::clamp(
        target_correction - planned_acceleration_correction_, -max_delta, max_delta);
    }
    planned_acceleration_correction_stamp_ = now();
    decision.state.acceleration = std::clamp(
      planned_acceleration + planned_acceleration_correction_,
      state_acceleration_min_, state_acceleration_max_);
    decision.state.acceleration_valid = true;
    acceleration_source = "planned_robot_localization_correction";
  } else if (acceleration_usable && !planned_acceleration_valid) {
    decision.state.acceleration = std::clamp(
      measured_state.acceleration, state_acceleration_min_, state_acceleration_max_);
    decision.state.acceleration_valid = true;
    acceleration_source = "robot_localization";
    planned_acceleration_correction_valid_ = false;
    planned_acceleration_correction_ = 0.0;
  } else if (!acceleration_usable) {
    planned_acceleration_correction_valid_ = false;
    planned_acceleration_correction_ = 0.0;
  }
  if (state_fresh) {
    decision.state.source_stamp = measured_state.header.stamp;
    decision.state.source_age = state_age;
  }
  decision.state.terminal_brake_latched = terminal_brake_latched_;
  decision.state.should_stop_for_path_obstacle = should_stop_for_path_obstacle;
  if (decision.hasStitchingPrefix()) {
    pending_stitching_prefix_ = decision.stitching_prefix;
    pending_stitching_boundary_ = decision.stitching_boundary;
    pending_stitching_boundary_.terminal_brake_latched = terminal_brake_latched_;
    pending_stitching_boundary_.should_stop_for_path_obstacle =
      should_stop_for_path_obstacle;
    pending_stitching_boundary_.source_stamp = decision.state.source_stamp;
    pending_stitching_boundary_.source_age = decision.state.source_age;
  }
  if (speed_plan_cycle_.active) {
    speed_plan_cycle_.initial_motion = decision.state;
    speed_plan_cycle_.initial_motion_valid = true;
    speed_plan_cycle_.initial_motion_source = decision.reason.has_value() ?
      nav2_speed_planner::initialMotionSourceName(*decision.reason) : "inherited";
    speed_plan_cycle_.initial_motion_measured_v = decision.measured_velocity;
    speed_plan_cycle_.initial_motion_sampled_v = decision.sampled_velocity;
    speed_plan_cycle_.initial_motion_deviation = decision.velocity_deviation;
    speed_plan_cycle_.initial_motion_planned_a = planned_acceleration;
    speed_plan_cycle_.initial_motion_acceleration_source = acceleration_source;
    speed_plan_cycle_.initial_motion_state_velocity = measured_state.velocity;
    speed_plan_cycle_.initial_motion_state_acceleration_raw =
      measured_state.acceleration_raw;
    speed_plan_cycle_.initial_motion_state_acceleration = measured_state.acceleration;
    speed_plan_cycle_.initial_motion_state_correction =
      planned_acceleration_correction_valid_ ? planned_acceleration_correction_ :
      std::numeric_limits<double>::quiet_NaN();
    speed_plan_cycle_.initial_motion_state_velocity_std = measured_state.velocity_std;
    speed_plan_cycle_.initial_motion_state_acceleration_std =
      measured_state.acceleration_std;
    speed_plan_cycle_.initial_motion_state_velocity_age = measured_state.velocity_age;
    speed_plan_cycle_.initial_motion_state_acceleration_age = measured_state.acceleration_age;
    speed_plan_cycle_.initial_motion_state_stamp_skew = measured_state.stamp_skew;
    speed_plan_cycle_.initial_motion_state_rejected_samples =
      measured_state.rejected_samples;
    speed_plan_cycle_.initial_motion_state_acceleration_clamps =
      measured_state.acceleration_clamps;
    speed_plan_cycle_.initial_motion_state_stationary = measured_state.stationary;
    speed_plan_cycle_.initial_motion_state_valid =
      state_fresh && measured_state.valid;
  }
  if (decision.reason.has_value()) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Initial motion updated: reason=%s, velocity_inherited=%s, "
      "acceleration_source=%s.",
      nav2_speed_planner::initialMotionSourceName(*decision.reason),
      decision.velocity_inherited ? "true" : "false",
      acceleration_source.c_str());
  }
  return decision.state;
}

void ControllerServer::longitudinalStateCallback(
  const nav2_longitudinal_state_estimator::msg::LongitudinalState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(longitudinal_state_mutex_);
  latest_longitudinal_state_ = *msg;
  has_longitudinal_state_ = true;
}

bool ControllerServer::activeControllerCanAvoidPathObstacles(
  const geometry_msgs::msg::PoseStamped & pose)
{
  std::string controller_id = current_controller_;
  if (staged_control_enabled_) {
    const double distance_to_goal = getDistanceToGoal(pose);
    controller_id = distance_to_goal <= approach_distance_ ?
      approach_controller_ : track_controller_;
  }
  const auto controller = controllers_.find(controller_id);
  if (controller == controllers_.end()) {
    return false;
  }
  const auto * capability = dynamic_cast<const nav2_core::PathObstacleAvoidanceCapability *>(
    controller->second.get());
  return capability != nullptr && capability->canAvoidPathObstacles();
}

void ControllerServer::updateTerminalBrakeLatch(
  const nav2_core::SpeedPlanningDiagnostics & diagnostics,
  bool longitudinal_continuity_valid)
{
  // 障碍约束随costmap实时变化，不能污染同目标生命周期的末端制动锁存。
  if (diagnostics.terminal_branch ==
    nav2_core::SpeedPlanningTerminalBranch::PATH_OBSTACLE_STOP)
  {
    return;
  }
  if (terminal_brake_latched_ || !diagnostics.terminal_brake_required) {
    return;
  }

  // Speed planning runs before the staged controller selects ALIGN/TRACK for
  // the current cycle. On a new stationary goal there is no consumed
  // longitudinal target yet, so a transient planner infeasible diagnostic is
  // not allowed to create sticky terminal state. Keep the fail-safe behavior
  // for a moving robot: a positive or non-finite required braking distance is
  // sufficient to latch even before the first compatible control receipt.
  constexpr double kBrakingDistanceEpsilon = 1e-6;
  const bool motion_requires_braking =
    !std::isfinite(diagnostics.terminal_braking_distance) ||
    diagnostics.terminal_braking_distance > kBrakingDistanceEpsilon;
  if (!longitudinal_continuity_valid && !motion_requires_braking) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Terminal brake not set: no motion history, braking=0, available=%.3f m, "
      "infeasible=%s.",
      diagnostics.terminal_distance_available,
      diagnostics.terminal_infeasible ? "true" : "false");
    return;
  }

  terminal_brake_latched_ = true;
  RCLCPP_WARN(
    get_logger(),
    "Terminal brake set: available=%.3f m, braking=%.3f m, reserve=%.3f m, "
    "infeasible=%s.",
    diagnostics.terminal_distance_available,
    diagnostics.terminal_braking_distance,
    diagnostics.terminal_execution_reserve,
    diagnostics.terminal_infeasible ? "true" : "false");
}

void ControllerServer::updateSpeedPlanningContinuityAfterControl(
  bool control_succeeded, bool trajectory_target_consumed,
  double consumed_target_velocity, double consumed_target_acceleration,
  double consumed_target_relative_time)
{
  const bool longitudinal_tracking_phase =
    controller_phase_ == ControllerPhase::TRACK ||
    controller_phase_ == ControllerPhase::APPROACH;
  if (!control_succeeded || !trajectory_target_consumed || !longitudinal_tracking_phase ||
    current_trajectory_.points.empty())
  {
    invalidateSpeedPlanningContinuity();
    return;
  }
  initial_motion_arbiter_.accept(
    current_trajectory_, consumed_target_velocity, consumed_target_acceleration,
    consumed_target_relative_time, now());
}

bool ControllerServer::updateTerminalStallRecovery(
  const nav_2d_msgs::msg::Twist2D & measured_twist,
  bool trajectory_target_consumed,
  double consumed_target_velocity,
  double consumed_target_acceleration)
{
  const bool progress_stalled =
    std::isfinite(speed_plan_cycle_.local_path_progress_delta) &&
    std::fabs(speed_plan_cycle_.local_path_progress_delta) <=
    terminal_stall_max_progress_per_cycle_;
  const bool frozen_terminal_brake = terminal_stall_recovery_enabled_ &&
    terminal_stall_recovery_count_ == 0 &&
    controller_phase_ == ControllerPhase::APPROACH &&
    terminal_brake_latched_ &&
    speed_plan_cycle_.terminal_valid &&
    speed_plan_cycle_.terminal.terminal_branch ==
    nav2_core::SpeedPlanningTerminalBranch::GOAL_TERMINAL &&
    speed_plan_cycle_.local_path_length >= terminal_stall_min_remaining_distance_ &&
    !is_collision_stop_active_ &&
    trajectory_target_consumed &&
    std::isfinite(consumed_target_velocity) &&
    std::isfinite(consumed_target_acceleration) &&
    consumed_target_velocity >= terminal_stall_min_target_velocity_ &&
    consumed_target_acceleration <= terminal_stall_max_target_acceleration_ &&
    std::fabs(measured_twist.x) <= terminal_stall_stopped_velocity_ &&
    progress_stalled;

  if (!frozen_terminal_brake) {
    terminal_stall_consecutive_cycles_ = 0;
    if (speed_plan_cycle_.active) {
      speed_plan_cycle_.terminal_stall_consecutive_cycles = 0;
      speed_plan_cycle_.terminal_stall_recovery_count = terminal_stall_recovery_count_;
    }
    return false;
  }

  ++terminal_stall_consecutive_cycles_;
  const int required_cycles = std::max(
    1, static_cast<int>(std::ceil(
      terminal_stall_recovery_duration_ * std::max(controller_frequency_, 1.0))));
  if (speed_plan_cycle_.active) {
    speed_plan_cycle_.terminal_stall_consecutive_cycles =
      terminal_stall_consecutive_cycles_;
    speed_plan_cycle_.terminal_stall_recovery_count = terminal_stall_recovery_count_;
  }
  if (terminal_stall_consecutive_cycles_ < required_cycles) {
    return false;
  }

  terminal_stall_consecutive_cycles_ = 0;
  ++terminal_stall_recovery_count_;
  // Critical: invalidate only the accepted trajectory prefix. Do not call
  // resetStagedControlState() and do not clear terminal_brake_latched_. The
  // next cycle therefore reseeds from measured standstill while the S-curve
  // recovery remains capped by the terminal 0.05 m/s envelope.
  invalidateSpeedPlanningContinuity();
  if (speed_plan_cycle_.active) {
    speed_plan_cycle_.terminal_stall_recovery_triggered = true;
    speed_plan_cycle_.terminal_stall_consecutive_cycles = required_cycles;
    speed_plan_cycle_.terminal_stall_recovery_count = terminal_stall_recovery_count_;
  }
  RCLCPP_WARN(
    get_logger(),
    "Terminal stall recovery: target=(v=%.3f,a=%.3f), odom_v=%.3f, "
    "progress=%.4f, terminal latch preserved.",
    consumed_target_velocity, consumed_target_acceleration, measured_twist.x,
    speed_plan_cycle_.local_path_progress_delta);
  return true;
}

void ControllerServer::invalidateSpeedPlanningContinuity()
{
  initial_motion_arbiter_.reset();
}

bool ControllerServer::runPlanningPipeline(const nav_msgs::msg::Path & path)
{
  using PipelineClock = std::chrono::steady_clock;
  constexpr double kSlowPlanningPipelineWarnMs = 50.0;
  const auto pipeline_start = PipelineClock::now();
  double odom_ms = 0.0;
  double pose_ms = 0.0;
  double local_planner_ms = 0.0;
  double speed_set_plan_ms = 0.0;
  double speed_compute_ms = 0.0;
  double visualization_ms = 0.0;
  nav_msgs::msg::Path planning_path;

  const auto elapsed_ms = [](const auto & begin, const auto & end) {
      return std::chrono::duration<double, std::milli>(end - begin).count();
    };
  const auto log_pipeline_timing = [&](const char * result) {
      const double total_ms = elapsed_ms(pipeline_start, PipelineClock::now());
      const double measured_ms =
        odom_ms + pose_ms + local_planner_ms + speed_set_plan_ms + speed_compute_ms +
        visualization_ms;
      const double unaccounted_ms = std::max(0.0, total_ms - measured_ms);
      const size_t planning_points = planning_path.poses.size();
      const std::string & planning_frame = planning_path.header.frame_id;
      if (total_ms >= kSlowPlanningPipelineWarnMs) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Slow planning (ms): result=%s total=%.3f odom=%.3f pose=%.3f "
          "local=%.3f speed_set_plan=%.3f speed_compute=%.3f viz=%.3f other=%.3f "
          "input_points=%zu planning_points=%zu trajectory_points=%zu "
          "input_frame='%s' planning_frame='%s'.",
          result, total_ms, odom_ms, pose_ms, local_planner_ms, speed_set_plan_ms,
          speed_compute_ms, visualization_ms, unaccounted_ms,
          path.poses.size(), planning_points, current_trajectory_.points.size(),
          path.header.frame_id.c_str(), planning_frame.c_str());
      } else {
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Planning (ms): result=%s total=%.3f odom=%.3f pose=%.3f "
          "local=%.3f speed_set_plan=%.3f speed_compute=%.3f viz=%.3f other=%.3f "
          "input_points=%zu planning_points=%zu trajectory_points=%zu "
          "input_frame='%s' planning_frame='%s'.",
          result, total_ms, odom_ms, pose_ms, local_planner_ms, speed_set_plan_ms,
          speed_compute_ms, visualization_ms, unaccounted_ms,
          path.poses.size(), planning_points, current_trajectory_.points.size(),
          path.header.frame_id.c_str(), planning_frame.c_str());
      }
    };

  // 规划级（③④），控制循环内每周期执行一次：③ 从机器人投影点裁出后缀路径；
  // ④ 从最新 odom 速度 + 最新 costmap 重算速度剖面。
  current_trajectory_ = nav_interfaces::msg::RobotTrajectory();
  if (current_speed_planner_.empty()) {
    log_pipeline_timing("speed_planner_disabled");
    return true;  // ④ 未配置：灰度回退，controller 走 setPlan 几何路径
  }
  speed_plan_cycle_ = SpeedPlanCycleContext();
  speed_plan_cycle_.active = true;
  speed_plan_cycle_.cycle_id = ++speed_plan_cycle_id_;
  speed_plan_cycle_.path_id = speed_plan_path_id_;
  speed_plan_cycle_.planner_id = current_speed_planner_;
  if (path.poses.empty()) {
    speed_plan_cycle_.planning_result = "empty_input";
    publishSpeedPlanDiagnostics(
      false, false, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), "not_run");
    log_pipeline_timing("empty_input");
    return false;
  }
  auto sp = speed_planners_.find(current_speed_planner_);
  if (sp == speed_planners_.end()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Speed planner '%s' is not loaded. No command this cycle.",
      current_speed_planner_.c_str());
    speed_plan_cycle_.planning_result = "speed_planner_unavailable";
    publishSpeedPlanDiagnostics(
      false, false, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), "not_run");
    log_pipeline_timing("speed_planner_unavailable");
    return false;
  }

  const auto odom_start = PipelineClock::now();
  const nav_2d_msgs::msg::Twist2D odom_twist = getThresholdedTwist(odom_sub_->getTwist());
  odom_ms = elapsed_ms(odom_start, PipelineClock::now());
  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = odom_twist.x;
  current_velocity.linear.y = odom_twist.y;
  current_velocity.angular.z = odom_twist.theta;

  geometry_msgs::msg::PoseStamped planning_pose;
  const std::string planning_frame = path.header.frame_id.empty() ?
    path.poses.front().header.frame_id : path.header.frame_id;
  const auto pose_start = PipelineClock::now();
  const bool pose_available = !planning_frame.empty() && nav2_util::getCurrentPose(
    planning_pose, *costmap_ros_->getTfBuffer(), planning_frame,
    costmap_ros_->getBaseFrameID(), costmap_ros_->getTransformTolerance());
  pose_ms = elapsed_ms(pose_start, PipelineClock::now());
  if (!pose_available) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Robot pose not found in path frame '%s'. No command this cycle.",
      planning_frame.c_str());
    speed_plan_cycle_.planning_result = "pose_lookup_failed";
    publishSpeedPlanDiagnostics(
      false, false, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), "not_run");
    log_pipeline_timing("pose_lookup_failed");
    return false;
  }

  if (!current_local_planner_.empty()) {
    const auto local_planner_start = PipelineClock::now();
    if (!computeLocalPath(path, planning_pose, current_velocity, planning_path)) {
      local_planner_ms = elapsed_ms(local_planner_start, PipelineClock::now());
      const auto visualization_start = PipelineClock::now();
      publishSpeedPlan(current_trajectory_);
      visualization_ms = elapsed_ms(visualization_start, PipelineClock::now());
      speed_plan_cycle_.planning_result = "local_planner_failed";
      publishSpeedPlanDiagnostics(
        false, false, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(), "not_run");
      log_pipeline_timing("local_planner_failed");
      return false;
    }
    local_planner_ms = elapsed_ms(local_planner_start, PipelineClock::now());
  } else {
    planning_path = path;
  }
  speed_plan_cycle_.local_path_point_count = planning_path.poses.size();
  for (size_t index = 1; index < planning_path.poses.size(); ++index) {
    const auto & previous = planning_path.poses[index - 1].pose.position;
    const auto & current = planning_path.poses[index].pose.position;
    speed_plan_cycle_.local_path_length +=
      std::hypot(current.x - previous.x, current.y - previous.y);
  }
  // 本拍投影前进的弧长 = 上一拍剩余 − 本拍剩余（同一条全局路径才有可比性）。
  // 与 odom 位移对账即可判定裁剪是否"超额消耗"：两者应当相等，实测出现过
  // 一拍多裁 36mm 而机器人只走了 25mm 的情况（R58）。
  if (std::isfinite(prev_local_path_length_) &&
    prev_local_path_id_ == speed_plan_cycle_.path_id)
  {
    speed_plan_cycle_.local_path_progress_delta =
      prev_local_path_length_ - speed_plan_cycle_.local_path_length;
  }
  prev_local_path_length_ = speed_plan_cycle_.local_path_length;
  prev_local_path_id_ = speed_plan_cycle_.path_id;

  const bool should_stop_for_path_obstacle =
    !activeControllerCanAvoidPathObstacles(planning_pose);
  const auto initial_motion = buildInitialMotionState(
    planning_path, current_velocity, should_stop_for_path_obstacle);
  nav_msgs::msg::Path speed_planning_path = planning_path;
  nav2_core::InitialMotionState speed_planning_boundary = initial_motion;
  nav2_speed_planner::TrajectoryStitchingPlan stitching_plan;
  bool use_stitching_prefix = false;
  if (pending_stitching_prefix_.points.size() >= 2) {
    nav2_speed_planner::InitialMotionDecision stitching_decision;
    stitching_decision.stitching_prefix = pending_stitching_prefix_;
    stitching_decision.stitching_boundary = pending_stitching_boundary_;
    std::string rejection_reason;
    if (!nav2_speed_planner::prepareTrajectoryStitching(
        planning_path, stitching_decision, trajectory_stitching_max_path_error_,
        stitching_plan, &rejection_reason))
    {
      speed_plan_cycle_.stitching_result = rejection_reason;
    } else if (!sp->second->isTrajectoryPrefixSafe(
        stitching_plan.prefix, should_stop_for_path_obstacle))
    {
      speed_plan_cycle_.stitching_result = "prefix_safety_rejected";
    } else {
      use_stitching_prefix = true;
      speed_planning_path = stitching_plan.suffix_path;
      speed_planning_boundary = stitching_plan.suffix_boundary;
      speed_plan_cycle_.stitching_result = "stitched";
      speed_plan_cycle_.stitching_prefix_points = stitching_plan.prefix.points.size();
      speed_plan_cycle_.stitching_prefix_duration = stitching_plan.prefix_duration;
    }
  } else {
    speed_plan_cycle_.stitching_result = "prefix_unavailable";
  }

  const auto speed_set_plan_start = PipelineClock::now();
  sp->second->setPlan(speed_planning_path);
  speed_set_plan_ms = elapsed_ms(speed_set_plan_start, PipelineClock::now());
  const auto speed_compute_start = PipelineClock::now();
  auto planned_suffix = sp->second->computeSpeedPlan(
    speed_planning_path, speed_planning_boundary);
  const auto suffix_diagnostics = sp->second->getLastPlanningDiagnostics();
  if (use_stitching_prefix && !planned_suffix.points.empty() &&
    !suffix_diagnostics.terminal_infeasible)
  {
    current_trajectory_ = nav2_speed_planner::concatenateStitchedTrajectory(
      stitching_plan.prefix, planned_suffix);
    speed_plan_cycle_.trajectory_stitched = true;
  } else if (use_stitching_prefix) {
    // A suffix failure must not publish the old prefix by itself. Retry once
    // through the established full-path state-inheritance route; if that also
    // fails the existing empty-trajectory fail-safe stops this cycle.
    speed_plan_cycle_.stitching_result = planned_suffix.points.empty() ?
      "suffix_failed_full_replan" : "suffix_infeasible_full_replan";
    speed_plan_cycle_.stitching_prefix_points = 0;
    speed_plan_cycle_.stitching_prefix_duration = 0.0;
    sp->second->setPlan(planning_path);
    current_trajectory_ = sp->second->computeSpeedPlan(planning_path, initial_motion);
  } else {
    current_trajectory_ = std::move(planned_suffix);
  }
  // 同一导航任务的终点位姿只允许来自 NEW_GOAL 时冻结的 end_pose_。速度规划
  // 每周期可以更新中间轨迹，但不得用局部路径尾点重新定义厘米级终点或最终角。
  if (!current_trajectory_.points.empty()) {
    geometry_msgs::msg::PoseStamped fixed_goal = end_pose_;
    fixed_goal.header.stamp = rclcpp::Time(0);
    bool fixed_goal_valid = true;
    const std::string trajectory_frame = current_trajectory_.header.frame_id.empty() ?
      planning_frame : current_trajectory_.header.frame_id;
    if (!trajectory_frame.empty() && fixed_goal.header.frame_id != trajectory_frame) {
      geometry_msgs::msg::PoseStamped transformed_goal;
      rclcpp::Duration transform_tolerance(
        rclcpp::Duration::from_seconds(costmap_ros_->getTransformTolerance()));
      if (nav_2d_utils::transformPose(
          costmap_ros_->getTfBuffer(), trajectory_frame, fixed_goal,
          transformed_goal, transform_tolerance))
      {
        fixed_goal = transformed_goal;
      } else {
        fixed_goal_valid = false;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Cannot transform frozen goal from '%s' to trajectory frame '%s'; "
          "keeping planner endpoint for this cycle.",
          end_pose_.header.frame_id.c_str(), trajectory_frame.c_str());
      }
    }
    if (fixed_goal_valid) {
      current_trajectory_.points.back().pose = fixed_goal;
    }
  }
  const auto planning_diagnostics = sp->second->getLastPlanningDiagnostics();
  speed_plan_cycle_.terminal = planning_diagnostics;
  speed_plan_cycle_.terminal_valid = true;
  updateTerminalBrakeLatch(
    planning_diagnostics, initial_motion.acceleration_valid);
  if (planning_diagnostics.terminal_infeasible) {
    // 终点目标已是v=0，terminal_speed_cap只负责低速区上限。不可行时仍必须
    // 完整记录入口/兜底原因，避免把物理停车距离不足与求解器错误混为一谈。
    static const char * kKind[] = {"feasible", "physical", "numerical"};
    static const char * kReason[] = {
      "none", "stable_segment_unreachable", "fallback_unreachable",
      "duration_budget_invalid", "solver_error", "non_finite_output",
      "iteration_exhausted"};
    const auto & d = planning_diagnostics;
    const int k = (d.terminal_infeasible_kind >= 0 && d.terminal_infeasible_kind < 3) ?
      d.terminal_infeasible_kind : 0;
    const int er = (d.terminal_entry_reason >= 0 && d.terminal_entry_reason < 7) ?
      d.terminal_entry_reason : 0;
    const int fr = (d.terminal_infeasible_reason >= 0 && d.terminal_infeasible_reason < 7) ?
      d.terminal_infeasible_reason : 0;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Speed plan fallback: planner=%s kind=%s entry=%s reason=%s profile=%s "
      "anchor=%.3f available=%.3f braking=%.3f reserve=%.3f latched_reserve=%.3f "
      "initial_vx=%.3f initial_a=%.3f brake_latched=%d path_length=%.3f path_points=%zu "
      "branch=%s obstacle_index=%d obstacle_arc=%.3f "
      "solver=(code=%d v0=%.4f a0=%.4f vmax=%.4f target_v=%.4f distance=%.4f).",
      current_speed_planner_.c_str(), kKind[k], kReason[er], kReason[fr],
      d.terminal_result_provenance ? "braking_fallback" : "normal",
      d.terminal_anchor_s, d.terminal_distance_available, d.terminal_braking_distance,
      d.terminal_execution_reserve, d.terminal_latched_reserve,
      initial_motion.velocity.linear.x, initial_motion.acceleration,
      initial_motion.terminal_brake_latched ? 1 : 0,
      nav2_util::geometry_utils::calculate_path_length(planning_path),
      planning_path.poses.size(),
      speedPlanningTerminalBranchName(d.terminal_branch),
      d.first_obstacle_index, d.first_obstacle_arc,
      d.solver_code, d.solver_v0, d.solver_a0, d.solver_vmax,
      d.solver_target_v, d.solver_distance);
  }
  speed_compute_ms = elapsed_ms(speed_compute_start, PipelineClock::now());
  if (current_trajectory_.points.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Speed planner '%s' returned no points. No command this cycle.",
      current_speed_planner_.c_str());
    const auto visualization_start = PipelineClock::now();
    publishSpeedPlan(current_trajectory_);  // 清除 RViz 旧 marker
    visualization_ms = elapsed_ms(visualization_start, PipelineClock::now());
    speed_plan_cycle_.planning_result = "speed_planner_failed";
    publishSpeedPlanDiagnostics(
      false, false, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), "not_run");
    log_pipeline_timing("speed_planner_failed");
    return false;
  }

  // 轨迹按需分发：本周期跑哪个 controller 要到 computeStagedVelocity 才决定
  // （它依赖 pose 与到目标距离），这里广播给全部 staged controller 等于替不会
  // 跑的插件白做一次整条轨迹的 TF 变换。改为在 computeControllerVelocity 里
  // 按实际选中的 controller 分发一次——同一周期内先 setTrajectory 再
  // computeVelocityCommands，交棒那一拍拿到的仍是本周期轨迹，不引入陈旧。
  trajectory_dispatched_to_.clear();

  // RViz 可视化节流（每周期重算但无需每周期重画）。
  const auto visualization_start = PipelineClock::now();
  const rclcpp::Time viz_now = now();
  if (last_speed_plan_viz_time_.nanoseconds() == 0 ||
    (viz_now - last_speed_plan_viz_time_).seconds() >= 0.5)
  {
    publishSpeedPlan(current_trajectory_);
    last_speed_plan_viz_time_ = viz_now;
  }
  visualization_ms = elapsed_ms(visualization_start, PipelineClock::now());

  speed_plan_cycle_.planning_succeeded = true;
  speed_plan_cycle_.planning_result = "success";
  log_pipeline_timing("success");

  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "Speed plan ready: planner=%s trajectory_points=%zu odom_vx=%.3f initial_vx=%.3f "
    "initial_a=%.3f acceleration_inherited=%s source_age=%.3f s.",
    current_speed_planner_.c_str(), current_trajectory_.points.size(),
    current_velocity.linear.x, initial_motion.velocity.linear.x, initial_motion.acceleration,
    initial_motion.acceleration_valid ? "true" : "false", initial_motion.source_age);
  return true;
}

void ControllerServer::publishNavigationStatus(int32_t status, const std::string & message)
{
  if (nav_status_pub_ && nav_status_pub_->is_activated()) {
    auto msg = std::make_unique<nav_interfaces::msg::NavigationStatus>();
    msg->status = status;
    msg->message = message;
    nav_status_pub_->publish(std::move(msg));
    RCLCPP_INFO(get_logger(), "[PublishNavigationStatus] status: %d, message: %s", status, message.c_str());
  }
}

void ControllerServer::publishSpeedPlan(const nav_interfaces::msg::RobotTrajectory & trajectory)
{
  if (!speed_plan_publisher_ || !speed_plan_publisher_->is_activated()) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;

  // 先清除上一帧全部 marker（避免旧轨迹残留）。
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  if (trajectory.points.empty()) {
    speed_plan_publisher_->publish(marker_array);
    return;
  }

  std_msgs::msg::Header header = trajectory.header;
  if (header.frame_id.empty()) {
    header.frame_id = trajectory.points.front().pose.header.frame_id;
  }
  header.stamp = now();

  double v_peak = 1e-3;
  for (const auto & pt : trajectory.points) {
    v_peak = std::max(v_peak, std::fabs(pt.longitudinal_velocity));
  }

  // 速度着色轨迹线：红=0 → 绿=峰值 逐点渐变。
  visualization_msgs::msg::Marker line;
  line.header = header;
  line.ns = "speed_plan";
  line.id = 0;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.pose.orientation.w = 1.0;
  line.scale.x = 0.03;
  line.points.reserve(trajectory.points.size());
  line.colors.reserve(trajectory.points.size());
  for (const auto & pt : trajectory.points) {
    geometry_msgs::msg::Point p = pt.pose.pose.position;
    p.z += 0.05;
    line.points.push_back(p);
    const double ratio =
      std::clamp(std::fabs(pt.longitudinal_velocity) / v_peak, 0.0, 1.0);
    std_msgs::msg::ColorRGBA c;
    c.r = static_cast<float>(1.0 - ratio);
    c.g = static_cast<float>(ratio);
    c.b = 0.1f;
    c.a = 1.0f;
    line.colors.push_back(c);
  }
  marker_array.markers.push_back(line);

  // 抽样速度数值文本（全程约 20 个标签 + 末点必标）。
  const size_t n = trajectory.points.size();
  const size_t stride = std::max<size_t>(1, n / 20);
  int text_id = 1;
  for (size_t i = 0; i < n; i += stride) {
    const size_t idx = (i + stride >= n) ? n - 1 : i;  // 末段直接标末点
    const auto & pt = trajectory.points[idx];
    visualization_msgs::msg::Marker text;
    text.header = header;
    text.ns = "speed_plan_text";
    text.id = text_id++;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose = pt.pose.pose;
    text.pose.position.z += 0.25;
    text.scale.z = 0.12;
    text.color.r = 1.0f;
    text.color.g = 1.0f;
    text.color.b = 1.0f;
    text.color.a = 0.9f;
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%.2f", pt.longitudinal_velocity);
    text.text = buf;
    marker_array.markers.push_back(text);
    if (idx == n - 1) {
      break;
    }
  }

  // 关键锚点标注（语义来自 Phase A 的 region_type 透传）：
  // 避障停止点/障碍区起终点、曲率减速段起终点、蠕行区起点。
  {
    struct Anchor {size_t idx; const char * label; float r, g, b;};
    std::vector<Anchor> anchors;
    long first_obs = -1, last_obs = -1;
    long first_safety_margin = -1, first_stop_margin = -1;
    long first_curve = -1, last_curve = -1, first_creep = -1;
    for (size_t i = 0; i < trajectory.points.size(); ++i) {
      const auto & rt = trajectory.points[i].region_type;
      if (rt == "obstacle") {
        if (first_obs < 0) {first_obs = static_cast<long>(i);}
        last_obs = static_cast<long>(i);
      } else if (rt == "obstacle_safety_margin") {
        if (first_safety_margin < 0) {first_safety_margin = static_cast<long>(i);}
      } else if (rt == "obstacle_stop_margin") {
        if (first_stop_margin < 0) {first_stop_margin = static_cast<long>(i);}
      } else if (rt == "curve") {
        if (first_curve < 0) {first_curve = static_cast<long>(i);}
        last_curve = static_cast<long>(i);
      } else if (rt == "stop_zone") {
        if (first_creep < 0) {first_creep = static_cast<long>(i);}
      }
    }
    if (first_stop_margin >= 0) {
      anchors.push_back(
        {static_cast<size_t>(first_stop_margin), "obs_stop_margin", 1.0f, 0.5f, 0.0f});
    }
    if (first_safety_margin >= 0) {
      anchors.push_back(
        {static_cast<size_t>(first_safety_margin), "obs_safety_margin", 0.6f, 1.0f, 0.2f});
    }
    if (first_obs >= 0) {
      anchors.push_back({static_cast<size_t>(first_obs), "obs_start", 1.0f, 0.0f, 0.0f});
    }
    if (last_obs > first_obs) {
      anchors.push_back({static_cast<size_t>(last_obs), "obs_end", 1.0f, 0.0f, 0.0f});
    }
    if (first_curve >= 0) {
      anchors.push_back({static_cast<size_t>(first_curve), "curve_start", 0.6f, 0.2f, 0.9f});
    }
    if (last_curve > first_curve) {
      anchors.push_back({static_cast<size_t>(last_curve), "curve_end", 0.6f, 0.2f, 0.9f});
    }
    if (first_creep >= 0) {
      anchors.push_back({static_cast<size_t>(first_creep), "creep_start", 0.2f, 0.4f, 1.0f});
    }
    int anchor_id = 0;
    for (const auto & a : anchors) {
      const auto & pose = trajectory.points[a.idx].pose.pose;
      visualization_msgs::msg::Marker sphere;
      sphere.header = header;
      sphere.ns = "speed_plan_anchor";
      sphere.id = anchor_id;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose = pose;
      sphere.pose.position.z += 0.05;
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.07;
      sphere.color.r = a.r;
      sphere.color.g = a.g;
      sphere.color.b = a.b;
      sphere.color.a = 0.95f;
      marker_array.markers.push_back(sphere);

      visualization_msgs::msg::Marker label;
      label.header = header;
      label.ns = "speed_plan_anchor_text";
      label.id = anchor_id++;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose = pose;
      label.pose.position.z += 0.45;
      label.scale.z = 0.11;
      label.color.r = a.r;
      label.color.g = a.g;
      label.color.b = a.b;
      label.color.a = 1.0f;
      label.text = a.label;
      marker_array.markers.push_back(label);
    }
  }

  speed_plan_publisher_->publish(marker_array);
}

void ControllerServer::publishSpeedPlanDiagnostics(
  bool control_succeeded,
  bool control_receipt_valid,
  double consumed_target_velocity,
  double consumed_target_acceleration,
  const std::string & control_result)
{
  if (!speed_plan_cycle_.active) {
    return;
  }
  const SpeedPlanCycleContext cycle = speed_plan_cycle_;
  speed_plan_cycle_.active = false;
  if (!speed_plan_diagnostics_publisher_ ||
    !speed_plan_diagnostics_publisher_->is_activated())
  {
    return;
  }

  const bool receipt_valid =
    control_receipt_valid &&
    std::isfinite(consumed_target_velocity) &&
    std::isfinite(consumed_target_acceleration);
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  // 碰撞等待属于安全停车，不按控制失败上报。
  const bool is_collision_stop_wait = control_result == "collision_stop_wait";
  if (!cycle.planning_succeeded || (!control_succeeded && !is_collision_stop_wait)) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  } else if (is_collision_stop_wait ||
    (cycle.terminal_valid && cycle.terminal.terminal_infeasible))
  {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  }
  status.name = "navigation_speed_plan";
  if (!cycle.planning_succeeded) {
    status.message = cycle.planning_result;
  } else if (is_collision_stop_wait) {
    status.message = "collision_stop_wait";
  } else if (!control_succeeded) {
    status.message = control_result;
  } else if (cycle.terminal_valid && cycle.terminal.terminal_infeasible) {
    status.message = cycle.terminal.terminal_branch ==
      nav2_core::SpeedPlanningTerminalBranch::PATH_OBSTACLE_STOP ?
      "obstacle_braking_infeasible" : "terminal_infeasible";
  } else {
    status.message = "ok";
  }
  status.hardware_id = "nav2_controller";

  const auto add_value = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = value;
      status.values.push_back(std::move(item));
    };
  add_value("schema_version", "1");
  add_value("cycle_id", std::to_string(cycle.cycle_id));
  add_value("path_id", std::to_string(cycle.path_id));
  add_value("planning_succeeded", cycle.planning_succeeded ? "true" : "false");
  add_value("planning_result", cycle.planning_result);
  add_value("planner", cycle.planner_id);
  add_value("local_path_point_count", std::to_string(cycle.local_path_point_count));
  add_value("local_path_length", std::to_string(cycle.local_path_length));
  add_value("trajectory_point_count", std::to_string(current_trajectory_.points.size()));
  add_value("trajectory_stitched", cycle.trajectory_stitched ? "true" : "false");
  add_value("stitching_result", cycle.stitching_result);
  add_value("stitching_prefix_points", std::to_string(cycle.stitching_prefix_points));
  add_value(
    "stitching_prefix_duration", std::to_string(cycle.stitching_prefix_duration));
  add_value(
    "terminal_stall_recovery_triggered",
    cycle.terminal_stall_recovery_triggered ? "true" : "false");
  add_value(
    "terminal_stall_consecutive_cycles",
    std::to_string(cycle.terminal_stall_consecutive_cycles));
  add_value(
    "terminal_stall_recovery_count",
    std::to_string(cycle.terminal_stall_recovery_count));
  add_value("boundary_computed", cycle.initial_motion_valid ? "true" : "false");
  add_value("boundary_reason", cycle.initial_motion_source);
  // 裁决输入：boundary_velocity 是结果，这三个是它的来源。
  add_value("initial_motion_measured_v", std::to_string(cycle.initial_motion_measured_v));
  add_value("initial_motion_sampled_v", std::to_string(cycle.initial_motion_sampled_v));
  add_value("initial_motion_deviation", std::to_string(cycle.initial_motion_deviation));
  add_value("initial_motion_acceleration_source", cycle.initial_motion_acceleration_source);
  add_value("initial_motion_planned_a", std::to_string(cycle.initial_motion_planned_a));
  add_value(
    "initial_motion_state_velocity",
    std::to_string(cycle.initial_motion_state_velocity));
  add_value(
    "initial_motion_state_acceleration_raw",
    std::to_string(cycle.initial_motion_state_acceleration_raw));
  add_value(
    "initial_motion_state_acceleration",
    std::to_string(cycle.initial_motion_state_acceleration));
  add_value(
    "initial_motion_state_correction",
    std::to_string(cycle.initial_motion_state_correction));
  add_value(
    "initial_motion_state_velocity_std",
    std::to_string(cycle.initial_motion_state_velocity_std));
  add_value(
    "initial_motion_state_acceleration_std",
    std::to_string(cycle.initial_motion_state_acceleration_std));
  add_value(
    "initial_motion_state_velocity_age",
    std::to_string(cycle.initial_motion_state_velocity_age));
  add_value(
    "initial_motion_state_acceleration_age",
    std::to_string(cycle.initial_motion_state_acceleration_age));
  add_value(
    "initial_motion_state_stamp_skew",
    std::to_string(cycle.initial_motion_state_stamp_skew));
  add_value(
    "initial_motion_state_rejected_samples",
    std::to_string(cycle.initial_motion_state_rejected_samples));
  add_value(
    "initial_motion_state_acceleration_clamps",
    std::to_string(cycle.initial_motion_state_acceleration_clamps));
  add_value(
    "initial_motion_state_stationary",
    cycle.initial_motion_state_stationary ? "true" : "false");
  add_value(
    "initial_motion_state_valid",
    cycle.initial_motion_state_valid ? "true" : "false");
  add_value("local_path_progress_delta", std::to_string(cycle.local_path_progress_delta));
  if (cycle.initial_motion_valid) {
    add_value("boundary_velocity", std::to_string(cycle.initial_motion.velocity.linear.x));
    add_value("boundary_acceleration", std::to_string(cycle.initial_motion.acceleration));
    add_value(
      "boundary_acceleration_valid",
      cycle.initial_motion.acceleration_valid ? "true" : "false");
    add_value(
      "boundary_should_stop_for_path_obstacle",
      cycle.initial_motion.should_stop_for_path_obstacle ? "true" : "false");
    add_value("boundary_source_age", std::to_string(cycle.initial_motion.source_age));
    add_value(
      "boundary_source_stamp_sec", std::to_string(cycle.initial_motion.source_stamp.sec));
    add_value(
      "boundary_source_stamp_nanosec", std::to_string(cycle.initial_motion.source_stamp.nanosec));
  }
  add_value("terminal_valid", cycle.terminal_valid ? "true" : "false");
  add_value(
    "terminal_infeasible",
    cycle.terminal_valid && cycle.terminal.terminal_infeasible ? "true" : "false");
  add_value(
    "terminal_brake_required",
    cycle.terminal_valid && cycle.terminal.terminal_brake_required ? "true" : "false");
  add_value("terminal_brake_latched", terminal_brake_latched_ ? "true" : "false");
  add_value(
    "progress_check_paused_for_obstacle_stop",
    is_progress_check_paused_for_obstacle_stop_ ? "true" : "false");
  if (cycle.terminal_valid) {
    add_value(
      "terminal_branch", speedPlanningTerminalBranchName(cycle.terminal.terminal_branch));
    add_value(
      "first_obstacle_index", std::to_string(cycle.terminal.first_obstacle_index));
    add_value(
      "first_obstacle_arc", std::to_string(cycle.terminal.first_obstacle_arc));
    add_value("terminal_anchor_s", std::to_string(cycle.terminal.terminal_anchor_s));
    add_value(
      "terminal_execution_reserve",
      std::to_string(cycle.terminal.terminal_execution_reserve));
    add_value(
      "terminal_braking_distance",
      std::to_string(cycle.terminal.terminal_braking_distance));
    add_value(
      "terminal_distance_available",
      std::to_string(cycle.terminal.terminal_distance_available));
  }
  add_value("control_succeeded", control_succeeded ? "true" : "false");
  add_value("control_result", control_result);
  add_value("control_receipt_valid", receipt_valid ? "true" : "false");
  if (receipt_valid) {
    add_value("control_target_velocity", std::to_string(consumed_target_velocity));
    add_value("control_target_acceleration", std::to_string(consumed_target_acceleration));
  }
  add_value(
    "controller", control_result == "not_run" ? "not_run" : active_controller_for_log_);
  add_value("phase", controllerPhaseName(controller_phase_));

  message.status.push_back(status);
  speed_plan_diagnostics_publisher_->publish(message);
}

bool ControllerServer::isSamePathGoal(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty() || current_path_.poses.empty()) {
    return false;
  }

  geometry_msgs::msg::PoseStamped new_end_pose = path.poses.back();
  new_end_pose.header.frame_id = path.header.frame_id;
  geometry_msgs::msg::PoseStamped transformed_new_end_pose;
  try {
    rclcpp::Duration tolerance(
      rclcpp::Duration::from_seconds(costmap_ros_->getTransformTolerance()));
    if (!nav_2d_utils::transformPose(
      costmap_ros_->getTfBuffer(), end_pose_.header.frame_id,
      new_end_pose, transformed_new_end_pose, tolerance))
    {
      return false;
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Path end TF failed: %s. Treating path as a new goal.",
      ex.what());
    return false;
  }

  // 近目标重规划会做 position-only interpolation，path 终点可能有厘米级以内的微小差异。
  // 这里判断的是“业务目标是否相同”，不是要求两个 path 终点数学上完全重合。
  constexpr double kSameGoalPositionTolerance = 0.02;
  constexpr double kSameGoalYawTolerance = 0.05;
  const double goal_delta = nav2_util::geometry_utils::euclidean_distance(
    transformed_new_end_pose, end_pose_);
  const double yaw_delta = std::abs(
    angles::shortest_angular_distance(
      tf2::getYaw(transformed_new_end_pose.pose.orientation),
      tf2::getYaw(end_pose_.pose.orientation)));

  const bool same_goal =
    goal_delta <= kSameGoalPositionTolerance && yaw_delta <= kSameGoalYawTolerance;
  if (!same_goal) {
    RCLCPP_INFO(
      get_logger(),
      "Path goal changed: position_delta=%.4f m (limit=%.4f), "
      "yaw_delta=%.4f rad (limit=%.4f).",
      goal_delta, kSameGoalPositionTolerance, yaw_delta, kSameGoalYawTolerance);
  }
  return same_goal;
}

const char * ControllerServer::controllerPhaseName(ControllerPhase phase) const
{
  switch (phase) {
    case ControllerPhase::PREPARE:
      return "prepare";
    case ControllerPhase::ALIGN:
      return "align";
    case ControllerPhase::TRACK:
      return "track";
    case ControllerPhase::APPROACH:
      return "approach";
  }
  return "unknown";
}

std::string ControllerServer::getGoalCheckerType(const std::string & goal_checker_id) const
{
  for (size_t i = 0; i < goal_checker_ids_.size() && i < goal_checker_types_.size(); ++i) {
    if (goal_checker_ids_[i] == goal_checker_id) {
      return goal_checker_types_[i];
    }
  }
  return "unknown";
}

void ControllerServer::setControllerPhase(ControllerPhase phase, const std::string & reason)
{
  if (controller_phase_ == phase) {
    return;
  }
  RCLCPP_INFO(
    get_logger(),
    "Control phase: %s -> %s, reason=%s.",
    controllerPhaseName(controller_phase_),
    controllerPhaseName(phase),
    reason.c_str());
  controller_phase_ = phase;

  // nav_monitor 阶段回显：本函数已是阶段切换的一次性收敛点，直接复述
  if (enable_nav_monitor_echo_ && phase_echo_pub_ && phase_echo_pub_->is_activated()) {
    std_msgs::msg::String msg;
    msg.data = controllerPhaseName(phase);
    phase_echo_pub_->publish(msg);
  }
}

double ControllerServer::getDistanceToGoal(const geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::PoseStamped transformed_end_pose;
  try {
    rclcpp::Duration tolerance(
      rclcpp::Duration::from_seconds(costmap_ros_->getTransformTolerance()));
    if (!nav_2d_utils::transformPose(
      costmap_ros_->getTfBuffer(), pose.header.frame_id,
      end_pose_, transformed_end_pose, tolerance))
    {
      return std::numeric_limits<double>::infinity();
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Goal pose TF failed in staged control: %s",
      ex.what());
    return std::numeric_limits<double>::infinity();
  }

  const double dx = pose.pose.position.x - transformed_end_pose.pose.position.x;
  const double dy = pose.pose.position.y - transformed_end_pose.pose.position.y;
  return std::hypot(dx, dy);
}

bool ControllerServer::getTargetPointHeadingError(
  const geometry_msgs::msg::PoseStamped & pose,
  double & yaw_error)
{
  if (current_path_.poses.empty()) {
    return false;
  }

  geometry_msgs::msg::PoseStamped path_frame_pose;
  try {
    rclcpp::Duration tolerance(
      rclcpp::Duration::from_seconds(costmap_ros_->getTransformTolerance()));
    if (!nav_2d_utils::transformPose(
      costmap_ros_->getTfBuffer(), current_path_.header.frame_id,
      pose, path_frame_pose, tolerance))
    {
      return false;
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Robot pose TF failed in align: %s",
      ex.what());
    return false;
  }

  // 最近点：目标点搜索的起点（同 RPP carrot 选取，只往路径前方找）。
  size_t nearest_idx = 0;
  double nearest_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < current_path_.poses.size(); ++i) {
    const double dx = path_frame_pose.pose.position.x - current_path_.poses[i].pose.position.x;
    const double dy = path_frame_pose.pose.position.y - current_path_.poses[i].pose.position.y;
    const double dist = std::hypot(dx, dy);
    if (dist < nearest_dist) {
      nearest_dist = dist;
      nearest_idx = i;
    }
  }

  // 沿路径累计弧长找到第一个距机器人 >= align_target_distance_ 的目标点
  // （不够长则取终点）——align 朝向盯的是"目标点方位角"而非局部路径切向，
  // 与 RPP carrot 点取向原理一致。
  size_t target_idx = current_path_.poses.size() - 1;
  double accum = 0.0;
  for (size_t i = nearest_idx + 1; i < current_path_.poses.size(); ++i) {
    const auto & a = current_path_.poses[i - 1].pose.position;
    const auto & b = current_path_.poses[i].pose.position;
    accum += std::hypot(b.x - a.x, b.y - a.y);
    if (accum >= align_target_distance_) {
      target_idx = i;
      break;
    }
  }

  const auto & target = current_path_.poses[target_idx].pose.position;
  const double dx = target.x - path_frame_pose.pose.position.x;
  const double dy = target.y - path_frame_pose.pose.position.y;
  if (std::hypot(dx, dy) < 1e-3) {
    return false;  // 目标点与机器人重合（已在终点附近），无法定向
  }
  const double bearing_to_target = std::atan2(dy, dx);
  const double robot_yaw = tf2::getYaw(path_frame_pose.pose.orientation);
  yaw_error = angles::shortest_angular_distance(robot_yaw, bearing_to_target);
  return true;
}

bool ControllerServer::isCostmapCellBlocked(
  nav2_costmap_2d::Costmap2D * costmap, unsigned char cost_threshold, double x, double y)
{
  geometry_msgs::msg::PoseStamped base_sample;
  base_sample.header.frame_id = costmap_ros_->getBaseFrameID();
  base_sample.header.stamp = now();
  base_sample.pose.position.x = x;
  base_sample.pose.position.y = y;
  base_sample.pose.orientation.w = 1.0;

  geometry_msgs::msg::PoseStamped global_sample;
  try {
    rclcpp::Duration tolerance(
      rclcpp::Duration::from_seconds(costmap_ros_->getTransformTolerance()));
    if (!nav_2d_utils::transformPose(
      costmap_ros_->getTfBuffer(), costmap_ros_->getGlobalFrameID(),
      base_sample, global_sample, tolerance))
    {
      return false;
    }
  } catch (const tf2::TransformException &) {
    return false;
  }

  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap->worldToMap(global_sample.pose.position.x, global_sample.pose.position.y, mx, my)) {
    return false;
  }
  return costmap->getCost(mx, my) >= cost_threshold;
}

bool ControllerServer::isObstacleNearFootprint()
{
  auto costmap = costmap_ros_->getCostmap();
  if (costmap == nullptr) {
    return false;
  }

  // 检查区域边界取机器人真实 footprint（前沿/后沿/半宽），而非人工设定值——
  // 机型/footprint 配置变化时自动跟随。
  const auto & footprint = costmap_ros_->getRobotFootprint();
  double front_x = 0.0;
  double rear_x = 0.0;
  double half_width = 0.0;
  for (const auto & pt : footprint) {
    front_x = std::max(front_x, pt.x);
    rear_x = std::min(rear_x, pt.x);
    half_width = std::max(half_width, std::fabs(pt.y));
  }

  const double resolution = std::max(costmap->getResolution(), 0.02);
  const unsigned char cost_threshold = static_cast<unsigned char>(
    std::clamp(prepare_obstacle_cost_threshold_, 0, 255));

  // 前向：footprint 前沿 ~ 前沿 + obstacle_check_distance。
  for (double x = front_x + resolution; x <= front_x + prepare_obstacle_check_distance_;
    x += resolution)
  {
    for (double y = -half_width; y <= half_width; y += resolution) {
      if (isCostmapCellBlocked(costmap, cost_threshold, x, y)) {
        return true;
      }
    }
  }

  // 左右：footprint 全长（后沿~前沿）× 半宽~半宽+side_check_distance，两侧
  // 对称、单参数控制；<=0 关闭（默认开，不需要时显式调 0）。
  if (prepare_side_check_distance_ > 1e-6) {
    for (double x = rear_x; x <= front_x; x += resolution) {
      for (double dy = resolution; dy <= prepare_side_check_distance_; dy += resolution) {
        if (isCostmapCellBlocked(costmap, cost_threshold, x, half_width + dy) ||
          isCostmapCellBlocked(costmap, cost_threshold, x, -half_width - dy))
        {
          return true;
        }
      }
    }
  }

  return false;
}

bool ControllerServer::isRearObstacleDetected()
{
  auto costmap = costmap_ros_->getCostmap();
  if (costmap == nullptr) {
    return false;
  }

  const auto & footprint = costmap_ros_->getRobotFootprint();
  double rear_x = 0.0;
  double half_width = 0.0;
  for (const auto & pt : footprint) {
    rear_x = std::min(rear_x, pt.x);
    half_width = std::max(half_width, std::fabs(pt.y));
  }

  const double resolution = std::max(costmap->getResolution(), 0.02);
  const unsigned char cost_threshold = static_cast<unsigned char>(
    std::clamp(prepare_obstacle_cost_threshold_, 0, 255));

  // 后向：footprint 后沿 ~ 后沿 - obstacle_check_distance（复用前向同一检测
  // 距离参数——"从 footprint 边缘看多远"是同一语义，不因方向另设参数）。
  for (double x = rear_x - resolution; x >= rear_x - prepare_obstacle_check_distance_;
    x -= resolution)
  {
    for (double y = -half_width; y <= half_width; y += resolution) {
      if (isCostmapCellBlocked(costmap, cost_threshold, x, y)) {
        return true;
      }
    }
  }
  return false;
}

bool ControllerServer::shouldRunPrepareBackup()
{
  if (!staged_control_enabled_ || !prepare_enabled_) {
    prepare_backup_active_ = false;
    prepare_check_pending_ = false;
    return false;
  }

  if (prepare_backup_active_) {
    // 每周期先查后方（参考 nav2_behaviors::DriveOnHeading 的运行时 costmap
    // 校验）：身后一旦出现障碍（他人/动态物体移动进来，或距离/半宽估计有
    // 误差），立即中止后退，不等定时/定距条件用完才停。
    if (isRearObstacleDetected()) {
      prepare_backup_active_ = false;
      RCLCPP_WARN(
        get_logger(), "Prepare backup stopped: rear obstacle.");
      return false;
    }
    const double elapsed = (now() - prepare_backup_start_time_).seconds();
    if (prepare_backup_use_distance_) {
      geometry_msgs::msg::PoseStamped pose;
      const bool have_pose = getRobotPose(pose);
      const double traveled = have_pose ?
        std::hypot(
        pose.pose.position.x - prepare_backup_start_pose_.pose.position.x,
        pose.pose.position.y - prepare_backup_start_pose_.pose.position.y) : 0.0;
      // duration 作为定位失效/打滑时的安全兜底，防止距离条件永不满足。
      if (traveled < prepare_backup_distance_ && elapsed < prepare_backup_duration_) {
        return true;
      }
    } else if (elapsed < prepare_backup_duration_) {
      return true;
    }
    prepare_backup_active_ = false;
  }

  if (!prepare_check_pending_) {
    return false;
  }

  prepare_check_pending_ = false;
  if (!isObstacleNearFootprint()) {
    RCLCPP_INFO(
      get_logger(), "Prepare check: no front or side obstacle.");
    return false;
  }

  // 启动前也查一次后方：前方受阻但后方同样无路可退时，不盲目后退，交给
  // 后续 align/track 与全局避障处理（不是 prepare 的职责）。
  if (isRearObstacleDetected()) {
    RCLCPP_WARN(
      get_logger(),
      "Prepare backup skipped: front and rear are blocked.");
    return false;
  }

  prepare_backup_active_ = true;
  prepare_backup_start_time_ = now();
  getRobotPose(prepare_backup_start_pose_);
  RCLCPP_WARN(
    get_logger(),
    "Prepare backup started: front or side obstacle, speed=%.3f m/s, %s.",
    prepare_backup_speed_,
    prepare_backup_use_distance_ ?
    ("distance=" + std::to_string(prepare_backup_distance_) + "m").c_str() :
    ("duration=" + std::to_string(prepare_backup_duration_) + "s").c_str());
  return true;
}

void ControllerServer::resetStagedControlState()
{
  prepare_backup_active_ = false;
  prepare_check_pending_ = true;
  align_done_ = false;
  terminal_brake_latched_ = false;
  terminal_stall_consecutive_cycles_ = 0;
  terminal_stall_recovery_count_ = 0;
  is_collision_stop_active_ = false;
  is_progress_check_paused_for_obstacle_stop_ = false;
  invalidateSpeedPlanningContinuity();
  resetEndLocalizationCapabilities(goal_generation_);
}

nav2_core::EndLocalizationCapability * ControllerServer::getEndLocalizationCapability(
  const std::string & controller_id)
{
  const auto controller = controllers_.find(controller_id);
  if (controller == controllers_.end()) {
    return nullptr;
  }
  return dynamic_cast<nav2_core::EndLocalizationCapability *>(controller->second.get());
}

void ControllerServer::resetEndLocalizationCapabilities(uint64_t goal_generation)
{
  for (auto & controller : controllers_) {
    auto * capability = dynamic_cast<nav2_core::EndLocalizationCapability *>(
      controller.second.get());
    if (capability != nullptr) {
      capability->resetEndLocalization(goal_generation);
    }
  }
}

void ControllerServer::updateEndLocalizationGoal(
  const std::string & controller_id,
  const geometry_msgs::msg::PoseStamped & pose)
{
  auto * capability = getEndLocalizationCapability(controller_id);
  if (capability == nullptr) {
    return;
  }

  const std::string control_frame = pose.header.frame_id.empty() ?
    costmap_ros_->getGlobalFrameID() : pose.header.frame_id;
  geometry_msgs::msg::PoseStamped source_goal = end_pose_;
  source_goal.header.stamp = rclcpp::Time(0);
  geometry_msgs::msg::PoseStamped live_goal;
  rclcpp::Duration tolerance(
    rclcpp::Duration::from_seconds(costmap_ros_->getTransformTolerance()));
  try {
    if (!nav_2d_utils::transformPose(
        costmap_ros_->getTfBuffer(), control_frame, source_goal, live_goal, tolerance))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), kEndLocalizationWarningThrottleMs,
        "末端定位目标TF转换失败：controller=%s，source=%s，target=%s；"
        "本周期保留上一份有效目标。",
        controller_id.c_str(), source_goal.header.frame_id.c_str(), control_frame.c_str());
      return;
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kEndLocalizationWarningThrottleMs,
      "末端定位目标TF异常：controller=%s，source=%s，target=%s，error=%s；"
      "本周期保留上一份有效目标。",
      controller_id.c_str(), source_goal.header.frame_id.c_str(), control_frame.c_str(),
      ex.what());
    return;
  }

  live_goal.header.frame_id = control_frame;
  capability->setEndLocalizationGoal(live_goal, goal_generation_);
}

bool ControllerServer::getEndLocalizationSnapshot(
  const std::string & controller_id,
  nav2_core::EndLocalizationSnapshot & snapshot)
{
  auto * capability = getEndLocalizationCapability(controller_id);
  if (capability == nullptr) {
    return false;
  }
  snapshot = capability->getEndLocalizationSnapshot();
  return snapshot.goal_generation == goal_generation_;
}

bool ControllerServer::shouldUseApproachController(
  double distance_to_goal,
  bool & position_reference_latched)
{
  nav2_core::EndLocalizationSnapshot end_localization_snapshot;
  position_reference_latched = getEndLocalizationSnapshot(
      approach_controller_, end_localization_snapshot) &&
    end_localization_snapshot.position_goal_latched;
  // 一旦位置参考已锁存在Odom，短时目标无效或map修正都不能切回路径跟踪器。
  return position_reference_latched || distance_to_goal <= approach_distance_;
}

bool ControllerServer::applyEndLocalizationGoalToGoalChecker(
  const std::string & controller_id,
  geometry_msgs::msg::PoseStamped & goal_pose)
{
  nav2_core::EndLocalizationSnapshot end_localization_snapshot;
  if (!getEndLocalizationSnapshot(controller_id, end_localization_snapshot) ||
    !end_localization_snapshot.effective_goal_valid)
  {
    return false;
  }

  geometry_msgs::msg::PoseStamped effective_goal = end_localization_snapshot.effective_goal;
  if (effective_goal.header.frame_id == goal_pose.header.frame_id) {
    goal_pose.pose = effective_goal.pose;
    return true;
  }

  effective_goal.header.stamp = rclcpp::Time(0);
  geometry_msgs::msg::PoseStamped transformed_effective_goal;
  rclcpp::Duration tolerance(
    rclcpp::Duration::from_seconds(costmap_ros_->getTransformTolerance()));
  if (nav_2d_utils::transformPose(
      costmap_ros_->getTfBuffer(), goal_pose.header.frame_id,
      effective_goal, transformed_effective_goal, tolerance))
  {
    goal_pose.pose = transformed_effective_goal.pose;
    return true;
  }

  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), kEndLocalizationWarningThrottleMs,
    "GoalChecker无法转换PID末端锁存目标：source=%s，target=%s；"
    "本周期使用实时目标。",
    effective_goal.header.frame_id.c_str(), goal_pose.header.frame_id.c_str());
  return false;
}

geometry_msgs::msg::TwistStamped ControllerServer::computeControllerVelocity(
  const std::string & controller_id,
  const geometry_msgs::msg::PoseStamped & pose,
  const nav_2d_msgs::msg::Twist2D & twist)
{
  active_controller_for_log_ = controller_id;
  // 本周期轨迹只发给真正要跑的这个 controller（见 runPlanningPipeline 说明）。
  if (trajectory_dispatched_to_ != controller_id && !current_trajectory_.points.empty()) {
    controllers_[controller_id]->setTrajectory(current_trajectory_);
    trajectory_dispatched_to_ = controller_id;
  }
  updateEndLocalizationGoal(controller_id, pose);
  return controllers_[controller_id]->computeVelocityCommands(
    pose,
    nav_2d_utils::twist2Dto3D(twist),
    goal_checkers_[current_goal_checker_].get());
}

geometry_msgs::msg::TwistStamped ControllerServer::computeStagedVelocity(
  const geometry_msgs::msg::PoseStamped & pose,
  const nav_2d_msgs::msg::Twist2D & twist)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
  cmd_vel.header.stamp = now();

  if (!staged_control_enabled_) {
    setControllerPhase(ControllerPhase::TRACK, "staged control off");
    active_controller_for_log_ = current_controller_;
    return computeControllerVelocity(current_controller_, pose, twist);
  }

  if (shouldRunPrepareBackup()) {
    setControllerPhase(ControllerPhase::PREPARE, "front obstacle");
    active_controller_for_log_ = "direct_prepare_backup";
    cmd_vel.twist.linear.x = prepare_backup_speed_;
    return cmd_vel;
  }

  const double distance_to_goal = getDistanceToGoal(pose);
  bool position_reference_latched = false;
  const bool in_approach = shouldUseApproachController(
    distance_to_goal, position_reference_latched);
  if (in_approach) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 500,
      "Control choice: approach, goal_distance=%.3f, threshold=%.3f, "
      "position_reference_latched=%s, controller='%s'.",
      distance_to_goal, approach_distance_,
      position_reference_latched ? "true" : "false", approach_controller_.c_str());
    setControllerPhase(ControllerPhase::APPROACH, "near goal");
    active_controller_for_log_ = approach_controller_;
    return computeControllerVelocity(approach_controller_, pose, twist);
  }

  // align 只在本次导航"未完成过"时评估——一旦收敛转入 track，中途朝向
  // 误差再变大也不回退进 align，交给 track 控制器自身的朝向纠偏（RPP/MPPI
  // 均有内建机制）；align 是"出发前对齐一次"，不是持续兜底。
  if (!align_done_) {
    double heading_error = 0.0;
    const bool has_target_heading = getTargetPointHeadingError(pose, heading_error);
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 500,
      "Control choice: goal_distance=%.3f > %.3f, heading_valid=%s, "
      "heading_error=%.3f, align_limit=%.3f, target_distance=%.3f.",
      distance_to_goal,
      approach_distance_,
      has_target_heading ? "true" : "false",
      has_target_heading ? heading_error : 0.0,
      align_heading_tolerance_,
      align_target_distance_);
    if (has_target_heading && std::fabs(heading_error) > align_heading_tolerance_) {
      setControllerPhase(ControllerPhase::ALIGN, "heading error above limit");
      active_controller_for_log_ = "direct_align";
      const double abs_cmd = std::clamp(
        std::fabs(heading_error) * align_angular_kp_,
        align_min_angular_velocity_,
        align_max_angular_velocity_);
      cmd_vel.twist.angular.z = std::copysign(abs_cmd, heading_error);
      return cmd_vel;
    }
    // 收敛，或无法计算朝向（如路径过短）：本次导航不再进入 align。
    align_done_ = true;
  }

  setControllerPhase(ControllerPhase::TRACK, "align done");
  active_controller_for_log_ = track_controller_;
  return computeControllerVelocity(track_controller_, pose, twist);
}

bool ControllerServer::nearestPlannedSpeed(
  const geometry_msgs::msg::PoseStamped & pose, double & desired_v,
  size_t * nearest_index)
{
  const auto & pts = current_trajectory_.points;
  if (pts.empty()) {
    return false;
  }
  std::string traj_frame = current_trajectory_.header.frame_id;
  if (traj_frame.empty() && !pts.front().pose.header.frame_id.empty()) {
    traj_frame = pts.front().pose.header.frame_id;
  }
  geometry_msgs::msg::PoseStamped query = pose;
  if (!traj_frame.empty() && pose.header.frame_id != traj_frame) {
    geometry_msgs::msg::PoseStamped source_pose = pose;
    source_pose.header.stamp = rclcpp::Time(0);
    rclcpp::Duration tolerance(
      rclcpp::Duration::from_seconds(costmap_ros_->getTransformTolerance()));
    if (!nav_2d_utils::transformPose(
        costmap_ros_->getTfBuffer(), traj_frame, source_pose, query, tolerance))
    {
      return false;
    }
  }
  double best_d2 = std::numeric_limits<double>::max();
  size_t best_i = 0;
  for (size_t i = 0; i < pts.size(); ++i) {
    const double dx = pts[i].pose.pose.position.x - query.pose.position.x;
    const double dy = pts[i].pose.pose.position.y - query.pose.position.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_i = i;
    }
  }
  desired_v = pts[best_i].longitudinal_velocity;
  if (nearest_index != nullptr) {
    *nearest_index = best_i;
  }
  return true;
}

size_t ControllerServer::findClosestPoseIndex(
  const geometry_msgs::msg::PoseStamped & pose_in_path,
  const nav_msgs::msg::Path & path) const
{
  size_t closest_pose_idx = 0;
  double min_distance = std::numeric_limits<double>::max();
  for (size_t pose_idx = 0; pose_idx < path.poses.size(); ++pose_idx) {
    const double distance = nav2_util::geometry_utils::euclidean_distance(
      pose_in_path, path.poses[pose_idx]);
    if (distance < min_distance) {
      min_distance = distance;
      closest_pose_idx = pose_idx;
    }
  }
  return closest_pose_idx;
}

bool ControllerServer::shouldPauseProgressCheckForObstacleStop(
  const geometry_msgs::msg::PoseStamped & pose)
{
  constexpr double kPlannedStopVelocityEpsilon = 1e-3;
  if (!speed_plan_cycle_.active || !speed_plan_cycle_.planning_succeeded ||
    !speed_plan_cycle_.terminal_valid ||
    speed_plan_cycle_.terminal.terminal_branch !=
    nav2_core::SpeedPlanningTerminalBranch::PATH_OBSTACLE_STOP ||
    speed_plan_cycle_.terminal.terminal_infeasible)
  {
    return false;
  }

  // 首点是当前实测状态，静止起步时也可能为零。只有当前位置之后整条轨迹
  // 都保持零速，才说明这是“已完成计划停障”，而不是准备重新起步。
  double planned_speed = 0.0;
  size_t nearest_index = 0;
  if (!nearestPlannedSpeed(pose, planned_speed, &nearest_index) ||
    !std::isfinite(planned_speed))
  {
    return false;
  }
  for (size_t index = nearest_index; index < current_trajectory_.points.size(); ++index) {
    const double speed = current_trajectory_.points[index].longitudinal_velocity;
    if (!std::isfinite(speed) || std::fabs(speed) > kPlannedStopVelocityEpsilon) {
      return false;
    }
  }
  return true;
}

bool ControllerServer::checkProgress(geometry_msgs::msg::PoseStamped & pose)
{
  // 正常停障和碰撞急停都不应被误判为“没有进展”。
  const bool is_planned_obstacle_stop =
    !is_collision_stop_active_ && shouldPauseProgressCheckForObstacleStop(pose);
  if (is_collision_stop_active_ || is_planned_obstacle_stop) {
    if (!is_progress_check_paused_for_obstacle_stop_) {
      // 进入时清除旧基线，防止障碍解除后继承停障前已累计的超时时间。
      progress_checker_->reset();
      is_progress_check_paused_for_obstacle_stop_ = true;
      if (is_collision_stop_active_) {
        RCLCPP_INFO(
          get_logger(),
          "Hard stop active. Progress check paused.");
      } else {
        RCLCPP_INFO(
          get_logger(),
          "Planned obstacle stop. Progress check paused: obstacle_arc=%.3f m.",
          speed_plan_cycle_.terminal.first_obstacle_arc);
      }
    }
    return true;
  }

  if (is_progress_check_paused_for_obstacle_stop_) {
    progress_checker_->reset();
    is_progress_check_paused_for_obstacle_stop_ = false;
    RCLCPP_INFO(get_logger(), "Obstacle stop ended. Progress check restarted.");
  }
  return progress_checker_->check(pose);
}

void ControllerServer::computeAndPublishVelocity()
{
  constexpr double kSlowTimingWarnMs = 100.0;
  const auto publish_control_failure = [this](const std::string & result) {
      publishSpeedPlanDiagnostics(
        false, false, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(), result);
    };
  geometry_msgs::msg::PoseStamped pose;
  auto t_total_start = std::chrono::high_resolution_clock::now();

  auto t_pose_start = std::chrono::high_resolution_clock::now();
  if (!getRobotPose(pose)) {
    publish_control_failure("pose_lookup_failed");
    throw nav2_core::PlannerException(
            "Robot pose not found. Check TF, localization, and costmap frames.");
  }
  auto t_pose_end = std::chrono::high_resolution_clock::now();
  auto pose_ms = std::chrono::duration<double, std::milli>(t_pose_end - t_pose_start).count();

  auto t_progress_start = std::chrono::high_resolution_clock::now();
  if (!checkProgress(pose)) {
    publish_control_failure("progress_failed");
    throw nav2_core::PlannerException(
            "Progress check failed. Robot did not move enough in time.");
  }
  auto t_progress_end = std::chrono::high_resolution_clock::now();
  auto progress_ms = std::chrono::duration<double, std::milli>(t_progress_end - t_progress_start).count();

  auto t_odom_start = std::chrono::high_resolution_clock::now();
  const nav_2d_msgs::msg::Twist2D raw_twist = odom_sub_->getTwist();
  nav_2d_msgs::msg::Twist2D twist = getThresholdedTwist(raw_twist);
  auto t_odom_end = std::chrono::high_resolution_clock::now();
  auto odom_ms = std::chrono::duration<double, std::milli>(t_odom_end - t_odom_start).count();

  geometry_msgs::msg::TwistStamped cmd_vel_2d;
  bool control_succeeded = false;
  bool controller_target_valid = false;
  // 标记本周期是否因碰撞保持零速，用于发布正确的诊断状态。
  bool is_collision_stop_wait = false;
  double controller_target_v = std::numeric_limits<double>::quiet_NaN();
  double controller_target_a = std::numeric_limits<double>::quiet_NaN();
  double controller_target_t = std::numeric_limits<double>::quiet_NaN();

  auto t_compute_start = std::chrono::high_resolution_clock::now();
  try {
    cmd_vel_2d = computeStagedVelocity(pose, twist);
    last_valid_cmd_time_ = now();
    control_succeeded = true;
    if (is_collision_stop_active_) {
      is_collision_stop_active_ = false;
      RCLCPP_INFO(
        get_logger(),
        "Collision risk cleared. Hard stop ended.");
    }
  } catch (const nav2_core::CollisionException &) {
    // 碰撞属于可恢复的安全停车：保持 FollowPath，风险解除后继续控制。
    invalidateSpeedPlanningContinuity();
    is_collision_stop_wait = true;
    if (!is_collision_stop_active_) {
      is_collision_stop_active_ = true;
      RCLCPP_WARN(
        get_logger(),
        "Collision ahead. Hard stop active.");
      publishHardStopVelocity();
    }
    cmd_vel_2d = geometry_msgs::msg::TwistStamped();
    cmd_vel_2d.header.frame_id = costmap_ros_->getBaseFrameID();
    cmd_vel_2d.header.stamp = now();
  } catch (nav2_core::PlannerException & e) {
    invalidateSpeedPlanningContinuity();
    if (failure_tolerance_ > 0 || failure_tolerance_ == -1.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Controller command failed: %s", e.what());
      cmd_vel_2d.twist.angular.x = 0;
      cmd_vel_2d.twist.angular.y = 0;
      cmd_vel_2d.twist.angular.z = 0;
      cmd_vel_2d.twist.linear.x = 0;
      cmd_vel_2d.twist.linear.y = 0;
      cmd_vel_2d.twist.linear.z = 0;
      cmd_vel_2d.header.frame_id = costmap_ros_->getBaseFrameID();
      cmd_vel_2d.header.stamp = now();
      active_controller_for_log_ = "failure_tolerance_zero_cmd";
      if ((now() - last_valid_cmd_time_).seconds() > failure_tolerance_ &&
        failure_tolerance_ != -1.0)
      {
        publish_control_failure("controller_patience_exceeded");
        throw nav2_core::PlannerException(
                "Controller timeout. No valid command within failure_tolerance.");
      }
    } else {
      publish_control_failure("controller_exception");
      throw nav2_core::PlannerException(e.what());
    }
  } catch (const std::exception &) {
    invalidateSpeedPlanningContinuity();
    publish_control_failure("control_std_exception");
    throw;
  }
  if (control_succeeded &&
    (controller_phase_ == ControllerPhase::TRACK ||
    controller_phase_ == ControllerPhase::APPROACH))
  {
    const auto controller = controllers_.find(active_controller_for_log_);
    if (controller != controllers_.end()) {
      auto * receipt_capability = dynamic_cast<
        nav2_core::TrajectoryTrackingReceiptCapability *>(controller->second.get());
      if (receipt_capability != nullptr) {
        controller_target_valid = receipt_capability->getLastTrajectoryTrackingReceipt(
          controller_target_v, controller_target_a, controller_target_t);
      } else {
        controller_target_valid = controller->second->getLastTrajectoryTrackingTarget(
          controller_target_v, controller_target_a);
      }
    }
  }
  const bool terminal_stall_recovery_triggered = updateTerminalStallRecovery(
    twist, controller_target_valid, controller_target_v, controller_target_a);
  if (!terminal_stall_recovery_triggered) {
    updateSpeedPlanningContinuityAfterControl(
      control_succeeded, controller_target_valid, controller_target_v, controller_target_a,
      controller_target_t);
  }
  publishSpeedPlanDiagnostics(
    control_succeeded, controller_target_valid, controller_target_v, controller_target_a,
    control_succeeded ? "success" :
    (is_collision_stop_wait ? "collision_stop_wait" : "control_failed"));
  auto t_compute_end = std::chrono::high_resolution_clock::now();
  auto compute_ms = std::chrono::duration<double, std::milli>(t_compute_end - t_compute_start).count();

  // nav_monitor 期望速度回显：当前位置最近轨迹点的规划速度
  if (enable_nav_monitor_echo_ && desired_vel_echo_pub_ &&
    desired_vel_echo_pub_->is_activated())
  {
    double echo_desired_v = 0.0;
    nearestPlannedSpeed(pose, echo_desired_v);
    geometry_msgs::msg::Twist desired_msg;
    desired_msg.linear.x = echo_desired_v;
    desired_vel_echo_pub_->publish(desired_msg);
  }
  if (enable_nav_monitor_echo_ && controller_target_valid && controller_target_echo_pub_ &&
    controller_target_echo_pub_->is_activated())
  {
    geometry_msgs::msg::Vector3Stamped target_msg;
    target_msg.header.stamp = now();
    target_msg.header.frame_id = current_trajectory_.header.frame_id;
    target_msg.vector.x = controller_target_v;
    target_msg.vector.y = controller_target_a;
    controller_target_echo_pub_->publish(target_msg);
  }

  auto t_feedback_start = std::chrono::high_resolution_clock::now();
  std::shared_ptr<Action::Feedback> feedback = std::make_shared<Action::Feedback>();
  feedback->speed = std::hypot(cmd_vel_2d.twist.linear.x, cmd_vel_2d.twist.linear.y);

  // noted by hz
  // 找到最近的路径点，并计算剩余路径长度，然后作为 feedback 发布。
  // getRobotPose() 返回的是 local costmap global frame（当前配置为 odom），而
  // current_path_ 通常位于 map。最近点和剩余路径长度必须在同一坐标系计算，
  // 否则会把路径末点误判为最近点，使 distance_to_goal 长期为 0。
  geometry_msgs::msg::PoseStamped pose_in_path = pose;
  bool pose_in_path_valid = true;
  if (!current_path_.header.frame_id.empty() &&
    pose.header.frame_id != current_path_.header.frame_id)
  {
    try {
      rclcpp::Duration tolerance(
        rclcpp::Duration::from_seconds(costmap_ros_->getTransformTolerance()));
      // 使用 TF 队列中的最新变换，避免 pose 的采样时间戳稍早于 map->odom
      // 缓存区间时使反馈距离失效。
      geometry_msgs::msg::PoseStamped latest_tf_pose = pose;
      latest_tf_pose.header.stamp = rclcpp::Time(0);
      pose_in_path_valid = nav_2d_utils::transformPose(
        costmap_ros_->getTfBuffer(), current_path_.header.frame_id,
        latest_tf_pose, pose_in_path, tolerance);
    } catch (const tf2::TransformException & ex) {
      pose_in_path_valid = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Unable to transform robot pose from '%s' to path frame '%s' for feedback metrics: %s",
        pose.header.frame_id.c_str(), current_path_.header.frame_id.c_str(), ex.what());
    }
  }

  const size_t closest_pose_idx =
    pose_in_path_valid && !current_path_.poses.empty() ?
    findClosestPoseIndex(pose_in_path, current_path_) : 0u;
  feedback->distance_to_goal = pose_in_path_valid ?
    nav2_util::geometry_utils::calculate_path_length(current_path_, closest_pose_idx) :
    std::numeric_limits<double>::quiet_NaN();
  action_server_->publish_feedback(feedback);

  auto t_feedback_end = std::chrono::high_resolution_clock::now();
  auto feedback_ms = std::chrono::duration<double, std::milli>(t_feedback_end - t_feedback_start).count();

  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 500,
    "Control command: phase=%s, controller=%s, goal_checker=%s, "
    "vx=%.3f, vy=%.3f, wz=%.3f.",
    controllerPhaseName(controller_phase_),
    active_controller_for_log_.c_str(),
    current_goal_checker_.c_str(),
    cmd_vel_2d.twist.linear.x,
    cmd_vel_2d.twist.linear.y,
    cmd_vel_2d.twist.angular.z);

  auto t_publish_start = std::chrono::high_resolution_clock::now();
  publishVelocity(cmd_vel_2d);
  auto t_publish_end = std::chrono::high_resolution_clock::now();
  auto publish_ms = std::chrono::duration<double, std::milli>(t_publish_end - t_publish_start).count();

  auto t_total_end = std::chrono::high_resolution_clock::now();
  auto total_ms = std::chrono::duration<double, std::milli>(t_total_end - t_total_start).count();
  auto measured_ms = pose_ms + progress_ms + odom_ms + compute_ms + feedback_ms + publish_ms;
  auto unaccounted_ms = total_ms - measured_ms;

  if (total_ms >= kSlowTimingWarnMs) {
    RCLCPP_WARN(
      get_logger(),
      "Slow control (ms): total=%.3f pose=%.3f progress=%.3f odom=%.3f "
      "compute=%.3f feedback=%.3f publish=%.3f unaccounted=%.3f",
      total_ms, pose_ms, progress_ms, odom_ms, compute_ms, feedback_ms, publish_ms,
      unaccounted_ms);
  }
}

void ControllerServer::updateGlobalPath()
{
  if (action_server_->is_preempt_requested()) {
    auto goal = action_server_->accept_pending_goal();
    std::string requested_controller;
    if (!findControllerId(goal->controller_id, requested_controller)) {
      RCLCPP_INFO(
        get_logger(), "FollowPath stopped: controller '%s' is not valid.",
        goal->controller_id.c_str());
      action_server_->terminate_current();
      return;
    }

    std::string requested_goal_checker;
    if (!findGoalCheckerId(goal->goal_checker_id, requested_goal_checker)) {
      RCLCPP_INFO(
        get_logger(), "FollowPath stopped: goal checker '%s' is not valid.",
        goal->goal_checker_id.c_str());
      action_server_->terminate_current();
      return;
    }

    const bool same_controller = requested_controller == current_controller_;
    const bool same_goal_checker = requested_goal_checker == current_goal_checker_;
    const bool same_goal = isSamePathGoal(goal->path);
    if (same_goal && (!same_controller || !same_goal_checker)) {
      RCLCPP_INFO(
        get_logger(),
        "Path treated as a new goal: same_controller=%s, same_goal_checker=%s.",
        same_controller ? "true" : "false",
        same_goal_checker ? "true" : "false");
    }

    current_controller_ = requested_controller;
    current_goal_checker_ = requested_goal_checker;
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "FollowPath update accepted.");
    const auto update_type =
      same_controller && same_goal_checker && same_goal ?
      PathUpdateType::SAME_GOAL_GEOMETRY : PathUpdateType::NEW_GOAL;
    setPlannerPath(goal->path, update_type);
  }
}

void ControllerServer::publishVelocity(const geometry_msgs::msg::TwistStamped & velocity)
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>(velocity.twist);
  if (vel_publisher_->is_activated() && vel_publisher_->get_subscription_count() > 0) {
    vel_publisher_->publish(std::move(cmd_vel));
  }
}

void ControllerServer::publishZeroVelocity()
{
  invalidateSpeedPlanningContinuity();
  geometry_msgs::msg::TwistStamped velocity;
  velocity.twist.angular.x = 0;
  velocity.twist.angular.y = 0;
  velocity.twist.angular.z = 0;
  velocity.twist.linear.x = 0;
  velocity.twist.linear.y = 0;
  velocity.twist.linear.z = 0;
  velocity.header.frame_id = costmap_ros_->getBaseFrameID();
  velocity.header.stamp = now();

  if (odom_sub_) {
    const auto actual_twist = getThresholdedTwist(odom_sub_->getTwist());
    const size_t subscriber_count = vel_publisher_ ?
      vel_publisher_->get_subscription_count() : 0;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Zero speed requested: odom_vx=%.3f m/s, odom_vy=%.3f m/s, odom_wz=%.3f rad/s, "
      "subscribers=%zu.",
      actual_twist.x, actual_twist.y, actual_twist.theta, subscriber_count);
  }
  publishVelocity(velocity);
}

void ControllerServer::publishHardStopVelocity()
{
  if (hard_stop_publisher_ && hard_stop_publisher_->is_activated()) {
    hard_stop_publisher_->publish(std::make_unique<std_msgs::msg::Empty>());
    RCLCPP_INFO(
      get_logger(),
      "Hard stop sent: topic='%s', subscribers=%zu.",
      hard_stop_topic_.c_str(), hard_stop_publisher_->get_subscription_count());
  } else {
    RCLCPP_ERROR(
      get_logger(), "Hard stop publisher is off. Requesting zero speed.");
  }
  publishZeroVelocity();
}

bool ControllerServer::isGoalReached()
{
  geometry_msgs::msg::PoseStamped pose;

  if (!getRobotPose(pose)) {
    return false;
  }

  nav_2d_msgs::msg::Twist2D twist = getThresholdedTwist(odom_sub_->getTwist());
  geometry_msgs::msg::Twist velocity = nav_2d_utils::twist2Dto3D(twist);

  geometry_msgs::msg::PoseStamped transformed_end_pose;
  rclcpp::Duration tolerance(rclcpp::Duration::from_seconds(costmap_ros_->getTransformTolerance()));
  if (!nav_2d_utils::transformPose(
      costmap_ros_->getTfBuffer(), costmap_ros_->getGlobalFrameID(),
      end_pose_, transformed_end_pose, tolerance))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Goal pose TF failed in goal check.");
    return false;
  }

  const std::string localization_controller = staged_control_enabled_ ?
    approach_controller_ : current_controller_;
  applyEndLocalizationGoalToGoalChecker(localization_controller, transformed_end_pose);

  const bool goal_reached = goal_checkers_[current_goal_checker_]->isGoalReached(
    pose.pose, transformed_end_pose.pose,
    velocity);

  geometry_msgs::msg::Pose goal_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  const bool has_tolerances =
    goal_checkers_[current_goal_checker_]->getTolerances(goal_tolerance, vel_tolerance);

  const double goal_yaw = tf2::getYaw(transformed_end_pose.pose.orientation);
  const double robot_yaw = tf2::getYaw(pose.pose.orientation);
  const double dx = pose.pose.position.x - transformed_end_pose.pose.position.x;
  const double dy = pose.pose.position.y - transformed_end_pose.pose.position.y;
  const double lon_err = std::cos(goal_yaw) * dx + std::sin(goal_yaw) * dy;
  const double lat_err = -std::sin(goal_yaw) * dx + std::cos(goal_yaw) * dy;
  const double yaw_err = angles::shortest_angular_distance(robot_yaw, goal_yaw);
  const double yaw_tolerance = has_tolerances ? tf2::getYaw(goal_tolerance.orientation) :
    std::numeric_limits<double>::quiet_NaN();

  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 500,
    "Goal check: id=%s type=%s reached=%s "
    "goal=(frame=%s x=%.3f y=%.3f yaw=%.3f) "
    "robot=(frame=%s x=%.3f y=%.3f yaw=%.3f) "
    "error=(lon=%.4f lat=%.4f yaw=%.4f dist=%.4f) "
    "tolerance_valid=%s tolerance=(lon=%.4f lat=%.4f yaw=%.4f v=%.4f w=%.4f) "
    "odom=(v=%.4f w=%.4f)",
    current_goal_checker_.c_str(), getGoalCheckerType(current_goal_checker_).c_str(),
    goal_reached ? "true" : "false",
    transformed_end_pose.header.frame_id.c_str(),
    transformed_end_pose.pose.position.x,
    transformed_end_pose.pose.position.y,
    goal_yaw,
    pose.header.frame_id.c_str(),
    pose.pose.position.x,
    pose.pose.position.y,
    robot_yaw,
    lon_err, lat_err, yaw_err, std::hypot(lon_err, lat_err),
    has_tolerances ? "true" : "false",
    has_tolerances ? std::fabs(goal_tolerance.position.x) :
    std::numeric_limits<double>::quiet_NaN(),
    has_tolerances ? std::fabs(goal_tolerance.position.y) :
    std::numeric_limits<double>::quiet_NaN(),
    has_tolerances ? std::fabs(yaw_tolerance) : std::numeric_limits<double>::quiet_NaN(),
    has_tolerances ? std::fabs(vel_tolerance.linear.x) :
    std::numeric_limits<double>::quiet_NaN(),
    has_tolerances ? std::fabs(vel_tolerance.angular.z) :
    std::numeric_limits<double>::quiet_NaN(),
    std::hypot(velocity.linear.x, velocity.linear.y),
    std::fabs(velocity.angular.z));

  return goal_reached;
}

bool ControllerServer::getRobotPose(geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!costmap_ros_->getRobotPose(current_pose)) {
    return false;
  }
  pose = current_pose;
  return true;
}

void ControllerServer::speedLimitCallback(const nav2_msgs::msg::SpeedLimit::SharedPtr msg)
{
  const bool speed_limit_changed = !external_speed_limit_valid_ ||
    std::fabs(last_external_speed_limit_ - msg->speed_limit) > 1e-9 ||
    last_external_speed_limit_percentage_ != msg->percentage;
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->setSpeedLimit(msg->speed_limit, msg->percentage);
  }
  for (auto & lp : local_planners_) {
    lp.second->setSpeedLimit(msg->speed_limit, msg->percentage);
  }
  // 外部限速同步进速度规划包络（下次 computeSpeedPlan 生效）。
  for (auto & sp : speed_planners_) {
    sp.second->setSpeedLimit(msg->speed_limit, msg->percentage);
  }
  if (speed_limit_changed) {
    // A preserved prefix was generated under the previous limit and cannot be
    // retroactively lowered. Drop it so the very next cycle replans the complete
    // remaining path under the new external speed envelope. Repeated messages
    // with the same limit do not disable stitching continuously.
    invalidateSpeedPlanningContinuity();
  }
  external_speed_limit_valid_ = true;
  last_external_speed_limit_ = msg->speed_limit;
  last_external_speed_limit_percentage_ = msg->percentage;
}

rcl_interfaces::msg::SetParametersResult
ControllerServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    // Plugin parameters handle their own updates, but these namespaces belong to this server.
    if (name.find('.') != std::string::npos &&
      name.rfind("staged_control.", 0) != 0)
    {
      continue;
    }

    if (!dynamic_params_lock_.try_lock()) {
      RCLCPP_WARN(
        get_logger(),
        "Parameters cannot change while the controller is running.");
      result.successful = false;
      result.reason =
        "Parameters cannot change while the controller is running.";
      return result;
    }

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "controller_frequency") {
        controller_frequency_ = parameter.as_double();
      } else if (name == "min_x_velocity_threshold") {
        min_x_velocity_threshold_ = parameter.as_double();
      } else if (name == "min_y_velocity_threshold") {
        min_y_velocity_threshold_ = parameter.as_double();
      } else if (name == "min_theta_velocity_threshold") {
        min_theta_velocity_threshold_ = parameter.as_double();
      } else if (name == "failure_tolerance") {
        failure_tolerance_ = parameter.as_double();
      } else if (name == "staged_control.approach_distance") {
        approach_distance_ = parameter.as_double();
      } else if (name == "staged_control.align_heading_tolerance") {
        align_heading_tolerance_ = parameter.as_double();
      } else if (name == "staged_control.align_angular_kp") {
        align_angular_kp_ = parameter.as_double();
      } else if (name == "staged_control.align_max_angular_velocity") {
        align_max_angular_velocity_ = parameter.as_double();
      } else if (name == "staged_control.align_min_angular_velocity") {
        align_min_angular_velocity_ = parameter.as_double();
      } else if (name == "staged_control.prepare.obstacle_check_distance") {
        prepare_obstacle_check_distance_ = parameter.as_double();
      } else if (name == "staged_control.prepare.side_check_distance") {
        prepare_side_check_distance_ = parameter.as_double();
      } else if (name == "staged_control.prepare.backup_speed") {
        prepare_backup_speed_ = parameter.as_double();
      } else if (name == "staged_control.prepare.backup_duration") {
        prepare_backup_duration_ = parameter.as_double();
      } else if (name == "staged_control.prepare.backup_distance") {
        prepare_backup_distance_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == "current_controller") {
        const auto requested_controller = parameter.as_string();
        if (controllers_.find(requested_controller) == controllers_.end()) {
          RCLCPP_WARN(
            get_logger(),
            "Parameter rejected: current_controller='%s'. Available: %s",
            requested_controller.c_str(), controller_ids_concat_.c_str());
          dynamic_params_lock_.unlock();
          result.successful = false;
          result.reason = "Requested current_controller does not exist";
          return result;
        }
        default_controller_ = requested_controller;
      } else if (name == "current_goal_checker") {
        const auto requested_goal_checker = parameter.as_string();
        if (goal_checkers_.find(requested_goal_checker) == goal_checkers_.end()) {
          RCLCPP_WARN(
            get_logger(),
            "Parameter rejected: current_goal_checker='%s'. Available: %s",
            requested_goal_checker.c_str(), goal_checker_ids_concat_.c_str());
          dynamic_params_lock_.unlock();
          result.successful = false;
          result.reason = "Requested current_goal_checker does not exist";
          return result;
        }
        default_goal_checker_ = requested_goal_checker;
      } else if (name == "staged_control.track_controller") {
        const auto requested_controller = parameter.as_string();
        if (controllers_.find(requested_controller) == controllers_.end()) {
          RCLCPP_WARN(
            get_logger(),
            "Parameter rejected: staged_control.track_controller='%s'. Available: %s",
            requested_controller.c_str(), controller_ids_concat_.c_str());
          dynamic_params_lock_.unlock();
          result.successful = false;
          result.reason = "Requested staged_control.track_controller does not exist";
          return result;
        }
        track_controller_ = requested_controller;
      } else if (name == "staged_control.approach_controller") {
        const auto requested_controller = parameter.as_string();
        if (controllers_.find(requested_controller) == controllers_.end()) {
          RCLCPP_WARN(
            get_logger(),
            "Parameter rejected: staged_control.approach_controller='%s'. Available: %s",
            requested_controller.c_str(), controller_ids_concat_.c_str());
          dynamic_params_lock_.unlock();
          result.successful = false;
          result.reason = "Requested staged_control.approach_controller does not exist";
          return result;
        }
        approach_controller_ = requested_controller;
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == "staged_control.prepare.obstacle_cost_threshold") {
        prepare_obstacle_cost_threshold_ = static_cast<int>(parameter.as_int());
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == "staged_control.enabled") {
        staged_control_enabled_ = parameter.as_bool();
        prepare_backup_active_ = false;
      } else if (name == "staged_control.prepare.enabled") {
        prepare_enabled_ = parameter.as_bool();
        prepare_backup_active_ = false;
      } else if (name == "staged_control.prepare.backup_use_distance") {
        prepare_backup_use_distance_ = parameter.as_bool();
      }
    }

    dynamic_params_lock_.unlock();
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_controller

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_controller::ControllerServer)
