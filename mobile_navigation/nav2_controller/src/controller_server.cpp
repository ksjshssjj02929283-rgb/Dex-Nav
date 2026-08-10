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
#include <algorithm>
#include <cstdio>
#include <vector>
#include <memory>
#include <string>
#include <utility>
#include <limits>
#include <cmath>

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
  RCLCPP_INFO(get_logger(), "Creating controller server");

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
  declare_parameter("initial_motion.failsafe_deviation_cycles", 5);
  declare_parameter("initial_motion.goal_position_tolerance", 0.05);
  declare_parameter("initial_motion.goal_yaw_tolerance", 0.2);
  declare_parameter("initial_motion.branch_heading_tolerance", 0.7853981634);
  // nav_monitor 任务回显：默认关闭，零开销
  declare_parameter("enable_nav_monitor_echo", false);
  declare_parameter("min_x_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_y_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_theta_velocity_threshold", rclcpp::ParameterValue(0.0001));

  declare_parameter("speed_limit_topic", rclcpp::ParameterValue("speed_limit"));
  declare_parameter("hard_stop_topic", rclcpp::ParameterValue("cmd_vel_hard_stop"));

  declare_parameter("failure_tolerance", rclcpp::ParameterValue(0.0));
  declare_parameter("near_goal_progress.enabled", rclcpp::ParameterValue(true));
  declare_parameter(
    "near_goal_progress.hold_position_when_goal_pose_reached", rclcpp::ParameterValue(false));
  declare_parameter("near_goal_progress.distance", rclcpp::ParameterValue(0.25));
  declare_parameter(
    "near_goal_progress.required_movement_radius", rclcpp::ParameterValue(0.01));
  declare_parameter(
    "near_goal_progress.required_movement_angle", rclcpp::ParameterValue(0.005));
  declare_parameter(
    "near_goal_progress.movement_time_allowance", rclcpp::ParameterValue(8.0));
  declare_parameter(
    "near_goal_progress.lateral_stall_enabled", rclcpp::ParameterValue(false));
  declare_parameter(
    "near_goal_progress.lateral_stall_lon_tolerance", rclcpp::ParameterValue(0.02));
  declare_parameter(
    "near_goal_progress.lateral_stall_yaw_tolerance", rclcpp::ParameterValue(0.05));
  declare_parameter(
    "near_goal_progress.lateral_stall_lat_tolerance", rclcpp::ParameterValue(0.02));
  declare_parameter(
    "near_goal_progress.lateral_stall_min_improvement", rclcpp::ParameterValue(0.005));
  declare_parameter(
    "near_goal_progress.lateral_stall_time_allowance", rclcpp::ParameterValue(3.0));
  // near-goal hold 进入/释放门控：避免与 MPPI final_yaw 动态制动抢占控制权
  declare_parameter(
    "near_goal_progress.hold_enter_vx", rclcpp::ParameterValue(0.01));
  declare_parameter(
    "near_goal_progress.hold_enter_wz", rclcpp::ParameterValue(0.03));
  declare_parameter(
    "near_goal_progress.hold_release_lon_tolerance", rclcpp::ParameterValue(0.04));
  declare_parameter(
    "near_goal_progress.hold_release_lat_tolerance", rclcpp::ParameterValue(0.04));
  declare_parameter(
    "near_goal_progress.hold_release_yaw_tolerance", rclcpp::ParameterValue(0.04));
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

  RCLCPP_INFO(get_logger(), "Configuring controller interface");

  get_parameter("progress_checker_plugin", progress_checker_id_);
  if (progress_checker_id_ == default_progress_checker_id_) {
    nav2_util::declare_parameter_if_not_declared(
      node, default_progress_checker_id_ + ".plugin",
      rclcpp::ParameterValue(default_progress_checker_type_));
  }

  RCLCPP_INFO(get_logger(), "getting goal checker plugins..");
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
  RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_);

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
  initial_motion_arbiter_.configure(speed_plan_state_config);
  get_parameter("near_goal_progress.enabled", near_goal_progress_enabled_);
  get_parameter(
    "near_goal_progress.hold_position_when_goal_pose_reached",
    near_goal_hold_position_when_pose_reached_);
  get_parameter("near_goal_progress.distance", near_goal_progress_distance_);
  get_parameter(
    "near_goal_progress.required_movement_radius",
    near_goal_progress_required_movement_radius_);
  get_parameter(
    "near_goal_progress.required_movement_angle",
    near_goal_progress_required_movement_angle_);
  double near_goal_progress_movement_time_allowance;
  get_parameter(
    "near_goal_progress.movement_time_allowance",
    near_goal_progress_movement_time_allowance);
  near_goal_progress_time_allowance_ =
    rclcpp::Duration::from_seconds(near_goal_progress_movement_time_allowance);
  get_parameter(
    "near_goal_progress.lateral_stall_enabled", near_goal_lateral_stall_enabled_);
  get_parameter(
    "near_goal_progress.lateral_stall_lon_tolerance",
    near_goal_lateral_stall_lon_tolerance_);
  get_parameter(
    "near_goal_progress.lateral_stall_yaw_tolerance",
    near_goal_lateral_stall_yaw_tolerance_);
  get_parameter(
    "near_goal_progress.lateral_stall_lat_tolerance",
    near_goal_lateral_stall_lat_tolerance_);
  get_parameter(
    "near_goal_progress.lateral_stall_min_improvement",
    near_goal_lateral_stall_min_improvement_);
  double near_goal_lateral_stall_time_allowance;
  get_parameter(
    "near_goal_progress.lateral_stall_time_allowance",
    near_goal_lateral_stall_time_allowance);
  near_goal_lateral_stall_time_allowance_ =
    rclcpp::Duration::from_seconds(near_goal_lateral_stall_time_allowance);
  get_parameter("near_goal_progress.hold_enter_vx", near_goal_hold_enter_vx_);
  get_parameter("near_goal_progress.hold_enter_wz", near_goal_hold_enter_wz_);
  get_parameter(
    "near_goal_progress.hold_release_lon_tolerance",
    near_goal_hold_release_lon_tolerance_);
  get_parameter(
    "near_goal_progress.hold_release_lat_tolerance",
    near_goal_hold_release_lat_tolerance_);
  get_parameter(
    "near_goal_progress.hold_release_yaw_tolerance",
    near_goal_hold_release_yaw_tolerance_);
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
  RCLCPP_INFO(
    get_logger(),
    "Near-goal progress: enabled=%s hold_pose=%s distance=%.3f m movement=%.3f m "
    "angle=%.3f rad allowance=%.3f s",
    near_goal_progress_enabled_ ? "true" : "false",
    near_goal_hold_position_when_pose_reached_ ? "true" : "false",
    near_goal_progress_distance_,
    near_goal_progress_required_movement_radius_,
    near_goal_progress_required_movement_angle_,
    near_goal_progress_time_allowance_.seconds());
  RCLCPP_INFO(
    get_logger(),
    "Near-goal lateral stall: enabled=%s lon=%.3f m yaw=%.3f rad lat=%.3f m "
    "min_improvement=%.3f m allowance=%.3f s",
    near_goal_lateral_stall_enabled_ ? "true" : "false",
    near_goal_lateral_stall_lon_tolerance_,
    near_goal_lateral_stall_yaw_tolerance_,
    near_goal_lateral_stall_lat_tolerance_,
    near_goal_lateral_stall_min_improvement_,
    near_goal_lateral_stall_time_allowance_.seconds());

  costmap_ros_->configure();
  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

  try {
    progress_checker_type_ = nav2_util::get_plugin_type_param(node, progress_checker_id_);
    progress_checker_ = progress_checker_loader_.createUniqueInstance(progress_checker_type_);
    RCLCPP_INFO(
      get_logger(), "Created progress_checker : %s of type %s",
      progress_checker_id_.c_str(), progress_checker_type_.c_str());
    progress_checker_->initialize(node, progress_checker_id_);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      get_logger(),
      "Failed to create progress_checker. Exception: %s", ex.what());
    return nav2_util::CallbackReturn::FAILURE;
  }

  for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
    try {
      goal_checker_types_[i] = nav2_util::get_plugin_type_param(node, goal_checker_ids_[i]);
      nav2_core::GoalChecker::Ptr goal_checker =
        goal_checker_loader_.createUniqueInstance(goal_checker_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created goal checker : %s of type %s",
        goal_checker_ids_[i].c_str(), goal_checker_types_[i].c_str());
      goal_checker->initialize(node, goal_checker_ids_[i], costmap_ros_);
      goal_checkers_.insert({goal_checker_ids_[i], goal_checker});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create goal checker. Exception: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  goal_checker_ids_concat_.clear();
  for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
    goal_checker_ids_concat_ += goal_checker_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Controller Server has %s goal checkers available.", goal_checker_ids_concat_.c_str());

  get_parameter("current_goal_checker", default_goal_checker_);
  if (goal_checkers_.find(default_goal_checker_) == goal_checkers_.end()) {
    RCLCPP_FATAL(
      get_logger(),
      "Configured current_goal_checker '%s' does not exist. Available goal checkers are: %s.",
      default_goal_checker_.c_str(), goal_checker_ids_concat_.c_str());
    return nav2_util::CallbackReturn::FAILURE;
  }

  for (size_t i = 0; i != controller_ids_.size(); i++) {
    try {
      controller_types_[i] = nav2_util::get_plugin_type_param(node, controller_ids_[i]);
      nav2_core::Controller::Ptr controller =
        lp_loader_.createUniqueInstance(controller_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created controller : %s of type %s",
        controller_ids_[i].c_str(), controller_types_[i].c_str());
      controller->configure(
        node, controller_ids_[i],
        costmap_ros_->getTfBuffer(), costmap_ros_);
      controllers_.insert({controller_ids_[i], controller});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create controller. Exception: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  controller_ids_concat_.clear();
  for (size_t i = 0; i != controller_ids_.size(); i++) {
    controller_ids_concat_ += controller_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Controller Server has %s controllers available.", controller_ids_concat_.c_str());

  get_parameter("current_controller", default_controller_);
  if (controllers_.find(default_controller_) == controllers_.end()) {
    RCLCPP_FATAL(
      get_logger(),
      "Configured current_controller '%s' does not exist. Available controllers are: %s.",
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
        get_logger(), "Created local planner: %s of type %s",
        local_planner_ids_[i].c_str(), local_planner_types_[i].c_str());
      local_planner->configure(
        node, local_planner_ids_[i], costmap_ros_->getTfBuffer(), costmap_ros_);
      local_planners_.insert({local_planner_ids_[i], local_planner});
    } catch (const std::exception & ex) {
      RCLCPP_FATAL(get_logger(), "Failed to create local planner. Exception: %s", ex.what());
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
      "Local planning disabled (current_local_planner is empty); global path passthrough enabled.");
  } else if (local_planners_.find(current_local_planner_) == local_planners_.end()) {
    RCLCPP_FATAL(
      get_logger(),
      "Configured current_local_planner '%s' does not exist. Available local planners are: %s.",
      current_local_planner_.c_str(), local_planner_ids_concat_.c_str());
    return nav2_util::CallbackReturn::FAILURE;
  } else {
    RCLCPP_INFO(
      get_logger(), "Controller Server has %s local planners available, current: %s.",
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
        get_logger(), "Created speed planner : %s of type %s",
        speed_planner_ids_[i].c_str(), speed_planner_types_[i].c_str());
      speed_planner->configure(
        node, speed_planner_ids_[i],
        costmap_ros_->getTfBuffer(), costmap_ros_);
      speed_planners_.insert({speed_planner_ids_[i], speed_planner});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create speed planner. Exception: %s", ex.what());
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
      "Speed planning disabled (current_speed_planner is empty); "
      "controllers consume geometric paths only. 速度规划未启用，控制器仅消费几何路径。");
  } else if (speed_planners_.find(current_speed_planner_) == speed_planners_.end()) {
    RCLCPP_FATAL(
      get_logger(),
      "Configured current_speed_planner '%s' does not exist. Available speed planners are: %s.",
      current_speed_planner_.c_str(), speed_planner_ids_concat_.c_str());
    return nav2_util::CallbackReturn::FAILURE;
  } else {
    RCLCPP_INFO(
      get_logger(),
      "Controller Server has %s speed planners available, current: %s.",
      speed_planner_ids_concat_.c_str(), current_speed_planner_.c_str());
  }

  if (staged_control_enabled_) {
    if (controllers_.find(track_controller_) == controllers_.end()) {
      RCLCPP_FATAL(
        get_logger(),
        "Configured staged_control.track_controller '%s' does not exist. "
        "Available controllers are: %s.",
        track_controller_.c_str(), controller_ids_concat_.c_str());
      return nav2_util::CallbackReturn::FAILURE;
    }
    if (controllers_.find(approach_controller_) == controllers_.end()) {
      RCLCPP_FATAL(
        get_logger(),
        "Configured staged_control.approach_controller '%s' does not exist. "
        "Available controllers are: %s.",
        approach_controller_.c_str(), controller_ids_concat_.c_str());
      return nav2_util::CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(
      get_logger(),
      "Staged control enabled: prepare=%s align_tolerance=%.3f rad track=%s "
      "approach=%s approach_distance=%.3f m",
      prepare_enabled_ ? "true" : "false",
      align_heading_tolerance_,
      track_controller_.c_str(),
      approach_controller_.c_str(),
      approach_distance_);
  }

  odom_sub_ = std::make_unique<nav_2d_utils::OdomSubscriber>(node);
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
        get_logger(), "No controller was specified in action call. "
        "动作请求没有指定 controller，Server 将使用 current_controller 插件 %s。"
        "This warning will appear once.",
        default_controller_.c_str());
      current_controller = default_controller_;
      return true;
    }
  }

  if (controllers_.find(c_name) == controllers_.end()) {
    if (controllers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No controller was specified in action call. "
        "动作请求没有指定 controller，Server 将使用唯一已加载插件 %s。"
        "This warning will appear once.",
        controller_ids_concat_.c_str());
      current_controller = controllers_.begin()->first;
    } else {
      RCLCPP_ERROR(
        get_logger(), "FollowPath called with controller name %s, "
        "which does not exist. 请求的 controller_id 不存在。Available controllers are: %s.",
        c_name.c_str(), controller_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected controller: %s.", c_name.c_str());
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
        get_logger(), "No goal checker was specified in parameter 'current_goal_checker'."
        " 动作请求没有指定 goal_checker，Server 将使用 current_goal_checker 插件 %s。"
        "This warning will appear once.",
        default_goal_checker_.c_str());
      current_goal_checker = default_goal_checker_;
      return true;
    }
  }

  if (goal_checkers_.find(c_name) == goal_checkers_.end()) {
    if (goal_checkers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No goal checker was specified in parameter 'current_goal_checker'."
        " 动作请求没有指定 goal_checker，Server 将使用唯一已加载插件 %s。"
        "This warning will appear once.",
        goal_checker_ids_concat_.c_str());
      current_goal_checker = goal_checkers_.begin()->first;
    } else {
      RCLCPP_ERROR(
        get_logger(), "FollowPath called with goal_checker name %s in parameter"
        " 'current_goal_checker', which does not exist. 请求的 goal_checker_id 不存在。"
        "Available goal checkers are: %s.",
        c_name.c_str(), goal_checker_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected goal checker: %s.", c_name.c_str());
    current_goal_checker = c_name;
  }

  return true;
}

void ControllerServer::computeControl()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  RCLCPP_INFO(get_logger(), "Received a goal, begin computing control effort.");
  if (odom_sub_) {
    const auto start_twist = getThresholdedTwist(odom_sub_->getTwist());
    RCLCPP_INFO(
      get_logger(),
      "[ControllerStopDebug] FollowPath started: actual odom velocity vx=%.3f m/s, "
      "vy=%.3f m/s, wz=%.3f rad/s. If these values are not near zero immediately after a "
      "previous stop request, BT recovery restarted control before the robot fully stopped.",
      start_twist.x, start_twist.y, start_twist.theta);
  }

  publishNavigationStatus(nav_interfaces::msg::NavigationStatus::NAVIGATING);

  try {
    std::string c_name = action_server_->get_current_goal()->controller_id;
    std::string current_controller;
    if (findControllerId(c_name, current_controller)) {
      current_controller_ = current_controller;
    } else {
      action_server_->terminate_current();
      return;
    }

    std::string gc_name = action_server_->get_current_goal()->goal_checker_id;
    std::string current_goal_checker;
    if (findGoalCheckerId(gc_name, current_goal_checker)) {
      current_goal_checker_ = current_goal_checker;
    } else {
      action_server_->terminate_current();
      return;
    }

    setPlannerPath(
      action_server_->get_current_goal()->path,
      PathUpdateType::NEW_GOAL);
    progress_checker_->reset();
    resetNearGoalProgress();
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
        RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
        return;
      }

      if (action_server_->is_cancel_requested()) {
        RCLCPP_INFO(get_logger(), "Goal was canceled. Stopping the robot.");
        publishHardStopVelocity();
        publishNavigationStatus(nav_interfaces::msg::NavigationStatus::IDLE);
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
            "Goal was canceled while waiting for the costmap. Stopping the robot.");
          publishHardStopVelocity();
          publishNavigationStatus(nav_interfaces::msg::NavigationStatus::IDLE);
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
          get_logger(), "Global path update cost: %.6f ms",
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
            "Control loop missed its desired rate of %.4fHz after planning failure",
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
          get_logger(), "Local path update cost: %.6f ms",
          elapsed.count());
      }

      if (isGoalReached()) {
        RCLCPP_INFO(get_logger(), "Reached the goal!");
        resetStagedControlState();
        break;
      }

      end = std::chrono::steady_clock::now();
      elapsed = end - start;

      if (!loop_rate.sleep()) {
        RCLCPP_WARN(
          get_logger(), "Control loop missed its desired rate of %.4fHz, the cost time is %.6f ms",
          controller_frequency_, elapsed.count());
      }
    }
  } catch (nav2_core::CollisionException & e) {
    RCLCPP_ERROR(this->get_logger(), "Collision detected! %s", e.what());
    publishHardStopVelocity();
    publishNavigationStatus(nav_interfaces::msg::NavigationStatus::FAILED, e.what());
    action_server_->terminate_current();
    publishNavigationStatus(nav_interfaces::msg::NavigationStatus::IDLE);
    return;
  } catch (nav2_core::PlannerException & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishHardStopVelocity();
    publishNavigationStatus(nav_interfaces::msg::NavigationStatus::FAILED, e.what());
    action_server_->terminate_current();
    publishNavigationStatus(nav_interfaces::msg::NavigationStatus::IDLE);
    return;
  }catch (std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishHardStopVelocity();
    publishNavigationStatus(nav_interfaces::msg::NavigationStatus::FAILED, e.what());
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    action_server_->terminate_current(result);
    publishNavigationStatus(nav_interfaces::msg::NavigationStatus::IDLE);
    return;
  }
  RCLCPP_DEBUG(get_logger(), "Controller succeeded, setting result");

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
    "Providing path to the controller %s", current_controller_.c_str());
  if (path.poses.empty()) {
    throw nav2_core::PlannerException(
            "Invalid path, Path is empty. 路径无效：path.poses 为空，通常表示 FollowPath 收到了空路径，无法进行局部控制。");
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
  end_pose_ = path.poses.back();
  end_pose_.header.frame_id = path.header.frame_id;
  if (update_type == PathUpdateType::NEW_GOAL) {
    goal_checkers_[current_goal_checker_]->reset();
    near_goal_progress_acceptance_reached_ = false;
    near_goal_pose_hold_active_ = false;
    near_goal_hold_outside_accept_count_ = 0;
    controller_phase_ = ControllerPhase::TRACK;
    terminal_brake_latched_ = false;
  } else {
    RCLCPP_INFO(
      get_logger(),
      "同目标 path 仅替换几何：保留 controller phase=%s、goal/near-goal 状态和 "
      "InitialMotionArbiter 已消费 (v,a) 连续性、terminal_brake_latched=%s。",
      controllerPhaseName(controller_phase_), terminal_brake_latched_ ? "true" : "false");
  }

  RCLCPP_DEBUG(
    get_logger(), "Path end point is (%.2f, %.2f)",
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
      "Configured local planner '%s' is unavailable; halting this control cycle.",
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
      "Local planner '%s' failed or returned an empty path; halting this control cycle.",
      current_local_planner_.c_str());
    local_path = nav_msgs::msg::Path();
    return false;
  }
  return true;
}

nav2_core::InitialMotionState ControllerServer::buildInitialMotionState(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Twist & measured_velocity)
{
  auto decision = initial_motion_arbiter_.resolve(path, measured_velocity, now());
  decision.state.terminal_brake_latched = terminal_brake_latched_;
  if (speed_plan_cycle_.active) {
    speed_plan_cycle_.initial_motion = decision.state;
    speed_plan_cycle_.initial_motion_valid = true;
    speed_plan_cycle_.initial_motion_source = decision.reason.has_value() ?
      nav2_speed_planner::initialMotionSourceName(*decision.reason) : "inherited";
  }
  if (decision.reason.has_value()) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Speed planning initial_motion reseeded: reason=%s",
      nav2_speed_planner::initialMotionSourceName(*decision.reason));
  }
  return decision.state;
}

void ControllerServer::updateTerminalBrakeLatch(
  const nav2_core::SpeedPlanningDiagnostics & diagnostics,
  bool longitudinal_continuity_valid)
{
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
      "Terminal brake condition deferred: no longitudinal continuity and "
      "braking distance is zero (available=%.3f m, infeasible=%s).",
      diagnostics.terminal_distance_available,
      diagnostics.terminal_infeasible ? "true" : "false");
    return;
  }

  terminal_brake_latched_ = true;
  RCLCPP_WARN(
    get_logger(),
    "Terminal brake latched: available=%.3f m braking=%.3f m reserve=%.3f m "
    "infeasible=%s. Same-goal path updates may not re-arm cruise acceleration.",
    diagnostics.terminal_distance_available,
    diagnostics.terminal_braking_distance,
    diagnostics.terminal_execution_reserve,
    diagnostics.terminal_infeasible ? "true" : "false");
}

void ControllerServer::updateSpeedPlanningContinuityAfterControl(
  bool control_succeeded, bool trajectory_target_consumed,
  double consumed_target_velocity, double consumed_target_acceleration)
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
    current_trajectory_, consumed_target_velocity, consumed_target_acceleration, now());
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
  double dispatch_ms = 0.0;
  double current_dispatch_ms = 0.0;
  double track_dispatch_ms = 0.0;
  double approach_dispatch_ms = 0.0;
  double visualization_ms = 0.0;
  nav_msgs::msg::Path planning_path;

  const auto elapsed_ms = [](const auto & begin, const auto & end) {
      return std::chrono::duration<double, std::milli>(end - begin).count();
    };
  const auto log_pipeline_timing = [&](const char * result) {
      const double total_ms = elapsed_ms(pipeline_start, PipelineClock::now());
      const double measured_ms =
        odom_ms + pose_ms + local_planner_ms + speed_set_plan_ms + speed_compute_ms +
        dispatch_ms + visualization_ms;
      const double unaccounted_ms = std::max(0.0, total_ms - measured_ms);
      const size_t planning_points = planning_path.poses.size();
      const std::string & planning_frame = planning_path.header.frame_id;
      if (total_ms >= kSlowPlanningPipelineWarnMs) {
        RCLCPP_WARN(
          get_logger(),
          "Planning pipeline timing (ms): result=%s total=%.3f odom=%.3f pose=%.3f "
          "local=%.3f speed_set_plan=%.3f speed_compute=%.3f dispatch=%.3f "
          "(current=%.3f track=%.3f approach=%.3f) viz=%.3f "
          "unaccounted=%.3f points=%zu->%zu->%zu frames=%s->%s",
          result, total_ms, odom_ms, pose_ms, local_planner_ms, speed_set_plan_ms,
          speed_compute_ms, dispatch_ms, current_dispatch_ms, track_dispatch_ms,
          approach_dispatch_ms, visualization_ms, unaccounted_ms,
          path.poses.size(), planning_points, current_trajectory_.points.size(),
          path.header.frame_id.c_str(), planning_frame.c_str());
      } else {
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Planning pipeline timing (ms): result=%s total=%.3f odom=%.3f pose=%.3f "
          "local=%.3f speed_set_plan=%.3f speed_compute=%.3f dispatch=%.3f "
          "(current=%.3f track=%.3f approach=%.3f) viz=%.3f "
          "unaccounted=%.3f points=%zu->%zu->%zu frames=%s->%s",
          result, total_ms, odom_ms, pose_ms, local_planner_ms, speed_set_plan_ms,
          speed_compute_ms, dispatch_ms, current_dispatch_ms, track_dispatch_ms,
          approach_dispatch_ms, visualization_ms, unaccounted_ms,
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
      "Configured speed planner '%s' is unavailable; halting this control cycle.",
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

  if (!current_local_planner_.empty()) {
    geometry_msgs::msg::PoseStamped pose;
    const auto pose_start = PipelineClock::now();
    const std::string planning_frame = path.header.frame_id.empty() ?
      path.poses.front().header.frame_id : path.header.frame_id;
    const bool pose_available = !planning_frame.empty() && nav2_util::getCurrentPose(
      pose, *costmap_ros_->getTfBuffer(), planning_frame,
      costmap_ros_->getBaseFrameID(), costmap_ros_->getTransformTolerance());
    if (!pose_available) {
      pose_ms = elapsed_ms(pose_start, PipelineClock::now());
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Cannot get robot pose in path frame '%s' for local planning; "
        "halting this control cycle.", planning_frame.c_str());
      const auto visualization_start = PipelineClock::now();
      publishSpeedPlan(current_trajectory_);
      visualization_ms = elapsed_ms(visualization_start, PipelineClock::now());
      speed_plan_cycle_.planning_result = "pose_lookup_failed";
      publishSpeedPlanDiagnostics(
        false, false, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(), "not_run");
      log_pipeline_timing("pose_lookup_failed");
      return false;
    }
    pose_ms = elapsed_ms(pose_start, PipelineClock::now());
    const auto local_planner_start = PipelineClock::now();
    if (!computeLocalPath(path, pose, current_velocity, planning_path)) {
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

  const auto speed_set_plan_start = PipelineClock::now();
  sp->second->setPlan(planning_path);
  speed_set_plan_ms = elapsed_ms(speed_set_plan_start, PipelineClock::now());
  const auto speed_compute_start = PipelineClock::now();
  const auto initial_motion = buildInitialMotionState(planning_path, current_velocity);
  current_trajectory_ = sp->second->computeSpeedPlan(planning_path, initial_motion);
  const auto planning_diagnostics = sp->second->getLastPlanningDiagnostics();
  speed_plan_cycle_.terminal = planning_diagnostics;
  speed_plan_cycle_.terminal_valid = true;
  updateTerminalBrakeLatch(
    planning_diagnostics, initial_motion.acceleration_valid);
  if (planning_diagnostics.terminal_infeasible) {
    // 兜底剖面的目标速度就是 end_speed，因此一旦持续进入兜底，整条轨迹恒为
    // end_speed（实机曾表现为全程 0.05 蠕行 53.6s 走 2.38m）。且机器人降速后
    // v0 变低会让下一周期继续满足兜底条件——**自维持，出不来**。
    // 原日志只说"进了兜底"，不说为什么进；下面这些量才是定位所需的最小集合。
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
      "Speed plan diagnostic: planner=%s terminal_infeasible=true kind=%s "
      "entry=%s final=%s provenance=%s | anchor_s=%.3f available=%.3f braking=%.3f "
      "reserve=%.3f latched_reserve=%.3f | v0=%.3f a0=%.3f latched=%d path_len=%.3f "
      "points=%zu | branch=%s first_obs=%d@%.3f | solver: code=%d v0=%.4f a0=%.4f "
      "vmax=%.4f target_v=%.4f dist=%.4f; "
      "using a continuous reachable braking profile.",
      current_speed_planner_.c_str(), kKind[k], kReason[er], kReason[fr],
      d.terminal_result_provenance ? "braking_fallback" : "normal",
      d.terminal_anchor_s, d.terminal_distance_available, d.terminal_braking_distance,
      d.terminal_execution_reserve, d.terminal_latched_reserve,
      initial_motion.velocity.linear.x, initial_motion.acceleration,
      initial_motion.terminal_brake_latched ? 1 : 0,
      nav2_util::geometry_utils::calculate_path_length(planning_path),
      planning_path.poses.size(),
      d.terminal_branch == 1 ? "obstacle" : (d.terminal_branch == 2 ? "terminal" : "none"),
      d.first_obstacle_index, d.first_obstacle_arc,
      d.solver_code, d.solver_v0, d.solver_a0, d.solver_vmax,
      d.solver_target_v, d.solver_distance);
  }
  speed_compute_ms = elapsed_ms(speed_compute_start, PipelineClock::now());
  if (current_trajectory_.points.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Speed planner '%s' returned an empty trajectory; halting this control cycle.",
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

  const auto dispatch_trajectory = [&](const std::string & controller_id, double & stage_ms) {
      const auto dispatch_start = PipelineClock::now();
      controllers_[controller_id]->setTrajectory(current_trajectory_);
      stage_ms = elapsed_ms(dispatch_start, PipelineClock::now());
      dispatch_ms += stage_ms;
    };
  dispatch_trajectory(current_controller_, current_dispatch_ms);
  if (staged_control_enabled_) {
    if (track_controller_ != current_controller_) {
      dispatch_trajectory(track_controller_, track_dispatch_ms);
    }
    if (approach_controller_ != current_controller_ && approach_controller_ != track_controller_) {
      dispatch_trajectory(approach_controller_, approach_dispatch_ms);
    }
  }

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
    "Speed plan dispatched: planner=%s points=%zu v_start=%.3f a_start=%.3f inherited=%s "
    "source_age=%.3f",
    current_speed_planner_.c_str(), current_trajectory_.points.size(),
    current_velocity.linear.x, initial_motion.acceleration,
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
    long first_obs = -1, last_obs = -1, first_buf = -1;
    long first_curve = -1, last_curve = -1, first_creep = -1;
    for (size_t i = 0; i < trajectory.points.size(); ++i) {
      const auto & rt = trajectory.points[i].region_type;
      if (rt == "obstacle") {
        if (first_obs < 0) {first_obs = static_cast<long>(i);}
        last_obs = static_cast<long>(i);
      } else if (rt == "obstacle_buffer") {
        if (first_buf < 0) {first_buf = static_cast<long>(i);}
      } else if (rt == "curve") {
        if (first_curve < 0) {first_curve = static_cast<long>(i);}
        last_curve = static_cast<long>(i);
      } else if (rt == "stop_zone") {
        if (first_creep < 0) {first_creep = static_cast<long>(i);}
      }
    }
    if (first_buf >= 0) {
      anchors.push_back({static_cast<size_t>(first_buf), "obs_stop", 1.0f, 0.5f, 0.0f});
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
  if (!cycle.planning_succeeded || !control_succeeded) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  } else if (cycle.terminal_valid && cycle.terminal.terminal_infeasible) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  }
  status.name = "navigation_speed_plan";
  if (!cycle.planning_succeeded) {
    status.message = cycle.planning_result;
  } else if (!control_succeeded) {
    status.message = control_result;
  } else if (cycle.terminal_valid && cycle.terminal.terminal_infeasible) {
    status.message = "terminal_infeasible";
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
  add_value("boundary_computed", cycle.initial_motion_valid ? "true" : "false");
  add_value("boundary_reason", cycle.initial_motion_source);
  if (cycle.initial_motion_valid) {
    add_value("boundary_velocity", std::to_string(cycle.initial_motion.velocity.linear.x));
    add_value("boundary_acceleration", std::to_string(cycle.initial_motion.acceleration));
    add_value(
      "boundary_acceleration_valid",
      cycle.initial_motion.acceleration_valid ? "true" : "false");
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
  if (cycle.terminal_valid) {
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

void ControllerServer::resetNearGoalProgress()
{
  near_goal_baseline_pose_set_ = false;
  in_near_goal_progress_mode_ = false;
  resetNearGoalLateralStall();
}

void ControllerServer::resetNearGoalLateralStall()
{
  near_goal_lateral_stall_entry_count_ = 0;
  near_goal_lateral_stall_monitoring_ = false;
  near_goal_lateral_stall_baseline_error_ = 0.0;
  near_goal_lateral_stall_best_error_ = 0.0;
}

bool ControllerServer::isNearGoal(const geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::PoseStamped transformed_end_pose;
  try {
    rclcpp::Duration tolerance(
      rclcpp::Duration::from_seconds(costmap_ros_->getTransformTolerance()));
    if (!nav_2d_utils::transformPose(
      costmap_ros_->getTfBuffer(), pose.header.frame_id,
      end_pose_, transformed_end_pose, tolerance))
    {
      return false;
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Unable to transform end pose for near-goal progress check: %s. "
      "近目标进度检查无法转换终点位姿，请检查 TF。",
      ex.what());
    return false;
  }

  const double dx = pose.pose.position.x - transformed_end_pose.pose.position.x;
  const double dy = pose.pose.position.y - transformed_end_pose.pose.position.y;
  return std::hypot(dx, dy) <= near_goal_progress_distance_;
}

bool ControllerServer::checkNearGoalProgress(const geometry_msgs::msg::PoseStamped & pose)
{
  constexpr int kLateralStallEntryCycles = 5;

  if (!in_near_goal_progress_mode_) {
    RCLCPP_INFO(
      get_logger(),
      "Entering near-goal progress mode: distance=%.3f m, required movement=%.3f m, "
      "required angle=%.3f rad, allowance=%.3f s",
      near_goal_progress_distance_,
      near_goal_progress_required_movement_radius_,
      near_goal_progress_required_movement_angle_,
      near_goal_progress_time_allowance_.seconds());
    in_near_goal_progress_mode_ = true;
  }

  if (near_goal_lateral_stall_enabled_) {
    geometry_msgs::msg::PoseStamped transformed_end_pose;
    try {
      rclcpp::Duration tolerance(
        rclcpp::Duration::from_seconds(costmap_ros_->getTransformTolerance()));
      if (!nav_2d_utils::transformPose(
        costmap_ros_->getTfBuffer(), pose.header.frame_id,
        end_pose_, transformed_end_pose, tolerance))
      {
        resetNearGoalLateralStall();
      } else {
        const double goal_yaw = tf2::getYaw(transformed_end_pose.pose.orientation);
        const double current_yaw = tf2::getYaw(pose.pose.orientation);
        const double dx = pose.pose.position.x - transformed_end_pose.pose.position.x;
        const double dy = pose.pose.position.y - transformed_end_pose.pose.position.y;
        const double abs_lon_err = std::abs(std::cos(goal_yaw) * dx + std::sin(goal_yaw) * dy);
        const double abs_lat_err = std::abs(-std::sin(goal_yaw) * dx + std::cos(goal_yaw) * dy);
        const double abs_yaw_err = std::abs(
          angles::shortest_angular_distance(goal_yaw, current_yaw));

        const bool lat_outside_tolerance =
          abs_lat_err > near_goal_lateral_stall_lat_tolerance_;
        const bool entry_condition =
          abs_lon_err <= near_goal_lateral_stall_lon_tolerance_ &&
          abs_yaw_err <= near_goal_lateral_stall_yaw_tolerance_ &&
          lat_outside_tolerance;
        const bool left_monitoring_region =
          abs_lon_err > 2.0 * near_goal_lateral_stall_lon_tolerance_ ||
          abs_yaw_err > 2.0 * near_goal_lateral_stall_yaw_tolerance_;

        if (near_goal_lateral_stall_monitoring_) {
          if (!lat_outside_tolerance || left_monitoring_region) {
            RCLCPP_INFO(
              get_logger(),
              "Leaving near-goal lateral-stall monitoring: lon=%.4f m lat=%.4f m "
              "yaw=%.4f rad.",
              abs_lon_err, abs_lat_err, abs_yaw_err);
            resetNearGoalLateralStall();
          } else {
            near_goal_lateral_stall_best_error_ =
              std::min(near_goal_lateral_stall_best_error_, abs_lat_err);
            const double improvement =
              near_goal_lateral_stall_baseline_error_ -
              near_goal_lateral_stall_best_error_;

            if (improvement >= near_goal_lateral_stall_min_improvement_) {
              RCLCPP_INFO(
                get_logger(),
                "Near-goal lateral error improved: baseline=%.4f m best=%.4f m "
                "improvement=%.4f/%.4f m, restarting stagnation timer.",
                near_goal_lateral_stall_baseline_error_,
                near_goal_lateral_stall_best_error_, improvement,
                near_goal_lateral_stall_min_improvement_);
              near_goal_lateral_stall_baseline_error_ =
                near_goal_lateral_stall_best_error_;
              near_goal_lateral_stall_start_time_ = now();
            } else {
              const auto elapsed = now() - near_goal_lateral_stall_start_time_;
              if (elapsed > near_goal_lateral_stall_time_allowance_) {
                RCLCPP_WARN(
                  get_logger(),
                  "Near-goal lateral progress stalled: elapsed=%.3f s, lon=%.4f m, "
                  "lat=%.4f m, yaw=%.4f rad, baseline_lat=%.4f m, best_lat=%.4f m, "
                  "improvement=%.4f/%.4f m. FollowPath will fail so the behavior "
                  "tree can run local recovery.",
                  elapsed.seconds(), abs_lon_err, abs_lat_err, abs_yaw_err,
                  near_goal_lateral_stall_baseline_error_,
                  near_goal_lateral_stall_best_error_, improvement,
                  near_goal_lateral_stall_min_improvement_);
                return false;
              }
            }
          }
        } else if (entry_condition) {
          ++near_goal_lateral_stall_entry_count_;
          if (near_goal_lateral_stall_entry_count_ >= kLateralStallEntryCycles) {
            near_goal_lateral_stall_monitoring_ = true;
            near_goal_lateral_stall_baseline_error_ = abs_lat_err;
            near_goal_lateral_stall_best_error_ = abs_lat_err;
            near_goal_lateral_stall_start_time_ = now();
            RCLCPP_INFO(
              get_logger(),
              "Entering near-goal lateral-stall monitoring after %d cycles: "
              "lon=%.4f m lat=%.4f m yaw=%.4f rad, required improvement=%.4f m "
              "within %.3f s.",
              kLateralStallEntryCycles, abs_lon_err, abs_lat_err, abs_yaw_err,
              near_goal_lateral_stall_min_improvement_,
              near_goal_lateral_stall_time_allowance_.seconds());
          }
        } else {
          near_goal_lateral_stall_entry_count_ = 0;
        }
      }
    } catch (const tf2::TransformException & ex) {
      resetNearGoalLateralStall();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Unable to transform end pose for lateral-stall check: %s. "
        "横向误差停滞检查无法转换终点位姿，请检查 TF。",
        ex.what());
    }
  }

  if (!near_goal_baseline_pose_set_) {
    near_goal_baseline_pose_ = pose;
    near_goal_baseline_time_ = now();
    near_goal_baseline_pose_set_ = true;
    return true;
  }

  const double dx = pose.pose.position.x - near_goal_baseline_pose_.pose.position.x;
  const double dy = pose.pose.position.y - near_goal_baseline_pose_.pose.position.y;
  const double moved_distance = std::hypot(dx, dy);
  const double moved_angle = std::abs(
    angles::shortest_angular_distance(
      tf2::getYaw(near_goal_baseline_pose_.pose.orientation),
      tf2::getYaw(pose.pose.orientation)));

  if (moved_distance > near_goal_progress_required_movement_radius_ ||
    moved_angle > near_goal_progress_required_movement_angle_)
  {
    near_goal_baseline_pose_ = pose;
    near_goal_baseline_time_ = now();
    return true;
  }

  const auto elapsed = now() - near_goal_baseline_time_;
  if (elapsed <= near_goal_progress_time_allowance_) {
    return true;
  }

  RCLCPP_WARN(
    get_logger(),
    "Near-goal progress timeout: elapsed=%.3f s, moved=%.4f m, rotated=%.4f rad, "
    "required movement=%.4f m or angle=%.4f rad. "
    "近目标区域进度超时：机器人接近目标后没有满足最小位移或转角要求。",
    elapsed.seconds(), moved_distance, moved_angle,
    near_goal_progress_required_movement_radius_,
    near_goal_progress_required_movement_angle_);
  return false;
}

bool ControllerServer::isWithinGoalCheckerTolerance(
  const geometry_msgs::msg::PoseStamped & pose,
  bool mark_goal_reached_on_success,
  bool check_yaw)
{
  geometry_msgs::msg::Pose goal_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checkers_[current_goal_checker_]->getTolerances(goal_tolerance, vel_tolerance)) {
    return false;
  }

  geometry_msgs::msg::PoseStamped transformed_end_pose;
  try {
    rclcpp::Duration tolerance(
      rclcpp::Duration::from_seconds(costmap_ros_->getTransformTolerance()));
    if (!nav_2d_utils::transformPose(
      costmap_ros_->getTfBuffer(), pose.header.frame_id,
      end_pose_, transformed_end_pose, tolerance))
    {
      return false;
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Unable to transform end pose for near-goal tolerance check: %s. "
      "近目标兜底误差检查无法转换终点位姿，请检查 TF。",
      ex.what());
    return false;
  }

  const double goal_yaw = tf2::getYaw(transformed_end_pose.pose.orientation);
  const double dx = pose.pose.position.x - transformed_end_pose.pose.position.x;
  const double dy = pose.pose.position.y - transformed_end_pose.pose.position.y;
  // 使用目标点坐标系投影，和纵向/横向 goal_checker 的误差定义保持一致。
  const double lon_err = std::cos(goal_yaw) * dx + std::sin(goal_yaw) * dy;
  const double lat_err = -std::sin(goal_yaw) * dx + std::cos(goal_yaw) * dy;
  const double yaw_err = angles::shortest_angular_distance(
    tf2::getYaw(pose.pose.orientation), goal_yaw);

  const double invalid_field = std::numeric_limits<double>::lowest();
  auto is_valid_tolerance = [invalid_field](double tolerance) {
      return tolerance > invalid_field / 2.0;
    };

  const bool lon_ok =
    !is_valid_tolerance(goal_tolerance.position.x) ||
    std::fabs(lon_err) <= std::fabs(goal_tolerance.position.x);
  const bool lat_ok =
    !is_valid_tolerance(goal_tolerance.position.y) ||
    std::fabs(lat_err) <= std::fabs(goal_tolerance.position.y);
  const double yaw_tolerance = tf2::getYaw(goal_tolerance.orientation);
  const bool yaw_ok =
    !is_valid_tolerance(yaw_tolerance) ||
    std::fabs(yaw_err) <= std::fabs(yaw_tolerance);

  nav_2d_msgs::msg::Twist2D twist = getThresholdedTwist(odom_sub_->getTwist());
  geometry_msgs::msg::Twist velocity = nav_2d_utils::twist2Dto3D(twist);
  const bool trans_stopped_ok =
    !is_valid_tolerance(vel_tolerance.linear.x) ||
    std::hypot(velocity.linear.x, velocity.linear.y) <= std::fabs(vel_tolerance.linear.x);
  const bool rot_stopped_ok =
    !is_valid_tolerance(vel_tolerance.angular.z) ||
    std::fabs(velocity.angular.z) <= std::fabs(vel_tolerance.angular.z);

  const bool yaw_accept = !check_yaw || yaw_ok;
  const bool stopped_accept = !mark_goal_reached_on_success ||
    (trans_stopped_ok && rot_stopped_ok);
  const bool tolerance_accept = lon_ok && lat_ok && yaw_accept && stopped_accept;

  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 500,
    "GoalChecker tolerance: id=%s type=%s check_yaw=%s mark_reached=%s result=%s "
    "goal=(frame=%s x=%.3f y=%.3f yaw=%.3f) "
    "robot=(frame=%s x=%.3f y=%.3f yaw=%.3f) "
    "errors=(lon=%.4f lat=%.4f yaw=%.4f dist=%.4f) "
    "tolerances=(lon=%.4f lat=%.4f yaw=%.4f trans_vel=%.4f rot_vel=%.4f) "
    "velocity=(linear=%.4f angular=%.4f) "
    "checks=(lon=%s lat=%s yaw=%s trans_stopped=%s rot_stopped=%s)",
    current_goal_checker_.c_str(), getGoalCheckerType(current_goal_checker_).c_str(),
    check_yaw ? "true" : "false",
    mark_goal_reached_on_success ? "true" : "false",
    tolerance_accept ? "true" : "false",
    transformed_end_pose.header.frame_id.c_str(),
    transformed_end_pose.pose.position.x,
    transformed_end_pose.pose.position.y,
    goal_yaw,
    pose.header.frame_id.c_str(),
    pose.pose.position.x,
    pose.pose.position.y,
    tf2::getYaw(pose.pose.orientation),
    lon_err, lat_err, yaw_err, std::hypot(lon_err, lat_err),
    std::fabs(goal_tolerance.position.x),
    std::fabs(goal_tolerance.position.y),
    std::fabs(yaw_tolerance),
    std::fabs(vel_tolerance.linear.x),
    std::fabs(vel_tolerance.angular.z),
    std::hypot(velocity.linear.x, velocity.linear.y),
    std::fabs(velocity.angular.z),
    lon_ok ? "true" : "false",
    lat_ok ? "true" : "false",
    yaw_accept ? "true" : "false",
    trans_stopped_ok ? "true" : "false",
    rot_stopped_ok ? "true" : "false");

  if (tolerance_accept) {
    if (mark_goal_reached_on_success) {
      RCLCPP_WARN(
        get_logger(),
        "近目标进度检查超时，但当前误差已满足 goal_checker 接受容差，按到位成功处理："
        "lon_err=%.4f/%.4f, lat_err=%.4f/%.4f, yaw_err=%.4f/%.4f, "
        "linear_vel=%.4f, angular_vel=%.4f",
        lon_err, std::fabs(goal_tolerance.position.x),
        lat_err, std::fabs(goal_tolerance.position.y),
        yaw_err, std::fabs(yaw_tolerance),
        std::hypot(velocity.linear.x, velocity.linear.y),
        std::fabs(velocity.angular.z));
      if (goal_error_publisher_ && goal_error_publisher_->is_activated()) {
        diagnostic_msgs::msg::DiagnosticArray msg;
        msg.header.stamp = now();
        msg.header.frame_id = pose.header.frame_id;
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.name = "navigation_goal_error";
        status.message = "near_goal_progress_acceptance";
        status.hardware_id = "nav2_controller";

        auto add_value = [&status](const std::string & key, const std::string & value) {
            diagnostic_msgs::msg::KeyValue item;
            item.key = key;
            item.value = value;
            status.values.push_back(item);
          };
        auto add_double = [&add_value](const std::string & key, double value) {
            add_value(key, std::to_string(value));
          };
        auto add_bool = [&add_value](const std::string & key, bool value) {
            add_value(key, value ? "true" : "false");
          };

        add_value("result_type", "near_goal_progress_acceptance");
        add_double("goal_x", transformed_end_pose.pose.position.x);
        add_double("goal_y", transformed_end_pose.pose.position.y);
        add_double("goal_yaw", tf2::getYaw(transformed_end_pose.pose.orientation));
        add_double("current_x", pose.pose.position.x);
        add_double("current_y", pose.pose.position.y);
        add_double("current_yaw", tf2::getYaw(pose.pose.orientation));
        add_double("longitudinal_error", lon_err);
        add_double("lateral_error", lat_err);
        add_double("distance_error", std::hypot(lon_err, lat_err));
        add_double("yaw_error", yaw_err);
        add_double("desired_longitudinal_tolerance", std::fabs(goal_tolerance.position.x));
        add_double("desired_lateral_tolerance", std::fabs(goal_tolerance.position.y));
        add_double("accept_longitudinal_tolerance", std::fabs(goal_tolerance.position.x));
        add_double("accept_lateral_tolerance", std::fabs(goal_tolerance.position.y));
        add_double("yaw_tolerance", std::fabs(yaw_tolerance));
        add_bool("within_desired_tolerance", false);
        add_bool("within_accept_tolerance", true);
        add_bool("desired_window_stable", false);

        msg.status.push_back(status);
        goal_error_publisher_->publish(msg);
      }
      near_goal_progress_acceptance_reached_ = true;
    }
    return true;
  }

  if (!mark_goal_reached_on_success) {
    return false;
  }

  RCLCPP_WARN(
    get_logger(),
    "近目标进度检查超时，且当前误差仍未满足 goal_checker 接受容差："
    "lon_err=%.4f/%.4f, lat_err=%.4f/%.4f, yaw_err=%.4f/%.4f",
    lon_err, std::fabs(goal_tolerance.position.x),
    lat_err, std::fabs(goal_tolerance.position.y),
    yaw_err, std::fabs(yaw_tolerance));
  return false;
}

bool ControllerServer::isRobotStoppedForGoalChecker()
{
  geometry_msgs::msg::Pose goal_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checkers_[current_goal_checker_]->getTolerances(goal_tolerance, vel_tolerance)) {
    return false;
  }

  const double invalid_field = std::numeric_limits<double>::lowest();
  const auto is_valid_tolerance = [invalid_field](double tolerance) {
      return tolerance > invalid_field / 2.0;
    };
  const auto velocity = getThresholdedTwist(odom_sub_->getTwist());
  const bool trans_stopped =
    !is_valid_tolerance(vel_tolerance.linear.x) ||
    std::hypot(velocity.x, velocity.y) <= std::fabs(vel_tolerance.linear.x);
  const bool rot_stopped =
    !is_valid_tolerance(vel_tolerance.angular.z) ||
    std::fabs(velocity.theta) <= std::fabs(vel_tolerance.angular.z);
  return trans_stopped && rot_stopped;
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
      "Unable to transform pending path end pose for same-goal check: %s. "
      "无法转换待更新 path 的终点，按新路径处理。",
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
      "近目标 path 更新未被视为同一目标：goal_delta=%.4f/%.4f m, "
      "yaw_delta=%.4f/%.4f rad，将正常更新 path。",
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
    "Controller phase transition: %s -> %s (%s)",
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
      "Unable to transform end pose for staged control: %s. "
      "四阶段控制无法转换目标位姿，请检查 TF。",
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
      "Unable to transform robot pose for path heading alignment: %s. "
      "四阶段 align 无法转换机器人位姿，请检查 TF。",
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
        get_logger(), "Prepare backup aborted: obstacle detected behind the robot.");
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
      get_logger(), "Prepare phase checked once at navigation start: no obstacle near footprint.");
    return false;
  }

  // 启动前也查一次后方：前方受阻但后方同样无路可退时，不盲目后退，交给
  // 后续 align/track 与全局避障处理（不是 prepare 的职责）。
  if (isRearObstacleDetected()) {
    RCLCPP_WARN(
      get_logger(),
      "Prepare: front obstacle detected but rear is also blocked; skipping backup.");
    return false;
  }

  prepare_backup_active_ = true;
  prepare_backup_start_time_ = now();
  getRobotPose(prepare_backup_start_pose_);
  RCLCPP_WARN(
    get_logger(),
    "Prepare phase detected obstacle near footprint (front/side), backing up: speed=%.3f m/s %s",
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
  invalidateSpeedPlanningContinuity();
}

geometry_msgs::msg::TwistStamped ControllerServer::computeControllerVelocity(
  const std::string & controller_id,
  const geometry_msgs::msg::PoseStamped & pose,
  const nav_2d_msgs::msg::Twist2D & twist)
{
  active_controller_for_log_ = controller_id;
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
    setControllerPhase(ControllerPhase::TRACK, "staged_control disabled");
    active_controller_for_log_ = current_controller_;
    return computeControllerVelocity(current_controller_, pose, twist);
  }

  if (shouldRunPrepareBackup()) {
    setControllerPhase(ControllerPhase::PREPARE, "front obstacle backup");
    active_controller_for_log_ = "direct_prepare_backup";
    cmd_vel.twist.linear.x = prepare_backup_speed_;
    return cmd_vel;
  }

  const double distance_to_goal = getDistanceToGoal(pose);
  const bool in_approach = distance_to_goal <= approach_distance_;
  if (in_approach) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 500,
      "Staged control decision: approach has priority, distance_to_goal=%.3f <= "
      "approach_distance=%.3f; using controller=%s",
      distance_to_goal, approach_distance_, approach_controller_.c_str());
    setControllerPhase(ControllerPhase::APPROACH, "within approach distance");
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
      "Staged control decision: outside approach, distance_to_goal=%.3f > "
      "approach_distance=%.3f; target_heading_available=%s heading_error=%.3f "
      "align_tolerance=%.3f align_target_distance=%.3f",
      distance_to_goal,
      approach_distance_,
      has_target_heading ? "true" : "false",
      has_target_heading ? heading_error : 0.0,
      align_heading_tolerance_,
      align_target_distance_);
    if (has_target_heading && std::fabs(heading_error) > align_heading_tolerance_) {
      setControllerPhase(ControllerPhase::ALIGN, "target point heading error exceeds tolerance");
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

  setControllerPhase(ControllerPhase::TRACK, "align phase done, tracking");
  active_controller_for_log_ = track_controller_;
  return computeControllerVelocity(track_controller_, pose, twist);
}

bool ControllerServer::nearestPlannedSpeed(
  const geometry_msgs::msg::PoseStamped & pose, double & desired_v)
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
  return true;
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
            "Failed to obtain robot pose. 获取机器人位姿失败，通常需要检查 TF、定位状态或 costmap frame。");
  }
  auto t_pose_end = std::chrono::high_resolution_clock::now();
  auto pose_ms = std::chrono::duration<double, std::milli>(t_pose_end - t_pose_start).count();

  if (near_goal_pose_hold_active_) {
    publishZeroVelocity();
    if (isRobotStoppedForGoalChecker()) {
      // 停稳后检查误差状态：同时计算是否超出 release 容差、是否仍在 accept 容差内
      geometry_msgs::msg::PoseStamped transformed_end_pose;
      bool tf_ok = false;
      double lon_err = 0.0, lat_err = 0.0, yaw_err = 0.0;
      try {
        rclcpp::Duration tf_tol(
          rclcpp::Duration::from_seconds(costmap_ros_->getTransformTolerance()));
        if (nav_2d_utils::transformPose(
            costmap_ros_->getTfBuffer(), pose.header.frame_id,
            end_pose_, transformed_end_pose, tf_tol))
        {
          const double gy = tf2::getYaw(transformed_end_pose.pose.orientation);
          const double cy = tf2::getYaw(pose.pose.orientation);
          const double dx = pose.pose.position.x - transformed_end_pose.pose.position.x;
          const double dy = pose.pose.position.y - transformed_end_pose.pose.position.y;
          lon_err = std::cos(gy) * dx + std::sin(gy) * dy;
          lat_err = -std::sin(gy) * dx + std::cos(gy) * dy;
          yaw_err = angles::shortest_angular_distance(cy, gy);
          tf_ok = true;
        }
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Near-goal hold release check TF failed: %s. 保持 hold 状态。", ex.what());
      }

      if (tf_ok) {
        // 误差超出更宽松的 release 容差：立即释放，交回 MPPI 修正
        const bool outside_release_tolerance =
          std::fabs(lon_err) > near_goal_hold_release_lon_tolerance_ ||
          std::fabs(lat_err) > near_goal_hold_release_lat_tolerance_ ||
          std::fabs(yaw_err) > near_goal_hold_release_yaw_tolerance_;

        // 误差是否仍在 GoalChecker accept 容差内：直接复用现有函数，
        // 避免手动比较与自定义 GoalChecker（含精调/滑动窗口等）实际逻辑不一致
        const bool within_accept =
          isWithinGoalCheckerTolerance(pose, false, true);

        // 宽限计数：误差超出 accept 但未超 release，累计周期数；连续超 accept 容差则释放
        if (within_accept) {
          near_goal_hold_outside_accept_count_ = 0;
        } else {
          ++near_goal_hold_outside_accept_count_;
        }
        const bool release_after_grace =
          near_goal_hold_outside_accept_count_ >= near_goal_hold_outside_accept_cycles_;

        if (outside_release_tolerance || release_after_grace) {
          const size_t outside_accept_cycles = near_goal_hold_outside_accept_count_;
          near_goal_pose_hold_active_ = false;
          near_goal_hold_outside_accept_count_ = 0;
          RCLCPP_INFO(
            get_logger(),
            "Near-goal pose hold released: outside_accept_cycles=%zu, outside_release=%s. "
            "Returning control to MPPI.",
            outside_accept_cycles,
            outside_release_tolerance ? "true" : "false");
        } else {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Near-goal pose hold: chassis is stopped, waiting for GoalChecker confirmation. "
            "lon=%.4f lat=%.4f yaw=%.4f, outside_accept_count=%zu.",
            lon_err, lat_err, yaw_err, near_goal_hold_outside_accept_count_);
        }
      }
    }
    return;
  }

  // near-goal hold 进入门控：位姿满足容差 AND 底盘实际速度已足够低，才允许 ControllerServer 接管。
  // 否则把动态减速过程交给 MPPI final_yaw，避免在底盘仍有较大惯性时抢占造成卡顿。
  // 注意：必须用原始 odom 速度，不能用 getThresholdedTwist()，后者会把小速度按 min_*_threshold 清零，
  // 导致底盘仍有 0.02~0.03 rad/s 时被误判为已停稳而提前进入 hold。
  if (near_goal_hold_position_when_pose_reached_ &&
    isWithinGoalCheckerTolerance(pose, false, true))
  {
    const nav_2d_msgs::msg::Twist2D raw_odom_twist = odom_sub_->getTwist();
    const bool speed_low_enough_for_hold =
      std::fabs(raw_odom_twist.x) <= near_goal_hold_enter_vx_ &&
      std::fabs(raw_odom_twist.theta) <= near_goal_hold_enter_wz_;
    if (speed_low_enough_for_hold) {
      near_goal_pose_hold_active_ = true;
      near_goal_hold_outside_accept_count_ = 0;
      RCLCPP_INFO(
        get_logger(),
        "Near-goal pose reached and chassis is slow (raw vx=%.4f wz=%.4f <= thresholds %.3f/%.3f): "
        "holding zero velocity for final stop. GoalChecker will confirm success.",
        raw_odom_twist.x, raw_odom_twist.theta,
        near_goal_hold_enter_vx_, near_goal_hold_enter_wz_);
      publishZeroVelocity();
      return;
    }
    // 速度仍偏高：让 MPPI 继续完成动态减速，不抢占
  }

  auto t_progress_start = std::chrono::high_resolution_clock::now();
  const bool in_goal_accept_position = isWithinGoalCheckerTolerance(pose, false, false);
  if (near_goal_progress_enabled_ && isNearGoal(pose)) {
    if (!checkNearGoalProgress(pose)) {
      if (isWithinGoalCheckerTolerance(pose, true)) {
        publishZeroVelocity();
        publish_control_failure("goal_tolerance_zero_cmd");
        return;
      }
      publish_control_failure("near_goal_progress_failed");
      throw nav2_core::PlannerException(
              "Failed to make near-goal progress. 接近目标点后长时间没有足够位移/转角，可能卡住或被约束。");
    }
  } else if (!in_goal_accept_position) {
    if (in_near_goal_progress_mode_) {
      RCLCPP_INFO(get_logger(), "Leaving near-goal progress mode.");
      resetNearGoalProgress();
      progress_checker_->reset();
    }
    if (!progress_checker_->check(pose)) {
      publish_control_failure("progress_failed");
      throw nav2_core::PlannerException(
              "Failed to make progress. 进度检查失败，机器人在规定时间内没有有效前进，可能被障碍、空路径或控制失败卡住。");
    }
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
  double controller_target_v = std::numeric_limits<double>::quiet_NaN();
  double controller_target_a = std::numeric_limits<double>::quiet_NaN();

  auto t_compute_start = std::chrono::high_resolution_clock::now();
  try {
    cmd_vel_2d = computeStagedVelocity(pose, twist);
    last_valid_cmd_time_ = now();
    control_succeeded = true;
  } catch (nav2_core::CollisionException & e) {
    publishHardStopVelocity();
    publish_control_failure("collision_detected");
    throw;
  } catch (nav2_core::PlannerException & e) {
    invalidateSpeedPlanningContinuity();
    if (failure_tolerance_ > 0 || failure_tolerance_ == -1.0) {
      RCLCPP_WARN(this->get_logger(), "%s", e.what());
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
                "Controller patience exceeded. 局部控制器在 failure_tolerance 时间内没有生成有效速度，控制耐心超时。");
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
      controller_target_valid = controller->second->getLastTrajectoryTrackingTarget(
        controller_target_v, controller_target_a);
    }
  }
  updateSpeedPlanningContinuityAfterControl(
    control_succeeded, controller_target_valid, controller_target_v, controller_target_a);
  publishSpeedPlanDiagnostics(
    control_succeeded, controller_target_valid, controller_target_v, controller_target_a,
    control_succeeded ? "success" : "control_failed");
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
  // 找到最近的目标点，并计算到目标点的距离，然后作为feedback发布
  // Find the closest pose to current pose on global path
  nav_msgs::msg::Path & current_path = current_path_;
  auto find_closest_pose_idx =
    [&pose, &current_path]() {
      size_t closest_pose_idx = 0;
      double curr_min_dist = std::numeric_limits<double>::max();
      for (size_t curr_idx = 0; curr_idx < current_path.poses.size(); ++curr_idx) {
        double curr_dist = nav2_util::geometry_utils::euclidean_distance(
          pose, current_path.poses[curr_idx]);
        if (curr_dist < curr_min_dist) {
          curr_min_dist = curr_dist;
          closest_pose_idx = curr_idx;
        }
      }
      return closest_pose_idx;
    };

  const size_t closest_pose_idx = find_closest_pose_idx();
  feedback->distance_to_goal =
    nav2_util::geometry_utils::calculate_path_length(current_path_, closest_pose_idx);
  action_server_->publish_feedback(feedback);

  auto t_feedback_end = std::chrono::high_resolution_clock::now();
  auto feedback_ms = std::chrono::duration<double, std::milli>(t_feedback_end - t_feedback_start).count();

  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 500,
    "ControllerServer command: phase=%s controller=%s goal_checker=%s "
    "cmd_vel.linear=(x=%.3f y=%.3f z=%.3f) cmd_vel.angular=(x=%.3f y=%.3f z=%.3f)",
    controllerPhaseName(controller_phase_),
    active_controller_for_log_.c_str(),
    current_goal_checker_.c_str(),
    cmd_vel_2d.twist.linear.x,
    cmd_vel_2d.twist.linear.y,
    cmd_vel_2d.twist.linear.z,
    cmd_vel_2d.twist.angular.x,
    cmd_vel_2d.twist.angular.y,
    cmd_vel_2d.twist.angular.z);

  RCLCPP_DEBUG(get_logger(), "Publishing velocity at time %.2f", now().seconds());
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
      "ControllerServer timing (ms): total=%.3f getPose=%.3f progress=%.3f odom=%.3f "
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
        get_logger(), "Terminating action, invalid controller %s requested. 终止导航：请求的 controller_id 无效。",
        goal->controller_id.c_str());
      action_server_->terminate_current();
      return;
    }

    std::string requested_goal_checker;
    if (!findGoalCheckerId(goal->goal_checker_id, requested_goal_checker)) {
      RCLCPP_INFO(
        get_logger(), "Terminating action, invalid goal checker %s requested. 终止导航：请求的 goal_checker_id 无效。",
        goal->goal_checker_id.c_str());
      action_server_->terminate_current();
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    const bool same_controller = requested_controller == current_controller_;
    const bool same_goal_checker = requested_goal_checker == current_goal_checker_;
    const bool same_goal = isSamePathGoal(goal->path);
    const bool got_pose = getRobotPose(pose);
    const bool in_accept_position =
      got_pose && isWithinGoalCheckerTolerance(pose, false, false);
    if (same_controller && same_goal_checker && same_goal && in_accept_position)
    {
      RCLCPP_INFO(
        get_logger(),
        "忽略近目标 path 更新：目标点不变且当前位置已进入 goal_checker 2cm 接受窗口，"
        "继续使用当前末端路径完成精调，避免重置 goal_checker 和 MPPI 末端状态。pending path poses=%zu",
        goal->path.poses.size());
      return;
    }

    if (same_goal && !in_accept_position) {
      if (got_pose) {
        RCLCPP_INFO(
          get_logger(),
          "近目标 path 更新未忽略：目标点不变，但当前位置尚未进入 goal_checker 2cm 接受窗口。");
      } else {
        RCLCPP_INFO(
          get_logger(),
          "近目标 path 更新未忽略：目标点不变，但当前无法获取机器人位姿，无法确认是否进入 "
          "goal_checker 2cm 接受窗口。");
      }
    } else if (same_goal && (!same_controller || !same_goal_checker)) {
      RCLCPP_INFO(
        get_logger(),
        "近目标 path 更新未忽略：目标点不变，但 controller 或 goal_checker 已变化，"
        "same_controller=%s, same_goal_checker=%s。",
        same_controller ? "true" : "false",
        same_goal_checker ? "true" : "false");
    }

    current_controller_ = requested_controller;
    current_goal_checker_ = requested_goal_checker;
    RCLCPP_INFO(get_logger(), "Passing new path to controller.");
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
    RCLCPP_WARN(
      get_logger(),
      "[ControllerStopDebug] Publishing zero velocity: actual odom velocity vx=%.3f m/s, "
      "vy=%.3f m/s, wz=%.3f rad/s, cmd_vel subscribers=%zu. This is a stop request; "
      "the velocity smoother and chassis still need time to decelerate.",
      actual_twist.x, actual_twist.y, actual_twist.theta, subscriber_count);
  }
  publishVelocity(velocity);
}

void ControllerServer::publishHardStopVelocity()
{
  if (hard_stop_publisher_ && hard_stop_publisher_->is_activated()) {
    hard_stop_publisher_->publish(std::make_unique<std_msgs::msg::Empty>());
    RCLCPP_ERROR(
      get_logger(),
      "Hard-stop request published on '%s' (subscribers=%zu).",
      hard_stop_topic_.c_str(), hard_stop_publisher_->get_subscription_count());
  } else {
    RCLCPP_ERROR(
      get_logger(), "Hard-stop publisher is inactive; falling back to a zero cmd_vel request.");
  }
  publishZeroVelocity();
}

bool ControllerServer::isGoalReached()
{
  if (near_goal_progress_acceptance_reached_) {
    RCLCPP_INFO(
      get_logger(),
      "Reached the goal by near-goal acceptance fallback. "
      "近目标进度检查已确认误差满足接受容差，返回到位成功。");
    return true;
  }

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
      "Unable to transform end pose for goal checker plugin result logging.");
    return false;
  }

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
    "GoalChecker plugin result: id=%s type=%s result=%s "
    "goal=(frame=%s x=%.3f y=%.3f yaw=%.3f) "
    "robot=(frame=%s x=%.3f y=%.3f yaw=%.3f) "
    "errors=(lon=%.4f lat=%.4f yaw=%.4f dist=%.4f) "
    "tolerances_available=%s tolerances=(lon=%.4f lat=%.4f yaw=%.4f "
    "trans_vel=%.4f rot_vel=%.4f) velocity=(linear=%.4f angular=%.4f)",
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
      name.rfind("near_goal_progress.", 0) != 0 &&
      name.rfind("staged_control.", 0) != 0)
    {
      continue;
    }

    if (!dynamic_params_lock_.try_lock()) {
      RCLCPP_WARN(
        get_logger(),
        "Unable to dynamically change Parameters while the controller is currently running");
      result.successful = false;
      result.reason =
        "Unable to dynamically change Parameters while the controller is currently running";
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
      } else if (name == "near_goal_progress.distance") {
        near_goal_progress_distance_ = parameter.as_double();
      } else if (name == "near_goal_progress.required_movement_radius") {
        near_goal_progress_required_movement_radius_ = parameter.as_double();
      } else if (name == "near_goal_progress.required_movement_angle") {
        near_goal_progress_required_movement_angle_ = parameter.as_double();
      } else if (name == "near_goal_progress.movement_time_allowance") {
        near_goal_progress_time_allowance_ =
          rclcpp::Duration::from_seconds(parameter.as_double());
      } else if (name == "near_goal_progress.lateral_stall_lon_tolerance") {
        near_goal_lateral_stall_lon_tolerance_ = parameter.as_double();
        resetNearGoalLateralStall();
      } else if (name == "near_goal_progress.lateral_stall_yaw_tolerance") {
        near_goal_lateral_stall_yaw_tolerance_ = parameter.as_double();
        resetNearGoalLateralStall();
      } else if (name == "near_goal_progress.lateral_stall_lat_tolerance") {
        near_goal_lateral_stall_lat_tolerance_ = parameter.as_double();
        resetNearGoalLateralStall();
      } else if (name == "near_goal_progress.lateral_stall_min_improvement") {
        near_goal_lateral_stall_min_improvement_ = parameter.as_double();
        resetNearGoalLateralStall();
      } else if (name == "near_goal_progress.lateral_stall_time_allowance") {
        near_goal_lateral_stall_time_allowance_ =
          rclcpp::Duration::from_seconds(parameter.as_double());
        resetNearGoalLateralStall();
      } else if (name == "near_goal_progress.hold_enter_vx") {
        near_goal_hold_enter_vx_ = parameter.as_double();
      } else if (name == "near_goal_progress.hold_enter_wz") {
        near_goal_hold_enter_wz_ = parameter.as_double();
      } else if (name == "near_goal_progress.hold_release_lon_tolerance") {
        near_goal_hold_release_lon_tolerance_ = parameter.as_double();
      } else if (name == "near_goal_progress.hold_release_lat_tolerance") {
        near_goal_hold_release_lat_tolerance_ = parameter.as_double();
      } else if (name == "near_goal_progress.hold_release_yaw_tolerance") {
        near_goal_hold_release_yaw_tolerance_ = parameter.as_double();
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
            "Rejected current_controller '%s'. Available controllers are: %s.",
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
            "Rejected current_goal_checker '%s'. Available goal checkers are: %s.",
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
            "Rejected staged_control.track_controller '%s'. Available controllers are: %s.",
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
            "Rejected staged_control.approach_controller '%s'. Available controllers are: %s.",
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
      if (name == "near_goal_progress.enabled") {
        near_goal_progress_enabled_ = parameter.as_bool();
        resetNearGoalProgress();
      } else if (name == "near_goal_progress.hold_position_when_goal_pose_reached") {
        near_goal_hold_position_when_pose_reached_ = parameter.as_bool();
        near_goal_pose_hold_active_ = false;
        near_goal_hold_outside_accept_count_ = 0;
      } else if (name == "near_goal_progress.lateral_stall_enabled") {
        near_goal_lateral_stall_enabled_ = parameter.as_bool();
        resetNearGoalLateralStall();
      } else if (name == "staged_control.enabled") {
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
