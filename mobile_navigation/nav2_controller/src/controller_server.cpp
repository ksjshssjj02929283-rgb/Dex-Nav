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
  speed_planner_loader_("nav2_core", "nav2_core::SpeedPlanner")
{
  RCLCPP_INFO(get_logger(), "Creating controller server");

  declare_parameter("controller_frequency", 20.0);

  declare_parameter("progress_checker_plugin", default_progress_checker_id_);
  declare_parameter("goal_checker_plugins", default_goal_checker_ids_);
  declare_parameter("controller_plugins", default_ids_);
  declare_parameter("current_goal_checker", default_goal_checker_ids_.front());
  declare_parameter("current_controller", default_ids_.front());
  // ④ 速度规划：默认不启用（空列表 + 空选择器），YAML 显式配置后生效
  declare_parameter("speed_planner_plugins", std::vector<std::string>());
  declare_parameter("current_speed_planner", std::string(""));
  declare_parameter("min_x_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_y_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_theta_velocity_threshold", rclcpp::ParameterValue(0.0001));

  declare_parameter("speed_limit_topic", rclcpp::ParameterValue("speed_limit"));

  declare_parameter("failure_tolerance", rclcpp::ParameterValue(0.0));
  declare_parameter("near_goal_progress.enabled", rclcpp::ParameterValue(true));
  declare_parameter("near_goal_progress.distance", rclcpp::ParameterValue(0.25));
  declare_parameter(
    "near_goal_progress.required_movement_radius", rclcpp::ParameterValue(0.01));
  declare_parameter(
    "near_goal_progress.required_movement_angle", rclcpp::ParameterValue(0.005));
  declare_parameter(
    "near_goal_progress.movement_time_allowance", rclcpp::ParameterValue(8.0));
  declare_parameter("staged_control.enabled", rclcpp::ParameterValue(true));
  declare_parameter("staged_control.track_controller", rclcpp::ParameterValue("MPPIController"));
  declare_parameter("staged_control.approach_controller", rclcpp::ParameterValue("PIDController"));
  declare_parameter("staged_control.approach_distance", rclcpp::ParameterValue(0.3));
  declare_parameter(
    "staged_control.align_heading_tolerance", rclcpp::ParameterValue(0.5235987756));
  declare_parameter("staged_control.align_angular_kp", rclcpp::ParameterValue(1.5));
  declare_parameter("staged_control.align_max_angular_velocity", rclcpp::ParameterValue(0.5));
  declare_parameter("staged_control.align_min_angular_velocity", rclcpp::ParameterValue(0.08));
  declare_parameter("staged_control.prepare.enabled", rclcpp::ParameterValue(true));
  declare_parameter(
    "staged_control.prepare.obstacle_check_distance", rclcpp::ParameterValue(0.35));
  declare_parameter(
    "staged_control.prepare.obstacle_check_half_width", rclcpp::ParameterValue(0.25));
  declare_parameter(
    "staged_control.prepare.obstacle_cost_threshold", rclcpp::ParameterValue(253));
  declare_parameter("staged_control.prepare.backup_speed", rclcpp::ParameterValue(-0.08));
  declare_parameter("staged_control.prepare.backup_duration", rclcpp::ParameterValue(0.6));

  // The costmap node is used in the implementation of the controller
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "local_costmap", std::string{get_namespace()}, "local_costmap");
}

ControllerServer::~ControllerServer()
{
  progress_checker_.reset();
  goal_checkers_.clear();
  controllers_.clear();
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
  get_parameter("near_goal_progress.enabled", near_goal_progress_enabled_);
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
  get_parameter("staged_control.enabled", staged_control_enabled_);
  get_parameter("staged_control.track_controller", track_controller_);
  get_parameter("staged_control.approach_controller", approach_controller_);
  get_parameter("staged_control.approach_distance", approach_distance_);
  get_parameter("staged_control.align_heading_tolerance", align_heading_tolerance_);
  get_parameter("staged_control.align_angular_kp", align_angular_kp_);
  get_parameter("staged_control.align_max_angular_velocity", align_max_angular_velocity_);
  get_parameter("staged_control.align_min_angular_velocity", align_min_angular_velocity_);
  get_parameter("staged_control.prepare.enabled", prepare_enabled_);
  get_parameter("staged_control.prepare.obstacle_check_distance", prepare_obstacle_check_distance_);
  get_parameter("staged_control.prepare.obstacle_check_half_width", prepare_obstacle_check_half_width_);
  get_parameter("staged_control.prepare.obstacle_cost_threshold", prepare_obstacle_cost_threshold_);
  get_parameter("staged_control.prepare.backup_speed", prepare_backup_speed_);
  get_parameter("staged_control.prepare.backup_duration", prepare_backup_duration_);
  RCLCPP_INFO(
    get_logger(),
    "Near-goal progress: enabled=%s distance=%.3f m movement=%.3f m angle=%.3f rad "
    "allowance=%.3f s",
    near_goal_progress_enabled_ ? "true" : "false",
    near_goal_progress_distance_,
    near_goal_progress_required_movement_radius_,
    near_goal_progress_required_movement_angle_,
    near_goal_progress_time_allowance_.seconds());

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
  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  goal_error_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/navigation_goal_error",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
  // 速度规划可视化：transient_local 让 RViz 晚订阅也能收到最新一帧
  speed_plan_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/navigation/speed_plan",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

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
  for (auto & sp : speed_planners_) {
    sp.second->activate();
  }
  vel_publisher_->on_activate();
  goal_error_publisher_->on_activate();
  speed_plan_publisher_->on_activate();
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

  publishZeroVelocity();
  goal_error_publisher_->on_deactivate();
  speed_plan_publisher_->on_deactivate();
  vel_publisher_->on_deactivate();
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
  goal_error_publisher_.reset();
  speed_plan_publisher_.reset();
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

    setPlannerPath(action_server_->get_current_goal()->path);
    progress_checker_->reset();
    resetNearGoalProgress();

    last_valid_cmd_time_ = now();
    rclcpp::WallRate loop_rate(controller_frequency_);
    // added by hz
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    std::chrono::time_point<std::chrono::high_resolution_clock> end;
    std::chrono::duration<double, std::milli> elapsed;
    // end
    while (rclcpp::ok()) {
      if (action_server_ == nullptr || !action_server_->is_server_active()) {
        RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
        return;
      }

      if (action_server_->is_cancel_requested()) {
        RCLCPP_INFO(get_logger(), "Goal was canceled. Stopping the robot.");
        action_server_->terminate_all();
        publishZeroVelocity();
        return;
      }

      // Don't compute a trajectory until costmap is valid (after clear costmap)
      rclcpp::Rate r(100);
      while (!costmap_ros_->isCurrent()) {
        r.sleep();
      }
      // added by hz
      start = std::chrono::high_resolution_clock::now();
      updateGlobalPath();
      end = std::chrono::high_resolution_clock::now();
      elapsed = end - start;
      if(elapsed.count() >= 100)
      {
        RCLCPP_WARN(
          get_logger(), "Global path update cost: %.6f ms",
          elapsed.count());
      }
      // end

      // added by hz
      start = std::chrono::high_resolution_clock::now();
      computeAndPublishVelocity();
      end = std::chrono::high_resolution_clock::now();
      elapsed = end - start;
      if(elapsed.count() >= 100)
      {
        RCLCPP_WARN(
          get_logger(), "Local path update cost: %.6f ms",
          elapsed.count());
      }
      // end

      if (isGoalReached()) {
        RCLCPP_INFO(get_logger(), "Reached the goal!");
        prepare_backup_active_ = false;
        prepare_check_pending_ = true;
        break;
      }

      end = std::chrono::high_resolution_clock::now();
      elapsed = end - start;

      if (!loop_rate.sleep()) {
        // commented by hz
        RCLCPP_WARN(
          get_logger(), "Control loop missed its desired rate of %.4fHz, the cost time is %.6f ms",
          controller_frequency_, elapsed.count());
        // end
        // added by hz
        std::cout << "------------------------------------------------------------" << std::endl;
      }
    }
  } catch (nav2_core::PlannerException & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    action_server_->terminate_current();
    return;
  }catch (std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    action_server_->terminate_current(result);
    return;
  }
  RCLCPP_DEBUG(get_logger(), "Controller succeeded, setting result");

  publishZeroVelocity();

  // TODO(orduno) #861 Handle a pending preemption and set controller name
  action_server_->succeeded_current();
}

void ControllerServer::setPlannerPath(const nav_msgs::msg::Path & path)
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

  // ④ 规划级：path 更新事件驱动生成带速度轨迹并分发（未配置时为空操作）。
  runPlanningPipeline(path);

  end_pose_ = path.poses.back();
  end_pose_.header.frame_id = path.header.frame_id;
  goal_checkers_[current_goal_checker_]->reset();
  near_goal_progress_acceptance_reached_ = false;

  RCLCPP_DEBUG(
    get_logger(), "Path end point is (%.2f, %.2f)",
    end_pose_.pose.position.x, end_pose_.pose.position.y);

  current_path_ = path;
  controller_phase_ = ControllerPhase::TRACK;
}

void ControllerServer::runPlanningPipeline(const nav_msgs::msg::Path & path)
{
  // 规划级（③④）：③ LocalPlanner 暂缺 = 直通（global path 即输入）；
  // ④ SpeedPlanner 从 odom 当前速度出发生成 RobotTrajectory。
  // 分发契约：setTrajectory 仅在轨迹更新时调用一次，控制循环内不重算速度曲线。
  current_trajectory_ = nav_interfaces::msg::RobotTrajectory();
  if (current_speed_planner_.empty()) {
    return;  // 未配置：灰度回退，controller 走 setPlan 几何路径
  }
  auto sp = speed_planners_.find(current_speed_planner_);
  if (sp == speed_planners_.end()) {
    return;
  }

  const nav_2d_msgs::msg::Twist2D odom_twist = getThresholdedTwist(odom_sub_->getTwist());
  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = odom_twist.x;
  current_velocity.linear.y = odom_twist.y;
  current_velocity.angular.z = odom_twist.theta;

  sp->second->setPlan(path);
  current_trajectory_ = sp->second->computeSpeedPlan(path, current_velocity);
  if (current_trajectory_.points.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "Speed planner '%s' returned empty trajectory; controllers keep geometric path. "
      "速度规划返回空轨迹，控制器继续使用几何路径。",
      current_speed_planner_.c_str());
    publishSpeedPlan(current_trajectory_);  // 清除 RViz 旧 marker
    return;
  }

  controllers_[current_controller_]->setTrajectory(current_trajectory_);
  if (staged_control_enabled_) {
    if (track_controller_ != current_controller_) {
      controllers_[track_controller_]->setTrajectory(current_trajectory_);
    }
    if (approach_controller_ != current_controller_ && approach_controller_ != track_controller_) {
      controllers_[approach_controller_]->setTrajectory(current_trajectory_);
    }
  }

  publishSpeedPlan(current_trajectory_);

  RCLCPP_INFO(
    get_logger(),
    "Speed plan dispatched: planner=%s points=%zu v_start=%.3f",
    current_speed_planner_.c_str(), current_trajectory_.points.size(),
    current_velocity.linear.x);
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

  speed_plan_publisher_->publish(marker_array);
}

void ControllerServer::resetNearGoalProgress()
{
  near_goal_baseline_pose_set_ = false;
  in_near_goal_progress_mode_ = false;
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

  RCLCPP_INFO_THROTTLE(
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

bool ControllerServer::getPathHeadingError(
  const geometry_msgs::msg::PoseStamped & pose,
  double & yaw_error)
{
  if (current_path_.poses.size() < 2) {
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

  size_t closest_idx = 0;
  double closest_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < current_path_.poses.size(); ++i) {
    const double dx = path_frame_pose.pose.position.x - current_path_.poses[i].pose.position.x;
    const double dy = path_frame_pose.pose.position.y - current_path_.poses[i].pose.position.y;
    const double dist = std::hypot(dx, dy);
    if (dist < closest_dist) {
      closest_dist = dist;
      closest_idx = i;
    }
  }

  size_t from_idx = closest_idx;
  size_t to_idx = std::min(closest_idx + 3, current_path_.poses.size() - 1);
  if (to_idx == from_idx && from_idx > 0) {
    --from_idx;
  }
  if (to_idx == from_idx) {
    return false;
  }

  const auto & from = current_path_.poses[from_idx].pose.position;
  const auto & to = current_path_.poses[to_idx].pose.position;
  const double path_yaw = std::atan2(to.y - from.y, to.x - from.x);
  const double robot_yaw = tf2::getYaw(path_frame_pose.pose.orientation);
  yaw_error = angles::shortest_angular_distance(robot_yaw, path_yaw);
  return true;
}

bool ControllerServer::isFrontObstacleDetected()
{
  auto costmap = costmap_ros_->getCostmap();
  if (costmap == nullptr) {
    return false;
  }

  const double resolution = std::max(costmap->getResolution(), 0.02);
  const unsigned char cost_threshold = static_cast<unsigned char>(
    std::clamp(prepare_obstacle_cost_threshold_, 0, 255));

  for (double x = resolution; x <= prepare_obstacle_check_distance_; x += resolution) {
    for (double y = -prepare_obstacle_check_half_width_;
      y <= prepare_obstacle_check_half_width_;
      y += resolution)
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
          continue;
        }
      } catch (const tf2::TransformException &) {
        continue;
      }

      unsigned int mx = 0;
      unsigned int my = 0;
      if (!costmap->worldToMap(
          global_sample.pose.position.x, global_sample.pose.position.y, mx, my))
      {
        continue;
      }
      if (costmap->getCost(mx, my) >= cost_threshold) {
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
    if ((now() - prepare_backup_start_time_).seconds() < prepare_backup_duration_) {
      return true;
    }
    prepare_backup_active_ = false;
  }

  if (!prepare_check_pending_) {
    return false;
  }

  prepare_check_pending_ = false;
  if (!isFrontObstacleDetected()) {
    RCLCPP_INFO(get_logger(), "Prepare phase checked once at navigation start: no front obstacle.");
    return false;
  }

  prepare_backup_active_ = true;
  prepare_backup_start_time_ = now();
  RCLCPP_WARN(
    get_logger(),
    "Prepare phase detected front obstacle, backing up: speed=%.3f m/s duration=%.3f s",
    prepare_backup_speed_,
    prepare_backup_duration_);
  return true;
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
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "Staged control decision: approach has priority, distance_to_goal=%.3f <= "
      "approach_distance=%.3f; using controller=%s",
      distance_to_goal, approach_distance_, approach_controller_.c_str());
    setControllerPhase(ControllerPhase::APPROACH, "within approach distance");
    active_controller_for_log_ = approach_controller_;
    return computeControllerVelocity(approach_controller_, pose, twist);
  }

  double heading_error = 0.0;
  const bool has_path_heading = getPathHeadingError(pose, heading_error);
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 500,
    "Staged control decision: outside approach, distance_to_goal=%.3f > "
    "approach_distance=%.3f; path_heading_available=%s heading_error=%.3f "
    "align_tolerance=%.3f",
    distance_to_goal,
    approach_distance_,
    has_path_heading ? "true" : "false",
    has_path_heading ? heading_error : 0.0,
    align_heading_tolerance_);
  if (has_path_heading && std::fabs(heading_error) > align_heading_tolerance_) {
    setControllerPhase(ControllerPhase::ALIGN, "path heading error exceeds tolerance");
    active_controller_for_log_ = "direct_align";
    const double abs_cmd = std::clamp(
      std::fabs(heading_error) * align_angular_kp_,
      align_min_angular_velocity_,
      align_max_angular_velocity_);
    cmd_vel.twist.angular.z = std::copysign(abs_cmd, heading_error);
    return cmd_vel;
  }

  setControllerPhase(ControllerPhase::TRACK, "path heading aligned");
  active_controller_for_log_ = track_controller_;
  return computeControllerVelocity(track_controller_, pose, twist);
}

void ControllerServer::computeAndPublishVelocity()
{
  constexpr double kSlowTimingWarnMs = 100.0;
  geometry_msgs::msg::PoseStamped pose;
  auto t_total_start = std::chrono::high_resolution_clock::now();

  auto t_pose_start = std::chrono::high_resolution_clock::now();
  if (!getRobotPose(pose)) {
    throw nav2_core::PlannerException(
            "Failed to obtain robot pose. 获取机器人位姿失败，通常需要检查 TF、定位状态或 costmap frame。");
  }
  auto t_pose_end = std::chrono::high_resolution_clock::now();
  auto pose_ms = std::chrono::duration<double, std::milli>(t_pose_end - t_pose_start).count();

  auto t_progress_start = std::chrono::high_resolution_clock::now();
  const bool in_goal_accept_position = isWithinGoalCheckerTolerance(pose, false, false);
  if (near_goal_progress_enabled_ && isNearGoal(pose)) {
    if (!checkNearGoalProgress(pose)) {
      if (isWithinGoalCheckerTolerance(pose, true)) {
        publishZeroVelocity();
        return;
      }
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
      throw nav2_core::PlannerException(
              "Failed to make progress. 进度检查失败，机器人在规定时间内没有有效前进，可能被障碍、空路径或控制失败卡住。");
    }
  }
  auto t_progress_end = std::chrono::high_resolution_clock::now();
  auto progress_ms = std::chrono::duration<double, std::milli>(t_progress_end - t_progress_start).count();

  auto t_odom_start = std::chrono::high_resolution_clock::now();
  nav_2d_msgs::msg::Twist2D twist = getThresholdedTwist(odom_sub_->getTwist());
  auto t_odom_end = std::chrono::high_resolution_clock::now();
  auto odom_ms = std::chrono::duration<double, std::milli>(t_odom_end - t_odom_start).count();

  geometry_msgs::msg::TwistStamped cmd_vel_2d;

  auto t_compute_start = std::chrono::high_resolution_clock::now();
  try {
    cmd_vel_2d = computeStagedVelocity(pose, twist);
    last_valid_cmd_time_ = now();
  } catch (nav2_core::PlannerException & e) {
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
        throw nav2_core::PlannerException(
                "Controller patience exceeded. 局部控制器在 failure_tolerance 时间内没有生成有效速度，控制耐心超时。");
      }
    } else {
      throw nav2_core::PlannerException(e.what());
    }
  }
  auto t_compute_end = std::chrono::high_resolution_clock::now();
  auto compute_ms = std::chrono::duration<double, std::milli>(t_compute_end - t_compute_start).count();

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

  feedback->distance_to_goal =
    nav2_util::geometry_utils::calculate_path_length(current_path_, find_closest_pose_idx());
  action_server_->publish_feedback(feedback);
  auto t_feedback_end = std::chrono::high_resolution_clock::now();
  auto feedback_ms = std::chrono::duration<double, std::milli>(t_feedback_end - t_feedback_start).count();

  RCLCPP_INFO_THROTTLE(
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
    setPlannerPath(goal->path);
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
  geometry_msgs::msg::TwistStamped velocity;
  velocity.twist.angular.x = 0;
  velocity.twist.angular.y = 0;
  velocity.twist.angular.z = 0;
  velocity.twist.linear.x = 0;
  velocity.twist.linear.y = 0;
  velocity.twist.linear.z = 0;
  velocity.header.frame_id = costmap_ros_->getBaseFrameID();
  velocity.header.stamp = now();
  publishVelocity(velocity);
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

  RCLCPP_INFO_THROTTLE(
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
      } else if (name == "staged_control.prepare.obstacle_check_half_width") {
        prepare_obstacle_check_half_width_ = parameter.as_double();
      } else if (name == "staged_control.prepare.backup_speed") {
        prepare_backup_speed_ = parameter.as_double();
      } else if (name == "staged_control.prepare.backup_duration") {
        prepare_backup_duration_ = parameter.as_double();
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
      } else if (name == "staged_control.enabled") {
        staged_control_enabled_ = parameter.as_bool();
        prepare_backup_active_ = false;
      } else if (name == "staged_control.prepare.enabled") {
        prepare_enabled_ = parameter.as_bool();
        prepare_backup_active_ = false;
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
