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
  default_goal_checker_ids_{"goal_checker"},
  default_goal_checker_types_{"nav2_controller::SimpleGoalChecker"},
  lp_loader_("nav2_core", "nav2_core::Controller"),
  default_ids_{"FollowPath"},
  default_types_{"dwb_core::DWBLocalPlanner"}
{
  RCLCPP_INFO(get_logger(), "Creating controller server");

  declare_parameter("controller_frequency", 20.0);

  declare_parameter("progress_checker_plugin", default_progress_checker_id_);
  declare_parameter("goal_checker_plugins", default_goal_checker_ids_);
  declare_parameter("controller_plugins", default_ids_);
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

  for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
    goal_checker_ids_concat_ += goal_checker_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Controller Server has %s goal checkers available.", goal_checker_ids_concat_.c_str());

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

  for (size_t i = 0; i != controller_ids_.size(); i++) {
    controller_ids_concat_ += controller_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Controller Server has %s controllers available.", controller_ids_concat_.c_str());

  odom_sub_ = std::make_unique<nav_2d_utils::OdomSubscriber>(node);
  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  goal_error_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/navigation_goal_error",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());

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
  vel_publisher_->on_activate();
  goal_error_publisher_->on_activate();
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

  end_pose_ = path.poses.back();
  end_pose_.header.frame_id = path.header.frame_id;
  goal_checkers_[current_goal_checker_]->reset();
  near_goal_progress_acceptance_reached_ = false;

  RCLCPP_DEBUG(
    get_logger(), "Path end point is (%.2f, %.2f)",
    end_pose_.pose.position.x, end_pose_.pose.position.y);

  current_path_ = path;
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

  if (lon_ok && lat_ok && yaw_accept && stopped_accept) {
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
    cmd_vel_2d =
      controllers_[current_controller_]->computeVelocityCommands(
      pose,
      nav_2d_utils::twist2Dto3D(twist),
      goal_checkers_[current_goal_checker_].get());
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

  RCLCPP_DEBUG(get_logger(), "Publishing velocity at time %.2f", now().seconds());
  auto t_publish_start = std::chrono::high_resolution_clock::now();
  publishVelocity(cmd_vel_2d);
  auto t_publish_end = std::chrono::high_resolution_clock::now();
  auto publish_ms = std::chrono::duration<double, std::milli>(t_publish_end - t_publish_start).count();

  auto t_total_end = std::chrono::high_resolution_clock::now();
  auto total_ms = std::chrono::duration<double, std::milli>(t_total_end - t_total_start).count();
  auto measured_ms = pose_ms + progress_ms + odom_ms + compute_ms + feedback_ms + publish_ms;
  auto unaccounted_ms = total_ms - measured_ms;

  if (total_ms >= kSlowTimingWarnMs || 1) {
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
  nav_2d_utils::transformPose(
    costmap_ros_->getTfBuffer(), costmap_ros_->getGlobalFrameID(),
    end_pose_, transformed_end_pose, tolerance);

  return goal_checkers_[current_goal_checker_]->isGoalReached(
    pose.pose, transformed_end_pose.pose,
    velocity);
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
}

rcl_interfaces::msg::SetParametersResult
ControllerServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    // Plugin parameters handle their own updates, but near_goal_progress belongs to this server.
    if (name.find('.') != std::string::npos &&
      name.rfind("near_goal_progress.", 0) != 0)
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
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == "near_goal_progress.enabled") {
        near_goal_progress_enabled_ = parameter.as_bool();
        resetNearGoalProgress();
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
