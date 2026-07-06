// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Samsung Research America
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
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "builtin_interfaces/msg/duration.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "nav2_planner/planner_server.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2/utils.h"
// Generic planner-fallback interface: any planner implementing FallbackProvider
// can ask the server to dispatch the configured fallback planner.
#include "nav2_route_planner/planner_fallback.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_planner
{

PlannerServer::PlannerServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("planner_server", "", options),
  gp_loader_("nav2_core", "nav2_core::GlobalPlanner"),
  default_ids_{"GridBased"},
  default_types_{"nav2_navfn_planner/NavfnPlanner"},
  costmap_(nullptr)
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Declare this node's parameters
  declare_parameter("planner_plugins", default_ids_);
  declare_parameter("expected_planner_frequency", 1.0);
  // active_planner: which registered planner key to use when the action
  // caller's planner_id is empty or does not match any registered plugin.
  // Switchable at runtime: ros2 param set /planner_server active_planner <key>
  declare_parameter("active_planner", std::string(""));
  // override_planner_id: when true, ignore caller-supplied planner_id and
  // always dispatch to active_planner_. Used for runtime hot-swapping when
  // upstream BTs hard-code planner_id="GridBased". Switchable at runtime:
  //   ros2 param set /planner_server override_planner_id true|false
  declare_parameter("override_planner_id", false);
  // route_fallback_planner: when the dispatched planner is the route planner
  // and it returns no path SPECIFICALLY because the map has no route graph,
  // retry with this free-space planner. Empty = no fallback.
  declare_parameter("route_fallback_planner", std::string(""));

  // 末端直线插值统一后处理（原 smac fork 定制上收）：开关 + 参数
  declare_parameter("goal_line_interp_enable", false);
  declare_parameter("goal_line_interp_tail_distance", 0.3);
  declare_parameter("goal_line_interp_step", 0.02);

  get_parameter("planner_plugins", planner_ids_);
  if (planner_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
    }
  }

  // Setup the global costmap
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "global_costmap", std::string{get_namespace()}, "global_costmap");
}

PlannerServer::~PlannerServer()
{
  /*
   * Backstop ensuring this state is destroyed, even if deactivate/cleanup are
   * never called.
   */
  planners_.clear();
  costmap_thread_.reset();
}

nav2_util::CallbackReturn
PlannerServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  costmap_ros_->configure();
  costmap_ = costmap_ros_->getCostmap();

  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

  RCLCPP_DEBUG(
    get_logger(), "Costmap size: %d,%d",
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  tf_ = costmap_ros_->getTfBuffer();

  planner_types_.resize(planner_ids_.size());

  auto node = shared_from_this();

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    try {
      planner_types_[i] = nav2_util::get_plugin_type_param(
        node, planner_ids_[i]);
      nav2_core::GlobalPlanner::Ptr planner =
        gp_loader_.createUniqueInstance(planner_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created global planner plugin %s of type %s",
        planner_ids_[i].c_str(), planner_types_[i].c_str());
      planner->configure(node, planner_ids_[i], tf_, costmap_ros_);
      planners_.insert({planner_ids_[i], planner});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create global planner. Exception: %s",
        ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    planner_ids_concat_ += planner_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Planner Server has %s planners available.", planner_ids_concat_.c_str());

  // Resolve active_planner_ from parameter (default = first registered planner)
  {
    std::string active_param;
    get_parameter("active_planner", active_param);
    if (!active_param.empty() && planners_.find(active_param) != planners_.end()) {
      active_planner_ = active_param;
    } else {
      if (!active_param.empty()) {
        RCLCPP_WARN(
          get_logger(),
          "active_planner '%s' is not a registered planner; defaulting to '%s'",
          active_param.c_str(), planner_ids_.front().c_str());
      }
      active_planner_ = planner_ids_.empty() ? "" : planner_ids_.front();
    }
    RCLCPP_INFO(get_logger(), "Active planner set to '%s'", active_planner_.c_str());
  }

  // Resolve override flag
  get_parameter("override_planner_id", override_planner_id_);
  RCLCPP_INFO(
    get_logger(),
    "override_planner_id = %s (when true, every getPlan() ignores caller's "
    "planner_id and uses active_planner='%s')",
    override_planner_id_ ? "true" : "false", active_planner_.c_str());

  get_parameter("goal_line_interp_enable", goal_line_interp_enable_);
  get_parameter("goal_line_interp_tail_distance", goal_line_interp_tail_distance_);
  get_parameter("goal_line_interp_step", goal_line_interp_step_);
  if (goal_line_interp_enable_) {
    RCLCPP_INFO(
      get_logger(),
      "goal_line_interp enabled: tail_distance=%.2f step=%.2f",
      goal_line_interp_tail_distance_, goal_line_interp_step_);
  }

  // Resolve route→free-space fallback planner
  get_parameter("route_fallback_planner", route_fallback_planner_);
  if (!route_fallback_planner_.empty()) {
    if (planners_.count(route_fallback_planner_)) {
      RCLCPP_INFO(
        get_logger(),
        "route_fallback_planner = '%s' (used when the route planner finds no "
        "route graph)", route_fallback_planner_.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "route_fallback_planner '%s' is not a registered planner; route "
        "fallback disabled", route_fallback_planner_.c_str());
      route_fallback_planner_.clear();
    }
  }

  double expected_planner_frequency;
  get_parameter("expected_planner_frequency", expected_planner_frequency);
  if (expected_planner_frequency > 0) {
    max_planner_duration_ = 1 / expected_planner_frequency;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
      " than 0.0 to turn on duration overrrun warning messages", expected_planner_frequency);
    max_planner_duration_ = 0.0;
  }

  // Initialize pubs & subs
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);

  // Create the action servers for path planning to a pose and through poses
  action_server_pose_ = std::make_unique<ActionServerToPose>(
    shared_from_this(),
    "compute_path_to_pose",
    std::bind(&PlannerServer::computePlan, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);

  action_server_poses_ = std::make_unique<ActionServerThroughPoses>(
    shared_from_this(),
    "compute_path_through_poses",
    std::bind(&PlannerServer::computePlanThroughPoses, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  plan_publisher_->on_activate();
  action_server_pose_->activate();
  action_server_poses_->activate();
  costmap_ros_->activate();

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->activate();
  }

  auto node = shared_from_this();

  is_path_valid_service_ = node->create_service<nav2_msgs::srv::IsPathValid>(
    "is_path_valid",
    std::bind(
      &PlannerServer::isPathValid, this,
      std::placeholders::_1, std::placeholders::_2));

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&PlannerServer::dynamicParametersCallback, this, _1));

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_pose_->deactivate();
  action_server_poses_->deactivate();
  plan_publisher_->on_deactivate();

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

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->deactivate();
  }

  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  action_server_pose_.reset();
  action_server_poses_.reset();
  plan_publisher_.reset();
  tf_.reset();

  /*
   * Double check whether something else transitioned it to INACTIVE
   * already, e.g. the rcl preshutdown callback.
   */
  if (costmap_ros_->get_current_state().id() ==
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    costmap_ros_->cleanup();
  }

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->cleanup();
  }

  planners_.clear();
  costmap_thread_.reset();
  costmap_ = nullptr;
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

template<typename T>
bool PlannerServer::isServerInactive(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server == nullptr || !action_server->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return true;
  }

  return false;
}

void PlannerServer::waitForCostmap()
{
  // Don't compute a plan until costmap is valid (after clear costmap)
  rclcpp::Rate r(100);
  while (!costmap_ros_->isCurrent()) {
    r.sleep();
  }
}

template<typename T>
bool PlannerServer::isCancelRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server->is_cancel_requested()) {
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
    action_server->terminate_all();
    return true;
  }

  return false;
}

template<typename T>
void PlannerServer::getPreemptedGoalIfRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

template<typename T>
bool PlannerServer::getStartPose(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal,
  geometry_msgs::msg::PoseStamped & start)
{
  if (goal->use_start) {
    start = goal->start;
  } else if (!costmap_ros_->getRobotPose(start)) {
    action_server->terminate_current();
    return false;
  }

  return true;
}

template<typename T>
bool PlannerServer::transformPosesToGlobalFrame(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  geometry_msgs::msg::PoseStamped & curr_start,
  geometry_msgs::msg::PoseStamped & curr_goal)
{
  if (!costmap_ros_->transformPoseToGlobalFrame(curr_start, curr_start) ||
    !costmap_ros_->transformPoseToGlobalFrame(curr_goal, curr_goal))
  {
    RCLCPP_WARN(
      get_logger(), "Could not transform the start or goal pose in the costmap frame");
    action_server->terminate_current();
    return false;
  }

  return true;
}

template<typename T>
bool PlannerServer::validatePath(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  const geometry_msgs::msg::PoseStamped & goal,
  const nav_msgs::msg::Path & path,
  const std::string & planner_id)
{
  if (path.poses.size() == 0) {
    RCLCPP_WARN(
      get_logger(), "Planning algorithm %s failed to generate a valid"
      " path to (%.2f, %.2f)", planner_id.c_str(),
      goal.pose.position.x, goal.pose.position.y);
    action_server->terminate_current();
    return false;
  }

  RCLCPP_DEBUG(
    get_logger(),
    "Found valid path of size %zu to (%.2f, %.2f)",
    path.poses.size(), goal.pose.position.x,
    goal.pose.position.y);

  return true;
}

void
PlannerServer::computePlanThroughPoses()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  auto start_time = steady_clock_.now();

  // Initialize the ComputePathToPose goal and result
  auto goal = action_server_poses_->get_current_goal();
  auto result = std::make_shared<ActionThroughPoses::Result>();
  nav_msgs::msg::Path concat_path;

  try {
    if (isServerInactive(action_server_poses_) || isCancelRequested(action_server_poses_)) {
      return;
    }

    waitForCostmap();

    getPreemptedGoalIfRequested(action_server_poses_, goal);

    if (goal->goals.size() == 0) {
      RCLCPP_WARN(
        get_logger(),
        "Compute path through poses requested a plan with no viapoint poses, returning.");
      action_server_poses_->terminate_current();
    }

    // Use start pose if provided otherwise use current robot pose
    geometry_msgs::msg::PoseStamped start;
    if (!getStartPose(action_server_poses_, goal, start)) {
      return;
    }

    // Get consecutive paths through these points
    geometry_msgs::msg::PoseStamped curr_start, curr_goal;
    for (unsigned int i = 0; i != goal->goals.size(); i++) {
      // Get starting point
      if (i == 0) {
        curr_start = start;
      } else {
        // pick the end of the last planning task as the start for the next one
        // to allow for path tolerance deviations
        curr_start = concat_path.poses.back();
        curr_start.header = concat_path.header;
      }
      curr_goal = goal->goals[i];

      // Transform them into the global frame
      if (!transformPosesToGlobalFrame(action_server_poses_, curr_start, curr_goal)) {
        return;
      }

      // Get plan from start -> goal
      nav_msgs::msg::Path curr_path = getPlan(curr_start, curr_goal, goal->planner_id);

      // check path for validity
      if (!validatePath(action_server_poses_, curr_goal, curr_path, goal->planner_id)) {
        return;
      }

      // Concatenate paths together
      concat_path.poses.insert(
        concat_path.poses.end(), curr_path.poses.begin(), curr_path.poses.end());
      concat_path.header = curr_path.header;
    }

    // Publish the plan for visualization purposes
    result->path = concat_path;
    publishPlan(result->path);

    auto cycle_duration = steady_clock_.now() - start_time;
    result->planning_time = cycle_duration;

    if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_) {
      RCLCPP_WARN(
        get_logger(),
        "Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
        1 / max_planner_duration_, 1 / cycle_duration.seconds());
    }

    action_server_poses_->succeeded_current(result);
  } catch (std::exception & ex) {
    RCLCPP_WARN(
      get_logger(),
      "%s plugin failed to plan through %zu points with final goal (%.2f, %.2f): \"%s\"",
      goal->planner_id.c_str(), goal->goals.size(), goal->goals.back().pose.position.x,
      goal->goals.back().pose.position.y, ex.what());
    action_server_poses_->terminate_current();
  }
}

void
PlannerServer::computePlan()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  auto start_time = steady_clock_.now();

  // Initialize the ComputePathToPose goal and result
  auto goal = action_server_pose_->get_current_goal();
  auto result = std::make_shared<ActionToPose::Result>();

  try {
    if (isServerInactive(action_server_pose_) || isCancelRequested(action_server_pose_)) {
      return;
    }

    waitForCostmap();

    getPreemptedGoalIfRequested(action_server_pose_, goal);

    // Use start pose if provided otherwise use current robot pose
    geometry_msgs::msg::PoseStamped start;
    if (!getStartPose(action_server_pose_, goal, start)) {
      return;
    }

    // Transform them into the global frame
    geometry_msgs::msg::PoseStamped goal_pose = goal->goal;
    if (!transformPosesToGlobalFrame(action_server_pose_, start, goal_pose)) {
      return;
    }

    result->path = getPlan(start, goal_pose, goal->planner_id);

    if (!validatePath(action_server_pose_, goal_pose, result->path, goal->planner_id)) {
      return;
    }

    // Publish the plan for visualization purposes
    publishPlan(result->path);

    auto cycle_duration = steady_clock_.now() - start_time;
    result->planning_time = cycle_duration;

    if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_) {
      RCLCPP_WARN(
        get_logger(),
        "Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
        1 / max_planner_duration_, 1 / cycle_duration.seconds());
    }

    action_server_pose_->succeeded_current(result);
  } catch (std::exception & ex) {
    RCLCPP_WARN(
      get_logger(), "%s plugin failed to plan calculation to (%.2f, %.2f): \"%s\"",
      goal->planner_id.c_str(), goal->goal.pose.position.x,
      goal->goal.pose.position.y, ex.what());
    action_server_pose_->terminate_current();
  }
}

nav_msgs::msg::Path
PlannerServer::getPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & planner_id)
{
  RCLCPP_DEBUG(
    get_logger(), "Attempting to a find path from (%.2f, %.2f) to "
    "(%.2f, %.2f).", start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);

  // Resolve the effective planner. Two modes:
  //   (1) override_planner_id_ == true: ignore caller's planner_id and
  //       always use active_planner_ (hot-swap mode).
  //   (2) override_planner_id_ == false (nav2 default): caller's
  //       planner_id wins; active_planner_ is only used as fallback when
  //       caller's planner_id is empty or not registered.
  std::string effective_id = planner_id;

  if (override_planner_id_ &&
    !active_planner_.empty() && planners_.count(active_planner_))
  {
    // Mode (1): hot-swap override
    if (!planner_id.empty() && planner_id != active_planner_) {
      RCLCPP_DEBUG(
        get_logger(),
        "override_planner_id=true: caller requested '%s' but dispatching to active '%s'",
        planner_id.c_str(), active_planner_.c_str());
    }
    effective_id = active_planner_;
  } else if (planners_.find(effective_id) == planners_.end()) {
    // Mode (2): fallback to active_planner_ on miss
    if (!active_planner_.empty() && planners_.count(active_planner_)) {
      if (!planner_id.empty()) {
        RCLCPP_WARN(
          get_logger(),
          "Requested planner '%s' not found; falling back to active_planner '%s'",
          planner_id.c_str(), active_planner_.c_str());
      }
      effective_id = active_planner_;
    } else {
      RCLCPP_ERROR(
        get_logger(), "planner '%s' is not a valid planner. "
        "Planner names are: %s", planner_id.c_str(),
        planner_ids_concat_.c_str());
      return nav_msgs::msg::Path();
    }
  }

  auto path = planners_[effective_id]->createPlan(start, goal);

  // Generic planner fallback: if the dispatched planner produced no path AND
  // it implements FallbackProvider AND reports a non-NONE fallback reason
  // (e.g. NO_ROUTE_GRAPH), retry with the configured fallback planner. A
  // genuine empty result with reason NONE is NOT degraded.
  if (path.poses.empty() &&
    !route_fallback_planner_.empty() &&
    route_fallback_planner_ != effective_id &&
    planners_.count(route_fallback_planner_))
  {
    auto * fb = dynamic_cast<nav2_route_planner::FallbackProvider *>(
      planners_[effective_id].get());
    const auto reason = fb ? fb->lastFallbackReason()
      : nav2_route_planner::FallbackReason::NONE;
    if (reason != nav2_route_planner::FallbackReason::NONE) {
      RCLCPP_WARN(
        get_logger(),
        "Planner '%s' requests fallback (reason=%s); dispatching fallback "
        "planner '%s'",
        effective_id.c_str(), nav2_route_planner::toString(reason),
        route_fallback_planner_.c_str());
      path = planners_[route_fallback_planner_]->createPlan(start, goal);
    }
  }

  // 末端直线插值统一后处理（所有 planner 的输出都经过这里）。
  if (goal_line_interp_enable_) {
    applyGoalLineInterpolation(path, start, goal);
  }

  return path;
}

bool PlannerServer::isGoalFootprintFree(const geometry_msgs::msg::PoseStamped & goal)
{
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
  checker(costmap_);
  const double cost = checker.footprintCostAtPose(
    goal.pose.position.x, goal.pose.position.y,
    tf2::getYaw(goal.pose.orientation), costmap_ros_->getRobotFootprint());
  return cost < static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
}

void PlannerServer::appendGoalLine(
  nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & from,
  const geometry_msgs::msg::PoseStamped & goal)
{
  const double dx = goal.pose.position.x - from.pose.position.x;
  const double dy = goal.pose.position.y - from.pose.position.y;
  const double dist = std::hypot(dx, dy);
  const double step = std::max(goal_line_interp_step_, 1e-3);
  const int steps = std::max(1, static_cast<int>(dist / step));
  const double yaw_from = tf2::getYaw(from.pose.orientation);
  double yaw_diff = tf2::getYaw(goal.pose.orientation) - yaw_from;
  while (yaw_diff > M_PI) {yaw_diff -= 2.0 * M_PI;}
  while (yaw_diff < -M_PI) {yaw_diff += 2.0 * M_PI;}

  for (int i = 1; i <= steps; ++i) {
    const double f = static_cast<double>(i) / steps;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = from.pose.position.x + f * dx;
    pose.pose.position.y = from.pose.position.y + f * dy;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_from + f * yaw_diff);
    pose.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(pose);
  }
}

void PlannerServer::applyGoalLineInterpolation(
  nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  const auto dist_to_goal = [&goal](const geometry_msgs::msg::Pose & p) {
      return std::hypot(
        p.position.x - goal.pose.position.x, p.position.y - goal.pose.position.y);
    };

  if (path.poses.empty()) {
    // 失败兜底：仅在已贴近 goal 且 goal footprint 无碰撞时直线补线——
    // 远距离失败仍交给 BT 等待重规划，不掩盖规划失败。判定半径与尾段
    // 替换窗口共用 tail_distance（同一语义：goal 附近直线接管范围）。
    const double dist = dist_to_goal(start.pose);
    if (dist > goal_line_interp_tail_distance_ || !isGoalFootprintFree(goal)) {
      return;
    }
    RCLCPP_WARN(
      get_logger(),
      "goal_line_interp: planner returned empty path near goal (%.2fm) — "
      "using straight-line interpolation start→goal.", dist);
    path.header.frame_id = costmap_ros_->getGlobalFrameID();
    path.header.stamp = now();
    geometry_msgs::msg::PoseStamped start_pose = start;
    start_pose.header = path.header;
    path.poses.push_back(start_pose);
    appendGoalLine(path, start_pose, goal);
    return;
  }

  // 成功路径：尾点已进入替换窗口 → 截断窗口内尾段（末端机动，如朝向对齐
  // 弯钩），直线插值补到 goal。窗口外路径原样保留。
  if (dist_to_goal(path.poses.back().pose) > goal_line_interp_tail_distance_ ||
    !isGoalFootprintFree(goal))
  {
    return;
  }
  size_t keep = path.poses.size();
  while (keep > 1 && dist_to_goal(path.poses[keep - 1].pose) <=
    goal_line_interp_tail_distance_)
  {
    --keep;
  }
  path.poses.resize(keep);
  RCLCPP_DEBUG(
    get_logger(),
    "goal_line_interp: replaced path tail within %.2fm of goal with straight "
    "interpolation (%zu poses kept).", goal_line_interp_tail_distance_, keep);
  appendGoalLine(path, path.poses.back(), goal);
}

void
PlannerServer::publishPlan(const nav_msgs::msg::Path & path)
{
  auto msg = std::make_unique<nav_msgs::msg::Path>(path);
  if (plan_publisher_->is_activated() && plan_publisher_->get_subscription_count() > 0) {
    plan_publisher_->publish(std::move(msg));
  }
}

void PlannerServer::isPathValid(
  const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
  std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response)
{
  response->is_valid = true;

  if (request->path.poses.empty()) {
    response->is_valid = false;
    return;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  unsigned int closest_point_index = 0;
  if (costmap_ros_->getRobotPose(current_pose)) {
    float current_distance = std::numeric_limits<float>::max();
    float closest_distance = current_distance;
    geometry_msgs::msg::Point current_point = current_pose.pose.position;
    for (unsigned int i = 0; i < request->path.poses.size(); ++i) {
      geometry_msgs::msg::Point path_point = request->path.poses[i].pose.position;

      current_distance = nav2_util::geometry_utils::euclidean_distance(
        current_point,
        path_point);

      if (current_distance < closest_distance) {
        closest_point_index = i;
        closest_distance = current_distance;
      }
    }

    /**
     * The lethal check starts at the closest point to avoid points that have already been passed
     * and may have become occupied
     */
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
    unsigned int mx = 0;
    unsigned int my = 0;
    for (unsigned int i = closest_point_index; i < request->path.poses.size(); ++i) {
      costmap_->worldToMap(
        request->path.poses[i].pose.position.x,
        request->path.poses[i].pose.position.y, mx, my);
      unsigned int cost = costmap_->getCost(mx, my);

      if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        response->is_valid = false;
      }
    }
  }
}

rcl_interfaces::msg::SetParametersResult
PlannerServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "expected_planner_frequency") {
        if (parameter.as_double() > 0) {
          max_planner_duration_ = 1 / parameter.as_double();
        } else {
          RCLCPP_WARN(
            get_logger(),
            "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
            " than 0.0 to turn on duration overrrun warning messages", parameter.as_double());
          max_planner_duration_ = 0.0;
        }
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == "active_planner") {
        const std::string requested = parameter.as_string();
        if (planners_.find(requested) == planners_.end()) {
          RCLCPP_WARN(
            get_logger(),
            "Rejecting active_planner='%s': not a registered planner. Available: %s",
            requested.c_str(), planner_ids_concat_.c_str());
          result.successful = false;
          result.reason = "unknown planner key: " + requested;
          return result;
        }
        RCLCPP_INFO(
          get_logger(), "active_planner: '%s' -> '%s'",
          active_planner_.c_str(), requested.c_str());
        active_planner_ = requested;
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == "override_planner_id") {
        const bool old_v = override_planner_id_;
        override_planner_id_ = parameter.as_bool();
        RCLCPP_INFO(
          get_logger(),
          "override_planner_id: %s -> %s (active_planner='%s')",
          old_v ? "true" : "false",
          override_planner_id_ ? "true" : "false",
          active_planner_.c_str());
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_planner

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_planner::PlannerServer)
