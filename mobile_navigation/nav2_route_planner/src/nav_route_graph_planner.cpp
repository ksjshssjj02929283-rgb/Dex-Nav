// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include "nav2_route_planner/nav_route_graph_planner.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

using rcl_interfaces::msg::ParameterType;

namespace nav2_route_planner
{

NavRouteGraphPlanner::NavRouteGraphPlanner() = default;

void NavRouteGraphPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_node_ = parent;
  auto node = parent_node_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  global_frame_ = costmap_ros_ ? costmap_ros_->getGlobalFrameID() : std::string("map");

  // ── Declare parameters ─────────────────────────────────────────────
  using nav2_util::declare_parameter_if_not_declared;
  declare_parameter_if_not_declared(node, name_ + ".asset_manager_node_name",
    rclcpp::ParameterValue(params_.asset_manager_node_name));
  declare_parameter_if_not_declared(node, name_ + ".data_load_timeout_sec",
    rclcpp::ParameterValue(params_.data_load_timeout_sec));
  declare_parameter_if_not_declared(node, name_ + ".projection_node_threshold_m",
    rclcpp::ParameterValue(params_.projection_node_threshold_m));
  declare_parameter_if_not_declared(node, name_ + ".projection_edge_threshold_m",
    rclcpp::ParameterValue(params_.projection_edge_threshold_m));
  declare_parameter_if_not_declared(node, name_ + ".w_distance_cost",
    rclcpp::ParameterValue(params_.weights.w_distance));
  declare_parameter_if_not_declared(node, name_ + ".w_floor_penalty",
    rclcpp::ParameterValue(params_.weights.w_floor_penalty));
  declare_parameter_if_not_declared(node, name_ + ".w_door_penalty",
    rclcpp::ParameterValue(params_.weights.w_door_penalty));
  declare_parameter_if_not_declared(node, name_ + ".w_elevator_penalty",
    rclcpp::ParameterValue(params_.weights.w_elevator_penalty));
  declare_parameter_if_not_declared(node, name_ + ".w_stair_factor",
    rclcpp::ParameterValue(params_.weights.w_stair_factor));
  declare_parameter_if_not_declared(node, name_ + ".w_user_pref_mult",
    rclcpp::ParameterValue(params_.weights.w_user_pref_mult));
  declare_parameter_if_not_declared(node, name_ + ".prefer_no_stairs",
    rclcpp::ParameterValue(params_.prefs.prefer_no_stairs));
  declare_parameter_if_not_declared(node, name_ + ".prefer_no_elevator",
    rclcpp::ParameterValue(params_.prefs.prefer_no_elevator));
  declare_parameter_if_not_declared(node, name_ + ".waypoints_per_meter",
    rclcpp::ParameterValue(params_.waypoints_per_meter));
  declare_parameter_if_not_declared(node, name_ + ".bezier_samples_per_edge",
    rclcpp::ParameterValue(params_.bezier_samples_per_edge));
  declare_parameter_if_not_declared(node, name_ + ".publish_route_plan",
    rclcpp::ParameterValue(params_.publish_route_plan));
  declare_parameter_if_not_declared(node, name_ + ".use_final_approach_orientation",
    rclcpp::ParameterValue(params_.use_final_approach_orientation));

  node->get_parameter(name_ + ".asset_manager_node_name", params_.asset_manager_node_name);
  node->get_parameter(name_ + ".data_load_timeout_sec", params_.data_load_timeout_sec);
  node->get_parameter(name_ + ".projection_node_threshold_m", params_.projection_node_threshold_m);
  node->get_parameter(name_ + ".projection_edge_threshold_m", params_.projection_edge_threshold_m);
  node->get_parameter(name_ + ".w_distance_cost", params_.weights.w_distance);
  node->get_parameter(name_ + ".w_floor_penalty", params_.weights.w_floor_penalty);
  node->get_parameter(name_ + ".w_door_penalty", params_.weights.w_door_penalty);
  node->get_parameter(name_ + ".w_elevator_penalty", params_.weights.w_elevator_penalty);
  node->get_parameter(name_ + ".w_stair_factor", params_.weights.w_stair_factor);
  node->get_parameter(name_ + ".w_user_pref_mult", params_.weights.w_user_pref_mult);
  node->get_parameter(name_ + ".prefer_no_stairs", params_.prefs.prefer_no_stairs);
  node->get_parameter(name_ + ".prefer_no_elevator", params_.prefs.prefer_no_elevator);
  node->get_parameter(name_ + ".waypoints_per_meter", params_.waypoints_per_meter);
  node->get_parameter(name_ + ".bezier_samples_per_edge", params_.bezier_samples_per_edge);
  node->get_parameter(name_ + ".publish_route_plan", params_.publish_route_plan);
  node->get_parameter(name_ + ".use_final_approach_orientation", params_.use_final_approach_orientation);

  // ── Construct components ───────────────────────────────────────────
  graph_ = std::make_shared<TopoGraph>();
  loader_ = std::make_shared<RouteGraphLoader>(parent_node_,
      params_.asset_manager_node_name, graph_);
  StartGoalProjector::Params proj_p;
  proj_p.node_threshold_m = params_.projection_node_threshold_m;
  proj_p.edge_threshold_m = params_.projection_edge_threshold_m;
  proj_p.bezier_samples_per_edge = params_.bezier_samples_per_edge;
  projector_ = std::make_shared<StartGoalProjector>(proj_p);
  searcher_ = std::make_shared<AStarSearcher>();
  PathInterpolator::Params interp_p;
  interp_p.waypoints_per_meter = params_.waypoints_per_meter;
  interp_p.bezier_samples_per_edge = params_.bezier_samples_per_edge;
  interp_p.use_final_approach_orientation = params_.use_final_approach_orientation;
  interpolator_ = std::make_shared<PathInterpolator>(interp_p);
  rp_publisher_ = std::make_shared<RoutePlanPublisher>(parent_node_,
      "~/" + name_ + "/route_plan");

  RCLCPP_INFO(logger_,
    "nav2_route_planner '%s' configured. asset_node=%s "
    "node_threshold=%.2fm edge_threshold=%.2fm",
    name_.c_str(), params_.asset_manager_node_name.c_str(),
    params_.projection_node_threshold_m, params_.projection_edge_threshold_m);
}

void NavRouteGraphPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up nav2_route_planner '%s'", name_.c_str());
  rp_publisher_.reset();
  interpolator_.reset();
  searcher_.reset();
  projector_.reset();
  loader_.reset();
  graph_.reset();
}

void NavRouteGraphPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating nav2_route_planner '%s'", name_.c_str());
  if (rp_publisher_) {rp_publisher_->activate();}

  // Best-effort initial load. If it fails, createPlan will retry.
  if (loader_) {
    bool ok = loader_->loadSync(params_.data_load_timeout_sec);
    if (!ok) {
      RCLCPP_WARN(logger_,
        "Initial RouteGraph load failed; will retry on first createPlan().");
    }
  }

  auto node = parent_node_.lock();
  if (node) {
    dyn_params_handler_ = node->add_on_set_parameters_callback(
      std::bind(&NavRouteGraphPlanner::dynamicParametersCallback, this,
        std::placeholders::_1));
  }
}

void NavRouteGraphPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating nav2_route_planner '%s'", name_.c_str());
  if (rp_publisher_) {rp_publisher_->deactivate();}
  dyn_params_handler_.reset();
}

nav_msgs::msg::Path NavRouteGraphPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path empty_path;
  empty_path.header.frame_id = global_frame_;
  empty_path.header.stamp = clock_->now();

  // Lazy load if first activate didn't succeed.
  if (!graph_->isReady()) {
    RCLCPP_WARN(logger_, "TopoGraph not ready, attempting synchronous load");
    if (!loader_->loadSync(params_.data_load_timeout_sec)) {
      RCLCPP_ERROR(logger_, "Cannot load RouteGraph from %s",
        params_.asset_manager_node_name.c_str());
      return empty_path;
    }
  }

  auto lock = graph_->lockShared();

  // ── Step 1: project start ──────────────────────────────────────────
  auto start_proj = projector_->project(start, *graph_);
  if (start_proj.type == StartGoalProjector::ResultType::INVALID) {
    RCLCPP_ERROR(logger_,
      "Start pose (%.2f, %.2f) not within %.2fm of any node or %.2fm of any edge",
      start.pose.position.x, start.pose.position.y,
      params_.projection_node_threshold_m, params_.projection_edge_threshold_m);
    return empty_path;
  }

  // ── Step 2: project goal (must hit a NavPoint) ─────────────────────
  auto goal_id = projector_->projectToNode(goal, *graph_);
  if (!goal_id) {
    RCLCPP_ERROR(logger_,
      "Goal pose (%.2f, %.2f) does not match any NavPoint within %.2fm",
      goal.pose.position.x, goal.pose.position.y,
      params_.projection_node_threshold_m);
    return empty_path;
  }

  // ── Step 3: A* search ──────────────────────────────────────────────
  AStarSearcher::Request req;
  req.start = start_proj;
  req.goal_node_id = *goal_id;
  req.weights = params_.weights;
  req.prefs = params_.prefs;
  // Read live preferences in case dynamic params changed them since
  // configure() ran.
  if (auto node = parent_node_.lock()) {
    node->get_parameter(name_ + ".prefer_no_stairs", req.prefs.prefer_no_stairs);
    node->get_parameter(name_ + ".prefer_no_elevator", req.prefs.prefer_no_elevator);
  }

  auto t0 = std::chrono::steady_clock::now();
  auto result = searcher_->search(req, *graph_);
  auto dt = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now() - t0).count();

  if (!result.success) {
    RCLCPP_ERROR(logger_,
      "A* failed: %s (start=%s goal=%s expanded=%zu)",
      result.failure_reason.c_str(),
      start_proj.id.c_str(), goal_id->c_str(), result.expanded_nodes);
    return empty_path;
  }
  RCLCPP_INFO(logger_,
    "A* found path: %zu nodes, %zu edges, cost=%.2f, expanded=%zu, %ldus",
    result.node_ids.size(), result.edge_ids.size(),
    result.total_cost, result.expanded_nodes, static_cast<long>(dt));

  // ── Step 4: interpolate ────────────────────────────────────────────
  auto plan = interpolator_->interpolate(start, goal, start_proj, *goal_id,
      result, *graph_);

  // ── Step 5: publish RoutePlan + return Path ───────────────────────
  auto stamp = clock_->now();
  if (params_.publish_route_plan && rp_publisher_) {
    rp_publisher_->publish(plan.toMsg(global_frame_, stamp));
  }
  return plan.toNavPath(global_frame_, stamp, params_.use_final_approach_orientation);
}

rcl_interfaces::msg::SetParametersResult
NavRouteGraphPlanner::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (const auto & p : parameters) {
    const auto & name = p.get_name();
    const auto type = p.get_type();
    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == name_ + ".projection_node_threshold_m") {
        params_.projection_node_threshold_m = p.as_double();
      } else if (name == name_ + ".projection_edge_threshold_m") {
        params_.projection_edge_threshold_m = p.as_double();
      } else if (name == name_ + ".w_distance_cost") {
        params_.weights.w_distance = p.as_double();
      } else if (name == name_ + ".w_floor_penalty") {
        params_.weights.w_floor_penalty = p.as_double();
      } else if (name == name_ + ".w_door_penalty") {
        params_.weights.w_door_penalty = p.as_double();
      } else if (name == name_ + ".w_elevator_penalty") {
        params_.weights.w_elevator_penalty = p.as_double();
      } else if (name == name_ + ".w_stair_factor") {
        params_.weights.w_stair_factor = p.as_double();
      } else if (name == name_ + ".w_user_pref_mult") {
        params_.weights.w_user_pref_mult = p.as_double();
      } else if (name == name_ + ".data_load_timeout_sec") {
        params_.data_load_timeout_sec = p.as_double();
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == name_ + ".waypoints_per_meter") {
        params_.waypoints_per_meter = p.as_int();
      } else if (name == name_ + ".bezier_samples_per_edge") {
        params_.bezier_samples_per_edge = p.as_int();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == name_ + ".prefer_no_stairs") {
        params_.prefs.prefer_no_stairs = p.as_bool();
      } else if (name == name_ + ".prefer_no_elevator") {
        params_.prefs.prefer_no_elevator = p.as_bool();
      } else if (name == name_ + ".publish_route_plan") {
        params_.publish_route_plan = p.as_bool();
      } else if (name == name_ + ".use_final_approach_orientation") {
        params_.use_final_approach_orientation = p.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_route_planner

PLUGINLIB_EXPORT_CLASS(
  nav2_route_planner::NavRouteGraphPlanner,
  nav2_core::GlobalPlanner)
