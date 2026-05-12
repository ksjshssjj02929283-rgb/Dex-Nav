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
  declare_parameter_if_not_declared(node, name_ + ".costmap_aware_mode",
    rclcpp::ParameterValue(params_.ca.costmap_aware_mode));
  declare_parameter_if_not_declared(node, name_ + ".obstacle_cost_threshold",
    rclcpp::ParameterValue(params_.ca.obstacle_cost_threshold));
  declare_parameter_if_not_declared(node, name_ + ".obstacle_margin_before_m",
    rclcpp::ParameterValue(params_.ca.obstacle_margin_before_m));
  declare_parameter_if_not_declared(node, name_ + ".obstacle_margin_after_m",
    rclcpp::ParameterValue(params_.ca.obstacle_margin_after_m));
  declare_parameter_if_not_declared(node, name_ + ".bypass_max_replan_length_m",
    rclcpp::ParameterValue(params_.ca.bypass_max_replan_length_m));
  declare_parameter_if_not_declared(node, name_ + ".theta_star_max_corners",
    rclcpp::ParameterValue(params_.ca.theta_star_max_corners));
  declare_parameter_if_not_declared(node, name_ + ".theta_star_allow_unknown",
    rclcpp::ParameterValue(params_.ca.theta_star_allow_unknown));

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
  node->get_parameter(name_ + ".costmap_aware_mode", params_.ca.costmap_aware_mode);
  node->get_parameter(name_ + ".obstacle_cost_threshold", params_.ca.obstacle_cost_threshold);
  node->get_parameter(name_ + ".obstacle_margin_before_m", params_.ca.obstacle_margin_before_m);
  node->get_parameter(name_ + ".obstacle_margin_after_m", params_.ca.obstacle_margin_after_m);
  node->get_parameter(name_ + ".bypass_max_replan_length_m", params_.ca.bypass_max_replan_length_m);
  node->get_parameter(name_ + ".theta_star_max_corners", params_.ca.theta_star_max_corners);
  node->get_parameter(name_ + ".theta_star_allow_unknown", params_.ca.theta_star_allow_unknown);

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

  ca_layer_ = std::make_shared<CostmapAwareLayer>(parent_node_, costmap_ros_);
  ca_layer_->configure(params_.ca);

  RCLCPP_INFO(logger_,
    "nav2_route_planner '%s' configured. asset_node=%s "
    "node_threshold=%.2fm edge_threshold=%.2fm "
    "costmap_aware_mode=%s",
    name_.c_str(), params_.asset_manager_node_name.c_str(),
    params_.projection_node_threshold_m, params_.projection_edge_threshold_m,
    params_.ca.costmap_aware_mode ? "ON" : "OFF");
}

void NavRouteGraphPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up nav2_route_planner '%s'", name_.c_str());
  if (ca_layer_) {ca_layer_->cleanup();}
  ca_layer_.reset();
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

  // Step 2.5 (start transition) and Step 5.5 (goal alignment) are handled
  // after Step 4 below, using Theta* for both — see "Step 4.5" and
  // "Step 4.6" below. The old static-line lethal check has been removed:
  // Theta*'s own isUnsafeToPlan covers "start/goal in lethal" cases, and
  // Theta* additionally finds an obstacle-avoiding curve when the
  // straight line would have failed.

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

  // ── Step 4: interpolate (路网内部分) ──────────────────────────────
  auto plan = interpolator_->interpolate(start, goal, start_proj, *goal_id,
      result, *graph_);
  auto stamp = clock_->now();

  // ── Step 4.5: 起点过渡段用 Theta* 重新规划 ────────────────────────
  // Plan 的第一段（"__start_transition__"）是 interpolator 用直线生成
  // 的占位段。这里用 Theta* 替换，让起点 → 路网入口自动避障。
  // Theta* 失败 = 起点在 lethal 内或起点到入口不可达 → 拒绝整个 plan。
  if (ca_layer_ && !plan.segments.empty()) {
    auto & first_seg = plan.segments.front();
    if (first_seg.edge_id == "__start_transition__" &&
        first_seg.samples.size() >= 2)
    {
      const auto & seg_start = first_seg.samples.front();
      const auto & seg_end = first_seg.samples.back();
      auto theta_path = ca_layer_->planWithThetaStar(seg_start, seg_end);
      if (!theta_path) {
        RCLCPP_ERROR(logger_,
          "Theta* failed for start transition (%.2f, %.2f) -> (%.2f, %.2f) — "
          "start unreachable (in obstacle / out of bounds / no path)",
          seg_start.pose.position.x, seg_start.pose.position.y,
          seg_end.pose.position.x, seg_end.pose.position.y);
        return empty_path;
      }
      first_seg.samples = std::move(*theta_path);
      first_seg.edge_id = "__start_transition_theta__";
    }
  }

  // ── Step 4.6: 终点对齐段用 Theta* 重新规划 ────────────────────────
  if (ca_layer_ && !plan.segments.empty()) {
    auto & last_seg = plan.segments.back();
    if (last_seg.edge_id == "__goal_alignment__" &&
        last_seg.samples.size() >= 2)
    {
      const auto & seg_start = last_seg.samples.front();
      const auto & seg_end = last_seg.samples.back();
      auto theta_path = ca_layer_->planWithThetaStar(seg_start, seg_end);
      if (!theta_path) {
        RCLCPP_ERROR(logger_,
          "Theta* failed for goal alignment (%.2f, %.2f) -> (%.2f, %.2f) — "
          "goal unreachable (in obstacle / out of bounds / no path)",
          seg_start.pose.position.x, seg_start.pose.position.y,
          seg_end.pose.position.x, seg_end.pose.position.y);
        return empty_path;
      }
      last_seg.samples = std::move(*theta_path);
      last_seg.edge_id = "__goal_alignment_theta__";
    }
  }

  // ── Step 5: convert to nav_msgs/Path + RoutePlan ──────────────────
  auto nav_path = plan.toNavPath(global_frame_, stamp,
      params_.use_final_approach_orientation);

  // ── Step 5.5: Layer 2 — 路网内 Theta* 局部绕障（opt-in） ──────────
  // 路网内的边段如果被运行时障碍打断，Theta* 在该区段做局部绕障。
  // 起点过渡和终点对齐段在 4.5/4.6 已 Theta* 化，所以本步骤聚焦
  // 中间路网边。RoutePlan 段语义不变。
  if (ca_layer_ && ca_layer_->isLayer2Enabled() && nav_path.poses.size() > 2) {
    auto obstacle = ca_layer_->detectObstacle(nav_path.poses);
    if (obstacle.has_obstacle) {
      const auto & seg_start = nav_path.poses[obstacle.start_index];
      const auto & seg_end = nav_path.poses[obstacle.end_index];
      RCLCPP_INFO(logger_,
        "Layer 2: obstacle on path [%zu..%zu] of %zu; attempting bypass "
        "from (%.2f,%.2f) to (%.2f,%.2f)",
        obstacle.start_index, obstacle.end_index, nav_path.poses.size(),
        seg_start.pose.position.x, seg_start.pose.position.y,
        seg_end.pose.position.x, seg_end.pose.position.y);
      auto bypass = ca_layer_->planWithThetaStar(seg_start, seg_end);
      if (bypass && !bypass->empty()) {
        std::vector<geometry_msgs::msg::PoseStamped> spliced;
        spliced.reserve(nav_path.poses.size() + bypass->size());
        spliced.insert(spliced.end(),
          nav_path.poses.begin(),
          nav_path.poses.begin() + obstacle.start_index);
        for (auto & p : *bypass) {
          p.header.frame_id = global_frame_;
          p.header.stamp = stamp;
          spliced.push_back(p);
        }
        spliced.insert(spliced.end(),
          nav_path.poses.begin() + obstacle.end_index + 1,
          nav_path.poses.end());
        const auto old_size = nav_path.poses.size();
        nav_path.poses = std::move(spliced);
        RCLCPP_INFO(logger_,
          "Layer 2: bypass spliced (path size %zu -> %zu, +%zu bypass samples)",
          old_size, nav_path.poses.size(), bypass->size());
      } else {
        RCLCPP_WARN(logger_,
          "Layer 2: bypass failed; returning original path (controller will handle)");
      }
    }
  }

  // ── Step 6: publish RoutePlan (graph-level semantics) + return Path ─
  if (params_.publish_route_plan && rp_publisher_) {
    rp_publisher_->publish(plan.toMsg(global_frame_, stamp));
  }
  return nav_path;
}

rcl_interfaces::msg::SetParametersResult
NavRouteGraphPlanner::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  bool ca_dirty = false;
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
      } else if (name == name_ + ".obstacle_cost_threshold") {
        params_.ca.obstacle_cost_threshold = p.as_double();
        ca_dirty = true;
      } else if (name == name_ + ".obstacle_margin_before_m") {
        params_.ca.obstacle_margin_before_m = p.as_double();
        ca_dirty = true;
      } else if (name == name_ + ".obstacle_margin_after_m") {
        params_.ca.obstacle_margin_after_m = p.as_double();
        ca_dirty = true;
      } else if (name == name_ + ".bypass_max_replan_length_m") {
        params_.ca.bypass_max_replan_length_m = p.as_double();
        ca_dirty = true;
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == name_ + ".waypoints_per_meter") {
        params_.waypoints_per_meter = p.as_int();
      } else if (name == name_ + ".bezier_samples_per_edge") {
        params_.bezier_samples_per_edge = p.as_int();
      } else if (name == name_ + ".theta_star_max_corners") {
        params_.ca.theta_star_max_corners = p.as_int();
        ca_dirty = true;
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
      } else if (name == name_ + ".costmap_aware_mode") {
        const bool old_v = params_.ca.costmap_aware_mode;
        params_.ca.costmap_aware_mode = p.as_bool();
        ca_dirty = true;
        RCLCPP_INFO(logger_,
          "costmap_aware_mode: %s -> %s",
          old_v ? "true" : "false",
          params_.ca.costmap_aware_mode ? "true" : "false");
      } else if (name == name_ + ".theta_star_allow_unknown") {
        params_.ca.theta_star_allow_unknown = p.as_bool();
        ca_dirty = true;
      }
    }
  }
  if (ca_dirty && ca_layer_) {
    ca_layer_->updateParams(params_.ca);
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_route_planner

PLUGINLIB_EXPORT_CLASS(
  nav2_route_planner::NavRouteGraphPlanner,
  nav2_core::GlobalPlanner)
