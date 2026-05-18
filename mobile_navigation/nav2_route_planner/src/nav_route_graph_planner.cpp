// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include "nav2_route_planner/nav_route_graph_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
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
  // Project 19 / DEC-004: thresholds become diagnostic-only — the
  // projector always returns the globally nearest NODE / EDGE.
  // DEC-005: shortcut threshold for "same-side" start/goal pairs.
  declare_parameter_if_not_declared(node, name_ + ".shortcut_max_proj_distance_m",
    rclcpp::ParameterValue(params_.shortcut_max_proj_distance_m));

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
  node->get_parameter(name_ + ".shortcut_max_proj_distance_m", params_.shortcut_max_proj_distance_m);

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

  // Debug marker publisher — visualizes each detected obstacle's
  // start_id / end_id position so the user can verify detection
  // before trusting bypass behavior. Transient_local QoS so a late
  // RViz subscription still receives the most recent set.
  if (node) {
    rclcpp::QoS marker_qos(1);
    marker_qos.transient_local();
    obstacle_markers_pub_ = node->create_publisher<
      visualization_msgs::msg::MarkerArray>(
      "~/" + name_ + "/obstacle_segments", marker_qos);
  }

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
  obstacle_markers_pub_.reset();
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
  if (obstacle_markers_pub_) {obstacle_markers_pub_->on_activate();}

  // Non-blocking background load. Sync waits inside activate() would
  // deadlock the SingleThreadedExecutor on the service response — see
  // RouteGraphLoader docs. createPlan() will simply return an empty path
  // if the load hasn't completed yet, and the caller can retry.
  if (loader_) {loader_->startBackgroundLoad();}

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
  if (loader_) {loader_->stopBackgroundLoad();}
  if (rp_publisher_) {rp_publisher_->deactivate();}
  if (obstacle_markers_pub_) {obstacle_markers_pub_->on_deactivate();}
  dyn_params_handler_.reset();
}

nav_msgs::msg::Path NavRouteGraphPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path empty_path;
  empty_path.header.frame_id = global_frame_;
  empty_path.header.stamp = clock_->now();

  // Background loader is fully async — a sync wait here would deadlock the
  // SingleThreadedExecutor (the executor thread is the one running this
  // callback AND the one that would deliver the service response). If the
  // graph isn't ready yet, return an empty path; the BT/caller retries.
  if (!graph_->isReady()) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
      "RouteGraph not ready yet (background load from %s still pending). "
      "Returning empty path; caller should retry.",
      params_.asset_manager_node_name.c_str());
    return empty_path;
  }

  auto lock = graph_->lockShared();

  // Project 19 / DEC-004 — unified pipeline (always 4 steps):
  //   1. Project start and goal onto the road graph (nearest NODE or
  //      EDGE projection — no thresholds; the user may click anywhere).
  //   2. Run A* between the two projections on the graph itself.
  //      ON_EDGE projections enter / exit via the nearer endpoint of
  //      the projected edge; the partial-edge cost is bookkeeping for
  //      the search, and the interpolator then walks the unwalked half
  //      of that edge to land exactly on `projected_xy`.
  //   3. Interpolator emits `__start_transition__` (start → projection)
  //      and `__goal_alignment__` (projection → goal) as free-space
  //      placeholders, plus `__start_edge_in__` / `__goal_edge_out__`
  //      for the projected-edge half walks.
  //   4. Theta* rewrites ONLY the two free-space placeholders. The
  //      graph segments (edges + edge_in/edge_out) keep their natural
  //      geometry — this is what "drive onto the road network" means.
  //
  // The legacy bounded `project()` / `projectToNode()` paths are kept
  // in the projector class for unit-test compatibility, but are not
  // used by the planner anymore.

  // ── Step 1: project start and goal ─────────────────────────────────
  // DEC-011: projectPreferEdge keeps the plan ON the road graph by
  // forcing ON_EDGE whenever any enabled edge exists — a NavPoint
  // that sits just off an edge would otherwise produce a plan that
  // briefly leaves the network.
  auto start_proj = projector_->projectPreferEdge(start, *graph_);
  if (start_proj.type == StartGoalProjector::ResultType::INVALID) {
    RCLCPP_ERROR(logger_,
      "Start pose (%.2f, %.2f) — graph has no nodes or edges, cannot plan",
      start.pose.position.x, start.pose.position.y);
    return empty_path;
  }
  if (start_proj.distance > params_.projection_node_threshold_m &&
      start_proj.distance > params_.projection_edge_threshold_m)
  {
    RCLCPP_INFO(logger_,
      "Start pose (%.2f, %.2f) is %.2fm from nearest graph element "
      "(beyond diagnostic thresholds %.2f/%.2fm) — bridging with Theta*",
      start.pose.position.x, start.pose.position.y, start_proj.distance,
      params_.projection_node_threshold_m,
      params_.projection_edge_threshold_m);
  }
  // DEC-014 diagnostic: dump the full projection state so we can see
  // exactly where projected_xy lands and how far along the edge it sits.
  {
    auto e_opt = graph_->getEdge(start_proj.id);
    const double edge_len = e_opt ? e_opt->length : 0.0;
    const double frac = (start_proj.edge_total_samples > 1)
      ? static_cast<double>(start_proj.edge_sample_index) /
        static_cast<double>(start_proj.edge_total_samples - 1)
      : 0.0;
    RCLCPP_INFO(logger_,
      "  start_proj: type=%s id=%s start=(%.3f,%.3f) projected_xy=(%.3f,%.3f) "
      "dist_to_proj=%.3fm  edge_idx=%zu/%zu  t=%.3f  edge_len=%.2fm  "
      "walked_from_from=%.2fm walked_from_to=%.2fm",
      start_proj.type == StartGoalProjector::ResultType::ON_EDGE ? "ON_EDGE"
        : (start_proj.type == StartGoalProjector::ResultType::ON_NODE ? "ON_NODE" : "INVALID"),
      start_proj.id.c_str(),
      start.pose.position.x, start.pose.position.y,
      start_proj.projected_xy.first, start_proj.projected_xy.second,
      start_proj.distance,
      start_proj.edge_sample_index,
      (start_proj.edge_total_samples > 0 ? start_proj.edge_total_samples - 1 : 0u),
      frac, edge_len,
      edge_len * frac, edge_len * (1.0 - frac));
  }

  auto goal_proj = projector_->projectPreferEdge(goal, *graph_);
  if (goal_proj.type == StartGoalProjector::ResultType::INVALID) {
    RCLCPP_ERROR(logger_,
      "Goal pose (%.2f, %.2f) — graph has no nodes or edges, cannot plan",
      goal.pose.position.x, goal.pose.position.y);
    return empty_path;
  }
  if (goal_proj.distance > params_.projection_node_threshold_m &&
      goal_proj.distance > params_.projection_edge_threshold_m)
  {
    RCLCPP_INFO(logger_,
      "Goal pose (%.2f, %.2f) is %.2fm from nearest graph element "
      "(beyond diagnostic thresholds) — bridging with Theta*",
      goal.pose.position.x, goal.pose.position.y, goal_proj.distance);
  }
  // DEC-014 diagnostic: same dump for goal.
  {
    auto e_opt = graph_->getEdge(goal_proj.id);
    const double edge_len = e_opt ? e_opt->length : 0.0;
    const double frac = (goal_proj.edge_total_samples > 1)
      ? static_cast<double>(goal_proj.edge_sample_index) /
        static_cast<double>(goal_proj.edge_total_samples - 1)
      : 0.0;
    std::string from_id_s = e_opt ? e_opt->from_id : std::string("?");
    std::string to_id_s = e_opt ? e_opt->to_id : std::string("?");
    RCLCPP_INFO(logger_,
      "  goal_proj : type=%s id=%s (from=%s to=%s) goal=(%.3f,%.3f) "
      "projected_xy=(%.3f,%.3f) dist_to_proj=%.3fm  edge_idx=%zu/%zu  t=%.3f  "
      "edge_len=%.2fm  walked_from_from=%.2fm walked_from_to=%.2fm",
      goal_proj.type == StartGoalProjector::ResultType::ON_EDGE ? "ON_EDGE"
        : (goal_proj.type == StartGoalProjector::ResultType::ON_NODE ? "ON_NODE" : "INVALID"),
      goal_proj.id.c_str(), from_id_s.c_str(), to_id_s.c_str(),
      goal.pose.position.x, goal.pose.position.y,
      goal_proj.projected_xy.first, goal_proj.projected_xy.second,
      goal_proj.distance,
      goal_proj.edge_sample_index,
      (goal_proj.edge_total_samples > 0 ? goal_proj.edge_total_samples - 1 : 0u),
      frac, edge_len,
      edge_len * frac, edge_len * (1.0 - frac));
  }

  // ── Same-side shortcut (DEC-005) ───────────────────────────────────
  //
  // If the start and goal both enter the road network at essentially
  // the SAME point, graph routing would force the robot through a
  // remote NavPoint and back — a visible U-turn / detour. Detect this
  // case (projection points within `shortcut_max_proj_distance_m` of
  // each other) and just Theta\* directly. Skipped when the threshold
  // is 0.0 or when Theta\* fails (we fall through to graph routing).
  if (params_.shortcut_max_proj_distance_m > 0.0) {
    auto projXY =
      [&](const StartGoalProjector::Result & pr) -> std::pair<double, double> {
        if (pr.type == StartGoalProjector::ResultType::ON_NODE) {
          auto n_opt = graph_->getNode(pr.id);
          if (n_opt) {return {n_opt->x, n_opt->y};}
        }
        return pr.projected_xy;
      };
    const auto s_xy = projXY(start_proj);
    const auto g_xy = projXY(goal_proj);
    const double proj_dist = std::hypot(
      s_xy.first - g_xy.first, s_xy.second - g_xy.second);
    if (proj_dist < params_.shortcut_max_proj_distance_m) {
      auto direct = ca_layer_->planWithThetaStar(start, goal);
      if (direct && !direct->empty()) {
        RCLCPP_INFO(logger_,
          "Same-side shortcut: start_proj↔goal_proj=%.2fm < %.2fm — "
          "skipping graph; Theta* direct plan with %zu samples",
          proj_dist, params_.shortcut_max_proj_distance_m, direct->size());
        nav_msgs::msg::Path out;
        out.header.frame_id = global_frame_;
        out.header.stamp = clock_->now();
        out.poses.reserve(direct->size());
        for (auto & p : *direct) {
          p.header.frame_id = global_frame_;
          p.header.stamp = out.header.stamp;
          out.poses.push_back(std::move(p));
        }
        if (!out.poses.empty()) {
          if (params_.use_final_approach_orientation &&
              out.poses.size() >= 2)
          {
            const auto & pl = out.poses.back().pose.position;
            const auto & pp = out.poses[out.poses.size() - 2].pose.position;
            const double yaw = std::atan2(pl.y - pp.y, pl.x - pp.x);
            auto & q = out.poses.back().pose.orientation;
            q.x = 0.0; q.y = 0.0;
            q.z = std::sin(yaw * 0.5);
            q.w = std::cos(yaw * 0.5);
          } else {
            out.poses.back().pose.orientation = goal.pose.orientation;
          }
        }
        return out;
      }
      RCLCPP_WARN(logger_,
        "Same-side shortcut: proj distance %.2fm < threshold but Theta* "
        "direct failed; falling through to graph routing",
        proj_dist);
    }
  }

  // A* needs a goal NODE — for ON_EDGE goals we pick the closer endpoint
  // of the projected edge. The interpolator's `__goal_edge_out__`
  // segment then walks the rest of that edge to land at `projected_xy`.
  // DEC-009 (2026-05-18): inspired by the legacy ROS1 waypoint_global_
  // planner — when the goal projects onto an edge interior, the plan
  // is structured as 5 stages: (1) start projection, (2) goal
  // projection, (3) graph A* to "the previous waypoint on the goal
  // edge" (the endpoint A* naturally enters from), (4) walk the goal
  // edge from that previous waypoint to projected_xy, (5) Theta* from
  // projected_xy to goal pose.
  //
  // For stage (3) we try BOTH endpoints of goal_proj.id with the goal
  // edge BLACKLISTED so A* must reach the endpoint via an alternative
  // path (never via goal_proj.id itself). We pick whichever (endpoint,
  // A*-path) pair minimises (A*.total_cost + residual edge distance to
  // projected_xy). Without the blacklist, the natural shortest A*
  // path frequently arrives at the chosen endpoint by going _through_
  // goal_proj.id (i.e. NavPoint 0 -> e_20 -> NavPoint 2), so the plan
  // would include e_20 in the A* edge_ids AND again in __goal_edge_out__
  // — a visible U-turn. Blacklisting forces the search to find an
  // honest alternative.
  //
  // Fallback: if both endpoints are unreachable with the blacklist
  // (e.g. goal_proj.id is the only ingress), retry without it; the
  // interpolator's truncation safety net in path_interpolator.cpp
  // trims that edge at projected_xy.
  std::string goal_node_id;
  AStarSearcher::Result result;
  long dt = 0;
  {
    AStarSearcher::Request req;
    req.start = start_proj;
    req.weights = params_.weights;
    req.prefs = params_.prefs;
    if (auto node = parent_node_.lock()) {
      node->get_parameter(name_ + ".prefer_no_stairs", req.prefs.prefer_no_stairs);
      node->get_parameter(name_ + ".prefer_no_elevator", req.prefs.prefer_no_elevator);
    }

    if (goal_proj.type == StartGoalProjector::ResultType::ON_NODE) {
      // Simple case: A* directly to the goal node.
      req.goal_node_id = goal_proj.id;
      auto t0 = std::chrono::steady_clock::now();
      result = searcher_->search(req, *graph_);
      dt = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - t0).count();
      goal_node_id = goal_proj.id;
    } else {
      // ON_EDGE: try both endpoints with blacklist; keep the lower
      // (A*.total_cost + residual_to_projected_xy) candidate.
      auto e_opt = graph_->getEdge(goal_proj.id);
      if (!e_opt) {
        RCLCPP_ERROR(logger_,
          "Goal pose (%.2f, %.2f) projects onto edge '%s' but the edge "
          "is missing from the graph",
          goal.pose.position.x, goal.pose.position.y,
          goal_proj.id.c_str());
        return empty_path;
      }
      // Edge length walked when we leave from `endpoint` to reach
      // projected_xy (proportional to how far along the sample array
      // projected_xy sits relative to `endpoint`).
      const std::size_t orig_max = (goal_proj.edge_total_samples > 1)
        ? (goal_proj.edge_total_samples - 1) : 1u;
      const double frac_from = static_cast<double>(
        goal_proj.edge_sample_index) / static_cast<double>(orig_max);
      const double residual_from = e_opt->length * frac_from;
      const double residual_to = e_opt->length * (1.0 - frac_from);

      req.blacklisted_edge_ids.insert(goal_proj.id);
      auto try_endpoint =
        [&](const std::string & endpoint, double residual)
        -> std::pair<bool, AStarSearcher::Result>
        {
          AStarSearcher::Request r = req;
          r.goal_node_id = endpoint;
          auto t0 = std::chrono::steady_clock::now();
          auto out = searcher_->search(r, *graph_);
          dt += std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t0).count();
          (void)residual;
          return {out.success, out};
        };
      auto [ok_from, res_from] = try_endpoint(e_opt->from_id, residual_from);
      auto [ok_to, res_to] = try_endpoint(e_opt->to_id, residual_to);

      const double total_from = ok_from
        ? res_from.total_cost + residual_from
        : std::numeric_limits<double>::infinity();
      const double total_to = ok_to
        ? res_to.total_cost + residual_to
        : std::numeric_limits<double>::infinity();

      if (ok_from || ok_to) {
        if (total_from <= total_to) {
          result = std::move(res_from);
          goal_node_id = e_opt->from_id;
          RCLCPP_INFO(logger_,
            "DEC-009: chose 'from' endpoint='%s' (cost+residual=%.2f, "
            "alt 'to'=%s %.2f) — A* path avoids goal edge '%s'",
            e_opt->from_id.c_str(), total_from,
            ok_to ? "" : "(unreachable)", total_to, goal_proj.id.c_str());
        } else {
          result = std::move(res_to);
          goal_node_id = e_opt->to_id;
          RCLCPP_INFO(logger_,
            "DEC-009: chose 'to' endpoint='%s' (cost+residual=%.2f, "
            "alt 'from'=%s %.2f) — A* path avoids goal edge '%s'",
            e_opt->to_id.c_str(), total_to,
            ok_from ? "" : "(unreachable)", total_from,
            goal_proj.id.c_str());
        }
      } else {
        // Fallback: both blacklisted attempts failed (goal edge is the
        // sole approach). Drop the blacklist; the interpolator will
        // truncate the goal edge at projected_xy as a safety net.
        const std::size_t total = (goal_proj.edge_total_samples > 1)
          ? (goal_proj.edge_total_samples - 1) : 1u;
        const bool proj_near_from = (goal_proj.edge_sample_index * 2) < total;
        goal_node_id = proj_near_from ? e_opt->to_id : e_opt->from_id;
        AStarSearcher::Request r2 = req;
        r2.blacklisted_edge_ids.clear();
        r2.goal_node_id = goal_node_id;
        auto t0 = std::chrono::steady_clock::now();
        result = searcher_->search(r2, *graph_);
        dt += std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - t0).count();
        RCLCPP_WARN(logger_,
          "DEC-009: both endpoints unreachable with goal edge '%s' "
          "blacklisted; fell back to far endpoint='%s' WITHOUT "
          "blacklist — interpolator will truncate the goal edge",
          goal_proj.id.c_str(), goal_node_id.c_str());
      }
    }
  }

  if (!result.success) {
    RCLCPP_ERROR(logger_,
      "A* failed: %s (start=%s goal=%s expanded=%zu)",
      result.failure_reason.c_str(),
      start_proj.id.c_str(), goal_node_id.c_str(), result.expanded_nodes);
    return empty_path;
  }
  RCLCPP_INFO(logger_,
    "A* found path: %zu nodes, %zu edges, cost=%.2f, expanded=%zu, %ldus",
    result.node_ids.size(), result.edge_ids.size(),
    result.total_cost, result.expanded_nodes, dt);

  // ── Step 3: interpolate ────────────────────────────────────────────
  // DEC-014 diagnostic: dump edge_pts that interpolator will see for
  // goal_proj.id — verifies sampleEdge actually returns 2 samples for
  // LINE, and shows what the endpoints look like geometrically.
  if (goal_proj.type == StartGoalProjector::ResultType::ON_EDGE) {
    auto e_opt = graph_->getEdge(goal_proj.id);
    if (e_opt) {
      auto edge_pts = graph_->sampleEdge(*e_opt, params_.bezier_samples_per_edge);
      RCLCPP_INFO(logger_,
        "  goal_proj edge dump: id=%s geom=%s control_points=%zu "
        "sample_count=%zu",
        goal_proj.id.c_str(),
        e_opt->geometry == EdgeGeometry::LINE ? "LINE" : "BEZIER",
        e_opt->control_points.size(),
        edge_pts.size());
      const std::size_t show = std::min<std::size_t>(edge_pts.size(), 6u);
      for (std::size_t i = 0; i < show; ++i) {
        RCLCPP_INFO(logger_, "    edge_pts[%zu] = (%.3f, %.3f)",
          i, edge_pts[i].first, edge_pts[i].second);
      }
      if (edge_pts.size() > show) {
        RCLCPP_INFO(logger_, "    ... edge_pts[%zu] = (%.3f, %.3f)",
          edge_pts.size() - 1,
          edge_pts.back().first, edge_pts.back().second);
      }
    }
  }
  auto plan = interpolator_->interpolate(start, goal, start_proj, goal_proj,
      result, *graph_);
  auto stamp = clock_->now();
  // DEC-014 diagnostic: dump each interpolated segment so we can see
  // which one contributed the obstacle-bearing samples.
  {
    std::size_t cum = 0;
    RCLCPP_INFO(logger_, "  interpolate produced %zu segments:",
      plan.segments.size());
    for (std::size_t si = 0; si < plan.segments.size(); ++si) {
      const auto & s = plan.segments[si];
      double seg_len = 0.0;
      for (std::size_t k = 1; k < s.samples.size(); ++k) {
        seg_len += std::hypot(
          s.samples[k].pose.position.x - s.samples[k - 1].pose.position.x,
          s.samples[k].pose.position.y - s.samples[k - 1].pose.position.y);
      }
      const auto & sf = s.samples.empty() ? geometry_msgs::msg::PoseStamped() : s.samples.front();
      const auto & sb = s.samples.empty() ? geometry_msgs::msg::PoseStamped() : s.samples.back();
      RCLCPP_INFO(logger_,
        "    seg[%zu] '%s' samples=%zu  cum_idx=[%zu..%zu]  geom_len=%.2fm  "
        "first=(%.2f,%.2f)  last=(%.2f,%.2f)",
        si, s.edge_id.c_str(), s.samples.size(),
        cum, cum + (s.samples.empty() ? 0u : s.samples.size() - 1),
        seg_len,
        sf.pose.position.x, sf.pose.position.y,
        sb.pose.position.x, sb.pose.position.y);
      cum += s.samples.size();
    }
  }

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

  // ── Step 5.5: Layer 2 — Theta* obstacle bypass + obstacle markers ──
  //
  // Single obstacle-detection entry: detectAllObstacles returns ALL
  // contiguous lethal runs along nav_path, sorted ascending and non-
  // overlapping. The marker publisher always emits the detected
  // start_id / end_id positions for visual verification, regardless of
  // whether bypass is enabled or succeeds.
  //
  // Splice order is STRICTLY: A (path[0..start_id-1]) → Theta* (bypass)
  // → B (path[end_id+1..]). Each bypass is computed on the ORIGINAL
  // nav_path (not the spliced one), and applied in REVERSE order so
  // earlier indices stay valid across splices.
  if (ca_layer_ && nav_path.poses.size() > 2) {
    auto obstacles = ca_layer_->detectAllObstacles(nav_path.poses);

    // Always publish obstacle markers for visual verification.
    if (obstacle_markers_pub_) {
      visualization_msgs::msg::MarkerArray ma;
      // Clear previous round's markers.
      visualization_msgs::msg::Marker clr;
      clr.header.frame_id = global_frame_;
      clr.header.stamp = stamp;
      clr.ns = "obstacle_segments";
      clr.action = visualization_msgs::msg::Marker::DELETEALL;
      ma.markers.push_back(clr);
      int mid = 0;
      for (std::size_t k = 0; k < obstacles.size(); ++k) {
        const auto & ob = obstacles[k];
        // start_id — GREEN sphere + label
        visualization_msgs::msg::Marker s_sphere;
        s_sphere.header.frame_id = global_frame_;
        s_sphere.header.stamp = stamp;
        s_sphere.ns = "obstacle_segments";
        s_sphere.id = mid++;
        s_sphere.type = visualization_msgs::msg::Marker::SPHERE;
        s_sphere.action = visualization_msgs::msg::Marker::ADD;
        s_sphere.pose = nav_path.poses[ob.start_index].pose;
        s_sphere.scale.x = s_sphere.scale.y = s_sphere.scale.z = 0.18;
        s_sphere.color.r = 0.0f; s_sphere.color.g = 1.0f;
        s_sphere.color.b = 0.0f; s_sphere.color.a = 0.9f;
        ma.markers.push_back(s_sphere);
        visualization_msgs::msg::Marker s_text;
        s_text.header = s_sphere.header;
        s_text.ns = "obstacle_segments";
        s_text.id = mid++;
        s_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        s_text.action = visualization_msgs::msg::Marker::ADD;
        s_text.pose = s_sphere.pose;
        s_text.pose.position.z += 0.30;
        s_text.scale.z = 0.20;
        s_text.color.r = s_text.color.g = s_text.color.b = 1.0f;
        s_text.color.a = 1.0f;
        s_text.text = "obs#" + std::to_string(k) +
          " start_id=" + std::to_string(ob.start_index);
        ma.markers.push_back(s_text);
        // end_id — RED sphere + label
        visualization_msgs::msg::Marker e_sphere = s_sphere;
        e_sphere.id = mid++;
        e_sphere.pose = nav_path.poses[ob.end_index].pose;
        e_sphere.color.r = 1.0f; e_sphere.color.g = 0.0f;
        ma.markers.push_back(e_sphere);
        visualization_msgs::msg::Marker e_text = s_text;
        e_text.id = mid++;
        e_text.pose = e_sphere.pose;
        e_text.pose.position.z += 0.30;
        e_text.text = "obs#" + std::to_string(k) +
          " end_id=" + std::to_string(ob.end_index);
        ma.markers.push_back(e_text);
        // Orange LINE_STRIP spanning [start_id..end_id] on the path
        visualization_msgs::msg::Marker line;
        line.header = s_sphere.header;
        line.ns = "obstacle_segments";
        line.id = mid++;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.05;
        line.color.r = 1.0f; line.color.g = 0.5f;
        line.color.b = 0.0f; line.color.a = 0.8f;
        for (std::size_t i = ob.start_index; i <= ob.end_index; ++i) {
          line.points.push_back(nav_path.poses[i].pose.position);
        }
        ma.markers.push_back(line);
      }
      obstacle_markers_pub_->publish(ma);
      if (!obstacles.empty()) {
        RCLCPP_INFO(logger_,
          "Layer 2: published %zu obstacle marker(s) on "
          "'~/%s/obstacle_segments'",
          obstacles.size(), name_.c_str());
      }
    }

    // Apply Theta* bypass only when costmap-aware mode is on.
    if (ca_layer_->isLayer2Enabled() && !obstacles.empty()) {
      RCLCPP_INFO(logger_,
        "Layer 2: detected %zu obstacle run(s) on path of %zu samples",
        obstacles.size(), nav_path.poses.size());

      // Process in REVERSE order so earlier obstacles' indices remain
      // valid after splicing later ones.
      std::size_t bypass_count = 0;
      for (auto it = obstacles.rbegin(); it != obstacles.rend(); ++it) {
        const auto & ob = *it;
        if (ob.end_index >= nav_path.poses.size()) {continue;}
        const auto & seg_start = nav_path.poses[ob.start_index];
        const auto & seg_end = nav_path.poses[ob.end_index];

        auto bypass = ca_layer_->planWithThetaStar(seg_start, seg_end);
        if (!bypass || bypass->empty()) {
          RCLCPP_WARN(logger_,
            "Layer 2: bypass failed for obstacle [%zu..%zu] "
            "(%.2f,%.2f)->(%.2f,%.2f) — keeping original path here",
            ob.start_index, ob.end_index,
            seg_start.pose.position.x, seg_start.pose.position.y,
            seg_end.pose.position.x, seg_end.pose.position.y);
          continue;
        }

        // ── Splice order: A → Theta* → B (strictly, in this order) ───
        // A = nav_path[0 .. start_id - 1]   (everything before obstacle)
        // Theta* = bypass dense samples (start→goal order, terminating
        //          at seg_end's geometry)
        // B = nav_path[end_id + 1 .. last]  (everything after obstacle)
        std::vector<geometry_msgs::msg::PoseStamped> spliced;
        spliced.reserve(nav_path.poses.size() + bypass->size());
        // A:
        spliced.insert(spliced.end(),
          nav_path.poses.begin(),
          nav_path.poses.begin() + ob.start_index);
        // Theta*:
        for (auto & p : *bypass) {
          p.header.frame_id = global_frame_;
          p.header.stamp = stamp;
          spliced.push_back(p);
        }
        // B:
        spliced.insert(spliced.end(),
          nav_path.poses.begin() + ob.end_index + 1,
          nav_path.poses.end());

        RCLCPP_INFO(logger_,
          "Layer 2: spliced bypass for obstacle [%zu..%zu]: "
          "|A|=%zu + |Theta*|=%zu + |B|=%zu = %zu",
          ob.start_index, ob.end_index,
          ob.start_index,
          bypass->size(),
          nav_path.poses.size() - (ob.end_index + 1),
          spliced.size());

        nav_path.poses = std::move(spliced);
        ++bypass_count;
      }
      RCLCPP_INFO(logger_,
        "Layer 2: spliced %zu/%zu bypasses (final path size = %zu)",
        bypass_count, obstacles.size(), nav_path.poses.size());
    }
  }

  // ── Step 5.7: 最终朝向兜底 ─────────────────────────────────────────
  // Theta* 重写过的段（Step 4.6 / Step 5.5）输出的 sample orientation 都是
  // identity quaternion (yaw=0)。Interpolator 在 Step 4 已把最后 sample
  // 设为 goal.pose.orientation，但若该 sample 被后续 Theta* 段覆盖（典型
  // 场景：goal 落在 NavPoint 的 projection_node_threshold_m 范围内，触发
  // __goal_alignment__ → Theta* 重写），就会丢失用户指定的终点角度，
  // 表现为"机器人到达后朝向 +X 轴而非给定方向"。
  // 这里在所有路径变换完成后做最终修正：use_final_approach_orientation
  // 关闭时强制写入用户给的 goal.orientation；开启时按路径切线方向计算。
  if (!nav_path.poses.empty()) {
    if (params_.use_final_approach_orientation && nav_path.poses.size() >= 2) {
      const auto & p_last = nav_path.poses.back().pose.position;
      const auto & p_prev = nav_path.poses[nav_path.poses.size() - 2].pose.position;
      const double yaw = std::atan2(p_last.y - p_prev.y, p_last.x - p_prev.x);
      auto & q = nav_path.poses.back().pose.orientation;
      q.x = 0.0;
      q.y = 0.0;
      q.z = std::sin(yaw * 0.5);
      q.w = std::cos(yaw * 0.5);
    } else {
      nav_path.poses.back().pose.orientation = goal.pose.orientation;
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
      } else if (name == name_ + ".shortcut_max_proj_distance_m") {
        params_.shortcut_max_proj_distance_m = p.as_double();
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
