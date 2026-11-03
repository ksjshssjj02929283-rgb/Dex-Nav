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

#include <functional>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

using rcl_interfaces::msg::ParameterType;

namespace nav2_route_planner
{

namespace
{

/// Single source of truth for the planner's declared parameters.
///
/// Each entry binds:
///   • `key`            — yaml leaf name (full name = name_ + "." + key)
///   • `default_value`  — read from a freshly-constructed `Params`
///   • `apply`          — write back from an rclcpp::Parameter
///   • `triggers_ca`    — whether dynamic updates should re-push to
///                        CostmapAwareLayer (avoids a separate flag)
///
/// Adding / removing parameters now means a single line edit; the
/// configure-time `declare_parameter_if_not_declared`, the
/// configure-time `get_parameter`, and the dynamic callback all read
/// from the same table.
struct ParamHandler
{
  std::string key;
  std::function<rclcpp::ParameterValue(const NavRouteGraphPlanner::Params &)>
    default_value;
  std::function<void(NavRouteGraphPlanner::Params &, const rclcpp::Parameter &)>
    apply;
  bool triggers_ca{false};
};

using P = NavRouteGraphPlanner::Params;

const std::vector<ParamHandler> & paramTable()
{
  static const std::vector<ParamHandler> table = {
    // ── Data source ──────────────────────────────────────────────────
    {"asset_manager_node_name",
      [](const P & p) {return rclcpp::ParameterValue(p.asset_manager_node_name);},
      [](P & p, const rclcpp::Parameter & v) {p.asset_manager_node_name = v.as_string();},
      false},
    {"data_load_timeout_sec",
      [](const P & p) {return rclcpp::ParameterValue(p.data_load_timeout_sec);},
      [](P & p, const rclcpp::Parameter & v) {p.data_load_timeout_sec = v.as_double();},
      false},

    // ── Projection (diagnostic thresholds) ──────────────────────────
    {"projection_node_threshold_m",
      [](const P & p) {return rclcpp::ParameterValue(p.projection_node_threshold_m);},
      [](P & p, const rclcpp::Parameter & v) {p.projection_node_threshold_m = v.as_double();},
      false},
    {"projection_edge_threshold_m",
      [](const P & p) {return rclcpp::ParameterValue(p.projection_edge_threshold_m);},
      [](P & p, const rclcpp::Parameter & v) {p.projection_edge_threshold_m = v.as_double();},
      false},
    {"node_snap_threshold_m",
      [](const P & p) {return rclcpp::ParameterValue(p.node_snap_threshold_m);},
      [](P & p, const rclcpp::Parameter & v) {p.node_snap_threshold_m = v.as_double();},
      false},

    // ── Same-side shortcut ──────────────────────────────────────────
    {"enable_same_side_shortcut",
      [](const P & p) {return rclcpp::ParameterValue(p.enable_same_side_shortcut);},
      [](P & p, const rclcpp::Parameter & v) {p.enable_same_side_shortcut = v.as_bool();},
      false},
    {"shortcut_max_proj_distance_m",
      [](const P & p) {return rclcpp::ParameterValue(p.shortcut_max_proj_distance_m);},
      [](P & p, const rclcpp::Parameter & v) {p.shortcut_max_proj_distance_m = v.as_double();},
      false},

    // ── A* cost weights ─────────────────────────────────────────────
    {"w_distance_cost",
      [](const P & p) {return rclcpp::ParameterValue(p.weights.w_distance);},
      [](P & p, const rclcpp::Parameter & v) {p.weights.w_distance = v.as_double();},
      false},
    {"w_floor_penalty",
      [](const P & p) {return rclcpp::ParameterValue(p.weights.w_floor_penalty);},
      [](P & p, const rclcpp::Parameter & v) {p.weights.w_floor_penalty = v.as_double();},
      false},
    {"w_door_penalty",
      [](const P & p) {return rclcpp::ParameterValue(p.weights.w_door_penalty);},
      [](P & p, const rclcpp::Parameter & v) {p.weights.w_door_penalty = v.as_double();},
      false},
    {"w_elevator_penalty",
      [](const P & p) {return rclcpp::ParameterValue(p.weights.w_elevator_penalty);},
      [](P & p, const rclcpp::Parameter & v) {p.weights.w_elevator_penalty = v.as_double();},
      false},
    {"w_stair_factor",
      [](const P & p) {return rclcpp::ParameterValue(p.weights.w_stair_factor);},
      [](P & p, const rclcpp::Parameter & v) {p.weights.w_stair_factor = v.as_double();},
      false},
    {"w_user_pref_mult",
      [](const P & p) {return rclcpp::ParameterValue(p.weights.w_user_pref_mult);},
      [](P & p, const rclcpp::Parameter & v) {p.weights.w_user_pref_mult = v.as_double();},
      false},

    // ── User preferences ────────────────────────────────────────────
    {"prefer_no_stairs",
      [](const P & p) {return rclcpp::ParameterValue(p.prefs.prefer_no_stairs);},
      [](P & p, const rclcpp::Parameter & v) {p.prefs.prefer_no_stairs = v.as_bool();},
      false},
    {"prefer_no_elevator",
      [](const P & p) {return rclcpp::ParameterValue(p.prefs.prefer_no_elevator);},
      [](P & p, const rclcpp::Parameter & v) {p.prefs.prefer_no_elevator = v.as_bool();},
      false},

    // ── Interpolation ───────────────────────────────────────────────
    {"waypoints_per_meter",
      [](const P & p) {return rclcpp::ParameterValue(p.waypoints_per_meter);},
      [](P & p, const rclcpp::Parameter & v) {p.waypoints_per_meter = v.as_int();},
      false},
    {"bezier_samples_per_edge",
      [](const P & p) {return rclcpp::ParameterValue(p.bezier_samples_per_edge);},
      [](P & p, const rclcpp::Parameter & v) {p.bezier_samples_per_edge = v.as_int();},
      false},

    // ── Output ──────────────────────────────────────────────────────
    {"publish_route_plan",
      [](const P & p) {return rclcpp::ParameterValue(p.publish_route_plan);},
      [](P & p, const rclcpp::Parameter & v) {p.publish_route_plan = v.as_bool();},
      false},
    {"use_final_approach_orientation",
      [](const P & p) {return rclcpp::ParameterValue(p.use_final_approach_orientation);},
      [](P & p, const rclcpp::Parameter & v) {p.use_final_approach_orientation = v.as_bool();},
      false},

    // ── Route costmap avoidance (triggers_ca=true → push to ca_layer) ─
    {"enable_route_costmap_avoidance",
      [](const P & p) {
        return rclcpp::ParameterValue(p.ca.enable_route_costmap_avoidance);
      },
      [](P & p, const rclcpp::Parameter & v) {
        p.ca.enable_route_costmap_avoidance = v.as_bool();
      },
      true},
    {"obstacle_cost_threshold",
      [](const P & p) {return rclcpp::ParameterValue(p.ca.obstacle_cost_threshold);},
      [](P & p, const rclcpp::Parameter & v) {p.ca.obstacle_cost_threshold = v.as_double();},
      true},
    {"obstacle_margin_before_m",
      [](const P & p) {return rclcpp::ParameterValue(p.ca.obstacle_margin_before_m);},
      [](P & p, const rclcpp::Parameter & v) {p.ca.obstacle_margin_before_m = v.as_double();},
      true},
    {"obstacle_margin_after_m",
      [](const P & p) {return rclcpp::ParameterValue(p.ca.obstacle_margin_after_m);},
      [](P & p, const rclcpp::Parameter & v) {p.ca.obstacle_margin_after_m = v.as_double();},
      true},
    {"bypass_max_replan_length_m",
      [](const P & p) {return rclcpp::ParameterValue(p.ca.bypass_max_replan_length_m);},
      [](P & p, const rclcpp::Parameter & v) {p.ca.bypass_max_replan_length_m = v.as_double();},
      true},
    {"theta_star_max_corners",
      [](const P & p) {return rclcpp::ParameterValue(p.ca.theta_star_max_corners);},
      [](P & p, const rclcpp::Parameter & v) {p.ca.theta_star_max_corners = v.as_int();},
      true},
    {"theta_star_allow_unknown",
      [](const P & p) {return rclcpp::ParameterValue(p.ca.theta_star_allow_unknown);},
      [](P & p, const rclcpp::Parameter & v) {p.ca.theta_star_allow_unknown = v.as_bool();},
      true},
  };
  return table;
}

}  // namespace

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

  // ── Declare + load parameters via table-driven registry ───────────
  // All ~24 parameters defined in `paramTable()` at the top of this
  // file — one entry per parameter, declare/get/dynamic-update share
  // it. Add or remove a parameter: edit one table row.
  for (const auto & h : paramTable()) {
    const std::string full_name = name_ + "." + h.key;
    nav2_util::declare_parameter_if_not_declared(
      node, full_name, h.default_value(params_));
    rclcpp::Parameter p;
    if (node->get_parameter(full_name, p)) {
      h.apply(params_, p);
    }
  }

  // ── Construct components ───────────────────────────────────────────
  graph_ = std::make_shared<TopoGraph>();
  loader_ = std::make_shared<RouteGraphLoader>(parent_node_,
      params_.asset_manager_node_name, graph_);
  StartGoalProjector::Params proj_p;
  proj_p.node_threshold_m = params_.projection_node_threshold_m;
  proj_p.edge_threshold_m = params_.projection_edge_threshold_m;
  proj_p.bezier_samples_per_edge = params_.bezier_samples_per_edge;
  proj_p.node_snap_threshold_m = params_.node_snap_threshold_m;
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
    "enable_route_costmap_avoidance=%s",
    name_.c_str(), params_.asset_manager_node_name.c_str(),
    params_.projection_node_threshold_m, params_.projection_edge_threshold_m,
    params_.ca.enable_route_costmap_avoidance ? "ON" : "OFF");
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
  // Plan pipeline (each stage is a named helper below; createPlan stays
  // a readable orchestrator):
  //   1. project start and goal onto the road graph
  //   2. obstacle-aware projection adjustment (slide a projection that
  //      landed inside a lethal-cost run past the obstacle + margin)
  //   3. same-side shortcut (Theta* direct when both projections sit
  //      within shortcut_max_proj_distance_m of each other)
  //   4. augment the graph with a virtual node per ON_EDGE projection,
  //      then plain node-to-node A* (one uniform path — the augmentation
  //      absorbs the edge-projected start/goal and same-edge cases)
  //   5. interpolate the A* node/edge sequence into segments
  //   6. Theta*-rewrite the two free-space transition segments
  //   7. Layer 2 obstacle bypass + diagnostic markers
  //   8. final-orientation enforcement
  //   9. publish RoutePlan, return nav_msgs/Path

  nav_msgs::msg::Path empty_path;
  empty_path.header.frame_id = global_frame_;
  empty_path.header.stamp = clock_->now();
  last_fallback_reason_ = FallbackReason::NONE;  // set below only on the no-graph bail-out

  // Per-request synchronous reload: ensure each plan uses the latest
  // RouteGraph from map_asset_manager. Timings logged for
  // observability — total includes service round-trip; rebuild is
  // the in-memory graph build alone. Failures fall through to the
  // existing isReady() check; the previous graph (if any) is kept.
  if (loader_) {
    const auto lr = loader_->syncLoad(std::chrono::seconds(2));
    if (lr.success) {
      RCLCPP_INFO(logger_,
        "RouteGraph syncLoad: total=%ldms rebuild=%ldms "
        "(%zu nodes / %zu edges)",
        static_cast<long>(lr.duration_total.count()),
        static_cast<long>(lr.duration_rebuild.count()),
        lr.node_count, lr.edge_count);
    } else {
      RCLCPP_WARN(logger_,
        "RouteGraph syncLoad failed (service_ready=%d, timed_out=%d, "
        "service_reported_failure=%d, total=%ldms) — falling back to "
        "previously loaded graph if any",
        lr.service_ready ? 1 : 0,
        lr.timed_out ? 1 : 0,
        lr.service_reported_failure ? 1 : 0,
        static_cast<long>(lr.duration_total.count()));
    }
  }

  // After syncLoad the graph must be ready to plan on, i.e. loaded AND
  // non-empty. isReady() is false both when nothing loaded yet and when the
  // map has no road network (0 nodes). Either way ask PlannerServer to fall
  // back to a free-space planner (isLoaded() only distinguishes the log).
  if (!graph_->isReady()) {
    last_fallback_reason_ = FallbackReason::NO_ROUTE_GRAPH;
    if (!graph_->isLoaded()) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
        "RouteGraph not loaded yet (syncLoad failed AND background load from "
        "%s still pending). Returning empty path for fallback / retry.",
        params_.asset_manager_node_name.c_str());
    } else {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
        "RouteGraph is empty (0 nodes) — map has no road network; returning "
        "empty path (PlannerServer falls back to free-space planner if configured).");
    }
    return empty_path;
  }

  auto lock = graph_->lockShared();

  // Step 1 — project start and goal (with diagnostic logs).
  auto projs = projectEndpoints(start, goal);
  if (!projs) {return empty_path;}
  auto & [start_proj, goal_proj] = *projs;

  const bool route_avoidance_enabled = ca_layer_ && ca_layer_->isEnabled();

  // An exact shared-node goal stays ON_NODE. When route avoidance is enabled,
  // an unsafe node fails closed instead of being reassigned to an arbitrary
  // incident edge and shifted to a different topology.
  if (route_avoidance_enabled &&
    goal_proj.type == StartGoalProjector::ResultType::ON_NODE &&
    !ca_layer_->isEndpointSafe(goal))
  {
    RCLCPP_ERROR(logger_,
      "Goal node '%s' is not safe in the current costmap; preserving ON_NODE "
      "topology and returning an empty path instead of shifting to another edge.",
      goal_proj.id.c_str());
    return empty_path;
  }

  // Step 2 — obstacle-aware projection adjustment. If a projection
  // lands inside a lethal-cost run on its edge, the transition Theta*
  // rewrite (step 4) would fail with "endpoint unreachable" and the whole
  // plan would be rejected. Slide the projection along the edge past the
  // obstacle +
  // margin so the rewrite has a reachable, clear endpoint to plan to.
  //   start_proj slides toward `goal`  — into the goal-side of the route
  //   goal_proj  slides toward `start` — into the start-side of the route
  // Both shift inward, on the reachable half of the obstacle.
  if (route_avoidance_enabled) {
    if (auto shifted = ca_layer_->shiftProjectionPastObstacle(
        start_proj, goal, *graph_,
        params_.bezier_samples_per_edge, params_.waypoints_per_meter))
    {
      if (shifted->projected_xy != start_proj.projected_xy) {
        RCLCPP_INFO(logger_,
          "start_proj shifted past obstacle on edge '%s' "
          "(%.2f,%.2f) -> (%.2f,%.2f). 起点投影 start_proj 落在障碍/高代价区域，已沿 edge 滑动到 clear point。",
          start_proj.id.c_str(),
          start_proj.projected_xy.first, start_proj.projected_xy.second,
          shifted->projected_xy.first, shifted->projected_xy.second);
      }
      start_proj = *shifted;
    } else {
      RCLCPP_ERROR(logger_,
        "start_proj on edge '%s' is inside an obstacle and the edge has "
        "no clear point in the slide direction. 起点投影 start_proj 位于障碍/高代价区域，且沿当前滑动方向找不到 clear point，"
        "本次全局规划将返回 empty path。",
        start_proj.id.c_str());
      return empty_path;
    }
    if (auto shifted = ca_layer_->shiftProjectionPastObstacle(
        goal_proj, start, *graph_,
        params_.bezier_samples_per_edge, params_.waypoints_per_meter))
    {
      if (shifted->projected_xy != goal_proj.projected_xy) {
        RCLCPP_INFO(logger_,
          "goal_proj shifted past obstacle on edge '%s' "
          "(%.2f,%.2f) -> (%.2f,%.2f). 目标投影 goal_proj 落在障碍/高代价区域，已沿 edge 滑动到 clear point。",
          goal_proj.id.c_str(),
          goal_proj.projected_xy.first, goal_proj.projected_xy.second,
          shifted->projected_xy.first, shifted->projected_xy.second);
      }
      goal_proj = *shifted;
    } else {
      RCLCPP_ERROR(logger_,
        "goal_proj on edge '%s' is inside an obstacle and the edge has "
        "no clear point in the slide direction. 目标投影 goal_proj 位于障碍/高代价区域，且沿当前滑动方向找不到 clear point，"
        "本次全局规划将返回 empty path。",
        goal_proj.id.c_str());
      return empty_path;
    }
  }

  // Step 3 — same-side shortcut: when start and goal projections sit very close,
  // skip the graph and let Theta* connect start→goal directly.
  // This is independently controlled by `enable_same_side_shortcut` and
  // `shortcut_max_proj_distance_m`, separated from `enable_route_costmap_avoidance`.
  if (params_.enable_same_side_shortcut) {
    if (auto shortcut = trySameSideShortcut(start, goal, start_proj, goal_proj)) {
      return *shortcut;
    }
  }

  // Step 4 — augment the graph with a virtual node at each ON_EDGE
  // projection, then run plain node-to-node A*. The augmentation turns
  // every "start/goal partway along an edge" case (including start & goal
  // sharing one edge) into ordinary nodes, so there is a single uniform
  // search path here — no per-endpoint blacklist, no same-edge branch.
  auto aug = buildAugmentedGraph(
    graph_, start_proj, goal_proj, params_.bezier_samples_per_edge);

  AStarSearcher::Request req;
  req.start_node_id = aug.start_id;
  req.goal_node_id = aug.goal_id;
  req.weights = params_.weights;
  req.prefs = params_.prefs;
  if (auto node = parent_node_.lock()) {
    node->get_parameter(name_ + ".prefer_no_stairs", req.prefs.prefer_no_stairs);
    node->get_parameter(name_ + ".prefer_no_elevator", req.prefs.prefer_no_elevator);
  }
  auto t0 = std::chrono::steady_clock::now();
  auto search = searcher_->search(req, *aug.graph);
  const long search_us = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now() - t0).count();
  if (!search.success) {
    RCLCPP_ERROR(logger_,
      "A* failed: %s (start=%s goal=%s expanded=%zu)",
      search.failure_reason.c_str(),
      aug.start_id.c_str(), aug.goal_id.c_str(), search.expanded_nodes);
    return empty_path;
  }
  RCLCPP_INFO(logger_,
    "A* found path: %zu nodes, %zu edges, cost=%.2f, expanded=%zu, %ldus",
    search.node_ids.size(), search.edge_ids.size(),
    search.total_cost, search.expanded_nodes, search_us);

  // Step 5 — interpolate uniformly over the A* node/edge sequence.
  auto plan = interpolator_->interpolate(start, goal, search, *aug.graph);
  if (plan.segments.empty()) {return empty_path;}
  auto stamp = clock_->now();

  // Step 6 — Theta*-rewrite the two transition placeholder segments.
  if (route_avoidance_enabled && !plan.segments.empty()) {
    if (!rewriteTransitionWithThetaStar(
        plan.segments.front(), "__start_transition__"))
    {
      return empty_path;
    }
    if (!rewriteTransitionWithThetaStar(
        plan.segments.back(), "__goal_alignment__"))
    {
      return empty_path;
    }
  }

  auto nav_path = plan.toNavPath(global_frame_, stamp);

  // Step 7 — Layer 2 obstacle detect + Theta* bypass + markers.
  applyObstacleBypass(nav_path, stamp);

  // Step 8 — final-orientation enforcement.
  enforceGoalOrientation(nav_path, goal);

  // Step 9 — publish RoutePlan + return.
  if (params_.publish_route_plan && rp_publisher_) {
    rp_publisher_->publish(plan.toMsg(global_frame_, stamp));
  }
  return nav_path;
}

rcl_interfaces::msg::SetParametersResult
NavRouteGraphPlanner::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  // Table-driven: look each parameter up in `paramTable()`, apply its
  // setter, OR-accumulate the `triggers_ca` flag. Keeps each parameter's
  // name in exactly one place instead of repeating it across a key
  // string + type-check + branch.
  const std::string prefix = name_ + ".";
  bool ca_dirty = false;
  for (const auto & p : parameters) {
    const std::string & full = p.get_name();
    if (full.size() <= prefix.size() ||
        full.compare(0, prefix.size(), prefix) != 0)
    {
      continue;
    }
    const std::string key = full.substr(prefix.size());
    for (const auto & h : paramTable()) {
      if (h.key == key) {
        h.apply(params_, p);
        ca_dirty = ca_dirty || h.triggers_ca;
        break;
      }
    }
  }
  if (ca_dirty && ca_layer_) {
    ca_layer_->updateParams(params_.ca);
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

// ── createPlan helpers ──────────────────────────────────────────────

namespace
{

/// Helper for the projection diagnostic log.
const char * resultTypeName(StartGoalProjector::ResultType t)
{
  switch (t) {
    case StartGoalProjector::ResultType::ON_EDGE: return "ON_EDGE";
    case StartGoalProjector::ResultType::ON_NODE: return "ON_NODE";
    default: return "INVALID";
  }
}

void logProjectionDiagnostic(
  const rclcpp::Logger & logger,
  const char * tag,
  const StartGoalProjector::Result & proj,
  const geometry_msgs::msg::PoseStamped & raw,
  const TopoGraph & graph)
{
  auto e_opt = graph.getEdge(proj.id);
  const double edge_len = e_opt ? e_opt->length : 0.0;
  const double frac = (proj.edge_total_samples > 1)
    ? static_cast<double>(proj.edge_sample_index) /
      static_cast<double>(proj.edge_total_samples - 1)
    : 0.0;
  const std::string from_id_s = e_opt ? e_opt->from_id : std::string("?");
  const std::string to_id_s = e_opt ? e_opt->to_id : std::string("?");
  RCLCPP_INFO(logger,
    "  %s: type=%s id=%s (from=%s to=%s) raw=(%.3f,%.3f) "
    "projected_xy=(%.3f,%.3f) dist_to_proj=%.3fm  edge_idx=%zu/%zu  "
    "t=%.3f  edge_len=%.2fm  walked_from_from=%.2fm walked_from_to=%.2fm",
    tag,
    resultTypeName(proj.type),
    proj.id.c_str(),
    from_id_s.c_str(), to_id_s.c_str(),
    raw.pose.position.x, raw.pose.position.y,
    proj.projected_xy.first, proj.projected_xy.second,
    proj.distance,
    proj.edge_sample_index,
    (proj.edge_total_samples > 0 ? proj.edge_total_samples - 1 : 0u),
    frac, edge_len,
    edge_len * frac, edge_len * (1.0 - frac));
}

void applyFinalOrientation(
  geometry_msgs::msg::Pose & last_pose,
  const geometry_msgs::msg::Point & prev_point,
  const geometry_msgs::msg::Quaternion & goal_orientation,
  bool use_approach_orientation,
  bool have_prev)
{
  if (use_approach_orientation && have_prev) {
    const double yaw = std::atan2(
      last_pose.position.y - prev_point.y,
      last_pose.position.x - prev_point.x);
    last_pose.orientation.x = 0.0;
    last_pose.orientation.y = 0.0;
    last_pose.orientation.z = std::sin(yaw * 0.5);
    last_pose.orientation.w = std::cos(yaw * 0.5);
  } else {
    last_pose.orientation = goal_orientation;
  }
}

}  // namespace

std::optional<
  std::pair<StartGoalProjector::Result, StartGoalProjector::Result>>
NavRouteGraphPlanner::projectEndpoints(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  // projectPreferEdge keeps the plan ON the road graph by forcing
  // ON_EDGE whenever any enabled edge exists.
  auto start_proj = projector_->projectPreferEdge(start, *graph_);
  if (start_proj.type == StartGoalProjector::ResultType::INVALID) {
    RCLCPP_ERROR(logger_,
      "Start pose (%.2f, %.2f) — graph has no nodes or edges, cannot plan",
      start.pose.position.x, start.pose.position.y);
    return std::nullopt;
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
  logProjectionDiagnostic(logger_, "start_proj", start_proj, start, *graph_);

  auto goal_proj = projector_->projectGoalUnlimited(goal, *graph_);
  if (goal_proj.type == StartGoalProjector::ResultType::INVALID) {
    RCLCPP_ERROR(logger_,
      "Goal pose (%.2f, %.2f) — graph has no nodes or edges, cannot plan",
      goal.pose.position.x, goal.pose.position.y);
    return std::nullopt;
  }
  if (goal_proj.distance > params_.projection_node_threshold_m &&
      goal_proj.distance > params_.projection_edge_threshold_m)
  {
    RCLCPP_INFO(logger_,
      "Goal pose (%.2f, %.2f) is %.2fm from nearest graph element "
      "(beyond diagnostic thresholds) — bridging with Theta*",
      goal.pose.position.x, goal.pose.position.y, goal_proj.distance);
  }
  logProjectionDiagnostic(logger_, "goal_proj ", goal_proj, goal, *graph_);

  return std::make_pair(std::move(start_proj), std::move(goal_proj));
}

std::optional<nav_msgs::msg::Path>
NavRouteGraphPlanner::trySameSideShortcut(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const StartGoalProjector::Result & start_proj,
  const StartGoalProjector::Result & goal_proj)
{
  if (params_.shortcut_max_proj_distance_m <= 0.0) {return std::nullopt;}
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
  if (proj_dist >= params_.shortcut_max_proj_distance_m) {
    return std::nullopt;
  }

  // Execute Theta* planning for shortcut without requiring `enable_route_costmap_avoidance`.
  auto direct = ca_layer_->planWithThetaStar(start, goal, false);
  if (!direct || direct->empty()) {
    RCLCPP_WARN(logger_,
      "Same-side shortcut: proj distance %.2fm < threshold but Theta* "
      "direct failed; falling through to graph routing. 诊断(diagnostic): %s",
      proj_dist, ca_layer_->describeLastThetaStarFailure().c_str());
    return std::nullopt;
  }

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
    const bool have_prev = out.poses.size() >= 2;
    const auto prev_point = have_prev
      ? out.poses[out.poses.size() - 2].pose.position
      : geometry_msgs::msg::Point();
    applyFinalOrientation(out.poses.back().pose, prev_point,
      goal.pose.orientation,
      params_.use_final_approach_orientation, have_prev);
  }
  return out;
}

void NavRouteGraphPlanner::enforceGoalOrientation(
  nav_msgs::msg::Path & nav_path,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (nav_path.poses.empty()) {return;}
  const bool have_prev = nav_path.poses.size() >= 2;
  const auto prev_point = have_prev
    ? nav_path.poses[nav_path.poses.size() - 2].pose.position
    : geometry_msgs::msg::Point();
  applyFinalOrientation(nav_path.poses.back().pose, prev_point,
    goal.pose.orientation,
    params_.use_final_approach_orientation, have_prev);
}

bool NavRouteGraphPlanner::rewriteTransitionWithThetaStar(
  InterpolatedSegment & seg, const std::string & expected_id)
{
  if (seg.edge_id != expected_id || seg.samples.size() < 2) {
    return true;  // not the placeholder we expected; leave as-is
  }
  const auto & seg_start = seg.samples.front();
  const auto & seg_end = seg.samples.back();
  auto theta_path = ca_layer_->planWithThetaStar(seg_start, seg_end);
  if (!theta_path) {
    RCLCPP_ERROR(logger_,
      "Theta* 接入段规划失败(transition planning failed) for '%s': "
      "(%.2f, %.2f) -> (%.2f, %.2f). 具体原因(diagnostic): %s",
      expected_id.c_str(),
      seg_start.pose.position.x, seg_start.pose.position.y,
      seg_end.pose.position.x, seg_end.pose.position.y,
      ca_layer_->describeLastThetaStarFailure().c_str());
    return false;
  }
  seg.samples = std::move(*theta_path);
  // Preserve the historical "*_theta__" tag (e.g. __start_transition_theta__)
  // so any downstream consumer that pattern-matches keeps working.
  seg.edge_id = expected_id.substr(0, expected_id.size() - 2) + "_theta__";
  return true;
}

namespace
{

constexpr double kMarkerSphereScale = 0.18;
constexpr double kMarkerTextScale = 0.20;
constexpr double kMarkerLineWidth = 0.05;
constexpr double kMarkerTextZOffset = 0.30;

/// Build the obstacle-segment MarkerArray for RViz: each detected
/// obstacle gets a green start sphere + label, a red end sphere +
/// label, and an orange LINE_STRIP spanning the path samples inside.
visualization_msgs::msg::MarkerArray buildObstacleMarkerArray(
  const std::vector<CostmapAwareLayer::ObstacleSegment> & obstacles,
  const std::vector<geometry_msgs::msg::PoseStamped> & path,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  visualization_msgs::msg::MarkerArray ma;

  // Clear previous round's markers.
  visualization_msgs::msg::Marker clr;
  clr.header.frame_id = frame_id;
  clr.header.stamp = stamp;
  clr.ns = "obstacle_segments";
  clr.action = visualization_msgs::msg::Marker::DELETEALL;
  ma.markers.push_back(clr);

  int mid = 0;
  for (std::size_t k = 0; k < obstacles.size(); ++k) {
    const auto & ob = obstacles[k];

    // Start sphere (green) + label.
    visualization_msgs::msg::Marker s_sphere;
    s_sphere.header.frame_id = frame_id;
    s_sphere.header.stamp = stamp;
    s_sphere.ns = "obstacle_segments";
    s_sphere.id = mid++;
    s_sphere.type = visualization_msgs::msg::Marker::SPHERE;
    s_sphere.action = visualization_msgs::msg::Marker::ADD;
    s_sphere.pose = path[ob.start_index].pose;
    s_sphere.scale.x = s_sphere.scale.y = s_sphere.scale.z = kMarkerSphereScale;
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
    s_text.pose.position.z += kMarkerTextZOffset;
    s_text.scale.z = kMarkerTextScale;
    s_text.color.r = s_text.color.g = s_text.color.b = 1.0f;
    s_text.color.a = 1.0f;
    s_text.text = "obs#" + std::to_string(k) +
      " start_id=" + std::to_string(ob.start_index);
    ma.markers.push_back(s_text);

    // End sphere (red) + label.
    visualization_msgs::msg::Marker e_sphere = s_sphere;
    e_sphere.id = mid++;
    e_sphere.pose = path[ob.end_index].pose;
    e_sphere.color.r = 1.0f; e_sphere.color.g = 0.0f;
    ma.markers.push_back(e_sphere);

    visualization_msgs::msg::Marker e_text = s_text;
    e_text.id = mid++;
    e_text.pose = e_sphere.pose;
    e_text.pose.position.z += kMarkerTextZOffset;
    e_text.text = "obs#" + std::to_string(k) +
      " end_id=" + std::to_string(ob.end_index);
    ma.markers.push_back(e_text);

    // Orange LINE_STRIP spanning the obstacle's path samples.
    visualization_msgs::msg::Marker line;
    line.header = s_sphere.header;
    line.ns = "obstacle_segments";
    line.id = mid++;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = kMarkerLineWidth;
    line.color.r = 1.0f; line.color.g = 0.5f;
    line.color.b = 0.0f; line.color.a = 0.8f;
    for (std::size_t i = ob.start_index; i <= ob.end_index; ++i) {
      line.points.push_back(path[i].pose.position);
    }
    ma.markers.push_back(line);
  }
  return ma;
}

}  // namespace

void NavRouteGraphPlanner::applyObstacleBypass(
  nav_msgs::msg::Path & nav_path, const rclcpp::Time & stamp)
{
  if (!ca_layer_ || nav_path.poses.size() <= 2) {return;}
  auto obstacles = ca_layer_->detectAllObstacles(nav_path.poses);

  // Always publish markers for diagnostic visualization.
  if (obstacle_markers_pub_) {
    auto ma = buildObstacleMarkerArray(
      obstacles, nav_path.poses, global_frame_, stamp);
    obstacle_markers_pub_->publish(ma);
    if (!obstacles.empty()) {
      RCLCPP_INFO(logger_,
        "Layer 2: published %zu obstacle marker(s) on "
        "'~/%s/obstacle_segments'",
        obstacles.size(), name_.c_str());
    }
  }

  // Bypass-splice only when total route costmap avoidance is on.
  if (!ca_layer_->isEnabled() || obstacles.empty()) {return;}

  RCLCPP_INFO(logger_,
    "Layer 2: detected %zu obstacle run(s) on path of %zu samples",
    obstacles.size(), nav_path.poses.size());

  // Process REVERSE so earlier indices remain valid after later splices.
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
        "(%.2f,%.2f)->(%.2f,%.2f) — keeping original path here. "
        "诊断(diagnostic): %s",
        ob.start_index, ob.end_index,
        seg_start.pose.position.x, seg_start.pose.position.y,
        seg_end.pose.position.x, seg_end.pose.position.y,
        ca_layer_->describeLastThetaStarFailure().c_str());
      continue;
    }

    // Splice A → Theta* → B (in this order, on the ORIGINAL nav_path).
    std::vector<geometry_msgs::msg::PoseStamped> spliced;
    spliced.reserve(nav_path.poses.size() + bypass->size());
    spliced.insert(spliced.end(),
      nav_path.poses.begin(),
      nav_path.poses.begin() + ob.start_index);
    for (auto & p : *bypass) {
      p.header.frame_id = global_frame_;
      p.header.stamp = stamp;
      spliced.push_back(p);
    }
    spliced.insert(spliced.end(),
      nav_path.poses.begin() + ob.end_index + 1,
      nav_path.poses.end());

    RCLCPP_INFO(logger_,
      "Layer 2: spliced bypass for obstacle [%zu..%zu]: "
      "|A|=%zu + |Theta*|=%zu + |B|=%zu = %zu",
      ob.start_index, ob.end_index, ob.start_index,
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

}  // namespace nav2_route_planner

PLUGINLIB_EXPORT_CLASS(
  nav2_route_planner::NavRouteGraphPlanner,
  nav2_core::GlobalPlanner)
