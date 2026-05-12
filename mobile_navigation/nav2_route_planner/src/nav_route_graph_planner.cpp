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

  // ── Step 2: project goal (must hit a graph node) ───────────────────
  // Goal must land on a topology node — either a NavPoint or a Junction.
  // Both are valid destinations; A* and the interpolator treat them
  // uniformly.
  auto goal_id = projector_->projectToNode(goal, *graph_);
  if (!goal_id) {
    RCLCPP_ERROR(logger_,
      "Goal pose (%.2f, %.2f) does not match any NavPoint or Junction within %.2fm",
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
