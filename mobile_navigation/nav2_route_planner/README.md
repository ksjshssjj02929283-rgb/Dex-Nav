# nav2_route_planner

Nav2 GlobalPlanner plugin that plans on a topological road-graph (NavPoints +
Junctions + Edges) served by `map_asset_manager`. It outputs the standard
`nav_msgs/Path` for Nav2 controllers and, in parallel, publishes
`nav_interfaces/RoutePlan` carrying per-segment semantics (`max_speed`,
`lane_subtype`, regulatory-element references, `floor`) that downstream
`RuntimeBinder` / behavior-tree subtrees can consume.

## Components

| File | Role |
|------|------|
| `topo_graph.{hpp,cpp}` | In-memory thread-safe graph (nodes / edges / sampling) |
| `route_graph_loader.{hpp,cpp}` | Pulls graph + NavPoints from `map_asset_manager` services |
| `start_goal_projector.{hpp,cpp}` | Projects free-space pose onto nearest NavPoint or edge |
| `cost_function.{hpp,cpp}` | Edge/heuristic cost (distance + cross-floor + reg-element + prefs) |
| `astar_searcher.{hpp,cpp}` | A* over the road-graph, supports start-on-edge entry |
| `path_interpolator.{hpp,cpp}` | LINE/BEZIER sampling, start transition, goal alignment |
| `costmap_aware_layer.{hpp,cpp}` | Optional costmap-driven projection, Theta* transitions, and in-graph bypass |
| `route_plan_publisher.{hpp,cpp}` | Lifecycle-aware `RoutePlan` topic publisher (transient_local) |
| `nav_route_graph_planner.{hpp,cpp}` | Plugin entry point, glues everything to `nav2_core::GlobalPlanner` |

## Topics & services

* Subscribes (via clients): `/map_asset_manager/get_route_graph`,
  `/map_asset_manager/get_all_nav_points`
* Publishes:
  - `nav_msgs/Path` via standard Nav2 `~/plan` (PlannerServer hosts this)
  - `nav_interfaces/RoutePlan` on `~/<planner_id>/route_plan`
    (e.g. `/planner_server/RouteGridPlanner/route_plan`), QoS reliable +
    transient_local

## Quick start

1. Build:

   ```bash
   cd ~/dexforce/8_interfaces && colcon build --packages-select nav_interfaces
   source install/setup.bash
   cd ~/dexforce/1_navigation/nav_dev_ws/dexe_mobile_nav
   colcon build --packages-select nav2_route_planner
   source install/setup.bash
   ```

2. Make sure `map_asset_manager` is running and a `RouteGraph` has been
   created (RViz tools or service calls). Then merge
   `config/nav2_params_route.yaml` into your robot's nav2_params.yaml so
   that `planner_server` loads `RouteGridPlanner` (and optionally a
   `FallbackGridBased` planner for off-graph goals).

3. Send a goal pose that lies near a NavPoint (within 0.5 m by default);
   the resulting `nav_msgs/Path` follows the road graph.

## Algorithm summary

```
createPlan(start, goal):
  graph = lazy-load via RouteGraphLoader if not ready
  start_proj = projector_->projectPreferEdge(start)
  goal_proj  = projector_->projectGoalUnlimited(goal) # shared-node tie -> ON_NODE
  if enable_route_costmap_avoidance:
    validate shared-node goal; shift unsafe ON_EDGE projections
    optionally use same-side Theta* shortcut
  result = AStarSearcher::search(augmented_graph, weights, prefs)
  plan   = PathInterpolator::interpolate(start, goal, ..., result)
  if enable_route_costmap_avoidance:
    Theta*-rewrite start/goal transitions and in-graph obstacle bypass
  publish RoutePlan(plan)                        # parallel, with semantics
  return    plan.toNavPath()                     # nav_msgs::Path for Nav2
```

A* mirrors the ROS1 `waypoint-global-planner` Dijkstra logic but uses a
priority queue and an admissible heuristic so it scales to large graphs.
Path interpolation (LINE / Bernstein-Bézier) is a stateless port of the
ROS1 `InterpolatePath` / `genBezierPointsByControlPoints` routines.

## Route costmap avoidance switch

`RouteGridPlanner.enable_route_costmap_avoidance` is the master switch for
costmap-driven geometry changes inside `NavRouteGraphPlanner` and supports
runtime parameter updates.

| Value | RouteGridPlanner behavior |
|------|----------------------------|
| `false` (default) | No projection shifting, same-side Theta*, start/goal transition Theta*, or in-graph Theta* bypass. Road-graph A* still selects graph topology. |
| `true` | Enables all four costmap-driven route-geometry stages. An unsafe exact shared-node goal fails closed instead of being moved onto an arbitrary incident edge. |

Controller-side collision checking and PlannerServer's no-route-graph fallback
are independent safety/availability mechanisms and are not disabled by this
parameter.

```bash
ros2 param set /planner_server \
  RouteGridPlanner.enable_route_costmap_avoidance true
```

The former `RouteGridPlanner.costmap_aware_mode` parameter was removed in
version `0.4.6`; launch/configuration files must use the new name.

## Safety boundary

Disabling route-level costmap avoidance is intended for controlled road-graph
operation and diagnosis. It must not be used to disable controller collision
stopping, and it does not make a straight start/goal transition safe to drive.
