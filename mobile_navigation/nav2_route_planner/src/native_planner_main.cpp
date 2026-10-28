// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <nlohmann/json.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_route_planner/astar_searcher.hpp"
#include "nav2_route_planner/graph_augmenter.hpp"
#include "nav2_route_planner/path_interpolator.hpp"
#include "nav2_route_planner/start_goal_projector.hpp"
#include "nav2_route_planner/topo_graph.hpp"

using json = nlohmann::json;
using namespace nav2_route_planner;

int main(int /*argc*/, char ** /*argv*/)
{
  std::ios::sync_with_stdio(false);
  std::cin.tie(nullptr);

  // Read JSON payload from stdin
  json input;
  try {
    std::cin >> input;
  } catch (const std::exception & e) {
    json err;
    err["success"] = false;
    err["error"] = std::string("JSON parse error: ") + e.what();
    std::cout << err.dump() << std::endl;
    return 1;
  }

  const auto t_start = std::chrono::steady_clock::now();

  try {
    // 1. Build TopoGraph
    auto graph = std::make_shared<TopoGraph>();
    std::vector<TopoNode> nodes;
    std::vector<TopoEdge> edges;

    std::unordered_map<std::string, std::pair<double, double>> node_coords;

    if (input.contains("nodes") && input["nodes"].is_object()) {
      for (auto it = input["nodes"].begin(); it != input["nodes"].end(); ++it) {
        const std::string id = it.key();
        const double x = it.value().value("x", 0.0);
        const double y = it.value().value("y", 0.0);
        node_coords[id] = {x, y};

        TopoNode n;
        n.id = id;
        n.kind = NodeKind::NAV_POINT;
        n.x = x;
        n.y = y;
        n.floor = "1";
        nodes.push_back(std::move(n));
      }
    }

    if (input.contains("edges") && input["edges"].is_array()) {
      for (const auto & e_json : input["edges"]) {
        TopoEdge e;
        e.id = e_json.value("id", "");
        e.from_id = e_json.value("from", "");
        e.to_id = e_json.value("to", "");
        e.bidirectional = e_json.value("bidirectional", false);
        const std::string geom_type = e_json.value("type", "line");
        e.geometry = (geom_type == "bezier") ? EdgeGeometry::BEZIER : EdgeGeometry::LINE;
        e.enabled = true;
        e.floor = "1";

        if (e.geometry == EdgeGeometry::BEZIER) {
          double cx = 0.0, cy = 0.0;
          if (e_json.contains("ctrl") && e_json["ctrl"].is_object()) {
            cx = e_json["ctrl"].value("x", 0.0);
            cy = e_json["ctrl"].value("y", 0.0);
          }
          auto fn_it = node_coords.find(e.from_id);
          auto tn_it = node_coords.find(e.to_id);
          if (fn_it != node_coords.end() && tn_it != node_coords.end()) {
            e.control_points = { fn_it->second, {cx, cy}, tn_it->second };
          }
        }
        edges.push_back(std::move(e));
      }
    }

    graph->rebuildFrom(std::move(nodes), std::move(edges));

    // 2. Parse start and goal poses
    geometry_msgs::msg::PoseStamped start_pose;
    geometry_msgs::msg::PoseStamped goal_pose;

    if (input.contains("start")) {
      start_pose.pose.position.x = input["start"].value("x", 0.0);
      start_pose.pose.position.y = input["start"].value("y", 0.0);
      const double heading = input["start"].value("heading", 0.0);
      start_pose.pose.orientation.z = std::sin(heading / 2.0);
      start_pose.pose.orientation.w = std::cos(heading / 2.0);
    }

    if (input.contains("goal")) {
      goal_pose.pose.position.x = input["goal"].value("x", 0.0);
      goal_pose.pose.position.y = input["goal"].value("y", 0.0);
      const double heading = input["goal"].value("heading", 0.0);
      goal_pose.pose.orientation.z = std::sin(heading / 2.0);
      goal_pose.pose.orientation.w = std::cos(heading / 2.0);
    }

    // 3. Project start and goal
    StartGoalProjector::Params proj_params;
    proj_params.bezier_samples_per_edge = 40;
    proj_params.node_snap_threshold_m = input.value("node_snap_threshold_m", 0.4);
    StartGoalProjector projector(proj_params);

    auto start_proj = projector.projectPreferEdge(start_pose, *graph);
    auto goal_proj = projector.projectPreferEdge(goal_pose, *graph);

    if (start_proj.type == StartGoalProjector::ResultType::INVALID ||
        goal_proj.type == StartGoalProjector::ResultType::INVALID)
    {
      json out;
      out["success"] = false;
      out["error"] = "Start or Goal cannot be projected (empty road network)";
      std::cout << out.dump() << std::endl;
      return 0;
    }

    // 4. Augment Graph with Virtual Nodes
    auto aug = buildAugmentedGraph(graph, start_proj, goal_proj, 40);

    // 5. Run Topological A* Search
    AStarSearcher searcher;
    AStarSearcher::Request astar_req;
    astar_req.start_node_id = aug.start_id;
    astar_req.goal_node_id = aug.goal_id;

    auto search_res = searcher.search(astar_req, *aug.graph);

    // 6. Interpolate Geometry Path
    PathInterpolator::Params interp_params;
    interp_params.waypoints_per_meter = 20;
    interp_params.bezier_samples_per_edge = 40;
    PathInterpolator interpolator(interp_params);

    auto interp_plan = interpolator.interpolate(start_pose, goal_pose, search_res, *aug.graph);

    const auto t_end = std::chrono::steady_clock::now();
    const long duration_us = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();

    // 7. Output result JSON
    json out;
    out["success"] = search_res.success;
    out["source"] = "libnav2_route_planner.so (Native C++)";
    out["search_time_us"] = duration_us;
    out["total_cost"] = search_res.total_cost;
    out["expanded_nodes"] = search_res.expanded_nodes;
    out["topo_nodes"] = search_res.node_ids;
    out["edge_ids"] = search_res.edge_ids;

    if (!search_res.success) {
      out["error"] = "拓扑 A* 搜索失败：未找到从起点吸附边/点 (" + aug.start_id + ") 到目标吸附边/点 (" + aug.goal_id + ") 的有效路径。可能原因：车道单向逆行阻断、两区域之间车道断开或未连通。";
    }

    out["start_proj"] = {
      {"type", start_proj.type == StartGoalProjector::ResultType::ON_EDGE ? "ON_EDGE" : "ON_NODE"},
      {"id", start_proj.id},
      {"distance", start_proj.distance},
      {"x", start_proj.projected_xy.first},
      {"y", start_proj.projected_xy.second}
    };

    out["goal_proj"] = {
      {"type", goal_proj.type == StartGoalProjector::ResultType::ON_EDGE ? "ON_EDGE" : "ON_NODE"},
      {"id", goal_proj.id},
      {"distance", goal_proj.distance},
      {"x", goal_proj.projected_xy.first},
      {"y", goal_proj.projected_xy.second}
    };

    std::vector<json> path_points;
    for (const auto & seg : interp_plan.segments) {
      for (const auto & sample : seg.samples) {
        json pt;
        pt["x"] = sample.pose.position.x;
        pt["y"] = sample.pose.position.y;
        pt["edge_id"] = seg.edge_id;
        path_points.push_back(pt);
      }
    }
    out["path_points"] = path_points;
    out["total_samples"] = path_points.size();

    std::cout << out.dump() << std::endl;
  } catch (const std::exception & e) {
    json err;
    err["success"] = false;
    err["error"] = std::string("Exception in Native Planner: ") + e.what();
    std::cout << err.dump() << std::endl;
    return 1;
  }

  return 0;
}
