// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include "nav2_route_planner/path_interpolator.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_route_planner
{

namespace
{

geometry_msgs::msg::Quaternion yawToQuat(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

}  // namespace

std::vector<std::pair<double, double>> PathInterpolator::sampleBezier(
  const std::vector<std::pair<double, double>> & cp,
  int num_samples)
{
  // Bernstein binomial — small N, recompute is fine.
  auto binom = [](int n, int k) {
      if (k == 0 || k == n) {return 1.0;}
      double v = 1.0;
      for (int i = 0; i < k; ++i) {v = v * (n - i) / (i + 1);}
      return v;
    };
  std::vector<std::pair<double, double>> out;
  out.reserve(static_cast<std::size_t>(num_samples) + 1);
  const int n = static_cast<int>(cp.size()) - 1;
  for (int s = 0; s <= num_samples; ++s) {
    double t = static_cast<double>(s) / num_samples;
    double bx = 0.0;
    double by = 0.0;
    for (int i = 0; i <= n; ++i) {
      double b = binom(n, i) * std::pow(1.0 - t, n - i) * std::pow(t, i);
      bx += b * cp[i].first;
      by += b * cp[i].second;
    }
    out.emplace_back(bx, by);
  }
  return out;
}

void PathInterpolator::emitLinear(
  std::vector<geometry_msgs::msg::PoseStamped> & out,
  double x1, double y1, double x2, double y2) const
{
  const double dist = std::hypot(x2 - x1, y2 - y1);
  if (dist < 1e-6) {return;}
  const int loops = std::max(1, static_cast<int>(dist * p_.waypoints_per_meter));
  for (int k = 1; k <= loops; ++k) {
    double t = static_cast<double>(k) / loops;
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = x1 + t * (x2 - x1);
    p.pose.position.y = y1 + t * (y2 - y1);
    p.pose.orientation.w = 1.0;
    out.push_back(std::move(p));
  }
}

void PathInterpolator::computeOrientations(std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  if (poses.size() < 2) {return;}
  for (std::size_t i = 0; i + 1 < poses.size(); ++i) {
    double dx = poses[i + 1].pose.position.x - poses[i].pose.position.x;
    double dy = poses[i + 1].pose.position.y - poses[i].pose.position.y;
    if (std::hypot(dx, dy) < 1e-6) {
      // Carry forward previous orientation if any
      if (i > 0) {
        poses[i].pose.orientation = poses[i - 1].pose.orientation;
      } else {
        poses[i].pose.orientation.w = 1.0;
      }
      continue;
    }
    poses[i].pose.orientation = yawToQuat(std::atan2(dy, dx));
  }
  // Last pose copies its predecessor's heading.
  poses.back().pose.orientation = poses[poses.size() - 2].pose.orientation;
}

InterpolatedRoutePlan PathInterpolator::interpolate(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const StartGoalProjector::Result & start_proj,
  const std::string & goal_node_id,
  const AStarSearcher::Result & search_result,
  const TopoGraph & graph) const
{
  InterpolatedRoutePlan plan;
  if (!search_result.success || search_result.node_ids.empty()) {
    return plan;
  }

  // ── Step 1: start transition (start_pose → first node on path) ────────
  // The first node in search_result.node_ids is the entry node
  // (either an ON_NODE projection, or one of the two endpoints of
  // an ON_EDGE projection — A* picked whichever had the cheaper
  // overall cost).
  const std::string first_node_id = search_result.node_ids.front();
  auto first_node_opt = graph.getNode(first_node_id);
  if (!first_node_opt) {return plan;}

  std::vector<geometry_msgs::msg::PoseStamped> transition_samples;
  transition_samples.push_back(start);
  if (start_proj.type == StartGoalProjector::ResultType::ON_EDGE) {
    // First go to the projection point, then to the entry node.
    emitLinear(transition_samples,
      start.pose.position.x, start.pose.position.y,
      start_proj.projected_xy.first, start_proj.projected_xy.second);
    emitLinear(transition_samples,
      start_proj.projected_xy.first, start_proj.projected_xy.second,
      first_node_opt->x, first_node_opt->y);
  } else {
    emitLinear(transition_samples,
      start.pose.position.x, start.pose.position.y,
      first_node_opt->x, first_node_opt->y);
  }

  InterpolatedSegment transition_seg;
  transition_seg.edge_id = "__start_transition__";
  transition_seg.lane_subtype = "transition";
  transition_seg.floor = first_node_opt->floor;
  transition_seg.samples = std::move(transition_samples);
  plan.segments.push_back(std::move(transition_seg));
  plan.floor_sequence.push_back(first_node_opt->floor);

  // ── Step 2: per-edge sampling along A* result ─────────────────────────
  for (std::size_t i = 0; i < search_result.edge_ids.size(); ++i) {
    const std::string & eid = search_result.edge_ids[i];
    const std::string & from_id = search_result.node_ids[i];
    auto e_opt = graph.getEdge(eid);
    if (!e_opt) {continue;}
    const TopoEdge & e = *e_opt;

    // Determine traversal direction.
    const bool reverse = (e.from_id != from_id);

    auto edge_pts = graph.sampleEdge(e, p_.bezier_samples_per_edge);
    if (edge_pts.size() < 2) {continue;}
    if (reverse) {
      std::reverse(edge_pts.begin(), edge_pts.end());
    }

    // Build the segment poses by linearly interpolating between
    // consecutive sampled control points so we hit waypoints_per_meter
    // density even on long LINE segments.
    std::vector<geometry_msgs::msg::PoseStamped> seg_samples;
    geometry_msgs::msg::PoseStamped p0;
    p0.pose.position.x = edge_pts.front().first;
    p0.pose.position.y = edge_pts.front().second;
    p0.pose.orientation.w = 1.0;
    seg_samples.push_back(std::move(p0));
    for (std::size_t k = 1; k < edge_pts.size(); ++k) {
      emitLinear(seg_samples,
        edge_pts[k - 1].first, edge_pts[k - 1].second,
        edge_pts[k].first, edge_pts[k].second);
    }

    InterpolatedSegment seg;
    seg.edge_id = e.id;
    seg.lane_subtype = e.lane_subtype;
    seg.max_speed = e.max_speed;
    seg.regulatory_element_ids = e.regulatory_element_ids;
    seg.floor = e.floor;
    seg.samples = std::move(seg_samples);
    plan.segments.push_back(std::move(seg));

    // Track floor changes.
    if (plan.floor_sequence.empty() || plan.floor_sequence.back() != e.floor) {
      plan.floor_sequence.push_back(e.floor);
      plan.floor_changes++;
    }
    plan.total_distance += e.length;
  }

  // ── Step 3: goal alignment (last segment may end at goal node, but
  // the caller asked for `goal` pose specifically). Add a short alignment
  // segment if goal is offset from the goal node, and override final
  // orientation. ─────────────────────────────────────────────────────────
  auto goal_node_opt = graph.getNode(goal_node_id);
  if (goal_node_opt) {
    const double dx = goal.pose.position.x - goal_node_opt->x;
    const double dy = goal.pose.position.y - goal_node_opt->y;
    if (std::hypot(dx, dy) > 1e-3 || true) {
      // Always emit a final alignment so the controller sees `goal` as
      // the last waypoint, matching nav2_theta_star_planner behavior.
      InterpolatedSegment goal_seg;
      goal_seg.edge_id = "__goal_alignment__";
      goal_seg.lane_subtype = "transition";
      goal_seg.floor = goal_node_opt->floor;
      std::vector<geometry_msgs::msg::PoseStamped> g_samples;
      emitLinear(g_samples,
        goal_node_opt->x, goal_node_opt->y,
        goal.pose.position.x, goal.pose.position.y);
      // Append the goal pose itself to terminate the path exactly.
      geometry_msgs::msg::PoseStamped gp = goal;
      g_samples.push_back(std::move(gp));
      goal_seg.samples = std::move(g_samples);
      plan.segments.push_back(std::move(goal_seg));
    }
  }

  // ── Step 4: orientations + plan_id ───────────────────────────────────
  for (auto & seg : plan.segments) {
    computeOrientations(seg.samples);
  }

  // Force the very last sample's orientation to the goal's heading
  // (matches nav2_theta_star_planner's end-of-plan behavior).
  if (!plan.segments.empty() && !plan.segments.back().samples.empty()) {
    if (!p_.use_final_approach_orientation) {
      plan.segments.back().samples.back().pose.orientation = goal.pose.orientation;
    }
  }

  // Synthesize a plan id from start/goal coords (good enough for logs).
  std::ostringstream id;
  id << "plan_" << static_cast<int64_t>(start.pose.position.x * 1000) << "_"
     << static_cast<int64_t>(start.pose.position.y * 1000) << "_to_"
     << goal_node_id;
  plan.plan_id = id.str();
  return plan;
}

// ── Conversions ─────────────────────────────────────────────────────────

nav_msgs::msg::Path InterpolatedRoutePlan::toNavPath(
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  bool /*use_final_approach_orientation*/) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = stamp;
  for (const auto & seg : segments) {
    for (const auto & p : seg.samples) {
      geometry_msgs::msg::PoseStamped ps = p;
      ps.header.frame_id = frame_id;
      ps.header.stamp = stamp;
      path.poses.push_back(std::move(ps));
    }
  }
  return path;
}

nav_interfaces::msg::RoutePlan InterpolatedRoutePlan::toMsg(
  const std::string & frame_id,
  const rclcpp::Time & stamp) const
{
  nav_interfaces::msg::RoutePlan msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.plan_id = plan_id;
  msg.floor_sequence = floor_sequence;
  msg.total_distance = static_cast<float>(total_distance);
  msg.estimated_time = static_cast<float>(estimated_time);
  msg.floor_changes = floor_changes;
  msg.segments.reserve(segments.size());
  for (const auto & seg : segments) {
    nav_interfaces::msg::RoutePlanSegment sm;
    sm.edge_id = seg.edge_id;
    sm.lane_subtype = seg.lane_subtype;
    sm.max_speed = static_cast<float>(seg.max_speed);
    sm.regulatory_element_ids = seg.regulatory_element_ids;
    sm.floor = seg.floor;
    sm.samples.reserve(seg.samples.size());
    for (const auto & ps : seg.samples) {
      geometry_msgs::msg::PoseStamped p = ps;
      p.header.frame_id = frame_id;
      p.header.stamp = stamp;
      sm.samples.push_back(std::move(p));
    }
    msg.segments.push_back(std::move(sm));
  }
  return msg;
}

}  // namespace nav2_route_planner
