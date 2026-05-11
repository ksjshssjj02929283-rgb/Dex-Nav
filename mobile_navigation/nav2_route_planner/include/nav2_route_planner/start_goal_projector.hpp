// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_ROUTE_PLANNER__START_GOAL_PROJECTOR_HPP_
#define NAV2_ROUTE_PLANNER__START_GOAL_PROJECTOR_HPP_

#include <optional>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_route_planner/topo_graph.hpp"

namespace nav2_route_planner
{

/// Projects a free-space pose onto the road graph.
///
/// Algorithm (mirrors ROS1 `GetPointId` / `GetLineId` / `GetVerticalPoint`):
///   1. Find nearest NavPoint within `node_threshold` → ON_NODE
///   2. Else find nearest edge sample within `edge_threshold` → ON_EDGE
///   3. Else INVALID
class StartGoalProjector
{
public:
  enum class ResultType { ON_NODE, ON_EDGE, INVALID };

  struct Result
  {
    ResultType type{ResultType::INVALID};
    std::string id;                          // node_id or edge_id
    double distance{0.0};
    std::pair<double, double> projected_xy{0.0, 0.0};  // valid for ON_EDGE
    /// For ON_EDGE: index along the sampled polyline (used by interpolator
    /// to know where on the edge we joined). 0 = at from_id end.
    std::size_t edge_sample_index{0};
    std::size_t edge_total_samples{0};
  };

  struct Params
  {
    double node_threshold_m{0.5};
    double edge_threshold_m{1.0};
    int bezier_samples_per_edge{40};
  };

  explicit StartGoalProjector(Params p) : p_(p) {}

  /// Project an arbitrary pose. Caller must hold graph.lockShared().
  Result project(
    const geometry_msgs::msg::PoseStamped & pose,
    const TopoGraph & graph) const;

  /// Project requiring a NavPoint match (for goals).
  /// Returns the NavPoint id, or nullopt if no NavPoint within node_threshold.
  /// Caller must hold graph.lockShared().
  std::optional<std::string> projectToNode(
    const geometry_msgs::msg::PoseStamped & pose,
    const TopoGraph & graph) const;

  /// Vertical projection of (px,py) onto segment (x1,y1)-(x2,y2).
  /// `on_segment` is true when projection lies within the segment;
  /// otherwise the nearest endpoint is returned.
  static std::pair<double, double> verticalProject(
    double px, double py,
    double x1, double y1, double x2, double y2,
    bool & on_segment);

private:
  Params p_;
};

}  // namespace nav2_route_planner

#endif  // NAV2_ROUTE_PLANNER__START_GOAL_PROJECTOR_HPP_
