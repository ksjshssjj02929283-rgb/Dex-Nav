// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_ROUTE_PLANNER__INTELLIGENT_ROUTER_HPP_
#define NAV2_ROUTE_PLANNER__INTELLIGENT_ROUTER_HPP_

#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_route_planner/astar_searcher.hpp"
#include "nav2_route_planner/costmap_aware_layer.hpp"
#include "nav2_route_planner/start_goal_projector.hpp"
#include "nav2_route_planner/topo_graph.hpp"

namespace nav2_route_planner
{

/// "Should we go via the road graph, or just Theta\* directly?"
///
/// Compares the geometric length of two candidate plans:
///   • DIRECT  — a single Theta\* run from `start` straight to `goal`
///   • ROUTE   — start ↦ projection ↦ graph A\* ↦ projection ↦ goal
///
/// Picks DIRECT when the straight-line plan is geometrically shorter
/// (modulo a configurable `threshold_ratio` to bias toward staying on
/// the graph). Falls back to ROUTE if Theta\* fails (start/goal inside
/// lethal, no free-space path); falls back to DIRECT if the graph A\*
/// fails. The returned Decision carries the dense Theta\* path when
/// DIRECT is chosen so the caller can skip its own re-plan.
class IntelligentRouter
{
public:
  enum class Strategy {
    ROUTE_VIA_GRAPH,    // Go through the road graph (start/goal Theta\* approach segs handled by the caller)
    DIRECT_THETA_STAR   // Single Theta\* shot from start to goal, no graph traversal
  };

  struct Decision
  {
    Strategy strategy{Strategy::ROUTE_VIA_GRAPH};
    bool direct_feasible{false};         // True if Theta\* from start to goal succeeded
    double direct_distance{0.0};         // Cumulative geometric length of the direct Theta\* path (m)
    double route_distance{0.0};          // Estimated geometric length of the graph route (m)
    /// Cached Theta\* result for DIRECT mode. Caller can emit it as the
    /// final plan without re-running Theta\*. Empty for ROUTE_VIA_GRAPH.
    std::vector<geometry_msgs::msg::PoseStamped> direct_path;
    /// Human-readable reason — written to logs at INFO level so the
    /// dev can replay decisions from the logs alone.
    std::string reason;
  };

  struct Params
  {
    /// Master switch. False disables the comparison entirely — every
    /// plan goes via the graph. Used for A/B testing and as a safety
    /// kill-switch. Backwards-compatible default behaviour matches
    /// pre-2026-05 routing (always graph).
    bool enable{false};
    /// Bias toward staying on the graph. DIRECT wins only when
    /// `direct_distance * graph_preference_ratio < route_distance`.
    /// Examples:
    ///   1.0  — pure geometric race
    ///   1.3  — graph wins ties up to 30% longer (stay on graph)
    ///   0.8  — must beat graph by 20% to take the shortcut
    /// Default 1.0 (honest race) matches the user's "if direct < route
    /// then direct" spec.
    double graph_preference_ratio{1.0};
    /// If the graph route is shorter than this many metres, we always
    /// pick DIRECT — the graph plan would just be a few sample points
    /// detouring through a junction next to the goal. Default 0.0
    /// (disabled) keeps the pure ratio comparison in charge.
    double min_graph_route_m{0.0};
  };

  explicit IntelligentRouter(Params p) : p_(p) {}
  void updateParams(const Params & p) {p_ = p;}
  const Params & params() const {return p_;}

  /// Decide between routing strategies.
  ///
  /// Pre-conditions:
  ///   • `graph_search.success` may be false — the router treats that as
  ///     "graph route unavailable" and falls back to DIRECT (if feasible).
  ///   • `start_proj` / `goal_proj` should be UNLIMITED projections so
  ///     `route_distance` includes the approach legs honestly.
  ///   • Caller must hold `graph.lockShared()` for the duration of the
  ///     call (edge / node lookups happen here).
  ///
  /// The function will invoke `ca_layer.planWithThetaStar(start, goal)`
  /// at most once — failures are cached in the returned Decision.
  Decision decide(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const StartGoalProjector::Result & start_proj,
    const StartGoalProjector::Result & goal_proj,
    const AStarSearcher::Result & graph_search,
    const TopoGraph & graph,
    CostmapAwareLayer & ca_layer) const;

  /// Compute the geometric length of a dense pose sequence (sum of
  /// consecutive segment lengths). Exposed for tests; declared static so
  /// callers don't need a router instance.
  static double pathLength(
    const std::vector<geometry_msgs::msg::PoseStamped> & poses);

  /// Compute the geometric route length implied by a successful A\*
  /// search result PLUS the start / goal approach legs implied by the
  /// projection results. Used internally by `decide`, exposed for tests.
  static double estimateRouteDistance(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const StartGoalProjector::Result & start_proj,
    const StartGoalProjector::Result & goal_proj,
    const AStarSearcher::Result & graph_search,
    const TopoGraph & graph);

private:
  Params p_;
};

}  // namespace nav2_route_planner

#endif  // NAV2_ROUTE_PLANNER__INTELLIGENT_ROUTER_HPP_
