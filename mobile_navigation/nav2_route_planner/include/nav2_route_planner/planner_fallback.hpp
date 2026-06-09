// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_ROUTE_PLANNER__PLANNER_FALLBACK_HPP_
#define NAV2_ROUTE_PLANNER__PLANNER_FALLBACK_HPP_

#include <cstdint>

namespace nav2_route_planner
{

/// Why a planner asks the server to fall back to another planner.
///
/// Extend with new values as more "cannot plan, please delegate" conditions
/// appear. The planner server treats any non-NONE value as a fallback request.
enum class FallbackReason : uint8_t
{
  NONE = 0,          ///< no fallback; the plan stands or genuinely failed
  NO_ROUTE_GRAPH,    ///< the map has no route graph (no road network built)
};

inline const char * toString(FallbackReason r)
{
  switch (r) {
    case FallbackReason::NO_ROUTE_GRAPH:
      return "NO_ROUTE_GRAPH";
    case FallbackReason::NONE:
    default:
      return "NONE";
  }
}

/// Mix-in interface for a GlobalPlanner that can request the planner server to
/// dispatch a configured fallback planner when it could not produce a plan.
///
/// Contract: after an empty createPlan() result, the server queries
/// lastFallbackReason(); if it is not NONE (and a fallback planner is
/// configured) the server retries the request with the fallback planner.
/// A planner that returns an empty path WITHOUT a fallback reason is treated
/// as a genuine failure (no fallback).
class FallbackProvider
{
public:
  virtual ~FallbackProvider() = default;

  /// Fallback reason for the most recent createPlan(); NONE = no fallback.
  virtual FallbackReason lastFallbackReason() const = 0;
};

}  // namespace nav2_route_planner

#endif  // NAV2_ROUTE_PLANNER__PLANNER_FALLBACK_HPP_
