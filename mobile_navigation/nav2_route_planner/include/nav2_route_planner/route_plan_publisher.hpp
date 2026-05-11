// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_ROUTE_PLANNER__ROUTE_PLAN_PUBLISHER_HPP_
#define NAV2_ROUTE_PLANNER__ROUTE_PLAN_PUBLISHER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "nav_interfaces/msg/route_plan.hpp"

namespace nav2_route_planner
{

/// Publishes the per-segment RoutePlan in parallel to `nav_msgs/Path`.
/// Topic: `<planner_id>/route_plan` (transient_local + reliable so that
/// downstream RuntimeBinder/BehaviorTree subscribers that come up later
/// still receive the latest plan).
class RoutePlanPublisher
{
public:
  RoutePlanPublisher(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
    const std::string & topic_name);

  void activate();
  void deactivate();
  void publish(const nav_interfaces::msg::RoutePlan & plan);

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::string topic_name_;
  rclcpp_lifecycle::LifecyclePublisher<nav_interfaces::msg::RoutePlan>::SharedPtr pub_;
};

}  // namespace nav2_route_planner

#endif  // NAV2_ROUTE_PLANNER__ROUTE_PLAN_PUBLISHER_HPP_
