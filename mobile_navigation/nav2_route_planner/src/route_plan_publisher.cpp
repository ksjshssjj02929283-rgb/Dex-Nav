// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include "nav2_route_planner/route_plan_publisher.hpp"

namespace nav2_route_planner
{

RoutePlanPublisher::RoutePlanPublisher(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
  const std::string & topic_name)
: parent_(std::move(parent)),
  topic_name_(topic_name)
{
  auto node = parent_.lock();
  if (!node) {return;}
  // QoS: reliable + transient_local so subscribers that come up after the
  // first plan still receive it.
  rclcpp::QoS qos(1);
  qos.reliable();
  qos.transient_local();
  pub_ = node->create_publisher<nav_interfaces::msg::RoutePlan>(topic_name_, qos);
}

void RoutePlanPublisher::activate()
{
  if (pub_) {pub_->on_activate();}
}

void RoutePlanPublisher::deactivate()
{
  if (pub_) {pub_->on_deactivate();}
}

void RoutePlanPublisher::publish(const nav_interfaces::msg::RoutePlan & plan)
{
  if (pub_ && pub_->is_activated()) {
    pub_->publish(plan);
  }
}

}  // namespace nav2_route_planner
