// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include "mobile_bag_recorder/trigger_monitor.hpp"

#include <memory>
#include <string>
#include <utility>

#include "mobile_bag_recorder/trigger_sources.hpp"

namespace mobile_bag_recorder
{

TriggerMonitor::TriggerMonitor(
  rclcpp::Node * node, const TriggerConfig & config, TriggerCallback callback)
{
  for (auto & source : createBuiltinTriggerSources()) {
    if (!source->isEnabled(config)) {
      continue;
    }

    const auto unavailable = source->unavailableReason();
    if (!unavailable.empty()) {
      // 启用了但用不了必须说清楚，否则表现为"配了却没反应"
      RCLCPP_WARN(
        node->get_logger(), "Trigger source [%s] is enabled but unavailable: %s",
        source->name().c_str(), unavailable.c_str());
      continue;
    }

    source->setup(TriggerContext{node, config, callback});
    sources_.push_back(std::move(source));
  }

  RCLCPP_INFO(node->get_logger(), "%zu automatic trigger source(s) active", sources_.size());
}

}  // namespace mobile_bag_recorder
