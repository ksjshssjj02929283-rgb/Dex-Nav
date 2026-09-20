// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#ifndef MOBILE_BAG_RECORDER__TRIGGER_MONITOR_HPP_
#define MOBILE_BAG_RECORDER__TRIGGER_MONITOR_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "mobile_bag_recorder/trigger_source.hpp"

namespace mobile_bag_recorder
{

/// 触发源调度器：按配置启用内建触发源，把它们的上报汇聚到同一个回调。
/// 本身不含任何判定逻辑——判定属于各 TriggerSource，新增源不改这里。
class TriggerMonitor
{
public:
  TriggerMonitor(rclcpp::Node * node, const TriggerConfig & config, TriggerCallback callback);

  size_t activeSourceCount() const {return sources_.size();}

private:
  std::vector<std::unique_ptr<TriggerSource>> sources_;
};

}  // namespace mobile_bag_recorder

#endif  // MOBILE_BAG_RECORDER__TRIGGER_MONITOR_HPP_
