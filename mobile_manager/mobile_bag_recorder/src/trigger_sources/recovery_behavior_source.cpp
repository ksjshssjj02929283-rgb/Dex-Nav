// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <memory>
#include <string>

#ifdef MOBILE_BAG_RECORDER_HAS_BT_LOG
#include "nav2_msgs/msg/behavior_tree_log.hpp"
#endif

#include "mobile_bag_recorder/trigger_sources.hpp"

namespace mobile_bag_recorder
{

namespace
{

/// 行为树进入恢复类节点。需要 nav2_msgs：缺失时本源仍会被登记，
/// 但 unavailableReason() 会说明原因，由调度器打印告警而不是静默失效
class RecoveryBehaviorSource : public TriggerSource
{
public:
  std::string name() const override {return "recovery_behavior";}

  bool isEnabled(const TriggerConfig & config) const override
  {
    return config.on_recovery_behavior;
  }

  std::string unavailableReason() const override
  {
#ifdef MOBILE_BAG_RECORDER_HAS_BT_LOG
    return {};
#else
    return "built without nav2_msgs support";
#endif
  }

  void setup(const TriggerContext & context) override
  {
#ifdef MOBILE_BAG_RECORDER_HAS_BT_LOG
    context_ = context;
    sub_ = context.node->create_subscription<nav2_msgs::msg::BehaviorTreeLog>(
      context.config.behavior_tree_log_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this](const nav2_msgs::msg::BehaviorTreeLog::SharedPtr msg) {onLog(msg);});

    RCLCPP_INFO(
      context.node->get_logger(), "Trigger source [recovery_behavior]: %s (%zu nodes)",
      context.config.behavior_tree_log_topic.c_str(),
      context.config.recovery_node_names.size());
#else
    (void)context;
#endif
  }

private:
#ifdef MOBILE_BAG_RECORDER_HAS_BT_LOG
  void onLog(const nav2_msgs::msg::BehaviorTreeLog::SharedPtr msg)
  {
    for (const auto & event : msg->event_log) {
      if (event.current_status != "RUNNING" || event.previous_status == "RUNNING") {
        continue;
      }
      const auto & names = context_.config.recovery_node_names;
      if (std::find(names.begin(), names.end(), event.node_name) == names.end()) {
        continue;
      }
      context_.fire("recovery_" + event.node_name);
      return;
    }
  }

  TriggerContext context_;
  rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr sub_;
#endif
};

}  // namespace

std::unique_ptr<TriggerSource> makeRecoveryBehaviorSource()
{
  return std::make_unique<RecoveryBehaviorSource>();
}

}  // namespace mobile_bag_recorder
