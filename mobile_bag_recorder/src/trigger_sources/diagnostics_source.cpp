// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "mobile_bag_recorder/trigger_sources.hpp"

namespace mobile_bag_recorder
{

namespace
{

/// 规划/控制诊断出现 ERROR
class DiagnosticsSource : public TriggerSource
{
public:
  std::string name() const override {return "diagnostics";}

  bool isEnabled(const TriggerConfig & config) const override
  {
    return config.on_diagnostics_error;
  }

  void setup(const TriggerContext & context) override
  {
    context_ = context;
    sub_ = context.node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      context.config.diagnostics_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {onDiagnostics(msg);});

    RCLCPP_INFO(
      context.node->get_logger(), "Trigger source [diagnostics]: %s (level >= ERROR)",
      context.config.diagnostics_topic.c_str());
  }

private:
  void onDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
    bool has_error = false;
    std::string source;
    for (const auto & status : msg->status) {
      if (status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
        has_error = true;
        source = status.name;
        break;
      }
    }
    // 上升沿：持续 ERROR 期间不重复上报，恢复正常后才重新武装
    if (has_error && !in_error_) {
      context_.fire("diagnostics_" + (source.empty() ? std::string("error") : source));
    }
    in_error_ = has_error;
  }

  TriggerContext context_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_;
  bool in_error_{false};
};

}  // namespace

std::unique_ptr<TriggerSource> makeDiagnosticsSource()
{
  return std::make_unique<DiagnosticsSource>();
}

}  // namespace mobile_bag_recorder
