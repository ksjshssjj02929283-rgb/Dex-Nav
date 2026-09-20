// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "mobile_bag_recorder/trigger_sources.hpp"

namespace mobile_bag_recorder
{

namespace
{

/// 终点误差超阈值。读的是 controller_server 发布的 distance_error 字段
class GoalErrorSource : public TriggerSource
{
public:
  std::string name() const override {return "goal_error";}

  bool isEnabled(const TriggerConfig & config) const override
  {
    return config.goal_error_threshold_m > 0.0;
  }

  void setup(const TriggerContext & context) override
  {
    context_ = context;
    sub_ = context.node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      context.config.goal_error_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {onGoalError(msg);});

    RCLCPP_INFO(
      context.node->get_logger(), "Trigger source [goal_error]: %s (distance_error > %.3f m)",
      context.config.goal_error_topic.c_str(), context.config.goal_error_threshold_m);
  }

private:
  void onGoalError(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
    double worst = -1.0;
    for (const auto & status : msg->status) {
      for (const auto & item : status.values) {
        if (item.key != "distance_error") {
          continue;
        }
        try {
          worst = std::max(worst, std::stod(item.value));
        } catch (const std::exception &) {
          // 数值解析失败说明上游格式变了，忽略这一条而不是让节点崩掉
        }
      }
    }
    if (worst < 0.0) {
      return;
    }

    const bool exceeded = worst > context_.config.goal_error_threshold_m;
    if (exceeded && !exceeded_) {
      std::ostringstream oss;
      oss << "goal_error_" << std::fixed << std::setprecision(2) << worst << "m";
      context_.fire(oss.str());
    }
    exceeded_ = exceeded;
  }

  TriggerContext context_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_;
  bool exceeded_{false};
};

}  // namespace

std::unique_ptr<TriggerSource> makeGoalErrorSource()
{
  return std::make_unique<GoalErrorSource>();
}

}  // namespace mobile_bag_recorder
