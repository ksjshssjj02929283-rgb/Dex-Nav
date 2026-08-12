// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>

#include "action_msgs/msg/goal_status_array.hpp"

#include "mobile_bag_recorder/trigger_sources.hpp"

namespace mobile_bag_recorder
{

namespace
{

/// 已处理 goal 的记忆上限。终止过的 goal 没有保留价值，到顶清空即可，不必做 LRU
constexpr size_t kHandledKeysCap = 256;

/// 导航任务终态：ABORTED / CANCELED
class NavStatusSource : public TriggerSource
{
public:
  std::string name() const override {return "nav_status";}

  bool isEnabled(const TriggerConfig & config) const override
  {
    return config.on_nav_aborted || config.on_nav_canceled;
  }

  void setup(const TriggerContext & context) override
  {
    context_ = context;
    sub_ = context.node->create_subscription<action_msgs::msg::GoalStatusArray>(
      context.config.nav_status_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {onStatus(msg);});

    RCLCPP_INFO(
      context.node->get_logger(), "Trigger source [nav_status]: %s (aborted=%s, canceled=%s)",
      context.config.nav_status_topic.c_str(),
      context.config.on_nav_aborted ? "on" : "off",
      context.config.on_nav_canceled ? "on" : "off");
  }

private:
  static std::string uuidToString(const std::array<uint8_t, 16> & uuid)
  {
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (const auto byte : uuid) {
      oss << std::setw(2) << static_cast<int>(byte);
    }
    return oss.str();
  }

  bool markHandled(const std::string & key)
  {
    if (handled_keys_.count(key) > 0) {
      return false;
    }
    if (handled_keys_.size() >= kHandledKeysCap) {
      handled_keys_.clear();
    }
    handled_keys_.insert(key);
    return true;
  }

  void onStatus(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
  {
    for (const auto & status : msg->status_list) {
      const bool aborted = context_.config.on_nav_aborted &&
        status.status == action_msgs::msg::GoalStatus::STATUS_ABORTED;
      const bool canceled = context_.config.on_nav_canceled &&
        status.status == action_msgs::msg::GoalStatus::STATUS_CANCELED;
      if (!aborted && !canceled) {
        continue;
      }
      // GoalStatusArray 周期重发全量列表，同一 goal 的同一状态只认第一次
      const std::string key =
        uuidToString(status.goal_info.goal_id.uuid) + ":" + std::to_string(status.status);
      if (!markHandled(key)) {
        continue;
      }
      context_.fire(aborted ? "nav_aborted" : "nav_canceled");
    }
  }

  TriggerContext context_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr sub_;
  std::unordered_set<std::string> handled_keys_;
};

}  // namespace

std::unique_ptr<TriggerSource> makeNavStatusSource()
{
  return std::make_unique<NavStatusSource>();
}

}  // namespace mobile_bag_recorder
