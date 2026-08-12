// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#ifndef MOBILE_BAG_RECORDER__MOBILE_BAG_RECORDER_NODE_HPP_
#define MOBILE_BAG_RECORDER__MOBILE_BAG_RECORDER_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "mobile_bag_recorder/srv/trigger_record.hpp"

#include "mobile_bag_recorder/continuous_recorder.hpp"
#include "mobile_bag_recorder/event_recorder.hpp"
#include "mobile_bag_recorder/recorder_config.hpp"
#include "mobile_bag_recorder/topic_subscriber_set.hpp"
#include "mobile_bag_recorder/trigger_monitor.hpp"

namespace mobile_bag_recorder
{

/// 导航数据录包节点。
///
/// 常态分段录制与（后续）触发式事件录制共用同一套话题订阅，一条消息的序列化数据
/// 在各录制目的地之间共享，不重复订阅、不重复拷贝。
class MobileBagRecorderNode : public rclcpp::Node
{
public:
  explicit MobileBagRecorderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MobileBagRecorderNode() override;

  /// 构造后调用，完成目录准备、sink 装配与订阅启动
  void init();

private:
  void reportPendingTopics();

  RecorderConfig config_;
  std::string output_root_;

  std::shared_ptr<ContinuousRecorder> continuous_;
  std::shared_ptr<EventRecorder> event_;
  std::unique_ptr<TopicSubscriberSet> subscribers_;
  std::unique_ptr<TriggerMonitor> triggers_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_continuous_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_continuous_srv_;
  rclcpp::Service<mobile_bag_recorder::srv::TriggerRecord>::SharedPtr trigger_srv_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trigger_event_sub_;
  rclcpp::TimerBase::SharedPtr pending_report_timer_;
};

}  // namespace mobile_bag_recorder

#endif  // MOBILE_BAG_RECORDER__MOBILE_BAG_RECORDER_NODE_HPP_
