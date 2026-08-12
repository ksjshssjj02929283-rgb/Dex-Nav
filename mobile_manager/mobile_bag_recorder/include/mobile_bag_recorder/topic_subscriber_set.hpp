// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#ifndef MOBILE_BAG_RECORDER__TOPIC_SUBSCRIBER_SET_HPP_
#define MOBILE_BAG_RECORDER__TOPIC_SUBSCRIBER_SET_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

namespace mobile_bag_recorder
{

/// 录制目的地。常态录制与事件录制各实现一份，共用同一套订阅
class RecordingSink
{
public:
  virtual ~RecordingSink() = default;

  /// 本 sink 是否关心这个话题
  virtual bool wantsTopic(const std::string & topic) const = 0;

  /// 话题首次被发现并订阅时调用一次，sink 在这里登记 topic 元数据
  virtual void onTopicDiscovered(const rosbag2_storage::TopicMetadata & metadata) = 0;

  /// 每条消息调用。同一条消息的序列化数据在各 sink 之间共享，不复制
  virtual void onMessage(const rosbag2_storage::SerializedBagMessageSharedPtr & message) = 0;
};

/// 按配置清单订阅话题，把消息分发给各 sink。
/// 清单里尚未发布的话题不会导致失败——发现线程会持续轮询，话题上线后自动补订阅。
class TopicSubscriberSet
{
public:
  TopicSubscriberSet(rclcpp::Node * node, std::chrono::milliseconds poll_interval);

  /// 注册 sink。必须在 start() 之前调用
  void addSink(const std::shared_ptr<RecordingSink> & sink);

  /// 启动话题发现轮询（立即执行一次，之后按 poll_interval 周期）
  void start();

  void stop();

  /// 已订阅的话题数
  size_t subscribedCount() const;

  /// 配置清单里仍未出现的话题（用于启动后告警）
  std::vector<std::string> pendingTopics(const std::vector<std::string> & expected) const;

private:
  void pollOnce();
  void subscribeTopic(const std::string & topic, const std::string & type);

  rclcpp::Node * node_;
  std::chrono::milliseconds poll_interval_;
  rclcpp::TimerBase::SharedPtr poll_timer_;

  std::vector<std::shared_ptr<RecordingSink>> sinks_;
  std::unordered_map<std::string, rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  /// topic -> 关心它的 sink，订阅时算一次，避免每条消息都遍历全部 sink
  std::unordered_map<std::string, std::vector<std::shared_ptr<RecordingSink>>> topic_sinks_;
  /// 类型不可用等原因跳过的话题，只告警一次
  std::unordered_set<std::string> warned_topics_;
};

}  // namespace mobile_bag_recorder

#endif  // MOBILE_BAG_RECORDER__TOPIC_SUBSCRIBER_SET_HPP_
