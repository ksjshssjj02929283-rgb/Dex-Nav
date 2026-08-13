// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include "mobile_bag_recorder/topic_subscriber_set.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rosbag2_transport/qos.hpp"

namespace mobile_bag_recorder
{

TopicSubscriberSet::TopicSubscriberSet(
  rclcpp::Node * node, std::chrono::milliseconds poll_interval,
  std::unordered_map<std::string, int64_t> throttle_intervals_ns)
: node_(node), poll_interval_(poll_interval),
  throttle_intervals_ns_(std::move(throttle_intervals_ns))
{
}

void TopicSubscriberSet::addSink(const std::shared_ptr<RecordingSink> & sink)
{
  sinks_.push_back(sink);
}

void TopicSubscriberSet::start()
{
  pollOnce();
  poll_timer_ = node_->create_wall_timer(poll_interval_, [this]() {pollOnce();});
}

void TopicSubscriberSet::stop()
{
  if (poll_timer_) {
    poll_timer_->cancel();
    poll_timer_.reset();
  }
  subscriptions_.clear();
  topic_sinks_.clear();
}

size_t TopicSubscriberSet::subscribedCount() const
{
  return subscriptions_.size();
}

std::vector<std::string> TopicSubscriberSet::pendingTopics(
  const std::vector<std::string> & expected) const
{
  std::vector<std::string> pending;
  for (const auto & topic : expected) {
    if (subscriptions_.count(topic) == 0) {
      pending.push_back(topic);
    }
  }
  return pending;
}

void TopicSubscriberSet::pollOnce()
{
  const auto available = node_->get_topic_names_and_types();
  for (const auto & [topic, types] : available) {
    if (subscriptions_.count(topic) > 0 || types.empty()) {
      continue;
    }
    bool wanted = false;
    for (const auto & sink : sinks_) {
      if (sink->wantsTopic(topic)) {
        wanted = true;
        break;
      }
    }
    if (!wanted) {
      continue;
    }
    if (types.size() > 1 && warned_topics_.insert(topic).second) {
      RCLCPP_WARN(
        node_->get_logger(), "Topic %s has multiple types, recording it as the first one: %s",
        topic.c_str(), types.front().c_str());
    }
    subscribeTopic(topic, types.front());
  }
}

void TopicSubscriberSet::subscribeTopic(const std::string & topic, const std::string & type)
{
  // QoS 必须按发布端的实际 offer 适配：/map 是 transient_local、部分雷达话题是
  // best_effort，写死 QoS 常量会导致静默收不到数据。
  const auto endpoints = node_->get_publishers_info_by_topic(topic);
  const auto qos = rosbag2_transport::Rosbag2QoS::adapt_request_to_offers(topic, endpoints);

  rosbag2_storage::TopicMetadata metadata;
  metadata.name = topic;
  metadata.type = type;
  metadata.serialization_format = "cdr";
  metadata.offered_qos_profiles = "";

  std::vector<std::shared_ptr<RecordingSink>> interested;
  for (const auto & sink : sinks_) {
    if (sink->wantsTopic(topic)) {
      interested.push_back(sink);
    }
  }

  // 限速间隔在这里查一次并捕进回调，回调里不再查表
  int64_t min_interval_ns = 0;
  const auto throttle_it = throttle_intervals_ns_.find(topic);
  if (throttle_it != throttle_intervals_ns_.end()) {
    min_interval_ns = throttle_it->second;
  }
  // 上一条进入常态流的消息时间戳。回调都在同一条 executor 线程上串行执行，
  // 无需加锁；用 shared_ptr 让状态与订阅同生命周期。
  // 单独用 has_recorded 而不是拿 0 当哨兵：use_sim_time 且 /clock 尚未发布时
  // node->now() 返回 0，用 0 作哨兵会让每条消息都被当成"第一条"而放行，
  // 限速在仿真环境下静默失效
  auto last_recorded_ns = std::make_shared<int64_t>(0);
  auto has_recorded = std::make_shared<bool>(false);

  // 该话题是否有"不遵守限速"的 sink（事件流）。订阅时算一次，
  // 没有的话被限速的消息可以直接早退，连消息对象都不用构造
  bool has_throttle_exempt = false;
  for (const auto & sink : interested) {
    if (!sink->respectsThrottle()) {
      has_throttle_exempt = true;
      break;
    }
  }

  rclcpp::GenericSubscription::SharedPtr subscription;
  try {
    subscription = node_->create_generic_subscription(
      topic, type, qos,
      [this, topic, min_interval_ns, last_recorded_ns, has_recorded, has_throttle_exempt](
        std::shared_ptr<rclcpp::SerializedMessage> serialized) {
        const int64_t now_ns = node_->now().nanoseconds();

        // throttled 只决定"要不要进常态流"，事件流一律收全量
        bool throttled = false;
        if (min_interval_ns > 0) {
          // 第一条永远放行：/map 这类 transient_local 话题可能一辈子只发一次，
          // 丢掉它会让整个 bag 里没有地图
          if (*has_recorded && now_ns - *last_recorded_ns < min_interval_ns) {
            throttled = true;
            ++throttled_count_;
          } else {
            *last_recorded_ns = now_ns;
            *has_recorded = true;
          }
        }
        if (throttled && !has_throttle_exempt) {
          return;
        }

        auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        bag_message->topic_name = topic;
        bag_message->time_stamp = now_ns;
        // 别名构造：各 sink 共享同一份序列化数据，由 shared_ptr 保证生命周期，
        // 不做任何拷贝
        bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
          serialized,
          const_cast<rcutils_uint8_array_t *>(&serialized->get_rcl_serialized_message()));

        auto it = topic_sinks_.find(topic);
        if (it == topic_sinks_.end()) {
          return;
        }
        for (const auto & sink : it->second) {
          if (throttled && sink->respectsThrottle()) {
            continue;
          }
          sink->onMessage(bag_message);
        }
      });
  } catch (const std::exception & e) {
    if (warned_topics_.insert(topic).second) {
      RCLCPP_WARN(
        node_->get_logger(), "Failed to subscribe to %s (%s), skipped: %s",
        topic.c_str(), type.c_str(), e.what());
    }
    return;
  }

  for (const auto & sink : interested) {
    sink->onTopicDiscovered(metadata);
  }
  topic_sinks_[topic] = std::move(interested);
  subscriptions_[topic] = subscription;

  if (min_interval_ns > 0) {
    RCLCPP_INFO(
      node_->get_logger(), "Recording topic %s (%s) throttled to %.1f Hz",
      topic.c_str(), type.c_str(), 1e9 / static_cast<double>(min_interval_ns));
  } else {
    RCLCPP_INFO(node_->get_logger(), "Recording topic %s (%s)", topic.c_str(), type.c_str());
  }
}

}  // namespace mobile_bag_recorder
