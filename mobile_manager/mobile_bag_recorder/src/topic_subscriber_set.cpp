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
  std::unordered_map<std::string, int64_t> throttle_intervals_ns, int queue_depth)
: node_(node), poll_interval_(poll_interval),
  throttle_intervals_ns_(std::move(throttle_intervals_ns)),
  queue_depth_(queue_depth > 0 ? queue_depth : 1)
{
}

int64_t TopicSubscriberSet::nextStamp()
{
  const int64_t now_ns = node_->now().nanoseconds();
  int64_t last = last_stamp_ns_.load(std::memory_order_relaxed);
  for (;;) {
    if (now_ns > last) {
      if (last_stamp_ns_.compare_exchange_weak(
          last, now_ns, std::memory_order_relaxed, std::memory_order_relaxed))
      {
        return now_ns;
      }
      continue;
    }
    // 时钟没前进：可能是并发线程刚取过同一时刻（无害），也可能是系统时钟被回拨。
    // 只有明显倒退才计数告警，避免把同刻并发误报成时钟问题
    if (last - now_ns > 1000000) {  // 1 ms
      clock_regression_count_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "System clock went backwards by %.3f s while recording; bag timestamps are clamped "
        "to stay monotonic but absolute time is no longer trustworthy "
        "(check NTP/PTP: systemd-timesyncd and ptp4l must not both discipline the clock)",
        static_cast<double>(last - now_ns) / 1e9);
    }
    const int64_t clamped = last + 1;
    if (last_stamp_ns_.compare_exchange_weak(
        last, clamped, std::memory_order_relaxed, std::memory_order_relaxed))
    {
      return clamped;
    }
  }
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
  callback_groups_.clear();
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
  auto qos = rosbag2_transport::Rosbag2QoS::adapt_request_to_offers(topic, endpoints);
  // 只放大深度，保留适配得到的 reliability/durability。
  // rosbag2 默认深度 10：100 Hz 话题仅 100 ms 缓冲，executor 抖动一下就整批丢
  if (qos.get_rmw_qos_profile().history == RMW_QOS_POLICY_HISTORY_KEEP_LAST &&
    static_cast<int>(qos.get_rmw_qos_profile().depth) < queue_depth_)
  {
    qos.keep_last(static_cast<size_t>(queue_depth_));
  }

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
  // 上一条进入常态流的消息时间戳。本话题独占一个 MutuallyExclusive 回调组，
  // 同一话题的回调仍在同一时刻只跑一个，故无需加锁；用 shared_ptr 让状态与订阅同生命周期。
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

  // 每话题独占一个 MutuallyExclusive 回调组。
  // 这是把订阅与重活解耦的关键：多线程 executor 下，切段/触发/回收这些几百毫秒的活
  // 跑在默认组里，不会再挡住任何一路订阅；而同一话题的回调依然串行，
  // 限速状态保持无锁。
  auto group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = group;

  // ⚠️ 顺序契约：必须先让各 sink 登记话题（create_topic），再建订阅。
  // 订阅一建立就在自己的回调组里活跃；对 TRANSIENT_LOCAL（latched）话题，DDS 会把
  // 保留样本立刻投递，多线程 executor 下另一条线程可能在本函数返回前就跑 onMessage
  // → writer->write()。若此时该话题还没 create_topic，rosbag2 抛 runtime_error，
  // 回调里无人接 → std::terminate。实机（load 26）首次启动即中招，/active 由
  // cmd_vel_mux latched 发布。rosbag2 上游 RecorderImpl::subscribe_topic 同样先
  // create_topic 再订阅，理由相同。单线程 executor 下这个顺序曾"天然"成立，
  // 改多线程时被打破——这就是把架构假设埋进局部代码的代价。
  for (const auto & sink : interested) {
    sink->onTopicDiscovered(metadata);
  }

  rclcpp::GenericSubscription::SharedPtr subscription;
  try {
    subscription = node_->create_generic_subscription(
      topic, type, qos,
      // interested 按值捕获：回调里不再查 topic_sinks_ 映射表，
      // 既省一次哈希查找，也避免多线程 executor 下读写同一张表
      [this, topic, min_interval_ns, last_recorded_ns, has_recorded, has_throttle_exempt,
      interested](std::shared_ptr<rclcpp::SerializedMessage> serialized) {
        // 打戳与派发必须是一个原子步骤，否则并发的两路话题会以与时间戳相反的顺序
        // 进入 bag（实测 16–22 µs 逆序）。锁内只有打戳和入 cache，微秒级
        std::lock_guard<std::mutex> order_lock(dispatch_mutex_);

        // 用单调钳位后的时间戳，而不是裸 node->now()：系统时钟被回拨时，
        // 裸时间会让 bag 时间轴倒退，也会让限速判据 now - last 变成负数而永久限速
        const int64_t now_ns = nextStamp();

        // throttled 只决定"要不要进常态流"，事件流一律收全量
        bool throttled = false;
        if (min_interval_ns > 0) {
          // 第一条永远放行：/map 这类 transient_local 话题可能一辈子只发一次，
          // 丢掉它会让整个 bag 里没有地图
          if (*has_recorded && now_ns - *last_recorded_ns < min_interval_ns) {
            throttled = true;
            throttled_count_.fetch_add(1, std::memory_order_relaxed);
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

        for (const auto & sink : interested) {
          if (throttled && sink->respectsThrottle()) {
            continue;
          }
          // 取证工具绝不能因一次写入失败把自己杀掉（磁盘满、话题未登记、存储层异常）。
          // 订阅回调里抛出的异常 rclcpp 不接，会直接 std::terminate——实机 217 就是
          // 这么死的（exit -6，之后完全没有录包）。这里兜住：计数 + 限频 ERROR，
          // 其它话题照录。根因（登记顺序）已在 subscribeTopic 修掉，这是第二道防线
          try {
            sink->onMessage(bag_message);
          } catch (const std::exception & e) {
            const auto n = write_failures_.fetch_add(1, std::memory_order_relaxed) + 1;
            RCLCPP_ERROR_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 5000,
              "Sink write failed on %s (%lu failure(s) so far), message dropped: %s",
              topic.c_str(), static_cast<unsigned long>(n), e.what());
          }
        }
      },
      sub_options);
  } catch (const std::exception & e) {
    if (warned_topics_.insert(topic).second) {
      RCLCPP_WARN(
        node_->get_logger(), "Failed to subscribe to %s (%s), skipped: %s",
        topic.c_str(), type.c_str(), e.what());
    }
    return;
  }

  callback_groups_[topic] = group;
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
