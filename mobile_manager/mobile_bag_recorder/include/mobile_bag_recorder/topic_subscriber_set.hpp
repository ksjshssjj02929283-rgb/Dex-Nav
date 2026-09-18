// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#ifndef MOBILE_BAG_RECORDER__TOPIC_SUBSCRIBER_SET_HPP_
#define MOBILE_BAG_RECORDER__TOPIC_SUBSCRIBER_SET_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
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

  /// 该 sink 是否遵守限速。
  /// 常态流遵守——它 7x24 写盘，是磁盘与 CPU 的大头；
  /// 事件流不遵守——触发意味着要排查故障，取证需要原始分辨率，而它只在触发后
  /// 几十秒内写盘，代价仅是环形缓冲多占一点内存。
  virtual bool respectsThrottle() const {return true;}
};

/// 按配置清单订阅话题，把消息分发给各 sink。
/// 清单里尚未发布的话题不会导致失败——发现线程会持续轮询，话题上线后自动补订阅。
class TopicSubscriberSet
{
public:
  /// throttle_intervals_ns：topic -> 两条记录之间的最小间隔（纳秒），0 或缺省表示不限速。
  /// 限速只对 respectsThrottle() 为真的 sink 生效（常态流）；事件流照收全量，
  /// 因此环形缓冲里始终是原始分辨率的数据。
  /// queue_depth：订阅队列深度。默认 10 对 100 Hz 话题只有 100 ms 缓冲，
  /// executor 稍有抖动就整批丢消息；录包器是纯消费者，加大只占内存。
  TopicSubscriberSet(
    rclcpp::Node * node, std::chrono::milliseconds poll_interval,
    std::unordered_map<std::string, int64_t> throttle_intervals_ns = {},
    int queue_depth = 200);

  /// 注册 sink。必须在 start() 之前调用
  void addSink(const std::shared_ptr<RecordingSink> & sink);

  /// 启动话题发现轮询（立即执行一次，之后按 poll_interval 周期）
  void start();

  void stop();

  /// 已订阅的话题数
  size_t subscribedCount() const;

  /// 配置清单里仍未出现的话题（用于启动后告警）
  std::vector<std::string> pendingTopics(const std::vector<std::string> & expected) const;

  /// 因限速而未进入常态流的消息总数（事件流仍然收到了这些消息）
  uint64_t throttledCount() const {return throttled_count_.load(std::memory_order_relaxed);}

  /// 因系统时钟回拨而被单调钳位的消息数。
  /// 正常应恒为 0；非 0 说明 CLOCK_REALTIME 被向后调整过（NTP/PTP 打架的典型症状），
  /// 此时 bag 的时间轴虽被钳住不至于损坏，但绝对时间已不可信
  uint64_t clockRegressionCount() const
  {
    return clock_regression_count_.load(std::memory_order_relaxed);
  }

private:
  void pollOnce();
  void subscribeTopic(const std::string & topic, const std::string & type);

  /// 取录制时间戳，并保证全局单调递增。
  /// 系统时钟被回拨时 node->now() 会倒退，写进 bag 会得到乱序时间轴——mcap 要求
  /// 块内 log_time 非递减，sqlite 侧则会让 rqt_bag/回放的时间轴错乱。这里钳到
  /// 上一条 +1 ns：时钟回拨期间时间轴"停住"而不"倒退"，且计数可观测
  int64_t nextStamp();

  rclcpp::Node * node_;
  std::chrono::milliseconds poll_interval_;
  rclcpp::TimerBase::SharedPtr poll_timer_;

  std::vector<std::shared_ptr<RecordingSink>> sinks_;
  std::unordered_map<std::string, rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  /// topic -> 该订阅独占的回调组。每话题一个 MutuallyExclusive 组：
  /// 同一话题的回调仍然串行（限速状态因此无需加锁），话题之间可在多线程 executor 上
  /// 并行，且都不会被重活定时器挡住
  std::unordered_map<std::string, rclcpp::CallbackGroup::SharedPtr> callback_groups_;
  /// 类型不可用等原因跳过的话题，只告警一次
  std::unordered_set<std::string> warned_topics_;
  /// topic -> 最小录制间隔（纳秒）。订阅时查一次并捕进回调，不在每条消息上查表
  std::unordered_map<std::string, int64_t> throttle_intervals_ns_;
  int queue_depth_;
  /// 多线程 executor 下由多个话题回调并发自增，必须是原子的
  std::atomic<uint64_t> throttled_count_{0};
  std::atomic<uint64_t> clock_regression_count_{0};
  /// 上一条写入 bag 的时间戳，用于全局单调钳位
  std::atomic<int64_t> last_stamp_ns_{0};
  /// 派发序锁：保证「取时间戳」与「交给各 sink」是一个原子步骤。
  /// 多线程 executor 下，两路话题回调并发取戳后写入顺序可能交错，bag 里会出现
  /// 几十微秒的时间戳逆序——sqlite3 读时按 timestamp 排序无碍，但 mcap 的分块索引
  /// 依赖 log_time 有序，故不冒这个险。锁内只做「打戳 + 入 cache」，是微秒级操作；
  /// 真正的重活（开关 bag、倒 pre 窗口、回收）都在 JobQueue 线程上，不碰这把锁
  std::mutex dispatch_mutex_;
};

}  // namespace mobile_bag_recorder

#endif  // MOBILE_BAG_RECORDER__TOPIC_SUBSCRIBER_SET_HPP_
