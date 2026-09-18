// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#ifndef MOBILE_BAG_RECORDER__EVENT_RECORDER_HPP_
#define MOBILE_BAG_RECORDER__EVENT_RECORDER_HPP_

#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"

#include "mobile_bag_recorder/job_queue.hpp"
#include "mobile_bag_recorder/log_collector.hpp"
#include "mobile_bag_recorder/recorder_config.hpp"
#include "mobile_bag_recorder/retention_policy.hpp"
#include "mobile_bag_recorder/time_ring_buffer.hpp"
#include "mobile_bag_recorder/topic_subscriber_set.hpp"

namespace mobile_bag_recorder
{

/// 一次触发请求的结果
struct TriggerResult
{
  bool success{false};
  std::string message;
  std::string bag_path;
};

/// 触发式事件录制：平时把消息压进按秒裁剪的环形缓冲，触发时把缓冲倒进一个独立 bag，
/// 再续录 post_seconds 后关闭。产出的每个事件 bag 覆盖「触发点前 N 秒 + 后 M 秒」。
class EventRecorder : public RecordingSink
{
public:
  EventRecorder(
    rclcpp::Node * node, const RecorderConfig & config, const std::string & output_root);

  ~EventRecorder() override;

  /// 触发一次事件录制。
  /// pre/post 传 <=0 时沿用配置值；pre 覆盖只影响本次倒出的历史长度，不改变缓冲窗口本身
  TriggerResult trigger(
    const std::string & reason, const std::string & source = "",
    int pre_override = 0, int post_override = 0);

  /// 提前结束当前事件录制（正常由 post 定时器调用）
  void finish();

  bool isActive() const;
  int eventCount() const;

  /// 开 bag 期间因暂存队列满而丢弃的实时消息数。正常应为 0；
  /// 非 0 说明开 bag 太慢（磁盘忙）或 event.staging_limit 太小
  uint64_t stagingDropped() const;
  const TimeRingBuffer & buffer() const {return *buffer_;}

  bool wantsTopic(const std::string & topic) const override;
  void onTopicDiscovered(const rosbag2_storage::TopicMetadata & metadata) override;
  void onMessage(const rosbag2_storage::SerializedBagMessageSharedPtr & message) override;

  /// 事件录制不受限速影响：触发即意味着要排查故障，取证需要原始分辨率
  bool respectsThrottle() const override {return false;}

private:
  std::string makeEventUri(const std::string & reason) const;
  static std::string sanitize(const std::string & text);

  rclcpp::Node * node_;
  EventConfig config_;
  std::string storage_id_;
  int64_t writer_cache_size_;
  std::string event_root_;

  std::unordered_set<std::string> wanted_topics_;
  std::unordered_map<std::string, rosbag2_storage::TopicMetadata> known_topics_;

  std::unique_ptr<TimeRingBuffer> buffer_;
  std::unique_ptr<RetentionPolicy> retention_;
  /// 触发时一并留存的系统日志快照；log_capture.enabled=false 时为空指针
  std::unique_ptr<LogCollector> log_collector_;

  mutable std::mutex mutex_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::string current_uri_;
  bool active_{false};
  /// writer 正在后台打开中：此时 active_ 已为真但 writer_ 还是空，
  /// 实时消息先进 staging_，由开包任务倒进去后再放行，保证 bag 内时间连续
  bool opening_{false};
  std::deque<rosbag2_storage::SerializedBagMessageSharedPtr> staging_;
  /// 暂存上限（条）。开包期间磁盘若异常缓慢，宁可丢实时消息也不能把内存吃穿
  size_t staging_limit_{100000};
  uint64_t staging_dropped_{0};
  /// 每次触发 +1。开包任务完成时比对，避免"开包还没完成就被 finish"时
  /// 把已作废的 writer 装回去
  uint64_t session_id_{0};
  int event_count_{0};
  rclcpp::Time last_trigger_time_;
  bool has_triggered_{false};
  /// 本次触发的墙上时间与原因，留给日志快照定位窗口。用 system_clock 而不是
  /// node->now()：journalctl 只认墙上时间，而 ROS 时间可能是 sim time
  std::chrono::system_clock::time_point trigger_wall_time_;
  std::string current_reason_;

  rclcpp::TimerBase::SharedPtr post_timer_;

  /// 开 bag（倒 pre 窗口，实测几十 MB）、关 bag（刷 cache 落盘）、日志快照、回收旧包
  /// 全在这条线程上跑。放在最后声明：析构时最先销毁，join 掉工作线程后其它成员才析构
  std::unique_ptr<JobQueue> jobs_;
};

}  // namespace mobile_bag_recorder

#endif  // MOBILE_BAG_RECORDER__EVENT_RECORDER_HPP_
