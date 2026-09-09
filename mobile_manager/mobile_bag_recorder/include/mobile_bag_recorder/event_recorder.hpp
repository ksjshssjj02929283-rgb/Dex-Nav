// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#ifndef MOBILE_BAG_RECORDER__EVENT_RECORDER_HPP_
#define MOBILE_BAG_RECORDER__EVENT_RECORDER_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"

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
  int event_count_{0};
  rclcpp::Time last_trigger_time_;
  bool has_triggered_{false};
  /// 本次触发的墙上时间与原因，留给日志快照定位窗口。用 system_clock 而不是
  /// node->now()：journalctl 只认墙上时间，而 ROS 时间可能是 sim time
  std::chrono::system_clock::time_point trigger_wall_time_;
  std::string current_reason_;

  rclcpp::TimerBase::SharedPtr post_timer_;
};

}  // namespace mobile_bag_recorder

#endif  // MOBILE_BAG_RECORDER__EVENT_RECORDER_HPP_
