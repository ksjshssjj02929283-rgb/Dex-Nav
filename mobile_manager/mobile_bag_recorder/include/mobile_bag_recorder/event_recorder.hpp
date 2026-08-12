// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#ifndef MOBILE_BAG_RECORDER__EVENT_RECORDER_HPP_
#define MOBILE_BAG_RECORDER__EVENT_RECORDER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"

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

  mutable std::mutex mutex_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::string current_uri_;
  bool active_{false};
  int event_count_{0};
  rclcpp::Time last_trigger_time_;
  bool has_triggered_{false};

  rclcpp::TimerBase::SharedPtr post_timer_;
};

}  // namespace mobile_bag_recorder

#endif  // MOBILE_BAG_RECORDER__EVENT_RECORDER_HPP_
