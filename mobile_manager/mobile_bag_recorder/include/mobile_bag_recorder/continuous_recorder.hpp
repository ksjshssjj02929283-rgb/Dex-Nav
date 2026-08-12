// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#ifndef MOBILE_BAG_RECORDER__CONTINUOUS_RECORDER_HPP_
#define MOBILE_BAG_RECORDER__CONTINUOUS_RECORDER_HPP_

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
#include "mobile_bag_recorder/topic_subscriber_set.hpp"

namespace mobile_bag_recorder
{

/// 常态分段录制。
///
/// 切段自管而非依赖 rosbag2 的 max_bagfile_duration：后者切出的多个 .db3 共享同一份
/// metadata.yaml，删掉其中的旧文件会让 metadata 与实际不符、整包无法回放。自管切段让
/// 每段成为一个可独立回放、可独立删除的完整 bag，代价是切段瞬间几十毫秒的空隙。
class ContinuousRecorder : public RecordingSink
{
public:
  ContinuousRecorder(
    rclcpp::Node * node, const RecorderConfig & config, const std::string & output_root);

  ~ContinuousRecorder() override;

  /// 开第一段并启动切段定时器
  void start();

  /// 关闭当前段，停止切段
  void stop();

  bool isRecording() const;

  /// 已完成（已关闭）的段数
  int completedSegments() const;

  bool wantsTopic(const std::string & topic) const override;
  void onTopicDiscovered(const rosbag2_storage::TopicMetadata & metadata) override;
  void onMessage(const rosbag2_storage::SerializedBagMessageSharedPtr & message) override;

private:
  void openSegment();
  void closeSegment();
  void rollSegment();
  std::string makeSegmentUri() const;

  rclcpp::Node * node_;
  ContinuousConfig config_;
  std::string storage_id_;
  int64_t writer_cache_size_;
  std::string segment_root_;

  std::unordered_set<std::string> wanted_topics_;
  /// 已发现话题的元数据。每开新段都要在新 writer 上重新 create_topic
  std::unordered_map<std::string, rosbag2_storage::TopicMetadata> known_topics_;

  mutable std::mutex writer_mutex_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::string current_uri_;
  bool recording_{false};
  int completed_segments_{0};

  rclcpp::TimerBase::SharedPtr roll_timer_;
  std::unique_ptr<RetentionPolicy> retention_;
};

}  // namespace mobile_bag_recorder

#endif  // MOBILE_BAG_RECORDER__CONTINUOUS_RECORDER_HPP_
