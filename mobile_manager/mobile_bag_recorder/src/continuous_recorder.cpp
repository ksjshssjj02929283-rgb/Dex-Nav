// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include "mobile_bag_recorder/continuous_recorder.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"

namespace fs = std::filesystem;

namespace mobile_bag_recorder
{

ContinuousRecorder::ContinuousRecorder(
  rclcpp::Node * node, const RecorderConfig & config, const std::string & output_root)
: node_(node),
  config_(config.continuous),
  storage_id_(config.storage_id),
  writer_cache_size_(config.writer_cache_size),
  segment_root_(output_root + "/continuous")
{
  wanted_topics_.insert(config_.topics.begin(), config_.topics.end());

  std::error_code ec;
  fs::create_directories(segment_root_, ec);
  if (ec) {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed to create continuous recording directory: %s (%s)",
      segment_root_.c_str(), ec.message().c_str());
  }

  retention_ = std::make_unique<RetentionPolicy>(
    node_->get_logger(), segment_root_, config_.max_segments,
    static_cast<int64_t>(config_.max_total_size_mb) * 1024 * 1024);

  jobs_ = std::make_unique<JobQueue>("continuous_recorder");
}

ContinuousRecorder::~ContinuousRecorder()
{
  stop();
  // 关段与 metadata 落盘是异步的，必须等它跑完再让成员开始析构
  if (jobs_) {
    jobs_->drain();
  }
}

bool ContinuousRecorder::wantsTopic(const std::string & topic) const
{
  return wanted_topics_.count(topic) > 0;
}

void ContinuousRecorder::onTopicDiscovered(const rosbag2_storage::TopicMetadata & metadata)
{
  std::lock_guard<std::mutex> lock(writer_mutex_);
  known_topics_[metadata.name] = metadata;
  if (writer_) {
    writer_->create_topic(metadata);
  }
}

void ContinuousRecorder::onMessage(
  const rosbag2_storage::SerializedBagMessageSharedPtr & message)
{
  std::lock_guard<std::mutex> lock(writer_mutex_);
  if (!writer_ || !recording_) {
    return;
  }
  // writer_cache_size_ > 0 时 rosbag2 内部走 cache + consumer 线程，
  // write() 只是入队，不会在订阅回调里同步落盘
  writer_->write(message);
}

void ContinuousRecorder::start()
{
  if (!config_.enabled) {
    RCLCPP_INFO(
      node_->get_logger(), "Continuous recording is disabled (continuous.enabled=false)");
    return;
  }
  if (recording_) {
    return;
  }

  openSegment();
  // 重启后磁盘上可能已堆积旧段，启动即回收一次，不等到第一次切段
  if (config_.mode == ContinuousMode::kRolling) {
    retention_->enforce();
  }
  // 定时器只负责投递，切段本身在后台线程跑：关段要刷 cache 落盘、回收要递归扫目录
  // 再删文件，实测合起来几百毫秒。留在 executor 线程上会让所有订阅回调同时停摆
  roll_timer_ = node_->create_wall_timer(
    std::chrono::seconds(config_.segment_duration),
    [this]() {jobs_->post([this]() {rollSegment();});});

  RCLCPP_INFO(
    node_->get_logger(),
    "Continuous recording started: %d s per segment, mode %s, "
    "limit %d segments / %d MB, dir %s",
    config_.segment_duration,
    config_.mode == ContinuousMode::kRolling ? "rolling" : "limited",
    config_.max_segments, config_.max_total_size_mb, segment_root_.c_str());
}

void ContinuousRecorder::stop()
{
  if (roll_timer_) {
    roll_timer_->cancel();
    roll_timer_.reset();
  }
  // 可能还有已投递未执行的切段任务，先排空再关，避免关完又被开出一段
  if (jobs_) {
    jobs_->drain();
  }
  closeSegment();
}

bool ContinuousRecorder::isRecording() const
{
  std::lock_guard<std::mutex> lock(writer_mutex_);
  return recording_;
}

int ContinuousRecorder::completedSegments() const
{
  std::lock_guard<std::mutex> lock(writer_mutex_);
  return completed_segments_;
}

std::string ContinuousRecorder::makeSegmentUri() const
{
  const auto now = std::chrono::system_clock::now();
  const auto tt = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf{};
  localtime_r(&tt, &tm_buf);

  std::ostringstream oss;
  oss << segment_root_ << "/bag-" << std::put_time(&tm_buf, "%Y%m%d-%H%M%S");
  return oss.str();
}

void ContinuousRecorder::openSegment()
{
  // 建文件是磁盘操作，刻意不持 writer_mutex_：onMessage 也要拿这把锁，
  // 持锁做 I/O 会把所有订阅回调一起堵住（本包要消除的正是这类阻塞）
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = makeSegmentUri();
  storage_options.storage_id = storage_id_;
  storage_options.max_cache_size = static_cast<uint64_t>(writer_cache_size_);
  // 段边界由本类的定时器控制，不用 rosbag2 的 split：后者切出的多个 .db3 共享一份
  // metadata.yaml，无法按段独立删除
  storage_options.max_bagfile_duration = 0;
  storage_options.max_bagfile_size = 0;

  auto writer = std::make_unique<rosbag2_cpp::Writer>();
  try {
    writer->open(storage_options, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed to open recording segment: %s (%s)",
      storage_options.uri.c_str(), e.what());
    return;
  }

  {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    for (const auto & [name, metadata] : known_topics_) {
      (void)name;
      writer->create_topic(metadata);
    }
    writer_ = std::move(writer);
    current_uri_ = storage_options.uri;
    recording_ = true;
  }
  RCLCPP_INFO(node_->get_logger(), "Opened new segment: %s", storage_options.uri.c_str());
}

void ContinuousRecorder::closeSegment()
{
  std::string closed_uri;
  std::unique_ptr<rosbag2_cpp::Writer> closing;
  {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    if (!writer_) {
      return;
    }
    // 先把 writer 摘下来再关：close() 要把 cache 刷到盘上并写 metadata，
    // 持锁做这件事会把所有订阅回调堵住几百毫秒
    closing = std::move(writer_);
    recording_ = false;
    ++completed_segments_;
    closed_uri = current_uri_;
    current_uri_.clear();
  }
  try {
    closing->close();
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      node_->get_logger(), "Exception while closing recording segment: %s", e.what());
  }
  closing.reset();
  RCLCPP_INFO(node_->get_logger(), "Closed segment: %s", closed_uri.c_str());
}

void ContinuousRecorder::rollSegment()
{
  closeSegment();

  if (config_.mode == ContinuousMode::kLimited) {
    // 以磁盘上实际存在的段数判定，而不是内存计数器：后者在节点重启后归零，
    // limited 会变成"每重启一次再录 n 段"，永远停不下来
    const int on_disk = retention_->countEntries();
    if (on_disk >= config_.max_segments) {
      if (roll_timer_) {
        roll_timer_->cancel();
        roll_timer_.reset();
      }
      RCLCPP_INFO(
        node_->get_logger(),
        "limited mode reached %d segments (%d on disk), stopping continuous recording",
        config_.max_segments, on_disk);
      return;
    }
  }

  openSegment();

  // 回收必须在开新段之后：此刻磁盘上最后一个目录才是"正在写的段"，
  // enforce 的保护逻辑与 max_segments 语义（磁盘上最多保留这么多段）才对得上。
  // 放在开段之前会多留一段，实测 max_segments=2 时磁盘稳态是 3 个目录。
  if (config_.mode == ContinuousMode::kRolling) {
    const int removed = retention_->enforce();
    if (removed > 0) {
      RCLCPP_INFO(node_->get_logger(), "Reclaimed %d old segment(s)", removed);
    }
  }
}

}  // namespace mobile_bag_recorder
