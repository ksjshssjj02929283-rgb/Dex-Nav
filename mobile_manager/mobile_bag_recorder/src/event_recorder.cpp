// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include "mobile_bag_recorder/event_recorder.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rosbag2_storage/storage_options.hpp"

namespace fs = std::filesystem;

namespace mobile_bag_recorder
{

EventRecorder::EventRecorder(
  rclcpp::Node * node, const RecorderConfig & config, const std::string & output_root)
: node_(node),
  config_(config.event),
  storage_id_(config.storage_id),
  writer_cache_size_(config.writer_cache_size),
  event_root_(output_root + "/event"),
  last_trigger_time_(node->now())
{
  const auto topics = config.resolveEventTopics();
  wanted_topics_.insert(topics.begin(), topics.end());

  std::error_code ec;
  fs::create_directories(event_root_, ec);
  if (ec) {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed to create event recording directory: %s (%s)",
      event_root_.c_str(), ec.message().c_str());
  }

  buffer_ = std::make_unique<TimeRingBuffer>(
    static_cast<int64_t>(config_.pre_seconds) * 1000000000LL,
    static_cast<int64_t>(config_.max_buffer_mb) * 1024 * 1024);

  retention_ = std::make_unique<RetentionPolicy>(
    node_->get_logger(), event_root_, config_.max_events,
    static_cast<int64_t>(config_.max_total_size_mb) * 1024 * 1024);

  RCLCPP_INFO(
    node_->get_logger(),
    "Event recording armed: %zu topics, pre %d s / post %d s, cooldown %d s, "
    "buffer cap %d MB, dir %s",
    wanted_topics_.size(), config_.pre_seconds, config_.post_seconds,
    config_.cooldown_seconds, config_.max_buffer_mb, event_root_.c_str());
}

EventRecorder::~EventRecorder()
{
  finish();
}

bool EventRecorder::wantsTopic(const std::string & topic) const
{
  return wanted_topics_.count(topic) > 0;
}

void EventRecorder::onTopicDiscovered(const rosbag2_storage::TopicMetadata & metadata)
{
  std::lock_guard<std::mutex> lock(mutex_);
  known_topics_[metadata.name] = metadata;
  if (writer_ && active_) {
    writer_->create_topic(metadata);
  }
}

void EventRecorder::onMessage(const rosbag2_storage::SerializedBagMessageSharedPtr & message)
{
  // 录制期间也继续压缓冲：窗口必须一直滚动，否则本次事件结束后紧接着的下一次触发
  // 会拿到一段空白的前置数据
  buffer_->push(message);

  std::lock_guard<std::mutex> lock(mutex_);
  if (writer_ && active_) {
    writer_->write(message);
  }
}

std::string EventRecorder::sanitize(const std::string & text)
{
  std::string out;
  out.reserve(text.size());
  for (const char c : text) {
    out.push_back((std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-') ? c : '_');
  }
  if (out.size() > 40) {
    out.resize(40);
  }
  return out.empty() ? std::string("event") : out;
}

std::string EventRecorder::makeEventUri(const std::string & reason) const
{
  const auto now = std::chrono::system_clock::now();
  const auto tt = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf{};
  localtime_r(&tt, &tm_buf);

  std::ostringstream oss;
  oss << event_root_ << "/event-" << std::put_time(&tm_buf, "%Y%m%d-%H%M%S")
      << "-" << sanitize(reason);
  return oss.str();
}

TriggerResult EventRecorder::trigger(
  const std::string & reason, const std::string & source, int pre_override, int post_override)
{
  TriggerResult result;
  if (!config_.enabled) {
    result.message = "event recording is disabled (event.enabled=false)";
    return result;
  }

  const auto now = node_->now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_) {
      result.message = "an event recording is already in progress, trigger ignored";
      return result;
    }
    if (has_triggered_) {
      const double elapsed = (now - last_trigger_time_).seconds();
      if (elapsed < static_cast<double>(config_.cooldown_seconds)) {
        std::ostringstream oss;
        // 向上取整：剩余 0.4 s 时显示 "0 s remaining" 却仍然拒绝，读起来自相矛盾
        oss << "in cooldown, " << static_cast<int>(std::ceil(config_.cooldown_seconds - elapsed))
            << " s remaining";
        result.message = oss.str();
        return result;
      }
    }
  }

  auto pending = buffer_->snapshot();
  // pre 覆盖：只截取最近 pre_override 秒，缓冲窗口本身不变
  if (pre_override > 0 && !pending.empty()) {
    const int64_t cutoff =
      pending.back()->time_stamp - static_cast<int64_t>(pre_override) * 1000000000LL;
    const auto first = std::find_if(
      pending.begin(), pending.end(),
      [cutoff](const rosbag2_storage::SerializedBagMessageSharedPtr & m) {
        return m->time_stamp >= cutoff;
      });
    pending.erase(pending.begin(), first);
  }
  const double span = pending.size() < 2 ? 0.0 :
    static_cast<double>(pending.back()->time_stamp - pending.front()->time_stamp) / 1e9;
  const int post_seconds = post_override > 0 ? post_override : config_.post_seconds;

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = makeEventUri(reason);
  storage_options.storage_id = storage_id_;
  storage_options.max_cache_size = static_cast<uint64_t>(writer_cache_size_);
  storage_options.max_bagfile_duration = 0;
  storage_options.max_bagfile_size = 0;

  auto writer = std::make_unique<rosbag2_cpp::Writer>();
  try {
    writer->open(storage_options, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed to open event bag: %s (%s)",
      storage_options.uri.c_str(), e.what());
    result.message = std::string("failed to open event bag: ") + e.what();
    return result;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & [name, metadata] : known_topics_) {
      (void)name;
      writer->create_topic(metadata);
    }
    // 先把历史窗口倒进去，再放行实时消息，保证 bag 内时间连续
    for (const auto & buffered : pending) {
      writer->write(buffered);
    }
    writer_ = std::move(writer);
    current_uri_ = storage_options.uri;
    active_ = true;
    last_trigger_time_ = now;
    has_triggered_ = true;
  }

  post_timer_ = node_->create_wall_timer(
    std::chrono::seconds(post_seconds), [this]() {finish();});

  RCLCPP_INFO(
    node_->get_logger(),
    "Event triggered (%s%s): dumped %zu buffered messages spanning %.1f s, "
    "recording %d s more into %s",
    reason.c_str(), source.empty() ? "" : (" from " + source).c_str(),
    pending.size(), span, post_seconds, storage_options.uri.c_str());

  const uint64_t dropped = buffer_->droppedByCapacity();
  if (dropped > 0) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Ring buffer hit its %d MB cap and dropped %lu message(s) so far; "
      "the pre-trigger window may be shorter than %d s (raise event.max_buffer_mb "
      "or add heavy topics to event.exclude_topics)",
      config_.max_buffer_mb, static_cast<unsigned long>(dropped), config_.pre_seconds);
  }

  result.success = true;
  result.bag_path = storage_options.uri;
  result.message = "event recording started";
  return result;
}

void EventRecorder::finish()
{
  if (post_timer_) {
    post_timer_->cancel();
    post_timer_.reset();
  }

  std::string closed_uri;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!writer_ || !active_) {
      return;
    }
    try {
      writer_->close();
    } catch (const std::exception & e) {
      RCLCPP_WARN(node_->get_logger(), "Exception while closing event bag: %s", e.what());
    }
    writer_.reset();
    active_ = false;
    ++event_count_;
    closed_uri = current_uri_;
    current_uri_.clear();
  }

  RCLCPP_INFO(node_->get_logger(), "Event bag closed: %s", closed_uri.c_str());

  const int removed = retention_->enforce();
  if (removed > 0) {
    RCLCPP_INFO(node_->get_logger(), "Reclaimed %d old event bag(s)", removed);
  }
}

bool EventRecorder::isActive() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return active_;
}

int EventRecorder::eventCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return event_count_;
}

}  // namespace mobile_bag_recorder
