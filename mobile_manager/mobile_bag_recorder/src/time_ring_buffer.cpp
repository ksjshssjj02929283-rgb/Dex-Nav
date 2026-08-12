// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include "mobile_bag_recorder/time_ring_buffer.hpp"

#include <algorithm>
#include <vector>

namespace mobile_bag_recorder
{

namespace
{

int64_t messageBytes(const rosbag2_storage::SerializedBagMessageSharedPtr & message)
{
  if (!message || !message->serialized_data) {
    return 0;
  }
  return static_cast<int64_t>(message->serialized_data->buffer_length);
}

}  // namespace

TimeRingBuffer::TimeRingBuffer(int64_t window_ns, int64_t max_bytes)
: window_ns_(window_ns), max_bytes_(max_bytes)
{
}

void TimeRingBuffer::push(const rosbag2_storage::SerializedBagMessageSharedPtr & message)
{
  if (!message) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  entries_.push_back(message);
  current_bytes_ += messageBytes(message);
  evictLocked(message->time_stamp);
}

void TimeRingBuffer::evictLocked(int64_t newest_stamp)
{
  // 以最新消息的时间戳为基准而不是 wall clock：话题整体停发时窗口不应被凭空清空，
  // 时钟跳变也不会误伤缓冲
  const int64_t cutoff = newest_stamp - window_ns_;
  while (!entries_.empty() && entries_.front()->time_stamp < cutoff) {
    current_bytes_ -= messageBytes(entries_.front());
    entries_.pop_front();
  }

  while (!entries_.empty() && current_bytes_ > max_bytes_) {
    current_bytes_ -= messageBytes(entries_.front());
    entries_.pop_front();
    ++dropped_by_capacity_;
  }

  if (entries_.empty()) {
    current_bytes_ = 0;
  }
}

std::vector<rosbag2_storage::SerializedBagMessageSharedPtr> TimeRingBuffer::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return {entries_.begin(), entries_.end()};
}

void TimeRingBuffer::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  entries_.clear();
  current_bytes_ = 0;
}

size_t TimeRingBuffer::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return entries_.size();
}

int64_t TimeRingBuffer::bytes() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_bytes_;
}

uint64_t TimeRingBuffer::droppedByCapacity() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return dropped_by_capacity_;
}

double TimeRingBuffer::spanSeconds() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (entries_.size() < 2) {
    return 0.0;
  }
  const int64_t span = entries_.back()->time_stamp - entries_.front()->time_stamp;
  return static_cast<double>(span) / 1e9;
}

}  // namespace mobile_bag_recorder
