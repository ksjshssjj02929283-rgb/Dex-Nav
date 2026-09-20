// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#ifndef MOBILE_BAG_RECORDER__TIME_RING_BUFFER_HPP_
#define MOBILE_BAG_RECORDER__TIME_RING_BUFFER_HPP_

#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

#include "rosbag2_storage/serialized_bag_message.hpp"

namespace mobile_bag_recorder
{

/// 按**时间**保留最近 N 秒消息的环形缓冲。
///
/// 与 rosbag2 自带的 snapshot 缓冲（`CircularMessageCache`，按字节裁剪）不同：这里以消息
/// 时间戳为准裁剪，所以"触发点之前 N 秒"是确定的，不随数据率漂移。字节上限只作为防 OOM 的
/// 兜底——数据率突增时宁可让窗口短于 N 秒，也不能把内存吃穿。
class TimeRingBuffer
{
public:
  TimeRingBuffer(int64_t window_ns, int64_t max_bytes);

  void push(const rosbag2_storage::SerializedBagMessageSharedPtr & message);

  /// 取出当前缓冲内容（时间升序）。不清空缓冲——事件录制期间窗口要继续滚动，
  /// 以免下一次触发拿不到前置数据
  std::vector<rosbag2_storage::SerializedBagMessageSharedPtr> snapshot() const;

  void clear();

  size_t size() const;
  int64_t bytes() const;

  /// 因字节上限（而非时间窗口）被丢弃的消息数，用于判断窗口是否被内存压缩过
  uint64_t droppedByCapacity() const;

  /// 缓冲内最旧与最新消息的时间跨度（秒）；不足 2 条时返回 0
  double spanSeconds() const;

private:
  void evictLocked(int64_t newest_stamp);

  mutable std::mutex mutex_;
  std::deque<rosbag2_storage::SerializedBagMessageSharedPtr> entries_;
  int64_t window_ns_;
  int64_t max_bytes_;
  int64_t current_bytes_{0};
  uint64_t dropped_by_capacity_{0};
};

}  // namespace mobile_bag_recorder

#endif  // MOBILE_BAG_RECORDER__TIME_RING_BUFFER_HPP_
