// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include <cstring>
#include <memory>
#include <string>

#include "mobile_bag_recorder/time_ring_buffer.hpp"

using mobile_bag_recorder::TimeRingBuffer;

namespace
{

constexpr int64_t kSecond = 1000000000LL;

rosbag2_storage::SerializedBagMessageSharedPtr makeMessage(
  int64_t stamp_ns, size_t payload_bytes = 16, const std::string & topic = "/t")
{
  auto msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  msg->topic_name = topic;
  msg->time_stamp = stamp_ns;

  auto * array = new rcutils_uint8_array_t;
  *array = rcutils_get_zero_initialized_uint8_array();
  const auto alloc = rcutils_get_default_allocator();
  EXPECT_EQ(rcutils_uint8_array_init(array, payload_bytes, &alloc), RCUTILS_RET_OK);
  array->buffer_length = payload_bytes;
  std::memset(array->buffer, 0, payload_bytes);

  msg->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
    array, [](rcutils_uint8_array_t * p) {
      const auto ret = rcutils_uint8_array_fini(p);
      (void)ret;
      delete p;
    });
  return msg;
}

TEST(TimeRingBufferTest, KeepsOnlyMessagesWithinTimeWindow)
{
  TimeRingBuffer buffer(5 * kSecond, 100 * 1024 * 1024);

  for (int i = 0; i <= 10; ++i) {
    buffer.push(makeMessage(i * kSecond));
  }

  // 最新戳 10s，窗口 5s -> 保留 [5s, 10s] 共 6 条
  EXPECT_EQ(buffer.size(), 6u);
  const auto snap = buffer.snapshot();
  ASSERT_FALSE(snap.empty());
  EXPECT_EQ(snap.front()->time_stamp, 5 * kSecond);
  EXPECT_EQ(snap.back()->time_stamp, 10 * kSecond);
  EXPECT_NEAR(buffer.spanSeconds(), 5.0, 1e-6);
  EXPECT_EQ(buffer.droppedByCapacity(), 0u);
}

TEST(TimeRingBufferTest, ByteCapTrimsWindowAndIsCounted)
{
  // 窗口足够长，但字节上限只放得下 4 条 1KB 消息
  TimeRingBuffer buffer(3600 * kSecond, 4 * 1024);

  for (int i = 0; i < 10; ++i) {
    buffer.push(makeMessage(i * kSecond, 1024));
  }

  EXPECT_LE(buffer.size(), 4u);
  EXPECT_LE(buffer.bytes(), 4 * 1024);
  // 被字节上限挤掉的消息必须计数，否则无法察觉 pre 窗口被内存压缩过
  EXPECT_GT(buffer.droppedByCapacity(), 0u);
}

TEST(TimeRingBufferTest, SnapshotDoesNotConsumeBuffer)
{
  TimeRingBuffer buffer(10 * kSecond, 100 * 1024 * 1024);
  buffer.push(makeMessage(1 * kSecond));
  buffer.push(makeMessage(2 * kSecond));

  const auto first = buffer.snapshot();
  const auto second = buffer.snapshot();

  // 事件录制期间窗口要继续滚动，取快照不能清空缓冲
  EXPECT_EQ(first.size(), 2u);
  EXPECT_EQ(second.size(), 2u);
  EXPECT_EQ(buffer.size(), 2u);
}

TEST(TimeRingBufferTest, StalledStreamKeepsBufferedHistory)
{
  TimeRingBuffer buffer(5 * kSecond, 100 * 1024 * 1024);
  buffer.push(makeMessage(100 * kSecond));
  buffer.push(makeMessage(101 * kSecond));

  // 裁剪基准是最新消息时间戳而非 wall clock：话题停发后历史窗口应保留，
  // 否则故障时刚好停发的话题会没有任何前置数据
  EXPECT_EQ(buffer.size(), 2u);
  EXPECT_EQ(buffer.snapshot().front()->time_stamp, 100 * kSecond);
}

TEST(TimeRingBufferTest, ClearEmptiesEverything)
{
  TimeRingBuffer buffer(10 * kSecond, 100 * 1024 * 1024);
  buffer.push(makeMessage(kSecond));
  buffer.clear();

  EXPECT_EQ(buffer.size(), 0u);
  EXPECT_EQ(buffer.bytes(), 0);
  EXPECT_EQ(buffer.spanSeconds(), 0.0);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
