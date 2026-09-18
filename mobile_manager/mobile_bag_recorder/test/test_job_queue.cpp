// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "mobile_bag_recorder/job_queue.hpp"

using mobile_bag_recorder::JobQueue;

/// post 不阻塞调用方——这是整个改造的前提：executor 线程投完任务立刻返回去收消息
TEST(JobQueueTest, PostDoesNotBlockCaller)
{
  JobQueue queue("test");
  const auto begin = std::chrono::steady_clock::now();
  queue.post([]() {std::this_thread::sleep_for(std::chrono::milliseconds(200));});
  const auto elapsed = std::chrono::steady_clock::now() - begin;

  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 50);
  queue.drain();
}

/// 任务按投递顺序串行执行：关段必须先于开段完成，回收必须在关段之后
TEST(JobQueueTest, RunsJobsInOrder)
{
  JobQueue queue("test");
  std::vector<int> seen;
  for (int i = 0; i < 20; ++i) {
    queue.post([&seen, i]() {seen.push_back(i);});
  }
  queue.drain();

  ASSERT_EQ(seen.size(), 20u);
  for (int i = 0; i < 20; ++i) {
    EXPECT_EQ(seen[static_cast<size_t>(i)], i);
  }
}

/// drain 返回时队列必须真的空了且当前任务已执行完
TEST(JobQueueTest, DrainWaitsForCompletion)
{
  JobQueue queue("test");
  std::atomic<int> done{0};
  queue.post(
    [&done]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      done.fetch_add(1);
    });
  queue.drain();

  EXPECT_EQ(done.load(), 1);
  EXPECT_EQ(queue.pending(), 0u);
}

/// 任务抛异常不能掀翻工作线程，否则后续所有关段/回收会静默停摆
TEST(JobQueueTest, ThrowingJobDoesNotKillWorker)
{
  JobQueue queue("test");
  std::atomic<bool> after{false};
  queue.post([]() {throw std::runtime_error("boom");});
  queue.post([&after]() {after.store(true);});
  queue.drain();

  EXPECT_TRUE(after.load());
}

/// 停机时已入队的任务必须跑完：里面有 bag 关闭与 metadata 落盘
TEST(JobQueueTest, ShutdownFlushesQueuedJobs)
{
  std::atomic<int> count{0};
  {
    JobQueue queue("test");
    for (int i = 0; i < 10; ++i) {
      queue.post([&count]() {count.fetch_add(1);});
    }
  }  // 析构 → shutdown → join
  EXPECT_EQ(count.load(), 10);
}

/// shutdown 之后 post 应被忽略，且不得死锁
TEST(JobQueueTest, PostAfterShutdownIsIgnored)
{
  JobQueue queue("test");
  queue.shutdown();
  std::atomic<bool> ran{false};
  queue.post([&ran]() {ran.store(true);});
  EXPECT_FALSE(ran.load());
  EXPECT_EQ(queue.pending(), 0u);
}
