// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include "mobile_bag_recorder/job_queue.hpp"

#include <utility>

namespace mobile_bag_recorder
{

JobQueue::JobQueue(std::string name)
: name_(std::move(name))
{
  worker_ = std::thread([this]() {run();});
}

JobQueue::~JobQueue()
{
  shutdown();
}

void JobQueue::post(std::function<void()> job)
{
  if (!job) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
      return;
    }
    jobs_.push_back(std::move(job));
  }
  cv_.notify_one();
}

void JobQueue::drain()
{
  std::unique_lock<std::mutex> lock(mutex_);
  idle_cv_.wait(lock, [this] {return jobs_.empty() && !busy_;});
}

void JobQueue::shutdown()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
      return;
    }
    running_ = false;
  }
  cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

size_t JobQueue::pending() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return jobs_.size();
}

void JobQueue::run()
{
  for (;;) {
    std::function<void()> job;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this] {return !jobs_.empty() || !running_;});
      // 停机时也要把已入队的任务跑完：里面有 bag 关闭与 metadata 落盘
      if (jobs_.empty()) {
        return;
      }
      job = std::move(jobs_.front());
      jobs_.pop_front();
      busy_ = true;
    }

    // 任务自身异常不能掀翻工作线程，否则后续所有关段/回收都停摆且无声无息
    try {
      job();
    } catch (...) {
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      busy_ = false;
    }
    idle_cv_.notify_all();
  }
}

}  // namespace mobile_bag_recorder
