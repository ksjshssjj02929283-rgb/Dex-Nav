// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#ifndef MOBILE_BAG_RECORDER__JOB_QUEUE_HPP_
#define MOBILE_BAG_RECORDER__JOB_QUEUE_HPP_

#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace mobile_bag_recorder
{

/// 单线程后台任务队列。
///
/// 存在的理由：录包节点里有一批「几百毫秒级」的重活——开/关 bag（要刷 cache 落盘）、
/// 把 pre 窗口几十 MB 倒进 writer、回收旧 bag（递归扫目录算大小再删）。这些活原本跑在
/// executor 线程上，期间**所有订阅回调都不执行**，于是所有话题在 bag 里同时出现
/// 177–365 ms 的空洞——恰恰是出事那一瞬间最想看的数据。
///
/// 把重活挪到这里，executor 线程只负责收消息和入队。
///
/// 只用一条线程而不是线程池：这些活彼此有顺序依赖（关段必须在开段之前完成，
/// 回收必须在关段之后），串行执行天然满足，也省掉一层锁。
class JobQueue
{
public:
  /// name 只用于日志与线程命名，便于 top -H / gdb 里辨认
  explicit JobQueue(std::string name);

  /// 析构时等待队列排空——事件 bag 关闭、metadata 落盘这类收尾不能丢
  ~JobQueue();

  JobQueue(const JobQueue &) = delete;
  JobQueue & operator=(const JobQueue &) = delete;

  /// 入队一个任务。调用方线程不阻塞
  void post(std::function<void()> job);

  /// 阻塞直到队列排空且当前任务执行完毕（测试与关停时用）
  void drain();

  /// 停止接受新任务，等待已入队任务跑完后退出线程
  void shutdown();

  /// 队列中尚未开始执行的任务数
  size_t pending() const;

private:
  void run();

  std::string name_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::condition_variable idle_cv_;
  std::deque<std::function<void()>> jobs_;
  bool running_{true};
  bool busy_{false};
  std::thread worker_;
};

}  // namespace mobile_bag_recorder

#endif  // MOBILE_BAG_RECORDER__JOB_QUEUE_HPP_
