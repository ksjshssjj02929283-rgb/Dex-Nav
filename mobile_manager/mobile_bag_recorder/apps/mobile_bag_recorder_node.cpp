// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "mobile_bag_recorder/mobile_bag_recorder_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<mobile_bag_recorder::MobileBagRecorderNode>();
  node->init();

  // 必须用多线程 executor：本节点既有 40+ 路订阅回调，又有「开关 bag / 倒 pre 窗口 /
  // 回收旧包 / 话题发现轮询」这类重活。单线程 executor 下重活一跑，所有订阅回调同时
  // 停摆，bag 里会出现全话题同时缺数据的空洞——出事那一瞬间的数据恰恰就这么丢掉的。
  //
  // 订阅各自独占一个 MutuallyExclusive 回调组（见 TopicSubscriberSet），因此
  // 同一话题的回调仍然串行、话题之间可并行，限速状态无需加锁。
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), static_cast<size_t>(node->config().executor_threads));
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
