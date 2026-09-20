// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#ifndef MOBILE_BAG_RECORDER__LOG_COLLECTOR_HPP_
#define MOBILE_BAG_RECORDER__LOG_COLLECTOR_HPP_

#include <chrono>
#include <future>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "mobile_bag_recorder/recorder_config.hpp"

namespace mobile_bag_recorder
{

/// 事件 bag 的日志快照采集：把 systemd journal 里触发点前后的一段日志导出到事件 bag
/// 目录下的 logs/ 子目录，和数据放在一起。
///
/// 为什么取 journal 而不是 `~/.ros/log/`：机器人上的节点都由 auto_mode.sh 经
/// `systemd-cat -t dexe_xxx` 拉起，stdout/stderr 全进系统 journal，那里才是「全系统
/// 一条时间线」——既有各 ROS 节点的输出，也有 kernel / systemd / 网络这些 ROS 侧看不到的
/// 事件。而 `~/.ros/log/` 按「每次 launch 一个目录」组织（实测机上已堆了 6500+ 个），
/// 既切不出时间窗口，也拿不到 ROS 之外的信息。
///
/// 采集走子进程（fork + exec journalctl）且整个动作在后台线程上做：journalctl 扫几分钟
/// 日志在 Jetson 上是几百毫秒量级，不能占着执行器线程——那会连带卡住订阅回调。
class LogCollector
{
public:
  LogCollector(rclcpp::Logger logger, LogCaptureConfig config);

  /// 等待仍在跑的采集结束，避免节点析构时子进程/线程悬着
  ~LogCollector();

  /// 为一个已经关闭的事件 bag 采集日志快照。立即返回，实际工作在后台线程完成。
  ///
  /// \param bag_dir       事件 bag 目录（日志写到它下面的 logs/）
  /// \param reason        触发原因，只写进文件头，方便日志被单独拷走后还认得出处
  /// \param trigger_time  触发时刻的系统时钟；窗口起点 = 它减去 pre_seconds
  void capture(
    const std::string & bag_dir, const std::string & reason,
    std::chrono::system_clock::time_point trigger_time);

  /// 组装 journalctl 参数（含 argv[0]）。抽出来是为了可单测，不必真 fork 一个 journalctl
  static std::vector<std::string> buildJournalctlArgs(
    const LogCaptureConfig & config, std::chrono::system_clock::time_point since);

  /// 把 time_point 格成 journalctl `--since` 认的本地时间字面量
  static std::string formatTime(std::chrono::system_clock::time_point tp);

private:
  /// 后台线程体：建目录、写文件头、跑 journalctl 把输出追加进去
  void runCapture(
    const std::string & bag_dir, const std::string & reason,
    std::chrono::system_clock::time_point trigger_time);

  /// fork + exec，stdout/stderr 都重定向到 fd。超时未退则 SIGKILL。
  /// 返回子进程退出码；fork/waitpid 失败返回 -1，超时返回 -2，被信号杀死返回 -3
  int runToFile(const std::vector<std::string> & args, int fd) const;

  rclcpp::Logger logger_;
  LogCaptureConfig config_;

  std::mutex mutex_;
  /// 跑着的采集任务。std::async 的 future 析构会阻塞，所以只能等它跑完再丢，
  /// 不能直接覆盖——这里攒着，每次 capture 顺手清掉已完成的
  std::vector<std::future<void>> pending_;
};

}  // namespace mobile_bag_recorder

#endif  // MOBILE_BAG_RECORDER__LOG_COLLECTOR_HPP_
