// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include "mobile_bag_recorder/log_collector.hpp"

#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

namespace mobile_bag_recorder
{

namespace
{

/// 日志快照在事件 bag 里的位置。放子目录而不是和 .mcap 平铺，
/// 是为了 rosbag2 那边怎么扫都不会把它当成数据文件
constexpr const char * kLogSubdir = "logs";
constexpr const char * kJournalFile = "system-journal.log";

}  // namespace

LogCollector::LogCollector(rclcpp::Logger logger, LogCaptureConfig config)
: logger_(std::move(logger)), config_(std::move(config))
{
}

LogCollector::~LogCollector()
{
  std::lock_guard<std::mutex> lock(mutex_);
  // future 析构本来就会等，这里显式 get 一下只是为了意图清楚：
  // 每个任务内部都有超时，不会无限等下去
  for (auto & job : pending_) {
    if (job.valid()) {
      job.wait();
    }
  }
}

std::string LogCollector::formatTime(std::chrono::system_clock::time_point tp)
{
  const auto tt = std::chrono::system_clock::to_time_t(tp);
  std::tm tm_buf{};
  localtime_r(&tt, &tm_buf);
  std::ostringstream oss;
  oss << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S");
  return oss.str();
}

std::vector<std::string> LogCollector::buildJournalctlArgs(
  const LogCaptureConfig & config, std::chrono::system_clock::time_point since)
{
  // 不给 --until：不带它就是"截至当前"，正好等于采集时刻，窗口右端天然盖住 post 段。
  // -o short-precise 带微秒，才对得上 bag 里的消息时间戳（默认的 short 只到秒）
  std::vector<std::string> args{
    "journalctl", "--no-pager", "-o", "short-precise", "--since", formatTime(since)};

  if (config.max_lines > 0) {
    args.push_back("--lines");
    args.push_back(std::to_string(config.max_lines));
  }
  // 多个 -t 之间是或的关系；留空则全量采集（kernel / systemd 的消息也要，
  // 掉电、OOM、网卡抖动这些在 ROS 侧一个字都看不到）
  for (const auto & identifier : config.identifiers) {
    args.push_back("-t");
    args.push_back(identifier);
  }
  return args;
}

void LogCollector::capture(
  const std::string & bag_dir, const std::string & reason,
  std::chrono::system_clock::time_point trigger_time)
{
  if (!config_.enabled) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  // 清掉已完成的任务，别让 vector 随事件数无限涨
  pending_.erase(
    std::remove_if(
      pending_.begin(), pending_.end(),
      [](const std::future<void> & job) {
        return !job.valid() ||
               job.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
      }),
    pending_.end());

  pending_.push_back(
    std::async(
      std::launch::async,
      [this, bag_dir, reason, trigger_time]() {runCapture(bag_dir, reason, trigger_time);}));
}

void LogCollector::runCapture(
  const std::string & bag_dir, const std::string & reason,
  std::chrono::system_clock::time_point trigger_time)
{
  const fs::path log_dir = fs::path(bag_dir) / kLogSubdir;
  std::error_code ec;
  fs::create_directories(log_dir, ec);
  if (ec) {
    RCLCPP_WARN(
      logger_, "Failed to create log directory %s (%s), skipping log capture",
      log_dir.c_str(), ec.message().c_str());
    return;
  }

  const fs::path log_file = log_dir / kJournalFile;
  const auto since = trigger_time - std::chrono::seconds(config_.pre_seconds);
  const auto args = buildJournalctlArgs(config_, since);

  // 文件头里的命令行是给人照着重跑的，带空格的参数（--since 的时间字面量）要加引号
  std::ostringstream command;
  for (size_t i = 0; i < args.size(); ++i) {
    const bool quote = args[i].find(' ') != std::string::npos;
    command << (i == 0 ? "" : " ") << (quote ? "\"" : "") << args[i] << (quote ? "\"" : "");
  }

  {
    // 文件头：日志被单独拷出去之后，还得看得出它属于哪次触发、覆盖哪段时间
    std::ofstream header(log_file.string(), std::ios::trunc);
    if (!header) {
      RCLCPP_WARN(logger_, "Failed to open log snapshot file %s", log_file.c_str());
      return;
    }
    header << "# mobile_bag_recorder event log snapshot\n"
           << "# bag     : " << bag_dir << "\n"
           << "# reason  : " << reason << "\n"
           << "# trigger : " << formatTime(trigger_time) << "\n"
           << "# window  : " << formatTime(since) << " .. "
           << formatTime(std::chrono::system_clock::now())
           << "  (pre " << config_.pre_seconds << " s + the post-trigger recording)\n"
           << "# command : " << command.str() << "\n"
           << "#\n";
  }

  const int fd = ::open(log_file.c_str(), O_WRONLY | O_APPEND | O_CLOEXEC);
  if (fd < 0) {
    RCLCPP_WARN(logger_, "Failed to reopen log snapshot file %s", log_file.c_str());
    return;
  }
  const int rc = runToFile(args, fd);
  ::close(fd);

  const auto bytes = fs::file_size(log_file, ec);
  const double kb = ec ? 0.0 : static_cast<double>(bytes) / 1024.0;

  if (rc == 0) {
    RCLCPP_INFO(
      logger_, "Captured %.0f KB of system log into %s", kb, log_file.c_str());
    return;
  }

  // 失败不改变 bag 本身的价值，所以只告警不报错；journalctl 的 stderr 已经写进文件里了
  if (rc == -2) {
    RCLCPP_WARN(
      logger_,
      "Log capture timed out after %d s and was killed; %s holds only a partial snapshot "
      "(raise log_capture.timeout_seconds, or narrow the window via log_capture.pre_seconds "
      "/ max_lines / identifiers)",
      config_.timeout_seconds, log_file.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "Log capture failed (journalctl exit %d); see %s for its stderr. "
      "Set log_capture.enabled=false if this system does not log to the journal.",
      rc, log_file.c_str());
  }
}

int LogCollector::runToFile(const std::vector<std::string> & args, int fd) const
{
  std::vector<char *> argv;
  argv.reserve(args.size() + 1);
  for (const auto & arg : args) {
    argv.push_back(const_cast<char *>(arg.c_str()));
  }
  argv.push_back(nullptr);

  const pid_t pid = ::fork();
  if (pid < 0) {
    RCLCPP_WARN(logger_, "fork() failed, cannot capture logs");
    return -1;
  }

  if (pid == 0) {
    // 子进程：只能调 async-signal-safe 的东西，直到 exec 把这个进程整个换掉
    if (::dup2(fd, STDOUT_FILENO) < 0 || ::dup2(fd, STDERR_FILENO) < 0) {
      _exit(127);
    }
    ::execvp(argv[0], argv.data());
    _exit(127);  // exec 失败（journalctl 不存在）才会走到这
  }

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(config_.timeout_seconds);
  int status = 0;
  while (true) {
    const pid_t reaped = ::waitpid(pid, &status, WNOHANG);
    if (reaped == pid) {
      break;
    }
    if (reaped < 0) {
      return -1;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      ::kill(pid, SIGKILL);
      ::waitpid(pid, &status, 0);
      return -2;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  return WIFEXITED(status) ? WEXITSTATUS(status) : -3;
}

}  // namespace mobile_bag_recorder
