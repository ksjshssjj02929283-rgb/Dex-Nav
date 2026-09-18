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

  if (config.log_capture.enabled) {
    log_collector_ = std::make_unique<LogCollector>(node_->get_logger(), config.log_capture);
    RCLCPP_INFO(
      node_->get_logger(),
      "Event log snapshot enabled: last %d s of the system journal before each trigger "
      "will be written to <event bag>/logs/",
      config.log_capture.pre_seconds);
  }

  staging_limit_ = static_cast<size_t>(config_.staging_limit);
  jobs_ = std::make_unique<JobQueue>("event_recorder");

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
  // 关 bag、日志快照、回收都是异步的，必须等它们跑完再让成员开始析构
  if (jobs_) {
    jobs_->drain();
  }
}

bool EventRecorder::wantsTopic(const std::string & topic) const
{
  return wanted_topics_.count(topic) > 0;
}

void EventRecorder::onTopicDiscovered(const rosbag2_storage::TopicMetadata & metadata)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // 幂等：话题登记现在先于建订阅，订阅失败后下一轮 poll 会再次登记同一话题，
  // 不能对当前 writer 重复 create_topic
  if (!known_topics_.emplace(metadata.name, metadata).second) {
    return;
  }
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
  } else if (opening_) {
    // 开包任务正在后台倒 pre 窗口，此刻到达的实时消息先攒着，
    // 由开包任务在装上 writer 之前一并倒进去，bag 内时间才连续
    if (staging_.size() < staging_limit_) {
      staging_.push_back(message);
    } else {
      ++staging_dropped_;
    }
  }
}

uint64_t EventRecorder::stagingDropped() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return staging_dropped_;
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

  const std::string uri = makeEventUri(reason);

  uint64_t session = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // 先占住状态：active_ 立刻置真，重复触发会被前面的检查挡掉；
    // writer_ 仍为空、opening_ 为真，实时消息进 staging_ 等开包任务来倒
    active_ = true;
    opening_ = true;
    staging_.clear();
    staging_dropped_ = 0;
    current_uri_ = uri;
    last_trigger_time_ = now;
    has_triggered_ = true;
    trigger_wall_time_ = std::chrono::system_clock::now();
    current_reason_ = reason;
    session = ++session_id_;
  }

  post_timer_ = node_->create_wall_timer(
    std::chrono::seconds(post_seconds), [this]() {finish();});

  // 开包是重活：倒 60 s 的 pre 窗口实测几十 MB，再加建文件与 create_topic，
  // 合计几百毫秒。留在 executor 线程上会让所有订阅回调同时停摆——出事那一瞬间
  // 的数据恰恰就这么丢掉的，所以整段挪到后台线程。
  jobs_->post(
    [this, uri, session, pending = std::move(pending), reason, source, span, post_seconds]() {
      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = uri;
      storage_options.storage_id = storage_id_;
      storage_options.max_cache_size = static_cast<uint64_t>(writer_cache_size_);
      storage_options.max_bagfile_duration = 0;
      storage_options.max_bagfile_size = 0;

      auto writer = std::make_unique<rosbag2_cpp::Writer>();
      try {
        writer->open(storage_options, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          node_->get_logger(), "Failed to open event bag: %s (%s)", uri.c_str(), e.what());
        std::lock_guard<std::mutex> lock(mutex_);
        if (session_id_ == session) {
          active_ = false;
          opening_ = false;
          staging_.clear();
          current_uri_.clear();
          current_reason_.clear();
        }
        return;
      }

      // create_topic 必须先于任何 write。这里先按开包时刻的已知话题登记一遍，
      // 开包期间新发现的话题在装 writer 前再补登记（registered 去重）
      std::unordered_set<std::string> registered;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto & [name, metadata] : known_topics_) {
          writer->create_topic(metadata);
          registered.insert(name);
        }
      }

      // 倒 pre 窗口。刻意不持锁：这是最耗时的一段，持锁会把 onMessage 一起堵住
      for (const auto & buffered : pending) {
        writer->write(buffered);
      }

      size_t staged = 0;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (session_id_ != session || !active_) {
          // 开包期间已被 finish/新触发作废，丢弃这个 writer，别装回去
          try {
            writer->close();
          } catch (const std::exception & e) {
            RCLCPP_WARN(
              node_->get_logger(), "Exception while discarding stale event bag: %s", e.what());
          }
          return;
        }
        for (const auto & [name, metadata] : known_topics_) {
          if (registered.insert(name).second) {
            writer->create_topic(metadata);
          }
        }
        // 开包期间到达的实时消息补进去，接在 pre 窗口之后，bag 内时间连续
        for (const auto & staged_msg : staging_) {
          writer->write(staged_msg);
        }
        staged = staging_.size();
        staging_.clear();
        writer_ = std::move(writer);
        opening_ = false;
      }

      RCLCPP_INFO(
        node_->get_logger(),
        "Event triggered (%s%s): dumped %zu buffered messages spanning %.1f s "
        "(+%zu staged while opening), recording %d s more into %s",
        reason.c_str(), source.empty() ? "" : (" from " + source).c_str(),
        pending.size(), span, staged, post_seconds, uri.c_str());

      const uint64_t dropped_staging = stagingDropped();
      if (dropped_staging > 0) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Dropped %lu live message(s) while opening the event bag (disk too slow); "
          "the first moments after the trigger are incomplete",
          static_cast<unsigned long>(dropped_staging));
      }

      const uint64_t dropped = buffer_->droppedByCapacity();
      if (dropped > 0) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Ring buffer hit its %d MB cap and dropped %lu message(s) so far; "
          "the pre-trigger window may be shorter than %d s (raise event.max_buffer_mb "
          "or add heavy topics to event.exclude_topics)",
          config_.max_buffer_mb, static_cast<unsigned long>(dropped), config_.pre_seconds);
      }
    });

  result.success = true;
  result.bag_path = uri;
  result.message = "event recording started";
  return result;
}

void EventRecorder::finish()
{
  if (post_timer_) {
    post_timer_->cancel();
    post_timer_.reset();
  }

  std::unique_ptr<rosbag2_cpp::Writer> writer;
  std::string closed_uri;
  std::string closed_reason;
  std::chrono::system_clock::time_point closed_trigger_time;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) {
      return;
    }
    // session_id_ 前进一格：若开包任务还在跑，它装 writer 前会发现 session 不匹配，
    // 自行把那个 writer 关掉丢弃，不会装到一个已经结束的会话上
    ++session_id_;
    writer = std::move(writer_);
    active_ = false;
    opening_ = false;
    staging_.clear();
    ++event_count_;
    closed_uri = current_uri_;
    closed_reason = current_reason_;
    closed_trigger_time = trigger_wall_time_;
    current_uri_.clear();
    current_reason_.clear();
  }

  // 关 bag 要把 cache 刷到盘上、写 metadata；日志快照要调 journalctl；回收要递归
  // 扫目录算大小再删文件。三件加起来几百毫秒，留在 executor 线程上会让所有订阅
  // 回调同时停摆，于是事件 bag 收尾的那一刻反而丢数据
  // 转成 shared_ptr 再捕获：JobQueue 用 std::function 承载任务，而 std::function
  // 要求目标可拷贝，捕获 unique_ptr 的 lambda 是 move-only，编不过
  std::shared_ptr<rosbag2_cpp::Writer> closing_writer(std::move(writer));
  jobs_->post(
    [this, closing_writer, closed_uri, closed_reason, closed_trigger_time]() {
      if (closing_writer) {
        try {
          closing_writer->close();
        } catch (const std::exception & e) {
          RCLCPP_WARN(node_->get_logger(), "Exception while closing event bag: %s", e.what());
        }
      }
      RCLCPP_INFO(node_->get_logger(), "Event bag closed: %s", closed_uri.c_str());

      // 日志快照在 bag 关闭之后才采：这样窗口右端能盖住 post 段，
      // 「触发之后又发生了什么」和数据是同一段时间
      if (log_collector_) {
        log_collector_->capture(closed_uri, closed_reason, closed_trigger_time);
      }

      const int removed = retention_->enforce();
      if (removed > 0) {
        RCLCPP_INFO(node_->get_logger(), "Reclaimed %d old event bag(s)", removed);
      }
    });
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
