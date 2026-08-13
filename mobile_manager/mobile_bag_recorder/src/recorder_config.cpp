// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include "mobile_bag_recorder/recorder_config.hpp"

#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <unordered_set>
#include <vector>

namespace mobile_bag_recorder
{

namespace
{

/// 读取 int 参数并做下界校验；越界回落默认值并告警
int declareCheckedInt(
  rclcpp::Node * node, const std::string & name, int default_value, int min_value)
{
  const int value = node->declare_parameter(name, default_value);
  if (value < min_value) {
    RCLCPP_WARN(
      node->get_logger(), "Parameter %s=%d is below the minimum %d, falling back to default %d",
      name.c_str(), value, min_value, default_value);
    return default_value;
  }
  return value;
}

}  // namespace

ContinuousMode RecorderConfig::parseMode(const std::string & text, const rclcpp::Logger & logger)
{
  if (text == "rolling") {
    return ContinuousMode::kRolling;
  }
  if (text == "limited") {
    return ContinuousMode::kLimited;
  }
  RCLCPP_WARN(logger, "Unknown continuous.mode=\"%s\", falling back to rolling", text.c_str());
  return ContinuousMode::kRolling;
}

std::unordered_map<std::string, int64_t> RecorderConfig::parseThrottleRules(
  const std::vector<std::string> & rules, const rclcpp::Logger & logger)
{
  std::unordered_map<std::string, int64_t> intervals;
  for (const auto & rule : rules) {
    // 话题名里没有冒号，取最后一个冒号分隔，话题名本身怎么写都不会被截断
    const auto sep = rule.rfind(':');
    if (sep == std::string::npos || sep == 0 || sep + 1 == rule.size()) {
      RCLCPP_WARN(
        logger, "Ignoring malformed throttle rule \"%s\", expected \"<topic>:<max_hz>\"",
        rule.c_str());
      continue;
    }
    const std::string topic = rule.substr(0, sep);
    double hz = 0.0;
    try {
      hz = std::stod(rule.substr(sep + 1));
    } catch (const std::exception &) {
      RCLCPP_WARN(
        logger, "Ignoring throttle rule \"%s\": \"%s\" is not a number",
        rule.c_str(), rule.substr(sep + 1).c_str());
      continue;
    }
    if (hz <= 0.0) {
      // 0 会被理解成"完全不录"，但那应该靠从 topics 清单里删掉来表达，别在这里悄悄实现
      RCLCPP_WARN(
        logger, "Ignoring throttle rule \"%s\": rate must be > 0 "
        "(remove the topic from the list instead of throttling it to 0)", rule.c_str());
      continue;
    }
    intervals[topic] = static_cast<int64_t>(1e9 / hz);
  }
  return intervals;
}

RecorderConfig RecorderConfig::declareAndLoad(rclcpp::Node * node)
{
  RecorderConfig cfg;
  const auto logger = node->get_logger();

  cfg.output_dir = node->declare_parameter(
    "output_dir", std::string("~/.dexe/navigation/bags"));

  // 以下两项极少现场调整，故只在代码里给默认值，yaml 需要时再 override
  cfg.storage_id = node->declare_parameter("storage_id", std::string("sqlite3"));
  cfg.storage_fallback_id = node->declare_parameter(
    "storage_fallback_id", std::string("sqlite3"));
  cfg.writer_cache_size = node->declare_parameter(
    "writer_cache_size", static_cast<int64_t>(100 * 1024 * 1024));

  cfg.continuous.enabled = node->declare_parameter("continuous.enabled", true);
  cfg.continuous.segment_duration =
    declareCheckedInt(node, "continuous.segment_duration", 300, 1);
  cfg.continuous.mode = parseMode(
    node->declare_parameter("continuous.mode", std::string("rolling")), logger);
  cfg.continuous.max_segments = declareCheckedInt(node, "continuous.max_segments", 24, 1);
  cfg.continuous.max_total_size_mb =
    declareCheckedInt(node, "continuous.max_total_size_mb", 20480, 1);
  cfg.continuous.topics = node->declare_parameter(
    "continuous.topics", std::vector<std::string>{});

  // 限速是订阅层的行为（常态与事件共用一套订阅），故放在顶层而不是 continuous 下面
  cfg.throttle = node->declare_parameter("throttle", std::vector<std::string>{});

  cfg.event.enabled = node->declare_parameter("event.enabled", true);
  cfg.event.pre_seconds = declareCheckedInt(node, "event.pre_seconds", 60, 1);
  cfg.event.post_seconds = declareCheckedInt(node, "event.post_seconds", 30, 0);
  cfg.event.cooldown_seconds = declareCheckedInt(node, "event.cooldown_seconds", 120, 0);
  cfg.event.max_events = declareCheckedInt(node, "event.max_events", 50, 1);
  cfg.event.max_total_size_mb = declareCheckedInt(node, "event.max_total_size_mb", 5120, 1);
  cfg.event.max_buffer_mb = declareCheckedInt(node, "event.max_buffer_mb", 512, 1);
  cfg.event.inherit_continuous_topics =
    node->declare_parameter("event.inherit_continuous_topics", true);
  cfg.event.exclude_topics = node->declare_parameter(
    "event.exclude_topics", std::vector<std::string>{});
  cfg.event.extra_topics = node->declare_parameter(
    "event.extra_topics", std::vector<std::string>{});

  cfg.trigger.on_nav_aborted = node->declare_parameter("trigger.on_nav_aborted", true);
  cfg.trigger.on_nav_canceled = node->declare_parameter("trigger.on_nav_canceled", false);
  cfg.trigger.on_recovery_behavior =
    node->declare_parameter("trigger.on_recovery_behavior", false);
  cfg.trigger.on_diagnostics_error =
    node->declare_parameter("trigger.on_diagnostics_error", true);
  cfg.trigger.goal_error_threshold_m =
    node->declare_parameter("trigger.goal_error_threshold_m", 0.0);
  cfg.trigger.on_nav_stall = node->declare_parameter("trigger.on_nav_stall", true);
  cfg.trigger.stall_timeout_s = node->declare_parameter("trigger.stall_timeout_s", 20.0);
  cfg.trigger.stall_min_progress_m =
    node->declare_parameter("trigger.stall_min_progress_m", 0.2);
  cfg.trigger.stall_progress_source = node->declare_parameter(
    "trigger.stall_progress_source", cfg.trigger.stall_progress_source);
  cfg.trigger.recovery_node_names = node->declare_parameter(
    "trigger.recovery_node_names", cfg.trigger.recovery_node_names);

  // 话题名极少改动，只在代码里给默认值；确需重映射时 yaml 补一行即可
  cfg.trigger.nav_status_topic = node->declare_parameter(
    "trigger.nav_status_topic", cfg.trigger.nav_status_topic);
  cfg.trigger.diagnostics_topic = node->declare_parameter(
    "trigger.diagnostics_topic", cfg.trigger.diagnostics_topic);
  cfg.trigger.goal_error_topic = node->declare_parameter(
    "trigger.goal_error_topic", cfg.trigger.goal_error_topic);
  cfg.trigger.behavior_tree_log_topic = node->declare_parameter(
    "trigger.behavior_tree_log_topic", cfg.trigger.behavior_tree_log_topic);
  cfg.trigger.nav_feedback_topic = node->declare_parameter(
    "trigger.nav_feedback_topic", cfg.trigger.nav_feedback_topic);
  cfg.trigger.odom_topic = node->declare_parameter(
    "trigger.odom_topic", cfg.trigger.odom_topic);

  if (cfg.continuous.topics.empty()) {
    RCLCPP_WARN(
      logger,
      "continuous.topics is empty - nothing will be recorded, "
      "check that the params file is loaded");
  }

  return cfg;
}

std::vector<std::string> RecorderConfig::resolveEventTopics() const
{
  std::vector<std::string> result;
  const std::unordered_set<std::string> excluded(
    event.exclude_topics.begin(), event.exclude_topics.end());
  std::unordered_set<std::string> seen;

  if (event.inherit_continuous_topics) {
    for (const auto & topic : continuous.topics) {
      if (excluded.count(topic) == 0 && seen.insert(topic).second) {
        result.push_back(topic);
      }
    }
  }
  for (const auto & topic : event.extra_topics) {
    // extra_topics 是显式补录，优先级高于 exclude
    if (seen.insert(topic).second) {
      result.push_back(topic);
    }
  }
  return result;
}

std::string prepareOutputDir(const std::string & raw_dir, const rclcpp::Logger & logger)
{
  std::string expanded = raw_dir;
  if (!expanded.empty() && expanded[0] == '~') {
    const char * home = std::getenv("HOME");
    if (home == nullptr) {
      RCLCPP_ERROR(logger, "output_dir starts with ~ but HOME is not set: %s", raw_dir.c_str());
      return {};
    }
    expanded = std::string(home) + expanded.substr(1);
  }

  std::error_code ec;
  std::filesystem::create_directories(expanded, ec);
  if (ec) {
    RCLCPP_ERROR(
      logger, "Failed to create output directory: %s (%s)",
      expanded.c_str(), ec.message().c_str());
    return {};
  }
  return expanded;
}

}  // namespace mobile_bag_recorder

namespace
{

/// 试开一个临时 bag 再删掉。连插件可用性和目录写权限一起验了
bool probeStorage(
  const std::string & storage_id, const std::string & root, const rclcpp::Logger & logger)
{
  const std::string probe_uri = root + "/.storage_probe";
  std::error_code ec;
  std::filesystem::remove_all(probe_uri, ec);

  rosbag2_storage::StorageOptions options;
  options.uri = probe_uri;
  options.storage_id = storage_id;

  bool ok = false;
  try {
    rosbag2_cpp::Writer writer;
    writer.open(options, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});
    writer.close();
    ok = true;
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(logger, "Storage probe for \"%s\" failed: %s", storage_id.c_str(), e.what());
  }

  std::filesystem::remove_all(probe_uri, ec);
  return ok;
}

}  // namespace

namespace mobile_bag_recorder
{

std::string resolveStorageId(
  const std::string & preferred, const std::string & fallback,
  const std::string & root, const rclcpp::Logger & logger)
{
  if (probeStorage(preferred, root, logger)) {
    RCLCPP_INFO(logger, "Storage backend: %s", preferred.c_str());
    return preferred;
  }

  if (fallback.empty() || fallback == preferred) {
    RCLCPP_ERROR(
      logger,
      "Storage backend \"%s\" is unavailable and no fallback is configured. "
      "Nothing would be recorded, so recording is not started. "
      "Install the corresponding rosbag2 storage plugin or set storage_id to an available one.",
      preferred.c_str());
    return {};
  }

  if (probeStorage(fallback, root, logger)) {
    // 用 ERROR 而不是 WARN：换了后端是需要被看见的事，别让它淹在 INFO 里
    RCLCPP_ERROR(
      logger,
      "Storage backend \"%s\" is unavailable (plugin missing?), falling back to \"%s\". "
      "Recording continues, but the bag format is not the configured one.",
      preferred.c_str(), fallback.c_str());
    return fallback;
  }

  RCLCPP_ERROR(
    logger,
    "Neither storage backend \"%s\" nor fallback \"%s\" could be initialized "
    "(check the output directory is writable). Recording is not started.",
    preferred.c_str(), fallback.c_str());
  return {};
}

}  // namespace mobile_bag_recorder
