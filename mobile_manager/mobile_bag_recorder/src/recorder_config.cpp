// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include "mobile_bag_recorder/recorder_config.hpp"

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

RecorderConfig RecorderConfig::declareAndLoad(rclcpp::Node * node)
{
  RecorderConfig cfg;
  const auto logger = node->get_logger();

  cfg.output_dir = node->declare_parameter(
    "output_dir", std::string("~/.dexe/navigation/bags"));

  // 以下两项极少现场调整，故只在代码里给默认值，yaml 需要时再 override
  cfg.storage_id = node->declare_parameter("storage_id", std::string("sqlite3"));
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
