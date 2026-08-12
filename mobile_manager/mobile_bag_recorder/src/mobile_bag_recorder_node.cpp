// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include "mobile_bag_recorder/mobile_bag_recorder_node.hpp"

#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace mobile_bag_recorder
{

MobileBagRecorderNode::MobileBagRecorderNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mobile_bag_recorder_node", options)
{
  config_ = RecorderConfig::declareAndLoad(this);
}

MobileBagRecorderNode::~MobileBagRecorderNode()
{
  if (subscribers_) {
    subscribers_->stop();
  }
  if (continuous_) {
    continuous_->stop();
  }
}

void MobileBagRecorderNode::init()
{
  output_root_ = prepareOutputDir(config_.output_dir, get_logger());
  if (output_root_.empty()) {
    RCLCPP_ERROR(get_logger(), "Output directory unavailable, recording not started");
    return;
  }

  subscribers_ = std::make_unique<TopicSubscriberSet>(this, std::chrono::milliseconds(1000));

  if (config_.continuous.enabled) {
    continuous_ = std::make_shared<ContinuousRecorder>(this, config_, output_root_);
    subscribers_->addSink(continuous_);
    continuous_->start();
  }

  subscribers_->start();

  start_continuous_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/start_continuous",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      if (!continuous_) {
        response->success = false;
        response->message = "Continuous recording is disabled (continuous.enabled=false)";
        return;
      }
      continuous_->start();
      response->success = true;
      response->message = "Continuous recording started";
    });

  stop_continuous_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/stop_continuous",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      if (!continuous_) {
        response->success = false;
        response->message = "Continuous recording is disabled (continuous.enabled=false)";
        return;
      }
      continuous_->stop();
      response->success = true;
      response->message = "Continuous recording stopped";
    });

  // 话题上线有先后，启动即报缺失会有大量噪音；等一轮发现周期过去再报
  pending_report_timer_ = create_wall_timer(
    std::chrono::seconds(15), [this]() {reportPendingTopics();});

  RCLCPP_INFO(get_logger(), "mobile_bag_recorder ready, output root %s", output_root_.c_str());
}

void MobileBagRecorderNode::reportPendingTopics()
{
  pending_report_timer_->cancel();
  pending_report_timer_.reset();

  const auto pending = subscribers_->pendingTopics(config_.continuous.topics);
  RCLCPP_INFO(
    get_logger(), "Topic discovery: %zu subscribed, %zu still missing from the configured list",
    subscribers_->subscribedCount(), pending.size());

  if (pending.empty()) {
    return;
  }
  std::ostringstream oss;
  for (size_t i = 0; i < pending.size(); ++i) {
    oss << (i == 0 ? "" : " ") << pending[i];
  }
  // 未出现不等于配置错误：这些话题可能由尚未启动的子系统发布，上线后会自动补订阅
  RCLCPP_WARN(
    get_logger(), "Topics not published yet (will be picked up once they appear): %s",
    oss.str().c_str());
}

}  // namespace mobile_bag_recorder
