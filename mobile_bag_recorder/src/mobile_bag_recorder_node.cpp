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
  if (event_) {
    event_->finish();
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

  // 先确认存储后端真的能写：插件缺失时 rosbag2 只在每次开段时抛异常，节点却会继续
  // "正常"运行——订阅、限速、切段全在跑，唯独一个字节都不落盘
  const auto usable_storage = resolveStorageId(
    config_.storage_id, config_.storage_fallback_id, output_root_, get_logger());
  if (usable_storage.empty()) {
    RCLCPP_ERROR(get_logger(), "Storage self-check failed, recording not started");
    return;
  }
  config_.storage_id = usable_storage;

  const auto throttle_intervals =
    RecorderConfig::parseThrottleRules(config_.throttle, get_logger());
  subscribers_ = std::make_unique<TopicSubscriberSet>(
    this, std::chrono::milliseconds(1000), throttle_intervals);

  if (config_.continuous.enabled) {
    continuous_ = std::make_shared<ContinuousRecorder>(this, config_, output_root_);
    subscribers_->addSink(continuous_);
    continuous_->start();
  }

  if (config_.event.enabled) {
    event_ = std::make_shared<EventRecorder>(this, config_, output_root_);
    subscribers_->addSink(event_);
    triggers_ = std::make_unique<TriggerMonitor>(
      this, config_.trigger,
      [this](const std::string & reason) {
        const auto result = event_->trigger(reason, "auto");
        if (!result.success) {
          RCLCPP_INFO(
            get_logger(), "Trigger from %s not taken: %s",
            reason.c_str(), result.message.c_str());
        }
      });
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

  // 需要 ack 与 bag 路径的调用方（人工 / 上位机 / mission_manager）走 service
  trigger_srv_ = create_service<mobile_bag_recorder::srv::TriggerRecord>(
    "~/trigger",
    [this](
      const std::shared_ptr<mobile_bag_recorder::srv::TriggerRecord::Request> request,
      std::shared_ptr<mobile_bag_recorder::srv::TriggerRecord::Response> response) {
      if (!event_) {
        response->success = false;
        response->message = "event recording is disabled (event.enabled=false)";
        return;
      }
      const auto reason = request->reason.empty() ? std::string("manual") : request->reason;
      const auto result = event_->trigger(
        reason, request->source, request->pre_seconds, request->post_seconds);
      response->success = result.success;
      response->message = result.message;
      response->bag_path = result.bag_path;
    });

  // 导航栈内各模块上报异常走 topic：发布不阻塞，控制回路里也能安全调用，
  // 且调用方只需 std_msgs，不必依赖本包
  trigger_event_sub_ = create_subscription<std_msgs::msg::String>(
    "~/trigger_event", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      if (!event_) {
        return;
      }
      const auto reason = msg->data.empty() ? std::string("external") : msg->data;
      const auto result = event_->trigger(reason, "topic");
      if (!result.success) {
        RCLCPP_INFO(
          get_logger(), "Trigger from topic (%s) not taken: %s",
          reason.c_str(), result.message.c_str());
      }
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
    get_logger(),
    "Topic discovery: %zu subscribed, %zu still missing from the configured list, "
    "%lu message(s) dropped by throttling so far",
    subscribers_->subscribedCount(), pending.size(),
    static_cast<unsigned long>(subscribers_->throttledCount()));

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
