// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include "action_msgs/msg/goal_status_array.hpp"
#include "nav_msgs/msg/odometry.hpp"

#ifdef MOBILE_BAG_RECORDER_HAS_BT_LOG
#include "nav2_msgs/action/navigate_to_pose.hpp"
#endif

#include "mobile_bag_recorder/trigger_sources.hpp"

namespace mobile_bag_recorder
{

namespace
{

/// 导航卡住：任务进行中（action EXECUTING）**且**持续无实质推进。
///
/// 两个条件是与关系——只看"不动"会把避障等待、原地旋转、没有任务时的静止全报成故障；
/// 只看"任务进行中"则毫无意义。
///
/// 推进量有两种来源：
///   feedback：action 的 distance_remaining 减少量，能识别"在动但没接近目标"（绕圈）；
///   odom    ：机器人相对基准点的位移，识别不了绕圈，但不依赖 nav2_msgs。
/// auto 模式下有 nav2_msgs 就用 feedback，否则回退 odom。
class NavStallSource : public TriggerSource
{
public:
  std::string name() const override {return "nav_stall";}

  bool isEnabled(const TriggerConfig & config) const override {return config.on_nav_stall;}

  void setup(const TriggerContext & context) override
  {
    context_ = context;
    use_feedback_ = resolveUseFeedback(context.config.stall_progress_source);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    status_sub_ = context.node->create_subscription<action_msgs::msg::GoalStatusArray>(
      context.config.nav_status_topic, qos,
      [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {onStatus(msg);});

    if (use_feedback_) {
#ifdef MOBILE_BAG_RECORDER_HAS_BT_LOG
      feedback_sub_ =
        context.node->create_subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>(
        context.config.nav_feedback_topic, qos,
        [this](const nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage::SharedPtr msg) {
          progress_value_ = msg->feedback.distance_remaining;
          has_progress_ = true;
        });
#endif
    } else {
      odom_sub_ = context.node->create_subscription<nav_msgs::msg::Odometry>(
        context.config.odom_topic, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          x_ = msg->pose.pose.position.x;
          y_ = msg->pose.pose.position.y;
          has_progress_ = true;
        });
    }

    timer_ = context.node->create_wall_timer(
      std::chrono::milliseconds(500), [this]() {check();});

    RCLCPP_INFO(
      context.node->get_logger(),
      "Trigger source [nav_stall]: %s + %s, stalled if progress < %.2f m for %.1f s",
      context.config.nav_status_topic.c_str(),
      use_feedback_ ? context.config.nav_feedback_topic.c_str() :
      context.config.odom_topic.c_str(),
      context.config.stall_min_progress_m, context.config.stall_timeout_s);
  }

private:
  static bool resolveUseFeedback(const std::string & source)
  {
#ifdef MOBILE_BAG_RECORDER_HAS_BT_LOG
    return source != "odom";
#else
    // 没有 nav2_msgs 就拿不到 distance_remaining，只能用 odom 位移
    return false;
#endif
  }

  void onStatus(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
  {
    bool executing = false;
    for (const auto & status : msg->status_list) {
      if (status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
        executing = true;
        break;
      }
    }
    if (executing != executing_) {
      executing_ = executing;
      resetBaseline();
    }
  }

  void resetBaseline()
  {
    baseline_set_ = false;
    fired_ = false;
  }

  /// 相对基准的推进量（米）。feedback 用剩余距离的减少量，odom 用位移
  double progressSinceBaseline() const
  {
    if (use_feedback_) {
      // 远离目标同样算没有推进，故取非负
      return std::max(0.0, baseline_value_ - progress_value_);
    }
    return std::hypot(x_ - baseline_x_, y_ - baseline_y_);
  }

  void check()
  {
    if (!executing_ || !has_progress_) {
      return;
    }

    const auto now = context_.node->now();
    if (!baseline_set_) {
      baseline_time_ = now;
      baseline_value_ = progress_value_;
      baseline_x_ = x_;
      baseline_y_ = y_;
      baseline_set_ = true;
      return;
    }

    if (progressSinceBaseline() >= context_.config.stall_min_progress_m) {
      // 有实质推进，重新起算并解除已触发标记
      baseline_time_ = now;
      baseline_value_ = progress_value_;
      baseline_x_ = x_;
      baseline_y_ = y_;
      fired_ = false;
      return;
    }

    const double stalled_for = (now - baseline_time_).seconds();
    if (stalled_for < context_.config.stall_timeout_s || fired_) {
      return;
    }

    // 只报一次：卡住会持续满足条件，靠 fired_ 取边沿，恢复推进或任务结束才重新武装
    fired_ = true;
    std::ostringstream oss;
    oss << "nav_stall_" << static_cast<int>(stalled_for) << "s";
    context_.fire(oss.str());
  }

  TriggerContext context_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
#ifdef MOBILE_BAG_RECORDER_HAS_BT_LOG
  rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>::SharedPtr
    feedback_sub_;
#endif
  rclcpp::TimerBase::SharedPtr timer_;

  bool use_feedback_{false};
  bool executing_{false};
  bool has_progress_{false};
  bool baseline_set_{false};
  bool fired_{false};

  double progress_value_{0.0};
  double x_{0.0}, y_{0.0};
  rclcpp::Time baseline_time_;
  double baseline_value_{0.0};
  double baseline_x_{0.0}, baseline_y_{0.0};
};

}  // namespace

std::unique_ptr<TriggerSource> makeNavStallSource()
{
  return std::make_unique<NavStallSource>();
}

}  // namespace mobile_bag_recorder
