// Copyright (c) 2022 Samsung R&D Institute Russia
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_collision_monitor/range.hpp"

#include <math.h>
#include <cmath>
#include <algorithm>
#include <functional>
#include <utility>

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

Range::Range(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & source_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const std::string & global_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout,
  const bool base_shift_correction)
: Source(
    node, source_name, tf_buffer, base_frame_id, global_frame_id,
    transform_tolerance, source_timeout, base_shift_correction),
  outlier_filter_(std::make_unique<RangeFilter>(5, 0.30)),
  data_(nullptr)
{
  RCLCPP_INFO(logger_, "[%s]: Creating Range", source_name_.c_str());
}

Range::~Range()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying Range", source_name_.c_str());
  data_sub_.reset();
}

void Range::configure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string source_topic;

  getParameters(source_topic);

  rclcpp::QoS range_qos = rclcpp::SensorDataQoS();  // set to default
  data_sub_ = node->create_subscription<sensor_msgs::msg::Range>(
    source_topic, range_qos,
    std::bind(&Range::dataCallback, this, std::placeholders::_1));
}

void Range::getData(
  const rclcpp::Time & curr_time,
  std::vector<Point> & data) const
{
  // Keep one immutable message alive throughout TF lookup and point conversion.
  const auto snapshot = std::atomic_load(&data_);

  // Ignore data from the source if it is not being published yet or
  // not being published for a long time
  if (snapshot == nullptr) {
    return;
  }
  if (!sourceValid(snapshot->header.stamp, curr_time)) {
    return;
  }

  // Ignore data, if its range is out of scope of range sensor abilities
  const float effective_max_range = (max_detection_range_ > 0.0) ?
    std::min(static_cast<float>(max_detection_range_), snapshot->max_range) :
    snapshot->max_range;

  if (snapshot->range < snapshot->min_range || snapshot->range > effective_max_range) {
    RCLCPP_DEBUG(
      logger_,
      "[%s]: Data range %fm is out of {%f..%f} sensor span. Ignoring...",
      source_name_.c_str(), snapshot->range, snapshot->min_range, effective_max_range);
    return;
  }

  tf2::Transform tf_transform;
  if (!getLatestTransform(
      snapshot->header.frame_id, rclcpp::Time(snapshot->header.stamp), tf_transform))
  {
    return;
  }

  // Calculate poses and refill data array
  float angle;
  for (
    angle = -snapshot->field_of_view / 2;
    angle < snapshot->field_of_view / 2;
    angle += obstacles_angle_)
  {
    // Transform point coordinates from source frame -> to base frame
    tf2::Vector3 p_v3_s(
      snapshot->range * std::cos(angle),
      snapshot->range * std::sin(angle),
      0.0);
    tf2::Vector3 p_v3_b = tf_transform * p_v3_s;

    // Refill data array
    data.push_back({p_v3_b.x(), p_v3_b.y()});
  }

  // Make sure that last (field_of_view / 2) point will be in the data array
  angle = snapshot->field_of_view / 2;

  // Transform point coordinates from source frame -> to base frame
  tf2::Vector3 p_v3_s(
    snapshot->range * std::cos(angle),
    snapshot->range * std::sin(angle),
    0.0);
  tf2::Vector3 p_v3_b = tf_transform * p_v3_s;

  // Refill data array
  data.push_back({p_v3_b.x(), p_v3_b.y()});

  // 过程调试信息设为 DEBUG 级别，避免高频日志洪泛
  RCLCPP_DEBUG(
    logger_,
    "[%s]: Hit confirmed! range=%.3fm, points_count=%zu, sample_pt=(%.3f, %.3f)",
    source_name_.c_str(), snapshot->range, data.size(), data[0].x, data[0].y);
}

void Range::getParameters(std::string & source_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  getCommonParameters(source_topic);

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".obstacles_angle", rclcpp::ParameterValue(M_PI / 180));
  obstacles_angle_ = node->get_parameter(source_name_ + ".obstacles_angle").as_double();

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".max_detection_range", rclcpp::ParameterValue(0.0));
  max_detection_range_ = node->get_parameter(source_name_ + ".max_detection_range").as_double();

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".filter_outliers", rclcpp::ParameterValue(true));
  filter_outliers_ = node->get_parameter(source_name_ + ".filter_outliers").as_bool();

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".outlier_filter_window_size", rclcpp::ParameterValue(5));
  outlier_filter_window_size_ =
    node->get_parameter(source_name_ + ".outlier_filter_window_size").as_int();
  if (outlier_filter_window_size_ < 1) {
    outlier_filter_window_size_ = 1;
  }

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".max_jump_distance", rclcpp::ParameterValue(0.30));
  max_jump_distance_ = node->get_parameter(source_name_ + ".max_jump_distance").as_double();

  if (outlier_filter_) {
    outlier_filter_->setWindowSize(outlier_filter_window_size_);
    outlier_filter_->setMaxJumpDistance(max_jump_distance_);
  }

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".stop_only", rclcpp::ParameterValue(true));
  stop_only_ = node->get_parameter(source_name_ + ".stop_only").as_bool();

  nav2_util::declare_parameter_if_not_declared(
    node, "enable_range_slowdown", rclcpp::ParameterValue(false));
  if (node->get_parameter("enable_range_slowdown").as_bool()) {
    stop_only_ = false;
  }

  RCLCPP_INFO(
    logger_,
    "[%s]: Range configured: stop_only=%s, max_detection_range=%.2fm, outlier_filter=%s(win=%d, max_jump=%.2fm)",
    source_name_.c_str(), (stop_only_ ? "true" : "false (slowdown enabled)"),
    max_detection_range_,
    (filter_outliers_ ? "true" : "false"), outlier_filter_window_size_, max_jump_distance_);
}

void Range::dataCallback(sensor_msgs::msg::Range::ConstSharedPtr msg)
{
  last_source_stamp_ns_.store(
    rclcpp::Time(msg->header.stamp).nanoseconds(), std::memory_order_release);
  const auto receipt = std::chrono::steady_clock::now().time_since_epoch();
  last_received_steady_ns_.store(
    std::chrono::duration_cast<std::chrono::nanoseconds>(receipt).count(),
    std::memory_order_release);

  // 1. 基本有效性检查（非 NaN / 非 Inf / 大于 0）
  if (std::isnan(msg->range) || std::isinf(msg->range) || msg->range <= 0.0f) {
    if (outlier_filter_) {
      outlier_filter_->reset();
    }
    return;
  }

  // 2. 离群点滤波处理（使用独立的 RangeFilter 类进行滑动中值与脉冲跳变抑制）
  float filtered_range = msg->range;
  if (filter_outliers_ && outlier_filter_) {
    bool is_outlier = false;
    filtered_range = outlier_filter_->update(msg->range, is_outlier);
    if (is_outlier) {
      RCLCPP_DEBUG(
        logger_,
        "[%s]: Spike outlier suppressed: raw=%.3fm, filtered=%.3fm (diff=%.3fm > %.3fm)",
        source_name_.c_str(), msg->range, filtered_range,
        std::abs(msg->range - filtered_range), max_jump_distance_);
    }
  }

  // 3. 量程门限判定（严格使用 topic 中自带的 min_range）
  if (msg->range < msg->min_range) {
    return;
  }

  // 4. 过程调试：记录近距离回波。已取消连续命中门槛，单帧有效即参与决策。
  if (filtered_range <= 0.25f) {
    RCLCPP_DEBUG(
      logger_,
      "[%s]: Obstacle in range: dist=%.3fm",
      source_name_.c_str(), filtered_range);
  }

  // 5. 保存滤波后的数据快照
  auto filtered_msg = std::make_shared<sensor_msgs::msg::Range>(*msg);
  filtered_msg->range = filtered_range;
  sensor_msgs::msg::Range::ConstSharedPtr const_filtered_msg = std::move(filtered_msg);
  std::atomic_store(&data_, std::move(const_filtered_msg));
  data_sequence_.fetch_add(1, std::memory_order_release);
}

}  // namespace nav2_collision_monitor
