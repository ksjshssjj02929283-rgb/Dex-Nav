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
  data_(nullptr), processed_data_(nullptr)
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
  const auto snapshot = std::atomic_load(&processed_data_);

  // A failed transform invalidates the newly received frame
  if (snapshot == nullptr || !snapshot->transform_valid) {
    return;
  }
  if (!sourceValid(snapshot->source_stamp, curr_time)) {
    return;
  }

  data.insert(data.end(), snapshot->points.begin(), snapshot->points.end());
}

void Range::getParameters(std::string & source_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  getCommonParameters(source_topic);

  // ── 超声波探头参数化配置 ──
  // 1. obstacles_angle: 障碍物投影锥角采样分辨率（弧度，默认 3 度一个点以减少计算点数）
  // 2. max_detection_range: 有效探测距离截断（米），0.0 表示不限制；截断可防止超声波探测到远距离地杂波
  // 3. filter_outliers / outlier_filter_window_size / max_jump_distance / low_pass_alpha: 离群点滑动中值滤波与低通平滑
  // 4. stop_only / enable_range_slowdown: 点云分流控制（控制该探头是仅用于 STOP 停障，还是允许汇入 SLOWDOWN 减速多边形）
  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".obstacles_angle", rclcpp::ParameterValue(3.0 * M_PI / 180.0));
  obstacles_angle_ = node->get_parameter(source_name_ + ".obstacles_angle").as_double();
  if (obstacles_angle_ <= 0.0) {
    obstacles_angle_ = 3.0 * M_PI / 180.0;
  }

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".max_detection_range", rclcpp::ParameterValue(0.0));
  max_detection_range_ = node->get_parameter(source_name_ + ".max_detection_range").as_double();

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".filter_outliers", rclcpp::ParameterValue(false));
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

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".low_pass_alpha", rclcpp::ParameterValue(0.5));
  low_pass_alpha_ = node->get_parameter(source_name_ + ".low_pass_alpha").as_double();

  if (outlier_filter_) {
    outlier_filter_->setWindowSize(outlier_filter_window_size_);
    outlier_filter_->setMaxJumpDistance(max_jump_distance_);
    outlier_filter_->setLowPassAlpha(low_pass_alpha_);
  }

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".stop_only", rclcpp::ParameterValue(false));
  stop_only_ = node->get_parameter(source_name_ + ".stop_only").as_bool();

  nav2_util::declare_parameter_if_not_declared(
    node, "enable_range_slowdown", rclcpp::ParameterValue(false));
  if (node->get_parameter("enable_range_slowdown").as_bool()) {
    stop_only_ = false;
  }

  RCLCPP_INFO(
    logger_,
    "[%s]: Range configured: stop_only=%s, max_detection_range=%.2fm, outlier_filter=%s(win=%d, max_jump=%.2fm, lpf_alpha=%.2f)",
    source_name_.c_str(), (stop_only_ ? "true" : "false (slowdown enabled)"),
    max_detection_range_,
    (filter_outliers_ ? "true" : "false"), outlier_filter_window_size_, max_jump_distance_, low_pass_alpha_);
}

void Range::dataCallback(sensor_msgs::msg::Range::ConstSharedPtr msg)
{
  last_source_stamp_ns_.store(
    rclcpp::Time(msg->header.stamp).nanoseconds(), std::memory_order_release);
  const auto receipt = std::chrono::steady_clock::now().time_since_epoch();
  const auto receipt_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(receipt).count();
  last_received_steady_ns_.store(receipt_ns, std::memory_order_release);

  // 1. 基本有效性检查（非 NaN / 非 Inf / 大于 0）
  if (std::isnan(msg->range) || std::isinf(msg->range) || msg->range <= 0.0f) {
    if (outlier_filter_) {
      outlier_filter_->reset();
    }
    return;
  }

  // 2. 离群点滤波处理（使用独立的 RangeFilter 类进行滑动中值与脉冲跳变抑制）
  // 抑制孤立脉冲跳变、假回波与地面接缝反射，避免底盘偶发误刹车
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

  // 3. 量程门限判定（严格使用 topic 中自带的 min_range 过滤探头近场盲区）
  if (msg->range < msg->min_range) {
    return;
  }

  // 4. 过程调试：记录近距离回波。取消连续命中门槛，单帧滤波有效后即刻参与停障决策，确保高响应度
  if (filtered_range <= 0.25f) {
    RCLCPP_DEBUG(
      logger_,
      "[%s]: Obstacle in range: dist=%.3fm",
      source_name_.c_str(), filtered_range);
  }

  // 5. 保存滤波后的数据快照，提供无锁线程安全访问
  auto filtered_msg = std::make_shared<sensor_msgs::msg::Range>(*msg);
  filtered_msg->range = filtered_range;
  sensor_msgs::msg::Range::ConstSharedPtr const_filtered_msg = std::move(filtered_msg);
  std::atomic_store(&data_, const_filtered_msg);

  // 6. 构造处理后的 base 坐标系点云快照
  auto processed = std::make_shared<ProcessedRange>();
  processed->source_stamp = msg->header.stamp;

  auto node = node_.lock();
  if (!node) {
    std::atomic_store(&processed_data_, std::shared_ptr<const ProcessedRange>(processed));
    data_sequence_.fetch_add(1, std::memory_order_release);
    return;
  }

  // 量程截断判定（包含最大探测距离截断 max_detection_range_）
  const float effective_max_range = (max_detection_range_ > 0.0) ?
    std::min(static_cast<float>(max_detection_range_), msg->max_range) :
    msg->max_range;

  if (filtered_range < msg->min_range || filtered_range > effective_max_range) {
    // 距离超出探测范围，点集为空，但帧状态有效
    processed->transform_valid = true;
    std::atomic_store(&processed_data_, std::shared_ptr<const ProcessedRange>(processed));
    data_sequence_.fetch_add(1, std::memory_order_release);
    return;
  }

  tf2::Transform tf_transform;
  if (!getLatestTransform(msg->header.frame_id, rclcpp::Time(msg->header.stamp), tf_transform)) {
    std::atomic_store(&processed_data_, std::shared_ptr<const ProcessedRange>(processed));
    data_sequence_.fetch_add(1, std::memory_order_release);
    return;
  }

  last_valid_frame_steady_ns_.store(receipt_ns, std::memory_order_release);

  // 生成扇形角度点云并转换至 base 坐标系
  float angle;
  for (
    angle = -msg->field_of_view / 2;
    angle < msg->field_of_view / 2;
    angle += obstacles_angle_)
  {
    tf2::Vector3 p_v3_s(
      filtered_range * std::cos(angle),
      filtered_range * std::sin(angle),
      0.0);
    tf2::Vector3 p_v3_b = tf_transform * p_v3_s;
    const double px = p_v3_b.x();
    const double py = p_v3_b.y();
    if (px * px + py * py <= max_obstacle_distance_sq_) {
      processed->points.push_back({px, py});
    }
  }

  // 补上边界角点
  angle = msg->field_of_view / 2;
  tf2::Vector3 p_v3_s(
    filtered_range * std::cos(angle),
    filtered_range * std::sin(angle),
    0.0);
  tf2::Vector3 p_v3_b = tf_transform * p_v3_s;
  const double px = p_v3_b.x();
  const double py = p_v3_b.y();
  if (px * px + py * py <= max_obstacle_distance_sq_) {
    processed->points.push_back({px, py});
  }

  processed->transform_valid = true;
  std::atomic_store(&processed_data_, std::shared_ptr<const ProcessedRange>(processed));
  data_sequence_.fetch_add(1, std::memory_order_release);
}

}  // namespace nav2_collision_monitor
