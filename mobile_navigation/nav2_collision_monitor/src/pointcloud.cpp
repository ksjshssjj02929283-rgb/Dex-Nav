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

#include "nav2_collision_monitor/pointcloud.hpp"

#include <chrono>
#include <functional>
#include <utility>

#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

PointCloud::PointCloud(
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
  data_(nullptr), processed_data_(nullptr)
{
  RCLCPP_INFO(logger_, "[%s]: Creating PointCloud", source_name_.c_str());
}

PointCloud::~PointCloud()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying PointCloud", source_name_.c_str());
  data_sub_.reset();
}

void PointCloud::configure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string source_topic;

  getParameters(source_topic);

  rclcpp::QoS pointcloud_qos = rclcpp::SensorDataQoS();  // set to default
  data_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    source_topic, pointcloud_qos,
    std::bind(&PointCloud::dataCallback, this, std::placeholders::_1));
}

void PointCloud::getData(
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

void PointCloud::getParameters(std::string & source_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  getCommonParameters(source_topic);

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".min_height", rclcpp::ParameterValue(0.05));
  min_height_ = node->get_parameter(source_name_ + ".min_height").as_double();
  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".max_height", rclcpp::ParameterValue(0.5));
  max_height_ = node->get_parameter(source_name_ + ".max_height").as_double();
}

void PointCloud::dataCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  last_source_stamp_ns_.store(
    rclcpp::Time(msg->header.stamp).nanoseconds(), std::memory_order_release);
  const auto receipt = std::chrono::steady_clock::now().time_since_epoch();
  const auto receipt_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(receipt).count();
  last_received_steady_ns_.store(receipt_ns, std::memory_order_release);
  std::atomic_store(&data_, msg);

  auto processed = std::make_shared<ProcessedPointCloud>();
  processed->source_stamp = msg->header.stamp;

  auto node = node_.lock();
  if (!node) {
    std::atomic_store(&processed_data_, std::shared_ptr<const ProcessedPointCloud>(processed));
    data_sequence_.fetch_add(1, std::memory_order_release);
    return;
  }

  tf2::Transform tf_transform;
  if (!getLatestTransform(msg->header.frame_id, rclcpp::Time(msg->header.stamp), tf_transform)) {
    std::atomic_store(&processed_data_, std::shared_ptr<const ProcessedPointCloud>(processed));
    data_sequence_.fetch_add(1, std::memory_order_release);
    return;
  }

  // 坐标转换成功，记录有效帧单调时间戳 (A3 规范)
  last_valid_frame_steady_ns_.store(receipt_ns, std::memory_order_release);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  // Refill data array with PointCloud points in base frame
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    // Transform point coordinates from source frame -> to base frame
    tf2::Vector3 p_v3_s(*iter_x, *iter_y, *iter_z);
    tf2::Vector3 p_v3_b = tf_transform * p_v3_s;

    if (p_v3_b.z() >= min_height_ && p_v3_b.z() <= max_height_) {
      const double px = p_v3_b.x();
      const double py = p_v3_b.y();
      // 距离绝对剪枝：超出 max_obstacle_distance_sq_ 的远端点直接剔除
      if (px * px + py * py <= max_obstacle_distance_sq_) {
        processed->points.push_back({px, py});
      }
    }
  }

  processed->transform_valid = true;
  std::atomic_store(&processed_data_, std::shared_ptr<const ProcessedPointCloud>(processed));
  data_sequence_.fetch_add(1, std::memory_order_release);
}

}  // namespace nav2_collision_monitor
