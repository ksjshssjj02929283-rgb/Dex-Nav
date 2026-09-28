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

#include "nav2_collision_monitor/scan.hpp"

#include <cmath>
#include <functional>
#include <utility>

namespace nav2_collision_monitor
{

Scan::Scan(
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
  RCLCPP_INFO(logger_, "[%s]: Creating Scan", source_name_.c_str());
}

Scan::~Scan()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying Scan", source_name_.c_str());
  data_sub_.reset();
}

void Scan::configure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string source_topic;

  // Laser scanner has no own parameters
  getCommonParameters(source_topic);

  rclcpp::QoS scan_qos = rclcpp::SensorDataQoS();  // set to default
  data_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    source_topic, scan_qos,
    std::bind(&Scan::dataCallback, this, std::placeholders::_1));
}

void Scan::getData(
  const rclcpp::Time & curr_time,
  std::vector<Point> & data) const
{
  const auto snapshot = std::atomic_load(&processed_data_);

  // A failed transform invalidates the entire newly received frame. In particular, do not keep
  // returning an older valid frame while TF for newer sensor data is broken.
  if (snapshot == nullptr || !snapshot->transform_valid) {
    return;
  }
  if (!sourceValid(snapshot->source_stamp, curr_time)) {
    return;
  }

  data.insert(data.end(), snapshot->points.begin(), snapshot->points.end());
}

void Scan::dataCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  last_source_stamp_ns_.store(
    rclcpp::Time(msg->header.stamp).nanoseconds(), std::memory_order_release);
  const auto receipt = std::chrono::steady_clock::now().time_since_epoch();
  last_received_steady_ns_.store(
    std::chrono::duration_cast<std::chrono::nanoseconds>(receipt).count(),
    std::memory_order_release);
  std::atomic_store(&data_, msg);
  auto processed = std::make_shared<ProcessedScan>();
  processed->source_stamp = msg->header.stamp;

  auto node = node_.lock();
  if (!node) {
    std::atomic_store(&processed_data_, std::shared_ptr<const ProcessedScan>(processed));
    data_sequence_.fetch_add(1, std::memory_order_release);
    return;
  }

  // Transform once per sensor frame. The 100 Hz control loop only reads cached points.
  // Keep the acquisition timestamp on the source leg; only the target base pose is latest.
  tf2::Transform tf_transform;
  if (!getLatestTransform(msg->header.frame_id, rclcpp::Time(msg->header.stamp), tf_transform)) {
    std::atomic_store(&processed_data_, std::shared_ptr<const ProcessedScan>(processed));
    data_sequence_.fetch_add(1, std::memory_order_release);
    return;
  }

  // Beam geometry normally remains constant for the lifetime of a scanner. Cache the exact
  // directions produced by the original float-angle algorithm, avoiding thousands of repeated
  // sin/cos calls on geometry-stable scans without changing their numerical result.
  auto directions = std::atomic_load(&direction_cache_);
  if (!directions || directions->angle_min != msg->angle_min ||
    directions->angle_increment != msg->angle_increment ||
    directions->size != msg->ranges.size())
  {
    auto updated = std::make_shared<DirectionCache>();
    updated->angle_min = msg->angle_min;
    updated->angle_increment = msg->angle_increment;
    updated->size = msg->ranges.size();
    updated->unit_vectors.reserve(updated->size);
    float angle = msg->angle_min;
    for (std::size_t i = 0; i < updated->size; ++i) {
      updated->unit_vectors.push_back({std::cos(angle), std::sin(angle)});
      angle += msg->angle_increment;
    }
    directions = updated;
    std::atomic_store(&direction_cache_, std::shared_ptr<const DirectionCache>(updated));
  }

  processed->points.reserve(msg->ranges.size());
  for (size_t i = 0; i < msg->ranges.size(); i++) {
    if (msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max) {
      // Transform point coordinates from source frame -> to base frame
      tf2::Vector3 p_v3_s(
        msg->ranges[i] * directions->unit_vectors[i].x,
        msg->ranges[i] * directions->unit_vectors[i].y,
        0.0);
      tf2::Vector3 p_v3_b = tf_transform * p_v3_s;

      // Refill data array
      processed->points.push_back({p_v3_b.x(), p_v3_b.y()});
    }
  }
  processed->transform_valid = true;
  std::atomic_store(&processed_data_, std::shared_ptr<const ProcessedScan>(processed));
  data_sequence_.fetch_add(1, std::memory_order_release);
}

}  // namespace nav2_collision_monitor
