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

#include "nav2_collision_monitor/collision_monitor_node.hpp"

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace nav2_collision_monitor
{

void CollisionMonitor::publishPathStopMarkers(
  const std::vector<Point> & check_points) const
{
  const rclcpp::Time stamp = this->now();
  if (path_stop_marker_min_interval_ > 0.0) {
    const double elapsed = (stamp - last_path_stop_marker_pub_time_).seconds();
    if (elapsed < path_stop_marker_min_interval_ && elapsed >= 0.0) {
      return;
    }
    last_path_stop_marker_pub_time_ = stamp;
  }

  auto markers = std::make_unique<visualization_msgs::msg::MarkerArray>();

  // Clear markers left by a previously longer path before publishing the current set.
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  markers->markers.push_back(clear_marker);

  constexpr std::size_t circle_segments = 32;
  for (std::size_t i = 0; i < check_points.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_id_;
    marker.header.stamp = stamp;
    marker.ns = "path_stop_circles";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.color.r = 1.0f;
    marker.color.g = 0.1f;
    marker.color.b = 0.0f;
    marker.color.a = 0.8f;
    marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    marker.points.reserve(circle_segments + 1);
    for (std::size_t segment = 0; segment <= circle_segments; ++segment) {
      const double angle = 2.0 * M_PI * segment / circle_segments;
      geometry_msgs::msg::Point point;
      point.x = check_points[i].x + path_stop_radius_ * std::cos(angle);
      point.y = check_points[i].y + path_stop_radius_ * std::sin(angle);
      point.z = 0.02;
      marker.points.push_back(point);
    }
    markers->markers.push_back(std::move(marker));
  }

  path_stop_marker_pub_->publish(std::move(markers));
}

void CollisionMonitor::publishUltrasonicPointCloud(
  const rclcpp::Time & curr_time,
  const std::vector<Point> & points) const
{
  if (!ultrasonic_points_pub_ || ultrasonic_points_pub_->get_subscription_count() == 0) {
    return;
  }

  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  msg->header.stamp = curr_time;
  msg->header.frame_id = base_frame_id_;

  sensor_msgs::PointCloud2Modifier modifier(*msg);
  modifier.setPointCloud2Fields(
    3,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

  for (const auto & p : points) {
    *iter_x = static_cast<float>(p.x);
    *iter_y = static_cast<float>(p.y);
    *iter_z = 0.0f;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  ultrasonic_points_pub_->publish(std::move(msg));
}

void CollisionMonitor::publishPolygons() const
{
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    polygon->publish();
  }
}

}  // namespace nav2_collision_monitor
