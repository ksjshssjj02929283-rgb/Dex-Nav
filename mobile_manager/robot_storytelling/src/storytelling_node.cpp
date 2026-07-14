// Copyright 2026 DEXE Robot Engineer
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

#include "robot_storytelling/storytelling_node.hpp"

#include <chrono>
#include <string>

#include "robot_storytelling/story_evaluator.hpp"

namespace robot_storytelling
{

StorytellingNode::StorytellingNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("robot_storytelling", options)
{
  const double rate_hz = declare_parameter("rate_hz", 10.0);
  charger_radius_ = declare_parameter("charger_radius", 1.0);
  const std::string areas_topic =
    declare_parameter<std::string>("areas_topic", "/map_asset_manager/areas");
  const std::string rooms_topic =
    declare_parameter<std::string>("rooms_topic", "/map_asset_manager/rooms");
  const std::string odom_topic =
    declare_parameter<std::string>("odom_topic", "/feedback/odom");
  const std::string stories_topic =
    declare_parameter<std::string>("stories_topic", "/robot_stories");

  rclcpp::QoS latched(rclcpp::KeepLast(1));
  latched.transient_local().reliable();

  areas_sub_ = create_subscription<nav_interfaces::msg::AreaInfoArray>(
    areas_topic, latched,
    [this](const nav_interfaces::msg::AreaInfoArray::SharedPtr m) {onAreas(m);});
  rooms_sub_ = create_subscription<nav_interfaces::msg::RoomInfoArray>(
    rooms_topic, latched,
    [this](const nav_interfaces::msg::RoomInfoArray::SharedPtr m) {onRooms(m);});
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr m) {onOdom(m);});

  stories_pub_ = create_publisher<nav_interfaces::msg::RobotStories>(
    stories_topic, rclcpp::QoS(10));

  const auto period = std::chrono::duration<double>(1.0 / rate_hz);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() {onTimer();});

  RCLCPP_INFO(
    get_logger(),
    "robot_storytelling up. areas=%s rooms=%s odom=%s -> stories=%s (%.1fHz, charger_r=%.2f)",
    areas_topic.c_str(), rooms_topic.c_str(), odom_topic.c_str(),
    stories_topic.c_str(), rate_hz, charger_radius_);
}

void StorytellingNode::onAreas(const nav_interfaces::msg::AreaInfoArray::SharedPtr msg)
{
  areas_ = msg->areas;
}

void StorytellingNode::onRooms(const nav_interfaces::msg::RoomInfoArray::SharedPtr msg)
{
  rooms_ = msg->rooms;
}

void StorytellingNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_xy_ = std::make_pair(msg->pose.pose.position.x, msg->pose.pose.position.y);
}

void StorytellingNode::onTimer()
{
  if (!odom_xy_) {
    return;
  }
  nav_interfaces::msg::RobotStories out;
  out.header.stamp = now();
  out.stories = evaluateStories(
    odom_xy_->first, odom_xy_->second, areas_, rooms_, charger_radius_);
  stories_pub_->publish(out);
}

}  // namespace robot_storytelling
