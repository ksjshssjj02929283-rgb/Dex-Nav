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

#ifndef ROBOT_STORYTELLING__STORYTELLING_NODE_HPP_
#define ROBOT_STORYTELLING__STORYTELLING_NODE_HPP_

#include <optional>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_interfaces/msg/area_info_array.hpp"
#include "nav_interfaces/msg/room_info_array.hpp"
#include "nav_interfaces/msg/robot_stories.hpp"

namespace robot_storytelling
{

// 共享状态仅由单线程 executor（rclcpp::spin）上的回调与定时器访问，无锁安全。
// 若改用 MultiThreadedExecutor 需加锁。
class StorytellingNode : public rclcpp::Node
{
public:
  explicit StorytellingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onAreas(const nav_interfaces::msg::AreaInfoArray::SharedPtr msg);
  void onRooms(const nav_interfaces::msg::RoomInfoArray::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onTimer();

  double charger_radius_{1.0};

  std::vector<nav_interfaces::msg::AreaInfo> areas_;
  std::vector<nav_interfaces::msg::RoomInfo> rooms_;
  std::optional<std::pair<double, double>> odom_xy_;

  rclcpp::Subscription<nav_interfaces::msg::AreaInfoArray>::SharedPtr areas_sub_;
  rclcpp::Subscription<nav_interfaces::msg::RoomInfoArray>::SharedPtr rooms_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_interfaces::msg::RobotStories>::SharedPtr stories_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robot_storytelling

#endif  // ROBOT_STORYTELLING__STORYTELLING_NODE_HPP_
