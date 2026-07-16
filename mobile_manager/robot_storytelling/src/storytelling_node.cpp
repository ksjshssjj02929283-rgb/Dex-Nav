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
#include <sstream>
#include <string>

#include "robot_storytelling/teller_registry.hpp"

// 各 teller 的 .cpp（含 ROBOT_STORYTELLER_REGISTER 静态注册对象）与本文件
// 编译进同一个 storytelling_core 共享库（见 CMakeLists.txt），确保注册
// 对象的静态初始化一定会随库加载执行——不能把某个 teller 单独编成静态库
// 又不被其它代码直接引用符号，否则链接器可能整个丢弃该编译单元。

namespace robot_storytelling
{

StorytellingNode::StorytellingNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("robot_storytelling", options)
{
  const double rate_hz = declare_parameter("rate_hz", 10.0);
  const std::string areas_topic =
    declare_parameter<std::string>("areas_topic", "/map_asset_manager/areas");
  const std::string rooms_topic =
    declare_parameter<std::string>("rooms_topic", "/map_asset_manager/rooms");
  const std::string odom_topic =
    declare_parameter<std::string>("odom_topic", "/feedback/odom");
  const std::string stories_topic =
    declare_parameter<std::string>("stories_topic", "/robot_stories");
  const std::vector<std::string> enabled_tellers = declare_parameter<std::vector<std::string>>(
    "enabled_tellers", {"AreaAttributeTeller", "RouteAttributeTeller"});

  for (const auto & name : enabled_tellers) {
    auto teller = TellerRegistry::Instance().Create(name);
    if (!teller) {
      RCLCPP_ERROR(
        get_logger(),
        "enabled_tellers 里的 '%s' 未注册（检查类名拼写 / 是否忘记链接该 teller 的编译单元），跳过",
        name.c_str());
      continue;
    }
    teller->Init(this);
    RCLCPP_INFO(get_logger(), "teller 已启用: %s", teller->Name().c_str());
    tellers_.push_back(std::move(teller));
  }

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

  std::ostringstream teller_names;
  for (const auto & t : tellers_) {
    teller_names << t->Name() << " ";
  }
  RCLCPP_INFO(
    get_logger(),
    "robot_storytelling up. tellers=[ %s] areas=%s rooms=%s odom=%s -> stories=%s (%.1fHz)",
    teller_names.str().c_str(), areas_topic.c_str(), rooms_topic.c_str(), odom_topic.c_str(),
    stories_topic.c_str(), rate_hz);
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

  TellerContext ctx;
  ctx.robot_x = odom_xy_->first;
  ctx.robot_y = odom_xy_->second;
  ctx.areas = &areas_;
  ctx.rooms = &rooms_;

  nav_interfaces::msg::RobotStories out;
  out.header.stamp = now();
  for (const auto & teller : tellers_) {
    teller->Update(ctx, &out.stories);
  }
  stories_pub_->publish(out);
}

}  // namespace robot_storytelling
