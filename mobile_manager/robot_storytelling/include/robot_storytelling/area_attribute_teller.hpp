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

#ifndef ROBOT_STORYTELLING__AREA_ATTRIBUTE_TELLER_HPP_
#define ROBOT_STORYTELLING__AREA_ATTRIBUTE_TELLER_HPP_

#include <string>
#include <vector>

#include "robot_storytelling/teller_base.hpp"

namespace robot_storytelling
{

// 区域属性 teller（原 story_evaluator.cpp，2026-07-14 重命名 + 迁入插件框架）。
// 覆盖的语义事件都共享同一个计算基础——odom 点 与 Area/Room 多边形的几何
// 关系（point-in-polygon / 质心距离），这一点与路网属性 teller
// （RouteAttributeTeller，odom 与 RouteGraph 边的最近距离）计算基础不同，
// 因此拆成两个独立 teller，而不是把两者塞进同一个模块。
// MVP 三个事件：IN_ROOM / IN_SPEED_LIMIT / CLOSE_TO_CHARGER。
class AreaAttributeTeller : public TellerBase
{
public:
  void Init(rclcpp::Node * node) override;
  void Update(
    const TellerContext & ctx,
    std::vector<nav_interfaces::msg::Story> * stories) override;
  std::string Name() const override {return "AreaAttributeTeller";}

private:
  double charger_radius_{1.0};
};

// 纯函数：评估 (rx, ry) 触发的区域属性语义事件。与 ROS 节点无耦合，方便单测。
std::vector<nav_interfaces::msg::Story> evaluateAreaStories(
  double rx, double ry,
  const std::vector<nav_interfaces::msg::AreaInfo> & areas,
  const std::vector<nav_interfaces::msg::RoomInfo> & rooms,
  double charger_radius);

// 2D 点在多边形内判定（射线法）。polygon 是 geometry_msgs/Pose2D[]。
bool pointInPolygon(
  double px, double py,
  const std::vector<geometry_msgs::msg::Pose2D> & polygon);

}  // namespace robot_storytelling

#endif  // ROBOT_STORYTELLING__AREA_ATTRIBUTE_TELLER_HPP_
