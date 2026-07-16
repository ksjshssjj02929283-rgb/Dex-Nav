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

#ifndef ROBOT_STORYTELLING__TELLER_BASE_HPP_
#define ROBOT_STORYTELLING__TELLER_BASE_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_interfaces/msg/area_info.hpp"
#include "nav_interfaces/msg/room_info.hpp"
#include "nav_interfaces/msg/story.hpp"

namespace robot_storytelling
{

// 每周期共享给所有 teller 的通用输入（由 StorytellingNode 统一订阅一次）。
// teller 若需要额外输入（如 RouteAttributeTeller 的 RouteGraph），在自己的
// Init(node) 里建立自己的 subscription/client，存成成员变量，不塞进这里
// ——避免为单个 teller 的特殊输入污染共享 context。
struct TellerContext
{
  double robot_x{0.0};
  double robot_y{0.0};
  const std::vector<nav_interfaces::msg::AreaInfo> * areas{nullptr};
  const std::vector<nav_interfaces::msg::RoomInfo> * rooms{nullptr};
};

// Teller 插件接口（对齐 Apollo modules/storytelling/story_tellers/base_teller.h
// 的 BaseTeller：Init(config) + Update(stories)）。
// 每个 teller 是一个独立的语义事件检测单元；新增 teller 只需新建一对
// .hpp/.cpp 实现本接口 + ROBOT_STORYTELLER_REGISTER 宏自注册，不改动
// StorytellingNode 或其它 teller。
class TellerBase
{
public:
  virtual ~TellerBase() = default;

  // 可选：读取自己的 ROS2 参数、建立自己专属的 subscription/client。
  // 默认空实现——不需要额外输入的 teller（如 AreaAttributeTeller）可以不覆写。
  virtual void Init(rclcpp::Node * node) {(void)node;}

  // 每周期调用一次。往 stories 里 push_back 本 teller 检测到的事件；
  // 检测不到任何事件时不追加（不是"发一个空事件"）。
  virtual void Update(
    const TellerContext & ctx,
    std::vector<nav_interfaces::msg::Story> * stories) = 0;

  // teller 名字，用于日志和 enabled_tellers 参数里的标识（与类名一致）。
  virtual std::string Name() const = 0;
};

}  // namespace robot_storytelling

#endif  // ROBOT_STORYTELLING__TELLER_BASE_HPP_
