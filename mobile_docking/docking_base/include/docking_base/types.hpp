// Copyright (c) 2024 Open Navigation LLC
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

#ifndef DOCKING_BASE__TYPES_HPP_
#define DOCKING_BASE__TYPES_HPP_

#include <string>

#include "docking_base_msgs/action/dock_robot.hpp"
#include "docking_base_msgs/action/undock_robot.hpp"
#include "docking_base_core/docking_exceptions.hpp"

// DEC-014：移除 Dock 结构 / ChargingDock / DockPluginMap / DockMap（DockDatabase 已删）。
// dock 入口改为 DockRobot goal 的 dock_pose + dock_type，不再用 dock 实例/插件数据库。
typedef docking_base_msgs::action::DockRobot DockRobot;
typedef docking_base_msgs::action::UndockRobot UndockRobot;

#endif  // DOCKING_BASE__TYPES_HPP_
