// Copyright (c) 2026 dexforce
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

#ifndef NAV2_CORE__END_LOCALIZATION_HPP_
#define NAV2_CORE__END_LOCALIZATION_HPP_

#include <cstdint>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_core
{

/**
 * @brief 末端定位参考所处阶段。
 *
 * 位置接近目标后可在Odom中锁存位置；最终转角按控制器配置保留锁存或跟随实时位置。
 * 航向先使用实时map参考粗调，进入小角度范围后再锁存Odom航向做精调。
 * ControllerServer同时用该阶段保持PID控制权；两个锁存布尔量只描述参考来源。
 */
enum class EndLocalizationStage
{
  INACTIVE,
  ODOM_POSITION,
  MAP_YAW_COARSE,
  ODOM_YAW_FINE
};

/**
 * @brief 末端定位参考的只读快照。
 */
struct EndLocalizationSnapshot
{
  uint64_t goal_generation{0};
  uint64_t revision{0};
  EndLocalizationStage stage{EndLocalizationStage::INACTIVE};

  bool enabled{false};
  bool live_goal_valid{false};
  bool effective_goal_valid{false};
  bool position_goal_latched{false};
  bool yaw_goal_latched{false};

  geometry_msgs::msg::PoseStamped live_goal;
  geometry_msgs::msg::PoseStamped effective_goal;

  double live_position_error{0.0};
  double live_yaw_error{0.0};
  double effective_position_error{0.0};
  double effective_yaw_error{0.0};
  double position_correction_after_latch{0.0};
  double yaw_correction_after_latch{0.0};
};

/**
 * @brief 控制器可选能力：接收权威终点并维护末端定位参考。
 *
 * 独立接口避免修改Controller基类虚表。不支持该能力的控制器保持原行为。
 */
class EndLocalizationCapability
{
public:
  virtual ~EndLocalizationCapability() = default;

  // 新目标、取消或生命周期退出时清空旧目标状态。
  virtual void resetEndLocalization(uint64_t goal_generation) = 0;

  // ControllerServer在控制前传入已转换到当前控制坐标系的权威终点。
  virtual void setEndLocalizationGoal(
    const geometry_msgs::msg::PoseStamped & live_goal,
    uint64_t goal_generation) = 0;

  // 返回线程安全副本；调用方必须校验goal_generation。
  virtual EndLocalizationSnapshot getEndLocalizationSnapshot() const = 0;
};

inline const char * toString(EndLocalizationStage stage)
{
  switch (stage) {
    case EndLocalizationStage::INACTIVE:
      return "INACTIVE";
    case EndLocalizationStage::ODOM_POSITION:
      return "ODOM_POSITION";
    case EndLocalizationStage::MAP_YAW_COARSE:
      return "MAP_YAW_COARSE";
    case EndLocalizationStage::ODOM_YAW_FINE:
      return "ODOM_YAW_FINE";
  }
  return "UNKNOWN";
}

}  // namespace nav2_core

#endif  // NAV2_CORE__END_LOCALIZATION_HPP_
