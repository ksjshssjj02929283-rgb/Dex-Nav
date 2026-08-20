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

#ifndef NAV2_PID_CONTROLLER__END_LOCALIZATION_LOCK_HPP_
#define NAV2_PID_CONTROLLER__END_LOCALIZATION_LOCK_HPP_

#include <cstdint>
#include <mutex>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/end_localization.hpp"

namespace nav2_pid_controller
{

struct EndLocalizationLockConfig
{
  // 0表示关闭对应锁存通道。
  double switch_distance_m{0.0};
  double switch_angle_rad{0.0};
};

/**
 * @brief PID末端Odom锁存状态机。
 *
 * 类本身不查询TF、不订阅Odom，只消费同一控制周期的机器人位姿和权威终点，
 * 便于单元测试，也避免在控制热路径重复获取数据。
 */
class EndLocalizationLock
{
public:
  void configure(const EndLocalizationLockConfig & config);
  void reset(uint64_t goal_generation);
  void setLiveGoal(
    const geometry_msgs::msg::PoseStamped & live_goal,
    uint64_t goal_generation);

  // 位置控制前调用：必要时锁存目标位置，并返回本周期有效目标。
  nav2_core::EndLocalizationSnapshot updatePositionReference(
    const geometry_msgs::msg::PoseStamped & robot_pose);

  // PID阶段确定后调用：粗调阶段使用实时航向，精调阶段锁存Odom航向。
  nav2_core::EndLocalizationSnapshot updateYawReference(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    bool final_yaw_control);

  nav2_core::EndLocalizationSnapshot snapshot() const;

private:
  void resetLocked(uint64_t goal_generation);
  bool inputValidLocked(const geometry_msgs::msg::PoseStamped & robot_pose) const;
  void rebuildEffectiveGoalLocked(const geometry_msgs::msg::PoseStamped & robot_pose);

  mutable std::mutex mutex_;
  EndLocalizationLockConfig config_;
  nav2_core::EndLocalizationSnapshot snapshot_;
  geometry_msgs::msg::Point locked_position_;
  geometry_msgs::msg::Quaternion locked_orientation_;
  bool previous_final_yaw_control_{false};
};

}  // namespace nav2_pid_controller

#endif  // NAV2_PID_CONTROLLER__END_LOCALIZATION_LOCK_HPP_
