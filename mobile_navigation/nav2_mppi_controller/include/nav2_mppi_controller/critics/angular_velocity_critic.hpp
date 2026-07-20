// Copyright (c) 2026 Dexory
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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__ANGULAR_VELOCITY_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__ANGULAR_VELOCITY_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::AngularVelocityCritic
 * @brief 角速度软约束（双项，同权重）：
 *        ① 速度项可按阶段选择：
 *           - FINAL_YAW 跟踪按朝向误差算出的期望角速度；
 *           - POSITIONING 将期望角速度固定为 0，禁止最终 yaw 提前抢占位置控制。
 *        跟踪目标 yaw 时：
 *           wz_desired = min(wz_max, sqrt(2*az_max*|θ_err|)) * sign(θ_err)，
 *           θ_err = 归一化(goal_yaw - traj_yaw)（远离目标朝向=wz_max，接近时平滑降到 0，防朝向过冲）；
 *           当 |θ_err| <= stop_yaw_tolerance 时 wz_desired=0，使控制目标与 GoalChecker 的
 *           航向容差和停稳条件一致；
 *           罚 |wz - wz_desired|。
 *        ② 加速度项 罚相邻 |Δwz|（角加速度，平滑）。
 *        wz_max / az_max 从 FollowPath 全局参数读取。
 *        TRACK/ALIGN 权重仍应为 0；APPROACH 内部再按定位/最终转向切换上述目标。
 */
class AngularVelocityCritic : public CriticFunction
{
public:
  void initialize() override;

  void score(CriticData & data) override;

  void applyProfileSettings(const CriticProfileSettings & settings) override;

protected:
  unsigned int power_{0};
  float weight_{0};
  float wz_max_{0.75f};  // 从 FollowPath 全局参数读
  float az_max_{0.7f};   // 从 FollowPath 全局参数读（角减速度）
  float stop_yaw_tolerance_{0.02f};  // 进入最终航向容差后，期望角速度直接降为 0
  bool track_goal_yaw_{true};  // 只有 FINAL_YAW 才允许根据最终目标朝向主动生成 wz
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__ANGULAR_VELOCITY_CRITIC_HPP_
