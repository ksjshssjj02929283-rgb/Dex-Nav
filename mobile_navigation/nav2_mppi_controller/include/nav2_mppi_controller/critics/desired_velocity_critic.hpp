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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__DESIRED_VELOCITY_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__DESIRED_VELOCITY_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::DesiredVelocityCritic
 * @brief 速度软约束：按每个预测点到目标的剩余距离算期望速度
 *        v_desired = min(vx_max, sqrt(2 * a_decel * dist))，
 *        双边惩罚“当前(候选)速度与期望速度的差值”|vx - v_desired|（速度跟踪），
 *        让实际速度趋近期望速度，在时域内平滑匀减速、消除阶段边界硬降速。
 *        并附加线加速度平滑项(罚 |Δvx|，同 weight)使前进速度变化平缓。
 */
class DesiredVelocityCritic : public CriticFunction
{
public:
  void initialize() override;

  void score(CriticData & data) override;

  void applyProfileSettings(const CriticProfileSettings & settings) override;

protected:
  unsigned int power_{0};
  float weight_{0};
  float a_decel_{0};   // 期望减速度，决定减速曲线陡缓
  float vx_max_{0};    // 期望速度上限（取自 FollowPath 全局 vx_max）
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__DESIRED_VELOCITY_CRITIC_HPP_
