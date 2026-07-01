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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__LINEAR_VELOCITY_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__LINEAR_VELOCITY_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::LinearVelocityCritic
 * @brief 线速度软约束（双项，同权重）：
 *        ① 速度项 跟踪按剩余距离算的期望速度 v_desired = min(vx_max, sqrt(2*ax_max*d))
 *           （远处=vx_max 不降速，接近目标平滑降到 0）；罚 |vx - v_desired|。
 *        ② 加速度项 罚相邻 |Δvx|（线加速度，平滑）。
 *        vx_max / ax_max 从 FollowPath 全局参数读取（critic 内不自定义速度/加速度参数）。
 */
class LinearVelocityCritic : public CriticFunction
{
public:
  void initialize() override;

  void score(CriticData & data) override;

  void applyProfileSettings(const CriticProfileSettings & settings) override;

protected:
  unsigned int power_{0};
  float weight_{0};
  float vx_max_{0.7f};   // 从 FollowPath 全局参数读
  float ax_max_{0.5f};   // 从 FollowPath 全局参数读（作减速度）
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__LINEAR_VELOCITY_CRITIC_HPP_
