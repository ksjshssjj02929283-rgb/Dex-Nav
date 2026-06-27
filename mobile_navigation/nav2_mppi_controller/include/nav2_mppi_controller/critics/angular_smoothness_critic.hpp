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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__ANGULAR_SMOOTHNESS_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__ANGULAR_SMOOTHNESS_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::AngularSmoothnessCritic
 * @brief 角速度平滑软约束：惩罚候选轨迹中相邻时刻角速度的变化 |wz_t - wz_{t-1}|
 *        （正比于角加速度），使 MPPI 偏好角速度变化平缓的轨迹 → 旋转更平顺。
 *        与 az_max/jz_max 硬约束（限变化率上限）互补：硬约束设上限，本 critic 软偏好平滑。
 *        只罚"变化"不罚"幅值"，故不阻止必要的快速转向，只让转向过程顺滑。
 */
class AngularSmoothnessCritic : public CriticFunction
{
public:
  void initialize() override;

  void score(CriticData & data) override;

  void applyProfileSettings(const CriticProfileSettings & settings) override;

protected:
  unsigned int power_{0};
  float weight_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__ANGULAR_SMOOTHNESS_CRITIC_HPP_
