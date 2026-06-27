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
 * @brief 角速度软约束（双项，同权重）：① 角速度项 罚 |wz| 幅值（偏好更小/更稳转动）；
 *        ② 角加速度项 罚 |wz_t - wz_{t-1}|（平滑，正比于角加速度）。
 *        镜像线速度软约束(DesiredVelocityCritic)"跟踪 + 平滑"双项同权重的结构，
 *        使 MPPI 同时偏好"角速度小"且"角速度变化缓"的轨迹。
 *        与 az_max/jz_max 硬约束互补（硬约束设变化率上限，本 critic 软偏好）。
 *        注：含 |wz| 幅值项后会抑制转动，需转向自由的阶段(align/approach)应把该段权重设 0。
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
