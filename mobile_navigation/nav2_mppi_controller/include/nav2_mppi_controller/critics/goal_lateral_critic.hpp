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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__GOAL_LATERAL_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__GOAL_LATERAL_CRITIC_HPP_

#include <cstddef>

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::GoalLateralCritic
 * @brief Penalize lateral error in the goal pose frame near the goal.
 */
class GoalLateralCritic : public CriticFunction
{
public:
  void initialize() override;

  void score(CriticData & data) override;

  void applyProfileSettings(const CriticProfileSettings & settings) override;

protected:
  unsigned int power_{0};
  float weight_{0};
  float threshold_to_consider_{0};
  size_t last_n_points_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__GOAL_LATERAL_CRITIC_HPP_
