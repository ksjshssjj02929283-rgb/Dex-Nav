// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
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

#include "nav2_mppi_controller/critics/goal_critic.hpp"

namespace mppi::critics
{

void GoalCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 5.0);

  RCLCPP_INFO(
    logger_, "GoalCritic instantiated with %d power and %f weight.",
    power_, weight_);
}

void GoalCritic::score(CriticData & data)
{
  // 三阶段 MPPI 中 cost_weight <= 0 表示该阶段关闭此目标距离代价。
  if (!enabled_ || weight_ <= 0.0f) {
    return;
  }

  const auto goal_idx = data.path.x.size() - 1;

  const auto goal_x = data.path.x(goal_idx);
  const auto goal_y = data.path.y(goal_idx);

  auto dists = ((data.trajectories.x - goal_x).square() +
    (data.trajectories.y - goal_y).square()).sqrt();

  data.costs += (dists.rowwise().mean() * weight_).pow(power_);
}

void GoalCritic::applyGoalCriticSettings(const GoalCriticSettings & settings)
{
  weight_ = settings.weight;
}

void GoalCritic::applyProfileSettings(const CriticProfileSettings & settings)
{
  applyGoalCriticSettings(settings.goal);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::GoalCritic, mppi::critics::CriticFunction)
