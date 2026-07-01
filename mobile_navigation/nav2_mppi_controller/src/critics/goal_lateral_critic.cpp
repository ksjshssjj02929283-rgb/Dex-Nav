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

#include "nav2_mppi_controller/critics/goal_lateral_critic.hpp"

#include <algorithm>
#include <cmath>

namespace mppi::critics
{

void GoalLateralCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 2);
  getParam(weight_, "cost_weight", 5.0);

  RCLCPP_INFO(
    logger_,
    "GoalLateralCritic instantiated with %d power, %.3f weight",
    power_, weight_);
}

void GoalLateralCritic::score(CriticData & data)
{
  // 三阶段 MPPI 中 cost_weight <= 0 表示该阶段关闭横向误差代价。
  // 开关交给阶段权重（无距离门控）。
  if (
    !enabled_ || weight_ <= 0.0f || data.path.x.size() < 1 ||
    data.path.y.size() != data.path.x.size() || data.path.yaws.size() != data.path.x.size())
  {
    return;
  }

  const auto goal_idx = data.path.x.size() - 1;
  const float goal_x = data.path.x(goal_idx);
  const float goal_y = data.path.y(goal_idx);
  const float goal_yaw = data.path.yaws(goal_idx);
  const float goal_sin = std::sin(goal_yaw);
  const float goal_cos = std::cos(goal_yaw);

  const Eigen::Index time_steps = data.trajectories.x.cols();
  if (time_steps < 1) {
    return;
  }

  models::Vector cost = models::Vector::Zero(data.costs.size());
  for (Eigen::Index p = 0; p < time_steps; ++p) {
    // 将候选轨迹点与目标点的位置差投影到固定的目标位姿坐标系。
    // 横向轴为目标朝向的左侧，不随候选轨迹自身 yaw 变化。
    const auto dx = data.trajectories.x.col(p).array() - goal_x;
    const auto dy = data.trajectories.y.col(p).array() - goal_y;
    const auto lat_err = (-goal_sin * dx + goal_cos * dy).abs();
    cost += lat_err;
  }

  cost /= static_cast<float>(time_steps);
  data.costs += (cost * weight_).pow(power_);
}

void GoalLateralCritic::applyProfileSettings(const CriticProfileSettings & settings)
{
  // 三阶段 profile 覆盖横向误差代价参数
  weight_ = settings.goal_lateral.weight;
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::GoalLateralCritic, mppi::critics::CriticFunction)
