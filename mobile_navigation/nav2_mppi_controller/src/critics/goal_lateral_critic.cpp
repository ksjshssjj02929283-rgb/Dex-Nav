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
  getParam(last_n_points_, "last_n_points", 8);

  RCLCPP_INFO(
    logger_,
    "GoalLateralCritic instantiated with %d power, %.3f weight, %zu points",
    power_, weight_, last_n_points_);
}

void GoalLateralCritic::score(CriticData & data)
{
  // 三阶段 MPPI 中 cost_weight <= 0 表示该阶段关闭横向误差代价。
  // 开关交给阶段权重（无距离门控）。
  if (!enabled_ || weight_ <= 0.0f || data.path.x.size() < 1) {
    return;
  }

  const auto goal_idx = data.path.x.size() - 1;
  const float goal_x = data.path.x(goal_idx);
  const float goal_y = data.path.y(goal_idx);

  const Eigen::Index time_steps = data.trajectories.x.cols();
  if (time_steps < 1) {
    return;
  }

  const Eigen::Index sample_count = std::max<Eigen::Index>(
    1, std::min<Eigen::Index>(static_cast<Eigen::Index>(last_n_points_), time_steps));
  const Eigen::Index start_idx = time_steps - sample_count;

  models::Vector cost = models::Vector::Zero(data.costs.size());
  for (Eigen::Index p = start_idx; p < time_steps; ++p) {
    // 目标点在每个候选轨迹点(base_link)坐标系下的横向(左向)投影误差
    const auto gx = goal_x - data.trajectories.x.col(p).array();
    const auto gy = goal_y - data.trajectories.y.col(p).array();
    const models::Vector yaw = data.trajectories.yaws.col(p);
    const auto lat_err = (-yaw.sin() * gx + yaw.cos() * gy).abs();
    cost += lat_err;
  }

  cost /= static_cast<float>(sample_count);
  data.costs += (cost * weight_).pow(power_);
}

void GoalLateralCritic::applyProfileSettings(const CriticProfileSettings & settings)
{
  // 三阶段 profile 覆盖横向误差代价参数
  weight_ = settings.goal_lateral.weight;
  last_n_points_ = static_cast<size_t>(std::max(1, settings.goal_lateral.last_n_points));
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::GoalLateralCritic, mppi::critics::CriticFunction)
