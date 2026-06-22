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
  getParam(threshold_to_consider_, "threshold_to_consider", 1.5);
  getParam(last_n_points_, "last_n_points", 8);

  RCLCPP_INFO(
    logger_,
    "GoalLateralCritic instantiated with %d power, %.3f weight, %.3f threshold, %zu points",
    power_, weight_, threshold_to_consider_, last_n_points_);
}

void GoalLateralCritic::score(CriticData & data)
{
  // 三阶段 MPPI 中 cost_weight <= 0 表示该阶段关闭横向误差代价。
  if (!enabled_ || weight_ <= 0.0f || data.path.x.size() < 1 ||
    !utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path))
  {
    return;
  }

  const auto goal_idx = data.path.x.size() - 1;
  const float goal_x = data.path.x(goal_idx);
  const float goal_y = data.path.y(goal_idx);
  const float goal_yaw = data.path.yaws(goal_idx);
  const float cos_yaw = std::cos(goal_yaw);
  const float sin_yaw = std::sin(goal_yaw);

  const Eigen::Index time_steps = data.trajectories.x.cols();
  if (time_steps < 1) {
    return;
  }

  const Eigen::Index sample_count = std::max<Eigen::Index>(
    1, std::min<Eigen::Index>(static_cast<Eigen::Index>(last_n_points_), time_steps));
  const Eigen::Index start_idx = time_steps - sample_count;

  models::Vector cost = models::Vector::Zero(data.costs.size());
  for (Eigen::Index p = start_idx; p < time_steps; ++p) {
    const auto dx = data.trajectories.x.col(p).array() - goal_x;
    const auto dy = data.trajectories.y.col(p).array() - goal_y;
    // 在 MPPI 公共 frame 内，按目标朝向投影出横向误差
    const auto lat_err = (-sin_yaw * dx + cos_yaw * dy).abs();
    cost += lat_err;
  }

  cost /= static_cast<float>(sample_count);
  data.costs += (cost * weight_).pow(power_);
}

void GoalLateralCritic::applyProfileSettings(const CriticProfileSettings & settings)
{
  // 三阶段 profile 覆盖横向误差代价参数
  weight_ = settings.goal_lateral.weight;
  threshold_to_consider_ = settings.goal_lateral.threshold_to_consider;
  last_n_points_ = static_cast<size_t>(std::max(1, settings.goal_lateral.last_n_points));
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::GoalLateralCritic, mppi::critics::CriticFunction)
