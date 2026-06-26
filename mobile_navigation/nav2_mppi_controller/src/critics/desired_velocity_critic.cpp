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

#include "nav2_mppi_controller/critics/desired_velocity_critic.hpp"

#include <algorithm>
#include <cmath>

namespace mppi::critics
{

void DesiredVelocityCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);

  getParam(power_, "cost_power", 2);
  getParam(weight_, "cost_weight", 5.0);
  getParam(a_decel_, "a_decel", 0.3);
  // 期望速度上限取 FollowPath 全局 vx_max；远离目标时 v_desired 即为该上限（不施加额外约束）
  getParentParam(vx_max_, "vx_max", 0.7);

  RCLCPP_INFO(
    logger_,
    "DesiredVelocityCritic instantiated with %d power, %.3f weight, "
    "a_decel=%.3f, vx_max=%.3f",
    power_, weight_, a_decel_, vx_max_);
}

void DesiredVelocityCritic::score(CriticData & data)
{
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

  models::Vector cost = models::Vector::Zero(data.costs.size());
  for (Eigen::Index p = 0; p < time_steps; ++p) {
    const auto dx = data.trajectories.x.col(p).array() - goal_x;
    const auto dy = data.trajectories.y.col(p).array() - goal_y;
    const auto dist = (dx * dx + dy * dy).sqrt();
    // 按剩余距离算期望速度：v_desired = min(vx_max, sqrt(2 * a_decel * dist))
    // dist 大时 = vx_max（不降速），接近目标时平滑降到 0
    const auto v_desired = (2.0f * a_decel_ * dist).sqrt().min(vx_max_);
    // 单边惩罚：只在“速度超过期望速度”时计代价，等效为平滑下降的软速度上限
    // （远离目标 v_desired=vx_max，速度受硬约束 ≤ vx_max → 代价为 0，不干扰起步/中间段）
    const auto over = (data.state.vx.col(p).array() - v_desired).max(0.0f);
    cost += over;
  }

  cost /= static_cast<float>(time_steps);
  data.costs += (cost * weight_).pow(power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::DesiredVelocityCritic, mppi::critics::CriticFunction)
