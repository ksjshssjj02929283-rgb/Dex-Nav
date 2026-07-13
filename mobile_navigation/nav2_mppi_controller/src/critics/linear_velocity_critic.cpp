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

#include "nav2_mppi_controller/critics/linear_velocity_critic.hpp"

#include <algorithm>
#include <cmath>

namespace mppi::critics
{

void LinearVelocityCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);

  getParam(power_, "cost_power", 2);
  getParam(weight_, "cost_weight", 5.0);
  // 初始速度上限从 FollowPath 读取；运行后由三阶段 profile 持续覆盖。
  getParentParam(speed_max_, "vx_max", 0.7);
  getParentParam(ax_max_, "ax_max", 0.5);

  RCLCPP_INFO(
    logger_,
    "LinearVelocityCritic instantiated with %d power, %.3f weight, speed_max=%.3f, "
    "ax_max=%.3f",
    power_, weight_, speed_max_, ax_max_);
}

void LinearVelocityCritic::score(CriticData & data)
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

  // 只约束速度大小，不在这里选择前进/倒车方向；方向交给路径、目标和前进偏好 critic。
  models::Vector cost = models::Vector::Zero(data.costs.size());
  for (Eigen::Index p = 0; p < time_steps; ++p) {
    const auto dx = data.trajectories.x.col(p).array() - goal_x;
    const auto dy = data.trajectories.y.col(p).array() - goal_y;
    const auto dist = (dx * dx + dy * dy).sqrt();
    const auto speed_desired = (2.0f * ax_max_ * dist).sqrt().min(speed_max_);
    cost += (data.state.vx.col(p).array().abs() - speed_desired).abs();
  }
  cost /= static_cast<float>(time_steps);
  data.costs += (cost * weight_).pow(power_);

  // 加速度项：罚相邻 |Δvx|（线加速度，平滑），与速度项同权重(weight_)
  if (time_steps >= 2) {
    const auto dvx =
      (data.state.vx.rightCols(time_steps - 1) - data.state.vx.leftCols(time_steps - 1)).abs();
    data.costs += (dvx.rowwise().mean() * weight_).pow(power_);
  }
}

void LinearVelocityCritic::applyProfileSettings(const CriticProfileSettings & settings)
{
  weight_ = settings.linear_velocity.weight;
  if (settings.linear_velocity.speed_max > 0.0f) {
    speed_max_ = settings.linear_velocity.speed_max;
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::LinearVelocityCritic, mppi::critics::CriticFunction)
