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

#include "nav2_mppi_controller/critics/angular_velocity_critic.hpp"

#include <algorithm>
#include <cmath>

namespace mppi::critics
{

void AngularVelocityCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);

  getParam(power_, "cost_power", 2);
  getParam(weight_, "cost_weight", 10.0);
  // 角速度上限与角减速度从 FollowPath 全局参数读取（不在 critic 内自定义）
  getParentParam(wz_max_, "wz_max", 0.75);
  getParentParam(az_max_, "az_max", 0.7);
  getParam(stop_yaw_tolerance_, "stop_yaw_tolerance", 0.02);
  stop_yaw_tolerance_ = std::max(0.0f, stop_yaw_tolerance_);

  RCLCPP_INFO(
    logger_,
    "AngularVelocityCritic instantiated with %d power, %.3f weight, wz_max=%.3f, "
    "az_max=%.3f, stop_yaw_tolerance=%.3f rad",
    power_, weight_, wz_max_, az_max_, stop_yaw_tolerance_);
}

void AngularVelocityCritic::score(CriticData & data)
{
  if (!enabled_ || weight_ <= 0.0f || data.path.yaws.size() < 1) {
    return;
  }

  const auto goal_idx = data.path.yaws.size() - 1;
  const float goal_yaw = data.path.yaws(goal_idx);

  const Eigen::Index time_steps = data.state.wz.cols();
  if (time_steps < 1) {
    return;
  }

  // 速度项：按朝向误差算期望角速度 wz_desired = min(wz_max, sqrt(2*az_max*|θ_err|))*sign(θ_err)
  // θ_err = 归一化(goal_yaw - traj_yaw) 到 [-pi, pi]（atan2(sin,cos)）；罚 |wz - wz_desired|
  models::Vector cost = models::Vector::Zero(data.costs.size());
  for (Eigen::Index p = 0; p < time_steps; ++p) {
    const auto dyaw = goal_yaw - data.trajectories.yaws.col(p).array();
    const models::Vector err = dyaw.sin().binaryExpr(
      dyaw.cos(), [](float s, float c) {return std::atan2(s, c);});
    const models::Vector braking_wz =
      (2.0f * az_max_ * err.abs()).sqrt().min(wz_max_) * err.sign();
    const models::Vector wz_desired =
      (err.abs() > stop_yaw_tolerance_).select(braking_wz, 0.0f);
    cost += (data.state.wz.col(p).array() - wz_desired).abs();
  }
  cost /= static_cast<float>(time_steps);
  data.costs += (cost * weight_).pow(power_);

  // 加速度项：罚相邻 |Δwz|（角加速度，平滑），与速度项同权重(weight_)
  if (time_steps >= 2) {
    const auto dwz =
      (data.state.wz.rightCols(time_steps - 1) - data.state.wz.leftCols(time_steps - 1)).abs();
    data.costs += (dwz.rowwise().mean() * weight_).pow(power_);
  }
}

void AngularVelocityCritic::applyProfileSettings(const CriticProfileSettings & settings)
{
  weight_ = settings.angular_velocity.weight;
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::AngularVelocityCritic, mppi::critics::CriticFunction)
