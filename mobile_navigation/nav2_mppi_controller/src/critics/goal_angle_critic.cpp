// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include "nav2_mppi_controller/critics/goal_angle_critic.hpp"

#include <algorithm>
#include <cmath>

#include "tf2/utils.h"

namespace mppi::critics
{

void GoalAngleCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 3.0);
  getParam(wrong_way_cost_ratio_, "wrong_way_cost_ratio", 2.0);
  getParam(direction_deadband_, "direction_deadband", 0.05);
  wrong_way_cost_ratio_ = std::max(0.0f, wrong_way_cost_ratio_);
  direction_deadband_ = std::max(0.0f, direction_deadband_);

  RCLCPP_INFO(
    logger_,
    "GoalAngleCritic instantiated with %d power, %.3f weight, %.3f wrong-way ratio and "
    "%.3f rad direction deadband.",
    power_, weight_, wrong_way_cost_ratio_, direction_deadband_);
}

void GoalAngleCritic::score(CriticData & data)
{
  // 三阶段 MPPI 中 cost_weight <= 0 表示该阶段关闭目标朝向代价。
  if (!enabled_ || weight_ <= 0.0f) {
    return;
  }

  const auto goal_idx = data.path.x.size() - 1;
  const float goal_yaw = data.terminal_yaw_override.value_or(data.path.yaws(goal_idx));

  const models::Matrix signed_yaw_errors =
    utils::shortest_angular_distance(data.trajectories.yaws, goal_yaw);
  data.costs += (signed_yaw_errors.abs().rowwise().mean() * weight_).pow(power_);

  // 用本控制周期的真实姿态固定最短旋转方向，避免预测轨迹穿过 ±pi 后把长弧误判为短弧。
  // current_error = goal-current；符号为正时只允许正向转，为负时只允许负向转。
  if (
    wrong_way_cost_ratio_ > 0.0f &&
    data.state.wz.rows() == data.costs.size() && data.state.wz.cols() > 0)
  {
    const float current_yaw = tf2::getYaw(data.state.pose.pose.orientation);
    const float raw_current_error = goal_yaw - current_yaw;
    const float current_error = std::atan2(
      std::sin(raw_current_error), std::cos(raw_current_error));
    if (std::abs(current_error) > direction_deadband_) {
      const float desired_direction = current_error > 0.0f ? 1.0f : -1.0f;
      const models::Matrix wrong_way_rotation =
        (-desired_direction * data.state.wz).max(0.0f);
      data.costs +=
        (wrong_way_rotation.rowwise().mean() * weight_ * wrong_way_cost_ratio_).pow(power_);
    }
  }
}

void GoalAngleCritic::applyGoalAngleCriticSettings(const GoalAngleCriticSettings & settings)
{
  weight_ = settings.weight;
}

void GoalAngleCritic::applyProfileSettings(const CriticProfileSettings & settings)
{
  applyGoalAngleCriticSettings(settings.goal_angle);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::GoalAngleCritic,
  mppi::critics::CriticFunction)
