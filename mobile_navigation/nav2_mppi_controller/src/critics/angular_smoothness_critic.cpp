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

#include "nav2_mppi_controller/critics/angular_smoothness_critic.hpp"

namespace mppi::critics
{

void AngularSmoothnessCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 2);
  getParam(weight_, "cost_weight", 10.0);

  RCLCPP_INFO(
    logger_,
    "AngularSmoothnessCritic instantiated with %d power, %.3f weight",
    power_, weight_);
}

void AngularSmoothnessCritic::score(CriticData & data)
{
  if (!enabled_ || weight_ <= 0.0f) {
    return;
  }

  const Eigen::Index time_steps = data.state.wz.cols();
  if (time_steps < 2) {
    return;
  }

  // 角加速度软成本：相邻时刻角速度差 |wz_t - wz_{t-1}| 的均值。
  // 越平滑该值越小；power=2 放大不平滑的轨迹的代价。
  const auto dwz =
    (data.state.wz.rightCols(time_steps - 1) - data.state.wz.leftCols(time_steps - 1)).abs();
  data.costs += (dwz.rowwise().mean() * weight_).pow(power_);
}

void AngularSmoothnessCritic::applyProfileSettings(const CriticProfileSettings & settings)
{
  // 三阶段 profile 覆盖角速度平滑权重（起步段可低、停靠段可高）
  weight_ = settings.angular_smoothness.weight;
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::AngularSmoothnessCritic, mppi::critics::CriticFunction)
