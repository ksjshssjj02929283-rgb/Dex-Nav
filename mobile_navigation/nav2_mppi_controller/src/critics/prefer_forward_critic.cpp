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

#include "nav2_mppi_controller/critics/prefer_forward_critic.hpp"

namespace mppi::critics
{

void PreferForwardCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 5.0);
  getParam(
    threshold_to_consider_,
    "threshold_to_consider", 0.5);

  RCLCPP_INFO(
    logger_, "PreferForwardCritic instantiated with %d power and %f weight.", power_, weight_);
}

void PreferForwardCritic::score(CriticData & data)
{
  // 三阶段 MPPI 中 cost_weight <= 0 表示该阶段关闭前进偏好，允许末段小幅倒退修正。
  // threshold 距离门已删（同 path_align）。
  if (!enabled_ || weight_ <= 0.0f)
  {
    return;
  }

  auto backward_motion = (-data.state.vx).max(0.0f);
  data.costs += ((backward_motion * data.model_dt).rowwise().sum() * weight_).pow(power_);
}

void PreferForwardCritic::applyPreferForwardCriticSettings(
  const PreferForwardCriticSettings & settings)
{
  weight_ = settings.weight;
  threshold_to_consider_ = settings.threshold_to_consider;
}

void PreferForwardCritic::applyProfileSettings(const CriticProfileSettings & settings)
{
  applyPreferForwardCriticSettings(settings.prefer_forward);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PreferForwardCritic,
  mppi::critics::CriticFunction)
