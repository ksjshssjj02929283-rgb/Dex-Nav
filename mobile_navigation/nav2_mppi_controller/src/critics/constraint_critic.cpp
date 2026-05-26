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

#include "nav2_mppi_controller/critics/constraint_critic.hpp"

namespace mppi::critics
{

void ConstraintCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 4.0);
  RCLCPP_INFO(
    logger_, "ConstraintCritic instantiated with %d power and %f weight.",
    power_, weight_);

  float vx_max, vy_max, vx_min;
  getParentParam(vx_max, "vx_max", 0.5);
  getParentParam(vy_max, "vy_max", 0.0);
  getParentParam(vx_min, "vx_min", -0.35);

  const float min_sgn = vx_min > 0.0 ? 1.0 : -1.0;
  max_vel_ = sqrtf(vx_max * vx_max + vy_max * vy_max);
  min_vel_ = min_sgn * sqrtf(vx_min * vx_min + vy_max * vy_max);
}

void ConstraintCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  auto sgn = (data.state.vx > 0.0f).select(
    models::Matrix::Constant(data.state.vx.rows(), data.state.vx.cols(), 1.0f),
    models::Matrix::Constant(data.state.vx.rows(), data.state.vx.cols(), -1.0f));
  auto vel_total = sgn * (data.state.vx.square() + data.state.vy.square()).sqrt();
  auto out_of_max_bounds_motion = (vel_total - max_vel_).max(0.0f);
  auto out_of_min_bounds_motion = (min_vel_ - vel_total).max(0.0f);

  auto acker = dynamic_cast<AckermannMotionModel *>(data.motion_model.get());
  if (acker != nullptr) {
    auto & vx = data.state.vx;
    auto & wz = data.state.wz;
    auto out_of_turning_rad_motion =
      (acker->getMinTurningRadius() - (vx.abs() / wz.abs())).max(0.0f);

    data.costs += (((out_of_max_bounds_motion + out_of_min_bounds_motion +
      out_of_turning_rad_motion) * data.model_dt).rowwise().sum() * weight_).pow(power_);
    return;
  }

  data.costs += (((out_of_max_bounds_motion + out_of_min_bounds_motion) *
    data.model_dt).rowwise().sum() * weight_).pow(power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::ConstraintCritic, mppi::critics::CriticFunction)
