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

#include "nav2_mppi_controller/critics/path_follow_critic.hpp"

#include <algorithm>

namespace mppi::critics
{

void PathFollowCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(offset_from_furthest_, "offset_from_furthest", 6);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 5.0);
}

void PathFollowCritic::score(CriticData & data)
{
  // 三阶段 MPPI 中 cost_weight <= 0 表示该阶段关闭路径推进代价。
  // threshold 距离门已删（同 path_align）。
  if (!enabled_ || weight_ <= 0.0f || data.path.x.size() < 2)
  {
    return;
  }

  utils::setPathFurthestPointIfNotSet(data);
  utils::setPathCostsIfNotSet(data, costmap_ros_);
  const size_t path_size = data.path.x.size() - 1;

  auto offseted_idx = std::min(
    *data.furthest_reached_path_point + offset_from_furthest_, path_size);

  // Drive to the first valid path point, in case of dynamic obstacles on path
  // we want to drive past it, not through it
  bool valid = false;
  while (!valid && offseted_idx < path_size - 1) {
    valid = (*data.path_pts_valid)[offseted_idx];
    if (!valid) {
      offseted_idx++;
    }
  }

  const auto path_x = data.path.x(offseted_idx);
  const auto path_y = data.path.y(offseted_idx);

  const auto last_x = data.trajectories.x.col(data.trajectories.x.cols() - 1);
  const auto last_y = data.trajectories.y.col(data.trajectories.y.cols() - 1);

  auto dists = ((last_x - path_x).square() + (last_y - path_y).square()).sqrt();

  data.costs += (weight_ * dists).pow(power_);
}

void PathFollowCritic::applyPathFollowCriticSettings(const PathFollowCriticSettings & settings)
{
  weight_ = settings.weight;
  offset_from_furthest_ = static_cast<size_t>(std::max(0, settings.offset_from_furthest));
}

void PathFollowCritic::applyProfileSettings(const CriticProfileSettings & settings)
{
  applyPathFollowCriticSettings(settings.path_follow);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathFollowCritic,
  mppi::critics::CriticFunction)
