// Copyright (c) 2023 Open Navigation LLC
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

#include "nav2_mppi_controller/critics/path_align_critic.hpp"

#include <algorithm>

namespace mppi::critics
{

void PathAlignCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 10.0);

  getParam(max_path_occupancy_ratio_, "max_path_occupancy_ratio", 0.07);
  getParam(offset_from_furthest_, "offset_from_furthest", 20);
  getParam(trajectory_point_step_, "trajectory_point_step", 4);
  getParam(use_path_orientations_, "use_path_orientations", false);
  getParam(ignore_goal_orientation_, "ignore_goal_orientation", true);

  RCLCPP_INFO(
    logger_,
    "ReferenceTrajectoryCritic instantiated with %d power and %f weight",
    power_, weight_);
}

void PathAlignCritic::score(CriticData & data)
{
  // Don't apply close to goal, let the goal critics take over
  // 三阶段 MPPI 中 cost_weight <= 0 表示该阶段关闭路径贴合代价，直接跳过较重的路径匹配循环。
  // threshold 距离门已删（阶段系统管"近目标让位"；避免 approach_distance < threshold 时死区）。
  if (!enabled_ || weight_ <= 0.0f)
  {
    return;
  }

  // Don't apply when first getting bearing w.r.t. the path
  utils::setPathFurthestPointIfNotSet(data);
  const size_t path_segments_count = *data.furthest_reached_path_point;  // up to furthest only
  if (path_segments_count < offset_from_furthest_) {
    return;
  }

  // Don't apply when dynamic obstacles are blocking significant proportions of the local path
  utils::setPathCostsIfNotSet(data, costmap_ros_);
  const size_t closest_initial_path_point = utils::findPathTrajectoryInitialPoint(data);
  unsigned int invalid_ctr = 0;
  const float range = *data.furthest_reached_path_point - closest_initial_path_point;
  for (size_t i = closest_initial_path_point; i < *data.furthest_reached_path_point; i++) {
    if (!(*data.path_pts_valid)[i]) {invalid_ctr++;}
    if (static_cast<float>(invalid_ctr) / range > max_path_occupancy_ratio_ && invalid_ctr > 2) {
      return;
    }
  }

  const auto & P_x = data.path.x;  // path points
  const auto & P_y = data.path.y;  // path points
  const auto & P_yaw = data.path.yaws;  // path points

  const size_t batch_size = data.trajectories.x.rows();
  const size_t time_steps = data.trajectories.x.cols();
  models::Vector cost = models::Vector::Zero(data.costs.size());

  // Find integrated distance in the path
  std::vector<float> path_integrated_distances(path_segments_count, 0.0f);
  float dx = 0.0f, dy = 0.0f;
  for (unsigned int i = 1; i != path_segments_count; i++) {
    dx = P_x(i) - P_x(i - 1);
    dy = P_y(i) - P_y(i - 1);
    float curr_dist = sqrtf(dx * dx + dy * dy);
    path_integrated_distances[i] = path_integrated_distances[i - 1] + curr_dist;
  }

  float traj_integrated_distance = 0.0f;
  float summed_path_dist = 0.0f, dyaw = 0.0f;
  float num_samples = 0.0f;
  float Tx = 0.0f, Ty = 0.0f;
  for (size_t t = 0; t < batch_size; ++t) {
    traj_integrated_distance = 0.0f;
    summed_path_dist = 0.0f;
    num_samples = 0.0f;
    size_t path_pt = 0;
    for (size_t p = trajectory_point_step_; p < time_steps; p += trajectory_point_step_) {
      Tx = data.trajectories.x(t, p);
      Ty = data.trajectories.y(t, p);
      dx = Tx - data.trajectories.x(t, p - trajectory_point_step_);
      dy = Ty - data.trajectories.y(t, p - trajectory_point_step_);
      traj_integrated_distance += sqrtf(dx * dx + dy * dy);
      path_pt = utils::findClosestPathPt(
        path_integrated_distances, traj_integrated_distance, path_pt);

      // The nearest path point to align to needs to be not in collision, else
      // let the obstacle critic take over in this region due to dynamic obstacles
      if ((*data.path_pts_valid)[path_pt]) {
        dx = P_x(path_pt) - Tx;
        dy = P_y(path_pt) - Ty;
        num_samples += 1.0f;
        if (use_path_orientations_) {
          // path.back() 的 yaw 是用户最终朝向，只能在 FINAL_YAW 使用。START 模式
          // 若在 TRACK 预测窗口内读到它，就会在到点前边走边转。跟踪最后位置时
          // 继续使用倒数第二个真实路径点的 yaw，位置代价仍然指向终点坐标。
          const size_t yaw_path_pt =
            ignore_goal_orientation_ && path_pt + 1 == static_cast<size_t>(P_yaw.size()) &&
            path_pt > 0 ? path_pt - 1 : path_pt;
          dyaw = angles::shortest_angular_distance(
            P_yaw(yaw_path_pt), data.trajectories.yaws(t, p));
          summed_path_dist += sqrtf(dx * dx + dy * dy + dyaw * dyaw);
        } else {
          summed_path_dist += sqrtf(dx * dx + dy * dy);
        }
      }
    }
    if (num_samples > 0) {
      cost(t) = summed_path_dist / num_samples;
    } else {
      cost(t) = 0.0f;
    }
  }

  data.costs += (cost * weight_).pow(power_);
}

void PathAlignCritic::applyPathAlignCriticSettings(const PathAlignCriticSettings & settings)
{
  weight_ = settings.weight;
  offset_from_furthest_ = static_cast<size_t>(std::max(0, settings.offset_from_furthest));
  trajectory_point_step_ = static_cast<size_t>(std::max(1, settings.trajectory_point_step));
  max_path_occupancy_ratio_ = settings.max_path_occupancy_ratio;
  // 允许控制器按阶段关闭路径 yaw：趋近定位只贴合路径位置，最终 yaw 交给 GoalAngle。
  use_path_orientations_ = settings.use_path_orientations;
  ignore_goal_orientation_ = settings.ignore_goal_orientation;
}

void PathAlignCritic::applyProfileSettings(const CriticProfileSettings & settings)
{
  applyPathAlignCriticSettings(settings.path_align);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathAlignCritic,
  mppi::critics::CriticFunction)
