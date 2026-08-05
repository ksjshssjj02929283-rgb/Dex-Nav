// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include "nav2_mppi_controller/critics/path_angle_critic.hpp"

#include <algorithm>
#include <math.h>
#include <vector>

#include "tf2/utils.h"

namespace mppi::critics
{

void PathAngleCritic::initialize()
{
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);
  float vx_min;
  getParentParam(vx_min, "vx_min", -0.35);
  if (fabs(vx_min) < 1e-6) {  // zero
    reversing_allowed_ = false;
  } else if (vx_min < 0.0) {   // reversing possible
    reversing_allowed_ = true;
  }

  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(offset_from_furthest_, "offset_from_furthest", 4);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 2.0);
  getParam(
    max_angle_to_furthest_,
    "max_angle_to_furthest", 1.2);
  getParam(
    forward_preference_,
    "forward_preference", true);
  getParam(wrong_way_cost_ratio_, "wrong_way_cost_ratio", 2.0);
  getParam(direction_deadband_, "direction_deadband", 0.05);
  wrong_way_cost_ratio_ = std::max(0.0f, wrong_way_cost_ratio_);
  direction_deadband_ = std::max(0.0f, direction_deadband_);

  if (!reversing_allowed_) {
    forward_preference_ = true;
  }

  RCLCPP_INFO(
    logger_,
    "PathAngleCritic instantiated with %d power, %.3f weight, %.3f wrong-way ratio and "
    "%.3f rad direction deadband. Reversing %s",
    power_, weight_, wrong_way_cost_ratio_, direction_deadband_,
    reversing_allowed_ ? "allowed." : "not allowed.");
}

void PathAngleCritic::score(CriticData & data)
{
  // 三阶段 MPPI 中 cost_weight <= 0 表示该阶段关闭路径朝向代价。
  if (!enabled_ || weight_ <= 0.0f) {
    return;
  }

  // threshold 距离门已删（阶段系统管"近目标让位"，避免死区）。

  utils::setPathFurthestPointIfNotSet(data);

  auto offseted_idx = std::min(
    *data.furthest_reached_path_point + offset_from_furthest_,
    static_cast<size_t>(data.path.x.size() - 1));

  const float goal_x = data.path.x(offseted_idx);
  const float goal_y = data.path.y(offseted_idx);

  if (utils::posePointAngle(
      data.state.pose.pose, goal_x, goal_y, forward_preference_) < max_angle_to_furthest_)
  {
    return;
  }

  if (data.use_cuda && data.cuda_optimizer != nullptr &&
    data.cuda_optimizer->isCudaAvailable())
  {
    std::vector<float> gpu_costs;
    const auto & current_pose = data.state.pose.pose;
    const bool success = data.cuda_optimizer->evaluatePathAngleCosts(
      goal_x, goal_y, reversing_allowed_, forward_preference_, direction_deadband_,
      wrong_way_cost_ratio_, current_pose.position.x, current_pose.position.y,
      tf2::getYaw(current_pose.orientation), data.state.speed.angular.z, weight_, power_,
      gpu_costs);
    if (success && gpu_costs.size() == static_cast<size_t>(data.costs.size())) {
      data.costs += Eigen::Map<const Eigen::ArrayXf>(gpu_costs.data(), gpu_costs.size());
      return;
    }
    RCLCPP_WARN(logger_, "CUDA path-angle critic failed; using CPU scoring for this iteration");
  }

  models::Matrix yaws_between_points =
    (goal_y - data.trajectories.y).binaryExpr(
    goal_x - data.trajectories.x,
    [](float y, float x) {return std::atan2(y, x);});

  models::Matrix target_yaws = yaws_between_points;
  if (reversing_allowed_ && !forward_preference_) {
    const models::Matrix forward_yaw_errors =
      utils::shortest_angular_distance(data.trajectories.yaws, yaws_between_points).abs();
    target_yaws =
      (forward_yaw_errors < M_PI_2).select(
      yaws_between_points,
      utils::normalize_angles(yaws_between_points + static_cast<float>(M_PI)));
  }

  const models::Matrix signed_yaw_errors =
    utils::shortest_angular_distance(data.trajectories.yaws, target_yaws);
  data.costs += (signed_yaw_errors.abs().rowwise().mean() * weight_).pow(power_);

  // 用本控制周期的真实姿态固定最短旋转方向，避免预测轨迹跨过 ±pi 后方向符号翻转，
  // 导致候选轨迹沿 270° 长弧反向绕转仍然得不到稳定惩罚。
  // 只惩罚错误旋转，不惩罚 wz=0，因此仍允许机器人先停车或后退腾出空间。
  if (wrong_way_cost_ratio_ > 0.0f) {
    const auto & current_pose = data.state.pose.pose;
    const float current_yaw = tf2::getYaw(current_pose.orientation);
    const float current_target_yaw = std::atan2(
      goal_y - current_pose.position.y,
      goal_x - current_pose.position.x);
    float current_effective_target_yaw = current_target_yaw;
    if (reversing_allowed_ && !forward_preference_) {
      const float forward_error = std::abs(
        std::atan2(
          std::sin(current_target_yaw - current_yaw),
          std::cos(current_target_yaw - current_yaw)));
      const float reverse_target_yaw = current_target_yaw + static_cast<float>(M_PI);
      const float reverse_error = std::abs(
        std::atan2(
          std::sin(reverse_target_yaw - current_yaw),
          std::cos(reverse_target_yaw - current_yaw)));
      if (reverse_error < forward_error) {
        current_effective_target_yaw = reverse_target_yaw;
      }
    }

    const float current_error = std::atan2(
      std::sin(current_effective_target_yaw - current_yaw),
      std::cos(current_effective_target_yaw - current_yaw));
    if (std::abs(current_error) <= direction_deadband_) {
      return;
    }
    const float desired_direction = current_error > 0.0f ? 1.0f : -1.0f;
    models::Matrix wrong_way_rotation =
      (-desired_direction * data.state.wz).max(0.0f);
    data.costs +=
      (wrong_way_rotation.rowwise().mean() * weight_ * wrong_way_cost_ratio_).pow(power_);
  }
}

void PathAngleCritic::applyPathAngleCriticSettings(const PathAngleCriticSettings & settings)
{
  weight_ = settings.weight;
  offset_from_furthest_ = static_cast<size_t>(std::max(0, settings.offset_from_furthest));
  max_angle_to_furthest_ = settings.max_angle_to_furthest;
  forward_preference_ = reversing_allowed_ ? settings.forward_preference : true;
}

void PathAngleCritic::applyProfileSettings(const CriticProfileSettings & settings)
{
  applyPathAngleCriticSettings(settings.path_angle);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathAngleCritic,
  mppi::critics::CriticFunction)
