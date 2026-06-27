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

#include <cmath>
#include "nav2_mppi_controller/critics/cost_critic.hpp"

namespace mppi::critics
{

void CostCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(consider_footprint_, "consider_footprint", false);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 3.81);
  getParam(critical_cost_, "critical_cost", 300.0);
  getParam(collision_cost_, "collision_cost", 1000000.0);
  getParam(near_goal_distance_, "near_goal_distance", 0.5);
  getParam(inflation_scale_factor_, "cost_scaling_factor", 10.0);
  getParam(inflation_layer_name_, "inflation_layer_name", std::string(""));

  // 这两个参数属于 MPPI Controller 本体，而不是单个 critic。
  // 目的：只在接近终点时，让 MPPI 内部按更小的等效膨胀半径评分，
  // 不修改真实 local costmap，避免影响其它避障、恢复或外部订阅模块。
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);
  getParentParam(
    near_goal_cost_relaxation_distance_,
    "near_goal_cost_relaxation_distance", near_goal_distance_, ParameterType::Static);
  getParentParam(
    near_goal_effective_inflation_radius_,
    "near_goal_effective_inflation_radius", 0.0, ParameterType::Static);

  // Normalized by cost value to put in same regime as other weights
  weight_ /= 254.0f;

  collision_checker_.setCostmap(costmap_);
  possibly_inscribed_cost_ = findCircumscribedCost(costmap_ros_);

  if (possibly_inscribed_cost_ < 1.0f) {
    RCLCPP_ERROR(
      logger_,
      "Inflation layer either not found or inflation is not set sufficiently for "
      "optimized non-circular collision checking capabilities. It is HIGHLY recommended to set"
      " the inflation radius to be at MINIMUM half of the robot's largest cross-section. See "
      "github.com/ros-planning/navigation2/tree/main/nav2_smac_planner#potential-fields"
      " for full instructions. This will substantially impact run-time performance.");
  }

  RCLCPP_INFO(
    logger_,
    "InflationCostCritic instantiated with %d power and %f / %f weights. "
    "Critic will collision check based on %s cost. "
    "近目标 MPPI 软膨胀过滤: 生效距离=%.3fm, 等效膨胀半径=%.3fm.",
    power_, critical_cost_, weight_, consider_footprint_ ?
    "footprint" : "circular",
    near_goal_cost_relaxation_distance_, near_goal_effective_inflation_radius_);

  const float inscribed_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  if (near_goal_effective_inflation_radius_ > 0.0f &&
    near_goal_effective_inflation_radius_ <= inscribed_radius)
  {
    RCLCPP_INFO(
      logger_,
      "CostCritic 近目标过滤模式: 等效膨胀半径 %.3fm <= 机器人内切半径 %.3fm，"
      "近目标时会忽略非 lethal 的膨胀 cost，仅保留 lethal/unknown。",
      near_goal_effective_inflation_radius_, inscribed_radius);
  }
}

float CostCritic::findCircumscribedCost(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  bool inflation_layer_found = false;
  double result = -1.0;
  const double circum_radius = costmap->getLayeredCostmap()->getCircumscribedRadius();
  if (static_cast<float>(circum_radius) == circumscribed_radius_) {
    // early return if footprint size is unchanged
    return circumscribed_cost_;
  }

  // check if the costmap has an inflation layer
  for (auto layer = costmap->getLayeredCostmap()->getPlugins()->begin();
    layer != costmap->getLayeredCostmap()->getPlugins()->end();
    ++layer)
  {
    auto inflation_layer = std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(*layer);
    if (!inflation_layer ||
      (!inflation_layer_name_.empty() &&
      inflation_layer->getName() != inflation_layer_name_))
    {
      continue;
    }

    inflation_layer_found = true;
    const double resolution = costmap->getCostmap()->getResolution();
    result = inflation_layer->computeCost(circum_radius / resolution);
  }

  if (!inflation_layer_found) {
    RCLCPP_WARN(
      logger_,
      "No inflation layer found in costmap configuration. "
      "If this is an SE2-collision checking plugin, it cannot use costmap potential "
      "field to speed up collision checking by only checking the full footprint "
      "when robot is within possibly-inscribed radius of an obstacle. This may "
      "significantly slow down planning times and not avoid anything but absolute collisions!");
  }

  circumscribed_radius_ = static_cast<float>(circum_radius);
  circumscribed_cost_ = static_cast<float>(result);

  return circumscribed_cost_;
}

float CostCritic::distanceToObstacle(float cost) const
{
  if (inflation_scale_factor_ <= 0.0f || cost <= 0.0f) {
    return 0.0f;
  }

  const float min_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  return (inflation_scale_factor_ * min_radius - log(cost) + log(253.0f)) /
         inflation_scale_factor_ - min_radius;
}

float CostCritic::relaxedCostAtPose(
  float cost, bool near_goal_relaxation, size_t & filtered_soft_cost_count) const
{
  using namespace nav2_costmap_2d; // NOLINT

  if (!near_goal_relaxation ||
    near_goal_effective_inflation_radius_ <= 0.0f ||
    cost < 1.0f)
  {
    return cost;
  }

  const auto cost_value = static_cast<unsigned char>(cost);
  if (cost_value == LETHAL_OBSTACLE || cost_value == NO_INFORMATION) {
    return cost;
  }

  // 关键点：这里必须在 footprint 碰撞检查之前过滤。
  // 否则 0.2~0.4m 的软膨胀 cost 仍会触发 footprintCostAtPose()，
  // 行为就不等同于把 local costmap 的 inflation_radius 缩小到 0.2m。
  if (cost_value == INSCRIBED_INFLATED_OBSTACLE) {
    ++filtered_soft_cost_count;
    return FREE_SPACE;
  }

  const float inscribed_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  if (near_goal_effective_inflation_radius_ <= inscribed_radius) {
    ++filtered_soft_cost_count;
    return FREE_SPACE;
  }

  const float dist_to_obj = distanceToObstacle(cost);
  if (dist_to_obj >= near_goal_effective_inflation_radius_) {
    ++filtered_soft_cost_count;
    return FREE_SPACE;
  }

  return cost;
}

void CostCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  if (consider_footprint_) {
    // footprint may have changed since initialization if user has dynamic footprints
    possibly_inscribed_cost_ = findCircumscribedCost(costmap_ros_);
  }

  const bool near_goal = utils::withinPositionGoalTolerance(
    near_goal_distance_, data.state.pose.pose, data.path);
  const bool near_goal_relaxation = utils::withinPositionGoalTolerance(
    near_goal_cost_relaxation_distance_, data.state.pose.pose, data.path) &&
    near_goal_effective_inflation_radius_ > 0.0f;

  models::Vector repulsive_cost = models::Vector::Zero(data.costs.size());
  size_t filtered_soft_cost_count = 0;

  const size_t traj_len = data.trajectories.x.cols();
  bool all_trajectories_collide = true;
  for (Eigen::Index i = 0; i < data.trajectories.x.rows(); ++i) {
    bool trajectory_collide = false;
    const auto & traj = data.trajectories;
    float pose_cost;

    for (size_t j = 0; j < traj_len; j++) {
      // The costAtPose doesn't use orientation
      // The footprintCostAtPose will always return "INSCRIBED" if footprint is over it
      // So the center point has more information than the footprint
      pose_cost = costAtPose(traj.x(i, j), traj.y(i, j));
      if (pose_cost < 1.0f) {continue;}  // In free space

      pose_cost = relaxedCostAtPose(pose_cost, near_goal_relaxation, filtered_soft_cost_count);
      if (pose_cost < 1.0f) {continue;}

      if (inCollision(pose_cost, traj.x(i, j), traj.y(i, j), traj.yaws(i, j))) {
        trajectory_collide = true;
        break;
      }

      // Let near-collision trajectory points be punished severely
      // Note that we collision check based on the footprint actual,
      // but score based on the center-point cost regardless
      using namespace nav2_costmap_2d; // NOLINT
      if (pose_cost >= INSCRIBED_INFLATED_OBSTACLE) {
        repulsive_cost(i) += critical_cost_;
      } else if (!near_goal) {  // Generally prefer trajectories further from obstacles
        repulsive_cost(i) += pose_cost;
      }
    }

    if (!trajectory_collide) {
      all_trajectories_collide = false;
    } else {
      repulsive_cost(i) = collision_cost_;
    }
  }

  data.costs += (weight_ * repulsive_cost / traj_len).pow(power_);
  data.fail_flag = all_trajectories_collide;

  auto node = parent_.lock();
  if (near_goal_relaxation && node && !near_goal_relaxation_logged_) {
    RCLCPP_INFO(
      logger_,
      "CostCritic 进入近目标软膨胀过滤区: 距离阈值 %.3fm, 等效膨胀半径 %.3fm, "
      "本轮已忽略 %zu 个非 lethal 膨胀采样点。",
      near_goal_cost_relaxation_distance_, near_goal_effective_inflation_radius_,
      filtered_soft_cost_count);
    near_goal_relaxation_logged_ = true;
  } else if (!near_goal_relaxation) {
    near_goal_relaxation_logged_ = false;
  }

  if (near_goal_relaxation && filtered_soft_cost_count > 0) {
    if (node) {
      RCLCPP_DEBUG_THROTTLE(
        logger_, *node->get_clock(), 2000,
        "CostCritic 近目标过滤生效: 本轮忽略 %zu 个非 lethal 膨胀采样点, 生效距离 %.3fm, "
        "等效膨胀半径 %.3fm",
        filtered_soft_cost_count, near_goal_cost_relaxation_distance_,
        near_goal_effective_inflation_radius_);
    }
  }
}

/**
  * @brief Checks if cost represents a collision
  * @param cost Costmap cost
  * @return bool if in collision
  */
bool CostCritic::inCollision(float cost, float x, float y, float theta)
{
  bool is_tracking_unknown =
    costmap_ros_->getLayeredCostmap()->isTrackingUnknown();

  // If consider_footprint_ check footprint scort for collision
  if (consider_footprint_ &&
    (cost >= possibly_inscribed_cost_ || possibly_inscribed_cost_ < 1.0f))
  {
    cost = static_cast<float>(collision_checker_.footprintCostAtPose(
        x, y, theta, costmap_ros_->getRobotFootprint()));
  }

  switch (static_cast<unsigned char>(cost)) {
    using namespace nav2_costmap_2d; // NOLINT
    case (LETHAL_OBSTACLE):
      return true;
    case (INSCRIBED_INFLATED_OBSTACLE):
      return consider_footprint_ ? false : true;
    case (NO_INFORMATION):
      return is_tracking_unknown ? false : true;
  }

  return false;
}

float CostCritic::costAtPose(float x, float y)
{
  using namespace nav2_costmap_2d;   // NOLINT
  unsigned int x_i, y_i;
  if (!collision_checker_.worldToMap(x, y, x_i, y_i)) {
    return nav2_costmap_2d::NO_INFORMATION;
  }

  return collision_checker_.pointCost(x_i, y_i);
}

void CostCritic::applyProfileSettings(const CriticProfileSettings & settings)
{
  // 三阶段覆盖软代价权重；致命碰撞由 ObstaclesCritic critical 全局兜底
  weight_ = settings.cost.weight;
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::CostCritic,
  mppi::critics::CriticFunction)
