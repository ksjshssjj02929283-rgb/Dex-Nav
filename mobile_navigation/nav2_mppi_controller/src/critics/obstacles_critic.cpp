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

#include <cmath>
#include "nav2_mppi_controller/critics/obstacles_critic.hpp"

namespace mppi::critics
{

void ObstaclesCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(consider_footprint_, "consider_footprint", false);
  getParam(power_, "cost_power", 1);
  getParam(repulsion_weight_, "repulsion_weight", 1.5);
  getParam(critical_weight_, "critical_weight", 20.0);
  getParam(collision_cost_, "collision_cost", 10000.0);
  getParam(collision_margin_distance_, "collision_margin_distance", 0.10);
  getParam(near_goal_distance_, "near_goal_distance", 0.5);

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
    "ObstaclesCritic instantiated with %d power and %f / %f weights. "
    "Critic will collision check based on %s cost. "
    "近目标 MPPI 软膨胀过滤: 生效距离=%.3fm, 等效膨胀半径=%.3fm.",
    power_, critical_weight_, repulsion_weight_, consider_footprint_ ?
    "footprint" : "circular",
    near_goal_cost_relaxation_distance_, near_goal_effective_inflation_radius_);

  const float inscribed_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  if (near_goal_effective_inflation_radius_ > 0.0f &&
    near_goal_effective_inflation_radius_ <= inscribed_radius)
  {
    RCLCPP_INFO(
      logger_,
      "ObstaclesCritic 近目标过滤模式: 等效膨胀半径 %.3fm <= 机器人内切半径 %.3fm，"
      "近目标时会忽略非 lethal 的膨胀 cost，仅保留 lethal/unknown。",
      near_goal_effective_inflation_radius_, inscribed_radius);
  }
}

float ObstaclesCritic::findCircumscribedCost(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  double result = -1.0;
  bool inflation_layer_found = false;

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
    if (!inflation_layer) {
      continue;
    }

    inflation_layer_found = true;
    const double resolution = costmap->getCostmap()->getResolution();
    result = inflation_layer->computeCost(circum_radius / resolution);
    auto getParam = parameters_handler_->getParamGetter(name_);
    getParam(inflation_scale_factor_, "cost_scaling_factor", 10.0);
    getParam(inflation_radius_, "inflation_radius", 0.55);
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

float ObstaclesCritic::distanceToObstacle(const CollisionCost & cost) const
{
  const float scale_factor = inflation_scale_factor_;
  const float min_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  float dist_to_obj = (scale_factor * min_radius - log(cost.cost) + log(253.0f)) / scale_factor;

  // If not footprint collision checking, the cost is using the center point cost and
  // needs the radius subtracted to obtain the closest distance to the object
  if (!cost.using_footprint) {
    dist_to_obj -= min_radius;
  }

  return dist_to_obj;
}

void ObstaclesCritic::relaxCostAtPose(
  CollisionCost & cost, bool near_goal_relaxation, size_t & filtered_soft_cost_count) const
{
  using namespace nav2_costmap_2d; // NOLINT

  if (!near_goal_relaxation ||
    near_goal_effective_inflation_radius_ <= 0.0f ||
    cost.cost < 1.0f)
  {
    return;
  }

  const auto cost_value = static_cast<unsigned char>(cost.cost);
  if (cost_value == LETHAL_OBSTACLE || cost_value == NO_INFORMATION) {
    return;
  }

  // 关键点：这里必须在 footprint 碰撞检查之前过滤。
  // 否则 0.2~0.4m 的软膨胀 cost 仍会触发 footprintCostAtPose()，
  // 行为就不等同于把 local costmap 的 inflation_radius 缩小到 0.2m。
  if (cost_value == INSCRIBED_INFLATED_OBSTACLE) {
    cost.cost = FREE_SPACE;
    cost.using_footprint = false;
    ++filtered_soft_cost_count;
    return;
  }

  const float inscribed_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  if (near_goal_effective_inflation_radius_ <= inscribed_radius) {
    cost.cost = FREE_SPACE;
    cost.using_footprint = false;
    ++filtered_soft_cost_count;
    return;
  }

  const float dist_to_obj = distanceToObstacle(cost);
  if (dist_to_obj >= near_goal_effective_inflation_radius_) {
    cost.cost = FREE_SPACE;
    cost.using_footprint = false;
    ++filtered_soft_cost_count;
  }
}

void ObstaclesCritic::score(CriticData & data)
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

  models::Vector raw_cost = models::Vector::Zero(data.costs.size());
  models::Vector repulsive_cost = models::Vector::Zero(data.costs.size());
  size_t filtered_soft_cost_count = 0;

  const size_t traj_len = data.trajectories.x.cols();
  bool all_trajectories_collide = true;
  for (Eigen::Index i = 0; i < data.trajectories.x.rows(); ++i) {
    bool trajectory_collide = false;
    float traj_cost = 0.0f;
    const auto & traj = data.trajectories;
    CollisionCost pose_cost;

    for (size_t j = 0; j < traj_len; j++) {
      pose_cost = costAtPose(
        traj.x(i, j), traj.y(i, j), traj.yaws(i, j), near_goal_relaxation,
        filtered_soft_cost_count);
      if (pose_cost.cost < 1.0f) {continue;}  // In free space

      if (inCollision(pose_cost.cost)) {
        trajectory_collide = true;
        break;
      }

      // Cannot process repulsion if inflation layer does not exist
      if (inflation_radius_ == 0.0f || inflation_scale_factor_ == 0.0f) {
        continue;
      }

      const float dist_to_obj = distanceToObstacle(pose_cost);

      // Let near-collision trajectory points be punished severely
      if (dist_to_obj < collision_margin_distance_) {
        traj_cost += (collision_margin_distance_ - dist_to_obj);
      } else if (!near_goal) {  // Generally prefer trajectories further from obstacles
        repulsive_cost(i) += (inflation_radius_ - dist_to_obj);
      }
    }

    if (!trajectory_collide) {all_trajectories_collide = false;}
    raw_cost(i) = trajectory_collide ? collision_cost_ : traj_cost;
  }

  data.costs += (
    (critical_weight_ * raw_cost) +
    (repulsion_weight_ * repulsive_cost / traj_len)).pow(power_);
  data.fail_flag = all_trajectories_collide;

  auto node = parent_.lock();
  if (near_goal_relaxation && node && !near_goal_relaxation_logged_) {
    RCLCPP_INFO(
      logger_,
      "ObstaclesCritic 进入近目标软膨胀过滤区: 距离阈值 %.3fm, 等效膨胀半径 %.3fm, "
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
        "ObstaclesCritic 近目标过滤生效: 本轮忽略 %zu 个非 lethal 膨胀采样点, 生效距离 %.3fm, "
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
bool ObstaclesCritic::inCollision(float cost) const
{
  bool is_tracking_unknown =
    costmap_ros_->getLayeredCostmap()->isTrackingUnknown();

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

CollisionCost ObstaclesCritic::costAtPose(
  float x, float y, float theta, bool near_goal_relaxation,
  size_t & filtered_soft_cost_count)
{
  CollisionCost collision_cost;
  float & cost = collision_cost.cost;
  collision_cost.using_footprint = false;
  unsigned int x_i, y_i;
  if (!collision_checker_.worldToMap(x, y, x_i, y_i)) {
    cost = nav2_costmap_2d::NO_INFORMATION;
    return collision_cost;
  }
  cost = collision_checker_.pointCost(x_i, y_i);
  relaxCostAtPose(collision_cost, near_goal_relaxation, filtered_soft_cost_count);
  if (cost < 1.0f) {
    return collision_cost;
  }

  if (consider_footprint_ &&
    (cost >= possibly_inscribed_cost_ || possibly_inscribed_cost_ < 1.0f))
  {
    cost = static_cast<float>(collision_checker_.footprintCostAtPose(
        x, y, theta, costmap_ros_->getRobotFootprint()));
    collision_cost.using_footprint = true;
  }

  return collision_cost;
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::ObstaclesCritic,
  mppi::critics::CriticFunction)
