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

#include <algorithm>
#include <cmath>
#include "nav2_mppi_controller/critics/cost_critic.hpp"

namespace mppi::critics
{

namespace
{

enum class CollisionReason
{
  OutOfBounds,
  CenterLethal,
  CenterUnknown,
  FootprintLethal,
  FootprintUnknown,
  Other
};

const char * collisionReasonName(CollisionReason reason)
{
  switch (reason) {
    case CollisionReason::OutOfBounds:
      return "out_of_bounds";
    case CollisionReason::CenterLethal:
      return "center_lethal";
    case CollisionReason::CenterUnknown:
      return "center_unknown";
    case CollisionReason::FootprintLethal:
      return "footprint_lethal";
    case CollisionReason::FootprintUnknown:
      return "footprint_unknown";
    default:
      return "other";
  }
}

}  // namespace

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
    "Critic will collision check based on %s cost.",
    power_, critical_cost_, weight_, consider_footprint_ ?
    "footprint" : "circular");
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

  models::Vector repulsive_cost = models::Vector::Zero(data.costs.size());

  const size_t traj_len = data.trajectories.x.cols();
  const size_t trajectory_count = static_cast<size_t>(data.trajectories.x.rows());
  bool all_trajectories_collide = true;
  size_t collision_count = 0;
  size_t out_of_bounds_count = 0;
  size_t center_lethal_count = 0;
  size_t center_unknown_count = 0;
  size_t footprint_lethal_count = 0;
  size_t footprint_unknown_count = 0;
  size_t other_collision_count = 0;
  size_t earliest_collision_step = traj_len;
  size_t latest_collision_step = 0;
  double collision_step_sum = 0.0;
  float example_x = 0.0f;
  float example_y = 0.0f;
  float example_yaw = 0.0f;
  float example_center_cost = 0.0f;
  float example_evaluated_cost = 0.0f;
  CollisionReason example_reason = CollisionReason::Other;

  for (Eigen::Index i = 0; i < data.trajectories.x.rows(); ++i) {
    bool trajectory_collide = false;
    const auto & traj = data.trajectories;
    float pose_cost;

    for (size_t j = 0; j < traj_len; j++) {
      // The costAtPose doesn't use orientation
      // The footprintCostAtPose will always return "INSCRIBED" if footprint is over it
      // So the center point has more information than the footprint
      bool out_of_bounds = false;
      pose_cost = costAtPose(traj.x(i, j), traj.y(i, j), &out_of_bounds);
      if (pose_cost < 1.0f || out_of_bounds) {continue;}  // In free space or outside local costmap bounds

      float evaluated_cost = pose_cost;
      bool used_footprint = false;
      if (inCollision(
          pose_cost, traj.x(i, j), traj.y(i, j), traj.yaws(i, j),
          &evaluated_cost, &used_footprint))
      {
        trajectory_collide = true;
        ++collision_count;
        collision_step_sum += static_cast<double>(j);
        latest_collision_step = std::max(latest_collision_step, j);

        using namespace nav2_costmap_2d; // NOLINT
        CollisionReason reason = CollisionReason::Other;
        const auto center_cost_value = static_cast<unsigned char>(pose_cost);
        const auto evaluated_cost_value = static_cast<unsigned char>(evaluated_cost);
        if (out_of_bounds) {
          reason = CollisionReason::OutOfBounds;
          ++out_of_bounds_count;
        } else if (used_footprint && evaluated_cost_value == LETHAL_OBSTACLE) {
          reason = CollisionReason::FootprintLethal;
          ++footprint_lethal_count;
        } else if (used_footprint && evaluated_cost_value == NO_INFORMATION) {
          reason = CollisionReason::FootprintUnknown;
          ++footprint_unknown_count;
        } else if (!used_footprint && center_cost_value == LETHAL_OBSTACLE) {
          reason = CollisionReason::CenterLethal;
          ++center_lethal_count;
        } else if (!used_footprint && center_cost_value == NO_INFORMATION) {
          reason = CollisionReason::CenterUnknown;
          ++center_unknown_count;
        } else {
          ++other_collision_count;
        }

        if (j < earliest_collision_step) {
          earliest_collision_step = j;
          example_x = traj.x(i, j);
          example_y = traj.y(i, j);
          example_yaw = traj.yaws(i, j);
          example_center_cost = pose_cost;
          example_evaluated_cost = evaluated_cost;
          example_reason = reason;
        }
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

  if (all_trajectories_collide && costmap_) {
    const double origin_x = costmap_->getOriginX();
    const double origin_y = costmap_->getOriginY();
    const double size_x = costmap_->getSizeInMetersX();
    const double size_y = costmap_->getSizeInMetersY();
    const double map_center_x = origin_x + size_x * 0.5;
    const double map_center_y = origin_y + size_y * 0.5;
    const double robot_x = data.state.pose.pose.position.x;
    const double robot_y = data.state.pose.pose.position.y;
    const double average_collision_step = collision_count > 0 ?
      collision_step_sum / static_cast<double>(collision_count) : 0.0;

    RCLCPP_ERROR(
      logger_,
      "[MPPI][CostCritic] All trajectories collide: %zu/%zu trajectories, %zu steps. "
      "First-collision reasons: out_of_bounds=%zu, center_lethal=%zu, center_unknown=%zu, "
      "footprint_lethal=%zu, footprint_unknown=%zu, other=%zu. "
      "First-collision step (zero-based): min=%zu, avg=%.1f, max=%zu. "
      "Robot=(%.3f, %.3f), costmap origin=(%.3f, %.3f), size=(%.3f, %.3f), "
      "robot_offset_from_map_center=(%.3f, %.3f). "
      "Earliest example: source=%s, pose=(%.3f, %.3f, yaw=%.3f), "
      "center_cost=%.0f, evaluated_cost=%.0f.",
      collision_count, trajectory_count, traj_len,
      out_of_bounds_count, center_lethal_count, center_unknown_count,
      footprint_lethal_count, footprint_unknown_count, other_collision_count,
      earliest_collision_step, average_collision_step, latest_collision_step,
      robot_x, robot_y, origin_x, origin_y, size_x, size_y,
      robot_x - map_center_x, robot_y - map_center_y,
      collisionReasonName(example_reason), example_x, example_y, example_yaw,
      example_center_cost, example_evaluated_cost);
  }
}

/**
  * @brief Checks if cost represents a collision
  * @param cost Costmap cost
  * @return bool if in collision
  */
bool CostCritic::inCollision(
  float cost, float x, float y, float theta,
  float * evaluated_cost, bool * used_footprint)
{
  bool is_tracking_unknown =
    costmap_ros_->getLayeredCostmap()->isTrackingUnknown();

  if (used_footprint) {
    *used_footprint = false;
  }

  // If consider_footprint_ check footprint scort for collision
  if (consider_footprint_ &&
    (cost >= possibly_inscribed_cost_ || possibly_inscribed_cost_ < 1.0f))
  {
    cost = static_cast<float>(collision_checker_.footprintCostAtPose(
        x, y, theta, costmap_ros_->getRobotFootprint()));
    if (used_footprint) {
      *used_footprint = true;
    }
  }

  if (evaluated_cost) {
    *evaluated_cost = cost;
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

float CostCritic::costAtPose(float x, float y, bool * out_of_bounds)
{
  using namespace nav2_costmap_2d;   // NOLINT
  unsigned int x_i, y_i;
  if (out_of_bounds) {
    *out_of_bounds = false;
  }
  if (!collision_checker_.worldToMap(x, y, x_i, y_i)) {
    if (out_of_bounds) {
      *out_of_bounds = true;
    }
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
