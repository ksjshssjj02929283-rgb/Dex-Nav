#include "nav2_frenet_ilqr_controller/frenet_costs.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_frenet_ilqr_controller
{
namespace costs
{

void RclcppNodeCost::initialize(
  const std::string & cost_name,
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  cost_name_ = cost_name;
  node_ = parent;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_ ? costmap_ros_->getCostmap() : nullptr;
}

void LateralDistanceCost::initialize(
  const std::string & cost_name,
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  RclcppNodeCost::initialize(cost_name, parent, costmap_ros);
  auto node = parent.lock();
  if (!node) {
    return;
  }
  nav2_util::declare_parameter_if_not_declared(
    node, cost_name + ".K_lateral_distance", rclcpp::ParameterValue(
      5.0));
  node->get_parameter(cost_name + ".K_lateral_distance", lateral_distance_weight_);
}

double LateralDistanceCost::cost(
  const frenet_planner::FrenetTrajectory & frenet_trajectory,
  const frenet_planner::CartesianTrajectory &)
{
  double sum = 0.0;
  for (const auto & state : frenet_trajectory) {
    sum += std::abs(state[3]);
  }
  return lateral_distance_weight_ * sum;
}

void LongitudinalVelocityCost::initialize(
  const std::string & cost_name,
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  RclcppNodeCost::initialize(cost_name, parent, costmap_ros);
  auto node = parent.lock();
  if (!node) {
    return;
  }
  const bool canonical_was_set = node->has_parameter(cost_name + ".K_longitudinal_velocity");
  nav2_util::declare_parameter_if_not_declared(
    node, cost_name + ".K_longitudinal_velocity",
    rclcpp::ParameterValue(5.0));
  nav2_util::declare_parameter_if_not_declared(
    node, cost_name + ".K_longtitutal_velocity",
    rclcpp::ParameterValue(5.0));
  nav2_util::declare_parameter_if_not_declared(
    node, cost_name + ".desired_velocity", rclcpp::ParameterValue(
      0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, cost_name + ".distance_to_approach",
    rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(
    node, cost_name + ".desired_velocity_to_approach",
    rclcpp::ParameterValue(0.1));
  node->get_parameter(cost_name + ".K_longitudinal_velocity", longitudinal_velocity_weight_);
  if (!canonical_was_set) {
    double legacy_weight = longitudinal_velocity_weight_;
    node->get_parameter(cost_name + ".K_longtitutal_velocity", legacy_weight);
    if (std::abs(legacy_weight - longitudinal_velocity_weight_) > 1e-6) {
      longitudinal_velocity_weight_ = legacy_weight;
    }
  }
  node->get_parameter(cost_name + ".desired_velocity", desired_velocity_);
  node->get_parameter(cost_name + ".distance_to_approach", distance_to_approach_);
  node->get_parameter(cost_name + ".desired_velocity_to_approach", desired_velocity_to_approach_);
}

double LongitudinalVelocityCost::cost(
  const frenet_planner::FrenetTrajectory & frenet_trajectory,
  const frenet_planner::CartesianTrajectory &)
{
  double sum = 0.0;
  for (const auto & state : frenet_trajectory) {
    const double desired_velocity =
      info_.arclength - state[0] <
      distance_to_approach_ ? desired_velocity_to_approach_ : desired_velocity_;
    sum += std::abs(state[1] - desired_velocity);
  }
  return longitudinal_velocity_weight_ * sum;
}

void LatLonBalanceCost::initialize(
  const std::string & cost_name,
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  RclcppNodeCost::initialize(cost_name, parent, costmap_ros);
  auto node = parent.lock();
  if (!node) {
    return;
  }
  nav2_util::declare_parameter_if_not_declared(
    node, cost_name + ".K_latlon_balance", rclcpp::ParameterValue(
      15.0));
  node->get_parameter(cost_name + ".K_latlon_balance", balance_weight_);
}

double LatLonBalanceCost::cost(
  const frenet_planner::FrenetTrajectory & frenet_trajectory,
  const frenet_planner::CartesianTrajectory &)
{
  double sum = 0.0;
  for (const auto & state : frenet_trajectory) {
    sum += std::abs(state[1] * state[4]);
  }
  return balance_weight_ * sum;
}

void ObstacleCost::initialize(
  const std::string & cost_name,
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  RclcppNodeCost::initialize(cost_name, parent, costmap_ros);
  auto node = parent.lock();
  if (!node) {
    return;
  }
  nav2_util::declare_parameter_if_not_declared(
    node, cost_name + ".K_obstacle", rclcpp::ParameterValue(
      2.0));
  node->get_parameter(cost_name + ".K_obstacle", obstacle_weight_);
}

double ObstacleCost::cost(
  const frenet_planner::FrenetTrajectory &,
  const frenet_planner::CartesianTrajectory & cartesian_trajectory)
{
  if (!costmap_ || cartesian_trajectory.empty()) {
    return 0.0;
  }
  double sum = 0.0;
  for (const auto & state : cartesian_trajectory) {
    unsigned int map_x = 0U;
    unsigned int map_y = 0U;
    if (!costmap_->worldToMap(state[0], state[3], map_x, map_y)) {
      continue;
    }
    const unsigned char cost = costmap_->getCost(map_x, map_y);
    if (cost == nav2_costmap_2d::NO_INFORMATION || cost == 0) {
      continue;
    }
    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      sum += 100.0;
    } else {
      const double norm = static_cast<double>(cost) / 254.0;
      sum += norm * norm * 15.0;
    }
  }

  // 前瞻走廊阻挡代价 (Lookahead corridor obstacle cost)：
  // 沿轨迹末端航向向前探测 3.5m。若轨迹末端正对障碍物（直行走向死胡同），
  // 施加强大的前瞻惩罚，促使规划器在远处就主动选择向无障碍侧横向变道，
  // 避免短视 Horizon 下在障碍物鼻尖前因距离不足而无法变道停死。
  const auto & end_state = cartesian_trajectory.back();
  const double end_x = end_state[0];
  const double end_y = end_state[3];
  const double end_yaw = end_state[6];
  const double cos_yaw = std::cos(end_yaw);
  const double sin_yaw = std::sin(end_yaw);

  constexpr double kMaxLookahead = 3.5;
  constexpr double kStepLookahead = 0.15;
  for (double dist = 0.2; dist <= kMaxLookahead; dist += kStepLookahead) {
    const double check_x = end_x + dist * cos_yaw;
    const double check_y = end_y + dist * sin_yaw;
    unsigned int map_x = 0U;
    unsigned int map_y = 0U;
    if (costmap_->worldToMap(check_x, check_y, map_x, map_y)) {
      const unsigned char cost = costmap_->getCost(map_x, map_y);
      if (cost >= 200 && cost != nav2_costmap_2d::NO_INFORMATION) {
        sum += (kMaxLookahead - dist + 0.5) * 8.0;
        break;
      }
    }
  }

  return obstacle_weight_ * sum;
}

}  // namespace costs
}  // namespace nav2_frenet_ilqr_controller

PLUGINLIB_EXPORT_CLASS(
  nav2_frenet_ilqr_controller::costs::LateralDistanceCost,
  nav2_frenet_ilqr_controller::costs::RclcppNodeCost)
PLUGINLIB_EXPORT_CLASS(
  nav2_frenet_ilqr_controller::costs::LongitudinalVelocityCost,
  nav2_frenet_ilqr_controller::costs::RclcppNodeCost)
PLUGINLIB_EXPORT_CLASS(
  nav2_frenet_ilqr_controller::costs::LongtitutalVelocityCost,
  nav2_frenet_ilqr_controller::costs::RclcppNodeCost)
PLUGINLIB_EXPORT_CLASS(
  nav2_frenet_ilqr_controller::costs::LatLonBalanceCost,
  nav2_frenet_ilqr_controller::costs::RclcppNodeCost)
PLUGINLIB_EXPORT_CLASS(
  nav2_frenet_ilqr_controller::costs::ObstacleCost,
  nav2_frenet_ilqr_controller::costs::RclcppNodeCost)
