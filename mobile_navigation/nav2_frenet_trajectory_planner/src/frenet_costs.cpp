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
  if (!costmap_) {
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
    if (cost == nav2_costmap_2d::NO_INFORMATION) {
      continue;
    }
    sum += static_cast<double>(cost) / static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
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
