#include "nav2_frenet_ilqr_controller/frenet_policies.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_frenet_ilqr_controller
{
namespace policies
{

void RclcppNodePolicy::initialize(
  const std::string & policy_name,
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  policy_name_ = policy_name;
  node_ = parent;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_ ? costmap_ros_->getCostmap() : nullptr;
}

void ObstaclePolicy::initialize(
  const std::string & policy_name,
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  RclcppNodePolicy::initialize(policy_name, parent, costmap_ros);
  auto node = parent.lock();
  if (!node) {
    return;
  }
  nav2_util::declare_parameter_if_not_declared(
    node, policy_name + ".obstacle_cost_threshold",
    rclcpp::ParameterValue(253));
  node->get_parameter(policy_name + ".obstacle_cost_threshold", obstacle_cost_threshold_);
  obstacle_cost_threshold_ = std::clamp(obstacle_cost_threshold_, 1, 254);
}

bool ObstaclePolicy::checkIfFeasible(
  const frenet_planner::FrenetTrajectory &,
  const frenet_planner::CartesianTrajectory & cartesian_trajectory)
{
  if (!costmap_) {
    return true;
  }
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
    if (static_cast<int>(cost) >= obstacle_cost_threshold_) {
      return false;
    }
  }
  return true;
}

void ConstraintsPolicy::initialize(
  const std::string & policy_name,
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  RclcppNodePolicy::initialize(policy_name, parent, costmap_ros);
  auto node = parent.lock();
  if (!node) {
    return;
  }
  nav2_util::declare_parameter_if_not_declared(
    node, policy_name + ".max_speed", rclcpp::ParameterValue(
      0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, policy_name + ".max_acceleration", rclcpp::ParameterValue(
      2.5));
  node->get_parameter(policy_name + ".max_speed", max_speed_);
  node->get_parameter(policy_name + ".max_acceleration", max_acceleration_);
  max_speed_ = std::max(0.0, max_speed_);
  max_acceleration_ = std::max(0.0, max_acceleration_);
}

bool ConstraintsPolicy::checkIfFeasible(
  const frenet_planner::FrenetTrajectory &,
  const frenet_planner::CartesianTrajectory & cartesian_trajectory)
{
  for (const auto & state : cartesian_trajectory) {
    if (!state.allFinite()) {
      return false;
    }
    if (std::hypot(state[1], state[4]) > max_speed_ + 1e-6) {
      return false;
    }
    if (std::hypot(state[2], state[5]) > max_acceleration_ + 1e-6) {
      return false;
    }
  }
  return true;
}

void AccelerationPolicy::initialize(
  const std::string & policy_name,
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  RclcppNodePolicy::initialize(policy_name, parent, costmap_ros);
  auto node = parent.lock();
  if (!node) {
    return;
  }
  nav2_util::declare_parameter_if_not_declared(
    node, policy_name + ".min_acceleration", rclcpp::ParameterValue(
      0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, policy_name + ".max_acceleration", rclcpp::ParameterValue(
      2.5));
  node->get_parameter(policy_name + ".min_acceleration", min_acceleration_);
  node->get_parameter(policy_name + ".max_acceleration", max_acceleration_);
  max_acceleration_ = std::max(0.0, max_acceleration_);
  min_acceleration_ = std::min(min_acceleration_, max_acceleration_);
}

bool AccelerationPolicy::checkIfFeasible(
  const frenet_planner::FrenetTrajectory &,
  const frenet_planner::CartesianTrajectory & cartesian_trajectory)
{
  for (const auto & state : cartesian_trajectory) {
    if (!state.allFinite()) {
      return false;
    }
    const double acceleration = std::hypot(state[2], state[5]);
    if (acceleration < min_acceleration_ - 1e-6 || acceleration > max_acceleration_ + 1e-6) {
      return false;
    }
  }
  return true;
}

}  // namespace policies
}  // namespace nav2_frenet_ilqr_controller

PLUGINLIB_EXPORT_CLASS(
  nav2_frenet_ilqr_controller::policies::ObstaclePolicy,
  nav2_frenet_ilqr_controller::policies::RclcppNodePolicy)
PLUGINLIB_EXPORT_CLASS(
  nav2_frenet_ilqr_controller::policies::ConstraintsPolicy,
  nav2_frenet_ilqr_controller::policies::RclcppNodePolicy)
PLUGINLIB_EXPORT_CLASS(
  nav2_frenet_ilqr_controller::policies::AccelerationPolicy,
  nav2_frenet_ilqr_controller::policies::RclcppNodePolicy)
