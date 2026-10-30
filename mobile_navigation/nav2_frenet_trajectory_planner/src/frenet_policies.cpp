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

  nav2_util::declare_parameter_if_not_declared(
    node, policy_name + ".footprint_padding_m",
    rclcpp::ParameterValue(0.04));
  node->get_parameter(policy_name + ".footprint_padding_m", footprint_padding_m_);
}

namespace
{

int evaluateFootprintMaxCost(
  const nav2_costmap_2d::Costmap2D * costmap,
  double x,
  double y,
  double yaw,
  double padding_m)
{
  if (!costmap) {
    return 0;
  }
  // Robot chassis footprint: 0.70m x 0.50m (half_len = 0.35m, half_wid = 0.25m) + padding
  const double hl = 0.35 + padding_m;
  const double hw = 0.25 + padding_m;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  // Check center, 4 corners, and 4 edge midpoints of the robot footprint
  const std::pair<double, double> offsets[] = {
    {0.0, 0.0},
    {hl, hw}, {hl, -hw}, {-hl, hw}, {-hl, -hw},
    {hl, 0.0}, {-hl, 0.0}, {0.0, hw}, {0.0, -hw}
  };

  int max_cost = 0;
  for (const auto & off : offsets) {
    const double wx = x + c * off.first - s * off.second;
    const double wy = y + s * off.first + c * off.second;
    unsigned int mx = 0U;
    unsigned int my = 0U;
    if (!costmap->worldToMap(wx, wy, mx, my)) {
      continue;
    }
    const unsigned char cost = costmap->getCost(mx, my);
    if (cost == nav2_costmap_2d::NO_INFORMATION) {
      continue;
    }
    max_cost = std::max(max_cost, static_cast<int>(cost));
  }
  return max_cost;
}

}  // namespace

bool ObstaclePolicy::checkIfFeasible(
  const frenet_planner::FrenetTrajectory &,
  const frenet_planner::CartesianTrajectory & cartesian_trajectory)
{
  if (!costmap_ || cartesian_trajectory.empty()) {
    return true;
  }
  const double start_x = cartesian_trajectory.front()[0];
  const double start_y = cartesian_trajectory.front()[3];

  for (const auto & state : cartesian_trajectory) {
    const int max_cost = evaluateFootprintMaxCost(
      costmap_, state[0], state[3], state[6], footprint_padding_m_);
    
    // 起步脱困容差保护：在距起点位移 < 0.40m (车体半长与过渡区) 内，
    // 只要不触碰致死障碍 (max_cost < LETHAL_OBSTACLE 即 254)，允许向外侧绕障，
    // 防止低速下前几个离散点均位于膨胀区边缘导致全部候选轨迹被误杀死锁
    const double dist_from_start = std::hypot(state[0] - start_x, state[3] - start_y);
    if (dist_from_start < 0.40) {
      if (max_cost >= static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE)) {
        return false;
      }
    } else {
      if (max_cost >= obstacle_cost_threshold_) {
        return false;
      }
    }
  }
  return true;
}

bool ObstaclePolicy::findViolation(
  const frenet_planner::CartesianTrajectory & cartesian_trajectory,
  frenet_planner::PolicyViolation & violation) const
{
  if (!costmap_ || cartesian_trajectory.empty()) {
    return false;
  }
  const double start_x = cartesian_trajectory.front()[0];
  const double start_y = cartesian_trajectory.front()[3];

  for (std::size_t i = 0U; i < cartesian_trajectory.size(); ++i) {
    const auto & state = cartesian_trajectory[i];
    const int max_cost = evaluateFootprintMaxCost(
      costmap_, state[0], state[3], state[6], footprint_padding_m_);
    
    const double dist_from_start = std::hypot(state[0] - start_x, state[3] - start_y);
    bool is_violating = false;
    if (dist_from_start < 0.40) {
      if (max_cost >= static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE)) {
        is_violating = true;
      }
    } else {
      if (max_cost >= obstacle_cost_threshold_) {
        is_violating = true;
      }
    }

    if (is_violating) {
      violation.valid = true;
      violation.state_index = i;
      violation.x = state[0];
      violation.y = state[3];
      violation.cost = max_cost;
      return true;
    }
  }
  return false;
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
      1.2));
  nav2_util::declare_parameter_if_not_declared(
    node, policy_name + ".max_acceleration", rclcpp::ParameterValue(
      3.5));
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

bool ConstraintsPolicy::findViolation(
  const frenet_planner::CartesianTrajectory & cartesian_trajectory,
  frenet_planner::PolicyViolation & violation) const
{
  for (std::size_t i = 0U; i < cartesian_trajectory.size(); ++i) {
    const auto & state = cartesian_trajectory[i];
    if (!state.allFinite()) {
      violation.valid = true;
      violation.state_index = i;
      violation.non_finite = true;
      return true;
    }
    const double speed = std::hypot(state[1], state[4]);
    if (speed > max_speed_ + 1e-6) {
      violation.valid = true;
      violation.state_index = i;
      violation.speed = speed;
      return true;
    }
    const double acc = std::hypot(state[2], state[5]);
    if (acc > max_acceleration_ + 1e-6) {
      violation.valid = true;
      violation.state_index = i;
      violation.acceleration = acc;
      return true;
    }
  }
  return false;
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
