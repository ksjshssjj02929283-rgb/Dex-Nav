#ifndef NAV2_FRENET_ILQR_CONTROLLER__FRENET_POLICIES_HPP_
#define NAV2_FRENET_ILQR_CONTROLLER__FRENET_POLICIES_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_frenet_ilqr_controller/frenet_trajectory_planner.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_frenet_ilqr_controller
{
namespace policies
{

class RclcppNodePolicy : public frenet_planner::policies::Policy
{
public:
  virtual void initialize(
    const std::string & policy_name,
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros);

protected:
  std::string policy_name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
};

class ObstaclePolicy : public RclcppNodePolicy
{
public:
  void initialize(
    const std::string & policy_name,
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;
  bool checkIfFeasible(
    const frenet_planner::FrenetTrajectory & frenet_trajectory,
    const frenet_planner::CartesianTrajectory & cartesian_trajectory) override;

private:
  int obstacle_cost_threshold_{253};
};

class ConstraintsPolicy : public RclcppNodePolicy
{
public:
  void initialize(
    const std::string & policy_name,
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;
  bool checkIfFeasible(
    const frenet_planner::FrenetTrajectory & frenet_trajectory,
    const frenet_planner::CartesianTrajectory & cartesian_trajectory) override;

private:
  double max_speed_{0.8};
  double max_acceleration_{2.5};
};

class AccelerationPolicy : public RclcppNodePolicy
{
public:
  void initialize(
    const std::string & policy_name,
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;
  bool checkIfFeasible(
    const frenet_planner::FrenetTrajectory & frenet_trajectory,
    const frenet_planner::CartesianTrajectory & cartesian_trajectory) override;

private:
  double min_acceleration_{0.0};
  double max_acceleration_{2.5};
};

}  // namespace policies
}  // namespace nav2_frenet_ilqr_controller

#endif  // NAV2_FRENET_ILQR_CONTROLLER__FRENET_POLICIES_HPP_
