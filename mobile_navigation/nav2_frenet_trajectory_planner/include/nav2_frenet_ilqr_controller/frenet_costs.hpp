#ifndef NAV2_FRENET_ILQR_CONTROLLER__FRENET_COSTS_HPP_
#define NAV2_FRENET_ILQR_CONTROLLER__FRENET_COSTS_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_frenet_ilqr_controller/frenet_trajectory_planner.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_frenet_ilqr_controller
{
namespace costs
{

class RclcppNodeCost : public frenet_planner::costs::Cost
{
public:
  virtual void initialize(
    const std::string & cost_name,
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros);

protected:
  std::string cost_name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
};

class LateralDistanceCost : public RclcppNodeCost
{
public:
  void initialize(
    const std::string & cost_name,
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;
  double cost(
    const frenet_planner::FrenetTrajectory & frenet_trajectory,
    const frenet_planner::CartesianTrajectory & cartesian_trajectory) override;

private:
  double lateral_distance_weight_{5.0};
};

class LongitudinalVelocityCost : public RclcppNodeCost
{
public:
  void initialize(
    const std::string & cost_name,
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;
  double cost(
    const frenet_planner::FrenetTrajectory & frenet_trajectory,
    const frenet_planner::CartesianTrajectory & cartesian_trajectory) override;

private:
  double longitudinal_velocity_weight_{5.0};
  double desired_velocity_{0.5};
  double distance_to_approach_{0.8};
  double desired_velocity_to_approach_{0.1};
};

class LongtitutalVelocityCost : public LongitudinalVelocityCost
{
};

class LatLonBalanceCost : public RclcppNodeCost
{
public:
  void initialize(
    const std::string & cost_name,
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;
  double cost(
    const frenet_planner::FrenetTrajectory & frenet_trajectory,
    const frenet_planner::CartesianTrajectory & cartesian_trajectory) override;

private:
  double balance_weight_{15.0};
};

class ObstacleCost : public RclcppNodeCost
{
public:
  void initialize(
    const std::string & cost_name,
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;
  double cost(
    const frenet_planner::FrenetTrajectory & frenet_trajectory,
    const frenet_planner::CartesianTrajectory & cartesian_trajectory) override;

private:
  double obstacle_weight_{2.0};
};

}  // namespace costs
}  // namespace nav2_frenet_ilqr_controller

#endif  // NAV2_FRENET_ILQR_CONTROLLER__FRENET_COSTS_HPP_
