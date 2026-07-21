// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_PATH_PRUNING_LOCAL_PLANNER__PATH_PRUNING_LOCAL_PLANNER_HPP_
#define NAV2_PATH_PRUNING_LOCAL_PLANNER__PATH_PRUNING_LOCAL_PLANNER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/local_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_path_pruning_local_planner
{

class PathPruningLocalPlanner : public nav2_core::LocalPlanner
{
public:
  PathPruningLocalPlanner() = default;
  ~PathPruningLocalPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  bool computeLocalPlan(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav_msgs::msg::Path & local_path) override;

private:
  struct Projection
  {
    bool valid{false};
    size_t segment_index{0};
    double ratio{0.0};
    double arc_length{0.0};
    double distance_sq{0.0};
  };

  void rebuildArcLengths();
  Projection projectOntoPlan(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double lower_arc, double upper_arc) const;
  geometry_msgs::msg::PoseStamped poseAtArcLength(
    double arc_length, size_t & segment_index) const;
  geometry_msgs::msg::PoseStamped interpolatePose(
    size_t segment_index, double ratio) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Logger logger_{rclcpp::get_logger("PathPruningLocalPlanner")};
  std::string plugin_name_;
  std::string plan_frame_;

  nav_msgs::msg::Path global_plan_;
  std::vector<double> cumulative_arc_;
  bool progress_initialized_{false};
  double progress_arc_{0.0};

  double max_search_distance_{5.0};
  double max_backtrack_distance_{0.5};
  double transform_tolerance_{0.2};
  std::mutex mutex_;
};

}  // namespace nav2_path_pruning_local_planner

#endif  // NAV2_PATH_PRUNING_LOCAL_PLANNER__PATH_PRUNING_LOCAL_PLANNER_HPP_
