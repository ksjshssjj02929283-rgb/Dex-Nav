// Copyright (C) 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// ROS 2 Nav2 局部几何避障规划插件：实现冻结接口 nav2_core::LocalPlanner
// 只负责根据全局参考路径与实时 Costmap2D 生成无碰撞的 Frenet 采样局部路径，
// 底层速度控制交由 controller_server 中的下游控制器执行。

#ifndef NAV2_FRENET_ILQR_CONTROLLER__FRENET_LOCAL_PLANNER_HPP_
#define NAV2_FRENET_ILQR_CONTROLLER__FRENET_LOCAL_PLANNER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/local_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_frenet_ilqr_controller/frenet_costs.hpp"
#include "nav2_frenet_ilqr_controller/frenet_policies.hpp"
#include "nav2_frenet_ilqr_controller/frenet_trajectory_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace nav2_frenet_ilqr_controller
{

class FrenetLocalPlanner : public nav2_core::LocalPlanner
{
public:
  FrenetLocalPlanner() = default;
  ~FrenetLocalPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
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
    std::size_t segment_index{0U};
    double ratio{0.0};
    double arc_length{0.0};
    double lateral_offset{0.0};
    double yaw{0.0};
    Eigen::Vector2d point{Eigen::Vector2d::Zero()};
  };

  struct PreparedPlan
  {
    nav_msgs::msg::Path path;
    geometry_msgs::msg::PoseStamped pose;
    std::vector<double> cumulative_arc;
    bool transformed{false};
  };

  void rebuildArcLengths();
  Projection projectOntoPath(
    const nav_msgs::msg::Path & path,
    const std::vector<double> & cumulative_arc,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double lower_arc,
    double upper_arc) const;
  PreparedPlan preparePlanInCostmapFrame(const geometry_msgs::msg::PoseStamped & robot_pose) const;
  std::vector<frenet_planner::CartesianPoint> buildFrenetWaypoints(
    const nav_msgs::msg::Path & path,
    const Projection & projection) const;
  frenet_planner::CartesianState makeCartesianState(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) const;
  void appendPathTail(
    const frenet_planner::FrenetTrajectoryPlanner::PlanResult & result,
    const nav_msgs::msg::Path & path,
    const std::vector<double> & cumulative_arc,
    const Projection & projection,
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    nav_msgs::msg::Path & local_path) const;
  void loadPolicies(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);
  void loadCosts(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);
  void publishCandidates(
    const frenet_planner::DebugInfo & debug_info,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const;
  void setPoseFromCartesian(
    const frenet_planner::CartesianState & state,
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    geometry_msgs::msg::PoseStamped & pose) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("FrenetLocalPlanner")};
  rclcpp::Clock::SharedPtr clock_;

  nav_msgs::msg::Path raw_global_plan_;
  std::vector<double> cumulative_arc_;
  int last_projection_index_{-1};
  std::mutex mtx_;

  frenet_planner::FrenetTrajectoryPlannerConfig frenet_config_;
  std::unique_ptr<frenet_planner::FrenetTrajectoryPlanner> frenet_planner_;
  std::unique_ptr<pluginlib::ClassLoader<policies::RclcppNodePolicy>> policy_loader_;
  std::unique_ptr<pluginlib::ClassLoader<costs::RclcppNodeCost>> cost_loader_;
  std::vector<std::shared_ptr<policies::RclcppNodePolicy>> policies_;
  std::vector<std::shared_ptr<costs::RclcppNodeCost>> costs_;

  double waypoint_resolution_{0.25};
  double projection_search_distance_{5.0};
  double max_backtrack_distance_{0.5};
  double transform_tolerance_{0.2};
  double tail_rejoin_distance_{1.5};
  bool visualize_candidate_trajectories_{false};
  int max_visualized_candidates_{80};
  std::string candidate_paths_topic_{"/frenet_rpp_controller/candidate_paths"};
  std::string rollouts_topic_{"/controller_server/frenet_rollouts"};

  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    candidate_paths_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    rollouts_pub_;
};

}  // namespace nav2_frenet_ilqr_controller

#endif  // NAV2_FRENET_ILQR_CONTROLLER__FRENET_LOCAL_PLANNER_HPP_
