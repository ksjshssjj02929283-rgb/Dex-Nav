// Copyright (c) 2022 Samsung R&D Institute Russia
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

#ifndef NAV2_COLLISION_MONITOR__COLLISION_MONITOR_NODE_HPP_
#define NAV2_COLLISION_MONITOR__COLLISION_MONITOR_NODE_HPP_

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_interfaces/msg/obstacle_monitor_status.hpp"
#include "nav_interfaces/srv/set_polygon_padding.hpp"

#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"

#include "nav2_collision_monitor/types.hpp"
#include "nav2_collision_monitor/polygon.hpp"
#include "nav2_collision_monitor/circle.hpp"
#include "nav2_collision_monitor/source.hpp"
#include "nav2_collision_monitor/scan.hpp"
#include "nav2_collision_monitor/pointcloud.hpp"
#include "nav2_collision_monitor/range.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Collision Monitor ROS2 node
 */
class CollisionMonitor : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief Constructor for the nav2_collision_safery::CollisionMonitor
   * @param options Additional options to control creation of the node.
   */
  explicit CollisionMonitor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief Destructor for the nav2_collision_safery::CollisionMonitor
   */
  ~CollisionMonitor();

protected:
  /**
   * @brief: Initializes and obtains ROS-parameters, creates main subscribers and publishers,
   * creates polygons and data sources objects
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief: Activates LifecyclePublishers, polygons and main processor, creates bond connection
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief: Deactivates LifecyclePublishers, polygons and main processor, destroys bond connection
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief: Resets all subscribers/publishers, polygons/data sources arrays
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called in shutdown state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

protected:
  /**
   * @brief Callback for input cmd_vel
   * @param msg Input cmd_vel message
   */
  void cmdVelInCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  /**
   * @brief Publishes output cmd_vel. If robot was stopped more than stop_pub_timeout_ seconds,
   * quit to publish 0-velocity.
   * @param robot_action Robot action to publish
   */
  void publishVelocity(const Action & robot_action);

  /**
   * @brief Publishes real-time obstacle monitor status (FREE/SLOWDOWN/STOP).
   * Called every process() cycle, regardless of whether the state changed.
   * @param robot_action Robot action computed for the current cycle
   * @param action_polygon Polygon causing robot_action (nullptr if DO_NOTHING)
   */
  void publishObstacleStatus(
    const Action & robot_action, const std::shared_ptr<Polygon> action_polygon) const;

  /**
   * @brief Supporting routine obtaining all ROS-parameters
   * @param cmd_vel_in_topic Output name of cmd_vel_in topic
   * @param cmd_vel_out_topic Output name of cmd_vel_out topic
   * is required.
   * @return True if all parameters were obtained or false in failure case
   */
  bool getParameters(
    std::string & cmd_vel_in_topic,
    std::string & cmd_vel_out_topic);
  /**
   * @brief Supporting routine creating and configuring all polygons
   * @param base_frame_id Robot base frame ID
   * @param transform_tolerance Transform tolerance
   * @return True if all polygons were configured successfully or false in failure case
   */
  bool configurePolygons(
    const std::string & base_frame_id,
    const tf2::Duration & transform_tolerance);
  /**
   * @brief Supporting routine creating and configuring all data sources
   * @param base_frame_id Robot base frame ID
   * @param odom_frame_id Odometry frame ID. Used as global frame to get
   * source->base time inerpolated transform.
   * @param transform_tolerance Transform tolerance
   * @param source_timeout Maximum time interval in which data is considered valid
   * @param base_shift_correction Whether to correct source data towards to base frame movement,
   * considering the difference between current time and latest source time
   * @return True if all sources were configured successfully or false in failure case
   */
  bool configureSources(
    const std::string & base_frame_id,
    const std::string & odom_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & source_timeout,
    const bool base_shift_correction);

  /**
   * @brief Main processing routine
   * @param cmd_vel_in Input desired robot velocity
   * @param publish_cmd_vel Whether to publish the gated output cmd_vel. False for the idle
   * keep-alive path (idleKeepaliveCallback), which only refreshes obstacle_status/polygons:
   * the monitor must gate velocities, never originate them.
   */
  void process(const Velocity & cmd_vel_in, bool publish_cmd_vel = true);

  /**
   * @brief Idle keep-alive timer callback.
   *
   * obstacle_status is otherwise published only when cmd_vel_in arrives, which makes a safety
   * signal's rate depend on a downstream consumer's output. Two consequences motivated this:
   * a controller cannot distinguish "monitor died" from "monitor has nothing to react to"
   * (so it cannot run a staleness watchdog), and an obstacle present before the robot ever
   * moves stays unreported until the first non-zero velocity is published - too late.
   *
   * This fires only when no cmd_vel_in has arrived within the timer period, so the behaviour
   * and latency of the cmd_vel path are untouched while the robot is being commanded.
   * Evaluating with zero velocity is correct for STOP/SLOWDOWN polygons (their detection is
   * purely geometric); APPROACH degenerates to no-op, which is its meaning at zero velocity.
   */
  void idleKeepaliveCallback();

  /**
   * @brief Processes the polygon of STOP and SLOWDOWN action type
   * @param polygon Polygon to process
   * @param collision_points Array of 2D obstacle points
   * @param velocity Desired robot velocity
   * @param robot_action Output processed robot action
   * @return True if returned action is caused by current polygon, otherwise false
   */
  bool processStopSlowdown(
    const std::shared_ptr<Polygon> polygon,
    const std::vector<Point> & collision_points,
    const Velocity & velocity,
    Action & robot_action) const;

  /**
   * @brief Processes APPROACH action type
   * @param polygon Polygon to process
   * @param collision_points Array of 2D obstacle points
   * @param velocity Desired robot velocity
   * @param robot_action Output processed robot action
   * @return True if returned action is caused by current polygon, otherwise false
   */
  bool processApproach(
    const std::shared_ptr<Polygon> polygon,
    const std::vector<Point> & collision_points,
    const Velocity & velocity,
    Action & robot_action) const;

  /**
   * @brief Prints robot action and polygon caused it (if it was)
   * @param robot_action Robot action to print
   * @param action_polygon Pointer to a polygon causing a selected action
   */
  void printAction(
    const Action & robot_action, const std::shared_ptr<Polygon> action_polygon) const;

  /**
   * @brief Polygons publishing routine. Made for visualization.
   */
  void publishPolygons() const;

  /**
   * @brief 设置检测器膨胀大小的服务回调（本仓库相对上游 Nav2 的改动：动态停障距离）。
   *
   * 只写各 Polygon 的 padding_（atomic），不在这里改几何——形状在 process() 线程
   * 按当前 padding 现算，避免与碰撞判定竞争。
   * @param request 目标检测器名（空=全部）与 padding（NaN=恢复默认）
   * @param response 生效结果
   */
  void setPolygonPaddingCallback(
    const std::shared_ptr<nav_interfaces::srv::SetPolygonPadding::Request> request,
    std::shared_ptr<nav_interfaces::srv::SetPolygonPadding::Response> response);

  // ----- Variables -----

  /// @brief TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  /// @brief TF listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /// @brief Polygons array
  std::vector<std::shared_ptr<Polygon>> polygons_;

  /// @brief Data sources array
  std::vector<std::shared_ptr<Source>> sources_;

  // Input/output speed controls
  /// @beirf Input cmd_vel subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_in_sub_;
  /// @brief Output cmd_vel publisher
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_out_pub_;
  /// @brief Real-time obstacle monitor status publisher (FREE/SLOWDOWN/STOP)
  rclcpp_lifecycle::LifecyclePublisher<
    nav_interfaces::msg::ObstacleMonitorStatus>::SharedPtr obstacle_status_pub_;
  /// @brief 动态膨胀设置服务（本仓库相对上游 Nav2 的改动）
  rclcpp::Service<nav_interfaces::srv::SetPolygonPadding>::SharedPtr set_padding_srv_;

  /// @brief Whether main routine is active
  bool process_active_;

  /// @brief Robot base frame ID, used as obstacle_status.header.frame_id
  std::string base_frame_id_;
  /// @brief Previous robot action
  Action robot_action_prev_;
  /// @brief Latest timestamp when robot has 0-velocity
  rclcpp::Time stop_stamp_;
  /// @brief Timeout after which 0-velocity ceases to be published
  rclcpp::Duration stop_pub_timeout_;

  // Idle keep-alive: keeps obstacle_status alive while no cmd_vel_in is flowing.
  /// @brief Idle keep-alive timer (null when the feature is disabled)
  rclcpp::TimerBase::SharedPtr idle_keepalive_timer_;
  /// @brief Idle keep-alive period; also the "recent cmd_vel_in" threshold
  rclcpp::Duration idle_keepalive_period_;
  /// @brief Keep-alive rate Hz; <= 0 disables the timer entirely
  double idle_keepalive_rate_;
  /// @brief Latest timestamp when cmd_vel_in was received
  rclcpp::Time last_cmd_vel_in_stamp_;
};  // class CollisionMonitor

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__COLLISION_MONITOR_NODE_HPP_
