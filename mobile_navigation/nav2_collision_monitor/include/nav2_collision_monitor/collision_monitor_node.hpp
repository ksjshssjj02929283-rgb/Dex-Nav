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

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_interfaces/msg/obstacle_monitor_status.hpp"
#include "nav_interfaces/msg/tracking_path.hpp"
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
  enum class NavigationMode
  {
    IDLE,
    AUTOMONOUS_NAVIGATION,
    SCHEDULE_NAVIGATION,
    SCHEDULE_EDGING
  };

  enum class NavigationType
  {
    UNSET,
    AUTOMONOUS,
    SCHEDULE
  };

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
  void scheduleCmdVelCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void freeNavigationPathCallback(nav_msgs::msg::Path::ConstSharedPtr msg);
  void controlLoopCallback();
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
  void process(
    const Velocity & cmd_vel_in, bool publish_cmd_vel = true,
    bool idle_polygon_only = false, bool command_updated = true);

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

  /** @brief Cache the latest graph tracking path published by schedule_planner. */
  void trackingPathCallback(nav_interfaces::msg::TrackingPath::ConstSharedPtr msg);

  /**
   * @brief Check whether enough obstacle points lie in the path corridor ahead of the robot.
   * @param collision_points Obstacle points in base_frame_id_.
   * @return true when the configured path-stop threshold is exceeded.
   */
  bool hasObstacleOnPath(const std::vector<Point> & collision_points) const;

  /** @brief Transform a tracking path into base_frame_id_ using the latest cached TF. */
  bool transformTrackingPathToBase(
    const nav_interfaces::msg::TrackingPath & path,
    std::vector<Point> & points) const;

  /** @brief Build corridor sample points from the closest path projection forward. */
  std::vector<Point> makePathCheckPoints(const std::vector<Point> & path_points) const;

  /** @brief Check whether obstacle hits in the sampled path corridor exceed the threshold. */
  bool hasObstacleInPathCorridor(
    const std::vector<Point> & collision_points,
    const std::vector<Point> & check_points) const;

  /** @brief Publish path-stop sample circles for RViz. */
  void publishPathStopMarkers(const std::vector<Point> & check_points) const;

  /** @brief Accumulate and periodically report path-stop execution timing. */
  void recordPathStopTiming(const std::chrono::steady_clock::time_point & start_time);

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

  void setLoadedFootprintCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  geometry_msgs::msg::Polygon readFootprintParameter(
    const std::string & name, const std::vector<double> & defaults);
  void applyInternalFootprint(bool loaded);
  void applyNavigationPadding(NavigationMode mode);
  void enterIdle(const char * reason);

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
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr schedule_cmd_vel_sub_;
  rclcpp::Subscription<nav_interfaces::msg::TrackingPath>::SharedPtr tracking_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr free_navigation_path_sub_;
  /// @brief Output cmd_vel publisher
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_out_pub_;
  /// @brief Real-time obstacle monitor status publisher (FREE/SLOWDOWN/STOP)
  rclcpp_lifecycle::LifecyclePublisher<
    nav_interfaces::msg::ObstacleMonitorStatus>::SharedPtr obstacle_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
    visualization_msgs::msg::MarkerArray>::SharedPtr path_stop_marker_pub_;
  /// @brief 动态膨胀设置服务（本仓库相对上游 Nav2 的改动）
  rclcpp::Service<nav_interfaces::srv::SetPolygonPadding>::SharedPtr set_padding_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_loaded_footprint_srv_;

  std::vector<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Polygon>::SharedPtr>
    costmap_footprint_pubs_;

  // schedule_planner path-ahead stop detector
  bool path_stop_enabled_{false};
  std::string tracking_path_topic_{"tracking_path"};
  double path_stop_distance_{1.0};
  double path_stop_radius_{0.36};
  double path_point_interval_{0.10};
  int path_stop_max_points_{3};
  bool path_stop_visualize_{false};
  bool path_stop_timing_enabled_{false};
  std::string path_stop_marker_topic_{"path_stop_circles"};
  mutable std::mutex tracking_path_mutex_;
  std::mutex path_stop_timing_mutex_;
  std::chrono::steady_clock::time_point path_stop_timing_window_start_;
  std::uint64_t path_stop_timing_count_{0};
  std::int64_t path_stop_timing_total_ns_{0};
  std::int64_t path_stop_timing_max_ns_{0};
  mutable rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};
  nav_interfaces::msg::TrackingPath::ConstSharedPtr tracking_path_;

  struct VelocityMailbox
  {
    geometry_msgs::msg::Twist::ConstSharedPtr message;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    std::uint64_t sequence{0};
  };

  mutable std::mutex mailbox_mutex_;
  VelocityMailbox free_cmd_mailbox_;
  VelocityMailbox schedule_cmd_mailbox_;
  nav_msgs::msg::Path::ConstSharedPtr pending_free_path_;
  nav_interfaces::msg::TrackingPath::ConstSharedPtr pending_tracking_path_;
  std::uint64_t free_path_sequence_{0};
  std::uint64_t tracking_path_sequence_{0};
  std::uint64_t consumed_free_path_sequence_{0};
  std::uint64_t consumed_tracking_path_sequence_{0};
  std::uint64_t consumed_free_cmd_sequence_{0};
  std::uint64_t consumed_schedule_cmd_sequence_{0};
  std::optional<bool> pending_loaded_footprint_;

  rclcpp::TimerBase::SharedPtr control_loop_timer_;
  rclcpp::CallbackGroup::SharedPtr io_callback_group_;
  rclcpp::CallbackGroup::SharedPtr control_callback_group_;
  double control_frequency_{100.0};
  rclcpp::Duration cmd_vel_timeout_{0, 300000000};
  rclcpp::Duration idle_grace_period_{5, 0};
  NavigationMode navigation_mode_{NavigationMode::IDLE};
  NavigationType lifecycle_navigation_type_{NavigationType::UNSET};
  rclcpp::Time mode_last_cmd_stamp_{0, 0, RCL_ROS_TIME};
  std::string schedule_cmd_vel_topic_{"cmd_vel_nav"};
  std::string free_navigation_path_topic_{"plan"};
  double automonous_navigation_padding_{0.1};
  bool approach_enabled_{true};
  geometry_msgs::msg::Polygon normal_footprint_;
  geometry_msgs::msg::Polygon loaded_footprint_;
  bool loaded_mode_{false};

  /// @brief Whether main routine is active
  bool process_active_;

  /// @brief Robot base frame ID, used as obstacle_status.header.frame_id
  std::string base_frame_id_;
  tf2::Duration transform_tolerance_{tf2::durationFromSec(0.1)};
  /// @brief Previous robot action
  Action robot_action_prev_;
  /// @brief Latest timestamp when robot has 0-velocity
  rclcpp::Time stop_stamp_;
  /// @brief Timeout after which 0-velocity ceases to be published
  rclcpp::Duration stop_pub_timeout_;

};  // class CollisionMonitor

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__COLLISION_MONITOR_NODE_HPP_
