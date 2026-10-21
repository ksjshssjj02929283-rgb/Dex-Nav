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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/empty.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
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
  void freeNavigationPathCallback(nav_msgs::msg::Path::ConstSharedPtr msg);
  void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg);

  /// 速度回调写入的最新消息、接收时间与序号。
  struct VelocityMailbox
  {
    geometry_msgs::msg::Twist::ConstSharedPtr message;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    std::uint64_t sequence{0};
  };

  /// 单次控制周期的输入快照；new_* 表示尚未消费的路径更新。
  struct ControlInput
  {
    VelocityMailbox cmd;
    nav_msgs::msg::Path::ConstSharedPtr free_path;
    nav_interfaces::msg::TrackingPath::ConstSharedPtr tracking_path;
    bool new_free_path{false};
    bool new_tracking_path{false};
  };

  /// 串行执行单次控制周期；scan 安全检查先于导航和速度处理。
  void controlLoopCallback();
  /// 在邮箱锁内更新缓存并消费路径序号；返回引用仅供当前串行控制周期使用。
  const ControlInput & readControlInput();
  /// 更新已观察的序号；路径、速度或传感器数据变化时返回 true。
  bool hasUpdatedInput(const ControlInput & input);
  /// 传感器（激光或超声波等）缺失或超时时发布 STOP；仅在本周期可继续处理时返回 true。
  bool checkSensorTimeout(const rclcpp::Time & now);
  /// 根据配置的导航类型执行路径检查与 IDLE 恢复检查。
  void updateNavigationMode(const ControlInput & input, const rclcpp::Time & now);
  /// 处理速度新鲜度、IDLE 超时，以及活动或空闲状态下的碰撞检测。
  void processVelocityInput(
    const VelocityMailbox & cmd, bool input_updated, const rclcpp::Time & now);
  /// 接收新的自由导航路径，或由新鲜速度从 IDLE 恢复自动导航。
  void checkAutonomousNavigation(
    const nav_msgs::msg::Path::ConstSharedPtr & path, bool new_path,
    bool cmd_fresh, const rclcpp::Time & cmd_stamp, const rclcpp::Time & now);
  /// 根据路径 padding 选择普通/贴边调度，或从 IDLE 恢复上次的调度子模式。
  void checkScheduleNavigation(
    const nav_interfaces::msg::TrackingPath::ConstSharedPtr & path, bool new_path,
    bool cmd_fresh, const rclcpp::Time & cmd_stamp, const rclcpp::Time & now);
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
    const Action & robot_action,
    const std::shared_ptr<Polygon> action_polygon,
    const std::string & override_zone_name = "") const;

  /**
   * @brief Publishes ultrasonic points as PointCloud2 if there are active subscribers.
   * Does nothing and allocates nothing if subscription count is 0.
   * @param curr_time Current stamp for PointCloud2 header
   * @param points Ultrasonic points in base frame
   */
  void publishUltrasonicPointCloud(
    const rclcpp::Time & curr_time,
    const std::vector<Point> & points) const;

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
    const std::vector<const std::vector<Point> *> & point_buffers,
    const Velocity & velocity,
    Action & robot_action) const;

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

  /** @brief Record one performance duration sample in milliseconds. */
  void recordPerformanceDuration(
    std::vector<double> & samples,
    const std::chrono::steady_clock::time_point & start_time) const;

  /** @brief Record callback period/execution time and emit a complete statistics window. */
  void recordControlLoopTiming(const std::chrono::steady_clock::time_point & start_time);

  /** @brief Log P50/P95/P99/max and deadline misses for one metric. */
  void reportPerformanceMetric(
    const char * metric, const std::vector<double> & samples,
    double window_seconds) const;

  /** @brief Clear all samples at lifecycle activation or after a report. */
  void resetPerformanceTiming();

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

  void applyNavigationPadding(NavigationMode mode);
  void enterIdle(const char * reason);
  /// 进入 STOP 的上升沿通知 velocity_smoother 立即清零；须在更新 robot_action_prev_ 之前调用。
  void publishHardStopOnStopEdge(const char * reason);
  /// 只在转发时（非 IDLE 且有新鲜速度）发布一帧零速，避免空闲时与外部对接指令交替。
  bool publishZeroIfForwarding();
  static const char * navigationModeName(NavigationMode mode);
  void changeNavigationMode(NavigationMode mode, const char * reason);
  bool updateSpeedTier(const rclcpp::Time & now);
  static std::int64_t steadyNowNs()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

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
  rclcpp::Subscription<nav_interfaces::msg::TrackingPath>::SharedPtr tracking_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr free_navigation_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  /// @brief Output cmd_vel publisher
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_out_pub_;
  /// @brief Real-time obstacle monitor status publisher (FREE/SLOWDOWN/STOP)
  rclcpp_lifecycle::LifecyclePublisher<
    nav_interfaces::msg::ObstacleMonitorStatus>::SharedPtr obstacle_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
    visualization_msgs::msg::MarkerArray>::SharedPtr path_stop_marker_pub_;
  /// @brief velocity_smoother 的硬停输入，CM 进入 STOP 时发布一次
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>::SharedPtr hard_stop_pub_;
  /// @brief 超声波点云发布者（仅在有订阅时发布）
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    ultrasonic_points_pub_;
  /// @brief 动态膨胀设置服务（本仓库相对上游 Nav2 的改动），仅 schedule 模式创建
  rclcpp::Service<nav_interfaces::srv::SetPolygonPadding>::SharedPtr set_padding_srv_;

  // schedule_planner path-ahead stop detector
  bool path_stop_enabled_{false};
  std::string tracking_path_topic_{"tracking_path"};
  std::string ultrasonic_points_topic_{"ultrasonic_points"};
  bool enable_range_slowdown_{false};
  double path_stop_distance_{1.0};
  double path_stop_radius_{0.36};
  double path_point_interval_{0.10};
  int path_stop_max_points_{3};
  bool path_stop_visualize_{false};
  double path_stop_marker_pub_rate_{5.0};
  double path_stop_marker_min_interval_{0.2};
  mutable rclcpp::Time last_path_stop_marker_pub_time_{0, 0, RCL_ROS_TIME};
  bool path_stop_timing_enabled_{false};
  std::string path_stop_marker_topic_{"path_stop_circles"};
  mutable std::mutex tracking_path_mutex_;
  std::mutex path_stop_timing_mutex_;
  std::chrono::steady_clock::time_point path_stop_timing_window_start_;
  std::uint64_t path_stop_timing_count_{0};
  std::int64_t path_stop_timing_total_ns_{0};
  std::int64_t path_stop_timing_max_ns_{0};
  bool performance_timing_enabled_{false};
  double performance_report_interval_{60.0};
  std::chrono::steady_clock::time_point performance_window_start_;
  std::chrono::steady_clock::time_point last_control_loop_start_;
  bool have_last_control_loop_start_{false};
  mutable std::vector<double> control_period_samples_ms_;
  mutable std::vector<double> control_execution_samples_ms_;
  mutable std::vector<double> process_samples_ms_;
  mutable std::vector<double> approach_samples_ms_;
  mutable std::vector<double> path_corridor_samples_ms_;
  mutable rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};
  nav_interfaces::msg::TrackingPath::ConstSharedPtr tracking_path_;

  mutable std::mutex mailbox_mutex_;
  VelocityMailbox cmd_mailbox_;
  ControlInput control_input_;
  nav_msgs::msg::Path::ConstSharedPtr pending_free_path_;
  nav_interfaces::msg::TrackingPath::ConstSharedPtr pending_tracking_path_;
  std::uint64_t free_path_sequence_{0};
  std::uint64_t tracking_path_sequence_{0};
  std::uint64_t consumed_free_path_sequence_{0};
  std::uint64_t consumed_tracking_path_sequence_{0};
  std::uint64_t consumed_cmd_sequence_{0};
  std::uint64_t observed_cmd_sequence_{0};
  std::vector<std::uint64_t> observed_source_sequences_;

  rclcpp::TimerBase::SharedPtr control_loop_timer_;
  rclcpp::CallbackGroup::SharedPtr io_callback_group_;
  rclcpp::CallbackGroup::SharedPtr control_callback_group_;
  double control_frequency_{100.0};
  rclcpp::Duration cmd_vel_timeout_{0, 300000000};
  rclcpp::Duration idle_grace_period_{5, 0};
  NavigationMode navigation_mode_{NavigationMode::IDLE};
  // Last sub-mode selected by TrackingPath.padding.  IDLE is a transient safety state
  // (velocity timeout, footprint switch, etc.), so it must not erase whether the active
  // schedule path is normal navigation or edging.
  NavigationMode schedule_navigation_mode_{NavigationMode::SCHEDULE_NAVIGATION};
  NavigationType configured_navigation_type_{NavigationType::AUTOMONOUS};
  std::int64_t source_start_time_ns_{0};
  std::int64_t sensor_fault_timeout_ns_{500000000LL};
  bool sensor_fault_active_{false};
  int logged_velocity_state_{-1};
  rclcpp::Time mode_last_cmd_stamp_{0, 0, RCL_ROS_TIME};
  std::string free_navigation_path_topic_{"plan"};
  double automonous_navigation_padding_{0.1};
  bool approach_enabled_{true};

  // ── 自动导航模式停止区前向/后向速度分档自适应参数（A8 规范及倒车扩充）──
  std::string odom_topic_{"feedback/odom"};
  double speed_source_timeout_{0.2};            // 里程计更新超时阈值（秒），超时则强制降级至最高档
  std::int64_t speed_source_timeout_ns_{200000000LL};
  double tier_down_delay_{0.5};                 // 降档防抖延迟（秒），目标低速档需稳定持续该时长才允许降档
  std::int64_t tier_down_delay_ns_{500000000LL};
  std::vector<double> speed_tiers_;             // 各档位速度上限门限数组（严格递增，单位 m/s）
  std::vector<double> front_padding_tiers_;      // 各档位对应的前向额外外扩距离（单位 m）
  std::vector<double> rear_padding_tiers_;       // 各档位对应的后向额外外扩距离（单位 m，未显式配置时复用 front_padding_tiers_）
  bool speed_tiers_enabled_{false};             // 是否启用分档功能（两数组非空时启用）
  std::atomic<double> odom_linear_x_{0.0};       // 底盘当前前向线速度（vx）
  std::atomic<std::int64_t> odom_rx_steady_time_ns_{0}; // 里程计最新接收时间戳（单调时钟，纳秒）
  std::size_t current_speed_tier_{0};           // 当前生效的速度档位索引
  bool current_is_reverse_{false};              // 当前生效的运动方向（true: 倒车后退, false: 前进/静止）
  int pending_speed_tier_{-1};                  // 降档等待确认中的目标档位（-1 表示无等待）
  std::int64_t tier_down_start_steady_ns_{0};   // 降档计时起始时间戳（单调时钟）

  // ── 多源点云预分配缓冲池（在 100Hz 控制循环中复用，彻底避免频繁动态内存分配）──
  std::vector<Point> raw_points_buffer_;        // 激光雷达与 RGBD 点云（用于通用避障与 APPROACH 轨迹预测）
  std::vector<Point> range_slow_buffer_;        // 允许进入 SLOWDOWN 减速区的超声波探头点
  std::vector<Point> range_stop_only_buffer_;   // 仅用于 STOP 停障的超声波探头点
  std::vector<Point> ultrasonic_points_buffer_; // 汇聚的所有超声波点（用于发布 PointCloud2 调试话题）
  std::vector<Point> slowdown_points_buffer_;   // 减速多边形最终消费点池（雷达点 + 允许减速的超声波点）
  std::vector<Point> all_stop_points_buffer_;   // 停障多边形最终消费点池（雷达点 + 所有超声波点）

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

  // ── 停止保持与速度门控（A1、A6、A7 规范）──
  double clear_hold_time_{2.0};                 // 停止条件解除后的防抖保持时间（秒，默认 2.0s）
  std::int64_t clear_hold_time_ns_{2000000000LL};
  std::int64_t last_stop_condition_steady_ns_{0}; // 停止条件最后一次满足的时刻（单调时钟）
  std::string stop_hold_zone_name_{""};         // 保持期沿用的触发区域名
  bool is_forwarding_{false};                   // 当前是否处于活跃转发速度状态（非 IDLE 且输入速度新鲜）

  // ── 减速防抖保持（Slowdown Hold Time）──
  double slowdown_hold_time_{1.0};              // 减速条件满足后的防抖保持时间（秒，默认 1.0s）
  std::int64_t slowdown_hold_time_ns_{1000000000LL};
  std::int64_t last_slowdown_condition_steady_ns_{0}; // 减速条件最后一次满足的时刻（单调时钟）
  double slowdown_hold_ratio_{0.5};
  std::shared_ptr<Polygon> slowdown_hold_polygon_{nullptr};
  bool zero_vel_published_this_cycle_{false};   // 防重发标志：当前控制周期内是否已经发布过零速
  std::string last_sensor_fault_source_{""};    // 最近一次报错的传感器失效源
  std::string last_sensor_fault_reason_{""};    // 最近一次报错的传感器失效明细原因
};  // class CollisionMonitor

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__COLLISION_MONITOR_NODE_HPP_
