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

#include "nav2_collision_monitor/collision_monitor_node.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <exception>
#include <string>
#include <utility>
#include <functional>
#include <limits>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"

#include "nav2_util/node_utils.hpp"

#include "nav2_collision_monitor/kinematics.hpp"

namespace nav2_collision_monitor
{

using StatusMsg = nav_interfaces::msg::ObstacleMonitorStatus;

CollisionMonitor::CollisionMonitor(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("collision_monitor", "", options),
  process_active_(false), robot_action_prev_{DO_NOTHING, {-1.0, -1.0, -1.0}},
  stop_stamp_{0, 0, get_clock()->get_clock_type()}, stop_pub_timeout_(1.0, 0.0)
{
}

CollisionMonitor::~CollisionMonitor()
{
  polygons_.clear();
  sources_.clear();
}

nav2_util::CallbackReturn
CollisionMonitor::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Transform buffer and listener initialization
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  std::string cmd_vel_in_topic;
  std::string cmd_vel_out_topic;

  // Obtaining ROS parameters
  if (!getParameters(cmd_vel_in_topic, cmd_vel_out_topic)) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  auto node = shared_from_this();
  nav2_util::declare_parameter_if_not_declared(
    node, "obstacle_status_topic", rclcpp::ParameterValue("obstacle_status"));
  const std::string obstacle_status_topic = get_parameter("obstacle_status_topic").as_string();
  // 与 controller_server 共用 velocity_smoother 的硬停输入；空字符串表示不发布。
  nav2_util::declare_parameter_if_not_declared(
    node, "hard_stop_topic", rclcpp::ParameterValue("cmd_vel_hard_stop"));
  const std::string hard_stop_topic = get_parameter("hard_stop_topic").as_string();

  nav2_util::declare_parameter_if_not_declared(
    node, "control_frequency", rclcpp::ParameterValue(100.0));
  control_frequency_ = get_parameter("control_frequency").as_double();
  nav2_util::declare_parameter_if_not_declared(
    node, "cmd_vel_timeout", rclcpp::ParameterValue(0.3));
  cmd_vel_timeout_ = rclcpp::Duration::from_seconds(get_parameter("cmd_vel_timeout").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, "idle_grace_period", rclcpp::ParameterValue(5.0));
  idle_grace_period_ =
    rclcpp::Duration::from_seconds(get_parameter("idle_grace_period").as_double());
  if (control_frequency_ <= 0.0 || cmd_vel_timeout_.seconds() <= 0.0 ||
    idle_grace_period_.seconds() < cmd_vel_timeout_.seconds())
  {
    RCLCPP_ERROR(get_logger(), "Invalid control frequency or command timeout configuration");
    return nav2_util::CallbackReturn::FAILURE;
  }

  nav2_util::declare_parameter_if_not_declared(
    node, "navigation_type", rclcpp::ParameterValue("auto"));
  const auto navigation_type = get_parameter("navigation_type").as_string();
  if (navigation_type == "auto") {
    configured_navigation_type_ = NavigationType::AUTOMONOUS;
  } else if (navigation_type == "schedule") {
    configured_navigation_type_ = NavigationType::SCHEDULE;
  } else {
    RCLCPP_ERROR(get_logger(), "navigation_type must be 'auto' or 'schedule'");
    return nav2_util::CallbackReturn::FAILURE;
  }

  // 模式由参数固定：auto 链路本节点串联在速度链中并输出安全速度；schedule 链路
  // path_follower 直连 velocity_smoother，本节点仅旁路观察速度做障碍检测，不下发速度。
  // 两种模式共用 cmd_vel_in_topic 作为速度输入。
  nav2_util::declare_parameter_if_not_declared(
    node, "free_navigation_path_topic", rclcpp::ParameterValue("plan"));
  free_navigation_path_topic_ = get_parameter("free_navigation_path_topic").as_string();
  nav2_util::declare_parameter_if_not_declared(
    node, "PolygonStop.automonous_navigation_padding", rclcpp::ParameterValue(0.1));
  automonous_navigation_padding_ =
    get_parameter("PolygonStop.automonous_navigation_padding").as_double();
  nav2_util::declare_parameter_if_not_declared(
    node, "approach_enabled", rclcpp::ParameterValue(true));
  approach_enabled_ = get_parameter("approach_enabled").as_bool();
  nav2_util::declare_parameter_if_not_declared(
    node, "performance_timing_enabled", rclcpp::ParameterValue(false));
  performance_timing_enabled_ = get_parameter("performance_timing_enabled").as_bool();
  nav2_util::declare_parameter_if_not_declared(
    node, "performance_report_interval", rclcpp::ParameterValue(60.0));
  performance_report_interval_ = get_parameter("performance_report_interval").as_double();
  if (performance_report_interval_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "performance_report_interval must be greater than zero");
    return nav2_util::CallbackReturn::FAILURE;
  }

  // IO 组允许速度、路径和服务回调并行接收；它们只写 mailbox。控制组中只有
  // 100 Hz timer，可以保证模式、路径、Polygon 和速度输出只有一个写入者。
  io_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  control_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions io_options;
  io_options.callback_group = io_callback_group_;

  // 只有当速度输入话题与本节点输出话题同名时才需要屏蔽本进程发布的消息：此时若不做处理，
  // cmd_vel_in_sub_ 会订阅到自己发布的 cmd_vel（IDLE enterIdle 的零速、传感器超时零速或
  // auto 安全速度），被当作新指令回灌，形成自激/自锁。ignore_local_publications 只屏蔽
  // 本进程发出的消息，path_follower 等同名话题的其它发布者不受影响；输入与输出不同名时
  // 保持默认行为，避免误伤同进程的其它速度发布者。
  rclcpp::SubscriptionOptions cmd_vel_in_options = io_options;
  if (cmd_vel_in_topic == cmd_vel_out_topic) {
    cmd_vel_in_options.ignore_local_publications = true;
    RCLCPP_WARN(
      get_logger(),
      "cmd_vel_in_topic == cmd_vel_out_topic ('%s')；已启用 ignore_local_publications，"
      "本节点不会消费自己发布的 cmd_vel", cmd_vel_in_topic.c_str());
  }
  cmd_vel_in_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_in_topic, 1,
    std::bind(&CollisionMonitor::cmdVelInCallback, this, std::placeholders::_1),
    cmd_vel_in_options);

  // 每种模式只订阅自己的路径输入。
  if (configured_navigation_type_ == NavigationType::SCHEDULE) {
    tracking_path_sub_ = this->create_subscription<nav_interfaces::msg::TrackingPath>(
      tracking_path_topic_, rclcpp::QoS(1),
      std::bind(&CollisionMonitor::trackingPathCallback, this, std::placeholders::_1),
      io_options);
  } else {
    free_navigation_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      free_navigation_path_topic_, rclcpp::QoS(1),
      std::bind(&CollisionMonitor::freeNavigationPathCallback, this, std::placeholders::_1),
      io_options);
  }
  cmd_vel_out_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    cmd_vel_out_topic, 1);
  obstacle_status_pub_ = this->create_publisher<nav_interfaces::msg::ObstacleMonitorStatus>(
    obstacle_status_topic, 1);
  if (!hard_stop_topic.empty()) {
    hard_stop_pub_ = this->create_publisher<std_msgs::msg::Empty>(
      hard_stop_topic, rclcpp::QoS(1).reliable());
  }

  nav2_util::declare_parameter_if_not_declared(
    node, "ultrasonic_points_topic", rclcpp::ParameterValue("ultrasonic_points"));
  ultrasonic_points_topic_ = get_parameter("ultrasonic_points_topic").as_string();
  if (!ultrasonic_points_topic_.empty()) {
    ultrasonic_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      ultrasonic_points_topic_, 1);
  }

  nav2_util::declare_parameter_if_not_declared(
    node, "enable_range_slowdown", rclcpp::ParameterValue(false));
  enable_range_slowdown_ = get_parameter("enable_range_slowdown").as_bool();

  nav2_util::declare_parameter_if_not_declared(
    node, "PolygonStop.path_stop_enabled", rclcpp::ParameterValue(false));
  // 路径走廊只用于调度贴边模式。
  path_stop_enabled_ = get_parameter("PolygonStop.path_stop_enabled").as_bool() &&
    configured_navigation_type_ == NavigationType::SCHEDULE;
  nav2_util::declare_parameter_if_not_declared(
    node, "PolygonStop.tracking_path_topic", rclcpp::ParameterValue("tracking_path"));
  tracking_path_topic_ = get_parameter("PolygonStop.tracking_path_topic").as_string();
  nav2_util::declare_parameter_if_not_declared(
    node, "PolygonStop.path_stop_distance", rclcpp::ParameterValue(1.0));
  path_stop_distance_ = get_parameter("PolygonStop.path_stop_distance").as_double();
  nav2_util::declare_parameter_if_not_declared(
    node, "PolygonStop.path_stop_radius", rclcpp::ParameterValue(0.36));
  path_stop_radius_ = get_parameter("PolygonStop.path_stop_radius").as_double();
  nav2_util::declare_parameter_if_not_declared(
    node, "PolygonStop.path_point_interval", rclcpp::ParameterValue(0.10));
  path_point_interval_ = get_parameter("PolygonStop.path_point_interval").as_double();
  nav2_util::declare_parameter_if_not_declared(
    node, "PolygonStop.max_points", rclcpp::ParameterValue(3));
  path_stop_max_points_ = get_parameter("PolygonStop.max_points").as_int();
  nav2_util::declare_parameter_if_not_declared(
    node, "PolygonStop.path_stop_visualize", rclcpp::ParameterValue(false));
  path_stop_visualize_ = get_parameter("PolygonStop.path_stop_visualize").as_bool();
  nav2_util::declare_parameter_if_not_declared(
    node, "PolygonStop.path_stop_marker_pub_rate", rclcpp::ParameterValue(5.0));
  path_stop_marker_pub_rate_ =
    get_parameter("PolygonStop.path_stop_marker_pub_rate").as_double();
  path_stop_marker_min_interval_ =
    (path_stop_marker_pub_rate_ > 0.0) ? (1.0 / path_stop_marker_pub_rate_) : 0.0;
  nav2_util::declare_parameter_if_not_declared(
    node, "PolygonStop.path_stop_timing_enabled", rclcpp::ParameterValue(false));
  path_stop_timing_enabled_ =
    get_parameter("PolygonStop.path_stop_timing_enabled").as_bool();
  nav2_util::declare_parameter_if_not_declared(
    node, "PolygonStop.path_stop_marker_topic",
    rclcpp::ParameterValue("path_stop_circles"));
  path_stop_marker_topic_ = get_parameter("PolygonStop.path_stop_marker_topic").as_string();
  if (path_stop_enabled_) {
    if (path_stop_distance_ <= 0.0 || path_stop_radius_ <= 0.0 ||
      path_point_interval_ <= 0.0 || path_stop_max_points_ < 0)
    {
      RCLCPP_ERROR(get_logger(), "Invalid path stop detector parameters");
      return nav2_util::CallbackReturn::FAILURE;
    }
    if (path_stop_visualize_) {
      path_stop_marker_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
        path_stop_marker_topic_, rclcpp::QoS(1));
    }
  }

  // 动态膨胀设置服务（本仓库相对上游 Nav2 的改动）。auto 模式的停止区固定，
  // 不提供运行时修改入口。
  if (configured_navigation_type_ == NavigationType::SCHEDULE) {
    set_padding_srv_ = this->create_service<nav_interfaces::srv::SetPolygonPadding>(
      "~/set_polygon_padding",
      std::bind(
        &CollisionMonitor::setPolygonPaddingCallback, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, io_callback_group_);
  }

  // Speed tiers for autonomous navigation (A8)
  nav2_util::declare_parameter_if_not_declared(
    node, "odom_topic", rclcpp::ParameterValue("feedback/odom"));
  odom_topic_ = get_parameter("odom_topic").as_string();

  nav2_util::declare_parameter_if_not_declared(
    node, "speed_source_timeout", rclcpp::ParameterValue(0.2));
  speed_source_timeout_ = get_parameter("speed_source_timeout").as_double();

  nav2_util::declare_parameter_if_not_declared(
    node, "tier_down_delay", rclcpp::ParameterValue(0.5));
  tier_down_delay_ = get_parameter("tier_down_delay").as_double();

  nav2_util::declare_parameter_if_not_declared(
    node, "PolygonStop.speed_tiers", rclcpp::ParameterValue(std::vector<double>()));
  speed_tiers_ = get_parameter("PolygonStop.speed_tiers").as_double_array();

  nav2_util::declare_parameter_if_not_declared(
    node, "PolygonStop.front_padding_tiers", rclcpp::ParameterValue(std::vector<double>()));
  front_padding_tiers_ = get_parameter("PolygonStop.front_padding_tiers").as_double_array();

  nav2_util::declare_parameter_if_not_declared(
    node, "PolygonStop.rear_padding_tiers", rclcpp::ParameterValue(std::vector<double>()));
  rear_padding_tiers_ = get_parameter("PolygonStop.rear_padding_tiers").as_double_array();
  if (rear_padding_tiers_.empty() && !front_padding_tiers_.empty()) {
    rear_padding_tiers_ = front_padding_tiers_;
  }

  if (speed_tiers_.empty() && front_padding_tiers_.empty()) {
    speed_tiers_enabled_ = false;
  } else {
    speed_tiers_enabled_ = true;
    if (speed_tiers_.size() != front_padding_tiers_.size() || speed_tiers_.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "speed_tiers and front_padding_tiers must have the same non-zero size");
      return nav2_util::CallbackReturn::FAILURE;
    }
    if (rear_padding_tiers_.size() != speed_tiers_.size()) {
      RCLCPP_ERROR(
        get_logger(),
        "rear_padding_tiers and speed_tiers must have the same size");
      return nav2_util::CallbackReturn::FAILURE;
    }
    if (odom_topic_.empty()) {
      RCLCPP_ERROR(get_logger(), "odom_topic must not be empty when speed tiers are configured");
      return nav2_util::CallbackReturn::FAILURE;
    }
    if (!std::isfinite(speed_source_timeout_) || speed_source_timeout_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "speed_source_timeout must be positive and finite");
      return nav2_util::CallbackReturn::FAILURE;
    }
    if (!std::isfinite(tier_down_delay_) || tier_down_delay_ < 0.0) {
      RCLCPP_ERROR(get_logger(), "tier_down_delay must be non-negative and finite");
      return nav2_util::CallbackReturn::FAILURE;
    }
    for (std::size_t i = 0; i < speed_tiers_.size(); ++i) {
      if (!std::isfinite(speed_tiers_[i]) || speed_tiers_[i] <= 0.0) {
        RCLCPP_ERROR(
          get_logger(), "speed_tiers[%zu] (%.3f) must be positive and finite",
          i, speed_tiers_[i]);
        return nav2_util::CallbackReturn::FAILURE;
      }
      if (i > 0 && speed_tiers_[i] <= speed_tiers_[i - 1]) {
        RCLCPP_ERROR(get_logger(), "speed_tiers must be strictly increasing");
        return nav2_util::CallbackReturn::FAILURE;
      }
      if (!std::isfinite(front_padding_tiers_[i]) || front_padding_tiers_[i] < 0.0) {
        RCLCPP_ERROR(
          get_logger(), "front_padding_tiers[%zu] (%.3f) must be non-negative and finite",
          i, front_padding_tiers_[i]);
        return nav2_util::CallbackReturn::FAILURE;
      }
      if (i > 0 && front_padding_tiers_[i] < front_padding_tiers_[i - 1]) {
        RCLCPP_ERROR(get_logger(), "front_padding_tiers must be non-decreasing");
        return nav2_util::CallbackReturn::FAILURE;
      }
      if (!std::isfinite(rear_padding_tiers_[i]) || rear_padding_tiers_[i] < 0.0) {
        RCLCPP_ERROR(
          get_logger(), "rear_padding_tiers[%zu] (%.3f) must be non-negative and finite",
          i, rear_padding_tiers_[i]);
        return nav2_util::CallbackReturn::FAILURE;
      }
      if (i > 0 && rear_padding_tiers_[i] < rear_padding_tiers_[i - 1]) {
        RCLCPP_ERROR(get_logger(), "rear_padding_tiers must be non-decreasing");
        return nav2_util::CallbackReturn::FAILURE;
      }
    }
    speed_source_timeout_ns_ = static_cast<std::int64_t>(speed_source_timeout_ * 1e9);
    tier_down_delay_ns_ = static_cast<std::int64_t>(tier_down_delay_ * 1e9);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CollisionMonitor::odomCallback, this, std::placeholders::_1),
      io_options);
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionMonitor::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Activating lifecycle publishers
  cmd_vel_out_pub_->on_activate();
  // A5: Publish one 0-velocity frame on activation in auto mode
  if (configured_navigation_type_ == NavigationType::AUTOMONOUS) {
    cmd_vel_out_pub_->publish(std::make_unique<geometry_msgs::msg::Twist>());
  }
  obstacle_status_pub_->on_activate();
  if (hard_stop_pub_) {
    hard_stop_pub_->on_activate();
  }
  if (path_stop_marker_pub_) {
    path_stop_marker_pub_->on_activate();
  }
  if (ultrasonic_points_pub_) {
    ultrasonic_points_pub_->on_activate();
  }

  // Activating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    polygon->activate();
  }

  // Since polygons are being published when cmd_vel_in appears,
  // we need to publish polygons first time to display them at startup
  publishPolygons();

  // Activating main worker
  process_active_ = true;
  {
    std::lock_guard<std::mutex> lock(path_stop_timing_mutex_);
    path_stop_timing_count_ = 0;
    path_stop_timing_total_ns_ = 0;
    path_stop_timing_max_ns_ = 0;
    path_stop_timing_window_start_ = std::chrono::steady_clock::now();
  }
  resetPerformanceTiming();

  // 每次 lifecycle activate 都从安全的 IDLE 起步，并清除上个生命周期的输入。
  // 导航类型由 navigation_type 参数固定，只处理该类型的路径输入。
  navigation_mode_ = NavigationMode::IDLE;
  schedule_navigation_mode_ = NavigationMode::SCHEDULE_NAVIGATION;
  current_speed_tier_ = 0;
  current_is_reverse_ = false;
  pending_speed_tier_ = -1;
  tier_down_start_steady_ns_ = 0;
  last_stop_condition_steady_ns_ = 0;
  stop_hold_zone_name_.clear();
  sensor_fault_active_ = false;
  last_sensor_fault_source_.clear();
  last_sensor_fault_reason_.clear();
  is_forwarding_ = false;
  zero_vel_published_this_cycle_ = false;
  odom_linear_x_.store(0.0, std::memory_order_relaxed);
  odom_rx_steady_time_ns_.store(0, std::memory_order_relaxed);
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    if (polygon->getName() == "PolygonStop") {
      polygon->setFrontPadding(0.0);
      polygon->setRearPadding(0.0);
    } else {
      polygon->setFrontPadding(polygon->getDefaultFrontPadding());
      polygon->setRearPadding(polygon->getDefaultRearPadding());
    }
  }
  RCLCPP_INFO(
    get_logger(), "[FSM] activate: mode=IDLE type=%s cmd_timeout=%.3fs idle_timeout=%.3fs",
    configured_navigation_type_ == NavigationType::AUTOMONOUS ? "auto" : "schedule",
    cmd_vel_timeout_.seconds(), idle_grace_period_.seconds());
  mode_last_cmd_stamp_ = this->now();
  {
    std::lock_guard<std::mutex> lock(mailbox_mutex_);
    cmd_mailbox_ = VelocityMailbox{};
    control_input_ = ControlInput{};
    pending_free_path_.reset();
    pending_tracking_path_.reset();
    free_path_sequence_ = 0;
    tracking_path_sequence_ = 0;
    consumed_free_path_sequence_ = 0;
    consumed_tracking_path_sequence_ = 0;
    consumed_cmd_sequence_ = 0;
    observed_cmd_sequence_ = 0;
  }
  observed_source_sequences_.clear();
  observed_source_sequences_.reserve(sources_.size());
  for (const auto & source : sources_) {
    observed_source_sequences_.push_back(source->getDataSequence());
  }
  source_start_time_ns_ = steadyNowNs();
  const auto period = std::chrono::duration<double>(1.0 / control_frequency_);
  control_loop_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&CollisionMonitor::controlLoopCallback, this), control_callback_group_);
  RCLCPP_INFO(get_logger(), "Control loop started at %.2f Hz in IDLE", control_frequency_);

  // Creating bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionMonitor::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  control_loop_timer_.reset();

  // Deactivating main worker
  process_active_ = false;

  // Reset action type to default after worker deactivating
  robot_action_prev_ = {DO_NOTHING, {-1.0, -1.0, -1.0}};

  current_speed_tier_ = 0;
  pending_speed_tier_ = -1;
  tier_down_start_steady_ns_ = 0;
  last_stop_condition_steady_ns_ = 0;
  stop_hold_zone_name_.clear();
  is_forwarding_ = false;
  zero_vel_published_this_cycle_ = false;
  odom_linear_x_.store(0.0, std::memory_order_relaxed);
  odom_rx_steady_time_ns_.store(0, std::memory_order_relaxed);

  // Deactivating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    polygon->setFrontPadding(0.0);
    polygon->deactivate();
  }

  // A5: Before deactivating cmd_vel_out_pub_, publish one 0-velocity frame in auto mode
  if (configured_navigation_type_ == NavigationType::AUTOMONOUS &&
    cmd_vel_out_pub_ && cmd_vel_out_pub_->is_activated())
  {
    cmd_vel_out_pub_->publish(std::make_unique<geometry_msgs::msg::Twist>());
  }

  // Deactivating lifecycle publishers
  cmd_vel_out_pub_->on_deactivate();
  obstacle_status_pub_->on_deactivate();
  if (hard_stop_pub_) {
    hard_stop_pub_->on_deactivate();
  }
  if (path_stop_marker_pub_) {
    path_stop_marker_pub_->on_deactivate();
  }
  if (ultrasonic_points_pub_) {
    ultrasonic_points_pub_->on_deactivate();
  }

  // Destroying bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionMonitor::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  control_loop_timer_.reset();
  cmd_vel_in_sub_.reset();
  tracking_path_sub_.reset();
  free_navigation_path_sub_.reset();
  odom_sub_.reset();
  cmd_vel_out_pub_.reset();
  obstacle_status_pub_.reset();
  hard_stop_pub_.reset();
  path_stop_marker_pub_.reset();
  ultrasonic_points_pub_.reset();
  set_padding_srv_.reset();
  io_callback_group_.reset();
  control_callback_group_.reset();

  polygons_.clear();
  sources_.clear();

  tf_listener_.reset();
  tf_buffer_.reset();
  {
    std::lock_guard<std::mutex> lock(tracking_path_mutex_);
    tracking_path_.reset();
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionMonitor::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");

  return nav2_util::CallbackReturn::SUCCESS;
}

void CollisionMonitor::cmdVelInCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  // If message contains NaN or Inf, ignore
  if (!nav2_util::validateTwist(*msg)) {
    RCLCPP_ERROR(get_logger(), "Velocity message contains NaNs or Infs! Ignoring as invalid!");
    return;
  }

  // 回调线程不执行碰撞计算；保留最新一帧和接收时间，由控制周期统一消费。
  std::lock_guard<std::mutex> lock(mailbox_mutex_);
  cmd_mailbox_.message = std::move(msg);
  cmd_mailbox_.stamp = this->now();
  ++cmd_mailbox_.sequence;
}

void CollisionMonitor::freeNavigationPathCallback(nav_msgs::msg::Path::ConstSharedPtr msg)
{
  // 路径到达代表自由导航任务活跃；不在 DDS 回调中切换模式或修改 padding。
  std::lock_guard<std::mutex> lock(mailbox_mutex_);
  pending_free_path_ = std::move(msg);
  ++free_path_sequence_;
}

void CollisionMonitor::odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  double vx = msg->twist.twist.linear.x;
  if (!std::isfinite(vx)) {
    vx = 0.0;
  }
  odom_linear_x_.store(vx, std::memory_order_relaxed);
  odom_rx_steady_time_ns_.store(steadyNowNs(), std::memory_order_relaxed);
}

void CollisionMonitor::controlLoopCallback()
{
  if (!process_active_) {
    return;
  }
  const auto control_loop_start = std::chrono::steady_clock::now();
  const auto & input = readControlInput();
  bool input_updated = hasUpdatedInput(input);
  const rclcpp::Time now = this->now();
  zero_vel_published_this_cycle_ = false;
  const bool cmd_fresh = input.cmd.message && (now - input.cmd.stamp <= cmd_vel_timeout_);
  // 先根据输入更新导航模式与转发门控状态
  updateNavigationMode(input, now);
  is_forwarding_ = (navigation_mode_ != NavigationMode::IDLE) && cmd_fresh;

  // 传感器故障会发布停车，本周期不再处理后续速度。
  if (checkSensorTimeout(now)) {
    const bool speed_tier_changed = updateSpeedTier(now);
    const bool in_stop_or_hold = (robot_action_prev_.action_type == STOP) ||
      (steadyNowNs() - last_stop_condition_steady_ns_ < clear_hold_time_ns_);
    const bool in_slowdown_hold =
      (steadyNowNs() - last_slowdown_condition_steady_ns_ < slowdown_hold_time_ns_);
    input_updated = input_updated || speed_tier_changed || in_stop_or_hold || in_slowdown_hold;
    processVelocityInput(input.cmd, input_updated, now);
  }
  recordControlLoopTiming(control_loop_start);
}

const CollisionMonitor::ControlInput & CollisionMonitor::readControlInput()
{
  // 锁内只复制 shared_ptr、时间戳和 sequence。TF、几何计算及 publish 全部在锁外，
  // 避免 100 Hz 检测阻塞高频传感器/速度回调。
  {
    std::lock_guard<std::mutex> lock(mailbox_mutex_);
    control_input_.cmd = cmd_mailbox_;
    control_input_.free_path = pending_free_path_;
    control_input_.tracking_path = pending_tracking_path_;
    control_input_.new_free_path = free_path_sequence_ != consumed_free_path_sequence_;
    control_input_.new_tracking_path = tracking_path_sequence_ != consumed_tracking_path_sequence_;
    consumed_free_path_sequence_ = free_path_sequence_;
    consumed_tracking_path_sequence_ = tracking_path_sequence_;
  }
  return control_input_;
}

bool CollisionMonitor::hasUpdatedInput(const ControlInput & input)
{
  // 追踪路径、速度和传感器更新，避免对相同输入重复执行碰撞检测。
  bool input_updated = input.new_free_path || input.new_tracking_path;
  if (input.cmd.sequence != observed_cmd_sequence_) {
    observed_cmd_sequence_ = input.cmd.sequence;
    input_updated = true;
  }
  if (observed_source_sequences_.size() != sources_.size()) {
    observed_source_sequences_.resize(sources_.size(), 0);
  }
  for (std::size_t i = 0; i < sources_.size(); ++i) {
    const std::uint64_t sequence = sources_[i]->getDataSequence();
    if (sequence != observed_source_sequences_[i]) {
      observed_source_sequences_[i] = sequence;
      input_updated = true;
    }
  }
  return input_updated;
}

bool CollisionMonitor::checkSensorTimeout(const rclcpp::Time & now)
{
  // ── 传感器有效性与超时监测（A1、A3 规范）──
  // 1. 单调时钟检测数据源是否停止到达或从未到达；
  // 2. 消息头时间戳检测是否存在过大时延（旧数据）；
  // 3. TF 坐标转换连续失败判定（A3）：激光雷达仅在坐标转换成功时刷新 last_valid，
  //    若数据到达但 TF 连续失败超过 sensor_fault_timeout_ns_，与断流同等判定为失效停障；
  // 4. 从未收到的传感器从节点启动时刻 source_start_time_ns_ 开始计时；
  // 5. 覆盖所有启用的观测源（激光 scan、超声波 range、点云 pointcloud）。
  const auto steady_now = steadyNowNs();
  std::string stale_sensor;
  std::string failure_detail;
  for (const auto & source : sources_) {
    const auto fault_timeout_ns = source->getSensorFaultTimeoutNs();
    const auto last_received = source->getLastReceivedSteadyNs();
    const auto stamp = source->getLastSourceStampNs();
    if (std::dynamic_pointer_cast<Scan>(source)) {
      const auto last_valid = source->getLastValidFrameSteadyNs();
      if (last_received == 0) {
        // 启动后从未收到数据，且已超过超时阈值
        if (steady_now - source_start_time_ns_ >= fault_timeout_ns) {
          stale_sensor = source->getName();
          failure_detail = "no data";
          break;
        }
      } else if (steady_now - last_received >= fault_timeout_ns) {
        // 数据停更（断线/驱动崩溃）
        stale_sensor = source->getName();
        failure_detail = "no data";
        break;
      } else if (steady_now - last_valid >= fault_timeout_ns) {
        // 数据虽在到达，但 TF 坐标变换连续失败超时（A3 规范：看不见当没有障碍放行的漏洞修复）
        stale_sensor = source->getName();
        failure_detail = "transform failure";
        break;
      } else if (now.nanoseconds() - stamp >= fault_timeout_ns) {
        // 数据延迟到达，时间戳过旧
        stale_sensor = source->getName();
        failure_detail = "stale timestamp";
        break;
      }
    } else {
      const auto since = std::max(last_received, source_start_time_ns_);
      if (steady_now - since >= fault_timeout_ns ||
        (last_received != 0 && now.nanoseconds() - stamp >= fault_timeout_ns))
      {
        stale_sensor = source->getName();
        failure_detail = (last_received == 0 || steady_now - since >= fault_timeout_ns) ?
          "no data" : "stale timestamp";
        break;
      }
    }
  }
  if (!stale_sensor.empty()) {
    // ── 故障动作闭环 ──
    // 传感器失效时打印 ERROR 日志（带具体失效原因），刷新状态上报与停止保持计时
    if (!sensor_fault_active_ || stale_sensor != last_sensor_fault_source_ ||
      failure_detail != last_sensor_fault_reason_)
    {
      RCLCPP_ERROR(
        get_logger(), "Sensor %s timed out (%s); stopping",
        stale_sensor.c_str(), failure_detail.c_str());
    }
    sensor_fault_active_ = true;
    last_sensor_fault_source_ = stale_sensor;
    last_sensor_fault_reason_ = failure_detail;

    // 记录停障触发时刻，用于停止解除防抖保持（A7）
    last_stop_condition_steady_ns_ = steady_now;
    stop_hold_zone_name_.clear();

    // 发出硬停清零通知并输出零速（受转发门控 A6 保护），实时上报 STOP
    publishHardStopOnStopEdge("sensor timeout");
    publishZeroIfForwarding();
    publishObstacleStatus({STOP, {0.0, 0.0, 0.0}}, nullptr);
    robot_action_prev_ = {STOP, {0.0, 0.0, 0.0}};
    return false;
  }
  if (sensor_fault_active_) {
    RCLCPP_INFO(get_logger(), "All required sensors recovered");
    sensor_fault_active_ = false;
    last_sensor_fault_source_.clear();
    last_sensor_fault_reason_.clear();
  }
  return true;
}

void CollisionMonitor::updateNavigationMode(
  const ControlInput & input, const rclcpp::Time & now)
{
  const bool cmd_fresh = input.cmd.message && now - input.cmd.stamp <= cmd_vel_timeout_;
  // 只执行配置导航类型的路径和状态检查。
  switch (configured_navigation_type_) {
    case NavigationType::AUTOMONOUS:
      checkAutonomousNavigation(
        input.free_path, input.new_free_path, cmd_fresh, input.cmd.stamp, now);
      break;
    case NavigationType::SCHEDULE:
      checkScheduleNavigation(
        input.tracking_path, input.new_tracking_path, cmd_fresh, input.cmd.stamp, now);
      break;
  }
}

void CollisionMonitor::processVelocityInput(
  const VelocityMailbox & cmd, bool input_updated, const rclcpp::Time & now)
{
  // IDLE 没有活动速度源；两种导航类型共用同一条速度安全处理链路。
  const VelocityMailbox * active_cmd =
    navigation_mode_ == NavigationMode::IDLE ? nullptr : &cmd;

  // 只在速度新鲜度状态变化时记录日志，避免每个定时周期重复打印。
  const double velocity_age = active_cmd ?
    (now - (active_cmd->message ? active_cmd->stamp : mode_last_cmd_stamp_)).seconds() : 0.0;
  const int velocity_state = !active_cmd ? 0 : !active_cmd->message ? 1 :
    velocity_age <= cmd_vel_timeout_.seconds() ? 2 : 3;
  if (velocity_state != logged_velocity_state_) {
    const char * states[] = {"IDLE", "WAITING_FIRST", "FRESH", "STALE"};
    RCLCPP_INFO(
      get_logger(),
      "[FSM] velocity=%s mode=%s age=%.3fs sequence=%llu stamp=%.6f output=%s",
      states[velocity_state], navigationModeName(navigation_mode_), velocity_age,
      static_cast<unsigned long long>(active_cmd ? active_cmd->sequence : 0),
      active_cmd && active_cmd->message ? active_cmd->stamp.seconds() : 0.0,
      velocity_state == 2 && configured_navigation_type_ == NavigationType::AUTOMONOUS ?
      "filtered_velocity" : "status_only");
    logged_velocity_state_ = velocity_state;
  }

  if (active_cmd && active_cmd->message) {
    const rclcpp::Duration age = now - active_cmd->stamp;
    // 新鲜速度仅在输入更新时进入障碍检测；process() 决定是否需要发布安全速度。
    if (age <= cmd_vel_timeout_) {
      const auto & msg = *active_cmd->message;
      const bool command_updated = active_cmd->sequence != consumed_cmd_sequence_;
      consumed_cmd_sequence_ = active_cmd->sequence;
      if (input_updated) {
        // 速度下发按导航链路区分：
        // - auto：本节点串联在速度链里，输出过滤/限速后的安全速度。
        // - schedule：path_follower 直连 velocity_smoother，本节点只把该速度当作
        //   障碍检测的几何上下文（旁路观察），减速/停车由 path_follower 依据
        //   obstacle_status 执行。这里 publish_cmd_vel=false，避免同一障碍被两处
        //   各刹一次，也不把调度速度回灌进速度链。
        const bool publish_gated_velocity =
          configured_navigation_type_ == NavigationType::AUTOMONOUS;
        process(
          {msg.linear.x, msg.linear.y, msg.angular.z}, publish_gated_velocity, false,
          command_updated);
      }
      return;
    }
    // 超过 cmd_vel_timeout_（设计值 0.3 s）后不再使用旧速度检测，仅检查车身 STOP Polygon。
    // 期间恢复速度可继续原任务；连续 idle_grace_period_（设计值 5 s）未更新才进入 IDLE。
    // 实际超时阈值由对应参数配置决定。
    if (age >= idle_grace_period_) {
      RCLCPP_INFO(
        get_logger(), "[FSM] idle timeout: age=%.3fs limit=%.3fs",
        age.seconds(), idle_grace_period_.seconds());
      enterIdle("no velocity after task");
    }
  }

  // 路径已经切换了模式、但该速度源尚未给过第一帧时使用模式进入时间计时，
  // 达到 idle_grace_period_ 仍没有速度输入时进入 IDLE，并发布一次零速。
  if (active_cmd && !active_cmd->message) {
    const rclcpp::Duration age = now - mode_last_cmd_stamp_;
    if (age >= idle_grace_period_) {
      RCLCPP_INFO(
        get_logger(), "[FSM] first velocity timeout: age=%.3fs limit=%.3fs",
        age.seconds(), idle_grace_period_.seconds());
      enterIdle("task did not produce velocity");
    }
  }

  // IDLE 或等待下一任务期间仍以 100 Hz 检查车身周围 STOP Polygon；零速度
  // 仅用于几何检测上下文，不会从 collision monitor 生成新的运动指令。
  if (input_updated) {
    process({0.0, 0.0, 0.0}, false, true);
  }
}

void CollisionMonitor::checkAutonomousNavigation(
  const nav_msgs::msg::Path::ConstSharedPtr & path, bool new_path,
  bool cmd_fresh, const rclcpp::Time & cmd_stamp, const rclcpp::Time & now)
{
  // 只有非空的新自由导航路径才能通过路径输入激活自动导航。
  if (new_path && path && !path->poses.empty()) {
    changeNavigationMode(NavigationMode::AUTOMONOUS_NAVIGATION, "new automatic path");
    mode_last_cmd_stamp_ = now;
    applyNavigationPadding(navigation_mode_);
    std::lock_guard<std::mutex> lock(tracking_path_mutex_);
    tracking_path_.reset();
  }

  // 超时进入 IDLE 后，新鲜速度可恢复已配置的自动导航模式。
  if (navigation_mode_ == NavigationMode::IDLE && cmd_fresh) {
    changeNavigationMode(NavigationMode::AUTOMONOUS_NAVIGATION, "fresh automatic velocity");
    mode_last_cmd_stamp_ = cmd_stamp;
    applyNavigationPadding(navigation_mode_);
  }
}

void CollisionMonitor::checkScheduleNavigation(
  const nav_interfaces::msg::TrackingPath::ConstSharedPtr & path, bool new_path,
  bool cmd_fresh, const rclcpp::Time & cmd_stamp, const rclcpp::Time & now)
{
  // 有限且接近零的 padding 选择贴边模式；其他值选择普通调度模式。
  if (new_path && path && !path->poses.empty()) {
    const bool finite_padding = std::isfinite(path->padding);
    schedule_navigation_mode_ =
      finite_padding && std::fabs(path->padding) < 1e-6F ?
      NavigationMode::SCHEDULE_EDGING : NavigationMode::SCHEDULE_NAVIGATION;
    changeNavigationMode(schedule_navigation_mode_, "new tracking path");
    mode_last_cmd_stamp_ = now;
    applyNavigationPadding(navigation_mode_);
    if (navigation_mode_ == NavigationMode::SCHEDULE_NAVIGATION) {
      for (const auto & polygon : polygons_) {
        if (polygon->getName() == "PolygonStop") {
          polygon->setPadding(finite_padding ? path->padding : polygon->getDefaultPadding());
          break;
        }
      }
    }
    std::lock_guard<std::mutex> lock(tracking_path_mutex_);
    tracking_path_ = path;
  }

  // 新鲜速度从 IDLE 恢复时，沿用上次选择的调度子模式。
  if (navigation_mode_ == NavigationMode::IDLE && cmd_fresh) {
    changeNavigationMode(schedule_navigation_mode_, "fresh schedule velocity");
    mode_last_cmd_stamp_ = cmd_stamp;
    applyNavigationPadding(navigation_mode_);
  }
}

void CollisionMonitor::publishVelocity(const Action & robot_action)
{
  if (robot_action.req_vel.isZero()) {
    if (robot_action.action_type == STOP) {
      // STOP 期间每个控制周期都发零速，不受 stop_pub_timeout 限制 (A4)
    } else {
      if (!robot_action_prev_.req_vel.isZero()) {
        // Robot just stopped: saving stop timestamp and continue
        stop_stamp_ = this->now();
      } else if (this->now() - stop_stamp_ > stop_pub_timeout_) {
        // More than stop_pub_timeout_ passed after robot has been stopped.
        // Cease publishing output cmd_vel.
        return;
      }
    }
  }

  std::unique_ptr<geometry_msgs::msg::Twist> cmd_vel_out_msg =
    std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel_out_msg->linear.x = robot_action.req_vel.x;
  cmd_vel_out_msg->linear.y = robot_action.req_vel.y;
  cmd_vel_out_msg->angular.z = robot_action.req_vel.tw;
  // linear.z, angular.x and angular.y will remain 0.0

  cmd_vel_out_pub_->publish(std::move(cmd_vel_out_msg));
  if (robot_action.req_vel.isZero()) {
    zero_vel_published_this_cycle_ = true;
  }
}

void CollisionMonitor::publishObstacleStatus(
  const Action & robot_action,
  const std::shared_ptr<Polygon> action_polygon,
  const std::string & override_zone_name) const
{
  auto msg = std::make_unique<StatusMsg>();
  msg->header.stamp = this->now();
  msg->header.frame_id = base_frame_id_;

  switch (robot_action.action_type) {
    case STOP:
      msg->status = StatusMsg::STOP;
      msg->slowdown_ratio = 0.0f;
      break;
    case SLOWDOWN:
      msg->status = StatusMsg::SLOWDOWN;
      msg->slowdown_ratio = action_polygon ?
        static_cast<float>(action_polygon->getSlowdownRatio()) : 0.0f;
      break;
    case APPROACH:
      // APPROACH 也是"降速不停止"的语义，并入 SLOWDOWN 三态模型
      msg->status = StatusMsg::SLOWDOWN;
      msg->slowdown_ratio = 0.0f;
      break;
    default:  // DO_NOTHING
      msg->status = StatusMsg::FREE;
      msg->slowdown_ratio = 0.0f;
      break;
  }
  if (!override_zone_name.empty()) {
    msg->zone_name = override_zone_name;
  } else if (action_polygon) {
    if (action_polygon->getName() == "PolygonStop" &&
      navigation_mode_ == NavigationMode::AUTOMONOUS_NAVIGATION &&
      speed_tiers_enabled_)
    {
      msg->zone_name = "PolygonStop[t" + std::to_string(current_speed_tier_) + "]";
    } else {
      msg->zone_name = action_polygon->getName();
    }
  } else {
    msg->zone_name = "";
  }

  obstacle_status_pub_->publish(std::move(msg));
}

bool CollisionMonitor::getParameters(
  std::string & cmd_vel_in_topic,
  std::string & cmd_vel_out_topic)
{
  std::string base_frame_id, odom_frame_id;
  tf2::Duration transform_tolerance;
  rclcpp::Duration source_timeout(2.0, 0.0);

  auto node = shared_from_this();

  nav2_util::declare_parameter_if_not_declared(
    node, "cmd_vel_in_topic", rclcpp::ParameterValue("cmd_vel_raw"));
  cmd_vel_in_topic = get_parameter("cmd_vel_in_topic").as_string();
  nav2_util::declare_parameter_if_not_declared(
    node, "cmd_vel_out_topic", rclcpp::ParameterValue("cmd_vel"));
  cmd_vel_out_topic = get_parameter("cmd_vel_out_topic").as_string();

  nav2_util::declare_parameter_if_not_declared(
    node, "base_frame_id", rclcpp::ParameterValue("base_footprint"));
  base_frame_id = get_parameter("base_frame_id").as_string();
  base_frame_id_ = base_frame_id;
  nav2_util::declare_parameter_if_not_declared(
    node, "odom_frame_id", rclcpp::ParameterValue("odom"));
  odom_frame_id = get_parameter("odom_frame_id").as_string();
  nav2_util::declare_parameter_if_not_declared(
    node, "transform_tolerance", rclcpp::ParameterValue(0.1));
  transform_tolerance =
    tf2::durationFromSec(get_parameter("transform_tolerance").as_double());
  transform_tolerance_ = transform_tolerance;
  nav2_util::declare_parameter_if_not_declared(
    node, "source_timeout", rclcpp::ParameterValue(2.0));
  source_timeout =
    rclcpp::Duration::from_seconds(get_parameter("source_timeout").as_double());
  // 优先读取 sensor_fault_timeout；向后兼容旧名称 scan_fault_timeout
  nav2_util::declare_parameter_if_not_declared(
    node, "sensor_fault_timeout", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, "scan_fault_timeout", rclcpp::ParameterValue(0.5));

  double sensor_fault_timeout = get_parameter("sensor_fault_timeout").as_double();
  if (node->has_parameter("scan_fault_timeout") && sensor_fault_timeout == 0.5) {
    const double scan_timeout_val = get_parameter("scan_fault_timeout").as_double();
    if (scan_timeout_val != 0.5) {
      sensor_fault_timeout = scan_timeout_val;
    }
  }

  if (!std::isfinite(sensor_fault_timeout) || sensor_fault_timeout <= 0.0) {
    RCLCPP_ERROR(get_logger(), "sensor_fault_timeout must be positive and finite");
    return false;
  }
  sensor_fault_timeout_ns_ = static_cast<std::int64_t>(sensor_fault_timeout * 1e9);

  nav2_util::declare_parameter_if_not_declared(
    node, "base_shift_correction", rclcpp::ParameterValue(true));
  const bool base_shift_correction =
    get_parameter("base_shift_correction").as_bool();

  nav2_util::declare_parameter_if_not_declared(
    node, "base_shift_tolerance", rclcpp::ParameterValue(0.030));

  nav2_util::declare_parameter_if_not_declared(
    node, "stop_pub_timeout", rclcpp::ParameterValue(1.0));
  stop_pub_timeout_ =
    rclcpp::Duration::from_seconds(get_parameter("stop_pub_timeout").as_double());

  nav2_util::declare_parameter_if_not_declared(
    node, "clear_hold_time", rclcpp::ParameterValue(2.0));
  clear_hold_time_ = get_parameter("clear_hold_time").as_double();
  if (clear_hold_time_ < 0.0 || !std::isfinite(clear_hold_time_)) {
    RCLCPP_ERROR(get_logger(), "clear_hold_time must be >= 0 and finite");
    return false;
  }
  clear_hold_time_ns_ = static_cast<std::int64_t>(clear_hold_time_ * 1e9);

  nav2_util::declare_parameter_if_not_declared(
    node, "slowdown_hold_time", rclcpp::ParameterValue(0.0));
  slowdown_hold_time_ = get_parameter("slowdown_hold_time").as_double();
  if (slowdown_hold_time_ < 0.0 || !std::isfinite(slowdown_hold_time_)) {
    RCLCPP_ERROR(get_logger(), "slowdown_hold_time must be >= 0 and finite");
    return false;
  }
  slowdown_hold_time_ns_ = static_cast<std::int64_t>(slowdown_hold_time_ * 1e9);

  if (!configurePolygons(base_frame_id, transform_tolerance)) {
    return false;
  }

  if (
    !configureSources(
      base_frame_id, odom_frame_id, transform_tolerance, source_timeout, base_shift_correction))
  {
    return false;
  }

  return true;
}

bool CollisionMonitor::configurePolygons(
  const std::string & base_frame_id,
  const tf2::Duration & transform_tolerance)
{
  try {
    auto node = shared_from_this();

    nav2_util::declare_parameter_if_not_declared(
      node, "polygons", rclcpp::ParameterValue(std::vector<std::string>()));
    std::vector<std::string> polygon_names = get_parameter("polygons").as_string_array();
    for (std::string polygon_name : polygon_names) {
      // Leave it not initialized: the will cause an error if it will not set
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name + ".type", rclcpp::PARAMETER_STRING);
      const std::string polygon_type = get_parameter(polygon_name + ".type").as_string();

      if (polygon_type == "polygon") {
        polygons_.push_back(
          std::make_shared<Polygon>(
            node, polygon_name, tf_buffer_, base_frame_id, transform_tolerance));
      } else if (polygon_type == "circle") {
        const auto circle = std::make_shared<Circle>(
          node, polygon_name, tf_buffer_, base_frame_id, transform_tolerance);
        polygons_.push_back(circle);
      } else {  // Error if something else
        RCLCPP_ERROR(
          get_logger(),
          "[%s]: Unknown polygon type: %s",
          polygon_name.c_str(), polygon_type.c_str());
        return false;
      }

      // Configure last added polygon
      if (!polygons_.back()->configure()) {
        return false;
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while getting parameters: %s", ex.what());
    return false;
  }

  return true;
}

bool CollisionMonitor::configureSources(
  const std::string & base_frame_id,
  const std::string & odom_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout,
  const bool base_shift_correction)
{
  try {
    auto node = shared_from_this();

    // Leave it to be not initialized: to intentionally cause an error if it will not set
    nav2_util::declare_parameter_if_not_declared(
      node, "observation_sources", rclcpp::PARAMETER_STRING_ARRAY);
    std::vector<std::string> source_names = get_parameter("observation_sources").as_string_array();
    for (std::string source_name : source_names) {
      nav2_util::declare_parameter_if_not_declared(
        node, source_name + ".type",
        rclcpp::ParameterValue("scan"));  // Laser scanner by default
      const std::string source_type = get_parameter(source_name + ".type").as_string();

      if (source_type == "scan") {
        std::shared_ptr<Scan> s = std::make_shared<Scan>(
          node, source_name, tf_buffer_, base_frame_id, odom_frame_id,
          transform_tolerance, source_timeout, base_shift_correction);

        s->configure();

        sources_.push_back(s);
      } else if (source_type == "pointcloud") {
        std::shared_ptr<PointCloud> p = std::make_shared<PointCloud>(
          node, source_name, tf_buffer_, base_frame_id, odom_frame_id,
          transform_tolerance, source_timeout, base_shift_correction);

        p->configure();

        sources_.push_back(p);
      } else if (source_type == "range") {
        std::shared_ptr<Range> r = std::make_shared<Range>(
          node, source_name, tf_buffer_, base_frame_id, odom_frame_id,
          transform_tolerance, source_timeout, base_shift_correction);

        r->configure();

        sources_.push_back(r);
      } else {  // Error if something else
        RCLCPP_ERROR(
          get_logger(),
          "[%s]: Unknown source type: %s",
          source_name.c_str(), source_type.c_str());
        return false;
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while getting parameters: %s", ex.what());
    return false;
  }

  return true;
}

void CollisionMonitor::process(
  const Velocity & cmd_vel_in, bool publish_cmd_vel, bool idle_polygon_only,
  bool command_updated)
{
  // Current timestamp for all inner routines prolongation
  rclcpp::Time curr_time = this->now();

  // Do nothing if main worker in non-active state
  if (!process_active_) {
    return;
  }
  const auto process_start = std::chrono::steady_clock::now();

  // Points array collected from different data sources in a robot base frame. The control
  // callback is mutually exclusive, so retaining capacity here is safe and avoids a large
  raw_points_buffer_.clear();
  range_slow_buffer_.clear();
  range_stop_only_buffer_.clear();
  ultrasonic_points_buffer_.clear();

  // Each source takes an atomic snapshot of its latest immutable message.
  // Point conversion runs outside cache synchronization; this local vector has one writer.
  for (const auto & source : sources_) {
    if (std::dynamic_pointer_cast<Range>(source)) {
      const size_t prev_size = ultrasonic_points_buffer_.size();
      source->getData(curr_time, ultrasonic_points_buffer_);
      const size_t new_pts = ultrasonic_points_buffer_.size() - prev_size;
      if (new_pts > 0) {
        if (source->isStopOnly()) {
          range_stop_only_buffer_.insert(
            range_stop_only_buffer_.end(),
            ultrasonic_points_buffer_.begin() + prev_size,
            ultrasonic_points_buffer_.end());
        } else {
          range_slow_buffer_.insert(
            range_slow_buffer_.end(),
            ultrasonic_points_buffer_.begin() + prev_size,
            ultrasonic_points_buffer_.end());
        }
      }
    } else {
      source->getData(curr_time, raw_points_buffer_);
    }
  }

  // 没有订阅者时不发布超声波点云；有订阅者时发布
  if (ultrasonic_points_pub_ && ultrasonic_points_pub_->get_subscription_count() > 0) {
    publishUltrasonicPointCloud(curr_time, ultrasonic_points_buffer_);
  }

  // ── 零拷贝点云缓冲池分流组合 ──
  // 1. SLOWDOWN 减速多边形：雷达/RGBD 点 + 允许减速的超声波点
  const std::vector<const std::vector<Point> *> slowdown_buffers{
    &raw_points_buffer_, &range_slow_buffer_};
  // 2. STOP 停障多边形：雷达/RGBD 点 + 允许减速的超声波点 + 仅停障的超声波点
  const std::vector<const std::vector<Point> *> all_stop_buffers{
    &raw_points_buffer_, &range_slow_buffer_, &range_stop_only_buffer_};
  // 3. APPROACH 速度轨迹预测：仅使用雷达/RGBD 原始点云 raw_points_buffer_

  // ── 空旷无点判定（A2 规范）──
  // 深度相机与激光雷达在开阔空旷场景下可能有效检出 0 个障碍点；
  // 遵照 CE 口径：全源无点只打印 WARN 限速日志，严禁误触发停障，保证空旷场地顺畅通行
  if (raw_points_buffer_.empty() && range_slow_buffer_.empty() && range_stop_only_buffer_.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), throttle_clock_, 5000,
      "No collision points received from any source");
  }

  // ── 外轮廓有效性检查（A1 规范）──
  for (const auto & polygon : polygons_) {
    polygon->updatePolygon();
    if (polygon->missingFootprint()) {
      const auto steady_now = steadyNowNs();
      last_stop_condition_steady_ns_ = steady_now;
      stop_hold_zone_name_.clear();

      publishHardStopOnStopEdge("footprint missing");
      publishZeroIfForwarding();
      publishObstacleStatus({STOP, {0.0, 0.0, 0.0}}, nullptr);
      robot_action_prev_ = {STOP, {0.0, 0.0, 0.0}};
      return;
    }
  }

  // 默认动作：无避障干预，正常下发输入速度
  Action robot_action{DO_NOTHING, cmd_vel_in};
  // 触发当前避障动作的多边形指针
  std::shared_ptr<Polygon> action_polygon;
  bool path_stop = false;

  for (std::shared_ptr<Polygon> polygon : polygons_) {
    if (robot_action.action_type == STOP) {
      // 一旦前序优先级更高的多边形已判定必须 STOP，无需继续判定低级动作
      break;
    }

    const ActionType at = polygon->getActionType();
    // IDLE 只保留近身静态停车区，跳过 SLOWDOWN、APPROACH 和后面的路径走廊
    if (idle_polygon_only && at != STOP) {
      continue;
    }
    // 正常导航启用 APPROACH 速度轨迹预测，可通过 approach_enabled 显式关闭
    if (!approach_enabled_ && at == APPROACH) {
      continue;
    }
    if (at == STOP) {
      // STOP 停障多边形消费全量点云（雷达 + RGBD + 4路超声波，零拷贝透传 buffers），单帧滤波有效即刻触发刹停
      if (processStopSlowdown(polygon, all_stop_buffers, cmd_vel_in, robot_action)) {
        action_polygon = polygon;
      }
    } else if (at == SLOWDOWN) {
      // SLOWDOWN 减速多边形消费雷达点与允许减速的超声波探头点（零拷贝透传）
      if (processStopSlowdown(polygon, slowdown_buffers, cmd_vel_in, robot_action)) {
        action_polygon = polygon;
      }
    } else if (at == APPROACH) {
      const auto approach_start = std::chrono::steady_clock::now();
      // APPROACH 速度轨迹预测：同时兼容雷达/深度相机点与允许减速的超声波探头点（零拷贝分流分别评估，取最安全值）
      if (!raw_points_buffer_.empty()) {
        if (processApproach(polygon, raw_points_buffer_, cmd_vel_in, robot_action)) {
          action_polygon = polygon;
        }
      }
      if (!range_slow_buffer_.empty()) {
        if (processApproach(polygon, range_slow_buffer_, cmd_vel_in, robot_action)) {
          action_polygon = polygon;
        }
      }
      if (performance_timing_enabled_) {
        recordPerformanceDuration(approach_samples_ms_, approach_start);
      }
    }
  }

  // 路径走廊检测仅用于调度贴边模式
  if (path_stop_enabled_ && !idle_polygon_only &&
    navigation_mode_ == NavigationMode::SCHEDULE_EDGING)
  {
    const auto path_stop_start = std::chrono::steady_clock::now();
    // 路径走廊具备最高停车优先级，覆盖多边形动作
    if (hasObstacleOnPath(raw_points_buffer_)) {
      robot_action = {STOP, {0.0, 0.0, 0.0}};
      action_polygon.reset();
      path_stop = true;
    }
    if (path_stop_timing_enabled_) {
      recordPathStopTiming(path_stop_start);
    }
    if (performance_timing_enabled_) {
      recordPerformanceDuration(path_corridor_samples_ms_, path_stop_start);
    }
  }
  const auto steady_now = steadyNowNs();
  std::string current_zone_name;

  // ── 避障状态判定与停止解除保持状态机（A1、A7、O6 规范）──
  if (robot_action.action_type == STOP) {
    // 1. 本周期物理条件满足停障（障碍物入区）
    if (action_polygon) {
      // O6 规范：若处于自动导航分档模式，上报的区域名附带档位标识（如 PolygonStop[t1]），便于事后录包分析
      if (action_polygon->getName() == "PolygonStop" &&
        navigation_mode_ == NavigationMode::AUTOMONOUS_NAVIGATION &&
        speed_tiers_enabled_)
      {
        current_zone_name = "PolygonStop[t" + std::to_string(current_speed_tier_) + "]";
      } else {
        current_zone_name = action_polygon->getName();
      }
    } else if (path_stop) {
      current_zone_name = "tracking_path";
    }
    // 记录停障触发最新时刻，并锁定本次停障区域
    last_stop_condition_steady_ns_ = steady_now;
    stop_hold_zone_name_ = current_zone_name;

    // 停障优先级最高，清空减速保持
    last_slowdown_condition_steady_ns_ = 0;
  } else if (steady_now - last_stop_condition_steady_ns_ < clear_hold_time_ns_) {
    // 2. A7 规范：停止解除防抖保持期内（障碍刚离开，但时间未满 clear_hold_time 2.0s）
    // 强制继续锁定输出 STOP，沿用最后一次停障的区域名，杜绝障碍在边界徘徊时车体反复起步刹车
    robot_action = {STOP, {0.0, 0.0, 0.0}};
    action_polygon.reset();
    current_zone_name = stop_hold_zone_name_;

    // 停障保持期间清空减速保持
    last_slowdown_condition_steady_ns_ = 0;
  } else {
    // 3. 停止保持期满，解除停障锁定
    stop_hold_zone_name_.clear();

    if (robot_action.action_type == SLOWDOWN) {
      // 3.1 本周期物理触发 SLOWDOWN：刷新减速触发时间戳并记录减速上下文
      last_slowdown_condition_steady_ns_ = steady_now;
      slowdown_hold_polygon_ = action_polygon;
      slowdown_hold_ratio_ = action_polygon ? action_polygon->getSlowdownRatio() : 0.5;
      if (action_polygon) {
        current_zone_name = action_polygon->getName();
      }
    } else if (steady_now - last_slowdown_condition_steady_ns_ < slowdown_hold_time_ns_) {
      // 3.2 减速防抖保持期内（障碍瞬时跳变丢失或颠簸漏检，时间未满 slowdown_hold_time 1.0s）
      // 强制维持 SLOWDOWN，杜绝减速中途车体突然全速猛冲
      robot_action.action_type = SLOWDOWN;
      robot_action.req_vel = cmd_vel_in * slowdown_hold_ratio_;
      action_polygon = slowdown_hold_polygon_;
      current_zone_name = slowdown_hold_polygon_ ? slowdown_hold_polygon_->getName() : "PolygonSlow";
    } else if (robot_action.action_type == APPROACH) {
      if (action_polygon) {
        current_zone_name = action_polygon->getName();
      }
    } else {
      slowdown_hold_polygon_.reset();
    }
  }

  const bool action_changed = robot_action.action_type != robot_action_prev_.action_type;
  if (robot_action.action_type == STOP) {
    // 进入 STOP 的上升沿向平滑器发送硬停清零通知（仅在转发新鲜速度时触发，A6 门控）
    publishHardStopOnStopEdge(path_stop ? "obstacle on tracking path" : "obstacle in stop zone");
  }
  if (action_changed) {
    // Report changed robot behavior
    if (path_stop) {
      RCLCPP_INFO(
        get_logger(), "Robot to stop: obstacle detected within %.2f m ahead on tracking path",
        path_stop_distance_);
    } else {
      printAction(robot_action, action_polygon);
    }
  }

  // Publish real-time obstacle status every cycle (not just on state change)
  publishObstacleStatus(robot_action, action_polygon, current_zone_name);

  // Publish polygons for better visualization
  publishPolygons();

  // APPROACH 类型不变时，传感器更新仍可改变安全速度（包括降为零）。
  // 必须发布新的安全速度，不能只按动作类型判断变化。
  const bool velocity_changed =
    robot_action.req_vel.x != robot_action_prev_.req_vel.x ||
    robot_action.req_vel.y != robot_action_prev_.req_vel.y ||
    robot_action.req_vel.tw != robot_action_prev_.req_vel.tw;

  if (robot_action.action_type == STOP) {
    if (publish_cmd_vel && is_forwarding_ && !zero_vel_published_this_cycle_) {
      publishVelocity(robot_action);
    }
    robot_action_prev_ = robot_action;
  } else if (publish_cmd_vel && (command_updated || action_changed || velocity_changed)) {
    // Publish required robot velocity
    publishVelocity(robot_action);
    robot_action_prev_ = robot_action;
  } else if (!publish_cmd_vel) {
    // IDLE 只更新 Action 类型；不能用零速度覆盖上一条运动速度，
    // 否则再次进入导航模式时会丢失“刚停车”的边沿判断。
    robot_action_prev_.action_type = robot_action.action_type;
  } else {
    robot_action_prev_ = robot_action;
  }
  if (performance_timing_enabled_) {
    recordPerformanceDuration(process_samples_ms_, process_start);
  }
}

void CollisionMonitor::recordPerformanceDuration(
  std::vector<double> & samples,
  const std::chrono::steady_clock::time_point & start_time) const
{
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start_time).count();
  samples.push_back(elapsed_ms);
}

void CollisionMonitor::reportPerformanceMetric(
  const char * metric, const std::vector<double> & samples,
  double window_seconds) const
{
  if (samples.empty()) {
    RCLCPP_INFO(
      get_logger(),
      "[PERF] mode=%s metric=%s window=%.3fs samples=0",
      navigationModeName(navigation_mode_), metric, window_seconds);
    return;
  }

  std::vector<double> sorted(samples);
  std::sort(sorted.begin(), sorted.end());
  const auto percentile = [&sorted](double ratio) {
      const auto rank = static_cast<std::size_t>(
        std::ceil(ratio * static_cast<double>(sorted.size())));
      return sorted[std::min(sorted.size() - 1, std::max<std::size_t>(1, rank) - 1)];
    };
  const auto over_10ms = static_cast<std::size_t>(std::count_if(
      samples.begin(), samples.end(), [](double value) {return value > 10.0;}));
  const double frequency_hz = std::string(metric) == "control_period" && window_seconds > 0.0 ?
    static_cast<double>(samples.size()) / window_seconds : 0.0;

  RCLCPP_INFO(
    get_logger(),
    "[PERF] mode=%s metric=%s window=%.3fs samples=%zu frequency=%.3fHz "
    "p50=%.3fms p95=%.3fms p99=%.3fms max=%.3fms over_10ms=%zu",
    navigationModeName(navigation_mode_), metric, window_seconds, samples.size(), frequency_hz,
    percentile(0.50), percentile(0.95), percentile(0.99), sorted.back(), over_10ms);
}

void CollisionMonitor::resetPerformanceTiming()
{
  performance_window_start_ = std::chrono::steady_clock::now();
  last_control_loop_start_ = {};
  have_last_control_loop_start_ = false;
  control_period_samples_ms_.clear();
  control_execution_samples_ms_.clear();
  process_samples_ms_.clear();
  approach_samples_ms_.clear();
  path_corridor_samples_ms_.clear();
}

void CollisionMonitor::recordControlLoopTiming(
  const std::chrono::steady_clock::time_point & start_time)
{
  if (!performance_timing_enabled_) {
    return;
  }

  if (have_last_control_loop_start_) {
    const double period_ms = std::chrono::duration<double, std::milli>(
      start_time - last_control_loop_start_).count();
    control_period_samples_ms_.push_back(period_ms);
  }
  last_control_loop_start_ = start_time;
  have_last_control_loop_start_ = true;
  recordPerformanceDuration(control_execution_samples_ms_, start_time);

  const auto finish_time = std::chrono::steady_clock::now();
  const double window_seconds = std::chrono::duration<double>(
    finish_time - performance_window_start_).count();
  if (window_seconds < performance_report_interval_) {
    return;
  }

  reportPerformanceMetric("control_period", control_period_samples_ms_, window_seconds);
  reportPerformanceMetric("control_execution", control_execution_samples_ms_, window_seconds);
  reportPerformanceMetric("process", process_samples_ms_, window_seconds);
  reportPerformanceMetric("approach", approach_samples_ms_, window_seconds);
  reportPerformanceMetric("path_corridor", path_corridor_samples_ms_, window_seconds);
  resetPerformanceTiming();
}

void CollisionMonitor::trackingPathCallback(
  nav_interfaces::msg::TrackingPath::ConstSharedPtr msg)
{
  // padding==0 的贴边判断、padding 应用和活动路径替换均留给控制主循环。
  std::lock_guard<std::mutex> lock(mailbox_mutex_);
  pending_tracking_path_ = std::move(msg);
  ++tracking_path_sequence_;
}

bool CollisionMonitor::hasObstacleOnPath(const std::vector<Point> & collision_points) const
{
  // 本检测器是现有 Polygon STOP/SLOWDOWN/APPROACH 之外的补充：它不检查机器人周围的
  // 固定区域，而是检查 schedule_planner 当前下发路径上、机器人前方指定距离内的走廊。
  // collision_points 已由各 Source 转换到 base_frame_id_，因此后续所有几何计算均为二维。
  if (!path_stop_enabled_) {
    return false;
  }

  // TrackingPath 的订阅回调和速度处理可能运行在不同 executor 线程。这里只在锁内复制
  // shared_ptr，实际路径计算放在锁外，避免点数较多时阻塞下一条路径的接收。
  nav_interfaces::msg::TrackingPath::ConstSharedPtr path;
  {
    std::lock_guard<std::mutex> lock(tracking_path_mutex_);
    path = tracking_path_;
  }
  if (!path || path->poses.empty() || path->header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), throttle_clock_, 20000,
      "Path-stop detector enabled, but no valid tracking path is available");
    return false;
  }

  std::vector<Point> path_points;
  if (!transformTrackingPathToBase(*path, path_points)) {
    return false;
  }

  const std::vector<Point> check_points = makePathCheckPoints(path_points);
  if (path_stop_visualize_ && path_stop_marker_pub_ &&
    path_stop_marker_pub_->is_activated() &&
    path_stop_marker_pub_->get_subscription_count() > 0)
  {
    publishPathStopMarkers(check_points);
  }
  return hasObstacleInPathCorridor(collision_points, check_points);
}

bool CollisionMonitor::transformTrackingPathToBase(
  const nav_interfaces::msg::TrackingPath & path,
  std::vector<Point> & points) const
{
  // TrackingPath 通常位于 map 坐标系，而障碍点位于机器人 base 坐标系。
  // TF 不可用时忽略路径检测，仍由原有固定 Polygon 检测器负责安全判定。
  tf2::Transform path_to_base;
  try {
    // TimePointZero means "latest available" in tf2. Query the buffer directly so each
    // detection cycle uses the newest transform currently present in the TF cache queue,
    // instead of associating the transform with the tracking path's timestamp.
    const geometry_msgs::msg::TransformStamped latest_transform = tf_buffer_->lookupTransform(
      base_frame_id_, path.header.frame_id, tf2::TimePointZero, transform_tolerance_);
    tf2::fromMsg(latest_transform.transform, path_to_base);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), throttle_clock_, 5000,
      "Path-stop detector cannot get latest transform from %s to %s: %s",
      path.header.frame_id.c_str(), base_frame_id_.c_str(), ex.what());
    return false;
  }
  points.clear();
  points.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    const tf2::Vector3 p = path_to_base * tf2::Vector3(
      pose.position.x, pose.position.y, pose.position.z);
    points.push_back({p.x(), p.y()});
  }
  return true;
}

std::vector<Point> CollisionMonitor::makePathCheckPoints(
  const std::vector<Point> & points) const
{
  // 在每条路径线段上计算机器人原点的垂足，取距离原点最近的垂足作为“当前路径位置”。
  // start_segment 表示垂足所在的线段，start_t 是该线段上的归一化位置：
  //   projection = points[i] + start_t * (points[i + 1] - points[i])。
  // 与直接选择最近离散点相比，投影不受 schedule_planner 路径采样间距影响。
  std::size_t start_segment = 0;
  double start_t = 0.0;
  double nearest_sq = std::numeric_limits<double>::infinity();
  if (points.size() == 1) {
    nearest_sq = points[0].x * points[0].x + points[0].y * points[0].y;
  } else {
    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
      const double dx = points[i + 1].x - points[i].x;
      const double dy = points[i + 1].y - points[i].y;
      const double len_sq = dx * dx + dy * dy;
      double t = 0.0;
      if (len_sq > 1e-12) {
        // 原点投影到直线上的参数为 -dot(a, b-a)/|b-a|²；限制到 [0, 1] 后，
        // 得到原点在线段上的最近点。长度近似为零的重复路径点按端点处理。
        t = std::max(
          0.0, std::min(
            1.0,
            -(points[i].x * dx + points[i].y * dy) / len_sq));
      }
      const double x = points[i].x + t * dx;
      const double y = points[i].y + t * dy;
      const double dist_sq = x * x + y * y;
      if (dist_sq < nearest_sq) {
        nearest_sq = dist_sq;
        start_segment = i;
        start_t = t;
      }
    }
  }
  // 从最近投影点开始遍历原始路径点。距离上一个取点累计达到
  // path_point_interval_ 时，直接取当前路径点，不在线段上做精确插值。
  std::vector<Point> check_points;
  if (points.size() == 1) {
    if (std::sqrt(nearest_sq) <= path_stop_distance_) {
      check_points.push_back(points[0]);
    }
  } else {
    Point previous{
      points[start_segment].x + start_t * (points[start_segment + 1].x - points[start_segment].x),
      points[start_segment].y + start_t * (points[start_segment + 1].y - points[start_segment].y)};
    check_points.push_back(previous);
    double traversed = 0.0;
    double distance_since_last_sample = 0.0;
    for (std::size_t i = start_segment; i + 1 < points.size(); ++i) {
      const Point target = points[i + 1];
      const double dx = target.x - previous.x;
      const double dy = target.y - previous.y;
      const double length = std::hypot(dx, dy);
      if (length <= 1e-9) {
        previous = target;
        continue;
      }

      traversed += length;
      distance_since_last_sample += length;
      if (distance_since_last_sample >= path_point_interval_) {
        check_points.push_back(target);
        distance_since_last_sample = 0.0;
      }

      previous = target;
      if (traversed >= path_stop_distance_) {
        break;
      }
    }
  }
  return check_points;
}

void CollisionMonitor::recordPathStopTiming(
  const std::chrono::steady_clock::time_point & start_time)
{
  const auto finish_time = std::chrono::steady_clock::now();
  const auto elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    finish_time - start_time).count();
  constexpr std::int64_t slow_detection_threshold_ns = 10'000'000;
  if (elapsed_ns > slow_detection_threshold_ns) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), throttle_clock_, 5000, "Path-stop slow detection: elapsed=%.3f ms, threshold=10.000 ms",
      elapsed_ns / 1.0e6);
  }

  std::lock_guard<std::mutex> lock(path_stop_timing_mutex_);
  ++path_stop_timing_count_;
  path_stop_timing_total_ns_ += elapsed_ns;
  path_stop_timing_max_ns_ = std::max(path_stop_timing_max_ns_, elapsed_ns);

  const double window_seconds = std::chrono::duration<double>(
    finish_time - path_stop_timing_window_start_).count();
  if (window_seconds < 60.0) {
    return;
  }

  const double frequency = path_stop_timing_count_ / window_seconds;
  const double average_ms =
    path_stop_timing_total_ns_ / static_cast<double>(path_stop_timing_count_) / 1.0e6;
  const double maximum_ms = path_stop_timing_max_ns_ / 1.0e6;
  RCLCPP_INFO(
    get_logger(),
    "Path-stop timing: frequency=%.2f Hz, average=%.3f ms, max=%.3f ms, samples=%llu",
    frequency, average_ms, maximum_ms,
    static_cast<unsigned long long>(path_stop_timing_count_));

  path_stop_timing_count_ = 0;
  path_stop_timing_total_ns_ = 0;
  path_stop_timing_max_ns_ = 0;
  path_stop_timing_window_start_ = finish_time;
}

bool CollisionMonitor::hasObstacleInPathCorridor(
  const std::vector<Point> & collision_points,
  const std::vector<Point> & check_points) const
{
  // 直接使用 path_stop_radius_ 计算障碍点到路径取点的距离。每个障碍点最多计数一次，
  // 即使它同时落入多个相邻采样圆，也不会重复累加。
  int hits = 0;
  const double radius_squared = path_stop_radius_ * path_stop_radius_;
  for (const Point & obstacle : collision_points) {
    bool inside = false;
    for (const Point & path_point : check_points) {
      const double dx = obstacle.x - path_point.x;
      const double dy = obstacle.y - path_point.y;
      if (dx * dx + dy * dy <= radius_squared) {
        inside = true;
        break;
      }
    }
    // 与 PolygonStop 本身语义一致：max_points 是允许的噪点数量。
    if (inside && ++hits > path_stop_max_points_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), throttle_clock_, 1000,
        "Path-stop obstacle hit: hits=%d threshold=%d, source_points=%zu, "
        "obstacle_relative=(x=%.3fm, y=%.3fm), "
        "lookahead=%.2fm sample_interval=%.3fm path_stop_radius=%.3fm",
        hits, path_stop_max_points_, collision_points.size(),
        obstacle.x, obstacle.y,
        path_stop_distance_, path_point_interval_, path_stop_radius_);
      return true;
    }
  }
  return false;
}

bool CollisionMonitor::processStopSlowdown(
  const std::shared_ptr<Polygon> polygon,
  const std::vector<const std::vector<Point> *> & point_buffers,
  const Velocity & velocity,
  Action & robot_action) const
{
  const int max_points = polygon->getMaxPoints();
  // 提前终止（Early-Exit 剪枝）：一旦内部点数达到 max_points + 1，说明该动作必定触发，立即停止后续点遍历
  const int early_exit_threshold = max_points + 1;
  const int points_inside = polygon->getPointsInside(point_buffers, early_exit_threshold);
  if (points_inside > 0 && polygon->getActionType() == STOP) {
    RCLCPP_DEBUG(
      get_logger(),
      "[%s]: STOP check: points_inside=%d, max_points_threshold=%d -> triggered=%s",
      polygon->getName().c_str(), points_inside, max_points,
      (points_inside > max_points ? "YES" : "NO"));
  }

  if (points_inside > max_points) {
    if (polygon->getActionType() == STOP) {
      // Setting up zero velocity for STOP model
      robot_action.action_type = STOP;
      robot_action.req_vel.x = 0.0;
      robot_action.req_vel.y = 0.0;
      robot_action.req_vel.tw = 0.0;
      return true;
    } else {  // SLOWDOWN
      const Velocity safe_vel = velocity * polygon->getSlowdownRatio();
      // Check that currently calculated velocity is safer than
      // chosen for previous shapes one
      if (safe_vel < robot_action.req_vel) {
        robot_action.action_type = SLOWDOWN;
        robot_action.req_vel = safe_vel;
        return true;
      }
    }
  }

  return false;
}

bool CollisionMonitor::processStopSlowdown(
  const std::shared_ptr<Polygon> polygon,
  const std::vector<Point> & collision_points,
  const Velocity & velocity,
  Action & robot_action) const
{
  return processStopSlowdown(polygon, {&collision_points}, velocity, robot_action);
}

bool CollisionMonitor::processApproach(
  const std::shared_ptr<Polygon> polygon,
  const std::vector<Point> & collision_points,
  const Velocity & velocity,
  Action & robot_action) const
{
  polygon->updatePolygon();

  // Obtain time before a collision
  const double collision_time = polygon->getCollisionTime(collision_points, velocity);
  if (collision_time >= 0.0) {
    // If collision will occurr, reduce robot speed
    const double change_ratio = collision_time / polygon->getTimeBeforeCollision();
    const Velocity safe_vel = velocity * change_ratio;
    // Check that currently calculated velocity is safer than
    // chosen for previous shapes one
    if (safe_vel < robot_action.req_vel) {
      robot_action.action_type = APPROACH;
      robot_action.req_vel = safe_vel;
      return true;
    }
  }

  return false;
}

void CollisionMonitor::printAction(
  const Action & robot_action, const std::shared_ptr<Polygon> action_polygon) const
{
  if (robot_action.action_type == STOP) {
    if (action_polygon) {
      // 统计障碍点来源（雷达 vs 超声波，零拷贝分段统计）
      const int laser_pts_inside = action_polygon->getPointsInside(raw_points_buffer_);
      const int range_pts_inside = action_polygon->getPointsInside(
        std::vector<const std::vector<Point> *>{&range_slow_buffer_, &range_stop_only_buffer_});
      const int all_pts_inside = laser_pts_inside + range_pts_inside;

      std::string source_details;
      if (range_pts_inside > 0 && laser_pts_inside > 0) {
        source_details = " (laser=" + std::to_string(laser_pts_inside) +
          " pts, ultrasonic=" + std::to_string(range_pts_inside) + " pts)";
      } else if (range_pts_inside > 0) {
        source_details = " (caused by ultrasonic, pts=" + std::to_string(range_pts_inside) + ")";
      } else {
        source_details = " (caused by laser/depth, pts=" + std::to_string(laser_pts_inside) + ")";
      }

      RCLCPP_INFO(
        get_logger(),
        "Robot to stop due to %s polygon [points_inside=%d, max_points=%d]%s",
        action_polygon->getName().c_str(), all_pts_inside, action_polygon->getMaxPoints(),
        source_details.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Robot to stop due to unknown/path-stop action");
    }
  } else if (robot_action.action_type == SLOWDOWN) {
    if (action_polygon) {
      const int laser_pts_inside = action_polygon->getPointsInside(raw_points_buffer_);
      const int range_pts_inside = action_polygon->getPointsInside(range_slow_buffer_);
      const int slow_pts_inside = laser_pts_inside + range_pts_inside;

      std::string source_details;
      if (range_pts_inside > 0 && laser_pts_inside > 0) {
        source_details = " (laser=" + std::to_string(laser_pts_inside) +
          " pts, ultrasonic=" + std::to_string(range_pts_inside) + " pts)";
      } else if (range_pts_inside > 0) {
        source_details = " (caused by ultrasonic, pts=" + std::to_string(range_pts_inside) + ")";
      } else {
        source_details = " (caused by laser/depth, pts=" + std::to_string(laser_pts_inside) + ")";
      }

      RCLCPP_INFO(
        get_logger(),
        "Robot to slowdown for %f percents due to %s polygon [points_inside=%d, max_points=%d]%s",
        action_polygon->getSlowdownRatio() * 100,
        action_polygon->getName().c_str(), slow_pts_inside, action_polygon->getMaxPoints(),
        source_details.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Robot to slowdown due to unknown polygon");
    }
  } else if (robot_action.action_type == APPROACH) {
    RCLCPP_INFO(
      get_logger(),
      "Robot to approach for %f seconds away from collision",
      action_polygon->getTimeBeforeCollision());
  } else {  // robot_action.action_type == DO_NOTHING
    RCLCPP_INFO(
      get_logger(),
      "Robot to continue normal operation");
  }
}

void CollisionMonitor::setPolygonPaddingCallback(
  const std::shared_ptr<nav_interfaces::srv::SetPolygonPadding::Request> request,
  std::shared_ptr<nav_interfaces::srv::SetPolygonPadding::Response> response)
{
  const bool all = request->polygon_name.empty();
  const double requested = static_cast<double>(request->padding);
  const bool to_default = std::isnan(requested) || requested < 0.0;

  std::string summary;
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    if (!all && polygon->getName() != request->polygon_name) {
      continue;
    }
    const double before = polygon->getPadding();
    polygon->setPadding(requested);
    const double after = polygon->getPadding();

    response->applied_names.push_back(polygon->getName());
    response->applied_paddings.push_back(static_cast<float>(after));
    summary += polygon->getName() + "=" + std::to_string(after) + " ";

    RCLCPP_INFO(
      get_logger(),
      "[%s]: padding %.3f -> %.3f m%s",
      polygon->getName().c_str(), before, after,
      to_default ? " (restored to default)" : "");
  }

  if (response->applied_names.empty()) {
    response->success = false;
    response->message = "no polygon named '" + request->polygon_name + "'";
    RCLCPP_WARN(get_logger(), "set_polygon_padding: %s", response->message.c_str());
    return;
  }

  response->success = true;
  response->message = summary;

  // 立刻重发一次多边形，rviz 上能马上看到新的检测范围
  publishPolygons();
}

void CollisionMonitor::applyNavigationPadding(NavigationMode mode)
{
  // footprint 表示真实车身/货物包络，padding 表示模式安全距离，两者不能互相替代。
  for (const auto & polygon : polygons_) {
    if (polygon->getName() != "PolygonStop") {
      polygon->setFrontPadding(polygon->getDefaultFrontPadding());
      polygon->setRearPadding(polygon->getDefaultRearPadding());
      continue;
    }
    if (mode == NavigationMode::AUTOMONOUS_NAVIGATION) {
      polygon->setLateralPaddingOnly(false);
      polygon->setPadding(automonous_navigation_padding_);
      if (speed_tiers_enabled_ && !front_padding_tiers_.empty()) {
        const double applied_front = current_is_reverse_ ? 0.0 : front_padding_tiers_[current_speed_tier_];
        const double applied_rear = current_is_reverse_ ? rear_padding_tiers_[current_speed_tier_] : 0.0;
        polygon->setFrontPadding(applied_front);
        polygon->setRearPadding(applied_rear);
      } else {
        polygon->setFrontPadding(0.0);
        polygon->setRearPadding(0.0);
      }
    } else if (mode == NavigationMode::SCHEDULE_EDGING) {
      polygon->setPadding(polygon->getDefaultPadding());
      polygon->setLateralPaddingOnly(true);
      polygon->setFrontPadding(0.0);
      polygon->setRearPadding(0.0);
    } else if (mode == NavigationMode::SCHEDULE_NAVIGATION) {
      polygon->setLateralPaddingOnly(false);
      polygon->setFrontPadding(0.0);
      polygon->setRearPadding(0.0);
    } else {
      polygon->setFrontPadding(0.0);
      polygon->setRearPadding(0.0);
    }
    break;
  }
}

const char * CollisionMonitor::navigationModeName(NavigationMode mode)
{
  switch (mode) {
    case NavigationMode::IDLE: return "IDLE";
    case NavigationMode::AUTOMONOUS_NAVIGATION: return "AUTOMONOUS_NAVIGATION";
    case NavigationMode::SCHEDULE_NAVIGATION: return "SCHEDULE_NAVIGATION";
    case NavigationMode::SCHEDULE_EDGING: return "SCHEDULE_EDGING";
  }
  return "UNKNOWN";
}

void CollisionMonitor::changeNavigationMode(NavigationMode mode, const char * reason)
{
  if (navigation_mode_ == mode) {
    return;
  }
  RCLCPP_INFO(
    get_logger(), "[FSM] mode %s -> %s reason=%s corridor=%d",
    navigationModeName(navigation_mode_), navigationModeName(mode), reason,
    mode == NavigationMode::SCHEDULE_EDGING && path_stop_enabled_);
  navigation_mode_ = mode;
}

// ── 自动导航模式停止区前向/后向速度分档自适应状态机（A8 规范及倒车扩充）──
// 1. 生效范围：仅在自动导航（AUTOMONOUS_NAVIGATION）模式下激活；空闲与调度模式前向/后向外扩置 0；
// 2. 选档来源：订阅底盘原始里程计 (/feedback/odom)，提取 twist.linear.x 及单调接收时刻；
// 3. 选档与方向规则：
//    - vx < -0.01 m/s：倒车方向，采用倒车速度绝对值 |vx| 匹配档位，停止区向后方外扩；
//    - vx >= 0.0 m/s：前进方向，采用前向速度 vx 匹配档位，停止区向前方外扩；
//    - [-0.01, 0.0]：微小速度区间保持当前方向，避免零速噪声导致频繁前后切换；
//    - 超速则封顶最高档（不报错）；
// 4. 超时保护：里程计停发超过 speed_source_timeout (0.2s) 强制顶格至最高档，并限速报 WARN；
// 5. 转移防抖：升档与方向翻转立即生效；降档必须在目标低速档持续稳定满 tier_down_delay (0.5s) 才允许降档。
bool CollisionMonitor::updateSpeedTier(const rclcpp::Time & /*now*/)
{
  if (!speed_tiers_enabled_ || speed_tiers_.empty()) {
    return false;
  }

  std::shared_ptr<Polygon> stop_polygon = nullptr;
  for (const auto & polygon : polygons_) {
    if (polygon->getName() == "PolygonStop") {
      stop_polygon = polygon;
      break;
    }
  }

  // 仅在 AUTOMONOUS_NAVIGATION 模式下分档；空闲与调度模式不分档（前向与后向外扩均置 0）
  if (navigation_mode_ != NavigationMode::AUTOMONOUS_NAVIGATION) {
    if (stop_polygon) {
      if (stop_polygon->getFrontPadding() != 0.0) {
        stop_polygon->setFrontPadding(0.0);
      }
      if (stop_polygon->getRearPadding() != 0.0) {
        stop_polygon->setRearPadding(0.0);
      }
    }
    current_speed_tier_ = 0;
    current_is_reverse_ = false;
    pending_speed_tier_ = -1;
    tier_down_start_steady_ns_ = 0;
    return false;
  }

  const std::int64_t now_steady_ns = steadyNowNs();
  const std::int64_t last_rx_ns = odom_rx_steady_time_ns_.load(std::memory_order_relaxed);
  const double vx = odom_linear_x_.load(std::memory_order_relaxed);

  // 判定运动方向（倒车判定门限 -0.01 m/s，前进判定门限 0.01 m/s，死区防抖）
  bool target_is_reverse = current_is_reverse_;
  if (vx < -0.01) {
    target_is_reverse = true;
  } else if (vx > 0.01) {
    target_is_reverse = false;
  }
  const double speed_magnitude = target_is_reverse ? -vx : vx;

  std::size_t target_tier = 0;
  const std::size_t max_tier = speed_tiers_.size() - 1;

  if (last_rx_ns == 0 || (now_steady_ns - last_rx_ns) > speed_source_timeout_ns_) {
    // 里程计超时未更新 → 强制顶格最高档，确保制动安全距离充足
    target_tier = max_tier;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Collision monitor: odom timeout (%.2f s > %.2f s), falling back to max tier",
      last_rx_ns == 0 ? 0.0 : (now_steady_ns - last_rx_ns) * 1e-9, speed_source_timeout_);
  } else {
    // 第一个满足 speed_magnitude <= speed_tiers[i] 的档位；超速按最高档封顶
    target_tier = max_tier;
    for (std::size_t i = 0; i < speed_tiers_.size(); ++i) {
      if (speed_magnitude <= speed_tiers_[i]) {
        target_tier = i;
        break;
      }
    }
  }

  std::size_t new_effective_tier = current_speed_tier_;

  // 方向发生切换时：立即响应新运动方向并重新确认档位
  if (target_is_reverse != current_is_reverse_) {
    new_effective_tier = target_tier;
    pending_speed_tier_ = -1;
    tier_down_start_steady_ns_ = 0;
  } else if (target_tier > current_speed_tier_) {
    // 升档逻辑：安全优先，立即生效扩展停止区
    new_effective_tier = target_tier;
    pending_speed_tier_ = -1;
    tier_down_start_steady_ns_ = 0;
  } else if (target_tier < current_speed_tier_) {
    // 降档逻辑：需目标低速档持续维持 tier_down_delay (0.5s)，若期间车速回升则重新计时
    if (pending_speed_tier_ != static_cast<int>(target_tier)) {
      pending_speed_tier_ = static_cast<int>(target_tier);
      tier_down_start_steady_ns_ = now_steady_ns;
    } else if (now_steady_ns - tier_down_start_steady_ns_ >= tier_down_delay_ns_) {
      new_effective_tier = target_tier;
      pending_speed_tier_ = -1;
      tier_down_start_steady_ns_ = 0;
    }
  } else {
    pending_speed_tier_ = -1;
    tier_down_start_steady_ns_ = 0;
  }

  const double target_front = target_is_reverse ? 0.0 : front_padding_tiers_[new_effective_tier];
  const double target_rear = target_is_reverse ? rear_padding_tiers_[new_effective_tier] : 0.0;

  bool tier_changed = false;
  if (new_effective_tier != current_speed_tier_ ||
    target_is_reverse != current_is_reverse_ ||
    (stop_polygon && (stop_polygon->getFrontPadding() != target_front ||
                      stop_polygon->getRearPadding() != target_rear)))
  {
    tier_changed = (new_effective_tier != current_speed_tier_ || target_is_reverse != current_is_reverse_);
    current_speed_tier_ = new_effective_tier;
    current_is_reverse_ = target_is_reverse;
    if (stop_polygon) {
      stop_polygon->setFrontPadding(target_front);
      stop_polygon->setRearPadding(target_rear);
    }
    RCLCPP_DEBUG(
      get_logger(), "Stop zone tier %zu (%s): front=%.2f m rear=%.2f m (vx %.2f m/s)",
      current_speed_tier_, current_is_reverse_ ? "reverse" : "forward",
      target_front, target_rear, vx);
  }

  return tier_changed;
}

// ── 硬停通知发布（A6 门控规范）──
// 1. 平滑器收到后立即输出零速并清除内部累积速度，制动不受加速度平滑限制；
// 2. 单边沿触发：进入 STOP 的上升沿瞬间发布 1 次，持续 STOP 期间不重复发；
// 3. A6 规范：仅在转发新鲜速度时（非 IDLE 且有上游有效速度）发布，
//    CM 空闲时不发布，避免干扰自动回充等直发底盘的业务指令。
void CollisionMonitor::publishHardStopOnStopEdge(const char * reason)
{
  if (robot_action_prev_.action_type == STOP || !is_forwarding_) {
    return;
  }
  if (hard_stop_pub_ && hard_stop_pub_->is_activated()) {
    hard_stop_pub_->publish(std::make_unique<std_msgs::msg::Empty>());
    RCLCPP_WARN(get_logger(), "Hard stop published: %s", reason);
  }
}

// ── 转发门控零速发布（A4、A6 规范）──
// 仅在当前处于活跃转发速度状态时发布零速并置位本周期已发标志；
// 空闲期不发零速，防止冲刷自动回充对接指令。
bool CollisionMonitor::publishZeroIfForwarding()
{
  if (!is_forwarding_) {
    return false;
  }
  if (cmd_vel_out_pub_ && cmd_vel_out_pub_->is_activated()) {
    cmd_vel_out_pub_->publish(std::make_unique<geometry_msgs::msg::Twist>());
    zero_vel_published_this_cycle_ = true;
    return true;
  }
  return false;
}

void CollisionMonitor::enterIdle(const char * reason)
{
  changeNavigationMode(NavigationMode::IDLE, reason);
  if (cmd_vel_out_pub_ && cmd_vel_out_pub_->is_activated()) {
    cmd_vel_out_pub_->publish(std::make_unique<geometry_msgs::msg::Twist>());
    RCLCPP_INFO(get_logger(), "[FSM] idle zero published: reason=%s", reason);
  }
}

}  // namespace nav2_collision_monitor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_collision_monitor::CollisionMonitor)
