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

CollisionMonitor::CollisionMonitor(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("collision_monitor", "", options),
  process_active_(false), robot_action_prev_{DO_NOTHING, {-1.0, -1.0, -1.0}},
  stop_stamp_{0, 0, get_clock()->get_clock_type()}, stop_pub_timeout_(1.0, 0.0),
  stop_hold_stamp_{0, 0, get_clock()->get_clock_type()}
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

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionMonitor::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Activating lifecycle publishers
  cmd_vel_out_pub_->on_activate();
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
  logged_velocity_state_ = -1;
  stop_hold_active_ = false;
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
  stop_hold_active_ = false;

  // Deactivating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    polygon->deactivate();
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

void CollisionMonitor::controlLoopCallback()
{
  if (!process_active_) {
    return;
  }
  const auto control_loop_start = std::chrono::steady_clock::now();
  const auto & input = readControlInput();
  const bool input_updated = hasUpdatedInput(input);
  const rclcpp::Time now = this->now();
  // 传感器故障会发布停车，本周期不再处理导航模式和速度。
  if (checkSensorTimeout(now)) {
    updateNavigationMode(input, now);
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
  // 单调时钟检测任一观测源未收到或停止到达的数据；消息时间戳检测延迟到达的旧数据。
  // 从未收到的传感器从 source_start_time_ns_ 开始计时。所有启用的观测源
  // （scan / range / pointcloud）超时都会触发停车，不再只监测激光雷达。
  const auto steady_now = steadyNowNs();
  std::string stale_sensor;
  for (const auto & source : sources_) {
    const auto last = source->getLastReceivedSteadyNs();
    const auto since = std::max(last, source_start_time_ns_);
    const auto stamp = source->getLastSourceStampNs();
    if (steady_now - since >= sensor_fault_timeout_ns_ ||
      (last != 0 && now.nanoseconds() - stamp >= sensor_fault_timeout_ns_))
    {
      stale_sensor = source->getName();
      break;
    }
  }
  if (!stale_sensor.empty()) {
    // 任一必需观测源超时时，持续发布零速和 STOP 状态。
    if (!sensor_fault_active_) {
      RCLCPP_ERROR(get_logger(), "%s timed out; stopping", stale_sensor.c_str());
    }
    sensor_fault_active_ = true;
    publishHardStopOnStopEdge("scan timeout");
    auto zero = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_vel_out_pub_->publish(std::move(zero));
    publishObstacleStatus({STOP, {0.0, 0.0, 0.0}}, nullptr);
    robot_action_prev_ = {STOP, {0.0, 0.0, 0.0}};
    return false;
  }
  if (sensor_fault_active_) {
    RCLCPP_INFO(get_logger(), "All required sensors recovered");
    sensor_fault_active_ = false;
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
    if (!robot_action_prev_.req_vel.isZero()) {
      // Robot just stopped: saving stop timestamp and continue
      stop_stamp_ = this->now();
    } else if (this->now() - stop_stamp_ > stop_pub_timeout_) {
      // More than stop_pub_timeout_ passed after robot has been stopped.
      // Cease publishing output cmd_vel.
      return;
    }
  }

  std::unique_ptr<geometry_msgs::msg::Twist> cmd_vel_out_msg =
    std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel_out_msg->linear.x = robot_action.req_vel.x;
  cmd_vel_out_msg->linear.y = robot_action.req_vel.y;
  cmd_vel_out_msg->angular.z = robot_action.req_vel.tw;
  // linear.z, angular.x and angular.y will remain 0.0

  cmd_vel_out_pub_->publish(std::move(cmd_vel_out_msg));
}

void CollisionMonitor::publishObstacleStatus(
  const Action & robot_action, const std::shared_ptr<Polygon> action_polygon) const
{
  auto msg = std::make_unique<nav_interfaces::msg::ObstacleMonitorStatus>();
  msg->header.stamp = this->now();
  msg->header.frame_id = base_frame_id_;

  switch (robot_action.action_type) {
    case STOP:
      msg->status = nav_interfaces::msg::ObstacleMonitorStatus::STOP;
      msg->slowdown_ratio = 0.0f;
      break;
    case SLOWDOWN:
      msg->status = nav_interfaces::msg::ObstacleMonitorStatus::SLOWDOWN;
      msg->slowdown_ratio = static_cast<float>(action_polygon->getSlowdownRatio());
      break;
    case APPROACH:
      // APPROACH 也是"降速不停止"的语义，并入 SLOWDOWN 三态模型
      msg->status = nav_interfaces::msg::ObstacleMonitorStatus::SLOWDOWN;
      msg->slowdown_ratio = 0.0f;
      break;
    default:  // DO_NOTHING
      msg->status = nav_interfaces::msg::ObstacleMonitorStatus::FREE;
      msg->slowdown_ratio = 0.0f;
      break;
  }
  msg->zone_name = action_polygon ? action_polygon->getName() : "";

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
    node, "stop_pub_timeout", rclcpp::ParameterValue(1.0));
  stop_pub_timeout_ =
    rclcpp::Duration::from_seconds(get_parameter("stop_pub_timeout").as_double());

  // 停车保持：STOP 触发后至少保持该时长再允许恢复；<=0 关闭。
  nav2_util::declare_parameter_if_not_declared(
    node, "stop_hold_time", rclcpp::ParameterValue(2.0));
  stop_hold_time_ =
    rclcpp::Duration::from_seconds(get_parameter("stop_hold_time").as_double());
  if (stop_hold_time_.seconds() < 0.0) {
    RCLCPP_ERROR(get_logger(), "stop_hold_time must be non-negative");
    return false;
  }

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
  slowdown_points_buffer_.clear();
  all_stop_points_buffer_.clear();

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

  // SLOWDOWN 多边形消费点：激光雷达点 + 允许减速的超声波点
  slowdown_points_buffer_.reserve(
    raw_points_buffer_.size() + range_slow_buffer_.size());
  slowdown_points_buffer_.insert(
    slowdown_points_buffer_.end(),
    raw_points_buffer_.begin(), raw_points_buffer_.end());
  slowdown_points_buffer_.insert(
    slowdown_points_buffer_.end(),
    range_slow_buffer_.begin(), range_slow_buffer_.end());

  // STOP 多边形消费点：所有点（通用雷达点 + 允许减速的超声波点 + 仅停障的超声波点）
  all_stop_points_buffer_.reserve(
    slowdown_points_buffer_.size() + range_stop_only_buffer_.size());
  all_stop_points_buffer_.insert(
    all_stop_points_buffer_.end(),
    slowdown_points_buffer_.begin(), slowdown_points_buffer_.end());
  all_stop_points_buffer_.insert(
    all_stop_points_buffer_.end(),
    range_stop_only_buffer_.begin(), range_stop_only_buffer_.end());

  std::vector<Point> & collision_points = raw_points_buffer_;
  std::vector<Point> & slowdown_points = slowdown_points_buffer_;
  std::vector<Point> & all_stop_points = all_stop_points_buffer_;

  for (const auto & polygon : polygons_) {
    polygon->updatePolygon();
    if (polygon->missingFootprint()) {
      publishHardStopOnStopEdge("footprint missing");
      cmd_vel_out_pub_->publish(std::make_unique<geometry_msgs::msg::Twist>());
      publishObstacleStatus({STOP, {0.0, 0.0, 0.0}}, nullptr);
      robot_action_prev_ = {STOP, {0.0, 0.0, 0.0}};
      return;
    }
  }

  // By default - there is no action
  Action robot_action{DO_NOTHING, cmd_vel_in};
  // Polygon causing robot action (if any)
  std::shared_ptr<Polygon> action_polygon;
  bool path_stop = false;

  for (std::shared_ptr<Polygon> polygon : polygons_) {
    if (robot_action.action_type == STOP) {
      // If robot already should stop, do nothing
      break;
    }

    const ActionType at = polygon->getActionType();
    // IDLE 只保留近身静态停车区，跳过 SLOWDOWN、APPROACH 和后面的路径走廊。
    if (idle_polygon_only && at != STOP) {
      continue;
    }
    // 正常导航启用 APPROACH 速度轨迹预测，可通过 approach_enabled 显式关闭。
    if (!approach_enabled_ && at == APPROACH) {
      continue;
    }
    if (at == STOP) {
      // STOP 多边形同时使用雷达与超声波点（若连续3帧检测到则触发停车）
      if (processStopSlowdown(polygon, all_stop_points, cmd_vel_in, robot_action)) {
        action_polygon = polygon;
      }
    } else if (at == SLOWDOWN) {
      // SLOWDOWN 多边形使用雷达点与可配置进入减速的超声波点
      if (processStopSlowdown(polygon, slowdown_points, cmd_vel_in, robot_action)) {
        action_polygon = polygon;
      }
    } else if (at == APPROACH) {
      const auto approach_start = std::chrono::steady_clock::now();
      // APPROACH 速度轨迹预测仅使用雷达点（超声波不做减速监测）
      if (processApproach(polygon, collision_points, cmd_vel_in, robot_action)) {
        action_polygon = polygon;
      }
      if (performance_timing_enabled_) {
        recordPerformanceDuration(approach_samples_ms_, approach_start);
      }
    }
  }

  // 路径走廊检测仅用于调度贴边模式。
  if (path_stop_enabled_ && !idle_polygon_only &&
    navigation_mode_ == NavigationMode::SCHEDULE_EDGING)
  {
    const auto path_stop_start = std::chrono::steady_clock::now();
    // The route-ahead detector has STOP priority over all polygon actions.
    if (hasObstacleOnPath(collision_points)) {
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

  // 停车保持（stop_hold_time_）：一旦进入 STOP 就至少保持该时长；期间即使障碍消失也维持
  // STOP，到时后再按最新检测结果恢复，避免抖动/漏检造成反复启停。该逻辑对 auto（下发零速）
  // 与 schedule（上报 STOP 状态）两条链路一致生效，IDLE 的车身检测不受影响。
  if (!idle_polygon_only && stop_hold_time_.seconds() > 0.0) {
    if (robot_action.action_type == STOP) {
      if (!stop_hold_active_) {
        stop_hold_active_ = true;
        stop_hold_stamp_ = curr_time;
      }
    } else if (stop_hold_active_) {
      if (curr_time - stop_hold_stamp_ < stop_hold_time_) {
        robot_action = {STOP, {0.0, 0.0, 0.0}};
      } else {
        stop_hold_active_ = false;
      }
    }
  }

  const bool action_changed = robot_action.action_type != robot_action_prev_.action_type;
  if (robot_action.action_type == STOP) {
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
  publishObstacleStatus(robot_action, action_polygon);

  // Publish polygons for better visualization
  publishPolygons();

  // APPROACH 类型不变时，传感器更新仍可改变安全速度（包括降为零）。
  // 必须发布新的安全速度，不能只按动作类型判断变化。
  const bool velocity_changed =
    robot_action.req_vel.x != robot_action_prev_.req_vel.x ||
    robot_action.req_vel.y != robot_action_prev_.req_vel.y ||
    robot_action.req_vel.tw != robot_action_prev_.req_vel.tw;
  if (publish_cmd_vel && (command_updated || action_changed || velocity_changed)) {
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

void CollisionMonitor::publishPathStopMarkers(
  const std::vector<Point> & check_points) const
{
  auto markers = std::make_unique<visualization_msgs::msg::MarkerArray>();

  // Clear markers left by a previously longer path before publishing the current set.
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  markers->markers.push_back(clear_marker);

  const rclcpp::Time stamp = this->now();
  constexpr std::size_t circle_segments = 32;
  for (std::size_t i = 0; i < check_points.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_id_;
    marker.header.stamp = stamp;
    marker.ns = "path_stop_circles";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.color.r = 1.0f;
    marker.color.g = 0.1f;
    marker.color.b = 0.0f;
    marker.color.a = 0.8f;
    marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    marker.points.reserve(circle_segments + 1);
    for (std::size_t segment = 0; segment <= circle_segments; ++segment) {
      const double angle = 2.0 * M_PI * segment / circle_segments;
      geometry_msgs::msg::Point point;
      point.x = check_points[i].x + path_stop_radius_ * std::cos(angle);
      point.y = check_points[i].y + path_stop_radius_ * std::sin(angle);
      point.z = 0.02;
      marker.points.push_back(point);
    }
    markers->markers.push_back(std::move(marker));
  }

  path_stop_marker_pub_->publish(std::move(markers));
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
  const std::vector<Point> & collision_points,
  const Velocity & velocity,
  Action & robot_action) const
{
  const int points_inside = polygon->getPointsInside(collision_points);
  const int max_points = polygon->getMaxPoints();
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
      // 统计障碍点来源（雷达 vs 超声波）
      const int laser_pts_inside = action_polygon->getPointsInside(raw_points_buffer_);
      const int all_pts_inside = action_polygon->getPointsInside(all_stop_points_buffer_);
      const int range_pts_inside = all_pts_inside - laser_pts_inside;

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
      const int slow_pts_inside = action_polygon->getPointsInside(slowdown_points_buffer_);
      const int range_pts_inside = slow_pts_inside - laser_pts_inside;

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

void CollisionMonitor::publishUltrasonicPointCloud(
  const rclcpp::Time & curr_time,
  const std::vector<Point> & points) const
{
  if (!ultrasonic_points_pub_ || ultrasonic_points_pub_->get_subscription_count() == 0) {
    return;
  }

  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  msg->header.stamp = curr_time;
  msg->header.frame_id = base_frame_id_;

  sensor_msgs::PointCloud2Modifier modifier(*msg);
  modifier.setPointCloud2Fields(
    3,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

  for (const auto & p : points) {
    *iter_x = static_cast<float>(p.x);
    *iter_y = static_cast<float>(p.y);
    *iter_z = 0.0f;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  ultrasonic_points_pub_->publish(std::move(msg));
}

void CollisionMonitor::publishPolygons() const
{
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    polygon->publish();
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
      continue;
    }
    if (mode == NavigationMode::AUTOMONOUS_NAVIGATION) {
      polygon->setLateralPaddingOnly(false);
      polygon->setPadding(automonous_navigation_padding_);
    } else if (mode == NavigationMode::SCHEDULE_EDGING) {
      polygon->setPadding(polygon->getDefaultPadding());
      polygon->setLateralPaddingOnly(true);
    } else if (mode == NavigationMode::SCHEDULE_NAVIGATION) {
      polygon->setLateralPaddingOnly(false);
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

void CollisionMonitor::publishHardStopOnStopEdge(const char * reason)
{
  // 平滑器收到后立即输出零速并清除内部速度，停车不受其减速度限制；
  // STOP 持续期间只发一次，解除后再次进入 STOP 会重新发布。
  if (robot_action_prev_.action_type == STOP) {
    return;
  }
  if (hard_stop_pub_ && hard_stop_pub_->is_activated()) {
    hard_stop_pub_->publish(std::make_unique<std_msgs::msg::Empty>());
    RCLCPP_WARN(get_logger(), "Hard stop published: %s", reason);
  }
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
