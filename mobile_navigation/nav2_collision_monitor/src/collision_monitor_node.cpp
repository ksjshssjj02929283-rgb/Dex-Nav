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
  idle_keepalive_period_(0, 0), idle_keepalive_rate_(0.0),
  last_cmd_vel_in_stamp_{0, 0, get_clock()->get_clock_type()}
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

  // Idle keep-alive rate: how often obstacle_status is refreshed while no cmd_vel_in flows.
  // <= 0 disables the timer, restoring the "publish only on cmd_vel_in" behaviour.
  nav2_util::declare_parameter_if_not_declared(
    node, "obstacle_status_idle_rate", rclcpp::ParameterValue(2.0));
  idle_keepalive_rate_ = get_parameter("obstacle_status_idle_rate").as_double();
  if (idle_keepalive_rate_ > 0.0) {
    idle_keepalive_period_ = rclcpp::Duration::from_seconds(1.0 / idle_keepalive_rate_);
  }

  cmd_vel_in_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_in_topic, 1,
    std::bind(&CollisionMonitor::cmdVelInCallback, this, std::placeholders::_1));
  cmd_vel_out_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    cmd_vel_out_topic, 1);
  obstacle_status_pub_ = this->create_publisher<nav_interfaces::msg::ObstacleMonitorStatus>(
    obstacle_status_topic, 1);

  nav2_util::declare_parameter_if_not_declared(
    node, "PolygonStop.path_stop_enabled", rclcpp::ParameterValue(false));
  path_stop_enabled_ = get_parameter("PolygonStop.path_stop_enabled").as_bool();
  nav2_util::declare_parameter_if_not_declared(
    node, "PolygonStop.tracking_path_topic", rclcpp::ParameterValue("/tracking_path"));
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
    tracking_path_sub_ = this->create_subscription<nav_interfaces::msg::TrackingPath>(
      tracking_path_topic_, rclcpp::QoS(1),
      std::bind(&CollisionMonitor::trackingPathCallback, this, std::placeholders::_1));
    if (path_stop_visualize_) {
      path_stop_marker_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
        path_stop_marker_topic_, rclcpp::QoS(1));
    }
  }

  // 动态膨胀设置服务（本仓库相对上游 Nav2 的改动）
  set_padding_srv_ = this->create_service<nav_interfaces::srv::SetPolygonPadding>(
    "~/set_polygon_padding",
    std::bind(
      &CollisionMonitor::setPolygonPaddingCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionMonitor::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Activating lifecycle publishers
  cmd_vel_out_pub_->on_activate();
  obstacle_status_pub_->on_activate();
  if (path_stop_marker_pub_) {
    path_stop_marker_pub_->on_activate();
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

  // Idle keep-alive timer: created on activate / destroyed on deactivate so it never runs
  // outside the active state. Treat "no cmd_vel_in yet" as stale, so obstacle_status starts
  // flowing right away instead of waiting for the first velocity command.
  if (idle_keepalive_rate_ > 0.0) {
    last_cmd_vel_in_stamp_ = this->now() - idle_keepalive_period_;
    idle_keepalive_timer_ = this->create_wall_timer(
      idle_keepalive_period_.to_chrono<std::chrono::nanoseconds>(),
      std::bind(&CollisionMonitor::idleKeepaliveCallback, this));
    RCLCPP_INFO(
      get_logger(), "Obstacle status idle keep-alive enabled at %.2f Hz", idle_keepalive_rate_);
  }

  // Creating bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionMonitor::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // Stop the idle keep-alive before the worker goes down
  idle_keepalive_timer_.reset();

  // Deactivating main worker
  process_active_ = false;

  // Reset action type to default after worker deactivating
  robot_action_prev_ = {DO_NOTHING, {-1.0, -1.0, -1.0}};

  // Deactivating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    polygon->deactivate();
  }

  // Deactivating lifecycle publishers
  cmd_vel_out_pub_->on_deactivate();
  obstacle_status_pub_->on_deactivate();
  if (path_stop_marker_pub_) {
    path_stop_marker_pub_->on_deactivate();
  }

  // Destroying bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionMonitor::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  idle_keepalive_timer_.reset();
  cmd_vel_in_sub_.reset();
  tracking_path_sub_.reset();
  cmd_vel_out_pub_.reset();
  obstacle_status_pub_.reset();
  path_stop_marker_pub_.reset();
  set_padding_srv_.reset();

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

  last_cmd_vel_in_stamp_ = this->now();
  process({msg->linear.x, msg->linear.y, msg->angular.z});
}

void CollisionMonitor::idleKeepaliveCallback()
{
  if (!process_active_) {
    return;
  }

  // Skip while cmd_vel_in is flowing: the main path already refreshes obstacle_status every
  // command, so this must not duplicate that work nor perturb the gated cmd_vel output.
  if (this->now() - last_cmd_vel_in_stamp_ < idle_keepalive_period_) {
    return;
  }

  // Evaluate at zero velocity and suppress the cmd_vel publish: a monitor gates velocities,
  // it must never originate them.
  process({0.0, 0.0, 0.0}, false);
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
  nav2_util::declare_parameter_if_not_declared(
    node, "base_shift_correction", rclcpp::ParameterValue(true));
  const bool base_shift_correction =
    get_parameter("base_shift_correction").as_bool();

  nav2_util::declare_parameter_if_not_declared(
    node, "stop_pub_timeout", rclcpp::ParameterValue(1.0));
  stop_pub_timeout_ =
    rclcpp::Duration::from_seconds(get_parameter("stop_pub_timeout").as_double());

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

void CollisionMonitor::process(const Velocity & cmd_vel_in, bool publish_cmd_vel)
{
  // Current timestamp for all inner routines prolongation
  rclcpp::Time curr_time = this->now();

  // Do nothing if main worker in non-active state
  if (!process_active_) {
    return;
  }

  // Points array collected from different data sources in a robot base frame
  std::vector<Point> collision_points;

  // Fill collision_points array from different data sources
  for (std::shared_ptr<Source> source : sources_) {
    source->getData(curr_time, collision_points);
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
    if (at == STOP || at == SLOWDOWN) {
      // Process STOP/SLOWDOWN for the selected polygon
      if (processStopSlowdown(polygon, collision_points, cmd_vel_in, robot_action)) {
        action_polygon = polygon;
      }
    } else if (at == APPROACH) {
      // Process APPROACH for the selected polygon
      if (processApproach(polygon, collision_points, cmd_vel_in, robot_action)) {
        action_polygon = polygon;
      }
    }
  }

  if (path_stop_enabled_) {
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
  }

  if (robot_action.action_type != robot_action_prev_.action_type) {
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

  if (publish_cmd_vel) {
    // Publish required robot velocity
    publishVelocity(robot_action);
    robot_action_prev_ = robot_action;
  } else {
    // Idle keep-alive: carry over the action type only, so a state change that happens while
    // the robot is idle is still detected/printed once. req_vel belongs to the cmd_vel path -
    // overwriting it here would make publishVelocity() miss the "robot just stopped" edge on
    // the next real command and skip forwarding that stop.
    robot_action_prev_.action_type = robot_action.action_type;
  }
}

void CollisionMonitor::trackingPathCallback(
  nav_interfaces::msg::TrackingPath::ConstSharedPtr msg)
{
  // padding 接近 0 表示贴边模式：PolygonStop 使用默认安全余量，但只沿车体左右
  // （base_link Y 方向）膨胀，前后保持基础轮廓。NaN/非零值均为正常模式，
  // 具体 padding 继续由既有 set_polygon_padding service 链路负责设置。
  constexpr float kEpsilon = 1e-6F;
  const bool edging_mode = std::fabs(msg->padding) < kEpsilon;
  for (const std::shared_ptr<Polygon> & polygon : polygons_) {
    if (polygon->getName() != "PolygonStop") {
      continue;
    }
    const bool mode_changed = polygon->isLateralPaddingOnly() != edging_mode;
    if (edging_mode) {
      // schedule_planner 在发布路径前会把 0 写入 service；贴边模式的左右余量
      // 应使用配置的 default_padding，而不是该作为模式标志的 0。
      polygon->setPadding(polygon->getDefaultPadding());
    }
    polygon->setLateralPaddingOnly(edging_mode);
    if (mode_changed) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), throttle_clock_, 1000,
        "PolygonStop switched to %s mode (padding: front/rear=%s, lateral=%.3f m)",
        edging_mode ? "edging" : "normal", edging_mode ? "0" : "uniform",
        polygon->getPadding());
      publishPolygons();
    }
    break;
  }

  std::lock_guard<std::mutex> lock(tracking_path_mutex_);
  tracking_path_ = std::move(msg);
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
    RCLCPP_WARN(
      get_logger(), "Path-stop slow detection: elapsed=%.3f ms, threshold=10.000 ms",
      elapsed_ns / 1.0e6);
  }

  std::lock_guard<std::mutex> lock(path_stop_timing_mutex_);
  ++path_stop_timing_count_;
  path_stop_timing_total_ns_ += elapsed_ns;
  path_stop_timing_max_ns_ = std::max(path_stop_timing_max_ns_, elapsed_ns);

  const double window_seconds = std::chrono::duration<double>(
    finish_time - path_stop_timing_window_start_).count();
  if (window_seconds < 5.0) {
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
  if (polygon->getPointsInside(collision_points) > polygon->getMaxPoints()) {
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
    RCLCPP_INFO(
      get_logger(),
      "Robot to stop due to %s polygon",
      action_polygon->getName().c_str());
  } else if (robot_action.action_type == SLOWDOWN) {
    RCLCPP_INFO(
      get_logger(),
      "Robot to slowdown for %f percents due to %s polygon",
      action_polygon->getSlowdownRatio() * 100,
      action_polygon->getName().c_str());
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

}  // namespace nav2_collision_monitor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_collision_monitor::CollisionMonitor)
