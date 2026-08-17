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
#include <cmath>
#include <exception>
#include <string>
#include <utility>
#include <functional>

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

  // Activating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    polygon->activate();
  }

  // Since polygons are being published when cmd_vel_in appears,
  // we need to publish polygons first time to display them at startup
  publishPolygons();

  // Activating main worker
  process_active_ = true;

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
  cmd_vel_out_pub_.reset();
  obstacle_status_pub_.reset();
  set_padding_srv_.reset();

  polygons_.clear();
  sources_.clear();

  tf_listener_.reset();
  tf_buffer_.reset();

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
        polygons_.push_back(
          std::make_shared<Circle>(
            node, polygon_name, tf_buffer_, base_frame_id, transform_tolerance));
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

  if (robot_action.action_type != robot_action_prev_.action_type) {
    // Report changed robot behavior
    printAction(robot_action, action_polygon);
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
