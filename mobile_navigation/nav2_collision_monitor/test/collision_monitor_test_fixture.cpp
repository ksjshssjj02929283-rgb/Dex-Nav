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

#include "collision_monitor_test_fixture.hpp"

Tester::Tester()
{
  cm_ = std::make_shared<CollisionMonitorWrapper>();

  footprint_pub_ = cm_->create_publisher<geometry_msgs::msg::PolygonStamped>(
    FOOTPRINT_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  scan_pub_ = cm_->create_publisher<sensor_msgs::msg::LaserScan>(
    SCAN_NAME, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pointcloud_pub_ = cm_->create_publisher<sensor_msgs::msg::PointCloud2>(
    POINTCLOUD_NAME, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  range_pub_ = cm_->create_publisher<sensor_msgs::msg::Range>(
    RANGE_NAME, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  cmd_vel_in_pub_ = cm_->create_publisher<geometry_msgs::msg::Twist>(
    CMD_VEL_IN_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  schedule_cmd_vel_pub_ = cm_->create_publisher<geometry_msgs::msg::Twist>(
    SCHEDULE_CMD_VEL_TOPIC, rclcpp::QoS(1));
  cmd_vel_out_sub_ = cm_->create_subscription<geometry_msgs::msg::Twist>(
    CMD_VEL_OUT_TOPIC, rclcpp::SystemDefaultsQoS(),
    std::bind(&Tester::cmdVelOutCallback, this, std::placeholders::_1));
  obstacle_status_sub_ = cm_->create_subscription<nav_interfaces::msg::ObstacleMonitorStatus>(
    OBSTACLE_STATUS_TOPIC, rclcpp::QoS(1),
    std::bind(&Tester::obstacleStatusCallback, this, std::placeholders::_1));
}

Tester::~Tester()
{
  footprint_pub_.reset();

  scan_pub_.reset();
  pointcloud_pub_.reset();
  range_pub_.reset();

  cmd_vel_in_pub_.reset();
  schedule_cmd_vel_pub_.reset();
  cmd_vel_out_sub_.reset();
  obstacle_status_sub_.reset();

  cm_.reset();
}

void Tester::setCommonParameters()
{
  cm_->declare_parameter(
    "cmd_vel_in_topic", rclcpp::ParameterValue(CMD_VEL_IN_TOPIC));
  cm_->set_parameter(
    rclcpp::Parameter("cmd_vel_in_topic", CMD_VEL_IN_TOPIC));
  cm_->declare_parameter(
    "cmd_vel_out_topic", rclcpp::ParameterValue(CMD_VEL_OUT_TOPIC));
  cm_->set_parameter(
    rclcpp::Parameter("cmd_vel_out_topic", CMD_VEL_OUT_TOPIC));
  cm_->declare_parameter(
    "schedule_cmd_vel_topic", rclcpp::ParameterValue(SCHEDULE_CMD_VEL_TOPIC));
  cm_->set_parameter(
    rclcpp::Parameter("schedule_cmd_vel_topic", SCHEDULE_CMD_VEL_TOPIC));
  cm_->declare_parameter(
    "obstacle_status_topic", rclcpp::ParameterValue(OBSTACLE_STATUS_TOPIC));

  cm_->declare_parameter(
    "base_frame_id", rclcpp::ParameterValue(BASE_FRAME_ID));
  cm_->set_parameter(
    rclcpp::Parameter("base_frame_id", BASE_FRAME_ID));
  cm_->declare_parameter(
    "odom_frame_id", rclcpp::ParameterValue(ODOM_FRAME_ID));
  cm_->set_parameter(
    rclcpp::Parameter("odom_frame_id", ODOM_FRAME_ID));

  cm_->declare_parameter(
    "transform_tolerance", rclcpp::ParameterValue(TRANSFORM_TOLERANCE));
  cm_->set_parameter(
    rclcpp::Parameter("transform_tolerance", TRANSFORM_TOLERANCE));
  cm_->declare_parameter(
    "source_timeout", rclcpp::ParameterValue(SOURCE_TIMEOUT));
  cm_->set_parameter(
    rclcpp::Parameter("source_timeout", SOURCE_TIMEOUT));

  cm_->declare_parameter(
    "stop_pub_timeout", rclcpp::ParameterValue(STOP_PUB_TIMEOUT));
  cm_->set_parameter(
    rclcpp::Parameter("stop_pub_timeout", STOP_PUB_TIMEOUT));
}

void Tester::addPolygon(
  const std::string & polygon_name, const PolygonType type,
  const double size, const std::string & at)
{
  if (type == POLYGON) {
    cm_->declare_parameter(
      polygon_name + ".type", rclcpp::ParameterValue("polygon"));
    cm_->set_parameter(
      rclcpp::Parameter(polygon_name + ".type", "polygon"));

    if (at != "approach") {
      const std::vector<double> points {
        size, size, size, -size, -size, -size, -size, size};
      cm_->declare_parameter(
        polygon_name + ".points", rclcpp::ParameterValue(points));
      cm_->set_parameter(
        rclcpp::Parameter(polygon_name + ".points", points));
    } else {  // at == "approach"
      cm_->declare_parameter(
        polygon_name + ".footprint_topic", rclcpp::ParameterValue(FOOTPRINT_TOPIC));
      cm_->set_parameter(
        rclcpp::Parameter(polygon_name + ".footprint_topic", FOOTPRINT_TOPIC));
    }
  } else if (type == CIRCLE) {
    cm_->declare_parameter(
      polygon_name + ".type", rclcpp::ParameterValue("circle"));
    cm_->set_parameter(
      rclcpp::Parameter(polygon_name + ".type", "circle"));

    cm_->declare_parameter(
      polygon_name + ".radius", rclcpp::ParameterValue(size));
    cm_->set_parameter(
      rclcpp::Parameter(polygon_name + ".radius", size));
  } else {  // type == POLYGON_UNKNOWN
    cm_->declare_parameter(
      polygon_name + ".type", rclcpp::ParameterValue("unknown"));
    cm_->set_parameter(
      rclcpp::Parameter(polygon_name + ".type", "unknown"));
  }

  cm_->declare_parameter(
    polygon_name + ".action_type", rclcpp::ParameterValue(at));
  cm_->set_parameter(
    rclcpp::Parameter(polygon_name + ".action_type", at));

  cm_->declare_parameter(
    polygon_name + ".max_points", rclcpp::ParameterValue(MAX_POINTS));
  cm_->set_parameter(
    rclcpp::Parameter(polygon_name + ".max_points", MAX_POINTS));

  cm_->declare_parameter(
    polygon_name + ".slowdown_ratio", rclcpp::ParameterValue(SLOWDOWN_RATIO));
  cm_->set_parameter(
    rclcpp::Parameter(polygon_name + ".slowdown_ratio", SLOWDOWN_RATIO));

  cm_->declare_parameter(
    polygon_name + ".time_before_collision", rclcpp::ParameterValue(TIME_BEFORE_COLLISION));
  cm_->set_parameter(
    rclcpp::Parameter(polygon_name + ".time_before_collision", TIME_BEFORE_COLLISION));

  cm_->declare_parameter(
    polygon_name + ".simulation_time_step", rclcpp::ParameterValue(SIMULATION_TIME_STEP));
  cm_->set_parameter(
    rclcpp::Parameter(polygon_name + ".simulation_time_step", SIMULATION_TIME_STEP));

  cm_->declare_parameter(
    polygon_name + ".visualize", rclcpp::ParameterValue(false));
  cm_->set_parameter(
    rclcpp::Parameter(polygon_name + ".visualize", false));

  cm_->declare_parameter(
    polygon_name + ".polygon_pub_topic", rclcpp::ParameterValue(polygon_name));
  cm_->set_parameter(
    rclcpp::Parameter(polygon_name + ".polygon_pub_topic", polygon_name));
}

void Tester::addSource(
  const std::string & source_name, const SourceType type)
{
  if (type == SCAN) {
    cm_->declare_parameter(
      source_name + ".type", rclcpp::ParameterValue("scan"));
    cm_->set_parameter(
      rclcpp::Parameter(source_name + ".type", "scan"));
  } else if (type == POINTCLOUD) {
    cm_->declare_parameter(
      source_name + ".type", rclcpp::ParameterValue("pointcloud"));
    cm_->set_parameter(
      rclcpp::Parameter(source_name + ".type", "pointcloud"));

    cm_->declare_parameter(
      source_name + ".min_height", rclcpp::ParameterValue(0.1));
    cm_->set_parameter(
      rclcpp::Parameter(source_name + ".min_height", 0.1));
    cm_->declare_parameter(
      source_name + ".max_height", rclcpp::ParameterValue(1.0));
    cm_->set_parameter(
      rclcpp::Parameter(source_name + ".max_height", 1.0));
  } else if (type == RANGE) {
    cm_->declare_parameter(
      source_name + ".type", rclcpp::ParameterValue("range"));
    cm_->set_parameter(
      rclcpp::Parameter(source_name + ".type", "range"));

    cm_->declare_parameter(
      source_name + ".obstacles_angle", rclcpp::ParameterValue(M_PI / 200));
    cm_->set_parameter(
      rclcpp::Parameter(source_name + ".obstacles_angle", M_PI / 200));
  } else {  // type == SOURCE_UNKNOWN
    cm_->declare_parameter(
      source_name + ".type", rclcpp::ParameterValue("unknown"));
    cm_->set_parameter(
      rclcpp::Parameter(source_name + ".type", "unknown"));
  }

  cm_->declare_parameter(
    source_name + ".topic", rclcpp::ParameterValue(source_name));
  cm_->set_parameter(
    rclcpp::Parameter(source_name + ".topic", source_name));
}

void Tester::setVectors(
  const std::vector<std::string> & polygons,
  const std::vector<std::string> & sources)
{
  cm_->declare_parameter("polygons", rclcpp::ParameterValue(polygons));
  cm_->set_parameter(rclcpp::Parameter("polygons", polygons));

  cm_->declare_parameter("observation_sources", rclcpp::ParameterValue(sources));
  cm_->set_parameter(rclcpp::Parameter("observation_sources", sources));
}

void Tester::sendTransforms(const rclcpp::Time & stamp)
{
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster =
    std::make_shared<tf2_ros::TransformBroadcaster>(cm_);

  geometry_msgs::msg::TransformStamped transform;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;

  // Fill TF buffer ahead for future transform usage in CollisionMonitor::process()
  const rclcpp::Duration ahead = 1000ms;
  for (rclcpp::Time t = stamp; t <= stamp + ahead; t += rclcpp::Duration(50ms)) {
    transform.header.stamp = t;

    // base_frame -> source_frame transform
    transform.header.frame_id = BASE_FRAME_ID;
    transform.child_frame_id = SOURCE_FRAME_ID;
    tf_broadcaster->sendTransform(transform);

    // odom_frame -> base_frame transform
    transform.header.frame_id = ODOM_FRAME_ID;
    transform.child_frame_id = BASE_FRAME_ID;
    tf_broadcaster->sendTransform(transform);
  }
}

void Tester::publishFootprint(const double radius, const rclcpp::Time & stamp)
{
  std::unique_ptr<geometry_msgs::msg::PolygonStamped> msg =
    std::make_unique<geometry_msgs::msg::PolygonStamped>();

  msg->header.frame_id = BASE_FRAME_ID;
  msg->header.stamp = stamp;

  geometry_msgs::msg::Point32 p;
  p.x = radius;
  p.y = radius;
  msg->polygon.points.push_back(p);
  p.x = radius;
  p.y = -radius;
  msg->polygon.points.push_back(p);
  p.x = -radius;
  p.y = -radius;
  msg->polygon.points.push_back(p);
  p.x = -radius;
  p.y = radius;
  msg->polygon.points.push_back(p);

  footprint_pub_->publish(std::move(msg));
}

void Tester::publishScan(const double dist, const rclcpp::Time & stamp)
{
  std::unique_ptr<sensor_msgs::msg::LaserScan> msg =
    std::make_unique<sensor_msgs::msg::LaserScan>();

  msg->header.frame_id = SOURCE_FRAME_ID;
  msg->header.stamp = stamp;

  msg->angle_min = 0.0;
  msg->angle_max = 2 * M_PI;
  msg->angle_increment = M_PI / 180;
  msg->time_increment = 0.0;
  msg->scan_time = 0.0;
  msg->range_min = 0.1;
  msg->range_max = dist + 1.0;
  std::vector<float> ranges(360, dist);
  msg->ranges = ranges;

  scan_pub_->publish(std::move(msg));
}

void Tester::publishPointCloud(const double dist, const rclcpp::Time & stamp)
{
  std::unique_ptr<sensor_msgs::msg::PointCloud2> msg =
    std::make_unique<sensor_msgs::msg::PointCloud2>();
  sensor_msgs::PointCloud2Modifier modifier(*msg);

  msg->header.frame_id = SOURCE_FRAME_ID;
  msg->header.stamp = stamp;

  modifier.setPointCloud2Fields(
    3, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(2);

  sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

  // Point 0: (dist, 0.01, 0.2)
  *iter_x = dist;
  *iter_y = 0.01;
  *iter_z = 0.2;
  ++iter_x; ++iter_y; ++iter_z;

  // Point 1: (dist, -0.01, 0.2)
  *iter_x = dist;
  *iter_y = -0.01;
  *iter_z = 0.2;

  pointcloud_pub_->publish(std::move(msg));
}

void Tester::publishRange(const double dist, const rclcpp::Time & stamp)
{
  std::unique_ptr<sensor_msgs::msg::Range> msg =
    std::make_unique<sensor_msgs::msg::Range>();

  msg->header.frame_id = SOURCE_FRAME_ID;
  msg->header.stamp = stamp;

  msg->radiation_type = 0;
  msg->field_of_view = M_PI / 10;
  msg->min_range = 0.1;
  msg->max_range = dist + 0.1;
  msg->range = dist;

  range_pub_->publish(std::move(msg));
}

void Tester::publishCmdVel(const double x, const double y, const double tw)
{
  // Reset cmd_vel_out_ before calling CollisionMonitor::process()
  cmd_vel_out_ = nullptr;

  std::unique_ptr<geometry_msgs::msg::Twist> msg =
    std::make_unique<geometry_msgs::msg::Twist>();

  msg->linear.x = x;
  msg->linear.y = y;
  msg->angular.z = tw;

  cmd_vel_in_pub_->publish(std::move(msg));
}

void Tester::publishScheduleCmdVel(const double x, const double y, const double tw)
{
  auto msg = std::make_unique<geometry_msgs::msg::Twist>();
  msg->linear.x = x;
  msg->linear.y = y;
  msg->angular.z = tw;
  schedule_cmd_vel_pub_->publish(std::move(msg));
}

bool Tester::waitData(
  const double expected_dist,
  const std::chrono::nanoseconds & timeout,
  const rclcpp::Time & stamp)
{
  rclcpp::Time start_time = cm_->now();
  while (rclcpp::ok() && cm_->now() - start_time <= rclcpp::Duration(timeout)) {
    if (cm_->correctDataReceived(expected_dist, stamp)) {
      return true;
    }
    rclcpp::spin_some(cm_->get_node_base_interface());
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

bool Tester::waitCmdVel(const std::chrono::nanoseconds & timeout)
{
  rclcpp::Time start_time = cm_->now();
  while (rclcpp::ok() && cm_->now() - start_time <= rclcpp::Duration(timeout)) {
    if (cmd_vel_out_) {
      return true;
    }
    rclcpp::spin_some(cm_->get_node_base_interface());
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

bool Tester::waitObstacleStatus(
  const std::uint8_t expected_status, const std::chrono::nanoseconds & timeout)
{
  obstacle_status_.reset();
  const rclcpp::Time start_time = cm_->now();
  while (rclcpp::ok() && cm_->now() - start_time <= rclcpp::Duration(timeout)) {
    rclcpp::spin_some(cm_->get_node_base_interface());
    if (obstacle_status_ && obstacle_status_->status == expected_status) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

void Tester::cmdVelOutCallback(geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vel_out_ = msg;
}

void Tester::obstacleStatusCallback(
  nav_interfaces::msg::ObstacleMonitorStatus::SharedPtr msg)
{
  obstacle_status_ = std::move(msg);
}

