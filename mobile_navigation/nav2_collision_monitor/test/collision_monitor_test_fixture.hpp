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

#ifndef COLLISION_MONITOR_TEST_FIXTURE_HPP_
#define COLLISION_MONITOR_TEST_FIXTURE_HPP_

#include <gtest/gtest.h>

#include <math.h>
#include <cmath>
#include <chrono>
#include <memory>
#include <utility>
#include <vector>
#include <string>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "std_msgs/msg/empty.hpp"

#include "tf2_ros/transform_broadcaster.h"

#include "nav2_collision_monitor/types.hpp"
#include "nav2_collision_monitor/collision_monitor_node.hpp"
#include "nav_interfaces/msg/tracking_path.hpp"
#include "nav_interfaces/msg/obstacle_monitor_status.hpp"

using namespace std::chrono_literals;

static constexpr double EPSILON = 1e-5;

static const char BASE_FRAME_ID[]{"base_link"};
static const char SOURCE_FRAME_ID[]{"base_source"};
static const char ODOM_FRAME_ID[]{"odom"};
static const char CMD_VEL_IN_TOPIC[]{"cmd_vel_in"};
static const char CMD_VEL_OUT_TOPIC[]{"cmd_vel_out"};
static const char OBSTACLE_STATUS_TOPIC[]{"obstacle_status"};
static const char FOOTPRINT_TOPIC[]{"footprint"};
static const char SCAN_NAME[]{"Scan"};
static const char POINTCLOUD_NAME[]{"PointCloud"};
static const char RANGE_NAME[]{"Range"};
static const int MAX_POINTS{1};
static const double SLOWDOWN_RATIO{0.7};
static const double TIME_BEFORE_COLLISION{1.0};
static const double SIMULATION_TIME_STEP{0.01};
static const double TRANSFORM_TOLERANCE{0.5};
static const double SOURCE_TIMEOUT{5.0};
// 多数用例只发一帧 scan 并持续数秒；失效检测由专门用例覆盖。
static const double SCAN_FAULT_TIMEOUT{5.0};
static const char HARD_STOP_TOPIC[]{"cmd_vel_hard_stop"};
static const double STOP_PUB_TIMEOUT{0.1};

enum PolygonType
{
  POLYGON_UNKNOWN = 0,
  POLYGON = 1,
  CIRCLE = 2
};

enum SourceType
{
  SOURCE_UNKNOWN = 0,
  SCAN = 1,
  POINTCLOUD = 2,
  RANGE = 3
};

class CollisionMonitorWrapper : public nav2_collision_monitor::CollisionMonitor
{
public:
  void start()
  {
    ASSERT_EQ(on_configure(get_current_state()), nav2_util::CallbackReturn::SUCCESS);
    ASSERT_EQ(on_activate(get_current_state()), nav2_util::CallbackReturn::SUCCESS);
  }

  void startManualControlLoop()
  {
    start();
    ASSERT_NE(control_loop_timer_, nullptr);
    control_loop_timer_->cancel();
  }

  void receiveFreeVelocity(double x)
  {
    auto msg = std::make_shared<geometry_msgs::msg::Twist>();
    msg->linear.x = x;
    cmdVelInCallback(msg);
  }

  void receiveScheduleVelocity(double x, double y = 0.0, double tw = 0.0)
  {
    auto msg = std::make_shared<geometry_msgs::msg::Twist>();
    msg->linear.x = x;
    msg->linear.y = y;
    msg->angular.z = tw;
    cmdVelInCallback(msg);
  }

  void receiveFreePath()
  {
    auto msg = std::make_shared<nav_msgs::msg::Path>();
    msg->poses.emplace_back();
    freeNavigationPathCallback(msg);
  }

  void runControlLoop()
  {
    controlLoopCallback();
  }

  std::size_t processSampleCount() const
  {
    return process_samples_ms_.size();
  }

  std::int64_t scanReceiptNs(const std::string & name) const
  {
    for (const auto & source : sources_) {
      if (source->getName() == name) {
        return source->getLastReceivedSteadyNs();
      }
    }
    return 0;
  }

  NavigationMode mode() const
  {
    return navigation_mode_;
  }

  std::vector<nav2_collision_monitor::Point> polygonPoints(const std::string & name) const
  {
    for (const auto & polygon : polygons_) {
      if (polygon->getName() == name) {
        std::vector<nav2_collision_monitor::Point> points;
        polygon->getPolygon(points);
        return points;
      }
    }
    return {};
  }

  double polygonPadding(const std::string & name) const
  {
    for (const auto & polygon : polygons_) {
      if (polygon->getName() == name) {
        return polygon->getPadding();
      }
    }
    return -1.0;
  }

  bool polygonLateralPaddingOnly(const std::string & name) const
  {
    for (const auto & polygon : polygons_) {
      if (polygon->getName() == name) {
        return polygon->isLateralPaddingOnly();
      }
    }
    return false;
  }

  void stop()
  {
    ASSERT_EQ(on_deactivate(get_current_state()), nav2_util::CallbackReturn::SUCCESS);
    ASSERT_EQ(on_cleanup(get_current_state()), nav2_util::CallbackReturn::SUCCESS);
    ASSERT_EQ(on_shutdown(get_current_state()), nav2_util::CallbackReturn::SUCCESS);
  }

  void configure()
  {
    ASSERT_EQ(on_configure(get_current_state()), nav2_util::CallbackReturn::SUCCESS);
  }

  void cant_configure()
  {
    ASSERT_EQ(on_configure(get_current_state()), nav2_util::CallbackReturn::FAILURE);
  }

  bool correctDataReceived(const double expected_dist, const rclcpp::Time & stamp)
  {
    for (std::shared_ptr<nav2_collision_monitor::Source> source : sources_) {
      std::vector<nav2_collision_monitor::Point> collision_points;
      source->getData(stamp, collision_points);
      if (collision_points.size() != 0) {
        const double dist = std::hypot(collision_points[0].x, collision_points[0].y);
        if (std::fabs(dist - expected_dist) <= EPSILON) {
          return true;
        }
      }
    }
    return false;
  }

  bool sourceTransformReady(const rclcpp::Time & stamp) const
  {
    return tf_buffer_ &&
           tf_buffer_->canTransform(ODOM_FRAME_ID, BASE_FRAME_ID, tf2::TimePointZero) &&
           tf_buffer_->canTransform(ODOM_FRAME_ID, SOURCE_FRAME_ID, stamp);
  }

  void receiveTrackingPath(
    const std::vector<nav2_collision_monitor::Point> & points, float padding = 0.0F)
  {
    auto msg = std::make_shared<nav_interfaces::msg::TrackingPath>();
    msg->header.frame_id = BASE_FRAME_ID;
    msg->header.stamp = now();
    msg->padding = padding;
    for (const auto & point : points) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = point.x;
      pose.position.y = point.y;
      msg->poses.push_back(pose);
    }
    trackingPathCallback(msg);
  }

  bool hasPaddingService() const {return set_padding_srv_ != nullptr;}
  bool hasTrackingPathSubscription() const {return tracking_path_sub_ != nullptr;}
  bool hasFreePathSubscription() const {return free_navigation_path_sub_ != nullptr;}
};  // CollisionMonitorWrapper

class Tester : public ::testing::Test
{
public:
  Tester();
  ~Tester();

  // Configuring
  void setCommonParameters();
  void addPolygon(
    const std::string & polygon_name, const PolygonType type,
    const double size, const std::string & at);
  void addSource(const std::string & source_name, const SourceType type);
  void setVectors(
    const std::vector<std::string> & polygons,
    const std::vector<std::string> & sources);

  // Setting TF chains
  void sendTransforms(const rclcpp::Time & stamp);

  // Publish robot footprint
  void publishFootprint(const double radius, const rclcpp::Time & stamp);

  // Main topic/data working routines
  void publishScan(const double dist, const rclcpp::Time & stamp);
  void publishPointCloud(const double dist, const rclcpp::Time & stamp);
  void publishRange(const double dist, const rclcpp::Time & stamp);
  void publishCmdVel(const double x, const double y, const double tw);
  void publishScheduleCmdVel(const double x, const double y, const double tw);
  bool waitData(
    const double expected_dist,
    const std::chrono::nanoseconds & timeout,
    const rclcpp::Time & stamp);
  bool waitCmdVel(const std::chrono::nanoseconds & timeout);
  bool waitObstacleStatus(
    const std::uint8_t expected_status, const std::chrono::nanoseconds & timeout);
  /// 在 timeout 内持续处理回调，返回累计收到的 hard stop 次数。
  int spinHardStops(const std::chrono::nanoseconds & timeout);

protected:
  void cmdVelOutCallback(geometry_msgs::msg::Twist::SharedPtr msg);
  void obstacleStatusCallback(nav_interfaces::msg::ObstacleMonitorStatus::SharedPtr msg);

  // CollisionMonitor node
  std::shared_ptr<CollisionMonitorWrapper> cm_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Footprint publisher
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;

  // Data source publishers
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;

  // Working with cmd_vel_in/cmd_vel_out
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_in_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_out_sub_;
  rclcpp::Subscription<nav_interfaces::msg::ObstacleMonitorStatus>::SharedPtr
    obstacle_status_sub_;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr hard_stop_sub_;

  geometry_msgs::msg::Twist::SharedPtr cmd_vel_out_;
  nav_interfaces::msg::ObstacleMonitorStatus::SharedPtr obstacle_status_;
  int hard_stop_count_{0};
};  // Tester

#endif  // COLLISION_MONITOR_TEST_FIXTURE_HPP_
