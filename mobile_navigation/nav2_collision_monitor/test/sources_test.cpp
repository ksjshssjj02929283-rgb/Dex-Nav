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

#include <gtest/gtest.h>

#include <math.h>
#include <cmath>
#include <chrono>
#include <atomic>
#include <thread>
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

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

#include "nav2_collision_monitor/types.hpp"
#include "nav2_collision_monitor/scan.hpp"
#include "nav2_collision_monitor/pointcloud.hpp"
#include "nav2_collision_monitor/range.hpp"

using namespace std::chrono_literals;

static constexpr double EPSILON = std::numeric_limits<float>::epsilon();

static const char BASE_FRAME_ID[]{"base_link"};
static const char SOURCE_FRAME_ID[]{"base_source"};
static const char GLOBAL_FRAME_ID[]{"odom"};
static const char SCAN_NAME[]{"LaserScan"};
static const char SCAN_TOPIC[]{"scan"};
static const char POINTCLOUD_NAME[]{"PointCloud"};
static const char POINTCLOUD_TOPIC[]{"pointcloud"};
static const char RANGE_NAME[]{"Range"};
static const char RANGE_TOPIC[]{"range"};
static const tf2::Duration TRANSFORM_TOLERANCE{tf2::durationFromSec(0.1)};
static const rclcpp::Duration DATA_TIMEOUT{rclcpp::Duration::from_seconds(5.0)};

class TestNode : public nav2_util::LifecycleNode
{
public:
  TestNode()
  : nav2_util::LifecycleNode("test_node")
  {
  }

  ~TestNode()
  {
    scan_pub_.reset();
    pointcloud_pub_.reset();
    range_pub_.reset();
  }

  void publishScan(const rclcpp::Time & stamp, const double range)
  {
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      SCAN_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    std::unique_ptr<sensor_msgs::msg::LaserScan> msg =
      std::make_unique<sensor_msgs::msg::LaserScan>();

    msg->header.frame_id = SOURCE_FRAME_ID;
    msg->header.stamp = stamp;

    msg->angle_min = 0.0;
    msg->angle_max = 2 * M_PI;
    msg->angle_increment = M_PI / 2;
    msg->time_increment = 0.0;
    msg->scan_time = 0.0;
    msg->range_min = 0.1;
    msg->range_max = 1.1;
    std::vector<float> ranges(4, range);
    msg->ranges = ranges;

    scan_pub_->publish(std::move(msg));
  }

  void publishPointCloud(const rclcpp::Time & stamp)
  {
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      POINTCLOUD_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    std::unique_ptr<sensor_msgs::msg::PointCloud2> msg =
      std::make_unique<sensor_msgs::msg::PointCloud2>();
    sensor_msgs::PointCloud2Modifier modifier(*msg);

    msg->header.frame_id = SOURCE_FRAME_ID;
    msg->header.stamp = stamp;

    modifier.setPointCloud2Fields(
      3, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(3);

    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

    // Point 0: (0.5, 0.5, 0.2)
    *iter_x = 0.5;
    *iter_y = 0.5;
    *iter_z = 0.2;
    ++iter_x; ++iter_y; ++iter_z;

    // Point 1: (-0.5, -0.5, 0.3)
    *iter_x = -0.5;
    *iter_y = -0.5;
    *iter_z = 0.3;
    ++iter_x; ++iter_y; ++iter_z;

    // Point 2: (1.0, 1.0, 10.0)
    *iter_x = 1.0;
    *iter_y = 1.0;
    *iter_z = 10.0;

    pointcloud_pub_->publish(std::move(msg));
  }

  void publishRange(const rclcpp::Time & stamp, const double range)
  {
    range_pub_ = this->create_publisher<sensor_msgs::msg::Range>(
      RANGE_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    std::unique_ptr<sensor_msgs::msg::Range> msg =
      std::make_unique<sensor_msgs::msg::Range>();

    msg->header.frame_id = SOURCE_FRAME_ID;
    msg->header.stamp = stamp;

    msg->radiation_type = 0;
    msg->field_of_view = M_PI / 10;
    msg->min_range = 0.1;
    msg->max_range = 1.1;
    msg->range = range;

    range_pub_->publish(std::move(msg));
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
};  // TestNode

class ScanWrapper : public nav2_collision_monitor::Scan
{
public:
  using nav2_collision_monitor::Scan::dataCallback;
  ScanWrapper(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const std::string & global_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & data_timeout,
    const bool base_shift_correction)
  : nav2_collision_monitor::Scan(
      node, source_name, tf_buffer, base_frame_id, global_frame_id,
      transform_tolerance, data_timeout, base_shift_correction)
  {}

  bool dataReceived() const
  {
    return std::atomic_load(&data_) != nullptr;
  }
};  // ScanWrapper

class PointCloudWrapper : public nav2_collision_monitor::PointCloud
{
public:
  using nav2_collision_monitor::PointCloud::dataCallback;
  PointCloudWrapper(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const std::string & global_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & data_timeout,
    const bool base_shift_correction)
  : nav2_collision_monitor::PointCloud(
      node, source_name, tf_buffer, base_frame_id, global_frame_id,
      transform_tolerance, data_timeout, base_shift_correction)
  {}

  bool dataReceived() const
  {
    return std::atomic_load(&data_) != nullptr;
  }
};  // PointCloudWrapper

class RangeWrapper : public nav2_collision_monitor::Range
{
public:
  using nav2_collision_monitor::Range::dataCallback;
  RangeWrapper(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const std::string & global_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & data_timeout,
    const bool base_shift_correction)
  : nav2_collision_monitor::Range(
      node, source_name, tf_buffer, base_frame_id, global_frame_id,
      transform_tolerance, data_timeout, base_shift_correction)
  {}

  bool dataReceived() const
  {
    return std::atomic_load(&data_) != nullptr;
  }
};  // RangeWrapper

class Tester : public ::testing::Test
{
public:
  Tester();
  ~Tester();

protected:
  // Data sources creation routine
  void createSources(const bool base_shift_correction = true);

  // Setting TF chains
  void sendTransforms(const rclcpp::Time & stamp);

  // Data sources working routines
  bool waitScan(const std::chrono::nanoseconds & timeout);
  bool waitPointCloud(const std::chrono::nanoseconds & timeout);
  bool waitRange(const std::chrono::nanoseconds & timeout);
  void checkScan(const std::vector<nav2_collision_monitor::Point> & data);
  void checkPointCloud(const std::vector<nav2_collision_monitor::Point> & data);
  void checkRange(const std::vector<nav2_collision_monitor::Point> & data);

  std::shared_ptr<TestNode> test_node_;
  std::shared_ptr<ScanWrapper> scan_;
  std::shared_ptr<PointCloudWrapper> pointcloud_;
  std::shared_ptr<RangeWrapper> range_;

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};  // Tester

Tester::Tester()
{
  test_node_ = std::make_shared<TestNode>();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(test_node_->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

Tester::~Tester()
{
  scan_.reset();
  pointcloud_.reset();
  range_.reset();

  test_node_.reset();

  tf_listener_.reset();
  tf_buffer_.reset();
}

void Tester::createSources(const bool base_shift_correction)
{
  // Create Scan object
  test_node_->declare_parameter(
    std::string(SCAN_NAME) + ".topic", rclcpp::ParameterValue(SCAN_TOPIC));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(SCAN_NAME) + ".topic", SCAN_TOPIC));

  scan_ = std::make_shared<ScanWrapper>(
    test_node_, SCAN_NAME, tf_buffer_,
    BASE_FRAME_ID, GLOBAL_FRAME_ID,
    TRANSFORM_TOLERANCE, DATA_TIMEOUT, base_shift_correction);
  scan_->configure();

  // Create PointCloud object
  test_node_->declare_parameter(
    std::string(POINTCLOUD_NAME) + ".topic", rclcpp::ParameterValue(POINTCLOUD_TOPIC));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(POINTCLOUD_NAME) + ".topic", POINTCLOUD_TOPIC));
  test_node_->declare_parameter(
    std::string(POINTCLOUD_NAME) + ".min_height", rclcpp::ParameterValue(0.1));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(POINTCLOUD_NAME) + ".min_height", 0.1));
  test_node_->declare_parameter(
    std::string(POINTCLOUD_NAME) + ".max_height", rclcpp::ParameterValue(1.0));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(POINTCLOUD_NAME) + ".max_height", 1.0));

  pointcloud_ = std::make_shared<PointCloudWrapper>(
    test_node_, POINTCLOUD_NAME, tf_buffer_,
    BASE_FRAME_ID, GLOBAL_FRAME_ID,
    TRANSFORM_TOLERANCE, DATA_TIMEOUT, base_shift_correction);
  pointcloud_->configure();

  // Create Range object
  test_node_->declare_parameter(
    std::string(RANGE_NAME) + ".topic", rclcpp::ParameterValue(RANGE_TOPIC));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(RANGE_NAME) + ".topic", RANGE_TOPIC));

  test_node_->declare_parameter(
    std::string(RANGE_NAME) + ".obstacles_angle", rclcpp::ParameterValue(M_PI / 199));

  range_ = std::make_shared<RangeWrapper>(
    test_node_, RANGE_NAME, tf_buffer_,
    BASE_FRAME_ID, GLOBAL_FRAME_ID,
    TRANSFORM_TOLERANCE, DATA_TIMEOUT, base_shift_correction);
  range_->configure();
}

void Tester::sendTransforms(const rclcpp::Time & stamp)
{
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster =
    std::make_shared<tf2_ros::TransformBroadcaster>(test_node_);

  geometry_msgs::msg::TransformStamped transform;

  // base_frame -> source_frame transform
  transform.header.frame_id = BASE_FRAME_ID;
  transform.child_frame_id = SOURCE_FRAME_ID;

  transform.header.stamp = stamp;
  transform.transform.translation.x = 0.1;
  transform.transform.translation.y = 0.1;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;

  tf_broadcaster->sendTransform(transform);

  // global_frame -> base_frame transform
  transform.header.frame_id = GLOBAL_FRAME_ID;
  transform.child_frame_id = BASE_FRAME_ID;

  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;

  tf_broadcaster->sendTransform(transform);
}

bool Tester::waitScan(const std::chrono::nanoseconds & timeout)
{
  rclcpp::Time start_time = test_node_->now();
  while (rclcpp::ok() && test_node_->now() - start_time <= rclcpp::Duration(timeout)) {
    if (scan_->dataReceived()) {
      return true;
    }
    rclcpp::spin_some(test_node_->get_node_base_interface());
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

bool Tester::waitPointCloud(const std::chrono::nanoseconds & timeout)
{
  rclcpp::Time start_time = test_node_->now();
  while (rclcpp::ok() && test_node_->now() - start_time <= rclcpp::Duration(timeout)) {
    if (pointcloud_->dataReceived()) {
      return true;
    }
    rclcpp::spin_some(test_node_->get_node_base_interface());
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

bool Tester::waitRange(const std::chrono::nanoseconds & timeout)
{
  rclcpp::Time start_time = test_node_->now();
  while (rclcpp::ok() && test_node_->now() - start_time <= rclcpp::Duration(timeout)) {
    if (range_->dataReceived()) {
      return true;
    }
    rclcpp::spin_some(test_node_->get_node_base_interface());
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

void Tester::checkScan(const std::vector<nav2_collision_monitor::Point> & data)
{
  ASSERT_EQ(data.size(), 4u);

  // Point 0: (1.0 + 0.1, 0.0 + 0.1)
  EXPECT_NEAR(data[0].x, 1.1, EPSILON);
  EXPECT_NEAR(data[0].y, 0.1, EPSILON);

  // Point 1: (0.0 + 0.1, 1.0 + 0.1)
  EXPECT_NEAR(data[1].x, 0.1, EPSILON);
  EXPECT_NEAR(data[1].y, 1.1, EPSILON);

  // Point 2: (-1.0 + 0.1, 0.0 + 0.1)
  EXPECT_NEAR(data[2].x, -0.9, EPSILON);
  EXPECT_NEAR(data[2].y, 0.1, EPSILON);

  // Point 3: (0.0 + 0.1, -1.0 + 0.1)
  EXPECT_NEAR(data[3].x, 0.1, EPSILON);
  EXPECT_NEAR(data[3].y, -0.9, EPSILON);
}

void Tester::checkPointCloud(const std::vector<nav2_collision_monitor::Point> & data)
{
  ASSERT_EQ(data.size(), 2u);

  // Point 0: (0.5 + 0.1, 0.5 + 0.1)
  EXPECT_NEAR(data[0].x, 0.6, EPSILON);
  EXPECT_NEAR(data[0].y, 0.6, EPSILON);

  // Point 1: (-0.5 + 0.1, -0.5 + 0.1)
  EXPECT_NEAR(data[1].x, -0.4, EPSILON);
  EXPECT_NEAR(data[1].y, -0.4, EPSILON);

  // Point 2 should be out of scope by height
}

void Tester::checkRange(const std::vector<nav2_collision_monitor::Point> & data)
{
  ASSERT_EQ(data.size(), 21u);

  const double angle_increment = M_PI / 199;
  double angle = -M_PI / (10 * 2);
  int i;
  for (i = 0; i < 199 / 10 + 1; i++) {
    ASSERT_NEAR(data[i].x, 1.0 * std::cos(angle) + 0.1, EPSILON);
    ASSERT_NEAR(data[i].y, 1.0 * std::sin(angle) + 0.1, EPSILON);
    angle += angle_increment;
  }
  // Check for the latest FoW/2 point
  angle = M_PI / (10 * 2);
  ASSERT_NEAR(data[i].x, 1.0 * std::cos(angle) + 0.1, EPSILON);
  ASSERT_NEAR(data[i].y, 1.0 * std::sin(angle) + 0.1, EPSILON);
}

// Alternate different-sized frames while converting points. Every returned batch must
// belong to one complete frame, including its metadata and backing storage.
template<typename SourceT, typename MessageT>
void checkConcurrentSnapshots(
  SourceT & source, const std::shared_ptr<MessageT> & first,
  const std::shared_ptr<MessageT> & second, const rclcpp::Time & stamp,
  std::size_t first_size, std::size_t second_size)
{
  source.dataCallback(first);
  std::atomic<bool> started{false};
  std::atomic<bool> stop{false};
  std::thread writer([&]() {
      source.dataCallback(second);
      started.store(true);
      while (!stop.load()) {
        source.dataCallback(first);
        source.dataCallback(second);
        std::this_thread::yield();
      }
    });
  while (!started.load()) {
    std::this_thread::yield();
  }
  bool consistent = true;
  for (int iteration = 0; iteration < 2000; ++iteration) {
    std::vector<nav2_collision_monitor::Point> points;
    source.getData(stamp, points);
    if (points.empty()) {
      consistent = false;
      break;
    }
    const double radius = std::hypot(points.front().x, points.front().y);
    const bool is_first = std::fabs(radius - 0.5) < 1e-5;
    const bool is_second = std::fabs(radius - 1.0) < 1e-5;
    consistent &= (is_first || is_second) &&
      points.size() == (is_first ? first_size : second_size);
    for (const auto & point : points) {
      consistent &= std::fabs(std::hypot(point.x, point.y) - radius) < 1e-5;
    }
  }
  stop.store(true);
  writer.join();
  EXPECT_TRUE(consistent);
}

TEST_F(Tester, testConcurrentScanSnapshots)
{
  createSources(false);
  const auto stamp = test_node_->now();
  auto first = std::make_shared<sensor_msgs::msg::LaserScan>();
  first->header.frame_id = BASE_FRAME_ID;
  first->header.stamp = stamp;
  first->range_min = 0.1;
  first->range_max = 2.0;
  first->angle_increment = 0.01;
  first->ranges.assign(8, 0.5F);
  auto second = std::make_shared<sensor_msgs::msg::LaserScan>(*first);
  second->ranges.assign(128, 1.0F);
  second->angle_increment = 0.02;
  checkConcurrentSnapshots(*scan_, first, second, stamp, 8, 128);
}

TEST_F(Tester, testScanTransformFailureInvalidatesProcessedCache)
{
  createSources(false);
  const auto stamp = test_node_->now();
  auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan->header.frame_id = BASE_FRAME_ID;
  scan->header.stamp = stamp;
  scan->range_min = 0.1F;
  scan->range_max = 2.0F;
  scan->angle_increment = 0.01F;
  scan->ranges.assign(8, 0.5F);

  scan_->dataCallback(scan);
  std::vector<nav2_collision_monitor::Point> points;
  scan_->getData(stamp, points);
  ASSERT_EQ(points.size(), 8u);

  // A newer frame with broken TF must not leave the previous valid obstacle set active.
  auto invalid_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);
  invalid_scan->header.frame_id = "missing_sensor_frame";
  scan_->dataCallback(invalid_scan);
  points.clear();
  scan_->getData(stamp, points);
  EXPECT_TRUE(points.empty());
}

TEST_F(Tester, testConcurrentPointCloudSnapshots)
{
  createSources(false);
  const auto stamp = test_node_->now();
  const auto make_cloud = [&](std::size_t count, float x) {
      auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      msg->header.frame_id = BASE_FRAME_ID;
      msg->header.stamp = stamp;
      sensor_msgs::PointCloud2Modifier modifier(*msg);
      modifier.setPointCloud2FieldsByString(1, "xyz");
      modifier.resize(count);
      sensor_msgs::PointCloud2Iterator<float> px(*msg, "x");
      sensor_msgs::PointCloud2Iterator<float> py(*msg, "y");
      sensor_msgs::PointCloud2Iterator<float> pz(*msg, "z");
      for (std::size_t i = 0; i < count; ++i, ++px, ++py, ++pz) {
        *px = x;
        *py = 0.0F;
        *pz = 0.2F;
      }
      return msg;
    };
  checkConcurrentSnapshots(*pointcloud_, make_cloud(8, 0.5F), make_cloud(128, 1.0F),
    stamp, 8, 128);
}

TEST_F(Tester, testConcurrentRangeSnapshots)
{
  createSources(false);
  const auto stamp = test_node_->now();
  auto first = std::make_shared<sensor_msgs::msg::Range>();
  first->header.frame_id = BASE_FRAME_ID;
  first->header.stamp = stamp;
  first->min_range = 0.1;
  first->max_range = 2.0;
  first->range = 0.5;
  first->field_of_view = 0.0;
  auto second = std::make_shared<sensor_msgs::msg::Range>(*first);
  second->range = 1.0;
  second->field_of_view = 0.1;
  range_->dataCallback(second);
  std::vector<nav2_collision_monitor::Point> expected;
  range_->getData(stamp, expected);
  ASSERT_GT(expected.size(), 1u);
  checkConcurrentSnapshots(*range_, first, second, stamp, 1, expected.size());
}

TEST_F(Tester, testGetData)
{
  rclcpp::Time curr_time = test_node_->now();

  createSources();

  sendTransforms(curr_time);

  // Publish data for sources
  test_node_->publishScan(curr_time, 1.0);
  test_node_->publishPointCloud(curr_time);
  test_node_->publishRange(curr_time, 1.0);

  // Wait until all sources will receive the data
  ASSERT_TRUE(waitScan(500ms));
  ASSERT_TRUE(waitPointCloud(500ms));
  ASSERT_TRUE(waitRange(500ms));

  // Check Scan data
  std::vector<nav2_collision_monitor::Point> data;
  scan_->getData(curr_time, data);
  checkScan(data);

  // Check Pointcloud data
  data.clear();
  pointcloud_->getData(curr_time, data);
  checkPointCloud(data);

  // Check Range data
  data.clear();
  range_->getData(curr_time, data);
  checkRange(data);
}

TEST_F(Tester, testGetOutdatedData)
{
  rclcpp::Time curr_time = test_node_->now();

  createSources();

  sendTransforms(curr_time);

  // Publish outdated data for sources
  test_node_->publishScan(curr_time - DATA_TIMEOUT - 1s, 1.0);
  test_node_->publishPointCloud(curr_time - DATA_TIMEOUT - 1s);
  test_node_->publishRange(curr_time - DATA_TIMEOUT - 1s, 1.0);

  // Wait until all sources will receive the data
  ASSERT_TRUE(waitScan(500ms));
  ASSERT_TRUE(waitPointCloud(500ms));
  ASSERT_TRUE(waitRange(500ms));

  // Scan data should be empty
  std::vector<nav2_collision_monitor::Point> data;
  scan_->getData(curr_time, data);
  ASSERT_EQ(data.size(), 0u);

  // Pointcloud data should be empty
  pointcloud_->getData(curr_time, data);
  ASSERT_EQ(data.size(), 0u);

  // Range data should be empty
  range_->getData(curr_time, data);
  ASSERT_EQ(data.size(), 0u);
}

TEST_F(Tester, testIncorrectFrameData)
{
  rclcpp::Time curr_time = test_node_->now();

  createSources();

  // Send incorrect transform
  sendTransforms(curr_time - 1s);

  // Publish data for sources
  test_node_->publishScan(curr_time, 1.0);
  test_node_->publishPointCloud(curr_time);
  test_node_->publishRange(curr_time, 1.0);

  // Wait until all sources will receive the data
  ASSERT_TRUE(waitScan(500ms));
  ASSERT_TRUE(waitPointCloud(500ms));
  ASSERT_TRUE(waitRange(500ms));

  // Scan data should be empty
  std::vector<nav2_collision_monitor::Point> data;
  scan_->getData(curr_time, data);
  ASSERT_EQ(data.size(), 0u);

  // Pointcloud data should be empty
  pointcloud_->getData(curr_time, data);
  ASSERT_EQ(data.size(), 0u);

  // Range data should be empty
  range_->getData(curr_time, data);
  ASSERT_EQ(data.size(), 0u);
}

TEST_F(Tester, testIncorrectData)
{
  rclcpp::Time curr_time = test_node_->now();

  createSources();

  sendTransforms(curr_time);

  // Publish data for sources
  test_node_->publishScan(curr_time, 2.0);
  test_node_->publishPointCloud(curr_time);
  test_node_->publishRange(curr_time, 2.0);

  // Wait until all sources will receive the data
  ASSERT_TRUE(waitScan(500ms));
  ASSERT_TRUE(waitRange(500ms));

  // Scan data should be empty
  std::vector<nav2_collision_monitor::Point> data;
  scan_->getData(curr_time, data);
  ASSERT_EQ(data.size(), 0u);

  // Range data should be empty
  range_->getData(curr_time, data);
  ASSERT_EQ(data.size(), 0u);
}

TEST_F(Tester, testIgnoreTimeShift)
{
  rclcpp::Time curr_time = test_node_->now();

  createSources(false);

  // Send incorrect transform
  sendTransforms(curr_time - 1s);

  // Publish data for sources
  test_node_->publishScan(curr_time, 1.0);
  test_node_->publishPointCloud(curr_time);
  test_node_->publishRange(curr_time, 1.0);

  // Wait until all sources will receive the data
  ASSERT_TRUE(waitScan(500ms));
  ASSERT_TRUE(waitPointCloud(500ms));
  ASSERT_TRUE(waitRange(500ms));

  // Scan data should be consistent
  std::vector<nav2_collision_monitor::Point> data;
  scan_->getData(curr_time, data);
  checkScan(data);

  // Pointcloud data should be consistent
  data.clear();
  pointcloud_->getData(curr_time, data);
  checkPointCloud(data);

  // Range data should be consistent
  data.clear();
  range_->getData(curr_time, data);
  checkRange(data);
}

int main(int argc, char ** argv)
{
  // Initialize the system
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  // Actual testing
  bool test_result = RUN_ALL_TESTS();

  // Shutdown
  rclcpp::shutdown();

  return test_result;
}
