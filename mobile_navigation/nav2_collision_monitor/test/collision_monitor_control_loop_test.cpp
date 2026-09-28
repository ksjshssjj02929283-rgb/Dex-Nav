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

TEST_F(Tester, testControlLoopConsumesFreeNavigationMailboxes)
{
  setCommonParameters();
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  setVectors({"PolygonStop"}, {});
  cm_->startManualControlLoop();

  EXPECT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::IDLE);

  // Subscription callbacks only update mailboxes. Mode and output must remain unchanged
  // until the deterministic 100 Hz control loop consumes the snapshot.
  cm_->receiveFreePath();
  cm_->receiveFreeVelocity(0.4);
  EXPECT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::IDLE);
  EXPECT_FALSE(cmd_vel_out_);

  cm_->runControlLoop();
  ASSERT_TRUE(waitCmdVel(200ms));
  EXPECT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::AUTOMONOUS_NAVIGATION);
  EXPECT_NEAR(cmd_vel_out_->linear.x, 0.4, EPSILON);
  EXPECT_NEAR(cm_->polygonPadding("PolygonStop"), 0.1, EPSILON);
  EXPECT_FALSE(cm_->polygonLateralPaddingOnly("PolygonStop"));

  cm_->stop();
}

TEST_F(Tester, testControlLoopSkipsProcessWithoutInputUpdate)
{
  setCommonParameters();
  cm_->declare_parameter("performance_timing_enabled", true);
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  setVectors({"PolygonStop"}, {});
  cm_->startManualControlLoop();

  cm_->receiveFreePath();
  cm_->receiveFreeVelocity(0.2);
  cm_->runControlLoop();
  ASSERT_EQ(cm_->processSampleCount(), 1u);

  // The timer still runs the FSM, but unchanged mailboxes and sources do not repeat geometry.
  cm_->runControlLoop();
  EXPECT_EQ(cm_->processSampleCount(), 1u);

  // An identical-valued message is still a new input event and must trigger detection.
  cm_->receiveFreeVelocity(0.2);
  cm_->runControlLoop();
  EXPECT_EQ(cm_->processSampleCount(), 2u);
  cm_->stop();
}

TEST_F(Tester, testControlLoopKeepsNavigationTypeForLifecycle)
{
  setCommonParameters();
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  setVectors({"PolygonStop"}, {});
  cm_->startManualControlLoop();

  cm_->receiveFreePath();
  cm_->receiveFreeVelocity(0.4);
  cm_->runControlLoop();
  ASSERT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::AUTOMONOUS_NAVIGATION);

  // Once AUTOMONOUS is selected, schedule path/velocity belong to an inactive chain and cannot
  // switch this active lifecycle to schedule navigation.
  cm_->receiveTrackingPath({{0.0, 0.0}, {1.0, 0.0}}, 0.0F);
  cm_->receiveScheduleVelocity(0.3);
  cm_->runControlLoop();
  EXPECT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::AUTOMONOUS_NAVIGATION);
  EXPECT_FALSE(cm_->polygonLateralPaddingOnly("PolygonStop"));

  cm_->stop();
}

TEST_F(Tester, testControlLoopScheduleFiltersVelocityAndEdgingMode)
{
  setCommonParameters();
  cm_->declare_parameter("navigation_type", "schedule");
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  cm_->declare_parameter("PolygonStop.default_padding", 0.2);
  setVectors({"PolygonStop"}, {});
  cm_->startManualControlLoop();

  const std::vector<nav2_collision_monitor::Point> route{{0.0, 0.0}, {1.0, 0.0}};
  cm_->receiveTrackingPath(route, 0.0F);
  cm_->receiveScheduleVelocity(0.3);
  cm_->runControlLoop();

  EXPECT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::SCHEDULE_EDGING);
  EXPECT_NEAR(cm_->polygonPadding("PolygonStop"), 0.2, EPSILON);
  EXPECT_TRUE(cm_->polygonLateralPaddingOnly("PolygonStop"));
  EXPECT_TRUE(waitObstacleStatus(nav_interfaces::msg::ObstacleMonitorStatus::FREE, 200ms));
  ASSERT_TRUE(waitCmdVel(200ms));
  EXPECT_NEAR(cmd_vel_out_->linear.x, 0.3, EPSILON);

  // A free-navigation path belongs to the inactive chain and cannot leave schedule edging.
  cm_->receiveFreePath();
  cm_->runControlLoop();
  EXPECT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::SCHEDULE_EDGING);
  EXPECT_TRUE(cm_->polygonLateralPaddingOnly("PolygonStop"));

  cm_->receiveTrackingPath(route, 0.15F);
  cm_->receiveScheduleVelocity(0.3);
  cm_->runControlLoop();
  EXPECT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::SCHEDULE_NAVIGATION);
  EXPECT_FALSE(cm_->polygonLateralPaddingOnly("PolygonStop"));
  ASSERT_TRUE(waitCmdVel(200ms));

  cm_->stop();
}

TEST_F(Tester, testTrackingPathNonFinitePaddingUsesDefault)
{
  setCommonParameters();
  cm_->declare_parameter("navigation_type", "schedule");
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  cm_->declare_parameter("PolygonStop.default_padding", 0.2);
  setVectors({"PolygonStop"}, {});
  cm_->startManualControlLoop();
  const std::vector<nav2_collision_monitor::Point> route{{0.0, 0.0}, {1.0, 0.0}};

  for (float padding : {std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()})
  {
    // A default-valued segment must not inherit the previous custom padding.
    cm_->receiveTrackingPath(route, 0.15F);
    cm_->runControlLoop();
    EXPECT_NEAR(cm_->polygonPadding("PolygonStop"), 0.15, EPSILON);
    cm_->receiveTrackingPath(route, padding);
    cm_->runControlLoop();
    EXPECT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::SCHEDULE_NAVIGATION);
    EXPECT_NEAR(cm_->polygonPadding("PolygonStop"), 0.2, EPSILON);
    EXPECT_FALSE(cm_->polygonLateralPaddingOnly("PolygonStop"));

    // Leaving edging must also restore uniform default padding.
    cm_->receiveTrackingPath(route, 0.0F);
    cm_->runControlLoop();
    ASSERT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::SCHEDULE_EDGING);
    cm_->receiveTrackingPath(route, padding);
    cm_->runControlLoop();
    EXPECT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::SCHEDULE_NAVIGATION);
    EXPECT_NEAR(cm_->polygonPadding("PolygonStop"), 0.2, EPSILON);
    EXPECT_FALSE(cm_->polygonLateralPaddingOnly("PolygonStop"));
  }
  cm_->stop();
}

TEST_F(Tester, testControlLoopVelocityTimeoutReturnsToIdle)
{
  setCommonParameters();
  cm_->declare_parameter("cmd_vel_timeout", 0.02);
  cm_->declare_parameter("idle_grace_period", 0.3);
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  setVectors({"PolygonStop"}, {});
  cm_->startManualControlLoop();

  cm_->receiveFreePath();
  cm_->receiveFreeVelocity(0.4);
  cm_->runControlLoop();
  ASSERT_TRUE(waitCmdVel(200ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.4, EPSILON);

  cmd_vel_out_.reset();
  std::this_thread::sleep_for(30ms);
  cm_->runControlLoop();
  EXPECT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::AUTOMONOUS_NAVIGATION);
  EXPECT_FALSE(waitCmdVel(30ms));

  std::this_thread::sleep_for(300ms);
  cm_->runControlLoop();
  EXPECT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::IDLE);
  ASSERT_TRUE(waitCmdVel(200ms));
  EXPECT_DOUBLE_EQ(cmd_vel_out_->linear.x, 0.0);
  EXPECT_DOUBLE_EQ(cmd_vel_out_->linear.y, 0.0);
  EXPECT_DOUBLE_EQ(cmd_vel_out_->angular.z, 0.0);
  cmd_vel_out_.reset();
  cm_->runControlLoop();
  EXPECT_FALSE(waitCmdVel(30ms));
  cm_->stop();
}

TEST_F(Tester, testPathWithoutVelocityPublishesStopOnlyOnIdle)
{
  setCommonParameters();
  cm_->declare_parameter("cmd_vel_timeout", 0.02);
  cm_->declare_parameter("idle_grace_period", 0.3);
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  setVectors({"PolygonStop"}, {});
  cm_->startManualControlLoop();
  cm_->receiveFreePath();
  cm_->runControlLoop();

  std::this_thread::sleep_for(30ms);
  cm_->runControlLoop();
  EXPECT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::AUTOMONOUS_NAVIGATION);
  EXPECT_FALSE(waitCmdVel(30ms));

  std::this_thread::sleep_for(300ms);
  cm_->runControlLoop();
  EXPECT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::IDLE);
  ASSERT_TRUE(waitCmdVel(200ms));
  EXPECT_DOUBLE_EQ(cmd_vel_out_->linear.x, 0.0);
  EXPECT_DOUBLE_EQ(cmd_vel_out_->linear.y, 0.0);
  EXPECT_DOUBLE_EQ(cmd_vel_out_->angular.z, 0.0);
  cmd_vel_out_.reset();
  cm_->runControlLoop();
  EXPECT_FALSE(waitCmdVel(30ms));
  cm_->stop();
}

TEST_F(Tester, testAnyOfFourScansTimingOutStopsAndRecovers)
{
  setCommonParameters();
  cm_->declare_parameter("scan_fault_timeout", 0.2);
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  const std::vector<std::string> names{"scan_a", "scan_b", "scan_c", "scan_d"};
  for (const auto & name : names) {
    addSource(name, SCAN);
  }
  setVectors({"PolygonStop"}, names);
  cm_->startManualControlLoop();

  std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> publishers;
  for (const auto & name : names) {
    publishers.push_back(
      cm_->create_publisher<sensor_msgs::msg::LaserScan>(
        name, rclcpp::SensorDataQoS()));
  }
  const auto send_scan = [&](std::size_t index) {
      sensor_msgs::msg::LaserScan scan;
      scan.header.frame_id = SOURCE_FRAME_ID;
      scan.header.stamp = cm_->now();
      scan.range_min = 0.1F;
      scan.range_max = 10.0F;
      scan.ranges = {5.0F};
      publishers[index]->publish(scan);
    };
  const auto wait_receipt = [&](std::size_t index, std::int64_t previous) {
      const auto deadline = std::chrono::steady_clock::now() + 500ms;
      while (std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(cm_->get_node_base_interface());
        if (cm_->scanReceiptNs(names[index]) > previous) {
          return true;
        }
        std::this_thread::sleep_for(5ms);
      }
      return false;
    };
  for (std::size_t i = 0; i < names.size(); ++i) {
    send_scan(i);
    ASSERT_TRUE(wait_receipt(i, 0));
  }

  for (std::size_t stale = 0; stale < names.size(); ++stale) {
    // Establish a fresh baseline before letting exactly one of the four scans expire.
    for (std::size_t i = 0; i < names.size(); ++i) {
      const auto previous = cm_->scanReceiptNs(names[i]);
      send_scan(i);
      ASSERT_TRUE(wait_receipt(i, previous));
    }
    cmd_vel_out_.reset();
    cm_->receiveFreeVelocity(0.3);
    cm_->runControlLoop();
    ASSERT_TRUE(waitCmdVel(200ms));
    EXPECT_NEAR(cmd_vel_out_->linear.x, 0.3, EPSILON);

    std::this_thread::sleep_for(220ms);
    for (std::size_t i = 0; i < names.size(); ++i) {
      if (i == stale) {
        continue;
      }
      const auto previous = cm_->scanReceiptNs(names[i]);
      send_scan(i);
      ASSERT_TRUE(wait_receipt(i, previous));
    }
    cmd_vel_out_.reset();
    obstacle_status_.reset();
    cm_->runControlLoop();
    ASSERT_TRUE(waitCmdVel(200ms));
    EXPECT_DOUBLE_EQ(cmd_vel_out_->linear.x, 0.0);
    ASSERT_NE(obstacle_status_, nullptr);
    EXPECT_EQ(obstacle_status_->status, nav_interfaces::msg::ObstacleMonitorStatus::STOP);

    const auto previous = cm_->scanReceiptNs(names[stale]);
    send_scan(stale);
    ASSERT_TRUE(wait_receipt(stale, previous));
    cmd_vel_out_.reset();
    cm_->receiveFreeVelocity(0.3);
    cm_->runControlLoop();
    ASSERT_TRUE(waitCmdVel(200ms));
    EXPECT_NEAR(cmd_vel_out_->linear.x, 0.3, EPSILON);
  }
  cm_->stop();
}

TEST_F(Tester, testScanNeverReceivedTimesOutAndStops)
{
  setCommonParameters();
  cm_->declare_parameter("scan_fault_timeout", 0.1);
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  addSource(SCAN_NAME, SCAN);
  setVectors({"PolygonStop"}, {SCAN_NAME});
  cm_->startManualControlLoop();

  cm_->receiveFreeVelocity(0.3);
  cm_->runControlLoop();
  ASSERT_TRUE(waitCmdVel(200ms));
  EXPECT_NEAR(cmd_vel_out_->linear.x, 0.3, EPSILON);

  std::this_thread::sleep_for(120ms);
  cmd_vel_out_.reset();
  obstacle_status_.reset();
  cm_->runControlLoop();
  ASSERT_TRUE(waitCmdVel(200ms));
  EXPECT_DOUBLE_EQ(cmd_vel_out_->linear.x, 0.0);
  ASSERT_NE(obstacle_status_, nullptr);
  EXPECT_EQ(obstacle_status_->status, nav_interfaces::msg::ObstacleMonitorStatus::STOP);
  cm_->stop();
}

TEST_F(Tester, testCostmapFootprintIsConsumedBeforeMotion)
{
  setCommonParameters();
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  cm_->declare_parameter("PolygonStop.use_footprint_topic", true);
  cm_->declare_parameter("PolygonStop.footprint_topic", FOOTPRINT_TOPIC);
  setVectors({"PolygonStop"}, {});
  cm_->startManualControlLoop();

  cm_->receiveFreeVelocity(0.3);
  cm_->runControlLoop();
  ASSERT_TRUE(waitCmdVel(200ms));
  EXPECT_DOUBLE_EQ(cmd_vel_out_->linear.x, 0.0);

  const auto stamp = cm_->now();
  publishFootprint(0.5, stamp);
  const auto deadline = std::chrono::steady_clock::now() + 500ms;
  bool updated = false;
  while (std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(cm_->get_node_base_interface());
    cm_->receiveFreeVelocity(0.3);
    cm_->runControlLoop();
    const auto points = cm_->polygonPoints("PolygonStop");
    if (points.size() == 4 && std::fabs(points.front().x - 0.5) < EPSILON) {
      updated = true;
      break;
    }
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(updated);
  cmd_vel_out_.reset();
  cm_->receiveFreeVelocity(0.3);
  cm_->runControlLoop();
  ASSERT_TRUE(waitCmdVel(200ms));
  EXPECT_NEAR(cmd_vel_out_->linear.x, 0.3, EPSILON);

  // A later costmap footprint update must replace the previous geometry as well.
  publishFootprint(0.6, cm_->now());
  updated = false;
  const auto second_deadline = std::chrono::steady_clock::now() + 500ms;
  while (std::chrono::steady_clock::now() < second_deadline) {
    rclcpp::spin_some(cm_->get_node_base_interface());
    cm_->receiveFreeVelocity(0.3);
    cm_->runControlLoop();
    const auto points = cm_->polygonPoints("PolygonStop");
    if (points.size() == 4 && std::fabs(points.front().x - 0.6) < EPSILON) {
      updated = true;
      break;
    }
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(updated);
  cm_->stop();
}

TEST_F(Tester, testInvalidNavigationTypeIsRejected)
{
  setCommonParameters();
  cm_->declare_parameter("navigation_type", "unknown");
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  setVectors({"PolygonStop"}, {});
  cm_->cant_configure();
}

TEST_F(Tester, testStateLogsDoNotRepeatForUnchangedInputs)
{
  setCommonParameters();
  cm_->declare_parameter("cmd_vel_timeout", 0.02);
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  setVectors({"PolygonStop"}, {});
  cm_->startManualControlLoop();
  testing::internal::CaptureStderr();
  for (int i = 0; i < 100; ++i) {
    cm_->receiveFreePath();
    cm_->receiveFreeVelocity(0.4);
    cm_->receiveScheduleVelocity(0.3);
    cm_->runControlLoop();
  }
  std::this_thread::sleep_for(30ms);
  for (int i = 0; i < 100; ++i) {
    cm_->runControlLoop();
  }
  cm_->receiveFreeVelocity(0.4);
  cm_->runControlLoop();
  const auto logs = testing::internal::GetCapturedStderr();
  const auto count = [&](const std::string & text) {
      std::size_t total = 0;
      for (auto pos = logs.find(text); pos != std::string::npos;
        pos = logs.find(text, pos + text.size()))
      {
        ++total;
      }
      return total;
    };
  EXPECT_EQ(count("[FSM] type locked:"), 0u);
  EXPECT_EQ(count("[FSM] ignoring inactive chain:"), 0u);
  EXPECT_EQ(count("[FSM] mode IDLE -> AUTOMONOUS_NAVIGATION"), 1u);
  EXPECT_EQ(count("[FSM] velocity=FRESH"), 2u);
  EXPECT_EQ(count("[FSM] velocity=STALE"), 1u);
  EXPECT_EQ(count("[FSM] idle zero published:"), 0u);
  cm_->stop();
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
