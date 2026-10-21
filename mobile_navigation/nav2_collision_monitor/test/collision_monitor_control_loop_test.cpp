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

TEST_F(Tester, testControlLoopScheduleBypassAndEdgingMode)
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
  // Schedule navigation is a passive consumer: path_follower receives obstacle_status and
  // remains the only component allowed to send velocity to velocity_smoother.
  EXPECT_FALSE(cmd_vel_out_);

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
  EXPECT_FALSE(cmd_vel_out_);

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
  std::this_thread::sleep_for(10ms);
  rclcpp::spin_some(cm_->get_node_base_interface());
  cmd_vel_out_.reset();
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
  cm_->set_parameter(rclcpp::Parameter("clear_hold_time", 0.0));
  cm_->set_parameter(rclcpp::Parameter("scan_fault_timeout", 0.2));
  cm_->set_parameter(rclcpp::Parameter("sensor_fault_timeout", 0.2));
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  const std::vector<std::string> names{"scan_a", "scan_b", "scan_c", "scan_d"};
  for (const auto & name : names) {
    addSource(name, SCAN);
  }
  setVectors({"PolygonStop"}, names);
  cm_->startManualControlLoop();
  sendTransforms(cm_->now());

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
    sendTransforms(cm_->now());
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
    cm_->receiveFreeVelocity(0.3);
    cm_->runControlLoop();
    ASSERT_TRUE(waitCmdVel(200ms));
    EXPECT_DOUBLE_EQ(cmd_vel_out_->linear.x, 0.0);
    ASSERT_NE(obstacle_status_, nullptr);
    EXPECT_EQ(obstacle_status_->status, nav_interfaces::msg::ObstacleMonitorStatus::STOP);

    const auto previous = cm_->scanReceiptNs(names[stale]);
    sendTransforms(cm_->now());
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
  cm_->set_parameter(rclcpp::Parameter("scan_fault_timeout", 0.1));
  cm_->set_parameter(rclcpp::Parameter("sensor_fault_timeout", 0.1));
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
  cm_->receiveFreeVelocity(0.3);
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
  cm_->set_parameter(rclcpp::Parameter("clear_hold_time", 0.0));
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

TEST_F(Tester, testHardStopPublishedOncePerStopEdge)
{
  setCommonParameters();
  cm_->set_parameter(rclcpp::Parameter("clear_hold_time", 0.0));
  cm_->set_parameter(rclcpp::Parameter("scan_fault_timeout", 0.1));
  cm_->set_parameter(rclcpp::Parameter("sensor_fault_timeout", 0.1));
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  addSource(SCAN_NAME, SCAN);
  setVectors({"PolygonStop"}, {SCAN_NAME});
  cm_->startManualControlLoop();
  sendTransforms(cm_->now());

  cm_->receiveFreeVelocity(0.3);
  cm_->runControlLoop();
  EXPECT_EQ(spinHardStops(50ms), 0);

  // scan 从未到达而超时：进入 STOP 发一次，STOP 持续期间不重复发。
  std::this_thread::sleep_for(120ms);
  for (int i = 0; i < 5; ++i) {
    cm_->receiveFreeVelocity(0.3);
    cm_->runControlLoop();
  }
  EXPECT_EQ(spinHardStops(100ms), 1);

  // scan 恢复且有新鲜速度后解除 STOP。
  publishScan(5.0, cm_->now());
  const auto deadline = std::chrono::steady_clock::now() + 500ms;
  while (cm_->scanReceiptNs(SCAN_NAME) == 0 && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(cm_->get_node_base_interface());
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_NE(cm_->scanReceiptNs(SCAN_NAME), 0);
  cmd_vel_out_.reset();
  cm_->receiveFreeVelocity(0.3);
  cm_->runControlLoop();
  ASSERT_TRUE(waitCmdVel(200ms));
  EXPECT_NEAR(cmd_vel_out_->linear.x, 0.3, EPSILON);
  EXPECT_EQ(spinHardStops(50ms), 1);

  // 再次进入 STOP 时重新发布。
  std::this_thread::sleep_for(120ms);
  cm_->receiveFreeVelocity(0.3);
  cm_->runControlLoop();
  cm_->receiveFreeVelocity(0.3);
  cm_->runControlLoop();
  EXPECT_EQ(spinHardStops(100ms), 2);
  cm_->stop();
}

TEST_F(Tester, testHardStopPublishedWhenFootprintMissing)
{
  setCommonParameters();
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  cm_->declare_parameter("PolygonStop.use_footprint_topic", true);
  cm_->declare_parameter("PolygonStop.footprint_topic", FOOTPRINT_TOPIC);
  setVectors({"PolygonStop"}, {});
  cm_->startManualControlLoop();

  for (int i = 0; i < 5; ++i) {
    cm_->receiveFreeVelocity(0.3);
    cm_->runControlLoop();
  }
  EXPECT_EQ(spinHardStops(100ms), 1);
  cm_->stop();
}

TEST_F(Tester, testAutoModeExposesNoScheduleInterfaces)
{
  setCommonParameters();
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  setVectors({"PolygonStop"}, {});
  cm_->configure();
  EXPECT_FALSE(cm_->hasPaddingService());
  EXPECT_FALSE(cm_->hasTrackingPathSubscription());
  EXPECT_TRUE(cm_->hasFreePathSubscription());
}

TEST_F(Tester, testScheduleModeExposesOnlyScheduleInterfaces)
{
  setCommonParameters();
  cm_->declare_parameter("navigation_type", "schedule");
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  setVectors({"PolygonStop"}, {});
  cm_->configure();
  EXPECT_TRUE(cm_->hasPaddingService());
  EXPECT_TRUE(cm_->hasTrackingPathSubscription());
  EXPECT_FALSE(cm_->hasFreePathSubscription());
}
TEST_F(Tester, testSpeedTierSelectionAndTimeout)
{
  setCommonParameters();
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  cm_->declare_parameter("PolygonStop.speed_tiers", std::vector<double>{0.20, 0.50, 0.70});
  cm_->declare_parameter("PolygonStop.front_padding_tiers", std::vector<double>{0.10, 0.30, 0.50});
  cm_->declare_parameter("tier_down_delay", 0.05);
  cm_->declare_parameter("speed_source_timeout", 0.1);
  setVectors({"PolygonStop"}, {});
  cm_->startManualControlLoop();

  // 1. 进入自动导航模式，初始里程计 0.10 m/s -> 低速档 (tier 0: front_padding = 0.10 m)
  cm_->receiveFreePath();
  cm_->receiveFreeVelocity(0.3);
  cm_->receiveOdom(0.10);
  cm_->runControlLoop();
  EXPECT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::AUTOMONOUS_NAVIGATION);
  EXPECT_EQ(cm_->currentSpeedTier(), 0u);
  EXPECT_NEAR(cm_->polygonFrontPadding("PolygonStop"), 0.10, EPSILON);

  // 2. 里程计 0.40 m/s -> 升档立即生效 (tier 1: front_padding = 0.30 m)
  cm_->receiveOdom(0.40);
  cm_->runControlLoop();
  EXPECT_EQ(cm_->currentSpeedTier(), 1u);
  EXPECT_NEAR(cm_->polygonFrontPadding("PolygonStop"), 0.30, EPSILON);

  // 3. 里程计 0.90 m/s -> 超出最高档封顶 (tier 2: front_padding = 0.50 m)
  cm_->receiveOdom(0.90);
  cm_->runControlLoop();
  EXPECT_EQ(cm_->currentSpeedTier(), 2u);
  EXPECT_NEAR(cm_->polygonFrontPadding("PolygonStop"), 0.50, EPSILON);

  // 4. 前向降档：降到 0.10 m/s (tier 0)，降档需满延时（0.05s）
  cm_->receiveOdom(0.10);
  cm_->runControlLoop();
  // 当周期不应立即降档
  EXPECT_EQ(cm_->currentSpeedTier(), 2u);
  EXPECT_NEAR(cm_->polygonFrontPadding("PolygonStop"), 0.50, EPSILON);

  // 5. 降档等待中回升，重新计时
  std::this_thread::sleep_for(20ms);
  cm_->receiveOdom(0.65);  // 回升到 tier 2
  cm_->runControlLoop();
  EXPECT_EQ(cm_->currentSpeedTier(), 2u);

  // 6. 再次降档并保持满延时
  cm_->receiveOdom(0.10);  // 目标 tier 0
  cm_->runControlLoop();
  EXPECT_EQ(cm_->currentSpeedTier(), 2u);
  std::this_thread::sleep_for(60ms);
  cm_->runControlLoop();
  EXPECT_EQ(cm_->currentSpeedTier(), 0u);
  EXPECT_NEAR(cm_->polygonFrontPadding("PolygonStop"), 0.10, EPSILON);

  // 7. 倒车方向（-0.45 m/s）：方向切换立即响应，匹配 tier 1，后向外扩 0.30 m，前向外扩清零
  cm_->receiveOdom(-0.45);
  cm_->runControlLoop();
  EXPECT_TRUE(cm_->isReverse());
  EXPECT_EQ(cm_->currentSpeedTier(), 1u);
  EXPECT_NEAR(cm_->polygonFrontPadding("PolygonStop"), 0.0, EPSILON);
  EXPECT_NEAR(cm_->polygonRearPadding("PolygonStop"), 0.30, EPSILON);

  // 8. 里程计超时 (>0.1s) -> 降级到最高档 (tier 2)
  std::this_thread::sleep_for(120ms);
  cm_->runControlLoop();
  EXPECT_EQ(cm_->currentSpeedTier(), 2u);
  EXPECT_NEAR(cm_->polygonRearPadding("PolygonStop"), 0.50, EPSILON);

  cm_->stop();
}

TEST_F(Tester, testSpeedTierReversePaddingAndObstacleTrigger)
{
  setCommonParameters();
  addPolygon("PolygonStop", POLYGON, 0.5, "stop");
  addSource("PointCloud", POINTCLOUD);
  cm_->declare_parameter("PolygonStop.speed_tiers", std::vector<double>{0.20, 0.50, 0.70});
  cm_->declare_parameter("PolygonStop.front_padding_tiers", std::vector<double>{0.10, 0.30, 0.50});
  cm_->declare_parameter("PolygonStop.rear_padding_tiers", std::vector<double>{0.10, 0.30, 0.50});
  setVectors({"PolygonStop"}, {"PointCloud"});
  cm_->startManualControlLoop();

  // 基础多边形是 0.5x0.5 ([-0.5, 0.5]^2)，uniform padding = 0.1
  // 在车尾正后方 x = -0.70 处放置点云障碍物。车尾边界 x_min = -0.5。
  // 在低速倒车档 (|vx| = 0.15 <= 0.20，rear_padding = 0.10)，停止区后沿为 -0.5 - 0.10 = -0.60，-0.70 处在停止区外。
  // 在中速倒车档 (|vx| = 0.40 <= 0.50，rear_padding = 0.30)，停止区后沿为 -0.5 - 0.30 = -0.80，-0.70 处在停止区内。
  const rclcpp::Time now = cm_->now();
  sendTransforms(now);
  publishPointCloud(-0.70, now);
  ASSERT_TRUE(waitData(std::hypot(-0.70, 0.01), 500ms, now));

  // 1. 低速倒车 (-0.15 m/s) -> rear_padding = 0.10 -> 障碍在区外 -> FREE
  cm_->receiveFreePath();
  cm_->receiveFreeVelocity(-0.2);
  cm_->receiveOdom(-0.15);
  cm_->runControlLoop();
  EXPECT_TRUE(cm_->isReverse());
  EXPECT_EQ(cm_->currentSpeedTier(), 0u);
  ASSERT_TRUE(waitObstacleStatus(nav_interfaces::msg::ObstacleMonitorStatus::FREE, 200ms));

  // 2. 升档至中速倒车 (-0.40 m/s) -> rear_padding = 0.30 -> -0.70 入区 -> 立即 STOP
  obstacle_status_.reset();
  cmd_vel_out_.reset();
  cm_->receiveOdom(-0.40);
  cm_->runControlLoop();
  EXPECT_TRUE(cm_->isReverse());
  EXPECT_EQ(cm_->currentSpeedTier(), 1u);
  ASSERT_TRUE(waitObstacleStatus(nav_interfaces::msg::ObstacleMonitorStatus::STOP, 200ms));
  EXPECT_EQ(obstacle_status_->zone_name, "PolygonStop[t1]");

  cm_->stop();
}

TEST_F(Tester, testSpeedTierObstacleTriggerOnTierUpAndZoneName)
{
  setCommonParameters();
  addPolygon("PolygonStop", POLYGON, 0.5, "stop");
  addSource("PointCloud", POINTCLOUD);
  cm_->declare_parameter("PolygonStop.speed_tiers", std::vector<double>{0.20, 0.50, 0.70});
  cm_->declare_parameter("PolygonStop.front_padding_tiers", std::vector<double>{0.10, 0.30, 0.50});
  setVectors({"PolygonStop"}, {"PointCloud"});
  cm_->startManualControlLoop();

  // 基础多边形是 0.5x0.5 ([-0.5, 0.5]^2)，uniform padding = 0.1
  // 在车头正前方 x = 0.70 处放置点云障碍物。车头边界 x_max = 0.5。
  // 在低速档 (front_padding = 0.10)，停止区前沿为 0.5 + 0.10 = 0.60，0.70 处在停止区外。
  // 在中速档 (front_padding = 0.30)，停止区前沿为 0.5 + 0.30 = 0.80，0.70 处在停止区内。
  const rclcpp::Time now = cm_->now();
  sendTransforms(now);
  publishPointCloud(0.70, now);
  ASSERT_TRUE(waitData(std::hypot(0.70, 0.01), 500ms, now));

  // 启动自动导航，初始里程计 0.15 m/s (低速档 tier 0)
  cm_->receiveFreePath();
  cm_->receiveFreeVelocity(0.3);
  cm_->receiveOdom(0.15);
  cm_->runControlLoop();
  EXPECT_EQ(cm_->currentSpeedTier(), 0u);
  ASSERT_TRUE(waitObstacleStatus(nav_interfaces::msg::ObstacleMonitorStatus::FREE, 200ms));
  ASSERT_TRUE(waitCmdVel(200ms));
  EXPECT_NEAR(cmd_vel_out_->linear.x, 0.3, EPSILON);

  // 只改里程计到 0.40 m/s（升到中速档 tier 1，无新速度，无新 scan/点云）
  // 当周期应升档，且档位变化并入 input_updated 触发重判，当周期输出 STOP！
  cmd_vel_out_.reset();
  cm_->receiveOdom(0.40);
  cm_->runControlLoop();
  EXPECT_EQ(cm_->currentSpeedTier(), 1u);
  ASSERT_TRUE(waitObstacleStatus(nav_interfaces::msg::ObstacleMonitorStatus::STOP, 200ms));
  // 验证 O6：上报的 zone_name 为 PolygonStop[t1]
  EXPECT_EQ(obstacle_status_->zone_name, "PolygonStop[t1]");
  ASSERT_TRUE(waitCmdVel(200ms));
  EXPECT_TRUE(cmd_vel_out_->linear.x == 0.0);

  cm_->stop();
}

TEST_F(Tester, testSpeedTierDisabledOrInScheduleMode)
{
  setCommonParameters();
  cm_->declare_parameter("navigation_type", "schedule");
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  cm_->declare_parameter("PolygonStop.speed_tiers", std::vector<double>{0.20, 0.50, 0.70});
  cm_->declare_parameter("PolygonStop.front_padding_tiers", std::vector<double>{0.10, 0.30, 0.50});
  setVectors({"PolygonStop"}, {});
  cm_->startManualControlLoop();

  // 调度模式下不分档，front_padding 保持 0.0
  const std::vector<nav2_collision_monitor::Point> route{{0.0, 0.0}, {1.0, 0.0}};
  cm_->receiveTrackingPath(route, 0.15F);
  cm_->receiveScheduleVelocity(0.3);
  cm_->receiveOdom(0.60);
  cm_->runControlLoop();

  EXPECT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::SCHEDULE_NAVIGATION);
  EXPECT_NEAR(cm_->polygonFrontPadding("PolygonStop"), 0.0, EPSILON);

  cm_->stop();
}

TEST_F(Tester, testSpeedTierLengthMismatch)
{
  setCommonParameters();
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  cm_->declare_parameter("PolygonStop.speed_tiers", std::vector<double>{0.20, 0.50});
  cm_->declare_parameter("PolygonStop.front_padding_tiers", std::vector<double>{0.10, 0.30, 0.50});
  setVectors({"PolygonStop"}, {});
  cm_->cant_configure();
}

TEST_F(Tester, testSpeedTierNotIncreasing)
{
  setCommonParameters();
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  cm_->declare_parameter("PolygonStop.speed_tiers", std::vector<double>{0.50, 0.20, 0.70});
  cm_->declare_parameter("PolygonStop.front_padding_tiers", std::vector<double>{0.10, 0.30, 0.50});
  setVectors({"PolygonStop"}, {});
  cm_->cant_configure();
}

TEST_F(Tester, testSpeedTierEmptyDisablesOdom)
{
  setCommonParameters();
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  cm_->declare_parameter("PolygonStop.speed_tiers", std::vector<double>{});
  cm_->declare_parameter("PolygonStop.front_padding_tiers", std::vector<double>{});
  setVectors({"PolygonStop"}, {});
  cm_->configure();
  EXPECT_FALSE(cm_->isOdomSubActive());
}

TEST_F(Tester, testStopHoldLatchesBeforeResuming)
{
  setCommonParameters();
  cm_->set_parameter(rclcpp::Parameter("clear_hold_time", 0.3));
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  addSource(SCAN_NAME, SCAN);
  setVectors({"PolygonStop"}, {SCAN_NAME});
  cm_->startManualControlLoop();

  // Geometry sources need the source->base transform before points can be converted.
  const rclcpp::Time curr_time = cm_->now();
  sendTransforms(curr_time);

  // An obstacle inside the stop polygon must latch STOP.
  publishScan(0.5, curr_time);
  {
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (cm_->scanReceiptNs(SCAN_NAME) == 0 && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(cm_->get_node_base_interface());
      std::this_thread::sleep_for(5ms);
    }
  }
  ASSERT_NE(cm_->scanReceiptNs(SCAN_NAME), 0);
  cm_->receiveFreeVelocity(0.3);
  cm_->runControlLoop();
  ASSERT_TRUE(waitObstacleStatus(nav_interfaces::msg::ObstacleMonitorStatus::STOP, 200ms));
  ASSERT_TRUE(waitCmdVel(200ms));
  EXPECT_DOUBLE_EQ(cmd_vel_out_->linear.x, 0.0);

  // Obstacle clears, but the stop is held for clear_hold_time before resuming.
  const std::int64_t previous_receipt = cm_->scanReceiptNs(SCAN_NAME);
  publishScan(5.0, cm_->now());
  {
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (cm_->scanReceiptNs(SCAN_NAME) == previous_receipt &&
      std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::spin_some(cm_->get_node_base_interface());
      std::this_thread::sleep_for(5ms);
    }
  }
  ASSERT_NE(cm_->scanReceiptNs(SCAN_NAME), previous_receipt);
  cmd_vel_out_.reset();
  cm_->receiveFreeVelocity(0.3);
  cm_->runControlLoop();
  EXPECT_TRUE(waitObstacleStatus(nav_interfaces::msg::ObstacleMonitorStatus::STOP, 150ms));
  // In auto mode the hold keeps commanding zero velocity; it must not resume the input.
  if (cmd_vel_out_) {
    EXPECT_DOUBLE_EQ(cmd_vel_out_->linear.x, 0.0);
    EXPECT_DOUBLE_EQ(cmd_vel_out_->angular.z, 0.0);
  }

  // After the hold window the monitor resumes with the latest command.
  std::this_thread::sleep_for(400ms);
  cmd_vel_out_.reset();
  cm_->receiveFreeVelocity(0.3);
  cm_->runControlLoop();
  EXPECT_TRUE(waitObstacleStatus(nav_interfaces::msg::ObstacleMonitorStatus::FREE, 200ms));
  ASSERT_TRUE(waitCmdVel(200ms));
  EXPECT_NEAR(cmd_vel_out_->linear.x, 0.3, EPSILON);

  cm_->stop();
}

TEST_F(Tester, testStopStatusAndFootprintMissingReported)
{
  using StatusMsg = nav_interfaces::msg::ObstacleMonitorStatus;
  setCommonParameters();
  cm_->set_parameter(rclcpp::Parameter("clear_hold_time", 0.0));
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  cm_->declare_parameter("PolygonStop.use_footprint_topic", true);
  cm_->declare_parameter("PolygonStop.footprint_topic", FOOTPRINT_TOPIC);
  addSource(SCAN_NAME, SCAN);
  setVectors({"PolygonStop"}, {SCAN_NAME});
  cm_->startManualControlLoop();

  const rclcpp::Time curr_time = cm_->now();
  sendTransforms(curr_time);

  // 1. 缺失 footprint 时触发 STOP
  cm_->receiveFreeVelocity(0.3);
  cm_->runControlLoop();
  ASSERT_TRUE(waitObstacleStatus(StatusMsg::STOP, 200ms));

  // 2. 收到有效 footprint，且远距离障碍 (5.0m) 无碰撞 -> FREE
  publishFootprint(0.5, curr_time);
  publishScan(5.0, curr_time);
  {
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (cm_->scanReceiptNs(SCAN_NAME) == 0 && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(cm_->get_node_base_interface());
      std::this_thread::sleep_for(5ms);
    }
  }
  cmd_vel_out_.reset();
  cm_->receiveFreeVelocity(0.3);
  cm_->runControlLoop();
  ASSERT_TRUE(waitObstacleStatus(StatusMsg::FREE, 200ms));

  // 3. 障碍物进入停障区 (0.3m) -> STOP，且带具体区域名
  const std::int64_t previous_receipt = cm_->scanReceiptNs(SCAN_NAME);
  publishScan(0.3, cm_->now());
  {
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (cm_->scanReceiptNs(SCAN_NAME) == previous_receipt &&
      std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::spin_some(cm_->get_node_base_interface());
      std::this_thread::sleep_for(5ms);
    }
  }
  cmd_vel_out_.reset();
  cm_->receiveFreeVelocity(0.3);
  cm_->runControlLoop();
  ASSERT_TRUE(waitObstacleStatus(StatusMsg::STOP, 200ms));
  EXPECT_FALSE(obstacle_status_->zone_name.empty());

  cm_->stop();
}

TEST_F(Tester, testEmptySceneWarnsWithoutStopping)
{
  using StatusMsg = nav_interfaces::msg::ObstacleMonitorStatus;
  setCommonParameters();
  cm_->set_parameter(rclcpp::Parameter("clear_hold_time", 0.0));
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  addSource(SCAN_NAME, SCAN);
  setVectors({"PolygonStop"}, {SCAN_NAME});
  cm_->startManualControlLoop();

  const rclcpp::Time curr_time = cm_->now();
  sendTransforms(curr_time);

  // 发布 ranges 全为 inf 的空旷 LaserScan
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = SOURCE_FRAME_ID;
  scan.header.stamp = curr_time;
  scan.range_min = 0.1F;
  scan.range_max = 10.0F;
  scan.ranges = {std::numeric_limits<float>::infinity()};
  scan_pub_->publish(scan);

  const auto deadline = std::chrono::steady_clock::now() + 500ms;
  while (cm_->scanReceiptNs(SCAN_NAME) == 0 && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(cm_->get_node_base_interface());
    std::this_thread::sleep_for(5ms);
  }

  cmd_vel_out_.reset();
  cm_->receiveFreeVelocity(0.3);
  cm_->runControlLoop();

  // 空旷无点只告警不停车：状态维持 FREE，且正常输出期望巡航速度 0.3 m/s
  ASSERT_TRUE(waitObstacleStatus(StatusMsg::FREE, 200ms));
  ASSERT_TRUE(waitCmdVel(200ms));
  EXPECT_NEAR(cmd_vel_out_->linear.x, 0.3, EPSILON);

  cm_->stop();
}

TEST_F(Tester, testTransformFailureTreatedAsSensorFault)
{
  using StatusMsg = nav_interfaces::msg::ObstacleMonitorStatus;
  setCommonParameters();
  cm_->set_parameter(rclcpp::Parameter("clear_hold_time", 0.0));
  cm_->set_parameter(rclcpp::Parameter("sensor_fault_timeout", 0.15));
  cm_->set_parameter(rclcpp::Parameter("scan_fault_timeout", 0.15));
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  addSource(SCAN_NAME, SCAN);
  setVectors({"PolygonStop"}, {SCAN_NAME});
  cm_->startManualControlLoop();

  // 先建立正常 TF 并注入一帧正常扫描建立基线
  sendTransforms(cm_->now());
  publishScan(5.0, cm_->now());
  {
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (cm_->scanReceiptNs(SCAN_NAME) == 0 && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(cm_->get_node_base_interface());
      std::this_thread::sleep_for(5ms);
    }
  }
  cm_->receiveFreeVelocity(0.3);
  cm_->runControlLoop();
  ASSERT_TRUE(waitObstacleStatus(StatusMsg::FREE, 200ms));

  // 持续发布雷达扫描，但时间戳推进到未来，使坐标系转换因 TF 外推超差失败
  std::this_thread::sleep_for(200ms);
  publishScan(5.0, cm_->now() + rclcpp::Duration::from_seconds(10.0));
  rclcpp::spin_some(cm_->get_node_base_interface());

  cm_->receiveFreeVelocity(0.3);
  cm_->runControlLoop();
  ASSERT_TRUE(waitObstacleStatus(StatusMsg::STOP, 200ms));

  cm_->stop();
}

TEST_F(Tester, testLifecycleActivationDeactivationZeroVelocity)
{
  setCommonParameters();
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  setVectors({"PolygonStop"}, {});

  // 1. 验证 on_activate 边界补发 1 帧安全零速
  cm_->configure();
  cmd_vel_out_.reset();
  cm_->activate();
  ASSERT_TRUE(waitCmdVel(200ms));
  EXPECT_DOUBLE_EQ(cmd_vel_out_->linear.x, 0.0);
  EXPECT_DOUBLE_EQ(cmd_vel_out_->angular.z, 0.0);

  // 2. 验证 on_deactivate 边界补发 1 帧安全零速
  cmd_vel_out_.reset();
  cm_->deactivate();
  ASSERT_TRUE(waitCmdVel(200ms));
  EXPECT_DOUBLE_EQ(cmd_vel_out_->linear.x, 0.0);
  EXPECT_DOUBLE_EQ(cmd_vel_out_->angular.z, 0.0);

  cm_->stop();
}

TEST_F(Tester, testIdleSensorFaultReportsStatusWithoutVelocity)
{
  using StatusMsg = nav_interfaces::msg::ObstacleMonitorStatus;
  setCommonParameters();
  cm_->set_parameter(rclcpp::Parameter("clear_hold_time", 0.0));
  cm_->set_parameter(rclcpp::Parameter("sensor_fault_timeout", 0.1));
  cm_->set_parameter(rclcpp::Parameter("scan_fault_timeout", 0.1));
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  addSource(SCAN_NAME, SCAN);
  setVectors({"PolygonStop"}, {SCAN_NAME});
  cm_->startManualControlLoop();

  // 确保处于 IDLE 模式（没有速度输入，也不在转发状态）
  std::this_thread::sleep_for(10ms);
  rclcpp::spin_some(cm_->get_node_base_interface());
  cmd_vel_out_.reset();

  // 等待传感器超时 (0.12s > 0.1s)
  std::this_thread::sleep_for(120ms);
  cm_->runControlLoop();

  // A6 核心断言：CM 空闲期传感器失效，必须正常上报 STOP 状态，但严禁向底盘发速度或清零
  ASSERT_TRUE(waitObstacleStatus(StatusMsg::STOP, 200ms));
  EXPECT_FALSE(waitCmdVel(50ms));
  EXPECT_EQ(spinHardStops(50ms), 0);

  cm_->stop();
}

// 9.3.1 UT-SLOW-01: 减速防抖保持时间与单帧丢波锁定测试
TEST_F(Tester, testSlowdownHoldTimeAndDebounceLock)
{
  using StatusMsg = nav_interfaces::msg::ObstacleMonitorStatus;
  setCommonParameters();
  cm_->set_parameter(rclcpp::Parameter("slowdown_hold_time", 1.0));
  cm_->set_parameter(rclcpp::Parameter("clear_hold_time", 0.0));

  addPolygon("PolygonStop", POLYGON, 0.2, "stop");
  addPolygon("PolygonSlow", POLYGON, 0.8, "slowdown");
  cm_->set_parameter(rclcpp::Parameter("PolygonSlow.slowdown_ratio", 0.5));
  cm_->set_parameter(rclcpp::Parameter("PolygonSlow.max_points", 0));

  addSource(SCAN_NAME, SCAN);
  setVectors({"PolygonStop", "PolygonSlow"}, {SCAN_NAME});
  cm_->startManualControlLoop();

  // 1. 初始时刻 t=0: 注入 0.6m/s 前进指令，注入 0.5m 障碍（落入 PolygonSlow，不在 PolygonStop）
  rclcpp::Time stamp = cm_->now();
  sendTransforms(stamp);
  cm_->receiveFreeVelocity(0.6);
  publishScan(0.5, stamp);
  ASSERT_TRUE(waitData(0.5, 500ms, stamp));

  cmd_vel_out_.reset();
  cm_->runControlLoop();

  // 断言首次触发 SLOWDOWN
  ASSERT_TRUE(waitObstacleStatus(StatusMsg::SLOWDOWN, 200ms));
  ASSERT_TRUE(waitCmdVel(200ms));
  EXPECT_NEAR(cmd_vel_out_->linear.x, 0.3, EPSILON);
  EXPECT_EQ(obstacle_status_->zone_name, "PolygonSlow");
  EXPECT_EQ(cm_->prevAction().action_type, nav2_collision_monitor::ActionType::SLOWDOWN);
  EXPECT_GT(cm_->lastSlowdownNs(), 0);

  // 2. 模拟探头丢波（单帧/连续数帧丢波，无障碍点），时间未满 1.0s
  stamp = cm_->now();
  sendTransforms(stamp);
  publishScan(2.0, stamp);  // 远距离无障碍
  ASSERT_TRUE(waitData(2.0, 500ms, stamp));

  std::this_thread::sleep_for(150ms);
  cm_->receiveFreeVelocity(0.6);
  cmd_vel_out_.reset();
  cm_->runControlLoop();

  // 断言：在保持期内维持 SLOWDOWN 锁定，严禁恢复 0.6m/s
  ASSERT_TRUE(waitObstacleStatus(StatusMsg::SLOWDOWN, 200ms));
  ASSERT_TRUE(waitCmdVel(200ms));
  EXPECT_NEAR(cmd_vel_out_->linear.x, 0.3, EPSILON);
  EXPECT_EQ(cm_->prevAction().action_type, nav2_collision_monitor::ActionType::SLOWDOWN);

  // 3. 保持期满（推进时间至大于 1.0s）
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  cm_->setLastSlowdownNs(now_ns - 1100000000LL);

  cm_->receiveFreeVelocity(0.6);
  cmd_vel_out_.reset();
  cm_->runControlLoop();

  // 断言：保持期满后平滑退出减速，速度恢复 0.6m/s，状态恢复 FREE
  ASSERT_TRUE(waitObstacleStatus(StatusMsg::FREE, 200ms));
  ASSERT_TRUE(waitCmdVel(200ms));
  EXPECT_NEAR(cmd_vel_out_->linear.x, 0.6, EPSILON);
  EXPECT_EQ(cm_->prevAction().action_type, nav2_collision_monitor::ActionType::DO_NOTHING);

  cm_->stop();
}

// 9.3.2 UT-SLOW-02: 减速保持期间触发更高优先级停障抢占测试
TEST_F(Tester, testSlowdownPreemptedByStop)
{
  using StatusMsg = nav_interfaces::msg::ObstacleMonitorStatus;
  setCommonParameters();
  cm_->set_parameter(rclcpp::Parameter("slowdown_hold_time", 1.0));
  cm_->set_parameter(rclcpp::Parameter("clear_hold_time", 0.0));

  addPolygon("PolygonStop", POLYGON, 0.2, "stop");
  addPolygon("PolygonSlow", POLYGON, 0.8, "slowdown");
  cm_->set_parameter(rclcpp::Parameter("PolygonSlow.slowdown_ratio", 0.5));
  cm_->set_parameter(rclcpp::Parameter("PolygonSlow.max_points", 0));
  cm_->set_parameter(rclcpp::Parameter("PolygonStop.max_points", 0));

  addSource(SCAN_NAME, SCAN);
  setVectors({"PolygonStop", "PolygonSlow"}, {SCAN_NAME});
  cm_->startManualControlLoop();

  // 1. 触发 SLOWDOWN 减速
  rclcpp::Time stamp = cm_->now();
  sendTransforms(stamp);
  cm_->receiveFreeVelocity(0.6);
  publishScan(0.5, stamp);
  ASSERT_TRUE(waitData(0.5, 500ms, stamp));

  cmd_vel_out_.reset();
  cm_->runControlLoop();
  ASSERT_TRUE(waitObstacleStatus(StatusMsg::SLOWDOWN, 200ms));
  ASSERT_TRUE(waitCmdVel(200ms));
  EXPECT_NEAR(cmd_vel_out_->linear.x, 0.3, EPSILON);
  EXPECT_EQ(cm_->prevAction().action_type, nav2_collision_monitor::ActionType::SLOWDOWN);
  EXPECT_GT(cm_->lastSlowdownNs(), 0);

  // 2. 在减速保持期内，近场突现障碍物（0.15m，落入 PolygonStop）
  publishScan(0.15, stamp);
  ASSERT_TRUE(waitData(0.15, 500ms, stamp));

  cm_->receiveFreeVelocity(0.6);
  cmd_vel_out_.reset();
  cm_->runControlLoop();

  // 断言：STOP 零延迟立即抢占 SLOWDOWN
  ASSERT_TRUE(waitObstacleStatus(StatusMsg::STOP, 200ms));
  ASSERT_TRUE(waitCmdVel(200ms));
  EXPECT_NEAR(cmd_vel_out_->linear.x, 0.0, EPSILON);
  EXPECT_EQ(cm_->prevAction().action_type, nav2_collision_monitor::ActionType::STOP);
  // 断言：last_slowdown_condition_steady_ns_ 被清零
  EXPECT_EQ(cm_->lastSlowdownNs(), 0);

  cm_->stop();
}

// 9.3.3 UT-POLY-03: PolygonSlow 独立 Padding 保持与生命周期测试
TEST_F(Tester, testPolygonSlowPaddingIndependenceAcrossLifecycle)
{
  setCommonParameters();
  addPolygon("PolygonStop", POLYGON, 0.5, "stop");
  cm_->declare_parameter("PolygonStop.default_padding", 0.2);
  cm_->declare_parameter("PolygonStop.automonous_navigation_padding", 0.1);

  addPolygon("PolygonSlow", POLYGON, 0.8, "slowdown");
  cm_->declare_parameter("PolygonSlow.default_padding", 0.1);
  cm_->declare_parameter("PolygonSlow.front_padding", 0.55);
  cm_->set_parameter(rclcpp::Parameter("PolygonSlow.slowdown_ratio", 0.5));

  setVectors({"PolygonStop", "PolygonSlow"}, {});
  cm_->startManualControlLoop();

  // 1. 验证启动后 PolygonSlow 的 front_padding 严格等于配置的 0.55m
  EXPECT_NEAR(cm_->polygonFrontPadding("PolygonSlow"), 0.55, EPSILON);

  // 2. 进入自由导航模式 (AUTOMONOUS_NAVIGATION)
  cm_->receiveFreePath();
  cm_->receiveFreeVelocity(0.4);
  cm_->runControlLoop();
  EXPECT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::AUTOMONOUS_NAVIGATION);

  // 断言：PolygonStop 生效自由导航 padding (0.1)，而 PolygonSlow front_padding 依然严格等于 0.55m，未被清零
  EXPECT_NEAR(cm_->polygonPadding("PolygonStop"), 0.1, EPSILON);
  EXPECT_NEAR(cm_->polygonFrontPadding("PolygonSlow"), 0.55, EPSILON);

  // 3. 切换至贴边模式 (SCHEDULE_EDGING)
  const std::vector<nav2_collision_monitor::Point> route{{0.0, 0.0}, {1.0, 0.0}};
  cm_->receiveTrackingPath(route, 0.0F);
  cm_->receiveScheduleVelocity(0.3);
  cm_->runControlLoop();

  // 断言：PolygonSlow front_padding 仍然保持 0.55m
  EXPECT_NEAR(cm_->polygonFrontPadding("PolygonSlow"), 0.55, EPSILON);

  cm_->stop();
}

// 9.3.4 UT-TIER-04: 升级版前向三档 Stop Padding 动态切换断言
TEST_F(Tester, testSpeedTiersAdaptiveStopPadding)
{
  setCommonParameters();
  cm_->declare_parameter("tier_down_delay", 0.0);
  cm_->declare_parameter("speed_source_timeout", 1.0);
  addPolygon("PolygonStop", POLYGON, 0.5, "stop");
  cm_->declare_parameter("PolygonStop.speed_tiers", std::vector<double>{0.35, 0.50, 0.70});
  cm_->declare_parameter("PolygonStop.front_padding_tiers", std::vector<double>{0.18, 0.26, 0.38});
  cm_->declare_parameter("PolygonStop.rear_padding_tiers", std::vector<double>{0.15, 0.20, 0.25});
  setVectors({"PolygonStop"}, {});
  cm_->startManualControlLoop();

  // 提前注入 odom 初始低速，避免未收到 odom 时默认落入超时的顶格最高档
  cm_->receiveOdom(0.20);

  // 激活自动导航模式
  cm_->receiveFreePath();
  cm_->receiveFreeVelocity(0.2);
  cm_->runControlLoop();
  EXPECT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::AUTOMONOUS_NAVIGATION);

  // 1. 低速档 tier 0: vx = 0.20 m/s <= 0.35 -> front_padding = 0.18m
  cm_->receiveOdom(0.20);
  cm_->runControlLoop();
  EXPECT_EQ(cm_->currentSpeedTier(), 0u);
  EXPECT_FALSE(cm_->isReverse());
  EXPECT_NEAR(cm_->polygonFrontPadding("PolygonStop"), 0.18, EPSILON);
  EXPECT_NEAR(cm_->polygonRearPadding("PolygonStop"), 0.0, EPSILON);

  // 2. 中速档 tier 1: vx = 0.40 m/s (0.35 < 0.40 <= 0.50) -> front_padding = 0.26m
  cm_->receiveOdom(0.40);
  cm_->runControlLoop();
  EXPECT_EQ(cm_->currentSpeedTier(), 1u);
  EXPECT_FALSE(cm_->isReverse());
  EXPECT_NEAR(cm_->polygonFrontPadding("PolygonStop"), 0.26, EPSILON);
  EXPECT_NEAR(cm_->polygonRearPadding("PolygonStop"), 0.0, EPSILON);

  // 3. 高速档 tier 2: vx = 0.60 m/s (0.50 < 0.60 <= 0.70) -> front_padding = 0.38m
  cm_->receiveOdom(0.60);
  cm_->runControlLoop();
  EXPECT_EQ(cm_->currentSpeedTier(), 2u);
  EXPECT_FALSE(cm_->isReverse());
  EXPECT_NEAR(cm_->polygonFrontPadding("PolygonStop"), 0.38, EPSILON);
  EXPECT_NEAR(cm_->polygonRearPadding("PolygonStop"), 0.0, EPSILON);

  // 4. 倒车自适应: vx = -0.40 m/s -> reverse = true, front_padding = 0.0, rear_padding = 0.20m
  cm_->receiveOdom(-0.40);
  cm_->runControlLoop();
  EXPECT_TRUE(cm_->isReverse());
  EXPECT_EQ(cm_->currentSpeedTier(), 1u);
  EXPECT_NEAR(cm_->polygonFrontPadding("PolygonStop"), 0.0, EPSILON);
  EXPECT_NEAR(cm_->polygonRearPadding("PolygonStop"), 0.20, EPSILON);

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
