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

  cm_->receiveTrackingPath(route, 0.15F);
  cm_->receiveScheduleVelocity(0.3);
  cm_->runControlLoop();
  EXPECT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::SCHEDULE_NAVIGATION);
  EXPECT_FALSE(cm_->polygonLateralPaddingOnly("PolygonStop"));
  EXPECT_FALSE(cmd_vel_out_);

  cm_->stop();
}

TEST_F(Tester, testFootprintSwitchPreservesScheduleEdgingMode)
{
  setCommonParameters();
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  cm_->declare_parameter("PolygonStop.default_padding", 0.2);
  setVectors({"PolygonStop"}, {});
  cm_->startManualControlLoop();

  const std::vector<nav2_collision_monitor::Point> route{{0.0, 0.0}, {1.0, 0.0}};
  cm_->receiveTrackingPath(route, 0.0F);
  cm_->receiveScheduleVelocity(0.3);
  cm_->runControlLoop();
  ASSERT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::SCHEDULE_EDGING);
  ASSERT_TRUE(cm_->polygonLateralPaddingOnly("PolygonStop"));

  // A footprint change intentionally publishes one zero command and enters IDLE
  // transiently, but a fresh command must resume the sub-mode selected by the path.
  cm_->requestLoadedFootprint(true);
  cm_->runControlLoop();
  EXPECT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::SCHEDULE_EDGING);
  EXPECT_TRUE(cm_->polygonLateralPaddingOnly("PolygonStop"));
  EXPECT_NEAR(cm_->polygonPadding("PolygonStop"), 0.2, EPSILON);
  ASSERT_TRUE(waitCmdVel(200ms));
  EXPECT_DOUBLE_EQ(cmd_vel_out_->linear.x, 0.0);

  cmd_vel_out_.reset();
  cm_->requestLoadedFootprint(false);
  cm_->receiveScheduleVelocity(0.3);
  cm_->runControlLoop();
  EXPECT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::SCHEDULE_EDGING);
  EXPECT_TRUE(cm_->polygonLateralPaddingOnly("PolygonStop"));
  EXPECT_NEAR(cm_->polygonPadding("PolygonStop"), 0.2, EPSILON);
  ASSERT_TRUE(waitCmdVel(200ms));
  EXPECT_DOUBLE_EQ(cmd_vel_out_->linear.x, 0.0);

  cm_->stop();
}

TEST_F(Tester, testTrackingPathNonFinitePaddingUsesDefault)
{
  setCommonParameters();
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

TEST_F(Tester, testControlLoopAppliesQueuedLoadedFootprint)
{
  setCommonParameters();
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  setVectors({"PolygonStop"}, {});
  cm_->startManualControlLoop();

  auto points = cm_->polygonPoints("PolygonStop");
  ASSERT_EQ(points.size(), 4u);
  EXPECT_NEAR(points.front().x, 0.35, EPSILON);
  EXPECT_NEAR(points.front().y, 0.29, EPSILON);

  // The service callback acknowledges and queues the request; geometry is replaced only
  // in the control thread so polygon readers always observe a complete footprint.
  cm_->requestLoadedFootprint(true);
  points = cm_->polygonPoints("PolygonStop");
  EXPECT_NEAR(points.front().x, 0.35, EPSILON);

  cm_->runControlLoop();
  points = cm_->polygonPoints("PolygonStop");
  ASSERT_EQ(points.size(), 4u);
  EXPECT_NEAR(points.front().x, 0.50, EPSILON);
  EXPECT_NEAR(points.front().y, 0.40, EPSILON);
  EXPECT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::IDLE);

  cm_->requestLoadedFootprint(false);
  cm_->runControlLoop();
  points = cm_->polygonPoints("PolygonStop");
  ASSERT_EQ(points.size(), 4u);
  EXPECT_NEAR(points.front().x, 0.35, EPSILON);
  EXPECT_NEAR(points.front().y, 0.29, EPSILON);

  cm_->stop();
}

TEST_F(Tester, testFootprintPublishedToBothCostmaps)
{
  setCommonParameters();
  addPolygon("PolygonStop", POLYGON, 1.0, "stop");
  setVectors({"PolygonStop"}, {});
  const std::vector<std::string> topics{
    "test_local_costmap/footprint", "test_global_costmap/footprint"};
  cm_->declare_parameter("costmap_footprint_topics", topics);
  std::vector<geometry_msgs::msg::Polygon::SharedPtr> received(2);
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr> subscriptions;
  for (std::size_t i = 0; i < topics.size(); ++i) {
    subscriptions.push_back(cm_->create_subscription<geometry_msgs::msg::Polygon>(
      topics[i], rclcpp::SystemDefaultsQoS(),
      [&, i](geometry_msgs::msg::Polygon::SharedPtr msg) {received[i] = msg;}));
  }
  const auto wait_footprints = [&]() {
      const auto deadline = std::chrono::steady_clock::now() + 500ms;
      while (std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(cm_->get_node_base_interface());
        if (received[0] && received[1]) {
          return true;
        }
        std::this_thread::sleep_for(10ms);
      }
      return false;
    };
  const auto check_footprints = [&](double x, double y) {
      for (const auto & footprint : received) {
        ASSERT_NE(footprint, nullptr);
        ASSERT_EQ(footprint->points.size(), 4u);
        EXPECT_NEAR(footprint->points[0].x, x, EPSILON);
        EXPECT_NEAR(footprint->points[0].y, y, EPSILON);
        EXPECT_NEAR(footprint->points[1].x, x, EPSILON);
        EXPECT_NEAR(footprint->points[1].y, -y, EPSILON);
        EXPECT_NEAR(footprint->points[2].x, -x, EPSILON);
        EXPECT_NEAR(footprint->points[2].y, -y, EPSILON);
        EXPECT_NEAR(footprint->points[3].x, -x, EPSILON);
        EXPECT_NEAR(footprint->points[3].y, y, EPSILON);
      }
    };

  cm_->startManualControlLoop();
  ASSERT_TRUE(wait_footprints());
  check_footprints(0.35, 0.29);

  // FREE mode applies detection padding, which must not reach the costmaps.
  cm_->receiveFreeVelocity(0.4);
  cm_->runControlLoop();
  EXPECT_GT(cm_->polygonPadding("PolygonStop"), 0.0);
  received.assign(2, nullptr);
  cm_->requestLoadedFootprint(true);
  rclcpp::spin_some(cm_->get_node_base_interface());
  EXPECT_FALSE(received[0]);
  EXPECT_FALSE(received[1]);
  cm_->runControlLoop();
  ASSERT_TRUE(wait_footprints());
  check_footprints(0.50, 0.40);

  received.assign(2, nullptr);
  cm_->requestLoadedFootprint(false);
  cm_->runControlLoop();
  ASSERT_TRUE(wait_footprints());
  check_footprints(0.35, 0.29);
  cm_->stop();
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
  EXPECT_EQ(count("[FSM] type locked:"), 1u);
  EXPECT_EQ(count("[FSM] ignoring inactive chain:"), 1u);
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
