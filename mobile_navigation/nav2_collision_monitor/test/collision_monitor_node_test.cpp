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

TEST_F(Tester, testProcessStopSlowdown)
{
  rclcpp::Time curr_time = cm_->now();

  // Set Collision Monitor parameters.
  // Making two polygons: outer polygon for slowdown and inner for robot stop.
  setCommonParameters();
  // This test focuses on STOP/SLOWDOWN transitions, so disable the minimum
  // stop-hold to keep resume timing deterministic.
  cm_->declare_parameter("stop_hold_time", rclcpp::ParameterValue(0.0));
  addPolygon("SlowDown", POLYGON, 2.0, "slowdown");
  addPolygon("Stop", POLYGON, 1.0, "stop");
  addSource(SCAN_NAME, SCAN);
  setVectors({"SlowDown", "Stop"}, {SCAN_NAME});

  // Start Collision Monitor node
  cm_->start();

  // Share TF
  sendTransforms(curr_time);

  // 1. Obstacle is far away from robot
  publishScan(3.0, curr_time);
  ASSERT_TRUE(waitData(3.0, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.5, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.2, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.1, EPSILON);

  // 2. Obstacle is in slowdown robot zone
  publishScan(1.5, curr_time);
  ASSERT_TRUE(waitData(1.5, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.5 * SLOWDOWN_RATIO, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.2 * SLOWDOWN_RATIO, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.1 * SLOWDOWN_RATIO, EPSILON);

  // 3. Obstacle is inside stop zone
  publishScan(0.5, curr_time);
  ASSERT_TRUE(waitData(0.5, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);
  // 进入停止区时通知平滑器硬停一次
  EXPECT_EQ(spinHardStops(100ms), 1);

  // 4. Restoring back normal operation
  publishScan(3.0, curr_time);
  ASSERT_TRUE(waitData(3.0, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.5, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.2, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.1, EPSILON);
  // 离开停止区不再发硬停
  EXPECT_EQ(spinHardStops(50ms), 1);

  // Stop Collision Monitor node
  cm_->stop();
}

TEST_F(Tester, testTrackingPathAheadStop)
{
  const rclcpp::Time curr_time = cm_->now();
  setCommonParameters();
  // This test focuses on STOP/SLOWDOWN transitions, so disable the minimum
  // stop-hold to keep resume timing deterministic.
  cm_->declare_parameter("stop_hold_time", rclcpp::ParameterValue(0.0));
  cm_->declare_parameter("navigation_type", "schedule");
  addPolygon("PolygonStop", CIRCLE, 0.2, "stop");
  addSource(POINTCLOUD_NAME, POINTCLOUD);
  setVectors({"PolygonStop"}, {POINTCLOUD_NAME});
  cm_->declare_parameter("PolygonStop.path_stop_enabled", true);
  cm_->declare_parameter("PolygonStop.path_stop_distance", 1.0);
  cm_->declare_parameter("PolygonStop.path_stop_radius", 0.2);
  cm_->declare_parameter("PolygonStop.path_point_interval", 0.10);
  cm_->start();
  sendTransforms(curr_time);

  // Original route points spaced by more than 10 cm are selected. Two cloud points at x=0.8
  // lie inside the translated PolygonStop circle and exceed its max_points threshold: STOP.
  const std::vector<nav2_collision_monitor::Point> route{
    {0.0, 0.0}, {0.4, 0.0}, {0.8, 0.0}, {1.2, 0.0}, {1.6, 0.0}, {2.0, 0.0}};
  cm_->receiveTrackingPath(route);
  publishPointCloud(0.8, curr_time);
  ASSERT_TRUE(waitData(std::hypot(0.8, 0.01), 500ms, curr_time));
  publishCmdVel(0.5, 0.0, 0.0);
  ASSERT_TRUE(waitObstacleStatus(nav_interfaces::msg::ObstacleMonitorStatus::STOP, 500ms));
  // Schedule navigation is a passive consumer: it reports obstacle_status, but never
  // sends velocity to the smoother, so cmd_vel_out stays empty.
  EXPECT_FALSE(waitCmdVel(100ms));

  // Ordinary schedule navigation must ignore the corridor obstacle at the same position.
  cm_->receiveTrackingPath(route, 0.15F);
  publishCmdVel(0.5, 0.0, 0.0);
  ASSERT_TRUE(waitObstacleStatus(nav_interfaces::msg::ObstacleMonitorStatus::FREE, 500ms));
  EXPECT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::SCHEDULE_NAVIGATION);
  // Schedule navigation is a passive consumer: it reports obstacle_status, but never
  // sends velocity to the smoother, so cmd_vel_out stays empty.
  EXPECT_FALSE(waitCmdVel(100ms));

  // Returning to edging enables corridor detection again.
  cm_->receiveTrackingPath(route, 0.0F);
  publishCmdVel(0.5, 0.0, 0.0);
  ASSERT_TRUE(waitObstacleStatus(nav_interfaces::msg::ObstacleMonitorStatus::STOP, 500ms));
  EXPECT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::SCHEDULE_EDGING);
  // Schedule navigation is a passive consumer: it reports obstacle_status, but never
  // sends velocity to the smoother, so cmd_vel_out stays empty.
  EXPECT_FALSE(waitCmdVel(100ms));

  // The same obstacle is outside a parallel route corridor: velocity is passed through.
  cm_->receiveTrackingPath(
  {
    {0.0, 1.0}, {0.4, 1.0}, {0.8, 1.0},
    {1.2, 1.0}, {1.6, 1.0}, {2.0, 1.0}});
  publishCmdVel(0.5, 0.0, 0.0);
  ASSERT_TRUE(waitObstacleStatus(nav_interfaces::msg::ObstacleMonitorStatus::FREE, 500ms));
  // Schedule navigation is a passive consumer: it reports obstacle_status, but never
  // sends velocity to the smoother, so cmd_vel_out stays empty.
  EXPECT_FALSE(waitCmdVel(100ms));

  // x=1.6 is beyond the 1 m lookahead and the 0.2 m PolygonStop radius, so it must not stop.
  cm_->receiveTrackingPath(route);
  publishPointCloud(1.6, curr_time);
  ASSERT_TRUE(waitData(std::hypot(1.6, 0.01), 500ms, curr_time));
  publishCmdVel(0.5, 0.0, 0.0);
  ASSERT_TRUE(waitObstacleStatus(nav_interfaces::msg::ObstacleMonitorStatus::FREE, 500ms));
  // Schedule navigation is a passive consumer: it reports obstacle_status, but never
  // sends velocity to the smoother, so cmd_vel_out stays empty.
  EXPECT_FALSE(waitCmdVel(100ms));

  cm_->stop();
}

TEST_F(Tester, testProcessApproach)
{
  rclcpp::Time curr_time = cm_->now();

  // Set Collision Monitor parameters.
  // Making one circle footprint for approach.
  setCommonParameters();
  addPolygon("Approach", CIRCLE, 1.0, "approach");
  addSource(SCAN_NAME, SCAN);
  setVectors({"Approach"}, {SCAN_NAME});

  // Start Collision Monitor node
  cm_->start();

  // Share TF
  sendTransforms(curr_time);

  // 1. Obstacle is far away from robot
  publishScan(3.0, curr_time);
  ASSERT_TRUE(waitData(3.0, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.5, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.2, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);

  // 2. Approaching obstacle (0.2 m ahead from robot footprint)
  publishScan(1.2, curr_time);
  ASSERT_TRUE(waitData(1.2, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  // change_ratio = (0.2 m / speed m/s) / TIME_BEFORE_COLLISION s
  // where speed = sqrt(0.5^2 + 0.2^2) ~= 0.538516481 m/s
  const double change_ratio = (0.2 / 0.538516481) / TIME_BEFORE_COLLISION;
  ASSERT_NEAR(
    cmd_vel_out_->linear.x, 0.5 * change_ratio, 0.5 * SIMULATION_TIME_STEP / TIME_BEFORE_COLLISION);
  ASSERT_NEAR(
    cmd_vel_out_->linear.y, 0.2 * change_ratio, 0.2 * SIMULATION_TIME_STEP / TIME_BEFORE_COLLISION);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);

  // 3. Obstacle is inside robot footprint
  publishScan(0.5, curr_time);
  ASSERT_TRUE(waitData(0.5, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);

  // 4. Restoring back normal operation
  publishScan(3.0, curr_time);
  ASSERT_TRUE(waitData(3.0, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.5, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.2, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);

  // Stop Collision Monitor node
  cm_->stop();
}

TEST_F(Tester, testApproachPublishesChangedVelocityWithoutNewCommand)
{
  setCommonParameters();
  // Keep the single input fresh while sensor messages are delivered by the executor.
  cm_->declare_parameter("cmd_vel_timeout", 2.0);
  addPolygon("Approach", CIRCLE, 1.0, "approach");
  addSource(SCAN_NAME, SCAN);
  setVectors({"Approach"}, {SCAN_NAME});
  cm_->startManualControlLoop();
  const auto stamp = cm_->now();
  sendTransforms(stamp);
  publishScan(1.2, stamp);
  ASSERT_TRUE(waitData(1.2, 500ms, stamp));
  cm_->receiveFreeVelocity(0.5);
  cm_->runControlLoop();
  ASSERT_TRUE(waitCmdVel(200ms));
  ASSERT_GT(cmd_vel_out_->linear.x, 0.0);
  ASSERT_LT(cmd_vel_out_->linear.x, 0.5);

  // The action remains APPROACH, but the obstacle now requires a complete stop.
  cmd_vel_out_.reset();
  publishScan(0.5, stamp);
  ASSERT_TRUE(waitData(0.5, 500ms, stamp));
  cm_->runControlLoop();
  ASSERT_TRUE(waitCmdVel(200ms));
  EXPECT_DOUBLE_EQ(cmd_vel_out_->linear.x, 0.0);
  EXPECT_DOUBLE_EQ(cmd_vel_out_->linear.y, 0.0);
  EXPECT_DOUBLE_EQ(cmd_vel_out_->angular.z, 0.0);
  cm_->stop();
}

TEST_F(Tester, testProcessApproachRotation)
{
  rclcpp::Time curr_time = cm_->now();

  // Set Collision Monitor parameters.
  // Making one circle footprint for approach.
  setCommonParameters();
  addPolygon("Approach", POLYGON, 1.0, "approach");
  addSource(RANGE_NAME, RANGE);
  setVectors({"Approach"}, {RANGE_NAME});

  // Start Collision Monitor node
  cm_->start();

  // Publish robot footprint
  publishFootprint(1.0, curr_time);

  // Share TF
  sendTransforms(curr_time);

  // 1. Obstacle is far away from robot
  publishRange(2.0, curr_time);
  ASSERT_TRUE(waitData(2.0, 500ms, curr_time));
  publishCmdVel(0.0, 0.0, M_PI / 4);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, M_PI / 4, EPSILON);

  // 2. Approaching rotation to obstacle ( M_PI / 4 - M_PI / 20 ahead from robot)
  publishRange(1.4, curr_time);
  ASSERT_TRUE(waitData(1.4, 500ms, curr_time));
  publishCmdVel(0.0, 0.0, M_PI / 4);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(
    cmd_vel_out_->linear.x, 0.0, EPSILON);
  ASSERT_NEAR(
    cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(
    cmd_vel_out_->angular.z,
    M_PI / 5,
    (M_PI / 4) * (SIMULATION_TIME_STEP / TIME_BEFORE_COLLISION));

  // 3. Obstacle is inside robot footprint
  publishRange(0.5, curr_time);
  ASSERT_TRUE(waitData(0.5, 500ms, curr_time));
  publishCmdVel(0.0, 0.0, M_PI / 4);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);

  // 4. Restoring back normal operation
  publishRange(2.5, curr_time);
  ASSERT_TRUE(waitData(2.5, 500ms, curr_time));
  publishCmdVel(0.0, 0.0, M_PI / 4);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, M_PI / 4, EPSILON);

  // Stop Collision Monitor node
  cm_->stop();
}

TEST_F(Tester, testCrossOver)
{
  rclcpp::Time curr_time = cm_->now();

  // Set Collision Monitor parameters.
  // Making two polygons: outer polygon for slowdown and inner circle
  // as robot footprint for approach.
  setCommonParameters();
  addPolygon("SlowDown", POLYGON, 2.0, "slowdown");
  addPolygon("Approach", CIRCLE, 1.0, "approach");
  addSource(POINTCLOUD_NAME, POINTCLOUD);
  addSource(RANGE_NAME, RANGE);
  setVectors({"SlowDown", "Approach"}, {POINTCLOUD_NAME, RANGE_NAME});

  // Start Collision Monitor node
  cm_->start();

  // Share TF
  sendTransforms(curr_time);

  // 1. Obstacle is not in the slowdown zone, but less than TIME_BEFORE_COLLISION (ahead in 1.5 m).
  // Robot should approach the obstacle.
  publishPointCloud(2.5, curr_time);
  ASSERT_TRUE(waitData(std::hypot(2.5, 0.01), 500ms, curr_time));
  publishCmdVel(3.0, 0.0, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  // change_ratio = (1.5 m / 3.0 m/s) / TIME_BEFORE_COLLISION s
  double change_ratio = (1.5 / 3.0) / TIME_BEFORE_COLLISION;
  ASSERT_NEAR(
    cmd_vel_out_->linear.x, 3.0 * change_ratio, 3.0 * SIMULATION_TIME_STEP / TIME_BEFORE_COLLISION);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);

  // 2. Obstacle is inside slowdown zone, but speed is too slow for approach
  publishRange(1.5, curr_time);
  ASSERT_TRUE(waitData(1.5, 500ms, curr_time));
  publishCmdVel(0.1, 0.0, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.1 * SLOWDOWN_RATIO, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);

  // 3. Increase robot speed to return again into approach mode.
  // The speed should be safer for approach mode, so robot will go to the approach (ahead in 0.5 m)
  // even while it is already inside slowdown area.
  publishCmdVel(1.0, 0.0, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  // change_ratio = (0.5 m / 1.0 m/s) / TIME_BEFORE_COLLISION s
  change_ratio = (0.5 / 1.0) / TIME_BEFORE_COLLISION;
  ASSERT_NEAR(
    cmd_vel_out_->linear.x, 1.0 * change_ratio, 1.0 * SIMULATION_TIME_STEP / TIME_BEFORE_COLLISION);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);

  // Stop Collision Monitor node
  cm_->stop();
}

TEST_F(Tester, testCeasePublishZeroVel)
{
  rclcpp::Time curr_time = cm_->now();

  // Configure stop and approach zones, and basic data source
  setCommonParameters();
  addPolygon("Stop", POLYGON, 1.0, "stop");
  addPolygon("Approach", CIRCLE, 2.0, "approach");
  addSource(SCAN_NAME, SCAN);
  setVectors({"Stop", "Approach"}, {SCAN_NAME});

  // Start Collision Monitor node
  cm_->start();

  // Share TF
  sendTransforms(curr_time);

  // 1. Obstacle is inside approach footprint: robot should stop
  publishScan(1.5, curr_time);
  ASSERT_TRUE(waitData(1.5, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);

  // Wait more than STOP_PUB_TIMEOUT time
  std::this_thread::sleep_for(std::chrono::duration<double>(STOP_PUB_TIMEOUT + 0.01));

  // 2. Check that zero cmd_vel_out velocity won't be published more for this case
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_FALSE(waitCmdVel(100ms));

  // 3. Release robot to normal operation
  publishScan(3.0, curr_time);
  ASSERT_TRUE(waitData(3.0, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.5, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.2, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.1, EPSILON);

  // 4. Obstacle is inside stop zone
  publishScan(0.5, curr_time);
  ASSERT_TRUE(waitData(0.5, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);

  // Wait more than STOP_PUB_TIMEOUT time
  std::this_thread::sleep_for(std::chrono::duration<double>(STOP_PUB_TIMEOUT + 0.01));

  // 5. Check that zero cmd_vel_out velocity won't be published more for this case
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_FALSE(waitCmdVel(100ms));

  // Stop Collision Monitor node
  cm_->stop();
}

TEST_F(Tester, testProcessNonActive)
{
  rclcpp::Time curr_time = cm_->now();

  setCommonParameters();
  addPolygon("Stop", POLYGON, 1.0, "stop");
  addSource(SCAN_NAME, SCAN);
  setVectors({"Stop"}, {SCAN_NAME});

  // Configure Collision Monitor node, but not activate
  cm_->configure();

  // Share TF
  sendTransforms(curr_time);

  // Call CollisionMonitor::process()
  publishCmdVel(1.0, 0.0, 0.0);
  // ... and check that cmd_vel_out was not published
  ASSERT_FALSE(waitCmdVel(100ms));

  // Stop Collision Monitor node
  cm_->stop();
}

TEST_F(Tester, testIncorrectPolygonType)
{
  setCommonParameters();
  addPolygon("UnknownShape", POLYGON_UNKNOWN, 1.0, "stop");
  addSource(SCAN_NAME, SCAN);
  setVectors({"UnknownShape"}, {SCAN_NAME});

  // Check that Collision Monitor node can not be configured for this parameters set
  cm_->cant_configure();
}

TEST_F(Tester, testIncorrectSourceType)
{
  setCommonParameters();
  addPolygon("Stop", POLYGON, 1.0, "stop");
  addSource("UnknownSource", SOURCE_UNKNOWN);
  setVectors({"Stop"}, {"UnknownSource"});

  // Check that Collision Monitor node can not be configured for this parameters set
  cm_->cant_configure();
}

TEST_F(Tester, testPolygonsNotSet)
{
  setCommonParameters();
  addPolygon("Stop", POLYGON, 1.0, "stop");
  addSource(SCAN_NAME, SCAN);

  // Check that Collision Monitor node can not be configured for this parameters set
  cm_->cant_configure();
}

TEST_F(Tester, testSourcesNotSet)
{
  setCommonParameters();
  addPolygon("Stop", POLYGON, 1.0, "stop");
  addSource(SCAN_NAME, SCAN);
  cm_->declare_parameter("polygons", rclcpp::ParameterValue("Stop"));
  cm_->set_parameter(rclcpp::Parameter("polygons", std::vector<std::string>{"Stop"}));

  // Check that Collision Monitor node can not be configured for this parameters set
  cm_->cant_configure();
}

TEST_F(Tester, testRangeStopSingleHit)
{
  rclcpp::Time curr_time = cm_->now();

  setCommonParameters();
  // This test focuses on STOP/SLOWDOWN transitions, so disable the minimum
  // stop-hold to keep resume timing deterministic.
  cm_->declare_parameter("stop_hold_time", rclcpp::ParameterValue(0.0));
  addPolygon("Stop", POLYGON, 1.0, "stop");
  addSource(RANGE_NAME, RANGE);
  setVectors({"Stop"}, {RANGE_NAME});

  // Range 已取消 min_consecutive_hits 门槛：单帧有效回波即触发障碍物。

  // Start Collision Monitor node
  cm_->start();

  // Share TF
  sendTransforms(curr_time);

  // Helper lambda to spin for range data arrival
  auto spinWait = [&](std::chrono::milliseconds ms) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < ms) {
      rclcpp::spin_some(cm_->get_node_base_interface());
      std::this_thread::sleep_for(10ms);
    }
  };

  // 1. Frame 1: obstacle at 0.5m (inside 1.0m stop polygon) -> stop immediately.
  publishRange(0.5, curr_time);
  ASSERT_TRUE(waitData(0.5, 500ms, curr_time));
  publishCmdVel(0.5, 0.0, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.0, EPSILON);

  // 2. Clear obstacle to 2.0m (outside 1.0m stop polygon) -> normal operation.
  publishRange(2.0, curr_time);
  spinWait(50ms);
  publishRange(2.0, curr_time);
  spinWait(50ms);
  publishCmdVel(0.5, 0.0, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.5, EPSILON);

  // Stop Collision Monitor node
  cm_->stop();
}

TEST_F(Tester, testRangeSlowdownConfigurable)
{
  rclcpp::Time curr_time = cm_->now();

  setCommonParameters();
  addPolygon("Slow", POLYGON, 1.0, "slowdown");
  addSource(RANGE_NAME, RANGE);
  setVectors({"Slow"}, {RANGE_NAME});

  // 配置 Range 允许进入减速模块 (stop_only = false)
  cm_->declare_parameter(std::string(RANGE_NAME) + ".stop_only", false);

  // Start Collision Monitor node
  cm_->start();
  sendTransforms(curr_time);

  auto spinWait = [&](std::chrono::milliseconds ms) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < ms) {
      rclcpp::spin_some(cm_->get_node_base_interface());
      std::this_thread::sleep_for(10ms);
    }
  };

  // 连续 3 帧发送 0.5m 障碍物（落在 1.0m 的 Slowdown 多边形内）
  publishRange(0.5, curr_time);
  spinWait(50ms);
  publishRange(0.5, curr_time);
  spinWait(50ms);
  publishRange(0.5, curr_time);
  ASSERT_TRUE(waitData(0.5, 500ms, curr_time));

  // 输入 1.0 m/s 速度，预期按照 slowdown_ratio (测试默认 0.7) 减速为 0.7 m/s
  publishCmdVel(1.0, 0.0, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.7, EPSILON);

  cm_->stop();
}

TEST_F(Tester, testRangePointCloudPublishOnSubscription)
{
  rclcpp::Time curr_time = cm_->now();

  setCommonParameters();
  addPolygon("Stop", POLYGON, 1.0, "stop");
  addSource(RANGE_NAME, RANGE);
  setVectors({"Stop"}, {RANGE_NAME});

  // Start Collision Monitor node
  cm_->start();
  sendTransforms(curr_time);

  auto spinWait = [&](std::chrono::milliseconds ms) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < ms) {
      rclcpp::spin_some(cm_->get_node_base_interface());
      std::this_thread::sleep_for(10ms);
    }
  };

  // 1. 无订阅者时发布 range 数据，验证节点正常工作，无崩溃
  publishRange(0.5, curr_time);
  spinWait(50ms);
  publishRange(0.5, curr_time);
  spinWait(50ms);
  publishRange(0.5, curr_time);
  ASSERT_TRUE(waitData(0.5, 500ms, curr_time));

  // 2. 创建订阅者订阅 "ultrasonic_points"
  sensor_msgs::msg::PointCloud2::SharedPtr received_cloud;
  auto cloud_sub = cm_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "ultrasonic_points", 1,
    [&](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      received_cloud = msg;
    });

  // 触发控制循环处理并等待接收点云
  publishCmdVel(0.5, 0.0, 0.0);
  auto start = std::chrono::steady_clock::now();
  while (!received_cloud && std::chrono::steady_clock::now() - start < 1000ms) {
    rclcpp::spin_some(cm_->get_node_base_interface());
    std::this_thread::sleep_for(10ms);
  }

  // 验证成功收到超声波点云，且包含点
  ASSERT_TRUE(received_cloud != nullptr);
  ASSERT_GT(received_cloud->width * received_cloud->height, 0u);

  cm_->stop();
}

TEST_F(Tester, testRangeOutlierFilter)
{
  rclcpp::Time curr_time = cm_->now();

  setCommonParameters();
  addPolygon("Stop", POLYGON, 1.0, "stop");
  addSource(RANGE_NAME, RANGE);
  setVectors({"Stop"}, {RANGE_NAME});

  // 配置 RangeFilter 离群点滤波：滑动窗口大小 5，最大跳变 0.30m
  cm_->declare_parameter(std::string(RANGE_NAME) + ".filter_outliers", true);
  cm_->declare_parameter(std::string(RANGE_NAME) + ".outlier_filter_window_size", 5);
  cm_->declare_parameter(std::string(RANGE_NAME) + ".max_jump_distance", 0.30);

  cm_->start();
  sendTransforms(curr_time);

  auto spinWait = [&](std::chrono::milliseconds ms) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < ms) {
      rclcpp::spin_some(cm_->get_node_base_interface());
      std::this_thread::sleep_for(10ms);
    }
  };

  // 1. 模拟突发离群跳变噪点（如打到外壳等假回波脉冲：1.5m -> 0.08m -> 1.5m）
  publishRange(1.5, curr_time);
  spinWait(20ms);
  publishRange(0.08, curr_time);  // 突发离群假回波
  spinWait(20ms);
  publishRange(1.5, curr_time);
  spinWait(20ms);

  // 预期：RangeFilter 自动抑制突发离群跳变，机器人正常通行不停车
  publishCmdVel(0.5, 0.0, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.5, EPSILON);

  // 2. 模拟真实障碍物（0.25m）连续到达
  publishRange(0.25, curr_time);
  spinWait(20ms);
  publishRange(0.25, curr_time);
  spinWait(20ms);
  publishRange(0.25, curr_time);
  ASSERT_TRUE(waitData(0.25, 500ms, curr_time));

  // 预期：真实障碍物正常触发停障
  publishCmdVel(0.5, 0.0, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.0, EPSILON);

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
