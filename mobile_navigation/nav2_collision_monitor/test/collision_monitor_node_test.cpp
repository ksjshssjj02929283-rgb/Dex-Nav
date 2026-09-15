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

  // 4. Restoring back normal operation
  publishScan(3.0, curr_time);
  ASSERT_TRUE(waitData(3.0, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.5, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.2, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.1, EPSILON);

  // Stop Collision Monitor node
  cm_->stop();
}

TEST_F(Tester, testTrackingPathAheadStop)
{
  const rclcpp::Time curr_time = cm_->now();
  setCommonParameters();
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
  publishScheduleCmdVel(0.5, 0.0, 0.0);
  ASSERT_TRUE(waitObstacleStatus(nav_interfaces::msg::ObstacleMonitorStatus::STOP, 500ms));
  EXPECT_FALSE(cmd_vel_out_);

  // Ordinary schedule navigation must ignore the corridor obstacle at the same position.
  cm_->receiveTrackingPath(route, 0.15F);
  publishScheduleCmdVel(0.5, 0.0, 0.0);
  ASSERT_TRUE(waitObstacleStatus(nav_interfaces::msg::ObstacleMonitorStatus::FREE, 500ms));
  EXPECT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::SCHEDULE_NAVIGATION);
  EXPECT_FALSE(cmd_vel_out_);

  // Returning to edging enables corridor detection again.
  cm_->receiveTrackingPath(route, 0.0F);
  publishScheduleCmdVel(0.5, 0.0, 0.0);
  ASSERT_TRUE(waitObstacleStatus(nav_interfaces::msg::ObstacleMonitorStatus::STOP, 500ms));
  EXPECT_EQ(cm_->mode(), CollisionMonitorWrapper::NavigationMode::SCHEDULE_EDGING);
  EXPECT_FALSE(cmd_vel_out_);

  // The same obstacle is outside a parallel route corridor: velocity is passed through.
  cm_->receiveTrackingPath(
  {
    {0.0, 1.0}, {0.4, 1.0}, {0.8, 1.0},
    {1.2, 1.0}, {1.6, 1.0}, {2.0, 1.0}});
  publishScheduleCmdVel(0.5, 0.0, 0.0);
  ASSERT_TRUE(waitObstacleStatus(nav_interfaces::msg::ObstacleMonitorStatus::FREE, 500ms));
  EXPECT_FALSE(cmd_vel_out_);

  // x=1.6 is beyond the 1 m lookahead and the 0.2 m PolygonStop radius, so it must not stop.
  cm_->receiveTrackingPath(route);
  publishPointCloud(1.6, curr_time);
  ASSERT_TRUE(waitData(std::hypot(1.6, 0.01), 500ms, curr_time));
  publishScheduleCmdVel(0.5, 0.0, 0.0);
  ASSERT_TRUE(waitObstacleStatus(nav_interfaces::msg::ObstacleMonitorStatus::FREE, 500ms));
  EXPECT_FALSE(cmd_vel_out_);

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
