// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_path_pruning_local_planner/path_pruning_local_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace
{

using nav2_path_pruning_local_planner::PathPruningLocalPlanner;

struct RclcppFixture
{
  RclcppFixture() {rclcpp::init(0, nullptr);}
  ~RclcppFixture() {rclcpp::shutdown();}
};

RclcppFixture g_rclcpp_fixture;

geometry_msgs::msg::PoseStamped makePose(
  double x, double y, const std::string & frame = "map")
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.w = 1.0;
  return pose;
}

nav_msgs::msg::Path makePath(
  const std::vector<std::pair<double, double>> & points)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto & point : points) {
    path.poses.push_back(makePose(point.first, point.second));
  }
  return path;
}

class PlannerFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    static int node_number = 0;
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "path_pruning_test_" + std::to_string(node_number++));
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    planner_ = std::make_shared<PathPruningLocalPlanner>();
    planner_->configure(node_, "PathPruner", tf_, nullptr);
  }

  nav_msgs::msg::Path compute(const geometry_msgs::msg::PoseStamped & robot_pose)
  {
    geometry_msgs::msg::Twist velocity;
    nav_msgs::msg::Path local_path;
    EXPECT_TRUE(planner_->computeLocalPlan(robot_pose, velocity, local_path));
    return local_path;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<PathPruningLocalPlanner> planner_;
};

TEST_F(PlannerFixture, StartsAtRobotProjectionAndPreservesTrueGoal)
{
  auto path = makePath({{0.0, 0.0}, {2.0, 0.0}, {4.0, 0.0}});
  tf2::Quaternion goal_orientation;
  goal_orientation.setRPY(0.0, 0.0, 1.2);
  path.poses.back().pose.orientation = tf2::toMsg(goal_orientation);
  planner_->setPlan(path);

  const auto local_path = compute(makePose(1.0, 1.0));

  ASSERT_EQ(local_path.poses.size(), 3u);
  EXPECT_NEAR(local_path.poses.front().pose.position.x, 1.0, 1e-9);
  EXPECT_NEAR(local_path.poses.front().pose.position.y, 0.0, 1e-9);
  EXPECT_EQ(local_path.poses.back(), path.poses.back());
}

TEST_F(PlannerFixture, PruningProgressDoesNotMoveBackward)
{
  planner_->setPlan(makePath({{0.0, 0.0}, {2.0, 0.0}, {4.0, 0.0}}));

  EXPECT_NEAR(compute(makePose(1.5, 0.0)).poses.front().pose.position.x, 1.5, 1e-9);
  EXPECT_NEAR(compute(makePose(2.5, 0.0)).poses.front().pose.position.x, 2.5, 1e-9);
  EXPECT_NEAR(compute(makePose(2.0, 0.0)).poses.front().pose.position.x, 2.5, 1e-9);
}

TEST_F(PlannerFixture, SearchWindowDoesNotJumpToLaterCrossingBranch)
{
  const auto crossing_path = makePath(
    {
      {-2.0, 0.0}, {0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0},
      {0.0, 2.0}, {-2.0, 2.0}, {-2.0, 0.05}, {0.0, 0.05}, {2.0, 0.05}});
  planner_->setPlan(crossing_path);

  compute(makePose(-1.0, 0.0));
  const auto local_path = compute(makePose(0.0, 0.04));

  ASSERT_GE(local_path.poses.size(), 2u);
  EXPECT_NEAR(local_path.poses.front().pose.position.x, 0.0, 1e-9);
  EXPECT_NEAR(local_path.poses.front().pose.position.y, 0.0, 1e-9);
  EXPECT_NEAR(local_path.poses[1].pose.position.x, 2.0, 1e-9);
  EXPECT_NEAR(local_path.poses[1].pose.position.y, 0.0, 1e-9);
}

TEST_F(PlannerFixture, NearGoalOutputRetainsExactGoalPose)
{
  auto path = makePath({{0.0, 0.0}, {4.0, 0.0}});
  path.poses.back().pose.position.z = 0.2;
  tf2::Quaternion goal_orientation;
  goal_orientation.setRPY(0.0, 0.0, -0.7);
  path.poses.back().pose.orientation = tf2::toMsg(goal_orientation);
  planner_->setPlan(path);

  const auto local_path = compute(makePose(3.999, 0.0));

  ASSERT_FALSE(local_path.poses.empty());
  EXPECT_EQ(local_path.poses.back(), path.poses.back());
}

TEST_F(PlannerFixture, ExactGoalRetainsProjectedStartAndTrueGoal)
{
  auto path = makePath({{0.0, 0.0}, {2.0, 0.0}, {4.0, 0.0}});
  planner_->setPlan(path);

  const auto local_path = compute(makePose(4.0, 0.0));

  ASSERT_EQ(local_path.poses.size(), 2u);
  EXPECT_NEAR(local_path.poses.front().pose.position.x, 4.0, 1e-9);
  EXPECT_EQ(local_path.poses.back(), path.poses.back());
}

TEST_F(PlannerFixture, PureRotationPlanPreservesTerminalOrientationSequence)
{
  auto path = makePath({{2.0, 3.0}, {2.0, 3.0}, {2.0, 3.0}});
  tf2::Quaternion middle_orientation;
  middle_orientation.setRPY(0.0, 0.0, 0.5);
  path.poses[1].pose.orientation = tf2::toMsg(middle_orientation);
  tf2::Quaternion goal_orientation;
  goal_orientation.setRPY(0.0, 0.0, 1.0);
  path.poses.back().pose.orientation = tf2::toMsg(goal_orientation);
  planner_->setPlan(path);

  const auto local_path = compute(makePose(2.0, 3.0));

  ASSERT_EQ(local_path.poses.size(), path.poses.size());
  EXPECT_EQ(local_path.poses, path.poses);
}

TEST_F(PlannerFixture, TransformsRobotPoseIntoPathFrame)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "odom";
  transform.transform.translation.x = 1.0;
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf_->setTransform(transform, "test", true));
  planner_->setPlan(makePath({{0.0, 0.0}, {2.0, 0.0}}));

  const auto local_path = compute(makePose(0.0, 0.0, "odom"));

  ASSERT_GE(local_path.poses.size(), 2u);
  EXPECT_NEAR(local_path.poses.front().pose.position.x, 1.0, 1e-9);
  EXPECT_EQ(local_path.header.frame_id, "map");
}

TEST_F(PlannerFixture, TransformFailureIsReported)
{
  planner_->setPlan(makePath({{0.0, 0.0}, {2.0, 0.0}}));
  geometry_msgs::msg::Twist velocity;
  nav_msgs::msg::Path local_path;

  EXPECT_FALSE(planner_->computeLocalPlan(makePose(0.0, 0.0, "unknown"), velocity, local_path));
  EXPECT_TRUE(local_path.poses.empty());
}

TEST_F(PlannerFixture, SinglePosePlanReturnsTheTrueGoal)
{
  auto path = makePath({{2.0, 3.0}});
  planner_->setPlan(path);

  const auto local_path = compute(makePose(2.0, 3.0));

  ASSERT_EQ(local_path.poses.size(), 1u);
  EXPECT_EQ(local_path.poses.front(), path.poses.front());
}

}  // namespace
