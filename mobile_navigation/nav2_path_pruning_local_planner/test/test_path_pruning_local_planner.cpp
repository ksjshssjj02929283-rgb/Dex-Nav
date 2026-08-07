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
#include <algorithm>
#include <cmath>
#include <vector>

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

TEST_F(PlannerFixture, UsesLatestTransformWhenPoseStampIsAhead)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.header.stamp = rclcpp::Time(9, 0, RCL_ROS_TIME);
  transform.child_frame_id = "odom";
  transform.transform.translation.x = 1.0;
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf_->setTransform(transform, "test", false));
  planner_->setPlan(makePath({{0.0, 0.0}, {2.0, 0.0}}));
  auto robot_pose = makePose(0.0, 0.0, "odom");
  robot_pose.header.stamp = rclcpp::Time(10, 0, RCL_ROS_TIME);

  const auto local_path = compute(robot_pose);

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

// ═══════════════════════════════════════════════════════════════════════
// 以下用例覆盖间距、弧长可观测性、进度回退三类缺口。
// 上方用例覆盖投影起点 / 进度单调 / 交叉分支 / 终端朝向 / TF；
// 以下补上间距、弧长可观测性、进度回退三类。
// ═══════════════════════════════════════════════════════════════════════

// 首点间距不受控：projected_start 按弧长插值落在原始段中间，
// 与下一原始点的距离可小到去重阈值（kDistanceEpsilonSq=1e-8 → 0.1 mm）。
// 下游速度规划的曲率用平均点距换算三点跨度，
// 隐含等距假设——本用例把该间距量化出来，作为契约讨论的事实依据。
TEST_F(PlannerFixture, ProjectedStartSpacingToNextPointIsUnbounded)
{
  // 原始点距 1.0 m；机器人投影落在距下一点仅 1 mm 处
  planner_->setPlan(makePath({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}}));
  const auto local_path = compute(makePose(0.999, 0.0));

  ASSERT_GE(local_path.poses.size(), 2u);
  const double dx = local_path.poses[1].pose.position.x -
    local_path.poses[0].pose.position.x;
  const double dy = local_path.poses[1].pose.position.y -
    local_path.poses[0].pose.position.y;
  const double spacing = std::hypot(dx, dy);

  // 现状：间距仅 ~1 mm，而其余点距 1.0 m —— 相差三个数量级。
  // 若将来给首点加间距下界（丢弃过近的投影点），本断言需相应改写。
  EXPECT_LT(spacing, 0.01)
    << "首点间距 " << spacing << " m —— 若已加入间距下界，请更新本用例";
}

// 同一条路径上点距可相差多个数量级，本层不做任何均匀化。
TEST_F(PlannerFixture, OutputPreservesInputSpacingIrregularity)
{
  // 前段密集（1 cm），后段稀疏（1 m）
  planner_->setPlan(
    makePath(
      {{0.0, 0.0}, {0.01, 0.0}, {0.02, 0.0}, {0.03, 0.0},
        {1.03, 0.0}, {2.03, 0.0}}));
  const auto local_path = compute(makePose(0.0, 0.0));

  ASSERT_GE(local_path.poses.size(), 4u);
  std::vector<double> spacing;
  for (size_t i = 1; i < local_path.poses.size(); ++i) {
    spacing.push_back(
      std::hypot(
        local_path.poses[i].pose.position.x - local_path.poses[i - 1].pose.position.x,
        local_path.poses[i].pose.position.y - local_path.poses[i - 1].pose.position.y));
  }
  const double lo = *std::min_element(spacing.begin(), spacing.end());
  const double hi = *std::max_element(spacing.begin(), spacing.end());
  // 原样透传输入的不均匀性（不重采样）——这是刻意的：等距重采样会毁掉
  // 终端原地转向序列（位置重合、朝向不同的点在弧长上是同一位置）。
  EXPECT_GT(hi / lo, 10.0) << "点距跨度 " << lo << " ~ " << hi << " m";
}

// 起点的全局绝对弧长未随输出携带：下游只能从 index 0 重数，每周期 s=0
// 都是当前位置，跨周期不可比。本用例证明该信息确实存在于本层内部
// （机器人前进后局部路径变短），只是没有出口。
TEST_F(PlannerFixture, ProgressIsObservableOnlyViaLengthShrink)
{
  planner_->setPlan(makePath({{0.0, 0.0}, {2.0, 0.0}, {4.0, 0.0}}));

  auto arc_len = [](const nav_msgs::msg::Path & p) {
      double s = 0.0;
      for (size_t i = 1; i < p.poses.size(); ++i) {
        s += std::hypot(
          p.poses[i].pose.position.x - p.poses[i - 1].pose.position.x,
          p.poses[i].pose.position.y - p.poses[i - 1].pose.position.y);
      }
      return s;
    };

  const double len0 = arc_len(compute(makePose(0.5, 0.0)));
  const double len1 = arc_len(compute(makePose(1.5, 0.0)));

  EXPECT_NEAR(len0 - len1, 1.0, 1e-6)
    << "前进 1 m，局部路径应短 1 m —— 进度信息存在但未随输出携带";
}

// progress_arc_ 单调不可回退：一旦投影被推到远端，无任何路径退回。
// 现状下唯一的复位是 setPlan（新全局路径）。本用例把该行为钉住，
// 以便将来加入回退判据时能看出差异。
TEST_F(PlannerFixture, ProgressNeverRecoversWithoutNewPlan)
{
  planner_->setPlan(makePath({{0.0, 0.0}, {2.0, 0.0}, {4.0, 0.0}}));

  compute(makePose(3.5, 0.0));                       // 进度推到 3.5
  const auto back = compute(makePose(0.5, 0.0));     // 机器人"回到"起点附近
  EXPECT_NEAR(back.poses.front().pose.position.x, 3.5, 1e-9)
    << "进度不回退（刻意设计）。若已实施回退判据，请更新本用例";

  planner_->setPlan(makePath({{0.0, 0.0}, {2.0, 0.0}, {4.0, 0.0}}));
  const auto after_replan = compute(makePose(0.5, 0.0));
  EXPECT_NEAR(after_replan.poses.front().pose.position.x, 0.5, 1e-9)
    << "setPlan 是当前唯一的复位路径";
}

// L1｜**输出不截断到投影搜索窗口**——这是本层的显式契约，不是疏漏。
//
// `max_projection_search_distance`（默认 5 m）只界定投影搜索范围；输出始终是
// 「投影点 → 全局终点」的全部剩余路径。理由：速度规划必须看到终点才能规划终端制动，
// 截成窗口会让终端约束在窗口外不可见。
//
// 本用例的存在意义是防止被"修好"：参数名容易让人以为该截断。若将来确实
// 要改成截断，本用例必须显式改写，而不是悄悄失效。
TEST_F(PlannerFixture, OutputIsNotTruncatedToProjectionSearchWindow)
{
  // 12 m 路径，远超默认 5 m 搜索窗口
  std::vector<std::pair<double, double>> points;
  for (int i = 0; i <= 12; ++i) {
    points.emplace_back(static_cast<double>(i), 0.0);
  }
  planner_->setPlan(makePath(points));

  const auto local_path = compute(makePose(1.0, 0.0));

  double out_len = 0.0;
  for (size_t i = 1; i < local_path.poses.size(); ++i) {
    const auto & a = local_path.poses[i - 1].pose.position;
    const auto & b = local_path.poses[i].pose.position;
    out_len += std::hypot(b.x - a.x, b.y - a.y);
  }

  EXPECT_NEAR(out_len, 11.0, 1e-6)
    << "输出应为剩余全长 11 m，而不是搜索窗口的 5 m。"
       "若已改为窗口截断，请显式更新本用例";
  EXPECT_NEAR(local_path.poses.back().pose.position.x, 12.0, 1e-9)
    << "终点必须始终在输出里——下游的终端制动依赖它";
}

// ───────────────── 输出系归位（local costmap 系）─────────────────
//
// 上面 15 个用例的 fixture 以 `nullptr` costmap 构造 → `costmapFrame()` 为空
// → 不做输出转换，故它们覆盖不到这条路径。本 fixture 挂一个 global_frame=odom
// 的真实 costmap，专门覆盖与 costmap frame 相关的行为。
class PlannerWithCostmapFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    static int node_number = 0;
    const std::string suffix = std::to_string(node_number++);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("pp_costmap_test_" + suffix);
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());

    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("pp_fake_costmap_" + suffix);
    costmap_->set_parameter(rclcpp::Parameter("global_frame", "odom"));
    costmap_->on_configure(rclcpp_lifecycle::State());

    planner_ = std::make_shared<PathPruningLocalPlanner>();
    planner_->configure(node_, "PathPruner", tf_, costmap_);
  }

  /// map→odom：平移 (dx, dy)，用于模拟定位修正/跳变
  void setMapToOdom(double dx, double dy)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "odom";     // 目标系
    t.child_frame_id = "map";       // 源系
    t.transform.translation.x = dx;
    t.transform.translation.y = dy;
    t.transform.rotation.w = 1.0;
    ASSERT_TRUE(tf_->setTransform(t, "test", true));
  }

  bool computeRaw(
    const geometry_msgs::msg::PoseStamped & robot_pose, nav_msgs::msg::Path & out)
  {
    geometry_msgs::msg::Twist velocity;
    return planner_->computeLocalPlan(robot_pose, velocity, out);
  }

  static double arcLen(const nav_msgs::msg::Path & p)
  {
    double len = 0.0;
    for (size_t i = 1; i < p.poses.size(); ++i) {
      const auto & a = p.poses[i - 1].pose.position;
      const auto & b = p.poses[i].pose.position;
      len += std::hypot(b.x - a.x, b.y - a.y);
    }
    return len;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<PathPruningLocalPlanner> planner_;
};

// 现状契约：**输出仍在 plan 系（map），不转到 local costmap 系**。
//
// 前提是控制器改为从轨迹本身取几何、不再自持一份 plan——Controller::setPlan
// 是「新计划」事件通道，不能承载逐周期变化的局部路径。
//
// 本用例钉的是待改的现状，不是理想终态；转换点上收之后应改为断言 "odom"。
TEST_F(PlannerWithCostmapFixture, OutputStaysInPlanFrameUntilControllerTakesGeometryFromTrajectory)
{
  setMapToOdom(10.0, 5.0);
  planner_->setPlan(makePath({{0.0, 0.0}, {2.0, 0.0}, {4.0, 0.0}}));

  nav_msgs::msg::Path local_path;
  ASSERT_TRUE(computeRaw(makePose(1.0, 0.0), local_path));

  EXPECT_EQ(local_path.header.frame_id, "map")
    << "现状：输出留在 plan 系，变换由下游各自再做一次。"
       "若已完成输出系归位，请把本用例改为断言 \"odom\"";
  ASSERT_GE(local_path.poses.size(), 2u);
  EXPECT_NEAR(local_path.poses.front().pose.position.x, 1.0, 1e-9)
    << "坐标未被变换";
}

// 本层持有 local costmap 并能报出它的 frame——这是输出系归位的前置能力，
// `frames_match` 诊断字段当前恒为 0，转换点上收之后应恒为 1。
TEST_F(PlannerWithCostmapFixture, CostmapFrameDiffersFromPlanFrameToday)
{
  setMapToOdom(0.0, 0.0);
  planner_->setPlan(makePath({{0.0, 0.0}, {2.0, 0.0}, {4.0, 0.0}}));

  nav_msgs::msg::Path local_path;
  ASSERT_TRUE(computeRaw(makePose(1.0, 0.0), local_path));

  EXPECT_EQ(costmap_->getGlobalFrameID(), "odom");
  EXPECT_NE(local_path.header.frame_id, costmap_->getGlobalFrameID())
    << "两系不一致是当前的已知缺口（诊断字段 frames_match=0）";
}

// 缺少 map→odom 时本层**不受影响**——因为它当前不做输出变换。
// 归位之后这里应改为「必须返回 false」（不沿用过期变换）。
TEST_F(PlannerWithCostmapFixture, MissingMapToOdomDoesNotAffectOutputToday)
{
  planner_->setPlan(makePath({{0.0, 0.0}, {2.0, 0.0}, {4.0, 0.0}}));

  nav_msgs::msg::Path local_path;
  EXPECT_TRUE(computeRaw(makePose(1.0, 0.0), local_path))
    << "现状不依赖 map→odom；归位后应改为断言 EXPECT_FALSE";
  EXPECT_FALSE(local_path.poses.empty());
}
