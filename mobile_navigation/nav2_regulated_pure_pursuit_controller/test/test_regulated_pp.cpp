// Copyright (c) 2021 Samsung Research America
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

#include <math.h>
#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "path_utils/path_utils.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_core/exceptions.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class BasicAPIRPP : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  BasicAPIRPP()
  : nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController() {}

  nav_msgs::msg::Path getPlan() {return global_plan_;}

  double getSpeed() {return desired_linear_vel_;}

  std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsgWrapper(
    const geometry_msgs::msg::PoseStamped & carrot_pose)
  {
    return createCarrotMsg(carrot_pose);
  }

  void setVelocityScaledLookAhead() {use_velocity_scaled_lookahead_dist_ = true;}
  void setCostRegulationScaling() {use_cost_regulated_linear_velocity_scaling_ = true;}
  void resetVelocityRegulationScaling() {use_regulated_linear_velocity_scaling_ = false;}

  double getLookAheadDistanceWrapper(const geometry_msgs::msg::Twist & twist)
  {
    return getLookAheadDistance(twist);
  }

  static geometry_msgs::msg::Point circleSegmentIntersectionWrapper(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2,
    double r)
  {
    return circleSegmentIntersection(p1, p2, r);
  }

  geometry_msgs::msg::PoseStamped getLookAheadPointWrapper(
    const double & dist, const nav_msgs::msg::Path & path)
  {
    return getLookAheadPoint(dist, path);
  }

  bool shouldRotateToPathWrapper(
    const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path)
  {
    return shouldRotateToPath(carrot_pose, angle_to_path);
  }

  bool shouldRotateToGoalHeadingWrapper(const geometry_msgs::msg::PoseStamped & carrot_pose)
  {
    return shouldRotateToGoalHeading(carrot_pose);
  }

  void rotateToHeadingWrapper(
    double & linear_vel, double & angular_vel,
    const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed,
    double angular_vel_max, double angular_kp)
  {
    return rotateToHeading(
      linear_vel, angular_vel, angle_to_path, curr_speed, angular_vel_max, angular_kp);
  }

  void setTrajSpeedPointsForTest(
    const std::vector<TrajSpeedPoint> & pts, double lookahead)
  {
    traj_speed_points_ = pts;
    traj_speed_lookahead_ = lookahead;
  }

  bool nearestTrajectorySpeedWrapper(
    const geometry_msgs::msg::PoseStamped & pose, double & v, double & a)
  {
    return nearestTrajectorySpeed(pose, v, a);
  }

  double trackLinearVelocityWrapper(double target, double measured, double ff)
  {
    return trackLinearVelocity(target, measured, ff);
  }

  void applyConstraintsWrapper(
    const double & curvature, const geometry_msgs::msg::Twist & curr_speed,
    const double & pose_cost, const nav_msgs::msg::Path & path, bool have_traj,
    double & linear_vel, double & sign)
  {
    return applyConstraints(
      curvature, curr_speed, pose_cost, path, have_traj,
      linear_vel, sign);
  }

  void applyTrackingFeedbackConstraintsWrapper(
    const nav_msgs::msg::Path & transformed_plan, double curvature, double near_curvature,
    double angle_to_heading, double & linear_vel, double & cross_track_error)
  {
    return applyTrackingFeedbackConstraints(
      transformed_plan, curvature, near_curvature, angle_to_heading,
      linear_vel, cross_track_error);
  }

  double findVelocitySignChangeWrapper(
    const nav_msgs::msg::Path & transformed_plan)
  {
    return findVelocitySignChange(transformed_plan);
  }

  nav_msgs::msg::Path transformGlobalPlanWrapper(
    const geometry_msgs::msg::PoseStamped & pose,
    geometry_msgs::msg::PoseStamped * robot_pose_in_plan = nullptr)
  {
    return transformGlobalPlan(pose, robot_pose_in_plan);
  }

  void setTrajectoryWrapper(const nav_interfaces::msg::RobotTrajectory & trajectory)
  {
    setTrajectory(trajectory);
  }

  bool trajectoryValidForTest() const {return trajectory_valid_;}
  std::string trajectoryFrameForTest() const {return trajectory_frame_;}
};

TEST(RegulatedPurePursuitTest, basicAPI)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testRPP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");

  // instantiate
  auto ctrl = std::make_shared<BasicAPIRPP>();
  costmap->on_configure(rclcpp_lifecycle::State());
  ctrl->configure(node, name, tf, costmap);
  ctrl->activate();
  ctrl->deactivate();
  ctrl->cleanup();

  // setPlan and get plan
  nav_msgs::msg::Path path;
  path.poses.resize(2);
  path.poses[0].header.frame_id = "fake_frame";
  ctrl->setPlan(path);
  EXPECT_EQ(ctrl->getPlan().poses.size(), 2ul);
  EXPECT_EQ(ctrl->getPlan().poses[0].header.frame_id, std::string("fake_frame"));

  // set speed limit
  const double base_speed = ctrl->getSpeed();
  EXPECT_EQ(ctrl->getSpeed(), base_speed);
  ctrl->setSpeedLimit(0.51, false);
  EXPECT_EQ(ctrl->getSpeed(), 0.51);
  ctrl->setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, false);
  EXPECT_EQ(ctrl->getSpeed(), base_speed);
  ctrl->setSpeedLimit(30, true);
  EXPECT_EQ(ctrl->getSpeed(), base_speed * 0.3);
  ctrl->setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, true);
  EXPECT_EQ(ctrl->getSpeed(), base_speed);
}

TEST(RegulatedPurePursuitTest, createCarrotMsg)
{
  auto ctrl = std::make_shared<BasicAPIRPP>();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "Hi!";
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 12.0;
  pose.pose.orientation.w = 0.5;

  auto rtn = ctrl->createCarrotMsgWrapper(pose);
  EXPECT_EQ(rtn->header.frame_id, std::string("Hi!"));
  EXPECT_EQ(rtn->point.x, 1.0);
  EXPECT_EQ(rtn->point.y, 12.0);
  EXPECT_EQ(rtn->point.z, 0.01);
}

TEST(RegulatedPurePursuitTest, findVelocitySignChange)
{
  auto ctrl = std::make_shared<BasicAPIRPP>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testRPPfindVelocitySignChange");
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "smb";
  auto time = node->get_clock()->now();
  pose.header.stamp = time;
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 0.0;

  nav_msgs::msg::Path path;
  path.poses.resize(3);
  path.header.frame_id = "smb";
  path.header.stamp = pose.header.stamp;
  path.poses[0].pose.position.x = 1.0;
  path.poses[0].pose.position.y = 1.0;
  path.poses[1].pose.position.x = 2.0;
  path.poses[1].pose.position.y = 2.0;
  path.poses[2].pose.position.x = -1.0;
  path.poses[2].pose.position.y = -1.0;
  ctrl->setPlan(path);
  auto rtn = ctrl->findVelocitySignChangeWrapper(path);
  EXPECT_EQ(rtn, sqrt(8.0));

  path.poses[2].pose.position.x = 3.0;
  path.poses[2].pose.position.y = 3.0;
  ctrl->setPlan(path);
  rtn = ctrl->findVelocitySignChangeWrapper(path);
  EXPECT_EQ(rtn, std::numeric_limits<double>::max());
}

using CircleSegmentIntersectionParam = std::tuple<
  std::pair<double, double>,
  std::pair<double, double>,
  double,
  std::pair<double, double>
>;

class CircleSegmentIntersectionTest
  : public ::testing::TestWithParam<CircleSegmentIntersectionParam>
{};

TEST_P(CircleSegmentIntersectionTest, circleSegmentIntersection)
{
  auto pair1 = std::get<0>(GetParam());
  auto pair2 = std::get<1>(GetParam());
  auto r = std::get<2>(GetParam());
  auto expected_pair = std::get<3>(GetParam());
  auto pair_to_point = [](std::pair<double, double> p) -> geometry_msgs::msg::Point {
      geometry_msgs::msg::Point point;
      point.x = p.first;
      point.y = p.second;
      point.z = 0.0;
      return point;
    };
  auto p1 = pair_to_point(pair1);
  auto p2 = pair_to_point(pair2);
  auto actual = BasicAPIRPP::circleSegmentIntersectionWrapper(p1, p2, r);
  auto expected_point = pair_to_point(expected_pair);
  EXPECT_DOUBLE_EQ(actual.x, expected_point.x);
  EXPECT_DOUBLE_EQ(actual.y, expected_point.y);
  // Expect that the intersection point is actually r away from the origin
  EXPECT_DOUBLE_EQ(r, std::hypot(actual.x, actual.y));
}

INSTANTIATE_TEST_SUITE_P(
  InterpolationTest,
  CircleSegmentIntersectionTest,
  testing::Values(
    // Origin to the positive X axis
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {2.0, 0.0},
  1.0,
  {1.0, 0.0}
},
    // Origin to hte negative X axis
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {-2.0, 0.0},
  1.0,
  {-1.0, 0.0}
},
    // Origin to the positive Y axis
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {0.0, 2.0},
  1.0,
  {0.0, 1.0}
},
    // Origin to the negative Y axis
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {0.0, -2.0},
  1.0,
  {0.0, -1.0}
},
    // non-origin to the X axis with non-unit circle, with the second point inside
    CircleSegmentIntersectionParam{
  {4.0, 0.0},
  {-1.0, 0.0},
  2.0,
  {2.0, 0.0}
},
    // non-origin to the Y axis with non-unit circle, with the second point inside
    CircleSegmentIntersectionParam{
  {0.0, 4.0},
  {0.0, -0.5},
  2.0,
  {0.0, 2.0}
},
    // origin to the positive X axis, on the circle
    CircleSegmentIntersectionParam{
  {2.0, 0.0},
  {0.0, 0.0},
  2.0,
  {2.0, 0.0}
},
    // origin to the positive Y axis, on the circle
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {0.0, 2.0},
  2.0,
  {0.0, 2.0}
},
    // origin to the upper-right quadrant (3-4-5 triangle)
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {6.0, 8.0},
  5.0,
  {3.0, 4.0}
},
    // origin to the lower-left quadrant (3-4-5 triangle)
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {-6.0, -8.0},
  5.0,
  {-3.0, -4.0}
},
    // origin to the upper-left quadrant (3-4-5 triangle)
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {-6.0, 8.0},
  5.0,
  {-3.0, 4.0}
},
    // origin to the lower-right quadrant (3-4-5 triangle)
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {6.0, -8.0},
  5.0,
  {3.0, -4.0}
}
));

TEST(RegulatedPurePursuitTest, lookaheadAPI)
{
  auto ctrl = std::make_shared<BasicAPIRPP>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testRPP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  ctrl->configure(node, name, tf, costmap);

  geometry_msgs::msg::Twist twist;

  // test getLookAheadDistance
  double rtn = ctrl->getLookAheadDistanceWrapper(twist);
  EXPECT_EQ(rtn, 0.6);  // default lookahead_dist

  // shouldn't be a function of speed
  twist.linear.x = 10.0;
  rtn = ctrl->getLookAheadDistanceWrapper(twist);
  EXPECT_EQ(rtn, 0.6);

  // now it should be a function of velocity, max out
  ctrl->setVelocityScaledLookAhead();
  rtn = ctrl->getLookAheadDistanceWrapper(twist);
  EXPECT_EQ(rtn, 0.9);  // 10 speed maxes out at max_lookahead_dist

  // check normal range
  twist.linear.x = 0.35;
  rtn = ctrl->getLookAheadDistanceWrapper(twist);
  EXPECT_NEAR(rtn, 0.525, 0.0001);  // 1.5 * 0.35

  // check minimum range
  twist.linear.x = 0.0;
  rtn = ctrl->getLookAheadDistanceWrapper(twist);
  EXPECT_EQ(rtn, 0.3);

  // test getLookAheadPoint
  double dist = 1.0;
  nav_msgs::msg::Path path;
  path.poses.resize(10);
  for (uint i = 0; i != path.poses.size(); i++) {
    path.poses[i].pose.position.x = static_cast<double>(i);
  }

  // test exact hits
  auto pt = ctrl->getLookAheadPointWrapper(dist, path);
  EXPECT_EQ(pt.pose.position.x, 1.0);

  // test getting next closest point without interpolation
  node->set_parameter(
    rclcpp::Parameter(
      name + ".use_interpolation",
      rclcpp::ParameterValue(false)));
  ctrl->configure(node, name, tf, costmap);
  dist = 3.8;
  pt = ctrl->getLookAheadPointWrapper(dist, path);
  EXPECT_EQ(pt.pose.position.x, 4.0);

  // test end of path
  dist = 100.0;
  pt = ctrl->getLookAheadPointWrapper(dist, path);
  EXPECT_EQ(pt.pose.position.x, 9.0);

  // test interpolation
  node->set_parameter(
    rclcpp::Parameter(
      name + ".use_interpolation",
      rclcpp::ParameterValue(true)));
  ctrl->configure(node, name, tf, costmap);
  dist = 3.8;
  pt = ctrl->getLookAheadPointWrapper(dist, path);
  EXPECT_EQ(pt.pose.position.x, 3.8);
}

TEST(RegulatedPurePursuitTest, rotateTests)
{
  auto ctrl = std::make_shared<BasicAPIRPP>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testRPP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  ctrl->configure(node, name, tf, costmap);

  // shouldRotateToPath
  geometry_msgs::msg::PoseStamped carrot;
  double angle_to_path_rtn;
  EXPECT_EQ(ctrl->shouldRotateToPathWrapper(carrot, angle_to_path_rtn), false);

  carrot.pose.position.x = 0.5;
  carrot.pose.position.y = 0.25;
  EXPECT_EQ(ctrl->shouldRotateToPathWrapper(carrot, angle_to_path_rtn), false);

  carrot.pose.position.x = 0.5;
  carrot.pose.position.y = 1.0;
  EXPECT_EQ(ctrl->shouldRotateToPathWrapper(carrot, angle_to_path_rtn), true);

  // shouldRotateToGoalHeading
  carrot.pose.position.x = 0.0;
  carrot.pose.position.y = 0.0;
  EXPECT_EQ(ctrl->shouldRotateToGoalHeadingWrapper(carrot), true);

  carrot.pose.position.x = 0.0;
  carrot.pose.position.y = 0.24;
  EXPECT_EQ(ctrl->shouldRotateToGoalHeadingWrapper(carrot), true);

  carrot.pose.position.x = 0.0;
  carrot.pose.position.y = 0.26;
  EXPECT_EQ(ctrl->shouldRotateToGoalHeadingWrapper(carrot), false);

  // rotateToHeading
  double lin_v = 10.0;
  double ang_v = 0.5;
  double angle_to_path = 0.4;
  geometry_msgs::msg::Twist curr_speed;
  curr_speed.angular.z = 1.75;

  // 角速度是 angle_to_path 的纯 P 控制（kp·angle，饱和于 angular_vel_max），
  // 不再是恒定最大值：这里传入 kp=1.0（rotate_to_path_angular_kp 默认值），
  // angle_to_path=0.4 时目标=0.4，再经运动学限幅（curr_speed±max_angular_accel_*dt
  // =curr±0.16）钳制。
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle_to_path, curr_speed, 1.8, 1.0);
  EXPECT_EQ(lin_v, 0.0);
  EXPECT_NEAR(ang_v, 1.59, 0.01);  // 目标 0.4 低于可行下限(1.75-0.16)，钳到下限

  // negative direction
  angle_to_path = -0.4;
  curr_speed.angular.z = -1.75;
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle_to_path, curr_speed, 1.8, 1.0);
  EXPECT_NEAR(ang_v, -1.59, 0.01);

  // kinematic clamping, no speed, some speed accelerating, some speed decelerating
  angle_to_path = 0.4;
  curr_speed.angular.z = 0.0;
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle_to_path, curr_speed, 1.8, 1.0);
  EXPECT_NEAR(ang_v, 0.16, 0.01);  // 目标 0.4 仍超可行上限(0.16)，钳到上限

  curr_speed.angular.z = 1.0;
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle_to_path, curr_speed, 1.8, 1.0);
  EXPECT_NEAR(ang_v, 0.84, 0.01);  // 目标 0.4 低于可行下限(1.0-0.16=0.84)，钳到下限

  angle_to_path = -0.4;
  curr_speed.angular.z = 1.0;
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle_to_path, curr_speed, 1.8, 1.0);
  EXPECT_NEAR(ang_v, 0.84, 0.01);  // 目标 -0.4 更低于可行下限(0.84)，同样钳到下限
}

// rotateToHeading 线速度连续减速：巡航速度进入 rotate 分支时不再瞬间
// 置零，按 max_linear_accel_ 逐周期衰减；多周期迭代（模拟测量跟随指令衰减）
// 应单调收敛到 0，且任意单周期降幅不超过 max_linear_accel_*dt。
TEST(RegulatedPurePursuitTest, rotateToHeadingLinearDecelRamp)
{
  auto ctrl = std::make_shared<BasicAPIRPP>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testRPP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  ctrl->configure(node, name, tf, costmap);

  // 单周期：默认 max_linear_accel=0.5、control_frequency=20 -> decel_step=0.025
  double lin_v = 0.0, ang_v = 0.0;
  geometry_msgs::msg::Twist curr_speed;
  curr_speed.linear.x = 0.6;
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, 0.4, curr_speed, 1.8, 1.0);
  EXPECT_GT(lin_v, 0.0);  // 不再瞬间置零
  EXPECT_NEAR(lin_v, 0.575, 1e-6);

  // 多周期：measured 跟随上一周期指令（完美 plant），应单调收敛到 0，
  // 且每一步的降幅不超过 decel_step（不出现比理论限幅更陡的阶跃）。
  double measured = 0.6;
  double prev = measured;
  for (int i = 0; i < 40; ++i) {
    lin_v = 0.0;
    ang_v = 0.0;
    curr_speed.linear.x = measured;
    ctrl->rotateToHeadingWrapper(lin_v, ang_v, 0.4, curr_speed, 1.8, 1.0);
    EXPECT_LE(prev - lin_v, 0.025 + 1e-9);
    EXPECT_GE(lin_v, 0.0);
    prev = lin_v;
    measured = lin_v;
  }
  EXPECT_EQ(lin_v, 0.0);
}

// rotateToHeading 角速度是 angle_to_path 的纯 P 控制（angular_kp·angle，
// 饱和于 angular_vel_max）：未饱和时精确正比于角度（验证线性关系），大角度
// 时饱和封顶。用完美 plant 多周期迭代到稳态（accel 限幅下会精确收敛到目标
// 值，非近似）。
TEST(RegulatedPurePursuitTest, rotateToHeadingAngularProportional)
{
  auto ctrl = std::make_shared<BasicAPIRPP>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testRPP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  ctrl->configure(node, name, tf, costmap);

  auto steadyStateAngularVel = [&](double angle) {
      double lin_v = 0.0;
      double ang_v = 0.0;
      geometry_msgs::msg::Twist curr_speed;
      for (int i = 0; i < 200; ++i) {
        ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle, curr_speed, 1.8, 1.0);
        curr_speed.angular.z = ang_v;
      }
      return ang_v;
    };

  // angular_vel_max=1.8, angular_kp=1.0：未饱和区间输出应精确等于 kp·angle。
  const double v_small = steadyStateAngularVel(0.1);    // 目标 0.1，远低于饱和上限
  const double v_double = steadyStateAngularVel(0.2);   // 目标 0.2，仍未饱和
  const double v_large = steadyStateAngularVel(3.0);    // 目标 3.0 > 1.8，饱和

  EXPECT_NEAR(v_small, 0.1, 0.01);
  EXPECT_NEAR(v_double, 0.2, 0.01);
  EXPECT_NEAR(v_double, 2.0 * v_small, 0.02);  // 双倍角度 -> 双倍角速度，验证线性比例
  EXPECT_NEAR(v_large, 1.8, 0.01);             // 饱和封顶于 angular_vel_max
}

// rotate_to_path 与 rotate_to_goal 的角速度/角度参数完全独立：同一个
// angle_to_path，喂给两套不同的 (angular_vel_max, angular_kp) 得到不同的
// 稳态角速度——证明两条调用路径不再共用同一份参数，调一个不会牵连另一个。
TEST(RegulatedPurePursuitTest, rotateToPathAndRotateToGoalAreDecoupled)
{
  auto ctrl = std::make_shared<BasicAPIRPP>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testRPP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  ctrl->configure(node, name, tf, costmap);

  auto steadyStateAngularVel = [&](double angle, double angular_vel_max, double angular_kp) {
      double lin_v = 0.0;
      double ang_v = 0.0;
      geometry_msgs::msg::Twist curr_speed;
      for (int i = 0; i < 200; ++i) {
        ctrl->rotateToHeadingWrapper(
          lin_v, ang_v, angle, curr_speed, angular_vel_max, angular_kp);
        curr_speed.angular.z = ang_v;
      }
      return ang_v;
    };

  const double angle = 0.3;
  // rotate_to_path 与 rotate_to_goal 用不同的 kp——同一个 angle 下两者的
  // 稳态输出应各自贴自己那套参数（都未饱和），互不干扰。
  const double v_path = steadyStateAngularVel(angle, /*angular_vel_max=*/ 0.5, /*kp=*/ 1.0);
  const double v_goal = steadyStateAngularVel(angle, /*angular_vel_max=*/ 1.8, /*kp=*/ 2.0);

  EXPECT_NEAR(v_path, 0.3, 0.02);  // kp=1.0：目标=0.3，未饱和
  EXPECT_NEAR(v_goal, 0.6, 0.02);  // kp=2.0：目标=0.6，未饱和
  EXPECT_NE(v_path, v_goal);
}

TEST(RegulatedPurePursuitTest, nearestTrajectorySpeedExcludesStateStamp)
{
  auto ctrl = std::make_shared<BasicAPIRPP>();
  geometry_msgs::msg::PoseStamped pose;  // 机器人在轨迹首点（状态戳）处
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  double v = 0.0, a = 0.0;

  // 场景 1：巡航超速——状态戳 0.65 高于前方剖面 0.6，目标必须取前方 0.6
  // （若把状态戳纳入 max，目标=自身速度，超速被固化）
  ctrl->setTrajSpeedPointsForTest(
    {{0.0, 0.0, 0.65, 0.0}, {0.05, 0.0, 0.6, 0.0}, {0.10, 0.0, 0.6, 0.0},
      {0.15, 0.0, 0.6, 0.0}},
    0.1);
  EXPECT_TRUE(ctrl->nearestTrajectorySpeedWrapper(pose, v, a));
  EXPECT_NEAR(v, 0.6, 1e-6);

  // 场景 2：减速坡道——状态戳 0.65（其 accel=-0.9 由实测反推，不得用作前馈），
  // 前方 0.45/0.30/0.15 递减（规划 accel=-0.4）。目标沿坡道取 0.45，
  // 前馈取首个前方点的 -0.4（与速度窗同基准，切断实测耦合）
  ctrl->setTrajSpeedPointsForTest(
    {{0.0, 0.0, 0.65, -0.9}, {0.05, 0.0, 0.45, -0.4}, {0.10, 0.0, 0.30, -0.4},
      {0.15, 0.0, 0.15, -0.4}},
    0.1);
  EXPECT_TRUE(ctrl->nearestTrajectorySpeedWrapper(pose, v, a));
  EXPECT_NEAR(v, 0.45, 1e-6);
  EXPECT_NEAR(a, -0.4, 1e-6);  // 前馈取首个前方点，不取状态戳

  // 场景 3：静止起步——状态戳 0，前方加速坡道 0.1/0.2，目标 >0（破自引用环）
  ctrl->setTrajSpeedPointsForTest(
    {{0.0, 0.0, 0.0, 0.3}, {0.05, 0.0, 0.1, 0.3}, {0.10, 0.0, 0.2, 0.3}},
    0.1);
  EXPECT_TRUE(ctrl->nearestTrajectorySpeedWrapper(pose, v, a));
  EXPECT_GT(v, 0.0);

  // 场景 4：窗口最大速度不在首个前方点，accel 必须跟随最大速度索引。
  ctrl->setTrajSpeedPointsForTest(
    {{0.0, 0.0, 0.0, 0.0}, {0.05, 0.0, 0.1, 0.1}, {0.10, 0.0, 0.25, 0.35},
      {0.15, 0.0, 0.2, -0.1}},
    0.1);
  EXPECT_TRUE(ctrl->nearestTrajectorySpeedWrapper(pose, v, a));
  EXPECT_NEAR(v, 0.25, 1e-6);
  EXPECT_NEAR(a, 0.35, 1e-6);

  // 场景 5：已在末点——无前方点，回退用最近点自身
  ctrl->setTrajSpeedPointsForTest({{0.0, 0.0, 0.075, 0.0}}, 0.1);
  EXPECT_TRUE(ctrl->nearestTrajectorySpeedWrapper(pose, v, a));
  EXPECT_NEAR(v, 0.075, 1e-6);
}

TEST(RegulatedPurePursuitTest, trackLinearVelocityCommandIntegrator)
{
  auto ctrl = std::make_shared<BasicAPIRPP>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testRPPSpeedLoop");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap_speed");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".desired_linear_vel", rclcpp::ParameterValue(1.0));
  ctrl->configure(node, name, tf, costmap);

  // T1 超速泄放：实测恒 0.65 高于目标 0.6——指令必须能累积下降并越过目标
  // （旧结构锚定实测：指令恒 ≈ 实测-一步，永远下不去）
  double v = 0.65;
  bool crossed_below_target = false;
  for (int i = 0; i < 500 && !crossed_below_target; ++i) {
    v = ctrl->trackLinearVelocityWrapper(0.6, 0.65, 0.0);
    if (v < 0.6) {crossed_below_target = true;}
  }
  EXPECT_TRUE(crossed_below_target);

  // T2 滞后 plant 主动制动：目标 0.1，plant 一阶滞后跟随指令——
  // 指令必须能显著低于实测（主动刹车），且最终把 plant 带到目标附近
  auto ctrl2 = std::make_shared<BasicAPIRPP>();
  ctrl2->configure(node, name, tf, costmap);
  double measured = 0.6;
  double min_cmd_minus_measured = 1.0;
  double cmd = measured;
  for (int i = 0; i < 500; ++i) {
    cmd = ctrl2->trackLinearVelocityWrapper(0.1, measured, -0.4);
    min_cmd_minus_measured = std::min(min_cmd_minus_measured, cmd - measured);
    measured = 0.8 * measured + 0.2 * cmd;  // plant 滞后跟随
    if (measured < 0.12) {break;}
  }
  EXPECT_LT(min_cmd_minus_measured, -0.05);  // 指令下越实测=主动制动权
  EXPECT_LT(measured, 0.15);

  // T3 reseed：指令与实测脱节超过阈值（0.3）时重新对齐实测
  double v3 = ctrl2->trackLinearVelocityWrapper(1.0, 1.0, 0.0);
  EXPECT_NEAR(v3, 1.0, 0.05);

  // T4 绝对界：目标越限时指令被 desired_linear_vel(1.0) 封顶
  auto ctrl3 = std::make_shared<BasicAPIRPP>();
  ctrl3->configure(node, name, tf, costmap);
  double m4 = 0.9;
  double v4 = m4;
  for (int i = 0; i < 500; ++i) {
    v4 = ctrl3->trackLinearVelocityWrapper(5.0, m4, 0.5);
    m4 = 0.8 * m4 + 0.2 * v4;
  }
  EXPECT_LE(v4, 1.0 + 1e-9);
  EXPECT_GT(v4, 0.95);

  // T5 下界：前向减速强前馈下压时指令不得穿零变负（滞后 plant，目标 0.05）
  auto ctrl5 = std::make_shared<BasicAPIRPP>();
  ctrl5->configure(node, name, tf, costmap);
  double m5 = 0.4;
  double min_cmd = 1.0;
  for (int i = 0; i < 300; ++i) {
    double c = ctrl5->trackLinearVelocityWrapper(0.05, m5, -0.5);
    min_cmd = std::min(min_cmd, c);
    m5 = 0.8 * m5 + 0.2 * c;
  }
  EXPECT_GE(min_cmd, 0.0);   // 不穿零
  EXPECT_LT(m5, 0.10);       // 仍能把 plant 带到目标附近

  // T6 上限：指令不得超过瞬时规划速度（巡航目标 0.6，plant 完美跟随）
  auto ctrl6 = std::make_shared<BasicAPIRPP>();
  ctrl6->configure(node, name, tf, costmap);
  double m6 = 0.0;
  double max_cmd = 0.0;
  for (int i = 0; i < 300; ++i) {
    double c = ctrl6->trackLinearVelocityWrapper(0.6, m6, 0.0);
    max_cmd = std::max(max_cmd, c);
    m6 = c;  // 完美 plant
  }
  EXPECT_LE(max_cmd, 0.6 + 1e-9);
  EXPECT_GT(m6, 0.55);
}

TEST(RegulatedPurePursuitTest, applyConstraints)
{
  auto ctrl = std::make_shared<BasicAPIRPP>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testRPP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);

  constexpr double approach_velocity_scaling_dist = 0.6;
  nav2_util::declare_parameter_if_not_declared(
    node,
    name + ".approach_velocity_scaling_dist",
    rclcpp::ParameterValue(approach_velocity_scaling_dist));

  ctrl->configure(node, name, tf, costmap);

  auto no_approach_path = path_utils::generate_path(
    geometry_msgs::msg::PoseStamped(), 0.1, {
    std::make_unique<path_utils::Straight>(approach_velocity_scaling_dist + 1.0)
  });

  double curvature = 0.5;
  geometry_msgs::msg::Twist curr_speed;
  double pose_cost = 0.0;
  double linear_vel = 0.0;
  double sign = 1.0;

  // 入口 linear_vel 是轨迹规划目标且约束链只降不升：目标为 0
  // （障碍零速/蠕行零点）时不得被调节下限抬升
  curr_speed.linear.x = 0.25;
  ctrl->applyConstraintsWrapper(
    curvature, curr_speed, pose_cost, no_approach_path, false,
    linear_vel, sign);
  EXPECT_EQ(linear_vel, 0.0);

  linear_vel = 1.0;
  curvature = 2.2222;  // radius 0.45 = regulated_linear_scaling_min_radius / 2
  curr_speed.linear.x = 0.5;
  ctrl->applyConstraintsWrapper(
    curvature, curr_speed, pose_cost, no_approach_path, false,
    linear_vel, sign);
  EXPECT_NEAR(linear_vel, 0.5, 0.01);  // lower by curvature

  linear_vel = 1.0;
  curvature = 1000.0;
  curr_speed.linear.x = 0.25;
  ctrl->applyConstraintsWrapper(
    curvature, curr_speed, pose_cost, no_approach_path, false,
    linear_vel, sign);
  EXPECT_NEAR(linear_vel, 0.25, 0.01);  // min out by curvature

  // Approach velocity scaling on a path with no distance left
  auto approach_path = path_utils::generate_path(
    geometry_msgs::msg::PoseStamped(), 0.1, {
    std::make_unique<path_utils::Straight>(0.0)
  });

  linear_vel = 1.0;
  curvature = 0.0;
  curr_speed.linear.x = 0.25;
  ctrl->applyConstraintsWrapper(
    curvature, curr_speed, pose_cost, approach_path, false,
    linear_vel, sign);
  EXPECT_NEAR(linear_vel, 0.05, 0.01);  // min out on min approach velocity

  // have_traj=true：终点收尾减速已由轨迹规划速度逐点编码，approach scaling
  // 应被跳过——同样零剩余距离的路径，不应再被压到 min_approach_linear_velocity
  linear_vel = 1.0;
  curvature = 0.0;
  curr_speed.linear.x = 0.25;
  ctrl->applyConstraintsWrapper(
    curvature, curr_speed, pose_cost, approach_path, true,
    linear_vel, sign);
  EXPECT_NEAR(linear_vel, 1.0, 0.01);  // approach scaling skipped, unaffected

  // now try with cost regulation (turn off velocity and only cost)
  // ctrl->setCostRegulationScaling();
  // ctrl->resetVelocityRegulationScaling();
  // curvature = 0.0;

  // min changable cost
  // pose_cost = 1;
  // linear_vel = 0.5;
  // curr_speed.linear.x = 0.5;
  // ctrl->applyConstraintsWrapper(
  //   dist_error, lookahead_dist, curvature, curr_speed, pose_cost, linear_vel);
  // EXPECT_NEAR(linear_vel, 0.498, 0.01);

  // max changing cost
  // pose_cost = 127;
  // curr_speed.linear.x = 0.255;
  // ctrl->applyConstraintsWrapper(
  //   dist_error, lookahead_dist, curvature, curr_speed, pose_cost, linear_vel);
  // EXPECT_NEAR(linear_vel, 0.255, 0.01);

  // over max cost thresh
  // pose_cost = 200;
  // curr_speed.linear.x = 0.25;
  // ctrl->applyConstraintsWrapper(
  //   dist_error, lookahead_dist, curvature, curr_speed, pose_cost, linear_vel);
  // EXPECT_NEAR(linear_vel, 0.25, 0.01);

  // test kinematic clamping
  // pose_cost = 200;
  // curr_speed.linear.x = 1.0;
  // ctrl->applyConstraintsWrapper(
  //   dist_error, lookahead_dist, curvature, curr_speed, pose_cost, linear_vel);
  // EXPECT_NEAR(linear_vel, 0.5, 0.01);
}

TEST(RegulatedPurePursuitTest, applyTrackingFeedbackConstraints)
{
  auto ctrl = std::make_shared<BasicAPIRPP>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testRPP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  ctrl->configure(node, name, tf, costmap);

  // 机器人在路径上（原点在直线路径首点），cross_track_error=0，用来隔离①。
  auto on_path = path_utils::generate_path(
    geometry_msgs::msg::PoseStamped(), 0.1, {
    std::make_unique<path_utils::Straight>(2.0)
  });

  // ②朝向误差减速隔离（curvature=0 关闭③，路径贴合关闭①）：
  // 默认 heading_error_scaling_min_angle=0.35，heading_error_scaling_max_angle=0.785，
  // min_heading_speed_scale=0.3。
  double linear_vel = 1.0, xte = 0.0;
  ctrl->applyTrackingFeedbackConstraintsWrapper(on_path, 0.0, 0.0, 0.2, linear_vel, xte);
  EXPECT_DOUBLE_EQ(linear_vel, 1.0);  // 阈值内不动

  linear_vel = 1.0;
  ctrl->applyTrackingFeedbackConstraintsWrapper(on_path, 0.0, 0.0, 0.5675, linear_vel, xte);
  EXPECT_NEAR(linear_vel, 0.65, 0.01);  // 区间中点，scale=1-(1-0.3)*0.5=0.65

  linear_vel = 1.0;
  ctrl->applyTrackingFeedbackConstraintsWrapper(on_path, 0.0, 0.0, 0.785, linear_vel, xte);
  EXPECT_NEAR(linear_vel, 0.3, 0.01);  // 达 rotate 阈值，触底 min_scale

  linear_vel = 1.0;
  ctrl->applyTrackingFeedbackConstraintsWrapper(on_path, 0.0, 0.0, 1.2, linear_vel, xte);
  EXPECT_NEAR(linear_vel, 0.3, 0.01);  // 超出区间仍钳在 min_scale，不反弹回升

  // ③角速度饱和联动降速隔离（angle=0 关闭②，near_curvature=0 只看远端 curvature）：
  // 默认 angular_gain=1.0, max_angular_vel=1.5。
  linear_vel = 1.0;
  ctrl->applyTrackingFeedbackConstraintsWrapper(on_path, 3.0, 0.0, 0.0, linear_vel, xte);
  EXPECT_NEAR(linear_vel, 0.5, 0.01);  // wz_demand=1*1*3=3>1.5 -> v=1.5/3=0.5

  linear_vel = 1.0;
  ctrl->applyTrackingFeedbackConstraintsWrapper(on_path, 0.5, 0.0, 0.0, linear_vel, xte);
  EXPECT_DOUBLE_EQ(linear_vel, 1.0);  // wz_demand=1*1*0.5=0.5<1.5 -> 不触发

  // 组合：②先降速，③在②的结果之上继续按当前 curvature 校正（不是简单相乘，
  // ③直接算出满足 wz 上限的绝对值）。
  linear_vel = 1.0;
  ctrl->applyTrackingFeedbackConstraintsWrapper(on_path, 10.0, 0.0, 0.785, linear_vel, xte);
  EXPECT_NEAR(linear_vel, 0.15, 0.01);  // ②先降到0.3，wz=1*0.3*10=3>1.5 -> ③再降到1.5/10=0.15

  // 近远双点：near_curvature 比远端 curvature 更尖锐时，③改用 near_curvature。
  linear_vel = 1.0;
  ctrl->applyTrackingFeedbackConstraintsWrapper(on_path, 0.5, 3.0, 0.0, linear_vel, xte);
  EXPECT_NEAR(linear_vel, 0.5, 0.01);  // far=0.5 本不触发，near=3.0 触发 -> v=1.5/3=0.5

  // 反向：远端 curvature 比 near_curvature 更尖锐时，③仍取远端（对称验证 max 逻辑）。
  linear_vel = 1.0;
  ctrl->applyTrackingFeedbackConstraintsWrapper(on_path, 3.0, 0.5, 0.0, linear_vel, xte);
  EXPECT_NEAR(linear_vel, 0.5, 0.01);  // far=3.0 触发 -> v=1.5/3=0.5，near 更小不影响结果

  // 两点都在阈值内（光滑路径的常见情况）：不触发，行为与只有远端曲率时一致。
  linear_vel = 1.0;
  ctrl->applyTrackingFeedbackConstraintsWrapper(on_path, 0.4, 0.45, 0.0, linear_vel, xte);
  EXPECT_DOUBLE_EQ(linear_vel, 1.0);
}

// shouldRotateToPath 滞回：进入转向用 rotate_to_path_entry_angle（0.785），
// 已在转向时退出用独立的更小阈值 rotate_to_path_exit_angle（0.35）——角度落在
// [0.35, 0.785] 区间时，是否仍判定“应转向”取决于上一周期的状态，不是单一阈值。
TEST(RegulatedPurePursuitTest, shouldRotateToPathHysteresis)
{
  auto ctrl = std::make_shared<BasicAPIRPP>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testRPP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  ctrl->configure(node, name, tf, costmap);

  auto carrotAt = [](double angle) {
      geometry_msgs::msg::PoseStamped p;
      p.pose.position.x = std::cos(angle);
      p.pose.position.y = std::sin(angle);
      return p;
    };
  double angle_rtn;

  // 区间内（0.5 rad），未在转向 -> 用进入阈值 0.785，不触发
  EXPECT_FALSE(ctrl->shouldRotateToPathWrapper(carrotAt(0.5), angle_rtn));

  // 超过进入阈值 -> 触发，状态切到"正在转向"
  EXPECT_TRUE(ctrl->shouldRotateToPathWrapper(carrotAt(0.9), angle_rtn));

  // 回落到区间内（0.5 rad）：若仍用进入阈值 0.785 会立刻判定"不转向"，
  // 滞回下应改用退出阈值 0.35，0.5>0.35 仍判定"转向"，不发生抖动。
  EXPECT_TRUE(ctrl->shouldRotateToPathWrapper(carrotAt(0.5), angle_rtn));

  // 真正降到退出阈值以下 -> 才退出转向状态
  EXPECT_FALSE(ctrl->shouldRotateToPathWrapper(carrotAt(0.2), angle_rtn));

  // 退出后状态复位，再次落在区间内 -> 又需要用进入阈值判定，不触发
  EXPECT_FALSE(ctrl->shouldRotateToPathWrapper(carrotAt(0.5), angle_rtn));
}

TEST(RegulatedPurePursuitTest, testDynamicParameter)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("Smactest");
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap->on_configure(rclcpp_lifecycle::State());
  auto ctrl =
    std::make_unique<nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController>();
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  ctrl->configure(node, "test", tf, costmap);
  ctrl->activate();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.desired_linear_vel", 1.0),
      rclcpp::Parameter("test.lookahead_dist", 7.0),
      rclcpp::Parameter("test.max_lookahead_dist", 7.0),
      rclcpp::Parameter("test.min_lookahead_dist", 6.0),
      rclcpp::Parameter("test.lookahead_time", 1.8),
      rclcpp::Parameter("test.rotate_to_path_angular_vel", 18.0),
      rclcpp::Parameter("test.min_approach_linear_velocity", 1.0),
      rclcpp::Parameter("test.max_allowed_time_to_collision_up_to_carrot", 2.0),
      rclcpp::Parameter("test.cost_scaling_dist", 2.0),
      rclcpp::Parameter("test.cost_scaling_gain", 4.0),
      rclcpp::Parameter("test.regulated_linear_scaling_min_radius", 10.0),
      rclcpp::Parameter("test.transform_tolerance", 30.0),
      rclcpp::Parameter("test.max_angular_accel", 3.0),
      rclcpp::Parameter("test.rotate_to_path_entry_angle", 0.7),
      rclcpp::Parameter("test.regulated_linear_scaling_min_speed", 4.0),
      rclcpp::Parameter("test.use_velocity_scaled_lookahead_dist", false),
      rclcpp::Parameter("test.use_regulated_linear_velocity_scaling", false),
      rclcpp::Parameter("test.use_cost_regulated_linear_velocity_scaling", false),
      rclcpp::Parameter("test.allow_reversing", false),
      rclcpp::Parameter("test.use_rotate_to_heading", false)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(node->get_parameter("test.desired_linear_vel").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.lookahead_dist").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("test.max_lookahead_dist").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("test.min_lookahead_dist").as_double(), 6.0);
  EXPECT_EQ(node->get_parameter("test.lookahead_time").as_double(), 1.8);
  EXPECT_EQ(node->get_parameter("test.rotate_to_path_angular_vel").as_double(), 18.0);
  EXPECT_EQ(node->get_parameter("test.min_approach_linear_velocity").as_double(), 1.0);
  EXPECT_EQ(
    node->get_parameter(
      "test.max_allowed_time_to_collision_up_to_carrot").as_double(), 2.0);
  EXPECT_EQ(node->get_parameter("test.cost_scaling_dist").as_double(), 2.0);
  EXPECT_EQ(node->get_parameter("test.cost_scaling_gain").as_double(), 4.0);
  EXPECT_EQ(node->get_parameter("test.regulated_linear_scaling_min_radius").as_double(), 10.0);
  EXPECT_EQ(node->get_parameter("test.transform_tolerance").as_double(), 30.0);
  EXPECT_EQ(node->get_parameter("test.max_angular_accel").as_double(), 3.0);
  EXPECT_EQ(node->get_parameter("test.rotate_to_path_entry_angle").as_double(), 0.7);
  EXPECT_EQ(node->get_parameter("test.regulated_linear_scaling_min_speed").as_double(), 4.0);
  EXPECT_EQ(node->get_parameter("test.use_velocity_scaled_lookahead_dist").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.use_regulated_linear_velocity_scaling").as_bool(), false);
  EXPECT_EQ(
    node->get_parameter(
      "test.use_cost_regulated_linear_velocity_scaling").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.allow_reversing").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.use_rotate_to_heading").as_bool(), false);
}

class TransformGlobalPlanTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ctrl_ = std::make_shared<BasicAPIRPP>();
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testRPP");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  }

  void configure_costmap(uint16_t width, double resolution)
  {
    constexpr char costmap_frame[] = "test_costmap_frame";
    constexpr char robot_frame[] = "test_robot_frame";

    auto results = costmap_->set_parameters(
    {
      rclcpp::Parameter("global_frame", costmap_frame),
      rclcpp::Parameter("robot_base_frame", robot_frame),
      rclcpp::Parameter("width", width),
      rclcpp::Parameter("height", width),
      rclcpp::Parameter("resolution", resolution)
    });
    for (const auto & result : results) {
      EXPECT_TRUE(result.successful) << result.reason;
    }

    rclcpp_lifecycle::State state;
    costmap_->on_configure(state);
  }

  void configure_controller(double max_robot_pose_search_dist)
  {
    std::string plugin_name = "test_rpp";
    nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name + ".max_robot_pose_search_dist",
      rclcpp::ParameterValue(max_robot_pose_search_dist));
    ctrl_->configure(node_, plugin_name, tf_buffer_, costmap_);
  }

  void setup_transforms(geometry_msgs::msg::Point & robot_position)
  {
    transform_time_ = node_->get_clock()->now();
    // Note: transforms go parent to child

    // We will have a separate path and costmap frame for completeness,
    // but we will leave them cooincident for convenience.
    geometry_msgs::msg::TransformStamped path_to_costmap;
    path_to_costmap.header.frame_id = PATH_FRAME;
    path_to_costmap.header.stamp = transform_time_;
    path_to_costmap.child_frame_id = COSTMAP_FRAME;
    path_to_costmap.transform.translation.x = 0.0;
    path_to_costmap.transform.translation.y = 0.0;
    path_to_costmap.transform.translation.z = 0.0;
    path_to_costmap.transform.rotation.w = 1.0;

    geometry_msgs::msg::TransformStamped costmap_to_robot;
    costmap_to_robot.header.frame_id = COSTMAP_FRAME;
    costmap_to_robot.header.stamp = transform_time_;
    costmap_to_robot.child_frame_id = ROBOT_FRAME;
    costmap_to_robot.transform.translation.x = robot_position.x;
    costmap_to_robot.transform.translation.y = robot_position.y;
    costmap_to_robot.transform.translation.z = robot_position.z;
    costmap_to_robot.transform.rotation.w = 1.0;

    tf2_msgs::msg::TFMessage tf_message;
    tf_message.transforms = {
      path_to_costmap,
      costmap_to_robot
    };
    for (const auto & transform : tf_message.transforms) {
      tf_buffer_->setTransform(transform, "test", false);
    }
    tf_buffer_->setUsingDedicatedThread(true);  // lying to let it do transforms
  }

  static constexpr char PATH_FRAME[] = "test_path_frame";
  static constexpr char COSTMAP_FRAME[] = "test_costmap_frame";
  static constexpr char ROBOT_FRAME[] = "test_robot_frame";

  std::shared_ptr<BasicAPIRPP> ctrl_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Time transform_time_;
};

// This tests that not only should nothing get pruned on a costmap
// that contains the entire global_plan, and also that it doesn't skip to the end of the path
// which is closer to the robot pose than the start.
TEST_F(TransformGlobalPlanTest, no_pruning_on_large_costmap)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = COSTMAP_FRAME;
  robot_pose.header.stamp = transform_time_;
  robot_pose.pose.position.x = -0.1;
  robot_pose.pose.position.y = 0.0;
  robot_pose.pose.position.z = 0.0;
  // A really big costmap
  // the max_costmap_extent should be 50m
  configure_costmap(100u, 0.1);
  configure_controller(5.0);
  setup_transforms(robot_pose.pose.position);

  // Set up test path;

  geometry_msgs::msg::PoseStamped start_of_path;
  start_of_path.header.frame_id = PATH_FRAME;
  start_of_path.header.stamp = transform_time_;
  start_of_path.pose.position.x = 0.0;
  start_of_path.pose.position.y = 0.0;
  start_of_path.pose.position.z = 0.0;

  constexpr double spacing = 0.1;
  constexpr double circle_radius = 1.0;

  auto global_plan = path_utils::generate_path(
    start_of_path, spacing, {
    std::make_unique<path_utils::LeftCircle>(circle_radius)
  });

  ctrl_->setPlan(global_plan);

  // Transform the plan

  auto transformed_plan = ctrl_->transformGlobalPlanWrapper(robot_pose);
  EXPECT_EQ(transformed_plan.poses.size(), global_plan.poses.size());
}

TEST_F(TransformGlobalPlanTest, future_stamped_pose_uses_latest_common_time)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = COSTMAP_FRAME;
  robot_pose.pose.position.x = -0.1;
  robot_pose.pose.orientation.w = 1.0;
  configure_costmap(100u, 0.1);
  configure_controller(5.0);
  setup_transforms(robot_pose.pose.position);
  robot_pose.header.stamp = transform_time_ + rclcpp::Duration::from_seconds(1.0);

  geometry_msgs::msg::PoseStamped start_of_path;
  start_of_path.header.frame_id = PATH_FRAME;
  start_of_path.header.stamp = transform_time_;
  start_of_path.pose.orientation.w = 1.0;
  auto global_plan = path_utils::generate_path(
    start_of_path, 0.1, {std::make_unique<path_utils::Straight>(1.0)});
  constexpr double goal_yaw = 1.2;
  global_plan.poses.back().pose.orientation.z = std::sin(goal_yaw / 2.0);
  global_plan.poses.back().pose.orientation.w = std::cos(goal_yaw / 2.0);
  ctrl_->setPlan(global_plan);

  geometry_msgs::msg::PoseStamped robot_pose_in_plan;
  const auto transformed_plan =
    ctrl_->transformGlobalPlanWrapper(robot_pose, &robot_pose_in_plan);

  EXPECT_FALSE(transformed_plan.poses.empty());
  EXPECT_EQ(rclcpp::Time(transformed_plan.header.stamp), transform_time_);
  EXPECT_EQ(rclcpp::Time(robot_pose_in_plan.header.stamp), transform_time_);
  EXPECT_EQ(robot_pose_in_plan.header.frame_id, PATH_FRAME);
  ASSERT_EQ(transformed_plan.poses.size(), global_plan.poses.size());
  EXPECT_NEAR(robot_pose_in_plan.pose.position.x, robot_pose.pose.position.x, 1e-9);
  EXPECT_NEAR(
    transformed_plan.poses.back().pose.position.x,
    global_plan.poses.back().pose.position.x - robot_pose.pose.position.x, 1e-9);
  EXPECT_NEAR(
    transformed_plan.poses.back().pose.orientation.z,
    global_plan.poses.back().pose.orientation.z, 1e-9);
  EXPECT_NEAR(
    transformed_plan.poses.back().pose.orientation.w,
    global_plan.poses.back().pose.orientation.w, 1e-9);
}

TEST_F(TransformGlobalPlanTest, trajectory_stays_in_source_frame_without_tf_lookup)
{
  configure_costmap(100u, 0.1);
  configure_controller(5.0);

  nav_interfaces::msg::RobotTrajectory trajectory;
  trajectory.header.frame_id = PATH_FRAME;
  trajectory.points.resize(2);
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    trajectory.points[i].pose.header.frame_id = PATH_FRAME;
    trajectory.points[i].pose.pose.position.x = static_cast<double>(i);
    trajectory.points[i].pose.pose.orientation.w = 1.0;
    trajectory.points[i].longitudinal_velocity = 0.2;
  }

  ctrl_->setTrajectoryWrapper(trajectory);

  EXPECT_TRUE(ctrl_->trajectoryValidForTest());
  EXPECT_EQ(ctrl_->trajectoryFrameForTest(), PATH_FRAME);
}

// This plan shouldn't get pruned because of the costmap,
// but should be half pruned because it is halfway around the circle
TEST_F(TransformGlobalPlanTest, transform_start_selection)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = COSTMAP_FRAME;
  robot_pose.header.stamp = transform_time_;
  robot_pose.pose.position.x = 0.0;
  robot_pose.pose.position.y = 4.0;  // on the other side of the circle
  robot_pose.pose.position.z = 0.0;
  // Could set orientation going the other way, but RPP doesn't care
  constexpr double spacing = 0.1;
  constexpr double circle_radius = 2.0;  // diameter 4

  // A really big costmap
  // the max_costmap_extent should be 50m
  configure_costmap(100u, 0.1);
  // This should just be at least half the circumference: pi*r ~= 6
  constexpr double max_robot_pose_search_dist = 10.0;
  configure_controller(max_robot_pose_search_dist);
  setup_transforms(robot_pose.pose.position);

  // Set up test path;

  geometry_msgs::msg::PoseStamped start_of_path;
  start_of_path.header.frame_id = PATH_FRAME;
  start_of_path.header.stamp = transform_time_;
  start_of_path.pose.position.x = 0.0;
  start_of_path.pose.position.y = 0.0;
  start_of_path.pose.position.z = 0.0;

  auto global_plan = path_utils::generate_path(
    start_of_path, spacing, {
    std::make_unique<path_utils::LeftCircle>(circle_radius)
  });

  ctrl_->setPlan(global_plan);

  // Transform the plan
  auto transformed_plan = ctrl_->transformGlobalPlanWrapper(robot_pose);
  EXPECT_NEAR(transformed_plan.poses.size(), global_plan.poses.size() / 2, 1);
  EXPECT_NEAR(transformed_plan.poses[0].pose.position.x, 0.0, 0.5);
  EXPECT_NEAR(transformed_plan.poses[0].pose.position.y, 0.0, 0.5);
}

// This should throw an exception when all poses are outside of the costmap
TEST_F(TransformGlobalPlanTest, all_poses_outside_of_costmap)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = COSTMAP_FRAME;
  robot_pose.header.stamp = transform_time_;
  // far away from the path
  robot_pose.pose.position.x = 1000.0;
  robot_pose.pose.position.y = 1000.0;
  robot_pose.pose.position.z = 0.0;
  // Could set orientation going the other way, but RPP doesn't care
  constexpr double spacing = 0.1;
  constexpr double circle_radius = 2.0;  // diameter 4

  // A "normal" costmap
  // the max_costmap_extent should be 50m
  configure_costmap(10u, 0.1);
  // This should just be at least half the circumference: pi*r ~= 6
  constexpr double max_robot_pose_search_dist = 10.0;
  configure_controller(max_robot_pose_search_dist);
  setup_transforms(robot_pose.pose.position);

  // Set up test path;

  geometry_msgs::msg::PoseStamped start_of_path;
  start_of_path.header.frame_id = PATH_FRAME;
  start_of_path.header.stamp = transform_time_;
  start_of_path.pose.position.x = 0.0;
  start_of_path.pose.position.y = 0.0;
  start_of_path.pose.position.z = 0.0;

  auto global_plan = path_utils::generate_path(
    start_of_path, spacing, {
    std::make_unique<path_utils::LeftCircle>(circle_radius)
  });

  ctrl_->setPlan(global_plan);

  // Transform the plan
  EXPECT_THROW(ctrl_->transformGlobalPlanWrapper(robot_pose), nav2_core::PlannerException);
}

// Should shortcut the circle if the circle is shorter than max_robot_pose_search_dist
TEST_F(TransformGlobalPlanTest, good_circle_shortcut)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = COSTMAP_FRAME;
  robot_pose.header.stamp = transform_time_;
  // far away from the path
  robot_pose.pose.position.x = -0.1;
  robot_pose.pose.position.y = 0.0;
  robot_pose.pose.position.z = 0.0;
  // Could set orientation going the other way, but RPP doesn't care
  constexpr double spacing = 0.1;
  constexpr double circle_radius = 2.0;  // diameter 4

  // A "normal" costmap
  // the max_costmap_extent should be 50m
  configure_costmap(100u, 0.1);
  // This should just be at least the circumference: 2*pi*r ~= 12
  constexpr double max_robot_pose_search_dist = 15.0;
  configure_controller(max_robot_pose_search_dist);
  setup_transforms(robot_pose.pose.position);

  // Set up test path;

  geometry_msgs::msg::PoseStamped start_of_path;
  start_of_path.header.frame_id = PATH_FRAME;
  start_of_path.header.stamp = transform_time_;
  start_of_path.pose.position.x = 0.0;
  start_of_path.pose.position.y = 0.0;
  start_of_path.pose.position.z = 0.0;

  auto global_plan = path_utils::generate_path(
    start_of_path, spacing, {
    std::make_unique<path_utils::LeftCircle>(circle_radius)
  });

  ctrl_->setPlan(global_plan);

  // Transform the plan
  auto transformed_plan = ctrl_->transformGlobalPlanWrapper(robot_pose);
  EXPECT_NEAR(transformed_plan.poses.size(), 1, 1);
  EXPECT_NEAR(transformed_plan.poses[0].pose.position.x, 0.0, 0.5);
  EXPECT_NEAR(transformed_plan.poses[0].pose.position.y, 0.0, 0.5);
}

// Simple costmap pruning on a straight line
TEST_F(TransformGlobalPlanTest, costmap_pruning)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = COSTMAP_FRAME;
  robot_pose.header.stamp = transform_time_;
  // far away from the path
  robot_pose.pose.position.x = -0.1;
  robot_pose.pose.position.y = 0.0;
  robot_pose.pose.position.z = 0.0;
  // Could set orientation going the other way, but RPP doesn't care
  constexpr double spacing = 1.0;

  // A "normal" costmap
  // the max_costmap_extent should be 50m
  configure_costmap(20u, 0.5);
  constexpr double max_robot_pose_search_dist = 10.0;
  configure_controller(max_robot_pose_search_dist);
  setup_transforms(robot_pose.pose.position);

  // Set up test path;

  geometry_msgs::msg::PoseStamped start_of_path;
  start_of_path.header.frame_id = PATH_FRAME;
  start_of_path.header.stamp = transform_time_;
  start_of_path.pose.position.x = 0.0;
  start_of_path.pose.position.y = 0.0;
  start_of_path.pose.position.z = 0.0;

  constexpr double path_length = 100.0;

  auto global_plan = path_utils::generate_path(
    start_of_path, spacing, {
    std::make_unique<path_utils::Straight>(path_length)
  });

  ctrl_->setPlan(global_plan);

  // Transform the plan
  auto transformed_plan = ctrl_->transformGlobalPlanWrapper(robot_pose);
  EXPECT_NEAR(transformed_plan.poses.size(), 10u, 1);
  EXPECT_NEAR(transformed_plan.poses[0].pose.position.x, 0.0, 0.5);
  EXPECT_NEAR(transformed_plan.poses[0].pose.position.y, 0.0, 0.5);
}

// Should prune out later portions of the path that come back into the costmap
TEST_F(TransformGlobalPlanTest, prune_after_leaving_costmap)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = COSTMAP_FRAME;
  robot_pose.header.stamp = transform_time_;
  // far away from the path
  robot_pose.pose.position.x = -0.1;
  robot_pose.pose.position.y = 0.0;
  robot_pose.pose.position.z = 0.0;
  // Could set orientation going the other way, but RPP doesn't care
  constexpr double spacing = 1.0;

  // A "normal" costmap
  // the max_costmap_extent should be 50m
  configure_costmap(20u, 0.5);
  constexpr double max_robot_pose_search_dist = 10.0;
  configure_controller(max_robot_pose_search_dist);
  setup_transforms(robot_pose.pose.position);

  // Set up test path;

  geometry_msgs::msg::PoseStamped start_of_path;
  start_of_path.header.frame_id = PATH_FRAME;
  start_of_path.header.stamp = transform_time_;
  start_of_path.pose.position.x = 0.0;
  start_of_path.pose.position.y = 0.0;
  start_of_path.pose.position.z = 0.0;

  constexpr double path_length = 100.0;

  auto global_plan = path_utils::generate_path(
    start_of_path, spacing, {
    std::make_unique<path_utils::Straight>(path_length),
    std::make_unique<path_utils::LeftTurnAround>(1.0),
    std::make_unique<path_utils::Straight>(path_length)
  });

  ctrl_->setPlan(global_plan);

  // Transform the plan
  auto transformed_plan = ctrl_->transformGlobalPlanWrapper(robot_pose);
  // This should be essentially the same as the regular straight path
  EXPECT_NEAR(transformed_plan.poses.size(), 10u, 1);
  EXPECT_NEAR(transformed_plan.poses[0].pose.position.x, 0.0, 0.5);
  EXPECT_NEAR(transformed_plan.poses[0].pose.position.y, 0.0, 0.5);
}
