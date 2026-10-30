#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include "nav2_frenet_ilqr_controller/frenet_trajectory_planner.hpp"

namespace fp = nav2_frenet_ilqr_controller::frenet_planner;

TEST(FrenetPolynomial, QuarticMeetsBoundaryConditions) {
  fp::QuarticTrajectoryPlanner planner;
  ASSERT_TRUE(planner.setCoefficients(1.0, 0.5, 0.2, 2.0, -0.1, 0.0, 3.0));
  EXPECT_NEAR(planner.dx(0.0), 0.5, 1e-9);
  EXPECT_NEAR(planner.ddx(0.0), 0.2, 1e-9);
  EXPECT_NEAR(planner.dx(3.0), 2.0, 1e-9);
  EXPECT_NEAR(planner.ddx(3.0), -0.1, 1e-9);
}

TEST(FrenetPolynomial, QuinticMeetsBoundaryConditions) {
  fp::QuinticTrajectoryPlanner planner;
  ASSERT_TRUE(planner.setCoefficients(1.0, 0.5, 0.2, 4.0, -0.3, 0.1, 0.0, 2.5));
  EXPECT_NEAR(planner.x(0.0), 1.0, 1e-9);
  EXPECT_NEAR(planner.dx(0.0), 0.5, 1e-9);
  EXPECT_NEAR(planner.ddx(0.0), 0.2, 1e-9);
  EXPECT_NEAR(planner.x(2.5), 4.0, 1e-9);
  EXPECT_NEAR(planner.dx(2.5), -0.3, 1e-9);
  EXPECT_NEAR(planner.ddx(2.5), 0.1, 1e-9);
}

TEST(FrenetFrame, StraightLineRoundTrip) {
  fp::FrenetFrameConverter converter;
  converter.createSegments({{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}});
  ASSERT_NEAR(converter.getArclength(), 10.0, 1e-9);

  fp::CartesianState cartesian = fp::CartesianState::Zero();
  cartesian[0] = 3.0;
  cartesian[3] = 1.0;
  const fp::FrenetState frenet = converter.convertCartesian2FrenetForSegment(cartesian, 0U);
  EXPECT_NEAR(frenet[0], 3.0, 1e-9);
  EXPECT_NEAR(frenet[3], 1.0, 1e-9);

  fp::FrenetState sample = fp::FrenetState::Zero();
  sample[0] = 6.0;
  sample[3] = 0.5;
  const fp::CartesianState converted = converter.convertFrenet2CartesianForSegment(sample, 0U);
  EXPECT_NEAR(converted[0], 6.0, 1e-9);
  EXPECT_NEAR(converted[3], 0.5, 1e-9);
}

TEST(FrenetFrame, LeftArcRoundTripPreservesSignedLateralState) {
  fp::FrenetFrameConverter converter;
  converter.createSegments({{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}});
  ASSERT_EQ(converter.segmentCount(), 3U);

  fp::FrenetState sample = fp::FrenetState::Zero();
  sample << 0.35, 1.2, 0.3, 0.25, 0.2, -0.1;
  const fp::CartesianState cartesian = converter.convertFrenet2CartesianForSegment(sample, 1U);
  EXPECT_GT(cartesian[3], 0.0);
  const fp::FrenetState round_trip = converter.convertCartesian2FrenetForSegment(cartesian, 1U);
  for (Eigen::Index index = 0; index < sample.size(); ++index) {
    EXPECT_NEAR(round_trip[index], sample[index], 1e-9) << "state index " << index;
  }
}

TEST(FrenetFrame, RightArcRoundTripPreservesSignedLateralState) {
  fp::FrenetFrameConverter converter;
  converter.createSegments({{0.0, 0.0}, {2.0, 0.0}, {2.0, -2.0}});
  ASSERT_EQ(converter.segmentCount(), 3U);

  fp::FrenetState sample = fp::FrenetState::Zero();
  sample << 0.35, 1.2, 0.3, 0.25, 0.2, -0.1;
  const fp::CartesianState cartesian = converter.convertFrenet2CartesianForSegment(sample, 1U);
  EXPECT_GT(cartesian[3], 0.0);
  const fp::FrenetState round_trip = converter.convertCartesian2FrenetForSegment(cartesian, 1U);
  for (Eigen::Index index = 0; index < sample.size(); ++index) {
    EXPECT_NEAR(round_trip[index], sample[index], 1e-9) << "state index " << index;
  }
}

TEST(FrenetFrame, ZeroSpeedArcUsesPathTangentForYaw) {
  fp::FrenetFrameConverter converter;
  converter.createSegments({{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}});
  fp::FrenetState sample = fp::FrenetState::Zero();
  sample[0] = 0.35;
  sample[3] = 0.25;
  const fp::CartesianState cartesian = converter.convertFrenet2CartesianForSegment(sample, 1U);

  constexpr double delta = 1e-4;
  fp::FrenetState before = sample;
  fp::FrenetState after = sample;
  before[0] -= delta;
  after[0] += delta;
  const fp::CartesianState before_cartesian =
    converter.convertFrenet2CartesianForSegment(before, 1U);
  const fp::CartesianState after_cartesian =
    converter.convertFrenet2CartesianForSegment(after, 1U);
  const double expected_yaw = std::atan2(
    after_cartesian[3] - before_cartesian[3],
    after_cartesian[0] - before_cartesian[0]);
  EXPECT_NEAR(cartesian[6], expected_yaw, 1e-6);
}

TEST(FrenetPlanner, SamplesTimeVelocityAndLateralCandidates) {
  fp::FrenetTrajectoryPlannerConfig config;
  config.min_lateral_distance = -0.5;
  config.max_lateral_distance = 0.5;
  config.step_lateral_distance = 0.5;
  config.min_longitudinal_velocity = 0.3;
  config.max_longitudinal_velocity = 0.5;
  config.step_longitudinal_velocity = 0.1;
  config.min_time_interval = 1.0;
  config.max_time_interval = 1.4;
  config.step_time_interval = 0.2;
  config.max_state_in_trajectory = 40;
  config.dt = 0.05;

  fp::FrenetTrajectoryPlanner planner(config);
  fp::CartesianState robot = fp::CartesianState::Zero();
  robot[0] = 0.0;
  robot[3] = 0.0;
  const auto result = planner.planByWaypoint(robot, {{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}});
  EXPECT_GE(result.cartesian.size(), 2U);
  EXPECT_GE(result.frenet.size(), 2U);
  EXPECT_NEAR(result.frenet.front()[0], 0.0, 1e-9);
  EXPECT_NEAR(result.arclength, 10.0, 1e-9);
}

TEST(FrenetPlanner, MergeAndStopUsesStoppingDistancePrediction) {
  fp::FrenetTrajectoryPlanner planner;
  fp::FrenetState moving = fp::FrenetState::Zero();
  moving[1] = 0.5;
  EXPECT_TRUE(planner.shouldMergeAndStop(moving, 0.1, 4.0));
  EXPECT_FALSE(planner.shouldMergeAndStop(moving, 100.0, 4.0));
}

TEST(FrenetPlanner, PolicyCanRejectEveryCandidate) {
  class RejectAllPolicy : public fp::policies::Policy
  {
public:
    bool checkIfFeasible(const fp::FrenetTrajectory &, const fp::CartesianTrajectory &) override
    {
      return false;
    }
  };

  fp::FrenetTrajectoryPlannerConfig config;
  config.min_lateral_distance = 0.0;
  config.max_lateral_distance = 0.1;
  config.step_lateral_distance = 0.1;
  config.min_longitudinal_velocity = 0.3;
  config.max_longitudinal_velocity = 0.4;
  config.step_longitudinal_velocity = 0.1;
  config.min_time_interval = 1.0;
  config.max_time_interval = 1.1;
  config.step_time_interval = 0.1;
  config.max_state_in_trajectory = 20;
  config.dt = 0.05;

  fp::FrenetTrajectoryPlanner planner(config);
  planner.addPolicy(std::make_shared<RejectAllPolicy>());
  fp::CartesianState robot = fp::CartesianState::Zero();
  const auto result = planner.planByWaypoint(robot, {{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}});
  EXPECT_TRUE(result.cartesian.empty());
}

TEST(FrenetPlanner, SamplingMatchesUpstreamHalfOpenRanges) {
  class RejectAllPolicy : public fp::policies::Policy
  {
public:
    bool checkIfFeasible(const fp::FrenetTrajectory &, const fp::CartesianTrajectory &) override
    {
      return false;
    }
  };

  fp::FrenetTrajectoryPlannerConfig config;
  config.min_lateral_distance = -0.5;
  config.max_lateral_distance = 0.5;
  config.step_lateral_distance = 0.5;
  config.min_longitudinal_velocity = 0.3;
  config.max_longitudinal_velocity = 0.5;
  config.step_longitudinal_velocity = 0.1;
  config.min_time_interval = 1.0;
  config.max_time_interval = 1.4;
  config.step_time_interval = 0.2;
  config.max_state_in_trajectory = 10;
  config.dt = 0.05;

  fp::FrenetTrajectoryPlanner planner(config);
  planner.addPolicy(std::make_shared<RejectAllPolicy>());
  auto debug_info = std::make_shared<fp::DebugInfo>();
  fp::CartesianState robot = fp::CartesianState::Zero();
  const auto result = planner.planByWaypoint(
    robot, {{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}}, debug_info);

  EXPECT_TRUE(result.cartesian.empty());
  // Upstream excludes the upper bound of each range: 2 x 2 x 2 candidates.
  EXPECT_EQ(debug_info->cartesian_trajectories.size(), 8U);
  EXPECT_EQ(debug_info->candidate_count, 8U);
  ASSERT_EQ(debug_info->policy_reject_counts.size(), 1U);
  EXPECT_EQ(debug_info->policy_reject_counts[0], 8U);
  EXPECT_TRUE(debug_info->has_first_reject);
}

TEST(FrenetPlanner, RobotBehindPathStartStillProducesFeasibleCandidates) {
  fp::FrenetTrajectoryPlannerConfig config;
  config.min_lateral_distance = -0.5;
  config.max_lateral_distance = 0.5;
  config.step_lateral_distance = 0.25;
  config.min_longitudinal_velocity = 0.2;
  config.max_longitudinal_velocity = 0.4;
  config.step_longitudinal_velocity = 0.1;
  config.min_time_interval = 1.0;
  config.max_time_interval = 1.2;
  config.step_time_interval = 0.1;
  config.max_state_in_trajectory = 25;
  config.dt = 0.05;

  fp::FrenetTrajectoryPlanner planner(config);
  auto debug_info = std::make_shared<fp::DebugInfo>();
  fp::CartesianState robot = fp::CartesianState::Zero();
  // A few millimetres behind the path start produces a negative raw projection.
  // This used to reject every candidate before obstacle policy checks ran.
  robot[0] = -0.03;
  const auto result = planner.planByWaypoint(
    robot, {{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}}, debug_info);

  ASSERT_GE(result.cartesian.size(), 2U);
  ASSERT_GE(result.frenet.size(), 2U);
  EXPECT_NEAR(result.frenet.front()[0], 0.0, 1e-9);
  EXPECT_NEAR(result.cartesian.front()[0], robot[0], 1e-9);
  EXPECT_GT(debug_info->candidate_count, 0U);
  EXPECT_LT(debug_info->invalid_cartesian_count, debug_info->candidate_count);
}

TEST(FrenetPlanner, CostPrefersRequestedLateralOffset) {
  class LateralCost : public fp::costs::Cost
  {
public:
    explicit LateralCost(double preferred)
    : preferred_(preferred) {}
    double cost(const fp::FrenetTrajectory & trajectory, const fp::CartesianTrajectory &) override
    {
      double sum = 0.0;
      for (const auto & state : trajectory) {
        sum += std::abs(state[3] - preferred_);
      }
      return sum;
    }

private:
    double preferred_;
  };

  fp::FrenetTrajectoryPlannerConfig config;
  config.min_lateral_distance = -0.5;
  config.max_lateral_distance = 1.0;
  config.step_lateral_distance = 0.5;
  config.min_longitudinal_velocity = 0.4;
  config.max_longitudinal_velocity = 0.5;
  config.step_longitudinal_velocity = 0.1;
  config.min_time_interval = 1.0;
  config.max_time_interval = 1.1;
  config.step_time_interval = 0.1;
  config.max_state_in_trajectory = 25;
  config.dt = 0.05;

  fp::FrenetTrajectoryPlanner planner(config);
  planner.addCost(std::make_shared<LateralCost>(0.5));
  fp::CartesianState robot = fp::CartesianState::Zero();
  const auto result = planner.planByWaypoint(robot, {{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}});
  ASSERT_GE(result.frenet.size(), 2U);
  EXPECT_GT(result.frenet.back()[3], 0.1);
}

TEST(FrenetPlanner, StraightLineScenarioSamplingAndAvoidanceVerification) {
  // 1. 直线场景配置：
  // 采样区间：
  // 横向 [-1.0, 1.0, step=0.25] -> 8 个候选
  // 纵向速度 [0.2, 0.6, step=0.1] -> 4 个候选
  // 时域 [1.0, 2.0, step=0.5] -> 2 个候选
  // 单轮规划采样候选数：8 * 4 * 2 = 64
  // 规划器采用滚动多轮拼接（kMaxPlanningIterations = 5 或直至达到 max_state_in_trajectory = 25）
  // 25 个点 / (每次约 21 点) -> 刚好运行 2 轮迭代拼接 (64 * 2 = 128 条采样)
  fp::FrenetTrajectoryPlannerConfig config;
  config.min_lateral_distance = -1.0;
  config.max_lateral_distance = 1.0;
  config.step_lateral_distance = 0.25;
  config.min_longitudinal_velocity = 0.2;
  config.max_longitudinal_velocity = 0.6;
  config.step_longitudinal_velocity = 0.1;
  config.min_time_interval = 1.0;
  config.max_time_interval = 2.0;
  config.step_time_interval = 0.5;
  config.max_state_in_trajectory = 25;  // 限制为 1 轮迭代即可达到的状态数
  config.dt = 0.05;

  // 参考线：从 (0, 0) 到 (15, 0)
  const std::vector<fp::CartesianPoint> waypoints = {
    {0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}, {15.0, 0.0}
  };

  // 场景 A：无障碍物，机器人初速度 0.3m/s，无偏角，位于原点
  {
    fp::FrenetTrajectoryPlanner planner(config);
    auto debug_info = std::make_shared<fp::DebugInfo>();
    fp::CartesianState robot = fp::CartesianState::Zero();
    robot[1] = 0.3;  // dx = 0.3 m/s

    const auto result = planner.planByWaypoint(robot, waypoints, debug_info);

    // 验证采样完备性与有效性
    EXPECT_GT(debug_info->candidate_count, 0U);
    EXPECT_EQ(debug_info->short_candidate_count, 0U);
    EXPECT_EQ(debug_info->invalid_cartesian_count, 0U);
    EXPECT_EQ(debug_info->feasible_candidate_count, debug_info->candidate_count);
    ASSERT_FALSE(result.cartesian.empty());
    ASSERT_FALSE(result.frenet.empty());

    // 笛卡尔轨迹点数必须满足离散要求
    EXPECT_GE(result.cartesian.size(), 20U);
    // 验证起点对齐
    EXPECT_NEAR(result.cartesian.front()[0], 0.0, 1e-4);
    EXPECT_NEAR(result.cartesian.front()[3], 0.0, 1e-4);
  }

  // 场景 B：带障碍物策略，正前方 (x=0.5, y=0.0) 存在小障碍物，检验最优轨迹成功产生横向偏置（绕障）
  {
    class CenterObstaclePolicy : public fp::policies::Policy
    {
public:
      CenterObstaclePolicy(double obs_x, double obs_y, double radius)
      : obs_x_(obs_x), obs_y_(obs_y), radius_(radius) {}

      bool checkIfFeasible(
        const fp::FrenetTrajectory &,
        const fp::CartesianTrajectory & cartesian_trajectory) override
      {
        fp::PolicyViolation v;
        return !findViolation(cartesian_trajectory, v);
      }

      bool findViolation(
        const fp::CartesianTrajectory & cartesian_trajectory,
        fp::PolicyViolation & violation) const override
      {
        for (std::size_t i = 0; i < cartesian_trajectory.size(); ++i) {
          double x = cartesian_trajectory[i][0];
          double y = cartesian_trajectory[i][3];
          double dist = std::hypot(x - obs_x_, y - obs_y_);
          if (dist < radius_) {
            violation.valid = true;
            violation.state_index = i;
            violation.x = x;
            violation.y = y;
            return true;
          }
        }
        return false;
      }

private:
      double obs_x_;
      double obs_y_;
      double radius_;
    };

    // 偏好中心线的代价函数：d^2 + (v - 0.4)^2
    class DefaultCost : public fp::costs::Cost
    {
public:
      double cost(
        const fp::FrenetTrajectory & frenet_trajectory,
        const fp::CartesianTrajectory &) override
      {
        double sum = 0.0;
        for (const auto & st : frenet_trajectory) {
          double d = st[3];
          double s_dot = st[1];
          sum += 2.0 * (d * d) + 1.0 * (s_dot - 0.4) * (s_dot - 0.4);
        }
        return sum;
      }
    };

    fp::FrenetTrajectoryPlanner planner(config);
    // 在 x=0.25, y=0.0 设置半径 0.20m 的障碍物（正阻挡直线前方，覆盖 x in [0.05, 0.45]）
    // 强制所有沿中心线 (l=0.0) 行驶的候选均发生碰撞并被剔除
    planner.addPolicy(std::make_shared<CenterObstaclePolicy>(0.25, 0.0, 0.20));
    planner.addCost(std::make_shared<DefaultCost>());

    auto debug_info = std::make_shared<fp::DebugInfo>();
    fp::CartesianState robot = fp::CartesianState::Zero();
    robot[1] = 0.2;

    const auto result = planner.planByWaypoint(robot, waypoints, debug_info);

    // 验证有候选被策略剔除，但依然能选出最优无碰撞绕障轨迹
    EXPECT_GT(debug_info->candidate_count, 0U);
    ASSERT_FALSE(debug_info->policy_reject_counts.empty());
    EXPECT_GT(debug_info->policy_reject_counts[0], 0U);
    EXPECT_LT(debug_info->policy_reject_counts[0], debug_info->candidate_count);
    EXPECT_GT(debug_info->feasible_candidate_count, 0U);
    ASSERT_FALSE(result.cartesian.empty());

    // 验证选出的轨迹发生横向位移，终点横向偏差显著避开中心障碍物
    const double final_lateral_offset = std::abs(result.cartesian.back()[3]);
    EXPECT_GE(final_lateral_offset, 0.20);
  }

  // 场景 C：高速 0.6m/s、短距离 2m 正前阻挡实车工况专项回归测试
  {
    fp::FrenetTrajectoryPlannerConfig config;
    config.min_lateral_distance = -0.9;
    config.max_lateral_distance = 0.9;
    config.step_lateral_distance = 0.15;
    config.min_longitudinal_velocity = 0.20;
    config.max_longitudinal_velocity = 0.70;
    config.step_longitudinal_velocity = 0.25;
    config.min_time_interval = 2.0;
    config.max_time_interval = 4.0;
    config.step_time_interval = 1.0;
    config.min_lateral_time_interval = 1.2;
    config.max_lateral_time_interval = 2.4;
    config.step_lateral_time_interval = 0.6;
    config.max_state_in_trajectory = 100;
    config.dt = 0.05;

    // 总长 3.5m 的直线参考线
    std::vector<fp::CartesianPoint> waypoints;
    for (double x = 0.0; x <= 3.5; x += 0.20) {
      waypoints.push_back({x, 0.0});
    }

    class ObstacleAt2mPolicy : public fp::policies::Policy
    {
public:
      ObstacleAt2mPolicy(double obs_x, double obs_y, double radius)
      : obs_x_(obs_x), obs_y_(obs_y), radius_(radius) {}

      bool checkIfFeasible(
        const fp::FrenetTrajectory &,
        const fp::CartesianTrajectory & cartesian_trajectory) override
      {
        fp::PolicyViolation v;
        return !findViolation(cartesian_trajectory, v);
      }

      bool findViolation(
        const fp::CartesianTrajectory & cartesian_trajectory,
        fp::PolicyViolation & violation) const override
      {
        for (std::size_t i = 0; i < cartesian_trajectory.size(); ++i) {
          const double x = cartesian_trajectory[i][0];
          const double y = cartesian_trajectory[i][3];
          if (std::hypot(x - obs_x_, y - obs_y_) < radius_) {
            violation.valid = true;
            violation.state_index = i;
            violation.x = x;
            violation.y = y;
            return true;
          }
        }
        return false;
      }

private:
      double obs_x_;
      double obs_y_;
      double radius_;
    };

    class SpeedAndLateralCost : public fp::costs::Cost
    {
public:
      double cost(
        const fp::FrenetTrajectory & frenet_trajectory,
        const fp::CartesianTrajectory &) override
      {
        double sum = 0.0;
        for (const auto & st : frenet_trajectory) {
          const double d = st[3];
          const double s_dot = st[1];
          // 偏好横向偏移适中且速度保持在 0.5m/s 附近
          sum += 1.5 * (d * d) + 1.0 * (s_dot - 0.5) * (s_dot - 0.5);
        }
        return sum;
      }
    };

    fp::FrenetTrajectoryPlanner planner(config);
    // 障碍物设置在 x=1.6m 处（距离起点不到 2m），安全包络半径 0.40m，正中挡死
    planner.addPolicy(std::make_shared<ObstacleAt2mPolicy>(1.60, 0.0, 0.40));
    planner.addCost(std::make_shared<SpeedAndLateralCost>());

    auto debug_info = std::make_shared<fp::DebugInfo>();
    fp::CartesianState robot = fp::CartesianState::Zero();
    // 初始运行速度为 0.6 m/s
    robot[1] = 0.60;

    const auto result = planner.planByWaypoint(robot, waypoints, debug_info);

    // 必须有被障碍物拦截的候选，同时必须存在至少一条可行解
    EXPECT_GT(debug_info->candidate_count, 0U);
    ASSERT_FALSE(debug_info->policy_reject_counts.empty());
    EXPECT_GT(debug_info->policy_reject_counts[0], 0U);
    EXPECT_GT(debug_info->feasible_candidate_count, 0U);
    ASSERT_FALSE(result.cartesian.empty());

    // 验证绕障平滑性：初始速度为 0.6m/s，且轨迹最大横向偏移避开障碍物包络
    EXPECT_NEAR(result.cartesian.front()[1], 0.60, 1e-4);
    double max_lateral_abs = 0.0;
    for (const auto & pt : result.cartesian) {
      max_lateral_abs = std::max(max_lateral_abs, std::abs(pt[3]));
    }
    EXPECT_GE(max_lateral_abs, 0.40);
  }
}
