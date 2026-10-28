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
