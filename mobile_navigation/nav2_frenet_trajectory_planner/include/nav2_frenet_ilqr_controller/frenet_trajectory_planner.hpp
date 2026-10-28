#ifndef NAV2_FRENET_ILQR_CONTROLLER__FRENET_TRAJECTORY_PLANNER_HPP_
#define NAV2_FRENET_ILQR_CONTROLLER__FRENET_TRAJECTORY_PLANNER_HPP_

#include <Eigen/Dense>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace nav2_frenet_ilqr_controller
{
namespace frenet_planner
{

using StateLateral = Eigen::Vector3d;
using StateLongitudinal = Eigen::Vector3d;
using FrenetState = Eigen::Matrix<double, 6, 1>;
using FrenetTrajectory = std::vector<FrenetState>;
using CartesianState = Eigen::Matrix<double, 7, 1>;
using CartesianTrajectory = std::vector<CartesianState>;
using CartesianPoint = Eigen::Vector2d;

struct FrenetTrajectoryPlannerConfig
{
  double min_lateral_distance{-1.5};
  double max_lateral_distance{1.5};
  double step_lateral_distance{0.25};
  double min_longitudinal_velocity{0.0};
  double max_longitudinal_velocity{0.8};
  double step_longitudinal_velocity{0.10};
  double min_time_interval{1.0};
  double max_time_interval{4.0};
  double step_time_interval{0.25};
  int max_state_in_trajectory{120};
  double dt{0.05};
};

struct Info
{
  double arclength{0.0};
};

struct DebugInfo
{
  std::vector<std::pair<CartesianTrajectory, double>> cartesian_trajectories;
};

class QuarticTrajectoryPlanner
{
public:
  bool setCoefficients(
    double x0, double dx0, double ddx0, double dx1, double ddx1, double t0,
    double t1);
  double x(double t) const;
  double dx(double t) const;
  double ddx(double t) const;

private:
  Eigen::Vector4d coefficients_{Eigen::Vector4d::Zero()};
  double x0_{0.0};
  bool valid_{false};
};

class QuinticTrajectoryPlanner
{
public:
  bool setCoefficients(
    double x0, double dx0, double ddx0, double x1, double dx1, double ddx1,
    double t0, double t1);
  double x(double t) const;
  double dx(double t) const;
  double ddx(double t) const;

private:
  Eigen::Matrix<double, 6, 1> coefficients_{Eigen::Matrix<double, 6, 1>::Zero()};
  bool valid_{false};
};

namespace policies
{
class Policy
{
public:
  virtual ~Policy() = default;
  virtual bool checkIfFeasible(
    const FrenetTrajectory & frenet_trajectory,
    const CartesianTrajectory & cartesian_trajectory) = 0;
};
}  // namespace policies

namespace costs
{
class Cost
{
public:
  virtual ~Cost() = default;
  virtual double cost(
    const FrenetTrajectory & frenet_trajectory,
    const CartesianTrajectory & cartesian_trajectory) = 0;
  void setInfo(const Info & info) {info_ = info;}

protected:
  Info info_;
};
}  // namespace costs

class FrenetFrameConverter
{
public:
  FrenetFrameConverter();
  ~FrenetFrameConverter();
  FrenetFrameConverter(const FrenetFrameConverter &) = delete;
  FrenetFrameConverter & operator=(const FrenetFrameConverter &) = delete;
  FrenetFrameConverter(FrenetFrameConverter &&) noexcept;
  FrenetFrameConverter & operator=(FrenetFrameConverter &&) noexcept;

  void createSegments(const std::vector<CartesianPoint> & waypoints);
  CartesianTrajectory convertFrenet2Cartesian(const FrenetTrajectory & trajectory) const;
  FrenetState convertCartesian2FrenetForSegment(
    const CartesianState & state,
    std::size_t segment_index) const;
  CartesianState convertFrenet2CartesianForSegment(
    const FrenetState & state,
    std::size_t segment_index) const;
  double getArclength() const;
  std::size_t segmentCount() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class FrenetTrajectoryPlanner
{
public:
  struct PlanResult
  {
    CartesianTrajectory cartesian;
    FrenetTrajectory frenet;
    std::shared_ptr<FrenetFrameConverter> frame;
    double arclength{0.0};
  };

  FrenetTrajectoryPlanner() = default;
  explicit FrenetTrajectoryPlanner(const FrenetTrajectoryPlannerConfig & config);

  void setConfig(const FrenetTrajectoryPlannerConfig & config);
  const FrenetTrajectoryPlannerConfig & getConfig() const {return config_;}

  PlanResult planByWaypoint(
    const CartesianState & robot_state,
    const std::vector<CartesianPoint> & waypoints,
    const std::shared_ptr<DebugInfo> & debug_info = nullptr);

  void addPolicy(const std::shared_ptr<policies::Policy> & policy);
  void addCost(const std::shared_ptr<costs::Cost> & cost);
  void clearPolicies();
  void clearCosts();
  std::size_t policyCount() const;
  std::size_t costCount() const;

  bool shouldMergeAndStop(
    const FrenetState & frenet_state, double arclength,
    double time_interval) const;

private:
  void makeYawTrajectoryFeasible(
    const CartesianState & robot_state,
    CartesianTrajectory & trajectory) const;

  FrenetTrajectoryPlannerConfig config_;
  std::vector<std::shared_ptr<policies::Policy>> policies_;
  std::vector<std::shared_ptr<costs::Cost>> costs_;
};

}  // namespace frenet_planner
}  // namespace nav2_frenet_ilqr_controller

#endif  // NAV2_FRENET_ILQR_CONTROLLER__FRENET_TRAJECTORY_PLANNER_HPP_
