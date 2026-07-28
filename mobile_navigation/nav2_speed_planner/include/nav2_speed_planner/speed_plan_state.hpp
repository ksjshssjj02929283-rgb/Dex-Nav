#ifndef NAV2_SPEED_PLANNER__SPEED_PLAN_STATE_HPP_
#define NAV2_SPEED_PLANNER__SPEED_PLAN_STATE_HPP_

#include <mutex>
#include <optional>

#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/speed_planner.hpp"
#include "nav_interfaces/msg/robot_trajectory.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/time.hpp"

namespace nav2_speed_planner
{

enum class BoundaryReason
{
  FIRST_CYCLE,
  EXPIRED,
  FRAME_MISMATCH,
  GOAL_MISMATCH,
  BRANCH_MISMATCH,
  SPATIAL_MISMATCH,
  VELOCITY_MISMATCH,
  INVALID_NUMERIC,
  EXPLICIT_INVALIDATION
};

const char * boundaryReasonName(BoundaryReason reason);

struct BoundaryDecision
{
  nav2_core::SpeedPlanningBoundary boundary;
  std::optional<BoundaryReason> reason;

  bool inherited() const
  {
    return boundary.acceleration_valid;
  }
};

class SpeedPlanState
{
public:
  struct Config
  {
    double max_age{0.5};
    double max_spatial_error{0.15};
    double max_velocity_error{0.25};
    double goal_position_tolerance{0.05};
    double goal_yaw_tolerance{0.2};
    double branch_heading_tolerance{0.7853981634};
  };

  SpeedPlanState();
  explicit SpeedPlanState(const Config & config);

  void configure(const Config & config);

  BoundaryDecision boundary(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Twist & measured_velocity,
    const rclcpp::Time & now) const;

  void accept(
    const nav_interfaces::msg::RobotTrajectory & trajectory,
    double consumed_target_velocity,
    double consumed_target_acceleration,
    const rclcpp::Time & now);

  void reset(BoundaryReason reason = BoundaryReason::EXPLICIT_INVALIDATION);

  bool valid() const;

private:
  void resetUnlocked(BoundaryReason reason);

  mutable std::mutex mutex_;
  Config config_;
  nav_interfaces::msg::RobotTrajectory accepted_trajectory_;
  double accepted_target_velocity_{0.0};
  double accepted_target_acceleration_{0.0};
  rclcpp::Time accepted_trajectory_time_{0, 0, RCL_ROS_TIME};
  bool valid_{false};
  BoundaryReason invalidation_reason_{BoundaryReason::FIRST_CYCLE};
};

}  // namespace nav2_speed_planner

#endif  // NAV2_SPEED_PLANNER__SPEED_PLAN_STATE_HPP_
