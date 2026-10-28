#include "nav2_frenet_ilqr_controller/frenet_trajectory_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace nav2_frenet_ilqr_controller
{
namespace frenet_planner
{
namespace
{

constexpr double kEpsilon = 1e-9;
constexpr double kStraightAngleThreshold = 0.1;
constexpr double kCurvatureDelta = 1.0;
constexpr std::size_t kMaxPlanningIterations = 32;

bool finiteState(const FrenetState & state) {return state.allFinite();}

std::vector<double> sampleRange(double minimum, double maximum, double step)
{
  std::vector<double> values;
  if (!std::isfinite(minimum) || !std::isfinite(maximum) || !std::isfinite(step) || step <= 0.0) {
    return values;
  }
  if (maximum + kEpsilon < minimum) {
    return values;
  }
  const double span = maximum - minimum;
  const std::size_t count = static_cast<std::size_t>(std::floor(span / step + kEpsilon));
  values.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    values.push_back(minimum + static_cast<double>(index) * step);
  }
  return values;
}

template<typename LongitudinalPlanner>
FrenetTrajectory sampleTrajectory(
  const LongitudinalPlanner & longitudinal,
  const QuinticTrajectoryPlanner & lateral,
  const FrenetTrajectoryPlannerConfig & config,
  std::size_t max_state_number,
  double time_interval)
{
  FrenetTrajectory trajectory;
  if (max_state_number < 2U || !std::isfinite(time_interval) || time_interval <= 0.0 ||
    !std::isfinite(config.dt) ||
    config.dt <= 0.0)
  {
    return trajectory;
  }

  const double maximum_time = std::min(
    config.dt * static_cast<double>(max_state_number),
    time_interval);
  if (maximum_time < 0.0) {
    return trajectory;
  }

  trajectory.reserve(max_state_number + 1U);
  const std::size_t sample_count =
    static_cast<std::size_t>(std::floor(maximum_time / config.dt + kEpsilon)) + 1U;
  for (std::size_t index = 0; index < sample_count; ++index) {
    const double t = std::min(static_cast<double>(index) * config.dt, maximum_time);
    FrenetState state;
    state << longitudinal.x(t), longitudinal.dx(t), longitudinal.ddx(t), lateral.x(t),
      lateral.dx(t),
      lateral.ddx(t);
    if (!finiteState(state)) {
      return {};
    }
    trajectory.push_back(state);
  }
  return trajectory;
}

FrenetTrajectory makeKeepVelocityTrajectory(
  const FrenetState & initial,
  const FrenetTrajectoryPlannerConfig & config,
  std::size_t max_state_number,
  double final_velocity,
  double final_lateral,
  double time_interval)
{
  QuarticTrajectoryPlanner longitudinal;
  if (!longitudinal.setCoefficients(
      initial[0], initial[1], initial[2], final_velocity, 0.0, 0.0,
      time_interval))
  {
    return {};
  }

  QuinticTrajectoryPlanner lateral;
  if (!lateral.setCoefficients(
      initial[3], initial[4], initial[5], final_lateral, 0.0, 0.0, 0.0,
      time_interval))
  {
    return {};
  }

  return sampleTrajectory(longitudinal, lateral, config, max_state_number, time_interval);
}

FrenetTrajectory makeMergeAndStopTrajectory(
  const FrenetState & initial,
  const FrenetTrajectoryPlannerConfig & config,
  std::size_t max_state_number,
  double arclength,
  double final_lateral,
  double time_interval)
{
  QuinticTrajectoryPlanner longitudinal;
  if (!longitudinal.setCoefficients(
      initial[0], initial[1], initial[2], arclength, 0.0, 0.0, 0.0,
      time_interval))
  {
    return {};
  }

  QuinticTrajectoryPlanner lateral;
  if (!lateral.setCoefficients(
      initial[3], initial[4], initial[5], final_lateral, 0.0, 0.0, 0.0,
      time_interval))
  {
    return {};
  }

  return sampleTrajectory(longitudinal, lateral, config, max_state_number, time_interval);
}

std::vector<FrenetTrajectory> generateKeepVelocityTrajectories(
  const FrenetState & initial,
  const FrenetTrajectoryPlannerConfig & config,
  std::size_t max_state_number)
{
  const auto times = sampleRange(
    config.min_time_interval, config.max_time_interval,
    config.step_time_interval);
  const auto velocities = sampleRange(
    config.min_longitudinal_velocity, config.max_longitudinal_velocity,
    config.step_longitudinal_velocity);
  const auto laterals =
    sampleRange(
    config.min_lateral_distance, config.max_lateral_distance,
    config.step_lateral_distance);

  std::vector<FrenetTrajectory> trajectories;
  trajectories.reserve(times.size() * velocities.size() * laterals.size());
  for (const double time_interval : times) {
    for (const double velocity : velocities) {
      for (const double lateral : laterals) {
        trajectories.push_back(
          makeKeepVelocityTrajectory(
            initial, config, max_state_number, velocity, lateral,
            time_interval));
      }
    }
  }
  return trajectories;
}

std::vector<FrenetTrajectory> generateMergeAndStopTrajectories(
  const FrenetState & initial,
  const FrenetTrajectoryPlannerConfig & config,
  std::size_t max_state_number,
  double arclength)
{
  const auto times = sampleRange(
    config.min_time_interval, config.max_time_interval,
    config.step_time_interval);
  const auto laterals =
    sampleRange(
    config.min_lateral_distance, config.max_lateral_distance,
    config.step_lateral_distance);

  std::vector<FrenetTrajectory> trajectories;
  trajectories.reserve(times.size() * laterals.size());
  for (const double time_interval : times) {
    for (const double lateral : laterals) {
      trajectories.push_back(
        makeMergeAndStopTrajectory(
          initial, config, max_state_number, arclength, lateral,
          time_interval));
    }
  }
  return trajectories;
}

std::optional<FrenetTrajectory> selectBestTrajectory(
  const std::vector<FrenetTrajectory> & candidates,
  const std::shared_ptr<FrenetFrameConverter> & frame,
  const Info & info,
  const std::vector<std::shared_ptr<policies::Policy>> & policies,
  const std::vector<std::shared_ptr<costs::Cost>> & costs,
  const std::shared_ptr<DebugInfo> & debug_info)
{
  std::optional<FrenetTrajectory> best;
  double best_cost = std::numeric_limits<double>::infinity();

  for (const auto & candidate : candidates) {
    if (candidate.size() < 2U) {
      continue;
    }
    const CartesianTrajectory cartesian = frame->convertFrenet2Cartesian(candidate);
    if (cartesian.size() < 2U) {
      continue;
    }

    bool feasible = true;
    for (const auto & policy : policies) {
      if (policy && !policy->checkIfFeasible(candidate, cartesian)) {
        feasible = false;
        break;
      }
    }
    if (!feasible) {
      if (debug_info) {
        debug_info->cartesian_trajectories.emplace_back(cartesian, -1.0);
      }
      continue;
    }

    double candidate_cost = 0.0;
    for (const auto & cost : costs) {
      if (!cost) {
        continue;
      }
      cost->setInfo(info);
      candidate_cost += cost->cost(candidate, cartesian);
    }
    if (!std::isfinite(candidate_cost)) {
      continue;
    }

    if (debug_info) {
      debug_info->cartesian_trajectories.emplace_back(cartesian, candidate_cost);
    }
    if (candidate_cost < best_cost) {
      best_cost = candidate_cost;
      best = candidate;
    }
  }

  return best;
}

double clampDot(double value) {return std::clamp(value, -1.0, 1.0);}

}  // namespace

bool QuarticTrajectoryPlanner::setCoefficients(
  double x0, double dx0, double ddx0, double dx1, double ddx1, double t0, double t1)
{
  valid_ = false;
  const double duration = t1 - t0;
  if (!std::isfinite(duration) || duration <= kEpsilon || !std::isfinite(x0) ||
    !std::isfinite(dx0) ||
    !std::isfinite(ddx0) || !std::isfinite(dx1) || !std::isfinite(ddx1))
  {
    return false;
  }

  Eigen::Matrix4d matrix;
  matrix << 1.0, 2.0 * t0, 3.0 * std::pow(t0, 2), 4.0 * std::pow(t0, 3), 0.0, 2.0, 6.0 * t0,
    12.0 * std::pow(t0, 2),
    1.0, 2.0 * t1, 3.0 * std::pow(t1, 2), 4.0 * std::pow(t1, 3), 0.0, 2.0, 6.0 * t1,
    12.0 * std::pow(t1, 2);
  Eigen::Vector4d rhs;
  rhs << dx0, ddx0, dx1, ddx1;
  const Eigen::Vector4d coefficients = matrix.colPivHouseholderQr().solve(rhs);
  if (!coefficients.allFinite()) {
    return false;
  }

  coefficients_ = coefficients;
  x0_ = x0;
  valid_ = true;
  return true;
}

double QuarticTrajectoryPlanner::x(double t) const
{
  if (!valid_) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return x0_ + coefficients_[0] * t + coefficients_[1] * t * t + coefficients_[2] * t * t * t +
         coefficients_[3] * t * t * t * t;
}

double QuarticTrajectoryPlanner::dx(double t) const
{
  if (!valid_) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return coefficients_[0] + 2.0 * coefficients_[1] * t + 3.0 * coefficients_[2] * t * t +
         4.0 * coefficients_[3] * t * t * t;
}

double QuarticTrajectoryPlanner::ddx(double t) const
{
  if (!valid_) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return 2.0 * coefficients_[1] + 6.0 * coefficients_[2] * t + 12.0 * coefficients_[3] * t * t;
}

bool QuinticTrajectoryPlanner::setCoefficients(
  double x0, double dx0, double ddx0, double x1, double dx1, double ddx1, double t0, double t1)
{
  valid_ = false;
  const double duration = t1 - t0;
  if (!std::isfinite(duration) || duration <= kEpsilon || !std::isfinite(x0) ||
    !std::isfinite(dx0) ||
    !std::isfinite(ddx0) || !std::isfinite(x1) || !std::isfinite(dx1) || !std::isfinite(ddx1))
  {
    return false;
  }

  Eigen::Matrix<double, 6, 6> matrix;
  matrix << 1.0, t0, std::pow(t0, 2), std::pow(t0, 3), std::pow(t0, 4), std::pow(t0, 5), 0.0, 1.0,
    2.0 * t0,
    3.0 * std::pow(t0, 2), 4.0 * std::pow(t0, 3), 5.0 * std::pow(t0, 4), 0.0, 0.0, 2.0, 6.0 * t0,
    12.0 * std::pow(t0, 2), 20.0 * std::pow(t0, 3), 1.0, t1, std::pow(t1, 2), std::pow(t1, 3),
    std::pow(t1, 4),
    std::pow(t1, 5), 0.0, 1.0, 2.0 * t1, 3.0 * std::pow(t1, 2), 4.0 * std::pow(t1, 3),
    5.0 * std::pow(t1, 4), 0.0,
    0.0, 2.0, 6.0 * t1, 12.0 * std::pow(t1, 2), 20.0 * std::pow(t1, 3);
  Eigen::Matrix<double, 6, 1> rhs;
  rhs << x0, dx0, ddx0, x1, dx1, ddx1;
  const Eigen::Matrix<double, 6, 1> coefficients = matrix.colPivHouseholderQr().solve(rhs);
  if (!coefficients.allFinite()) {
    return false;
  }

  coefficients_ = coefficients;
  valid_ = true;
  return true;
}

double QuinticTrajectoryPlanner::x(double t) const
{
  if (!valid_) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return coefficients_[0] + coefficients_[1] * t + coefficients_[2] * t * t + coefficients_[3] * t *
         t * t +
         coefficients_[4] * std::pow(t, 4) + coefficients_[5] * std::pow(t, 5);
}

double QuinticTrajectoryPlanner::dx(double t) const
{
  if (!valid_) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return coefficients_[1] + 2.0 * coefficients_[2] * t + 3.0 * coefficients_[3] * t * t +
         4.0 * coefficients_[4] * std::pow(t, 3) + 5.0 * coefficients_[5] * std::pow(t, 4);
}

double QuinticTrajectoryPlanner::ddx(double t) const
{
  if (!valid_) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return 2.0 * coefficients_[2] + 6.0 * coefficients_[3] * t + 12.0 * coefficients_[4] * t * t +
         20.0 * coefficients_[5] * std::pow(t, 3);
}

struct FrenetFrameConverter::Impl
{
  struct Segment
  {
    bool is_arc{false};
    double length{0.0};
    Eigen::Vector2d line_start{Eigen::Vector2d::Zero()};
    Eigen::Vector2d line_tangent{Eigen::Vector2d::UnitX()};
    Eigen::Vector2d center{Eigen::Vector2d::Zero()};
    Eigen::Vector2d circle_x{Eigen::Vector2d::UnitX()};
    Eigen::Vector2d circle_y{Eigen::Vector2d::UnitY()};
    double radius{0.0};
    double angle{0.0};
    int sign{1};

    CartesianState toCartesian(const FrenetState & state) const
    {
      CartesianState cartesian = CartesianState::Zero();
      if (!is_arc) {
        Eigen::Matrix2d rotation;
        rotation << line_tangent.x(), -line_tangent.y(), line_tangent.y(), line_tangent.x();
        cartesian({0, 3}) = rotation * state({0, 3}) + line_start;
        cartesian({1, 4}) = rotation * state({1, 4});
        cartesian({2, 5}) = rotation * state({2, 5});
        if (std::hypot(cartesian[1], cartesian[4]) > kEpsilon) {
          cartesian[6] = std::atan2(cartesian[4], cartesian[1]);
        } else {
          cartesian[6] = std::atan2(line_tangent.y(), line_tangent.x());
        }
        return cartesian;
      }

      const double cos_s_over_r = std::cos(state[0] / radius);
      const double sin_s_over_r = std::sin(state[0] / radius);
      const double r_plus_signed_d = radius + static_cast<double>(sign) * state[3];
      const Eigen::Vector2d x_cos_plus_y_sin = circle_x * cos_s_over_r + circle_y * sin_s_over_r;
      const Eigen::Vector2d y_cos_minus_x_sin = circle_y * cos_s_over_r - circle_x * sin_s_over_r;

      cartesian({0, 3}) = center + r_plus_signed_d * x_cos_plus_y_sin;
      cartesian({1, 4}) = (state[1] * r_plus_signed_d / radius) * y_cos_minus_x_sin +
        (static_cast<double>(sign) * state[4]) * x_cos_plus_y_sin;
      cartesian({2, 5}) =
        ((state[2] * r_plus_signed_d + static_cast<double>(sign) * 2.0 * state[1] * state[4]) /
        radius) *
        y_cos_minus_x_sin +
        (-std::pow(state[1], 2) * r_plus_signed_d / std::pow(radius, 2) +
        static_cast<double>(sign) * state[5]) *
        x_cos_plus_y_sin;
      if (std::hypot(cartesian[1], cartesian[4]) > kEpsilon) {
        cartesian[6] = std::atan2(cartesian[4], cartesian[1]);
      } else {
        cartesian[6] = std::atan2(y_cos_minus_x_sin.y(), y_cos_minus_x_sin.x());
      }
      return cartesian;
    }

    FrenetState toFrenet(const CartesianState & cartesian) const
    {
      FrenetState frenet = FrenetState::Zero();
      if (!is_arc) {
        Eigen::Matrix<double, 6, 2> transform;
        transform << cartesian[0] - line_start.x(), cartesian[3] - line_start.y(), cartesian[1],
          cartesian[4],
          cartesian[2], cartesian[5], cartesian[3] - line_start.y(),
          -(cartesian[0] - line_start.x()),
          cartesian[4], -cartesian[1], cartesian[5], -cartesian[2];
        return transform * line_tangent;
      }

      const Eigen::Vector2d xc_minus_c(cartesian[0] - center.x(), cartesian[3] - center.y());
      const double distance = xc_minus_c.norm();
      if (distance < kEpsilon) {
        return frenet;
      }
      const Eigen::Vector2d direction = xc_minus_c / distance;
      frenet[0] = radius * std::atan2(direction.dot(circle_y), direction.dot(circle_x));

      const double cos_s_over_r = std::cos(frenet[0] / radius);
      const double sin_s_over_r = std::sin(frenet[0] / radius);
      const Eigen::Vector2d y_cos_minus_x_sin = circle_y * cos_s_over_r - circle_x * sin_s_over_r;

      const Eigen::Vector2d velocity(cartesian[1], cartesian[4]);
      const Eigen::Vector2d acceleration(cartesian[2], cartesian[5]);
      const Eigen::Vector2d signed_normal =
        (static_cast<double>(sign) / distance) * xc_minus_c;
      const double tangent_velocity = y_cos_minus_x_sin.dot(velocity);
      const double tangent_acceleration = y_cos_minus_x_sin.dot(acceleration);
      frenet[1] = (radius / distance) * tangent_velocity;
      frenet[3] = static_cast<double>(sign) * (distance - radius);
      frenet[4] = signed_normal.dot(velocity);
      frenet[2] =
        (tangent_acceleration -
        2.0 * static_cast<double>(sign) * frenet[4] * frenet[1] / radius) *
        radius / distance;
      frenet[5] = signed_normal.dot(acceleration) +
        static_cast<double>(sign) * distance * frenet[1] * frenet[1] /
        (radius * radius);
      return frenet;
    }
  };

  std::vector<Segment> segments;
  double total_length{0.0};
};

FrenetFrameConverter::FrenetFrameConverter()
: impl_(std::make_unique<Impl>()) {}

FrenetFrameConverter::~FrenetFrameConverter() = default;
FrenetFrameConverter::FrenetFrameConverter(FrenetFrameConverter &&) noexcept = default;
FrenetFrameConverter & FrenetFrameConverter::operator=(FrenetFrameConverter &&) noexcept = default;

void FrenetFrameConverter::createSegments(const std::vector<CartesianPoint> & waypoints)
{
  impl_ = std::make_unique<Impl>();
  std::vector<CartesianPoint> points;
  points.reserve(waypoints.size());
  for (const auto & waypoint : waypoints) {
    if (!waypoint.allFinite()) {
      continue;
    }
    if (points.empty() || (waypoint - points.back()).norm() > 1e-5) {
      points.push_back(waypoint);
    }
  }
  if (points.size() < 2U) {
    return;
  }

  auto appendLine = [this](const Eigen::Vector2d & start, const Eigen::Vector2d & end) {
      const Eigen::Vector2d delta = end - start;
      const double length = delta.norm();
      if (length <= 1e-6) {
        return;
      }
      Impl::Segment segment;
      segment.is_arc = false;
      segment.length = length;
      segment.line_start = start;
      segment.line_tangent = delta / length;
      impl_->segments.push_back(segment);
      impl_->total_length += length;
    };

  CartesianPoint last_waypoint = points.front();
  for (std::size_t index = 1; index + 1U < points.size(); ++index) {
    const Eigen::Vector2d q_previous = last_waypoint;
    const Eigen::Vector2d q_current = points[index];
    const Eigen::Vector2d q_next = points[index + 1U];
    const Eigen::Vector2d previous_delta = q_current - q_previous;
    const Eigen::Vector2d next_delta = q_next - q_current;
    const double previous_length = previous_delta.norm();
    const double next_length = next_delta.norm();
    if (previous_length <= 1e-6 || next_length <= 1e-6) {
      last_waypoint = q_current;
      continue;
    }

    const Eigen::Vector2d previous_tangent = previous_delta / previous_length;
    const Eigen::Vector2d next_tangent = next_delta / next_length;
    const double angle = std::acos(clampDot(previous_tangent.dot(next_tangent)));
    if (!std::isfinite(angle) || angle < kStraightAngleThreshold ||
      angle > M_PI - kStraightAngleThreshold)
    {
      appendLine(last_waypoint, q_current);
      last_waypoint = q_current;
      continue;
    }

    const double half_angle = angle / 2.0;
    const double cosine_half = std::cos(half_angle);
    const double sine_half = std::sin(half_angle);
    if (std::abs(cosine_half) < 1e-6 || std::abs(1.0 - cosine_half) < 1e-6) {
      appendLine(last_waypoint, q_current);
      last_waypoint = q_current;
      continue;
    }

    const double tangent_cut =
      std::min(
      {previous_length / 2.0, next_length / 2.0,
        kCurvatureDelta * sine_half / (1.0 - cosine_half)});
    const double radius = tangent_cut / std::tan(half_angle);
    const Eigen::Vector2d bisector = next_tangent - previous_tangent;
    if (!std::isfinite(radius) || radius <= 1e-6 || bisector.norm() < 1e-6) {
      appendLine(last_waypoint, q_current);
      last_waypoint = q_current;
      continue;
    }

    const Eigen::Vector2d center = q_current + (bisector / bisector.norm()) *
      (radius / cosine_half);
    const Eigen::Vector2d first_arc_point = q_current - tangent_cut * previous_tangent;
    const Eigen::Vector2d center_to_first = first_arc_point - center;
    if (center_to_first.norm() < 1e-6) {
      appendLine(last_waypoint, q_current);
      last_waypoint = q_current;
      continue;
    }
    const Eigen::Vector2d circle_x = center_to_first / center_to_first.norm();
    // Match the upstream frame construction: the circle basis second axis is the
    // incoming tangent (not a fixed CCW rotation of the radial axis).  This keeps
    // left/right arcs and the signed lateral coordinate consistent.
    const Eigen::Vector2d circle_y = previous_tangent;
    const Eigen::Vector2d last_arc_point =
      center + radius * (circle_x * std::cos(angle) + circle_y * std::sin(angle));
    const double cross = next_tangent.x() * previous_tangent.y() - next_tangent.y() *
      previous_tangent.x();
    const int sign = cross > 0.0 ? 1 : -1;

    appendLine(last_waypoint, first_arc_point);
    Impl::Segment arc;
    arc.is_arc = true;
    arc.length = angle * radius;
    arc.center = center;
    arc.circle_x = circle_x;
    arc.circle_y = circle_y;
    arc.radius = radius;
    arc.angle = angle;
    arc.sign = sign;
    if (arc.length > 1e-6) {
      impl_->segments.push_back(arc);
      impl_->total_length += arc.length;
    }
    last_waypoint = last_arc_point;
  }
  appendLine(last_waypoint, points.back());
}

CartesianTrajectory FrenetFrameConverter::convertFrenet2Cartesian(
  const FrenetTrajectory & trajectory) const
{
  CartesianTrajectory cartesian;
  if (impl_->segments.empty()) {
    return cartesian;
  }
  cartesian.reserve(trajectory.size());
  std::size_t segment_index = 0U;
  double segment_start = 0.0;
  for (const auto & state : trajectory) {
    if (!std::isfinite(state[0]) || state[0] < -1e-6 || state[0] > impl_->total_length + 1e-6) {
      break;
    }
    const double s = std::clamp(state[0], 0.0, impl_->total_length);
    while (segment_index + 1U < impl_->segments.size() &&
      s > segment_start + impl_->segments[segment_index].length + 1e-6)
    {
      segment_start += impl_->segments[segment_index].length;
      ++segment_index;
    }
    FrenetState local_state = state;
    local_state[0] = std::max(
      0.0, std::min(
        s - segment_start,
        impl_->segments[segment_index].length));
    const CartesianState converted = impl_->segments[segment_index].toCartesian(local_state);
    if (!converted.allFinite()) {
      return {};
    }
    cartesian.push_back(converted);
  }
  return cartesian;
}

FrenetState FrenetFrameConverter::convertCartesian2FrenetForSegment(
  const CartesianState & state,
  std::size_t segment_index) const
{
  if (segment_index >= impl_->segments.size()) {
    return FrenetState::Zero();
  }
  return impl_->segments[segment_index].toFrenet(state);
}

CartesianState FrenetFrameConverter::convertFrenet2CartesianForSegment(
  const FrenetState & state,
  std::size_t segment_index) const
{
  if (segment_index >= impl_->segments.size()) {
    return CartesianState::Zero();
  }
  return impl_->segments[segment_index].toCartesian(state);
}

double FrenetFrameConverter::getArclength() const {return impl_->total_length;}

std::size_t FrenetFrameConverter::segmentCount() const {return impl_->segments.size();}

FrenetTrajectoryPlanner::FrenetTrajectoryPlanner(const FrenetTrajectoryPlannerConfig & config)
: config_(config) {}

void FrenetTrajectoryPlanner::setConfig(const FrenetTrajectoryPlannerConfig & config)
{
  config_ = config;
}

void FrenetTrajectoryPlanner::addPolicy(const std::shared_ptr<policies::Policy> & policy)
{
  if (policy) {
    policies_.push_back(policy);
  }
}

void FrenetTrajectoryPlanner::addCost(const std::shared_ptr<costs::Cost> & cost)
{
  if (cost) {
    costs_.push_back(cost);
  }
}

void FrenetTrajectoryPlanner::clearPolicies() {policies_.clear();}

void FrenetTrajectoryPlanner::clearCosts() {costs_.clear();}

std::size_t FrenetTrajectoryPlanner::policyCount() const {return policies_.size();}

std::size_t FrenetTrajectoryPlanner::costCount() const {return costs_.size();}

bool FrenetTrajectoryPlanner::shouldMergeAndStop(
  const FrenetState & frenet_state,
  double arclength,
  double time_interval) const
{
  QuarticTrajectoryPlanner longitudinal;
  if (!longitudinal.setCoefficients(
      frenet_state[0], frenet_state[1], frenet_state[2], 0.0, 0.0, 0.0,
      time_interval))
  {
    return false;
  }
  return longitudinal.x(time_interval) >= arclength;
}

void FrenetTrajectoryPlanner::makeYawTrajectoryFeasible(
  const CartesianState & robot_state,
  CartesianTrajectory & trajectory) const
{
  if (trajectory.empty()) {
    return;
  }

  auto wrapAngle = [](double angle) {
      while (angle > M_PI) {
        angle -= 2.0 * M_PI;
      }
      while (angle < -M_PI) {
        angle += 2.0 * M_PI;
      }
      return angle;
    };

  trajectory[0][6] = robot_state[6] + wrapAngle(trajectory[0][6] - robot_state[6]);
  for (std::size_t index = 1; index < trajectory.size(); ++index) {
    const double difference = wrapAngle(trajectory[index][6] - trajectory[index - 1U][6]);
    trajectory[index][6] = trajectory[index - 1U][6] + difference;
  }
}

FrenetTrajectoryPlanner::PlanResult FrenetTrajectoryPlanner::planByWaypoint(
  const CartesianState & robot_state,
  const std::vector<CartesianPoint> & waypoints,
  const std::shared_ptr<DebugInfo> & debug_info)
{
  PlanResult result;
  if (waypoints.size() < 2U || !robot_state.allFinite()) {
    return result;
  }

  auto frame = std::make_shared<FrenetFrameConverter>();
  frame->createSegments(waypoints);
  if (frame->segmentCount() == 0U || frame->getArclength() <= 1e-6) {
    return result;
  }

  result.frame = frame;
  result.arclength = frame->getArclength();
  Info info;
  info.arclength = result.arclength;

  FrenetState robot_frenet = frame->convertCartesian2FrenetForSegment(robot_state, 0U);
  if (!robot_frenet.allFinite()) {
    result.frame.reset();
    return result;
  }

  FrenetTrajectory planned;
  planned.push_back(robot_frenet);
  const std::size_t maximum_states =
    static_cast<std::size_t>(std::max(2, config_.max_state_in_trajectory));
  bool merge_and_stop = false;

  for (std::size_t iteration = 0; iteration < kMaxPlanningIterations; ++iteration) {
    const std::size_t remaining = maximum_states >
      planned.size() ? maximum_states - planned.size() : 0U;
    if (remaining < 2U) {
      break;
    }

    std::vector<FrenetTrajectory> candidates;
    if (merge_and_stop) {
      candidates = generateMergeAndStopTrajectories(
        planned.back(), config_, remaining, result.arclength);
    } else {
      candidates = generateKeepVelocityTrajectories(planned.back(), config_, remaining);
    }

    const auto selected = selectBestTrajectory(
      candidates, frame, info, policies_, costs_,
      debug_info);
    if (!selected.has_value() || selected->size() < 2U) {
      break;
    }

    const double previous_s = planned.back()[0];
    for (std::size_t index = 1U; index < selected->size(); ++index) {
      planned.push_back((*selected)[index]);
      if (shouldMergeAndStop(planned.back(), result.arclength, config_.max_time_interval)) {
        merge_and_stop = true;
        break;
      }
    }

    if (planned.back()[0] <= previous_s + 1e-5) {
      break;
    }
    if (shouldMergeAndStop(planned.back(), result.arclength, config_.max_time_interval)) {
      merge_and_stop = true;
    }
  }

  if (planned.size() < 2U) {
    result.frame.reset();
    return result;
  }
  result.frenet = planned;
  result.cartesian = frame->convertFrenet2Cartesian(planned);
  if (result.cartesian.empty()) {
    result.frame.reset();
    return result;
  }
  if (result.cartesian.size() < result.frenet.size()) {
    result.frenet.resize(result.cartesian.size());
  }
  result.cartesian[0] = robot_state;
  makeYawTrajectoryFeasible(robot_state, result.cartesian);
  return result;
}

}  // namespace frenet_planner
}  // namespace nav2_frenet_ilqr_controller
