// B 方案：QP Swerve Allocator
//
// 与 A 方案的主要区别：
// 1. 删除 DLS 可达速度求解；
// 2. 删除 QP 后的全局/单轮 drive_scale；
// 3. 枚举四轮 angle / angle+pi 候选组合，并为每种组合求解凸 QP；
// 4. QP 统一处理速度跟踪、连续性、侧向拖拽惩罚、轮速与车体速度约束。
//
// 不依赖 OSQP、qpOASES 等外部库；三变量 QP 使用主动集穷举求解。

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <vector>

#include "chassis_interfaces/msg/chassis_motor_control.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr std::size_t kWheelCount = 4;
constexpr std::size_t kTwistDimension = 3;

using Vector3 = std::array<float, kTwistDimension>;
using Matrix3 = std::array<std::array<float, kTwistDimension>, kTwistDimension>;

// =============================================================================
// 机械参数
// =============================================================================
// NEED CALIBRATION: 按轮轴中心实测，不要使用车体外壳尺寸。
constexpr float kChassisLengthM = 0.460f;
constexpr float kChassisWidthM = 0.345f;
constexpr float kWheelDiameterM = 0.107f;

// NEED CALIBRATION: 软限位应小于真实机械硬限位，并预留安装与零位误差。
constexpr float kSteerAngleLimitRad = 2.50f;
constexpr float kSteerLimitMarginRad = 0.08f;

constexpr float kControlPeriodS = 0.05f;  // 20 Hz
constexpr float kCommandTimeoutS = 0.50f;

// =============================================================================
// 上游速度变化率限制
// =============================================================================
// QP 负责可实现性与轮胎侧向拖拽权衡；这里仅保留一层输入加减速度限制。
// NEED CALIBRATION: 满载测试，以不打滑、不过度冲击且响应可接受为准。
constexpr bool kCommandGovernorEnabled = true;
constexpr float kMaxAccelX = 0.60f;    // m/s^2
constexpr float kMaxAccelY = 0.60f;    // m/s^2
constexpr float kMaxAccelYaw = 0.75f;  // rad/s^2
constexpr float kMaxBrakeX = 0.70f;    // m/s^2
constexpr float kMaxBrakeY = 0.55f;    // m/s^2
constexpr float kMaxBrakeYaw = 1.20f;  // rad/s^2

// =============================================================================
// QP 硬约束
// =============================================================================
// NEED CALIBRATION: 按底盘、电机和减速器真实能力填写。
constexpr float kMaxBodySpeedX = 1.20f;       // m/s
constexpr float kMaxBodySpeedY = 1.20f;       // m/s
constexpr float kMaxBodyYawRate = 0.75f;      // rad/s
constexpr float kMaxWheelLinearSpeed = 1.50f; // m/s

// 舵角误差越大，当前轮向可安全输出的纵向轮速上限越低。
// 该限制直接进入 QP，不再在 QP 后额外乘 global/local drive_scale。
// NEED CALIBRATION:
// - full：误差小于该值时不限制轮速；
// - crawl：误差达到该值时仅保留最小爬行能力；
// - min scale：越小越防摩地，越大越灵敏。
constexpr float kAlignFullSpeedErrorRad = 0.12f;  // 约 6.9 deg
constexpr float kAlignCrawlErrorRad = 0.90f;      // 约 51.6 deg
constexpr float kAlignMinimumSpeedScale = 0.08f;

// =============================================================================
// QP 目标函数权重
// =============================================================================
// 优化变量 x = [vx, vy, yaw_rate]。
// 目标函数同时考虑：命令跟踪、上一周期连续性、轮胎侧向速度、轮速跟踪和正则化。
//
// NEED CALIBRATION:
// 1) 底盘迟钝：优先增大 tracking，或减小 smooth / lateral-slip；
// 2) 换向摩地：优先增大 lateral-slip，或减小 align minimum scale；
// 3) 速度跳变：增大 smooth，但过大会拖慢响应；
// 4) yaw 跟踪弱：单独增大 kQpTrackYaw。
constexpr float kQpTrackX = 18.0f;
constexpr float kQpTrackY = 18.0f;
constexpr float kQpTrackYaw = 12.0f;

constexpr float kQpSmoothX = 0.35f;
constexpr float kQpSmoothY = 0.35f;
constexpr float kQpSmoothYaw = 0.25f;

constexpr float kQpLateralSlipWeight = 8.0f;
constexpr float kQpWheelSpeedTrackWeight = 0.60f;
constexpr float kQpRegularization = 0.01f;

// 离散候选模式代价。QP 会枚举每个轮子的 angle / angle+pi 组合。
// NEED CALIBRATION: 过大时不愿切换到新候选，过小时可能正反解来回跳。
constexpr float kQpCandidateCostWeight = 0.18f;
constexpr float kCandidateSwitchCost = 0.35f;
constexpr float kCandidateReverseCost = 0.20f;
constexpr float kLimitSoftZoneRad = 0.30f;
constexpr float kLimitCostGain = 0.25f;

// =============================================================================
// 低速稳定与求解器参数
// =============================================================================
// NEED CALIBRATION: 低于该速度时目标方向不可靠，保持当前舵角。
constexpr float kWheelVectorDeadbandMps = 0.03f;
constexpr float kLinearCommandDeadbandMps = 0.002f;
constexpr float kYawCommandDeadbandRadps = 0.003f;

constexpr float kQpFeasibilityTolerance = 1e-5f;
constexpr float kQpMultiplierTolerance = 1e-5f;
constexpr float kLinearSolveTolerance = 1e-8f;

float clampf(float value, float lower, float upper) {
  return std::max(lower, std::min(upper, value));
}

float wrapPi(float angle) {
  while (angle > kPi) {
    angle -= 2.0f * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0f * kPi;
  }
  return angle;
}

float angleDiff(float target, float current) {
  return wrapPi(target - current);
}

float speedToRpm(float speed_mps) {
  return speed_mps * 60.0f / (kPi * kWheelDiameterM);
}

float linearScaleFromError(float error, float full_speed_error,
                           float crawl_error, float minimum_scale) {
  if (error <= full_speed_error) {
    return 1.0f;
  }
  if (error >= crawl_error) {
    return minimum_scale;
  }

  const float ratio = (crawl_error - error) /
                      (crawl_error - full_speed_error);
  return minimum_scale + (1.0f - minimum_scale) *
                         clampf(ratio, 0.0f, 1.0f);
}

float limitVelocityRate(float target, float current, float max_accel,
                        float max_brake, float dt) {
  const bool crossing_zero = target * current < 0.0f;
  const bool reducing_magnitude = std::abs(target) < std::abs(current);
  const float rate = (crossing_zero || reducing_magnitude) ? max_brake : max_accel;
  const float maximum_delta = std::max(rate, 0.0f) * dt;
  const float delta = target - current;

  if (std::abs(delta) <= maximum_delta) {
    return target;
  }
  return current + std::copysign(maximum_delta, delta);
}

float dot(const Vector3 & lhs, const Vector3 & rhs) {
  return lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2];
}

Vector3 multiply(const Matrix3 & matrix, const Vector3 & vector) {
  Vector3 result{};
  for (std::size_t row = 0; row < kTwistDimension; ++row) {
    for (std::size_t column = 0; column < kTwistDimension; ++column) {
      result[row] += matrix[row][column] * vector[column];
    }
  }
  return result;
}

void addOuterProduct(Matrix3 & matrix, const Vector3 & vector, float weight) {
  for (std::size_t row = 0; row < kTwistDimension; ++row) {
    for (std::size_t column = 0; column < kTwistDimension; ++column) {
      matrix[row][column] += weight * vector[row] * vector[column];
    }
  }
}

bool solveLinearSystem(int dimension, float matrix[3][3], float rhs[3],
                       float solution[3]) {
  float augmented[3][4]{};
  for (int row = 0; row < dimension; ++row) {
    for (int column = 0; column < dimension; ++column) {
      augmented[row][column] = matrix[row][column];
    }
    augmented[row][dimension] = rhs[row];
  }

  for (int pivot = 0; pivot < dimension; ++pivot) {
    int best_row = pivot;
    float best_value = std::abs(augmented[pivot][pivot]);
    for (int row = pivot + 1; row < dimension; ++row) {
      const float value = std::abs(augmented[row][pivot]);
      if (value > best_value) {
        best_value = value;
        best_row = row;
      }
    }

    if (best_value < kLinearSolveTolerance) {
      return false;
    }

    if (best_row != pivot) {
      for (int column = pivot; column <= dimension; ++column) {
        std::swap(augmented[pivot][column], augmented[best_row][column]);
      }
    }

    const float pivot_value = augmented[pivot][pivot];
    for (int column = pivot; column <= dimension; ++column) {
      augmented[pivot][column] /= pivot_value;
    }

    for (int row = 0; row < dimension; ++row) {
      if (row == pivot) {
        continue;
      }
      const float factor = augmented[row][pivot];
      for (int column = pivot; column <= dimension; ++column) {
        augmented[row][column] -= factor * augmented[pivot][column];
      }
    }
  }

  for (int row = 0; row < dimension; ++row) {
    solution[row] = augmented[row][dimension];
  }
  return true;
}

bool invertMatrix3(const Matrix3 & input, Matrix3 & inverse) {
  for (std::size_t column = 0; column < kTwistDimension; ++column) {
    float matrix[3][3]{};
    float rhs[3]{};
    float solution[3]{};

    for (std::size_t row = 0; row < kTwistDimension; ++row) {
      for (std::size_t inner = 0; inner < kTwistDimension; ++inner) {
        matrix[row][inner] = input[row][inner];
      }
    }
    rhs[column] = 1.0f;

    if (!solveLinearSystem(3, matrix, rhs, solution)) {
      return false;
    }

    for (std::size_t row = 0; row < kTwistDimension; ++row) {
      inverse[row][column] = solution[row];
    }
  }
  return true;
}

}  // namespace

class MobileOmniController : public rclcpp::Node {
public:
  MobileOmniController() : Node("mobile_cmd_omni") {
    current_steer_angle_.fill(0.0f);
    previous_candidate_flipped_.fill(false);
    previous_drive_sign_.fill(1.0f);
    previous_allocated_twist_.fill(0.0f);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/control/cmd_vel_omni", 10,
      std::bind(&MobileOmniController::cmdVelCallback, this, std::placeholders::_1));

    feedback_sub_ = create_subscription<chassis_interfaces::msg::ChassisMotorControl>(
      "/feedback/omni_chassis_motor",
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
      std::bind(&MobileOmniController::feedbackCallback, this, std::placeholders::_1));

    motor_control_pub_ = create_publisher<chassis_interfaces::msg::ChassisMotorControl>(
      "/control/omni_chassis_motors", 10);

    last_cmd_time_ = now();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&MobileOmniController::publishControlCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Mobile omni controller started: QP B allocator, 16 steering-mode enumeration");
  }

private:
  struct SteerCandidate {
    float angle{0.0f};
    float drive_sign{1.0f};
    float discrete_cost{0.0f};
    bool flipped{false};
    bool valid{false};
  };

  struct LinearConstraint {
    Vector3 coefficients{};
    float upper_bound{0.0f};
  };

  struct QpProblem {
    Matrix3 hessian{};
    Vector3 gradient{};
    std::vector<LinearConstraint> constraints;
  };

  struct QpResult {
    Vector3 solution{};
    float objective{std::numeric_limits<float>::infinity()};
    bool success{false};
    int active_constraint_count{0};
  };

  struct AllocationResult {
    Vector3 twist{};
    std::array<float, kWheelCount> target_angle{};
    std::array<float, kWheelCount> steering_error{};
    std::array<bool, kWheelCount> flipped{};
    std::array<float, kWheelCount> drive_sign{};
    float qp_objective{std::numeric_limits<float>::infinity()};
    float total_score{std::numeric_limits<float>::infinity()};
    float max_lateral_speed{0.0f};
    float max_steering_error{0.0f};
    int mode_index{-1};
    int active_constraint_count{0};
    bool success{false};
  };

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr message) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    latest_cmd_ = *message;
    last_cmd_time_ = now();
  }

  void feedbackCallback(
      const chassis_interfaces::msg::ChassisMotorControl::SharedPtr message) {
    if (message->position.size() < kWheelCount) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Steering feedback contains %zu positions; expected at least %zu",
        message->position.size(), kWheelCount);
      return;
    }

    for (std::size_t i = 0; i < kWheelCount; ++i) {
      current_steer_angle_[i] = static_cast<float>(message->position[i]);
    }
    steering_feedback_received_ = true;
  }

  void resetCommandGovernor() {
    governed_vx_ = 0.0f;
    governed_vy_ = 0.0f;
    governed_yaw_rate_ = 0.0f;
    command_governor_initialized_ = false;
    previous_allocated_twist_.fill(0.0f);
  }

  void applyCommandGovernor(float vx_ref, float vy_ref, float yaw_rate_ref,
                            float & vx_cmd, float & vy_cmd, float & yaw_rate_cmd) {
    if (!kCommandGovernorEnabled) {
      vx_cmd = vx_ref;
      vy_cmd = vy_ref;
      yaw_rate_cmd = yaw_rate_ref;
      return;
    }

    if (!command_governor_initialized_) {
      governed_vx_ = 0.0f;
      governed_vy_ = 0.0f;
      governed_yaw_rate_ = 0.0f;
      command_governor_initialized_ = true;
    }

    governed_vx_ = limitVelocityRate(
      vx_ref, governed_vx_, kMaxAccelX, kMaxBrakeX, kControlPeriodS);
    governed_vy_ = limitVelocityRate(
      vy_ref, governed_vy_, kMaxAccelY, kMaxBrakeY, kControlPeriodS);
    governed_yaw_rate_ = limitVelocityRate(
      yaw_rate_ref, governed_yaw_rate_, kMaxAccelYaw, kMaxBrakeYaw,
      kControlPeriodS);

    vx_cmd = governed_vx_;
    vy_cmd = governed_vy_;
    yaw_rate_cmd = governed_yaw_rate_;
  }

  float limitProximityCost(float angle) const {
    const float effective_limit =
      std::max(kSteerAngleLimitRad - kSteerLimitMarginRad, 0.1f);
    const float remaining_margin = effective_limit - std::abs(angle);

    if (remaining_margin >= kLimitSoftZoneRad) {
      return 0.0f;
    }
    if (remaining_margin <= 0.0f) {
      return 1.0f;
    }
    return 1.0f - remaining_margin / kLimitSoftZoneRad;
  }

  float candidateDiscreteCost(std::size_t wheel_index,
                              const SteerCandidate & candidate) const {
    if (!candidate.valid) {
      return 1e6f;
    }

    float cost = std::abs(angleDiff(
      candidate.angle, current_steer_angle_[wheel_index]));

    if (candidate_memory_initialized_ &&
        candidate.flipped != previous_candidate_flipped_[wheel_index]) {
      cost += kCandidateSwitchCost;
    }

    if (candidate_memory_initialized_ &&
        candidate.drive_sign * previous_drive_sign_[wheel_index] < 0.0f) {
      cost += kCandidateReverseCost;
    }

    cost += kLimitCostGain * limitProximityCost(candidate.angle);
    return cost;
  }

  std::array<SteerCandidate, 2> buildSteerCandidates(
      std::size_t wheel_index, float raw_angle) const {
    const float effective_limit =
      std::max(kSteerAngleLimitRad - kSteerLimitMarginRad, 0.1f);

    std::array<SteerCandidate, 2> candidates{};

    candidates[0].angle = wrapPi(raw_angle);
    candidates[0].drive_sign = 1.0f;
    candidates[0].flipped = false;
    candidates[0].valid = std::abs(candidates[0].angle) <= effective_limit;
    candidates[0].discrete_cost = candidateDiscreteCost(wheel_index, candidates[0]);

    candidates[1].angle = wrapPi(raw_angle + kPi);
    candidates[1].drive_sign = -1.0f;
    candidates[1].flipped = true;
    candidates[1].valid = std::abs(candidates[1].angle) <= effective_limit;
    candidates[1].discrete_cost = candidateDiscreteCost(wheel_index, candidates[1]);

    // 正常软限位大于 90 度时，两个等价解至少有一个可行。
    // 若配置异常导致都不可行，提供一个夹紧后的安全候选。
    if (!candidates[0].valid && !candidates[1].valid) {
      candidates[0].angle = clampf(candidates[0].angle,
                                   -effective_limit, effective_limit);
      candidates[0].drive_sign = 1.0f;
      candidates[0].flipped = false;
      candidates[0].valid = true;
      candidates[0].discrete_cost = 1e4f;
    }

    return candidates;
  }

  Vector3 wheelLongitudinalRow(std::size_t wheel_index,
                               float steer_angle) const {
    const bool is_right = wheel_index == 1 || wheel_index == 3;
    const bool is_front = wheel_index == 0 || wheel_index == 1;
    const float x_sign = is_right ? 1.0f : -1.0f;
    const float y_sign = is_front ? 1.0f : -1.0f;
    const float cosine = std::cos(steer_angle);
    const float sine = std::sin(steer_angle);

    return Vector3{
      cosine,
      sine,
      cosine * x_sign * kChassisWidthM * 0.5f +
      sine * y_sign * kChassisLengthM * 0.5f};
  }

  Vector3 wheelLateralRow(std::size_t wheel_index,
                          float steer_angle) const {
    const bool is_right = wheel_index == 1 || wheel_index == 3;
    const bool is_front = wheel_index == 0 || wheel_index == 1;
    const float x_sign = is_right ? 1.0f : -1.0f;
    const float y_sign = is_front ? 1.0f : -1.0f;
    const float cosine = std::cos(steer_angle);
    const float sine = std::sin(steer_angle);

    return Vector3{
      -sine,
      cosine,
      -sine * x_sign * kChassisWidthM * 0.5f +
      cosine * y_sign * kChassisLengthM * 0.5f};
  }

  void addQuadraticReference(QpProblem & problem, std::size_t dimension,
                             float weight, float reference) const {
    problem.hessian[dimension][dimension] += weight;
    problem.gradient[dimension] -= weight * reference;
  }

  void addSymmetricBound(QpProblem & problem, const Vector3 & coefficients,
                         float absolute_bound) const {
    const float bound = std::max(absolute_bound, 0.0f);
    problem.constraints.push_back(LinearConstraint{coefficients, bound});
    problem.constraints.push_back(LinearConstraint{
      Vector3{-coefficients[0], -coefficients[1], -coefficients[2]}, bound});
  }

  QpProblem buildQpProblem(
      const Vector3 & command,
      const std::array<float, kWheelCount> & target_angle,
      const std::array<float, kWheelCount> & steering_error) const {
    QpProblem problem;
    problem.constraints.reserve(14);

    addQuadraticReference(problem, 0, kQpTrackX, command[0]);
    addQuadraticReference(problem, 1, kQpTrackY, command[1]);
    addQuadraticReference(problem, 2, kQpTrackYaw, command[2]);

    addQuadraticReference(problem, 0, kQpSmoothX, previous_allocated_twist_[0]);
    addQuadraticReference(problem, 1, kQpSmoothY, previous_allocated_twist_[1]);
    addQuadraticReference(problem, 2, kQpSmoothYaw, previous_allocated_twist_[2]);

    for (std::size_t dimension = 0; dimension < kTwistDimension; ++dimension) {
      problem.hessian[dimension][dimension] += kQpRegularization;
    }

    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      // 实际舵角决定当前轮胎能够无侧滑执行的方向。
      const Vector3 longitudinal = wheelLongitudinalRow(
        wheel, current_steer_angle_[wheel]);
      const Vector3 lateral = wheelLateralRow(
        wheel, current_steer_angle_[wheel]);

      // 侧向速度平方进入代价函数，减少重载轮胎横向拖拽。
      addOuterProduct(problem.hessian, lateral, kQpLateralSlipWeight);

      // 轻量跟踪命令在当前轮向上的投影，改善低权重方向的响应。
      const float requested_wheel_speed = dot(longitudinal, command);
      addOuterProduct(problem.hessian, longitudinal, kQpWheelSpeedTrackWeight);
      for (std::size_t dimension = 0; dimension < kTwistDimension; ++dimension) {
        problem.gradient[dimension] -=
          kQpWheelSpeedTrackWeight * requested_wheel_speed * longitudinal[dimension];
      }

      const float alignment_scale = linearScaleFromError(
        steering_error[wheel],
        kAlignFullSpeedErrorRad,
        kAlignCrawlErrorRad,
        kAlignMinimumSpeedScale);
      const float wheel_speed_limit = kMaxWheelLinearSpeed * alignment_scale;
      addSymmetricBound(problem, longitudinal, wheel_speed_limit);
    }

    addSymmetricBound(problem, Vector3{1.0f, 0.0f, 0.0f}, kMaxBodySpeedX);
    addSymmetricBound(problem, Vector3{0.0f, 1.0f, 0.0f}, kMaxBodySpeedY);
    addSymmetricBound(problem, Vector3{0.0f, 0.0f, 1.0f}, kMaxBodyYawRate);

    (void)target_angle;  // 目标角已通过 steering_error 进入约束。
    return problem;
  }

  float evaluateObjective(const QpProblem & problem,
                          const Vector3 & solution) const {
    const Vector3 hessian_times_solution = multiply(problem.hessian, solution);
    return 0.5f * dot(solution, hessian_times_solution) +
           dot(problem.gradient, solution);
  }

  bool isFeasible(const QpProblem & problem, const Vector3 & solution) const {
    for (const LinearConstraint & constraint : problem.constraints) {
      if (dot(constraint.coefficients, solution) >
          constraint.upper_bound + kQpFeasibilityTolerance) {
        return false;
      }
    }
    return true;
  }

  bool solveForActiveSet(
      const QpProblem & problem,
      const Matrix3 & inverse_hessian,
      const Vector3 & unconstrained_solution,
      const std::array<int, 3> & active_indices,
      int active_count,
      Vector3 & solution) const {
    if (active_count == 0) {
      solution = unconstrained_solution;
      return isFeasible(problem, solution);
    }

    float schur_matrix[3][3]{};
    float rhs[3]{};
    float multipliers[3]{};

    std::array<Vector3, 3> inverse_hessian_times_a{};
    for (int row = 0; row < active_count; ++row) {
      const LinearConstraint & constraint = problem.constraints[active_indices[row]];
      inverse_hessian_times_a[row] = multiply(inverse_hessian, constraint.coefficients);
      rhs[row] = dot(constraint.coefficients, unconstrained_solution) -
                 constraint.upper_bound;
    }

    for (int row = 0; row < active_count; ++row) {
      const LinearConstraint & row_constraint = problem.constraints[active_indices[row]];
      for (int column = 0; column < active_count; ++column) {
        schur_matrix[row][column] = dot(
          row_constraint.coefficients,
          inverse_hessian_times_a[column]);
      }
    }

    if (!solveLinearSystem(active_count, schur_matrix, rhs, multipliers)) {
      return false;
    }

    for (int i = 0; i < active_count; ++i) {
      if (multipliers[i] < -kQpMultiplierTolerance) {
        return false;
      }
    }

    solution = unconstrained_solution;
    for (int active = 0; active < active_count; ++active) {
      for (std::size_t dimension = 0; dimension < kTwistDimension; ++dimension) {
        solution[dimension] -=
          inverse_hessian_times_a[active][dimension] * multipliers[active];
      }
    }

    return isFeasible(problem, solution);
  }

  void considerActiveSet(
      const QpProblem & problem,
      const Matrix3 & inverse_hessian,
      const Vector3 & unconstrained_solution,
      const std::array<int, 3> & active_indices,
      int active_count,
      QpResult & best_result) const {
    Vector3 candidate{};
    if (!solveForActiveSet(
          problem, inverse_hessian, unconstrained_solution,
          active_indices, active_count, candidate)) {
      return;
    }

    const float objective = evaluateObjective(problem, candidate);
    if (!best_result.success || objective < best_result.objective) {
      best_result.solution = candidate;
      best_result.objective = objective;
      best_result.success = true;
      best_result.active_constraint_count = active_count;
    }
  }

  QpResult solveQp(const QpProblem & problem) const {
    QpResult result;

    Matrix3 inverse_hessian{};
    if (!invertMatrix3(problem.hessian, inverse_hessian)) {
      return result;
    }

    const Vector3 negative_gradient{
      -problem.gradient[0],
      -problem.gradient[1],
      -problem.gradient[2]};
    const Vector3 unconstrained_solution = multiply(
      inverse_hessian, negative_gradient);

    std::array<int, 3> active_indices{-1, -1, -1};
    considerActiveSet(
      problem, inverse_hessian, unconstrained_solution,
      active_indices, 0, result);

    const int constraint_count = static_cast<int>(problem.constraints.size());
    for (int first = 0; first < constraint_count; ++first) {
      active_indices[0] = first;
      considerActiveSet(
        problem, inverse_hessian, unconstrained_solution,
        active_indices, 1, result);
    }

    for (int first = 0; first < constraint_count; ++first) {
      for (int second = first + 1; second < constraint_count; ++second) {
        active_indices[0] = first;
        active_indices[1] = second;
        considerActiveSet(
          problem, inverse_hessian, unconstrained_solution,
          active_indices, 2, result);
      }
    }

    for (int first = 0; first < constraint_count; ++first) {
      for (int second = first + 1; second < constraint_count; ++second) {
        for (int third = second + 1; third < constraint_count; ++third) {
          active_indices[0] = first;
          active_indices[1] = second;
          active_indices[2] = third;
          considerActiveSet(
            problem, inverse_hessian, unconstrained_solution,
            active_indices, 3, result);
        }
      }
    }

    return result;
  }

  AllocationResult solveAllocation(const Vector3 & command) const {
    std::array<std::array<SteerCandidate, 2>, kWheelCount> candidates{};
    std::array<int, kWheelCount> candidate_count{};

    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      const bool is_right = wheel == 1 || wheel == 3;
      const bool is_front = wheel == 0 || wheel == 1;
      const float x_sign = is_right ? 1.0f : -1.0f;
      const float y_sign = is_front ? 1.0f : -1.0f;

      const float corner_vx =
        command[0] + x_sign * kChassisWidthM * command[2] * 0.5f;
      const float corner_vy =
        command[1] + y_sign * kChassisLengthM * command[2] * 0.5f;
      const float corner_speed = std::hypot(corner_vx, corner_vy);

      if (corner_speed < kWheelVectorDeadbandMps) {
        SteerCandidate hold;
        hold.angle = current_steer_angle_[wheel];
        hold.drive_sign = previous_drive_sign_[wheel];
        hold.flipped = previous_candidate_flipped_[wheel];
        hold.valid = true;
        hold.discrete_cost = 0.0f;
        candidates[wheel][0] = hold;
        candidate_count[wheel] = 1;
      } else {
        candidates[wheel] = buildSteerCandidates(
          wheel, std::atan2(corner_vy, corner_vx));
        candidate_count[wheel] = candidates[wheel][1].valid ? 2 : 1;
      }
    }

    AllocationResult best;

    // 四轮每轮最多两个候选，共 2^4 = 16 种离散模式。
    for (int mode = 0; mode < 16; ++mode) {
      std::array<float, kWheelCount> target_angle{};
      std::array<float, kWheelCount> steering_error{};
      std::array<bool, kWheelCount> flipped{};
      std::array<float, kWheelCount> drive_sign{};
      float discrete_cost = 0.0f;
      bool valid_mode = true;
      float max_steering_error = 0.0f;

      for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        const int candidate_index = (mode >> wheel) & 0x1;
        if (candidate_index >= candidate_count[wheel] ||
            !candidates[wheel][candidate_index].valid) {
          valid_mode = false;
          break;
        }

        const SteerCandidate & selected = candidates[wheel][candidate_index];
        target_angle[wheel] = selected.angle;
        flipped[wheel] = selected.flipped;
        drive_sign[wheel] = selected.drive_sign;
        steering_error[wheel] = std::abs(angleDiff(
          selected.angle, current_steer_angle_[wheel]));
        max_steering_error = std::max(
          max_steering_error, steering_error[wheel]);
        discrete_cost += selected.discrete_cost;
      }

      if (!valid_mode) {
        continue;
      }

      const QpProblem problem = buildQpProblem(
        command, target_angle, steering_error);
      const QpResult qp_result = solveQp(problem);
      if (!qp_result.success) {
        continue;
      }

      float max_lateral_speed = 0.0f;
      for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        const Vector3 lateral = wheelLateralRow(
          wheel, current_steer_angle_[wheel]);
        max_lateral_speed = std::max(
          max_lateral_speed, std::abs(dot(lateral, qp_result.solution)));
      }

      const float total_score = qp_result.objective +
        kQpCandidateCostWeight * discrete_cost;

      if (!best.success || total_score < best.total_score) {
        best.twist = qp_result.solution;
        best.target_angle = target_angle;
        best.steering_error = steering_error;
        best.flipped = flipped;
        best.drive_sign = drive_sign;
        best.qp_objective = qp_result.objective;
        best.total_score = total_score;
        best.max_lateral_speed = max_lateral_speed;
        best.max_steering_error = max_steering_error;
        best.mode_index = mode;
        best.active_constraint_count = qp_result.active_constraint_count;
        best.success = true;
      }
    }

    return best;
  }

  void computeWheels(float vx, float vy, float yaw_rate,
                     std::array<float, kWheelCount> & output_angle,
                     std::array<float, kWheelCount> & output_rpm) {
    const bool command_is_zero =
      std::hypot(vx, vy) < kLinearCommandDeadbandMps &&
      std::abs(yaw_rate) < kYawCommandDeadbandRadps;

    if (command_is_zero) {
      output_angle = current_steer_angle_;
      output_rpm.fill(0.0f);
      previous_allocated_twist_.fill(0.0f);
      return;
    }

    const Vector3 command{vx, vy, yaw_rate};
    const AllocationResult allocation = solveAllocation(command);

    if (!allocation.success) {
      output_angle = current_steer_angle_;
      output_rpm.fill(0.0f);
      previous_allocated_twist_.fill(0.0f);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "QP allocation failed; holding steering and commanding zero wheel speed");
      return;
    }

    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      output_angle[wheel] = allocation.target_angle[wheel];

      // 轮速按实际反馈舵角投影，避免舵轮未到位时按理论目标角硬推。
      const Vector3 longitudinal = wheelLongitudinalRow(
        wheel, current_steer_angle_[wheel]);
      const float wheel_speed = dot(longitudinal, allocation.twist);
      output_rpm[wheel] = speedToRpm(wheel_speed);
    }

    previous_allocated_twist_ = allocation.twist;
    candidate_memory_initialized_ = true;
    previous_candidate_flipped_ = allocation.flipped;
    previous_drive_sign_ = allocation.drive_sign;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "OMNI_B_QP: cmd=(%.3f,%.3f,%.3f) alloc=(%.3f,%.3f,%.3f) "
      "mode=%d active=%d steer_err_max=%.3f lateral_max=%.3f score=%.4f",
      command[0], command[1], command[2],
      allocation.twist[0], allocation.twist[1], allocation.twist[2],
      allocation.mode_index, allocation.active_constraint_count,
      allocation.max_steering_error, allocation.max_lateral_speed,
      allocation.total_score);
  }

  void publishControlCallback() {
    geometry_msgs::msg::Twist command;
    rclcpp::Time command_time;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command = latest_cmd_;
      command_time = last_cmd_time_;
    }

    const bool command_timed_out =
      (now() - command_time).seconds() > kCommandTimeoutS;

    float vx = 0.0f;
    float vy = 0.0f;
    float yaw_rate = 0.0f;

    if (command_timed_out) {
      resetCommandGovernor();
    } else {
      applyCommandGovernor(
        static_cast<float>(command.linear.x),
        static_cast<float>(command.linear.y),
        static_cast<float>(command.angular.z),
        vx, vy, yaw_rate);
    }

    std::array<float, kWheelCount> wheel_angle{};
    std::array<float, kWheelCount> wheel_rpm{};

    if (!steering_feedback_received_) {
      wheel_angle.fill(0.0f);
      wheel_rpm.fill(0.0f);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for steering feedback; wheel speed command is held at zero");
    } else {
      computeWheels(vx, vy, yaw_rate, wheel_angle, wheel_rpm);
    }

    chassis_interfaces::msg::ChassisMotorControl output;
    output.header.stamp = now();
    output.header.frame_id = "base_link";
    output.position.resize(kWheelCount);
    output.speed.resize(kWheelCount);

    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      output.position[wheel] = wheel_angle[wheel];
      output.speed[wheel] = wheel_rpm[wheel];
    }

    output.estop = 0;
    output.neutral = 0;
    motor_control_pub_->publish(output);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<chassis_interfaces::msg::ChassisMotorControl>::SharedPtr feedback_sub_;
  rclcpp::Publisher<chassis_interfaces::msg::ChassisMotorControl>::SharedPtr motor_control_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex command_mutex_;
  geometry_msgs::msg::Twist latest_cmd_;
  rclcpp::Time last_cmd_time_;

  std::array<float, kWheelCount> current_steer_angle_{};
  bool steering_feedback_received_{false};

  float governed_vx_{0.0f};
  float governed_vy_{0.0f};
  float governed_yaw_rate_{0.0f};
  bool command_governor_initialized_{false};

  Vector3 previous_allocated_twist_{};

  std::array<bool, kWheelCount> previous_candidate_flipped_{};
  std::array<float, kWheelCount> previous_drive_sign_{};
  bool candidate_memory_initialized_{false};
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MobileOmniController>());
  rclcpp::shutdown();
  return 0;
}
