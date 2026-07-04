// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include "nav2_mppi_controller/optimizer.hpp"

#include <chrono>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>

#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

namespace mppi
{

void Optimizer::initialize(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  ParametersHandler * param_handler)
{
  parent_ = parent;
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  parameters_handler_ = param_handler;

  auto node = parent_.lock();
  logger_ = node->get_logger();

  getParams();

  critic_manager_.on_configure(parent_, name_, costmap_ros_, parameters_handler_);
  noise_generator_.initialize(settings_, isHolonomic(), name_, parameters_handler_);

  reset();
}

void Optimizer::shutdown()
{
  noise_generator_.shutdown();
}

void Optimizer::getParams()
{
  std::string motion_model_name;

  auto & s = settings_;
  auto getParam = parameters_handler_->getParamGetter(name_);
  auto getParentParam = parameters_handler_->getParamGetter("");
  getParam(s.model_dt, "model_dt", 0.05f);
  getParam(s.time_steps, "time_steps", 56);
  getParam(s.batch_size, "batch_size", 1000);
  getParam(s.iteration_count, "iteration_count", 1);
  getParam(s.temperature, "temperature", 0.3f);
  getParam(s.gamma, "gamma", 0.015f);
  getParam(s.base_constraints.vx_max, "vx_max", 0.5);
  getParam(s.base_constraints.vx_min, "vx_min", -0.35);
  getParam(s.base_constraints.vy, "vy_max", 0.5);
  getParam(s.base_constraints.wz, "wz_max", 1.9);
  getParam(s.base_constraints.ax_max, "ax_max", 3.0f);
  getParam(s.base_constraints.ax_min, "ax_min", -3.0f);
  getParam(s.base_constraints.ay_max, "ay_max", 3.0f);
  getParam(s.base_constraints.ay_min, "ay_min", -3.0f);
  getParam(s.base_constraints.az_max, "az_max", 3.5f);
  getParam(s.base_constraints.jx_max, "jx_max", 1000.0f);
  getParam(s.base_constraints.jz_max, "jz_max", 1000.0f);
  getParam(s.sampling_std.vx, "vx_std", 0.2);
  getParam(s.sampling_std.vy, "vy_std", 0.2);
  getParam(s.sampling_std.wz, "wz_std", 0.4);
  getParam(s.retry_attempt_limit, "retry_attempt_limit", 1);
  getParam(s.open_loop, "open_loop", false);

  // 确保加速度参数符号正确
  s.base_constraints.ax_max = fabs(s.base_constraints.ax_max);
  s.base_constraints.az_max = fabs(s.base_constraints.az_max);
  s.base_constraints.jx_max = fabs(s.base_constraints.jx_max);
  s.base_constraints.jz_max = fabs(s.base_constraints.jz_max);
  if (s.base_constraints.ax_min > 0.0) {
    s.base_constraints.ax_min = -1.0 * s.base_constraints.ax_min;
    RCLCPP_WARN(logger_, "ax_min has wrong sign; auto-converted to negative");
  }
  if (s.base_constraints.ay_min > 0.0) {
    s.base_constraints.ay_min = -1.0 * s.base_constraints.ay_min;
    RCLCPP_WARN(logger_, "ay_min has wrong sign; auto-converted to negative");
  }

  getParam(motion_model_name, "motion_model", std::string("DiffDrive"));

  s.constraints = s.base_constraints;
  setMotionModel(motion_model_name);
  parameters_handler_->addPostCallback([this]() {reset();});

  double controller_frequency;
  getParentParam(controller_frequency, "controller_frequency", 0.0, ParameterType::Static);
  setOffset(controller_frequency);
}

void Optimizer::setOffset(double controller_frequency)
{
  const double controller_period = 1.0 / controller_frequency;
  constexpr double eps = 1e-6;

  if ((controller_period + eps) < settings_.model_dt) {
    RCLCPP_WARN(
      logger_,
      "Controller period is less then model dt, consider setting it equal");
  } else if (abs(controller_period - settings_.model_dt) < eps) {
    RCLCPP_INFO(
      logger_,
      "Controller period is equal to model dt. Control sequence "
      "shifting is ON");
    settings_.shift_control_sequence = true;
  } else {
    throw std::runtime_error(
            "Controller period more then model dt, set it equal to model dt");
  }
}

void Optimizer::reset()
{
  state_.reset(settings_.batch_size, settings_.time_steps);
  control_sequence_.reset(settings_.time_steps);
  control_history_[0] = {0.0, 0.0, 0.0};
  control_history_[1] = {0.0, 0.0, 0.0};
  control_history_[2] = {0.0, 0.0, 0.0};
  control_history_[3] = {0.0, 0.0, 0.0};

  if (settings_.open_loop) {
    last_command_vel_ = geometry_msgs::msg::Twist();
  }

  costs_ = models::Vector::Zero(settings_.batch_size);
  generated_trajectories_.reset(settings_.batch_size, settings_.time_steps);

  noise_generator_.reset(settings_, isHolonomic());
  motion_model_->initialize(settings_.constraints, settings_.model_dt);
  RCLCPP_INFO(logger_, "Optimizer reset");
}

geometry_msgs::msg::TwistStamped Optimizer::evalControl(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan, nav2_core::GoalChecker * goal_checker)
{
  prepare(robot_pose, robot_speed, plan, goal_checker);

  do {
    optimize();
  } while (fallback(critics_data_.fail_flag));

  auto control = getControlFromSequenceAsTwist(plan.header.stamp);

  last_command_vel_ = control.twist;

  if (settings_.shift_control_sequence) {
    shiftControlSequence();
  }

  return control;
}

void Optimizer::optimize()
{
  constexpr double kSlowOptimizeWarnMs = 100.0;

  for (size_t i = 0; i < settings_.iteration_count; ++i) {
    auto t_generate_start = std::chrono::high_resolution_clock::now();
    generateNoisedTrajectories();
    auto t_generate_end = std::chrono::high_resolution_clock::now();

    auto t_critic_start = std::chrono::high_resolution_clock::now();
    critic_manager_.evalTrajectoriesScores(critics_data_);
    auto t_critic_end = std::chrono::high_resolution_clock::now();

    auto t_update_start = std::chrono::high_resolution_clock::now();
    updateControlSequence();
    auto t_update_end = std::chrono::high_resolution_clock::now();

    const double generate_ms =
      std::chrono::duration<double, std::milli>(t_generate_end - t_generate_start).count();
    const double critic_ms =
      std::chrono::duration<double, std::milli>(t_critic_end - t_critic_start).count();
    const double update_ms =
      std::chrono::duration<double, std::milli>(t_update_end - t_update_start).count();
    const double total_ms = generate_ms + critic_ms + update_ms;

    if (
      total_ms >= kSlowOptimizeWarnMs || generate_ms >= kSlowOptimizeWarnMs ||
      critic_ms >= kSlowOptimizeWarnMs || update_ms >= kSlowOptimizeWarnMs)
    {
      RCLCPP_WARN(
        logger_,
        "Optimizer iteration %zu timing (ms): generate=%.3f critics=%.3f update=%.3f total=%.3f "
        "[batch=%u steps=%u fail=%d]. "
        "MPPI single optimization too long; use generate/critics/update to tell if sampling/prediction, critic scoring, or control-sequence update is slow.",
        i, generate_ms, critic_ms, update_ms, total_ms,
        settings_.batch_size, settings_.time_steps, critics_data_.fail_flag);
    }
  }
}

bool Optimizer::fallback(bool fail)
{
  static size_t counter = 0;

  if (!fail) {
    counter = 0;
    return false;
  }

  reset();

  if (++counter > settings_.retry_attempt_limit) {
    counter = 0;
    throw std::runtime_error(
            "Optimizer fail to compute path. MPPI optimizer failed repeatedly; no valid control from current path/costmap/constraints.");
  }

  return true;
}

void Optimizer::prepare(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan, nav2_core::GoalChecker * goal_checker)
{
  state_.pose = robot_pose;
  state_.speed = settings_.open_loop ? last_command_vel_ : robot_speed;
  path_ = utils::toTensor(plan);
  costs_.fill(0);

  critics_data_.fail_flag = false;
  critics_data_.goal_checker = goal_checker;
  critics_data_.motion_model = motion_model_;
  critics_data_.furthest_reached_path_point.reset();
  critics_data_.path_pts_valid.reset();
}

void Optimizer::shiftControlSequence()
{
  auto shift = [](models::Vector & sequence) {
      if (sequence.size() < 2) {
        return;
      }
      sequence.head(sequence.size() - 1) = sequence.tail(sequence.size() - 1).eval();
      sequence(sequence.size() - 1) = sequence(sequence.size() - 2);
    };

  shift(control_sequence_.vx);
  shift(control_sequence_.wz);

  if (isHolonomic()) {
    shift(control_sequence_.vy);
  }
}

void Optimizer::generateNoisedTrajectories()
{
  noise_generator_.setNoisedControls(state_, control_sequence_);
  noise_generator_.generateNextNoises();
  updateStateVelocities(state_);
  integrateStateVelocities(generated_trajectories_, state_);
}

bool Optimizer::isHolonomic() const {return motion_model_->isHolonomic();}

void Optimizer::applyControlSequenceConstraints()
{
  auto & s = settings_;

  const float max_delta_vx = s.model_dt * s.constraints.ax_max;
  const float min_delta_vx = s.model_dt * s.constraints.ax_min;
  const float max_delta_wz = s.model_dt * s.constraints.az_max;
  const float max_delta_ax = s.model_dt * s.constraints.jx_max;
  const float max_delta_wz_acc = s.model_dt * s.constraints.jz_max;

  float vx_last = std::clamp(control_sequence_.vx(0), s.constraints.vx_min, s.constraints.vx_max);
  float wz_last = std::clamp(control_sequence_.wz(0), -s.constraints.wz, s.constraints.wz);
  control_sequence_.vx(0) = vx_last;
  control_sequence_.wz(0) = wz_last;

  float vx_acc_last = (vx_last - state_.speed.linear.x) / s.model_dt;
  vx_acc_last = std::clamp(vx_acc_last, s.constraints.ax_min, s.constraints.ax_max);
  float wz_acc_last = (wz_last - state_.speed.angular.z) / s.model_dt;
  wz_acc_last = std::clamp(wz_acc_last, -s.constraints.az_max, s.constraints.az_max);

  float vy_last = 0.0f;
  if (isHolonomic()) {
    vy_last = std::clamp(control_sequence_.vy(0), -s.constraints.vy, s.constraints.vy);
    control_sequence_.vy(0) = vy_last;
  }

  for (unsigned int i = 1; i < control_sequence_.vx.size(); i++) {
    // 线加速度约束
    float vx_curr = std::clamp(control_sequence_.vx(i), s.constraints.vx_min, s.constraints.vx_max);
    if (vx_last > 0.0f) {
      vx_curr = std::clamp(vx_curr, vx_last + min_delta_vx, vx_last + max_delta_vx);
    } else {
      vx_curr = std::clamp(vx_curr, vx_last - max_delta_vx, vx_last - min_delta_vx);
    }

    float vx_acc = (vx_curr - vx_last) / s.model_dt;
    vx_acc = std::clamp(vx_acc, vx_acc_last - max_delta_ax, vx_acc_last + max_delta_ax);
    vx_acc = std::clamp(vx_acc, s.constraints.ax_min, s.constraints.ax_max);
    vx_curr = vx_last + vx_acc * s.model_dt;
    control_sequence_.vx(i) = vx_curr;
    vx_last = vx_curr;
    vx_acc_last = vx_acc;

    float wz_curr = std::clamp(control_sequence_.wz(i), -s.constraints.wz, s.constraints.wz);
    wz_curr = std::clamp(wz_curr, wz_last - max_delta_wz, wz_last + max_delta_wz);
    float wz_acc = (wz_curr - wz_last) / s.model_dt;
    wz_acc = std::clamp(wz_acc, wz_acc_last - max_delta_wz_acc, wz_acc_last + max_delta_wz_acc);
    wz_acc = std::clamp(wz_acc, -s.constraints.az_max, s.constraints.az_max);
    wz_curr = wz_last + wz_acc * s.model_dt;
    control_sequence_.wz(i) = wz_curr;
    wz_last = wz_curr;
    wz_acc_last = wz_acc;

    if (isHolonomic()) {
      const float max_delta_vy = s.model_dt * s.constraints.ay_max;
      const float min_delta_vy = s.model_dt * s.constraints.ay_min;

      float vy_curr = std::clamp(control_sequence_.vy(i), -s.constraints.vy, s.constraints.vy);
      if (vy_last > 0.0f) {
        vy_curr = std::clamp(vy_curr, vy_last + min_delta_vy, vy_last + max_delta_vy);
      } else {
        vy_curr = std::clamp(vy_curr, vy_last - max_delta_vy, vy_last - min_delta_vy);
      }
      control_sequence_.vy(i) = vy_curr;
      vy_last = vy_curr;
    }
  }

  // 加速度/加加速度回推可能把重新计算出的速度带出 profile 上限。
  // 在车辆运动学约束前再次硬限幅，保证最终发布的 offset 控制量也满足阶段速度约束。
  clampControlSequence();
  motion_model_->applyConstraints(control_sequence_);
}

void Optimizer::updateStateVelocities(
  models::State & state) const
{
  updateInitialStateVelocities(state);
  propagateStateVelocitiesFromInitials(state);
}

void Optimizer::updateInitialStateVelocities(
  models::State & state) const
{
  state.vx.col(0).setConstant(state.speed.linear.x);
  state.wz.col(0).setConstant(state.speed.angular.z);

  if (isHolonomic()) {
    state.vy.col(0).setConstant(state.speed.linear.y);
  }
}

void Optimizer::propagateStateVelocitiesFromInitials(
  models::State & state) const
{
  motion_model_->predict(state);
}

void Optimizer::integrateStateVelocities(
  models::Matrix & trajectory,
  const models::Matrix & sequence) const
{
  const float initial_yaw = tf2::getYaw(state_.pose.pose.orientation);
  float x = state_.pose.pose.position.x;
  float y = state_.pose.pose.position.y;
  float yaw = initial_yaw;

  for (Eigen::Index i = 0; i < sequence.rows(); ++i) {
    yaw += sequence(i, 1) * settings_.model_dt;
    trajectory(i, 2) = yaw;

    const float yaw_for_vel = i == 0 ? initial_yaw : trajectory(i - 1, 2);
    const float yaw_cos = cosf(yaw_for_vel);
    const float yaw_sin = sinf(yaw_for_vel);
    float dx = sequence(i, 0) * yaw_cos;
    float dy = sequence(i, 0) * yaw_sin;

    if (isHolonomic()) {
      dx -= sequence(i, 2) * yaw_sin;
      dy += sequence(i, 2) * yaw_cos;
    }

    x += dx * settings_.model_dt;
    y += dy * settings_.model_dt;
    trajectory(i, 0) = x;
    trajectory(i, 1) = y;
  }
}

void Optimizer::integrateStateVelocities(
  models::Trajectories & trajectories,
  const models::State & state) const
{
  const float initial_yaw = tf2::getYaw(state.pose.pose.orientation);

  for (Eigen::Index r = 0; r < state.vx.rows(); ++r) {
    float x = state.pose.pose.position.x;
    float y = state.pose.pose.position.y;
    float yaw = initial_yaw;

    for (Eigen::Index c = 0; c < state.vx.cols(); ++c) {
      yaw += state.wz(r, c) * settings_.model_dt;
      trajectories.yaws(r, c) = yaw;

      const float yaw_for_vel = c == 0 ? initial_yaw : trajectories.yaws(r, c - 1);
      const float yaw_cos = cosf(yaw_for_vel);
      const float yaw_sin = sinf(yaw_for_vel);
      float dx = state.vx(r, c) * yaw_cos;
      float dy = state.vx(r, c) * yaw_sin;

      if (isHolonomic()) {
        dx -= state.vy(r, c) * yaw_sin;
        dy += state.vy(r, c) * yaw_cos;
      }

      x += dx * settings_.model_dt;
      y += dy * settings_.model_dt;
      trajectories.x(r, c) = x;
      trajectories.y(r, c) = y;
    }
  }
}

models::Matrix Optimizer::getOptimizedTrajectory()
{
  models::Matrix sequence =
    models::Matrix::Zero(settings_.time_steps, isHolonomic() ? 3u : 2u);
  models::Matrix trajectories = models::Matrix::Zero(settings_.time_steps, 3);

  sequence.col(0) = control_sequence_.vx;
  sequence.col(1) = control_sequence_.wz;

  if (isHolonomic()) {
    sequence.col(2) = control_sequence_.vy;
  }

  integrateStateVelocities(trajectories, sequence);
  return trajectories;
}

void Optimizer::updateControlSequence()
{
  auto & s = settings_;
  auto bounded_noises_vx = state_.cvx.rowwise() - control_sequence_.vx.transpose();
  auto bounded_noises_wz = state_.cwz.rowwise() - control_sequence_.wz.transpose();
  costs_ += s.gamma / powf(s.sampling_std.vx, 2) *
    (bounded_noises_vx.rowwise() * control_sequence_.vx.transpose()).rowwise().sum();
  costs_ += s.gamma / powf(s.sampling_std.wz, 2) *
    (bounded_noises_wz.rowwise() * control_sequence_.wz.transpose()).rowwise().sum();

  if (isHolonomic()) {
    auto bounded_noises_vy = state_.cvy.rowwise() - control_sequence_.vy.transpose();
    costs_ += s.gamma / powf(s.sampling_std.vy, 2) *
      (bounded_noises_vy.rowwise() * control_sequence_.vy.transpose()).rowwise().sum();
  }

  const auto costs_normalized = costs_ - costs_.minCoeff();
  const auto exponents = (-1.0f / settings_.temperature * costs_normalized).exp();
  const models::Vector softmaxes = exponents / exponents.sum();

  for (Eigen::Index i = 0; i < state_.cvx.cols(); ++i) {
    control_sequence_.vx(i) = (state_.cvx.col(i) * softmaxes).sum();
    control_sequence_.wz(i) = (state_.cwz.col(i) * softmaxes).sum();
  }
  if (isHolonomic()) {
    for (Eigen::Index i = 0; i < state_.cvy.cols(); ++i) {
      control_sequence_.vy(i) = (state_.cvy.col(i) * softmaxes).sum();
    }
  }

  utils::savitskyGolayFilter(control_sequence_, control_history_, settings_);
  applyControlSequenceConstraints();
}

geometry_msgs::msg::TwistStamped Optimizer::getControlFromSequenceAsTwist(
  const builtin_interfaces::msg::Time & stamp)
{
  unsigned int offset = settings_.shift_control_sequence ? 1 : 0;

  auto vx = control_sequence_.vx(offset);
  auto wz = control_sequence_.wz(offset);

  if (isHolonomic()) {
    auto vy = control_sequence_.vy(offset);
    return utils::toTwistStamped(vx, vy, wz, stamp, costmap_ros_->getBaseFrameID());
  }

  return utils::toTwistStamped(vx, wz, stamp, costmap_ros_->getBaseFrameID());
}

void Optimizer::setMotionModel(const std::string & model)
{
  if (model == "DiffDrive") {
    motion_model_ = std::make_shared<DiffDriveMotionModel>();
  } else if (model == "Omni") {
    motion_model_ = std::make_shared<OmniMotionModel>();
  } else if (model == "Ackermann") {
    motion_model_ = std::make_shared<AckermannMotionModel>(parameters_handler_);
  } else {
    throw std::runtime_error(
            std::string(
              "Model " + model + " is not valid! Valid options are DiffDrive, Omni, "
              "or Ackermann"));
  }
}

void Optimizer::setSpeedLimit(double speed_limit, bool percentage)
{
  speed_limit_active_ = speed_limit != nav2_costmap_2d::NO_SPEED_LIMIT;
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;

  RCLCPP_INFO(
    logger_,
    "MPPI speed limit updated: active=%d value=%.3f percentage=%d. "
    "Received speed limit update; will combine with the current three-stage profile velocity constraints.",
    speed_limit_active_, speed_limit_, speed_limit_is_percentage_);

  applySpeedLimit();
  syncMotionModelConstraints();
  clampControlSequence();
  logActiveConstraints();
}

void Optimizer::applyProfile(const MppiProfileSettings & profile)
{
  auto & s = settings_;
  RCLCPP_DEBUG(
    logger_,
    "MPPI apply profile: base velocity before vx=[%.3f, %.3f] vy=%.3f wz=%.3f, "
    "new vx=[%.3f, %.3f] vy=%.3f wz=%.3f. "
    "Three-stage profile is overriding the MPPI base velocity constraints.",
    s.base_constraints.vx_min, s.base_constraints.vx_max,
    s.base_constraints.vy, s.base_constraints.wz,
    profile.velocity.vx_min, profile.velocity.vx_max,
    profile.velocity.vy_max, profile.velocity.wz_max);

  s.base_constraints.vx_max = profile.velocity.vx_max;
  s.base_constraints.vx_min = profile.velocity.vx_min;
  s.base_constraints.vy = profile.velocity.vy_max;
  s.base_constraints.wz = profile.velocity.wz_max;

  applySpeedLimit();
  syncMotionModelConstraints();
  clampControlSequence();
  critic_manager_.applyCriticProfileSettings(profile.critics);
  logActiveConstraints();
}

void Optimizer::applySpeedLimit()
{
  auto & s = settings_;
  s.constraints = s.base_constraints;

  if (speed_limit_active_) {
    if (speed_limit_is_percentage_) {
      const double ratio = speed_limit_ / 100.0;
      s.constraints.vx_max = s.base_constraints.vx_max * ratio;
      s.constraints.vx_min = s.base_constraints.vx_min * ratio;
      s.constraints.vy = s.base_constraints.vy * ratio;
      s.constraints.wz = s.base_constraints.wz * ratio;
    } else if (std::fabs(s.base_constraints.vx_max) < 1e-6) {
      RCLCPP_WARN(
        logger_,
        "MPPI absolute speed limit ignored because base vx_max is near zero. "
        "Current profile vx_max near 0; cannot compute ratio for absolute speed limit.");
    } else {
      const double ratio = speed_limit_ / s.base_constraints.vx_max;
      s.constraints.vx_max = s.base_constraints.vx_max * ratio;
      s.constraints.vx_min = s.base_constraints.vx_min * ratio;
      s.constraints.vy = s.base_constraints.vy * ratio;
      s.constraints.wz = s.base_constraints.wz * ratio;
    }
  }

  // 轨迹参考速度帽（④ 速度规划最近点速度）：只压前向 vx_max——
  // 减速尾段/曲率限速/障碍零速由剖面逐点编码；倒车与角速度能力保留
  // （prepare 倒退、align 原地旋转不受影响）。
  if (traj_speed_cap_ >= 0.0 && s.constraints.vx_max > traj_speed_cap_) {
    s.constraints.vx_max = traj_speed_cap_;
  }
}

void Optimizer::setTrajectorySpeedCap(double cap_mps)
{
  const double new_cap = cap_mps < 0.0 ? -1.0 : cap_mps;
  if (std::fabs(new_cap - traj_speed_cap_) < 1e-3) {
    return;  // 变化不显著：避免每周期重派生约束与截断控制序列
  }
  traj_speed_cap_ = new_cap;
  applySpeedLimit();
  syncMotionModelConstraints();
  clampControlSequence();
}

void Optimizer::clampControlSequence()
{
  auto & s = settings_;

  if (control_sequence_.vx.size() == 0) {
    return;
  }

  control_sequence_.vx =
    control_sequence_.vx.min(s.constraints.vx_max).max(s.constraints.vx_min);
  control_sequence_.wz =
    control_sequence_.wz.min(s.constraints.wz).max(-s.constraints.wz);

  if (isHolonomic() && control_sequence_.vy.size() > 0) {
    control_sequence_.vy =
      control_sequence_.vy.min(s.constraints.vy).max(-s.constraints.vy);
  }
}

void Optimizer::syncMotionModelConstraints()
{
  if (motion_model_) {
    motion_model_->initialize(settings_.constraints, settings_.model_dt);
  }
}

void Optimizer::logActiveConstraints() const
{
  const auto & s = settings_;
  RCLCPP_DEBUG(
    logger_,
    "MPPI active constraints: vx=[%.3f, %.3f] vy=%.3f wz=%.3f "
    "ax=[%.3f, %.3f] ay=[%.3f, %.3f] az=%.3f jx=%.3f jz=%.3f "
    "speed_limit_active=%d speed_limit=%.3f percentage=%d. "
    "These are the runtime constraints actually used for sampling, prediction, and output clamping.",
    s.constraints.vx_min, s.constraints.vx_max,
    s.constraints.vy, s.constraints.wz,
    s.constraints.ax_min, s.constraints.ax_max,
    s.constraints.ay_min, s.constraints.ay_max,
    s.constraints.az_max, s.constraints.jx_max, s.constraints.jz_max,
    speed_limit_active_, speed_limit_, speed_limit_is_percentage_);
}

models::Trajectories & Optimizer::getGeneratedTrajectories()
{
  return generated_trajectories_;
}

}  // namespace mppi
