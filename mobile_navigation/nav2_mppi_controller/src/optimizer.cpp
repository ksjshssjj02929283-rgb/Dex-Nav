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

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xnoalias.hpp>

#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

namespace mppi
{

using namespace xt::placeholders;  // NOLINT
using xt::evaluation_strategy::immediate;

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
  getParam(s.sampling_std.vx, "vx_std", 0.2);
  getParam(s.sampling_std.vy, "vy_std", 0.2);
  getParam(s.sampling_std.wz, "wz_std", 0.4);
  getParam(s.retry_attempt_limit, "retry_attempt_limit", 1);

  getParam(motion_model_name, "motion_model", std::string("DiffDrive"));

  /* ========== Adaptive MPPI parameters ========== */
  getParam(adapt_dist_max_,  "adaptive.dist_max",  1.2);
  getParam(adapt_dist_mid_,  "adaptive.dist_mid",  0.5);
  getParam(adapt_dist_near_, "adaptive.dist_near", 0.15);

  // Mid-distance targets
  getParam(target_vx_std_mid_, "adaptive.mid.vx_std", 0.08);
  getParam(target_wz_std_mid_, "adaptive.mid.wz_std", 0.15);
  getParam(target_temp_mid_,   "adaptive.mid.temperature", 0.18);

  // Near-goal targets
  getParam(target_vx_std_near_, "adaptive.near.vx_std", 0.05);
  getParam(target_wz_std_near_, "adaptive.near.wz_std", 0.10);
  getParam(target_temp_near_,   "adaptive.near.temperature", 0.15);

  // Safety minimums
  getParam(min_vx_std_, "adaptive.min.vx_std", 0.03);
  getParam(min_wz_std_, "adaptive.min.wz_std", 0.02);
  getParam(min_temperature_, "adaptive.min.temperature", 0.12);

  // Adaptation rates
  getParam(alpha_std_,  "adaptive.alpha.std",  0.5);
  getParam(alpha_temp_, "adaptive.alpha.temp", 0.2);
  getParam(alpha_vel_,  "adaptive.alpha.vel",  0.5);


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

  costs_ = xt::zeros<float>({settings_.batch_size});
  generated_trajectories_.reset(settings_.batch_size, settings_.time_steps);

  noise_generator_.reset(settings_, isHolonomic());
  RCLCPP_INFO(logger_, "Optimizer reset");
}

geometry_msgs::msg::TwistStamped Optimizer::evalControl(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan, nav2_core::GoalChecker * goal_checker)
{
  prepare(robot_pose, robot_speed, plan, goal_checker);

  do {
    // //TODO 更新控制序列生成参数
    updateParameters(robot_pose, plan);
    noise_generator_.reset(settings_, isHolonomic());
    // //TODO 更新轨迹评价参数
    // updateCriticsData();
    optimize();
  } while (fallback(critics_data_.fail_flag));

  utils::savitskyGolayFilter(control_sequence_, control_history_, settings_);
  auto control = getControlFromSequenceAsTwist(plan.header.stamp);

  if (settings_.shift_control_sequence) {
    shiftControlSequence();
  }

  return control;
}
void Optimizer::updateParameters(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const nav_msgs::msg::Path & plan)
{
  if (plan.poses.empty()) {
    return;
  }
  /*重置参数，防止第二次导航触发时参数为近距离参数*/
  getParams();

  /* ================= 1. 目标距离 ================= */
  const auto & goal_pose = plan.poses.back();
  const double dx = robot_pose.pose.position.x - goal_pose.pose.position.x;
  const double dy = robot_pose.pose.position.y - goal_pose.pose.position.y;
  const double dist = std::hypot(dx, dy);

  /* ================= 2. 超过作用范围直接返回 ================= */
  if (dist > 1.2) {
    return;
  }

  /* ================= 3. 读取基础参数 ================= */
  auto getParam = parameters_handler_->getParamGetter(name_);

  double base_vx_std = 0.20;
  double base_wz_std = 0.40;
  double base_temp   = 0.30;

  double base_vx_max = 0.50;
  double base_wz_max = 0.50;
  double base_vx_min = -0.50;

  getParam(base_vx_std, "vx_std", base_vx_std);
  getParam(base_wz_std, "wz_std", base_wz_std);
  getParam(base_temp,   "temperature", base_temp);

  getParam(base_vx_max, "vx_max", base_vx_max);
  getParam(base_wz_max, "wz_max", base_wz_max);
  getParam(base_vx_min, "vx_min", base_vx_min);

  /* ================= 4. 分段目标参数（关键） ================= */
  // double target_vx_std;
  // double target_wz_std;
  // double target_temp;

  double scale = std::clamp(0.1 + 0.5 * dist, 0.1, 0.7);

  /* ============ 5. 目标参数 ============ */
  double target_vx_std;
  double target_wz_std;
  double target_temp;

  if (dist > adapt_dist_mid_) {
    target_vx_std = settings_.sampling_std.vx * scale;
    target_wz_std = settings_.sampling_std.wz * scale;
    target_temp   = settings_.temperature     * scale;

  } else if (dist > adapt_dist_near_) {
    target_vx_std = target_vx_std_mid_;
    target_wz_std = target_wz_std_mid_;
    target_temp   = target_temp_mid_;

  } else {
    target_vx_std = target_vx_std_near_;
    target_wz_std = target_wz_std_near_;
    target_temp   = target_temp_near_;
  }

  /* ============ 6. 安全下限 ============ */
  target_vx_std = std::max(min_vx_std_, target_vx_std);
  target_wz_std = std::max(min_wz_std_, target_wz_std);
  target_temp   = std::max(min_temperature_, target_temp);


  // if (dist > 0.5) {
  //   // 中距离：正常 MPPI
  //   target_vx_std = base_vx_std * scale;
  //   target_wz_std = base_wz_std * scale;
  //   target_temp   = base_temp   * scale;

  // } else if (dist > 0.15) {
  //   // 接近目标：明显抑制探索
  //   target_vx_std = 0.08;
  //   target_wz_std = 0.15;
  //   target_temp   = 0.18;

  // }
  //  else {
  //   // 最后 15cm：准确定性
  //   target_vx_std = 0.08;
  //   target_wz_std = 0.1;
  //   target_temp   = 0.18;
  // }

  /* ============ 5. 速度约束目标 ============ */
  const double target_vx_max =
    std::max(0.08, settings_.base_constraints.vx_max * scale);
  const double target_wz_max =
    std::max(0.15, settings_.base_constraints.wz * scale);
  const double target_vx_min =
    std::min(-0.08, settings_.base_constraints.vx_min * scale);

  /* ================= 6. 调节率 ================= */
  // constexpr double alpha_std  = 0.5;   // sampling std
  // constexpr double alpha_temp = 0.2;   // temperature
  // constexpr double alpha_vel  = 0.5;   // constraints

  /* ================= 7. 滤波器 ================= */
  auto smooth = [](double cur, double tgt, double a) {
    return cur + a * (tgt - cur);
  };

  /* ============ 6. 平滑更新 ============ */
  settings_.sampling_std.vx =
    smooth(settings_.sampling_std.vx, target_vx_std, alpha_std_);
  settings_.sampling_std.wz =
    smooth(settings_.sampling_std.wz, target_wz_std, alpha_std_);

  settings_.temperature =
    smooth(settings_.temperature, target_temp, alpha_temp_);

  settings_.base_constraints.vx_max =
    smooth(settings_.base_constraints.vx_max, target_vx_max, alpha_vel_);
  settings_.base_constraints.wz =
    smooth(settings_.base_constraints.wz, target_wz_max, alpha_vel_);
  settings_.base_constraints.vx_min =
    smooth(settings_.base_constraints.vx_min, target_vx_min, alpha_vel_);

  /* ================= 8. 超近距离确定性兜底 ================= */
  // if (dist < 0.08) {
  //   settings_.sampling_std.vx = 0.01;
  //   settings_.sampling_std.wz = 0.01;
  //   settings_.temperature    = 0.10;
  // }
}

// void Optimizer::updateParameters(
//   const geometry_msgs::msg::PoseStamped & robot_pose, const nav_msgs::msg::Path & plan)
// {
//   if (plan.poses.empty()) {
//     return;
//   }

//   const auto & goal_pose = plan.poses.back();
//   const double dx = robot_pose.pose.position.x - goal_pose.pose.position.x;
//   const double dy = robot_pose.pose.position.y - goal_pose.pose.position.y;
//   const double distance_to_goal = std::sqrt(dx * dx + dy * dy);

//   double scale = 1.0;
//   if (distance_to_goal < 1.2) {
//     scale = 0.3 + 0.5 * distance_to_goal;
//   }
//   else{
//     return;
//   }
//   auto getParam = parameters_handler_->getParamGetter(name_);

//   double base_vx_std = 0.2, base_wz_std = 0.4, base_temp = 0.3;
//   double base_vx_max = 0.5, base_wz_max = 0.5;
//   double base_vx_min = -0.5;
//   getParam(base_vx_std, "vx_std", base_vx_std);
//   getParam(base_wz_std, "wz_std", base_wz_std);
//   getParam(base_temp, "temperature", base_temp);
  
//   getParam(base_vx_max, "vx_max", base_vx_max);
//   getParam(base_wz_max, "wz_max", base_wz_max);

//   getParam(base_vx_min, "vx_min", base_vx_min);
//   settings_.sampling_std.vx = std::max(0.1, base_vx_std * scale);
//   settings_.sampling_std.wz = std::max(0.05, base_wz_std * scale);
//   settings_.temperature = std::max(0.15, base_temp * scale);

//   // Also scale down max velocities
//   settings_.base_constraints.vx_max = std::max(0.1, base_vx_max * scale);
//   settings_.base_constraints.wz = std::max(0.15, base_wz_max * scale);

//   settings_.base_constraints.vx_min = std::min(-0.1, base_vx_min * scale);
// }


void Optimizer::optimize()
{
  for (size_t i = 0; i < settings_.iteration_count; ++i) {
    generateNoisedTrajectories();
    critic_manager_.evalTrajectoriesScores(critics_data_);
    updateControlSequence();
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
    throw std::runtime_error("Optimizer fail to compute path");
  }

  return true;
}

void Optimizer::prepare(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan, nav2_core::GoalChecker * goal_checker)
{
  state_.pose = robot_pose;
  state_.speed = robot_speed;
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
  using namespace xt::placeholders;  // NOLINT
  control_sequence_.vx = xt::roll(control_sequence_.vx, -1);
  control_sequence_.wz = xt::roll(control_sequence_.wz, -1);


  xt::view(control_sequence_.vx, -1) =
    xt::view(control_sequence_.vx, -2);

  xt::view(control_sequence_.wz, -1) =
    xt::view(control_sequence_.wz, -2);


  if (isHolonomic()) {
    control_sequence_.vy = xt::roll(control_sequence_.vy, -1);
    xt::view(control_sequence_.vy, -1) =
      xt::view(control_sequence_.vy, -2);
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

  if (isHolonomic()) {
    control_sequence_.vy = xt::clip(control_sequence_.vy, -s.constraints.vy, s.constraints.vy);
  }

  control_sequence_.vx = xt::clip(control_sequence_.vx, s.constraints.vx_min, s.constraints.vx_max);
  control_sequence_.wz = xt::clip(control_sequence_.wz, -s.constraints.wz, s.constraints.wz);

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
  xt::noalias(xt::view(state.vx, xt::all(), 0)) = state.speed.linear.x;
  xt::noalias(xt::view(state.wz, xt::all(), 0)) = state.speed.angular.z;

  if (isHolonomic()) {
    xt::noalias(xt::view(state.vy, xt::all(), 0)) = state.speed.linear.y;
  }
}

void Optimizer::propagateStateVelocitiesFromInitials(
  models::State & state) const
{
  motion_model_->predict(state);
}

void Optimizer::integrateStateVelocities(
  xt::xtensor<float, 2> & trajectory,
  const xt::xtensor<float, 2> & sequence) const
{
  float initial_yaw = tf2::getYaw(state_.pose.pose.orientation);

  const auto vx = xt::view(sequence, xt::all(), 0);
  const auto vy = xt::view(sequence, xt::all(), 2);
  const auto wz = xt::view(sequence, xt::all(), 1);

  auto traj_x = xt::view(trajectory, xt::all(), 0);
  auto traj_y = xt::view(trajectory, xt::all(), 1);
  auto traj_yaws = xt::view(trajectory, xt::all(), 2);

  xt::noalias(traj_yaws) = xt::cumsum(wz * settings_.model_dt, 0) + initial_yaw;

  auto && yaw_cos = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());
  auto && yaw_sin = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());

  const auto yaw_offseted = xt::view(traj_yaws, xt::range(1, _));

  xt::noalias(xt::view(yaw_cos, 0)) = cosf(initial_yaw);
  xt::noalias(xt::view(yaw_sin, 0)) = sinf(initial_yaw);
  xt::noalias(xt::view(yaw_cos, xt::range(1, _))) = xt::cos(yaw_offseted);
  xt::noalias(xt::view(yaw_sin, xt::range(1, _))) = xt::sin(yaw_offseted);

  auto && dx = xt::eval(vx * yaw_cos);
  auto && dy = xt::eval(vx * yaw_sin);

  if (isHolonomic()) {
    dx = dx - vy * yaw_sin;
    dy = dy + vy * yaw_cos;
  }

  xt::noalias(traj_x) = state_.pose.pose.position.x + xt::cumsum(dx * settings_.model_dt, 0);
  xt::noalias(traj_y) = state_.pose.pose.position.y + xt::cumsum(dy * settings_.model_dt, 0);
}

void Optimizer::integrateStateVelocities(
  models::Trajectories & trajectories,
  const models::State & state) const
{
  const float initial_yaw = tf2::getYaw(state.pose.pose.orientation);

  xt::noalias(trajectories.yaws) =
    xt::cumsum(state.wz * settings_.model_dt, 1) + initial_yaw;

  const auto yaws_cutted = xt::view(trajectories.yaws, xt::all(), xt::range(0, -1));

  auto && yaw_cos = xt::xtensor<float, 2>::from_shape(trajectories.yaws.shape());
  auto && yaw_sin = xt::xtensor<float, 2>::from_shape(trajectories.yaws.shape());
  xt::noalias(xt::view(yaw_cos, xt::all(), 0)) = cosf(initial_yaw);
  xt::noalias(xt::view(yaw_sin, xt::all(), 0)) = sinf(initial_yaw);
  xt::noalias(xt::view(yaw_cos, xt::all(), xt::range(1, _))) = xt::cos(yaws_cutted);
  xt::noalias(xt::view(yaw_sin, xt::all(), xt::range(1, _))) = xt::sin(yaws_cutted);

  auto && dx = xt::eval(state.vx * yaw_cos);
  auto && dy = xt::eval(state.vx * yaw_sin);

  if (isHolonomic()) {
    dx = dx - state.vy * yaw_sin;
    dy = dy + state.vy * yaw_cos;
  }

  xt::noalias(trajectories.x) = state.pose.pose.position.x +
    xt::cumsum(dx * settings_.model_dt, 1);
  xt::noalias(trajectories.y) = state.pose.pose.position.y +
    xt::cumsum(dy * settings_.model_dt, 1);
}

xt::xtensor<float, 2> Optimizer::getOptimizedTrajectory()
{
  auto && sequence =
    xt::xtensor<float, 2>::from_shape({settings_.time_steps, isHolonomic() ? 3u : 2u});
  auto && trajectories = xt::xtensor<float, 2>::from_shape({settings_.time_steps, 3});

  xt::noalias(xt::view(sequence, xt::all(), 0)) = control_sequence_.vx;
  xt::noalias(xt::view(sequence, xt::all(), 1)) = control_sequence_.wz;

  if (isHolonomic()) {
    xt::noalias(xt::view(sequence, xt::all(), 2)) = control_sequence_.vy;
  }

  integrateStateVelocities(trajectories, sequence);
  return std::move(trajectories);
}

void Optimizer::updateControlSequence()
{
  auto & s = settings_;
  auto bounded_noises_vx = state_.cvx - control_sequence_.vx;
  auto bounded_noises_wz = state_.cwz - control_sequence_.wz;
  xt::noalias(costs_) +=
    s.gamma / powf(s.sampling_std.vx, 2) * xt::sum(
    xt::view(control_sequence_.vx, xt::newaxis(), xt::all()) * bounded_noises_vx, 1, immediate);
  xt::noalias(costs_) +=
    s.gamma / powf(s.sampling_std.wz, 2) * xt::sum(
    xt::view(control_sequence_.wz, xt::newaxis(), xt::all()) * bounded_noises_wz, 1, immediate);

  if (isHolonomic()) {
    auto bounded_noises_vy = state_.cvy - control_sequence_.vy;
    xt::noalias(costs_) +=
      s.gamma / powf(s.sampling_std.vy, 2) * xt::sum(
      xt::view(control_sequence_.vy, xt::newaxis(), xt::all()) * bounded_noises_vy,
      1, immediate);
  }

  auto && costs_normalized = costs_ - xt::amin(costs_, immediate);
  auto && exponents = xt::eval(xt::exp(-1 / settings_.temperature * costs_normalized));
  auto && softmaxes = xt::eval(exponents / xt::sum(exponents, immediate));
  auto && softmaxes_extened = xt::eval(xt::view(softmaxes, xt::all(), xt::newaxis()));

  xt::noalias(control_sequence_.vx) = xt::sum(state_.cvx * softmaxes_extened, 0, immediate);
  xt::noalias(control_sequence_.wz) = xt::sum(state_.cwz * softmaxes_extened, 0, immediate);
  if (isHolonomic()) {
    xt::noalias(control_sequence_.vy) = xt::sum(state_.cvy * softmaxes_extened, 0, immediate);
  }

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
  auto & s = settings_;
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    s.constraints.vx_max = s.base_constraints.vx_max;
    s.constraints.vx_min = s.base_constraints.vx_min;
    s.constraints.vy = s.base_constraints.vy;
    s.constraints.wz = s.base_constraints.wz;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      double ratio = speed_limit / 100.0;
      s.constraints.vx_max = s.base_constraints.vx_max * ratio;
      s.constraints.vx_min = s.base_constraints.vx_min * ratio;
      s.constraints.vy = s.base_constraints.vy * ratio;
      s.constraints.wz = s.base_constraints.wz * ratio;
    } else {
      // Speed limit is expressed in absolute value
      double ratio = speed_limit / s.base_constraints.vx_max;
      s.constraints.vx_max = s.base_constraints.vx_max * ratio;
      s.constraints.vx_min = s.base_constraints.vx_min * ratio;
      s.constraints.vy = s.base_constraints.vy * ratio;
      s.constraints.wz = s.base_constraints.wz * ratio;
    }
  }
}

models::Trajectories & Optimizer::getGeneratedTrajectories()
{
  return generated_trajectories_;
}

}  // namespace mppi
