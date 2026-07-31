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

#ifndef NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_
#define NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_

#include <cstdint>
#include <algorithm>
#include <cmath>

#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/constraints.hpp"

#include "nav2_mppi_controller/tools/parameters_handler.hpp"

namespace mppi
{

/**
 * @class mppi::MotionModel
 * @brief Abstract motion model for modeling a vehicle
 */
class MotionModel
{
public:
  /**
    * @brief Constructor for mppi::MotionModel
    */
  MotionModel() = default;

  /**
    * @brief Destructor for mppi::MotionModel
    */
  virtual ~MotionModel() = default;

  /**
    * @brief 初始化运动模型，设置约束参数和时间步长
    * @param control_constraints 控制约束（含加速度限制）
    * @param model_dt 单步时间长度
    */
  void initialize(const models::ControlConstraints & control_constraints, float model_dt)
  {
    control_constraints_ = control_constraints;
    model_dt_ = model_dt;
  }

  /**
   * @brief With input velocities, find the vehicle's output velocities
   * @param state Contains control velocities to use to populate vehicle velocities
   *
   * 在原始实现基础上增加加速度约束：
   *   每个时间步的速度变化量不超过 ax_max/ax_min * dt（vx）和 az_max * dt（wz）。
   *   ax_min 为负值，用于非对称减速场景（例如前进减速比倒退减速更激进）。
   */
  virtual void predict(models::State & state)
  {
    const bool is_holo = isHolonomic();
    const float max_delta_vx = model_dt_ * control_constraints_.ax_max;
    const float min_delta_vx = model_dt_ * control_constraints_.ax_min;
    const float max_delta_wz = model_dt_ * control_constraints_.az_max;
    const float max_delta_ax = model_dt_ * control_constraints_.jx_max;
    const float max_delta_wz_acc = model_dt_ * control_constraints_.jz_max;

    const unsigned int n_cols = static_cast<unsigned int>(state.vx.cols());
    if (n_cols < 2) {
      return;
    }

    const Eigen::Index batch_size = state.vx.rows();
    resizeScratchBuffers(batch_size, is_holo);

    // 第一个控制步只施加速度/加速度约束，作为后续 jerk 约束的起点
    auto v_init = state.vx.col(0);
    auto cv_init = state.cvx.col(0);
    auto v_first = state.vx.col(1);
    lower_vx_ =
      (v_init > 0.0f).select(v_init + min_delta_vx, v_init - max_delta_vx);
    upper_vx_ =
      (v_init > 0.0f).select(v_init + max_delta_vx, v_init - min_delta_vx);
    cv_init = cv_init.min(upper_vx_).max(lower_vx_);
    v_first = cv_init;
    vx_acc_prev_ = ((v_first - v_init) / model_dt_).min(
      control_constraints_.ax_max).max(control_constraints_.ax_min);

    auto w_init = state.wz.col(0);
    auto cw_init = state.cwz.col(0);
    auto w_first = state.wz.col(1);
    cw_init = cw_init.min(w_init + max_delta_wz).max(w_init - max_delta_wz);
    w_first = cw_init;
    wz_acc_prev_ = ((w_first - w_init) / model_dt_).min(
      control_constraints_.az_max).max(-control_constraints_.az_max);

    if (is_holo) {
      const float max_delta_vy = model_dt_ * control_constraints_.ay_max;
      const float min_delta_vy = model_dt_ * control_constraints_.ay_min;

      auto vy_init = state.vy.col(0);
      auto cvy_init = state.cvy.col(0);
      auto vy_first = state.vy.col(1);

      lower_vy_ =
        (vy_init > 0.0f).select(vy_init + min_delta_vy, vy_init - max_delta_vy);
      upper_vy_ =
        (vy_init > 0.0f).select(vy_init + max_delta_vy, vy_init - min_delta_vy);
      cvy_init = cvy_init.min(upper_vy_).max(lower_vy_);
      vy_first = cvy_init;
      vy_acc_prev_ = ((vy_first - vy_init) / model_dt_).min(
        control_constraints_.ay_max).max(control_constraints_.ay_min);
    }

    for (unsigned int i = 2; i < n_cols; i++) {
      // 线加速度约束
      auto v_prev = state.vx.col(i - 1);
      auto cv = state.cvx.col(i - 1);
      auto v_next = state.vx.col(i);

      lower_vx_ =
        (v_prev > 0.0f).select(v_prev + min_delta_vx, v_prev - max_delta_vx);
      upper_vx_ =
        (v_prev > 0.0f).select(v_prev + max_delta_vx, v_prev - min_delta_vx);
      cv = cv.min(upper_vx_).max(lower_vx_);
      vx_acc_ = ((cv - v_prev) / model_dt_).min(
        vx_acc_prev_ + max_delta_ax).max(vx_acc_prev_ - max_delta_ax);
      vx_acc_ = vx_acc_.min(control_constraints_.ax_max).max(control_constraints_.ax_min);
      v_next = v_prev + vx_acc_ * model_dt_;
      cv = v_next;
      vx_acc_prev_ = vx_acc_;

      auto w_prev = state.wz.col(i - 1);
      auto cw = state.cwz.col(i - 1);
      auto w_next = state.wz.col(i);
      cw = cw.min(w_prev + max_delta_wz).max(w_prev - max_delta_wz);
      wz_acc_ = ((cw - w_prev) / model_dt_).min(
        wz_acc_prev_ + max_delta_wz_acc).max(wz_acc_prev_ - max_delta_wz_acc);
      wz_acc_ = wz_acc_.min(control_constraints_.az_max).max(-control_constraints_.az_max);
      w_next = w_prev + wz_acc_ * model_dt_;
      cw = w_next;
      wz_acc_prev_ = wz_acc_;

      if (is_holo) {
        const float max_delta_vy = model_dt_ * control_constraints_.ay_max;
        const float min_delta_vy = model_dt_ * control_constraints_.ay_min;

        auto vy_prev = state.vy.col(i - 1);
        auto cvy = state.cvy.col(i - 1);
        auto vy_next = state.vy.col(i);

        lower_vy_ =
          (vy_prev > 0.0f).select(vy_prev + min_delta_vy, vy_prev - max_delta_vy);
        upper_vy_ =
          (vy_prev > 0.0f).select(vy_prev + max_delta_vy, vy_prev - min_delta_vy);
        cvy = cvy.min(upper_vy_).max(lower_vy_);
        vy_acc_ = ((cvy - vy_prev) / model_dt_).min(
          vy_acc_prev_ + max_delta_ax).max(vy_acc_prev_ - max_delta_ax);
        vy_acc_ = vy_acc_.min(control_constraints_.ay_max).max(control_constraints_.ay_min);
        vy_next = vy_prev + vy_acc_ * model_dt_;
        cvy = vy_next;
        vy_acc_prev_ = vy_acc_;
      }
    }
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  virtual bool isHolonomic() = 0;

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  virtual void applyConstraints(models::ControlSequence & /*control_sequence*/) {}

protected:
  /**
   * @brief 按采样批次调整临时向量，仅在batch size变化时重新分配内存
   */
  void resizeScratchBuffers(Eigen::Index batch_size, bool is_holo)
  {
    if (vx_acc_prev_.size() != batch_size) {
      vx_acc_prev_.resize(batch_size);
      wz_acc_prev_.resize(batch_size);
      lower_vx_.resize(batch_size);
      upper_vx_.resize(batch_size);
      vx_acc_.resize(batch_size);
      wz_acc_.resize(batch_size);
    }

    if (is_holo && vy_acc_prev_.size() != batch_size) {
      vy_acc_prev_.resize(batch_size);
      lower_vy_.resize(batch_size);
      upper_vy_.resize(batch_size);
      vy_acc_.resize(batch_size);
    }
  }

  float model_dt_{0.0f};
  models::ControlConstraints control_constraints_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f};
  models::Vector vx_acc_prev_;
  models::Vector wz_acc_prev_;
  models::Vector vy_acc_prev_;
  models::Vector lower_vx_;
  models::Vector upper_vx_;
  models::Vector lower_vy_;
  models::Vector upper_vy_;
  models::Vector vx_acc_;
  models::Vector wz_acc_;
  models::Vector vy_acc_;
};

/**
 * @class mppi::AckermannMotionModel
 * @brief Ackermann motion model
 */
class AckermannMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::AckermannMotionModel
    */
  explicit AckermannMotionModel(ParametersHandler * param_handler)
  {
    auto getParam = param_handler->getParamGetter("AckermannConstraints");
    getParam(min_turning_r_, "min_turning_r", 0.2);
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return false;
  }

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  void applyConstraints(models::ControlSequence & control_sequence) override
  {
    auto & vx = control_sequence.vx;
    auto & wz = control_sequence.wz;

    const auto mask = (vx.abs() / wz.abs()) < min_turning_r_;
    const models::Vector constrained_wz = wz.unaryExpr(
      [](float value) {return value < 0.0f ? -1.0f : 1.0f;}) * vx / min_turning_r_;
    wz = mask.select(constrained_wz, wz);
  }

  /**
   * @brief Get minimum turning radius of ackermann drive
   * @return Minimum turning radius
   */
  float getMinTurningRadius() {return min_turning_r_;}

private:
  float min_turning_r_{0};
};

/**
 * @class mppi::DiffDriveMotionModel
 * @brief Differential drive motion model
 */
class DiffDriveMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::DiffDriveMotionModel
    */
  DiffDriveMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return false;
  }
};

/**
 * @class mppi::OmniMotionModel
 * @brief Omnidirectional motion model
 */
class OmniMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::OmniMotionModel
    */
  OmniMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return true;
  }
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_
