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

#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/constraints.hpp"
#include <xtensor/xmath.hpp>
#include <xtensor/xmasked_view.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xnoalias.hpp>

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
    using namespace xt::placeholders;  // NOLINT

    const bool is_holo = isHolonomic();
    const float max_delta_vx = model_dt_ * control_constraints_.ax_max;
    const float min_delta_vx = model_dt_ * control_constraints_.ax_min;
    const float max_delta_wz = model_dt_ * control_constraints_.az_max;
    const float max_delta_ax = model_dt_ * control_constraints_.jx_max;
    const float max_delta_wz_acc = model_dt_ * control_constraints_.jz_max;

    const unsigned int n_cols = static_cast<unsigned int>(state.vx.shape(1));
    if (n_cols < 2) {
      return;
    }

    const size_t batch_size = state.vx.shape(0);
    auto vx_acc_prev = xt::xtensor<float, 1>::from_shape({batch_size});
    auto wz_acc_prev = xt::xtensor<float, 1>::from_shape({batch_size});

    // 第一个控制步只施加速度/加速度约束，作为后续 jerk 约束的起点
    auto v_init = xt::view(state.vx, xt::all(), 0);
    auto cv_init = xt::view(state.cvx, xt::all(), 0);
    auto v_first = xt::view(state.vx, xt::all(), 1);
    auto lower_vx_init = xt::where(v_init > 0.0f, v_init + min_delta_vx, v_init - max_delta_vx);
    auto upper_vx_init = xt::where(v_init > 0.0f, v_init + max_delta_vx, v_init - min_delta_vx);
    xt::noalias(cv_init) = xt::clip(cv_init, lower_vx_init, upper_vx_init);
    xt::noalias(v_first) = cv_init;
    xt::noalias(vx_acc_prev) = xt::clip((v_first - v_init) / model_dt_,
      control_constraints_.ax_min, control_constraints_.ax_max);

    auto w_init = xt::view(state.wz, xt::all(), 0);
    auto cw_init = xt::view(state.cwz, xt::all(), 0);
    auto w_first = xt::view(state.wz, xt::all(), 1);
    xt::noalias(cw_init) = xt::clip(cw_init, w_init - max_delta_wz, w_init + max_delta_wz);
    xt::noalias(w_first) = cw_init;
    xt::noalias(wz_acc_prev) = xt::clip((w_first - w_init) / model_dt_,
      -control_constraints_.az_max, control_constraints_.az_max);

    xt::xtensor<float, 1> vy_acc_prev;
    if (is_holo) {
      const float max_delta_vy = model_dt_ * control_constraints_.ay_max;
      const float min_delta_vy = model_dt_ * control_constraints_.ay_min;

      vy_acc_prev = xt::xtensor<float, 1>::from_shape({batch_size});
      auto vy_init = xt::view(state.vy, xt::all(), 0);
      auto cvy_init = xt::view(state.cvy, xt::all(), 0);
      auto vy_first = xt::view(state.vy, xt::all(), 1);

      auto lower_vy_init = xt::where(vy_init > 0.0f, vy_init + min_delta_vy, vy_init - max_delta_vy);
      auto upper_vy_init = xt::where(vy_init > 0.0f, vy_init + max_delta_vy, vy_init - min_delta_vy);
      xt::noalias(cvy_init) = xt::clip(cvy_init, lower_vy_init, upper_vy_init);
      xt::noalias(vy_first) = cvy_init;
      xt::noalias(vy_acc_prev) = xt::clip((vy_first - vy_init) / model_dt_,
        control_constraints_.ay_min, control_constraints_.ay_max);
    }

    for (unsigned int i = 2; i < n_cols; i++) {
      // 线加速度约束
      auto v_prev = xt::view(state.vx, xt::all(), i - 1);
      auto cv     = xt::view(state.cvx, xt::all(), i - 1);
      auto v_next = xt::view(state.vx, xt::all(), i);

      auto lower_vx = xt::where(v_prev > 0.0f, v_prev + min_delta_vx, v_prev - max_delta_vx);
      auto upper_vx = xt::where(v_prev > 0.0f, v_prev + max_delta_vx, v_prev - min_delta_vx);
      xt::noalias(cv) = xt::clip(cv, lower_vx, upper_vx);
      auto vx_acc = xt::eval(xt::clip((cv - v_prev) / model_dt_,
        vx_acc_prev - max_delta_ax, vx_acc_prev + max_delta_ax));
      vx_acc = xt::eval(xt::clip(vx_acc, control_constraints_.ax_min, control_constraints_.ax_max));
      xt::noalias(v_next) = v_prev + vx_acc * model_dt_;
      xt::noalias(cv) = v_next;
      xt::noalias(vx_acc_prev) = vx_acc;

      auto w_prev = xt::view(state.wz, xt::all(), i - 1);
      auto cw     = xt::view(state.cwz, xt::all(), i - 1);
      auto w_next = xt::view(state.wz, xt::all(), i);
      xt::noalias(cw) = xt::clip(cw, w_prev - max_delta_wz, w_prev + max_delta_wz);
      auto wz_acc = xt::eval(xt::clip((cw - w_prev) / model_dt_,
        wz_acc_prev - max_delta_wz_acc, wz_acc_prev + max_delta_wz_acc));
      wz_acc = xt::eval(xt::clip(wz_acc, -control_constraints_.az_max, control_constraints_.az_max));
      xt::noalias(w_next) = w_prev + wz_acc * model_dt_;
      xt::noalias(cw) = w_next;
      xt::noalias(wz_acc_prev) = wz_acc;

      if (is_holo) {
        const float max_delta_vy = model_dt_ * control_constraints_.ay_max;
        const float min_delta_vy = model_dt_ * control_constraints_.ay_min;

        auto vy_prev = xt::view(state.vy, xt::all(), i - 1);
        auto cvy     = xt::view(state.cvy, xt::all(), i - 1);
        auto vy_next = xt::view(state.vy, xt::all(), i);

        auto lower_vy = xt::where(vy_prev > 0.0f, vy_prev + min_delta_vy, vy_prev - max_delta_vy);
        auto upper_vy = xt::where(vy_prev > 0.0f, vy_prev + max_delta_vy, vy_prev - min_delta_vy);
        xt::noalias(cvy) = xt::clip(cvy, lower_vy, upper_vy);
        auto vy_acc = xt::eval(xt::clip((cvy - vy_prev) / model_dt_,
          vy_acc_prev - max_delta_ax, vy_acc_prev + max_delta_ax));
        vy_acc = xt::eval(xt::clip(vy_acc, control_constraints_.ay_min, control_constraints_.ay_max));
        xt::noalias(vy_next) = vy_prev + vy_acc * model_dt_;
        xt::noalias(cvy) = vy_next;
        xt::noalias(vy_acc_prev) = vy_acc;
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
  float model_dt_{0.0f};
  models::ControlConstraints control_constraints_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f};
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

    auto view = xt::masked_view(wz, xt::fabs(vx) / xt::fabs(wz) < min_turning_r_);
    view = xt::sign(wz) * vx / min_turning_r_;
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
