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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__NOISE_GENERATOR_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__NOISE_GENERATOR_HPP_

#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "nav2_mppi_controller/models/optimizer_settings.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/state.hpp"

namespace mppi
{

/**
 * @class mppi::NoiseGenerator
 * @brief Generates noise trajectories from optimal trajectory
 */
class NoiseGenerator
{
public:
  /**
    * @brief Constructor for mppi::NoiseGenerator
    */
  NoiseGenerator() = default;

  /**
   * @brief Initialize noise generator with settings and model types
   * @param settings Settings of controller
   * @param is_holonomic If base is holonomic
   * @param name Namespace for configs
   * @param param_handler Get parameters util
   */
  void initialize(
    mppi::models::OptimizerSettings & settings,
    bool is_holonomic, const std::string & name, ParametersHandler * param_handler);

  /**
   * @brief Shutdown noise generator thread
   */
  void shutdown();

  /**
   * @brief Signal to the noise thread the controller is ready to generate a new
   * noised control for the next iteration
   */
  void generateNextNoises();

  /**
   * @brief set noised control_sequence to state controls
   * @return noises vx, vy, wz
   */
  void setNoisedControls(models::State & state, const models::ControlSequence & control_sequence);

  /**
   * @brief Reset noise generator with settings and model types
   * @param settings Settings of controller
   * @param is_holonomic If base is holonomic
   */
  void reset(mppi::models::OptimizerSettings & settings, bool is_holonomic);

  /**
   * @brief 二期条件层专用：只更新采样噪声标准差，不触发 noises_* 矩阵重分配。
   * 仅在 regenerate_noises=true（后台线程持续重采样）时才会被下一轮 generateNoisedControls()
   * 捡到；regenerate_noises=false 时噪声矩阵只在 initialize()/reset() 生成一次，本 setter
   * 更新的值要等到下次 reset() 才会体现在实际采样里。
   * @param sampling_std 新的 vx/vy/wz 采样标准差
   */
  void updateSamplingStd(const mppi::models::SamplingStd & sampling_std);

protected:
  /**
   * @brief Thread to execute noise generation process
   */
  void noiseThread();

  /**
   * @brief Generate random controls by gaussian noise with mean in
   * control_sequence_
   *
   * @return tensor of shape [ batch_size_, time_steps_, 2]
   * where 2 stands for v, w
   */
  void generateNoisedControls();

  models::Matrix noises_vx_;
  models::Matrix noises_vy_;
  models::Matrix noises_wz_;

  mppi::models::OptimizerSettings settings_;
  bool is_holonomic_;

  std::thread noise_thread_;
  std::condition_variable noise_cond_;
  std::mutex noise_lock_;
  bool active_{false}, ready_{false}, regenerate_noises_{false};
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__NOISE_GENERATOR_HPP_
