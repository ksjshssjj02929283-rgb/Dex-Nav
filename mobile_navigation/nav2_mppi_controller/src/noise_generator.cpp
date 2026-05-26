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

#include "nav2_mppi_controller/tools/noise_generator.hpp"

#include <memory>
#include <mutex>
#include <random>

namespace mppi
{

void NoiseGenerator::initialize(
  mppi::models::OptimizerSettings & settings, bool is_holonomic,
  const std::string & name, ParametersHandler * param_handler)
{
  settings_ = settings;
  is_holonomic_ = is_holonomic;
  active_ = true;

  auto getParam = param_handler->getParamGetter(name);
  getParam(regenerate_noises_, "regenerate_noises", false);

  if (regenerate_noises_) {
    noise_thread_ = std::thread(std::bind(&NoiseGenerator::noiseThread, this));
  } else {
    generateNoisedControls();
  }
}

void NoiseGenerator::shutdown()
{
  active_ = false;
  ready_ = true;
  noise_cond_.notify_all();
  if (noise_thread_.joinable()) {
    noise_thread_.join();
  }
}

void NoiseGenerator::generateNextNoises()
{
  // Trigger the thread to run in parallel to this iteration
  // to generate the next iteration's noises (if applicable).
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    ready_ = true;
  }
  noise_cond_.notify_all();
}

void NoiseGenerator::setNoisedControls(
  models::State & state,
  const models::ControlSequence & control_sequence)
{
  std::unique_lock<std::mutex> guard(noise_lock_);

  auto addControlSequence = [](models::Matrix & dst, const models::Matrix & noises,
      const models::Vector & sequence)
    {
      dst = noises;
      for (Eigen::Index i = 0; i < dst.rows(); ++i) {
        dst.row(i) += sequence.transpose();
      }
    };

  addControlSequence(state.cvx, noises_vx_, control_sequence.vx);
  addControlSequence(state.cvy, noises_vy_, control_sequence.vy);
  addControlSequence(state.cwz, noises_wz_, control_sequence.wz);
}

void NoiseGenerator::reset(mppi::models::OptimizerSettings & settings, bool is_holonomic)
{
  settings_ = settings;
  is_holonomic_ = is_holonomic;

  // Recompute the noises on reset, initialization, and fallback
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    noises_vx_ = models::Matrix::Zero(settings_.batch_size, settings_.time_steps);
    noises_vy_ = models::Matrix::Zero(settings_.batch_size, settings_.time_steps);
    noises_wz_ = models::Matrix::Zero(settings_.batch_size, settings_.time_steps);
    ready_ = true;
  }

  if (regenerate_noises_) {
    noise_cond_.notify_all();
  } else {
    generateNoisedControls();
  }
}

void NoiseGenerator::noiseThread()
{
  do {
    std::unique_lock<std::mutex> guard(noise_lock_);
    noise_cond_.wait(guard, [this]() {return ready_;});
    ready_ = false;
    generateNoisedControls();
  } while (active_);
}

void NoiseGenerator::generateNoisedControls()
{
  auto & s = settings_;

  static thread_local std::mt19937 generator{std::random_device{}()};
  auto fillNormal = [&](models::Matrix & noises, float stddev) {
      noises.resize(s.batch_size, s.time_steps);
      if (stddev <= 0.0f) {
        noises.setZero();
        return;
      }
      std::normal_distribution<float> distribution(0.0f, stddev);
      for (Eigen::Index r = 0; r < noises.rows(); ++r) {
        for (Eigen::Index c = 0; c < noises.cols(); ++c) {
          noises(r, c) = distribution(generator);
        }
      }
    };

  fillNormal(noises_vx_, s.sampling_std.vx);
  fillNormal(noises_wz_, s.sampling_std.wz);
  if (is_holonomic_) {
    fillNormal(noises_vy_, s.sampling_std.vy);
  }
}

}  // namespace mppi
