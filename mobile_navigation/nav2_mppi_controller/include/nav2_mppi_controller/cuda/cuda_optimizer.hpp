// Copyright (c) 2026 Mobile Navigation Team
// CUDA Accelerated Optimization Core for MPPI Controller

#ifndef NAV2_MPPI_CONTROLLER__CUDA__CUDA_OPTIMIZER_HPP_
#define NAV2_MPPI_CONTROLLER__CUDA__CUDA_OPTIMIZER_HPP_

#include <vector>
#include <memory>
#include <cstddef>
#include "nav2_mppi_controller/models/constraints.hpp"
#include "nav2_mppi_controller/cuda/cuda_kernels.h"

namespace mppi::cuda
{

/**
 * @class CudaOptimizer
 * @brief CUDA accelerated trajectory rollout and costmap evaluation backend for MPPI
 */
class CudaOptimizer
{
public:
  CudaOptimizer();
  ~CudaOptimizer();

  /**
   * @brief Initialize CUDA memory and random number generators
   */
  bool initialize(size_t batch_size, size_t time_steps, float model_dt, bool is_holonomic);

  /**
   * @brief Release GPU resources
   */
  void shutdown();

  /**
   * @brief Generate noised trajectories and roll out state velocities on GPU
   */
  bool generateNoisedTrajectories(
    const CudaState & init_state,
    const std::vector<float> & control_vx,
    const std::vector<float> & control_vy,
    const std::vector<float> & control_wz,
    const models::ControlConstraints & constraints,
    const models::SamplingStd & sampling_std,
    float * host_traj_x,
    float * host_traj_y,
    float * host_traj_yaws,
    float * host_cvx = nullptr,
    float * host_cvy = nullptr,
    float * host_cwz = nullptr,
    float goal_distance = -1.0f);

  /**
   * @brief Evaluate obstacle costs using CUDA parallel grid/footprint collision check
   */
  bool evaluateObstacleCosts(
    const unsigned char * costmap_data,
    unsigned int size_x,
    unsigned int size_y,
    float resolution,
    float origin_x,
    float origin_y,
    float circumscribed_cost,
    float inflation_scale_factor,
    float inflation_radius,
    float critical_weight,
    float repulsion_weight,
    bool near_goal,
    unsigned int power,
    std::vector<float> & host_costs);

  /**
   * @brief Evaluate path angle costs using CUDA
   */
  bool evaluatePathAngleCosts(
    float goal_x, float goal_y,
    bool reversing_allowed, bool forward_preference,
    float direction_deadband, float wrong_way_cost_ratio,
    float current_x, float current_y, float current_yaw, float current_wz,
    float weight, unsigned int power,
    std::vector<float> & host_costs);

  /**
   * @brief Synchronize CUDA Stream
   */
  bool synchronizeStream();

  /**
   * @brief Check if CUDA device is active and available
   */
  bool isCudaAvailable() const { return cuda_initialized_; }

private:
  bool cuda_initialized_{false};
  const unsigned char * last_costmap_ptr_{nullptr};
  size_t batch_size_{0};
  size_t time_steps_{0};
  float model_dt_{0.05f};
  bool is_holonomic_{false};

  // GPU Memory Pointers & CUDA Stream
  void * d_noises_{nullptr};
  void * d_traj_x_{nullptr};
  void * d_traj_y_{nullptr};
  void * d_traj_yaws_{nullptr};
  void * d_costmap_{nullptr};
  void * d_costs_{nullptr};
  void * d_cvx_{nullptr};
  void * d_cvy_{nullptr};
  void * d_cwz_{nullptr};
  void * d_ctrl_vx_{nullptr};
  void * d_ctrl_vy_{nullptr};
  void * d_ctrl_wz_{nullptr};
  void * curand_states_{nullptr};
  void * stream_{nullptr};
};

}  // namespace mppi::cuda

#endif  // NAV2_MPPI_CONTROLLER__CUDA__CUDA_OPTIMIZER_HPP_
