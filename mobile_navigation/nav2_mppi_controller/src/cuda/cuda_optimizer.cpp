// Copyright (c) 2026 Mobile Navigation Team
// C++ Host Wrapper for CUDA Accelerated MPPI

#include "nav2_mppi_controller/cuda/cuda_optimizer.hpp"
#include "nav2_mppi_controller/cuda/cuda_kernels.h"
#include <iostream>

namespace mppi::cuda
{

#ifdef MPPI_CUDA_ENABLED

CudaOptimizer::CudaOptimizer() = default;

CudaOptimizer::~CudaOptimizer()
{
  shutdown();
}

bool CudaOptimizer::initialize(size_t batch_size, size_t time_steps, float model_dt, bool is_holonomic)
{
  batch_size_ = batch_size;
  time_steps_ = time_steps;
  model_dt_ = model_dt;
  is_holonomic_ = is_holonomic;

  cuda_initialized_ = cuda_init_device(
    batch_size_, time_steps_,
    &curand_states_, &d_noises_, &d_traj_x_, &d_traj_y_, &d_traj_yaws_, &d_costs_,
    &d_cvx_, &d_cvy_, &d_cwz_,
    &d_ctrl_vx_, &d_ctrl_vy_, &d_ctrl_wz_, &stream_);

  return cuda_initialized_;
}

void CudaOptimizer::shutdown()
{
  if (cuda_initialized_) {
    cuda_free_device(
      curand_states_, d_noises_, d_traj_x_, d_traj_y_, d_traj_yaws_, d_costmap_, d_costs_,
      d_cvx_, d_cvy_, d_cwz_,
      d_ctrl_vx_, d_ctrl_vy_, d_ctrl_wz_, stream_);
    curand_states_ = d_noises_ = d_traj_x_ = d_traj_y_ = d_traj_yaws_ = d_costmap_ = d_costs_ = d_cvx_ = d_cvy_ = d_cwz_ = d_ctrl_vx_ = d_ctrl_vy_ = d_ctrl_wz_ = stream_ = nullptr;
    cuda_initialized_ = false;
  }
}

bool CudaOptimizer::synchronizeStream()
{
  if (!cuda_initialized_) return true;
  return cuda_synchronize_stream(stream_);
}

bool CudaOptimizer::generateNoisedTrajectories(
  const CudaState & init_state,
  const std::vector<float> & control_vx,
  const std::vector<float> & control_vy,
  const std::vector<float> & control_wz,
  const models::ControlConstraints & constraints,
  const models::SamplingStd & sampling_std,
  float * host_traj_x,
  float * host_traj_y,
  float * host_traj_yaws,
  float * host_cvx,
  float * host_cvy,
  float * host_cwz)
{
  bool res = cuda_generate_trajectories(
    batch_size_, time_steps_, model_dt_, is_holonomic_,
    init_state,
    control_vx.data(),
    control_vy.empty() ? nullptr : control_vy.data(),
    control_wz.data(),
    constraints, sampling_std,
    curand_states_, d_traj_x_, d_traj_y_, d_traj_yaws_,
    d_cvx_, d_cvy_, d_cwz_,
    d_ctrl_vx_, d_ctrl_vy_, d_ctrl_wz_,
    host_traj_x, host_traj_y, host_traj_yaws,
    host_cvx, host_cvy, host_cwz, stream_);

  // ⭐【轨迹数据屏障】确保轨迹从 GPU 回传到 CPU 内存后，CPU 侧的所有 Critic 才能读取计算
  synchronizeStream();
  return res;
}

bool CudaOptimizer::evaluateObstacleCosts(
  const unsigned char * costmap_data,
  unsigned int size_x,
  unsigned int size_y,
  float resolution,
  float origin_x,
  float origin_y,
  float inscribed_radius,
  float inflation_scale_factor,
  float inflation_radius,
  float collision_margin_distance,
  float collision_cost,
  float critical_weight,
  float repulsion_weight,
  bool near_goal,
  bool unknown_is_collision,
  unsigned int power,
  std::vector<float> & host_costs)
{
  if (!cuda_initialized_ || !costmap_data) return false;

  bool costmap_changed = (last_costmap_ptr_ != costmap_data);
  last_costmap_ptr_ = costmap_data;

  host_costs.resize(batch_size_);
  return cuda_evaluate_obstacles(
    batch_size_, time_steps_,
    d_traj_x_, d_traj_y_, &d_costmap_, d_costs_,
    costmap_data, costmap_changed, size_x, size_y, resolution, origin_x, origin_y,
    inscribed_radius, inflation_scale_factor, inflation_radius, collision_margin_distance,
    collision_cost, critical_weight, repulsion_weight, near_goal, unknown_is_collision, power,
    host_costs.data(), stream_);
}

bool CudaOptimizer::evaluatePathAngleCosts(
  float goal_x, float goal_y,
  bool reversing_allowed, bool forward_preference,
  float direction_deadband, float wrong_way_cost_ratio,
  float current_x, float current_y, float current_yaw, float current_wz,
  float weight, unsigned int power,
  std::vector<float> & host_costs)
{
  if (!cuda_initialized_) return false;

  host_costs.resize(batch_size_);
  return cuda_evaluate_path_angle(
    batch_size_, time_steps_,
    d_traj_x_, d_traj_y_, d_traj_yaws_, d_cwz_, d_costs_,
    goal_x, goal_y,
    reversing_allowed, forward_preference,
    direction_deadband, wrong_way_cost_ratio,
    current_x, current_y, current_yaw, current_wz,
    weight, power,
    host_costs.data(), stream_);
}

#endif  // MPPI_CUDA_ENABLED

}  // namespace mppi::cuda
