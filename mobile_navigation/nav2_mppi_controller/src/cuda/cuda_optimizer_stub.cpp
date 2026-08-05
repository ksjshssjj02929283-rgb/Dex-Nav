// Copyright (c) 2026 Mobile Navigation Team
// CUDA Accelerated Optimizer Fallback Stub (Pure C++ / CUDA Only)

#include "nav2_mppi_controller/cuda/cuda_optimizer.hpp"
#include <iostream>

namespace mppi::cuda
{

#ifndef MPPI_CUDA_ENABLED

CudaOptimizer::CudaOptimizer() = default;
CudaOptimizer::~CudaOptimizer() = default;

bool CudaOptimizer::initialize(size_t batch_size, size_t time_steps, float model_dt, bool is_holonomic)
{
  batch_size_ = batch_size;
  time_steps_ = time_steps;
  model_dt_ = model_dt;
  is_holonomic_ = is_holonomic;
  cuda_initialized_ = false;

  std::cout << "[MPPI CUDA] Pure CUDA mode active. NVCC compiler / CUDA device required for GPU acceleration." << std::endl;
  return false;
}

void CudaOptimizer::shutdown()
{
  cuda_initialized_ = false;
}

bool CudaOptimizer::synchronizeStream()
{
  return true;
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
  (void)init_state;
  (void)control_vx;
  (void)control_vy;
  (void)control_wz;
  (void)constraints;
  (void)sampling_std;
  (void)host_traj_x;
  (void)host_traj_y;
  (void)host_traj_yaws;
  (void)host_cvx;
  (void)host_cvy;
  (void)host_cwz;
  return false;
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
  (void)costmap_data;
  (void)size_x;
  (void)size_y;
  (void)resolution;
  (void)origin_x;
  (void)origin_y;
  (void)inscribed_radius;
  (void)inflation_scale_factor;
  (void)inflation_radius;
  (void)collision_margin_distance;
  (void)collision_cost;
  (void)critical_weight;
  (void)repulsion_weight;
  (void)near_goal;
  (void)unknown_is_collision;
  (void)power;
  (void)host_costs;
  return false;
}

bool CudaOptimizer::evaluatePathAngleCosts(
  float goal_x, float goal_y,
  bool reversing_allowed, bool forward_preference,
  float direction_deadband, float wrong_way_cost_ratio,
  float current_x, float current_y, float current_yaw, float current_wz,
  float weight, unsigned int power,
  std::vector<float> & host_costs)
{
  (void)goal_x;
  (void)goal_y;
  (void)reversing_allowed;
  (void)forward_preference;
  (void)direction_deadband;
  (void)wrong_way_cost_ratio;
  (void)current_x;
  (void)current_y;
  (void)current_yaw;
  (void)current_wz;
  (void)weight;
  (void)power;
  (void)host_costs;
  return false;
}

#endif  // MPPI_CUDA_ENABLED

}  // namespace mppi::cuda
