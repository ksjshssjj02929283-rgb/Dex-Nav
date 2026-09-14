// Copyright (c) 2026 Mobile Navigation Team
// Pure CUDA Kernel Launcher C-Interface (Zero C++ STL Header Pollution)

#ifndef NAV2_MPPI_CONTROLLER__CUDA__CUDA_KERNELS_H_
#define NAV2_MPPI_CONTROLLER__CUDA__CUDA_KERNELS_H_

#include <cstddef>
#include "nav2_mppi_controller/models/constraints.hpp"

namespace mppi::cuda
{

#ifdef MPPI_CUDA_TESTING
// Makes the zero-based allocation_index-th cudaMalloc in cuda_init_device() fail once.
// A negative value disables fault injection. Only available in BUILD_TESTING builds.
void cuda_set_allocation_failure_for_testing(int allocation_index);
#endif

struct CudaState
{
  float pose_x{0.0f};
  float pose_y{0.0f};
  float pose_yaw{0.0f};
  float speed_vx{0.0f};
  float speed_vy{0.0f};
  float speed_wz{0.0f};
};

bool cuda_init_device(
  size_t batch_size, size_t time_steps,
  void ** curand_states, void ** d_noises, void ** d_traj_x, void ** d_traj_y, void ** d_traj_yaws, void ** d_costs,
  void ** d_cvx, void ** d_cvy, void ** d_cwz,
  void ** d_ctrl_vx, void ** d_ctrl_vy, void ** d_ctrl_wz, void ** stream = nullptr);

void cuda_free_device(
  void * curand_states, void * d_noises, void * d_traj_x, void * d_traj_y, void * d_traj_yaws, void * d_costmap, void * d_costs,
  void * d_cvx, void * d_cvy, void * d_cwz,
  void * d_ctrl_vx, void * d_ctrl_vy, void * d_ctrl_wz, void * stream = nullptr);

bool cuda_synchronize_stream(void * stream);

bool cuda_generate_trajectories(
  size_t batch_size, size_t time_steps, float model_dt, bool is_holonomic,
  const CudaState & init_state,
  const float * control_vx, const float * control_vy, const float * control_wz,
  models::ControlConstraints constraints, models::SamplingStd sampling_std,
  void * curand_states, void * d_traj_x, void * d_traj_y, void * d_traj_yaws,
  void * d_cvx, void * d_cvy, void * d_cwz,
  void * d_ctrl_vx, void * d_ctrl_vy, void * d_ctrl_wz,
  float * host_traj_x, float * host_traj_y, float * host_traj_yaws,
  float * host_cvx, float * host_cvy, float * host_cwz, void * stream = nullptr);

bool cuda_evaluate_obstacles(
  size_t batch_size, size_t time_steps,
  void * d_traj_x, void * d_traj_y, void * d_traj_yaws, void * d_footprint_workspace,
  void ** d_costmap, void * d_costs,
  const unsigned char * host_costmap, bool costmap_changed, unsigned int size_x, unsigned int size_y,
  float resolution, float origin_x, float origin_y,
  float inscribed_radius, float inflation_scale_factor, float inflation_radius,
  float collision_margin_distance, float collision_cost, float critical_weight,
  float repulsion_weight, bool near_goal, bool unknown_is_collision, unsigned int power,
  const float * footprint_x, const float * footprint_y, unsigned int footprint_size,
  float possibly_inscribed_cost,
  float * host_costs, bool far_downsample, float far_start_ratio, unsigned int far_stride,
  void * stream = nullptr);

bool cuda_evaluate_path_angle(
  size_t batch_size, size_t time_steps,
  void * d_traj_x, void * d_traj_y, void * d_traj_yaws, void * d_cwz, void * d_costs,
  float goal_x, float goal_y,
  bool reversing_allowed, bool forward_preference,
  float direction_deadband, float wrong_way_cost_ratio,
  float current_x, float current_y, float current_yaw, float current_wz,
  float weight, unsigned int power,
  float * host_costs, bool far_downsample, float far_start_ratio, unsigned int far_stride,
  void * stream = nullptr);

bool cuda_evaluate_cost_critic(
  size_t batch_size, size_t time_steps,
  void * d_traj_x, void * d_traj_y, void * d_traj_yaws, void * d_footprint_workspace,
  void ** d_costmap, void * d_costs,
  const unsigned char * host_costmap, unsigned int size_x, unsigned int size_y,
  float resolution, float origin_x, float origin_y,
  float critical_cost, float collision_cost, float weight,
  bool near_goal, bool unknown_is_collision, unsigned int power,
  const float * footprint_x, const float * footprint_y, unsigned int footprint_size,
  float possibly_inscribed_cost, float * host_costs, void * stream = nullptr);

}  // namespace mppi::cuda

#endif  // NAV2_MPPI_CONTROLLER__CUDA__CUDA_KERNELS_H_
