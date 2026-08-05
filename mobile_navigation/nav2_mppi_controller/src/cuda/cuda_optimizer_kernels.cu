// Copyright (c) 2026 Mobile Navigation Team
// Pure CUDA Device Kernels for MPPI Controller

#include "nav2_mppi_controller/cuda/cuda_kernels.h"
#include <cuda_runtime.h>
#include <curand_kernel.h>
#include <math.h>
#include <stdio.h>

namespace mppi::cuda
{

__device__ __forceinline__ float clampf(float value, float lower, float upper)
{
  return fmaxf(lower, fminf(upper, value));
}

// ── CUDA Kernel: Initialize cuRAND states ──
__global__ void setup_curand_kernel(curandState * states, unsigned long long seed, size_t n)
{
  size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < n) {
    curand_init(seed, idx, 0, &states[idx]);
  }
}

// ── CUDA Kernel: Parallel Trajectory Rollout ──
__global__ void generate_trajectories_kernel(
  curandState * curand_states,
  size_t batch_size,
  size_t time_steps,
  float model_dt,
  bool is_holonomic,
  CudaState init_state,
  const float * __restrict__ control_vx,
  const float * __restrict__ control_vy,
  const float * __restrict__ control_wz,
  models::ControlConstraints constraints,
  models::SamplingStd sampling_std,
  float goal_distance,
  float * __restrict__ traj_x,
  float * __restrict__ traj_y,
  float * __restrict__ traj_yaws,
  float * __restrict__ cvx,
  float * __restrict__ cvy,
  float * __restrict__ cwz)
{
  size_t r = blockIdx.x * blockDim.x + threadIdx.x;
  if (r >= batch_size) return;

  curandState local_state = curand_states[r];

  float x = init_state.pose_x;
  float y = init_state.pose_y;
  float yaw = init_state.pose_yaw;

  const float max_delta_vx = model_dt * constraints.ax_max;
  const float min_delta_vx = model_dt * constraints.ax_min;
  const float max_delta_vy = model_dt * constraints.ay_max;
  const float min_delta_vy = model_dt * constraints.ay_min;
  const float max_delta_wz = model_dt * constraints.az_max;
  const float max_delta_ax = model_dt * constraints.jx_max;
  const float max_delta_wz_acc = model_dt * constraints.jz_max;

  float rollout_vx = init_state.speed_vx;
  float rollout_vy = is_holonomic ? init_state.speed_vy : 0.0f;
  float rollout_wz = init_state.speed_wz;
  float prev_vx_acc = 0.0f;
  float prev_vy_acc = 0.0f;
  float prev_wz_acc = 0.0f;

  float next_vx = init_state.speed_vx;
  float next_vy = is_holonomic ? init_state.speed_vy : 0.0f;
  float next_wz = init_state.speed_wz;

  // 根据当前机器到目标的剩余距离计算安全前向减速包络
  float safe_vx_cap = constraints.vx_max;
  if (goal_distance > 0.0f && goal_distance < 3.0f) {
    float decel = fabsf(constraints.ax_min) > 1e-3f ? fabsf(constraints.ax_min) : 0.6f;
    safe_vx_cap = fminf(constraints.vx_max, sqrtf(2.0f * decel * fmaxf(goal_distance, 0.01f)));
  }

  for (size_t c = 0; c < time_steps; ++c) {
    float noise_vx = curand_normal(&local_state) * sampling_std.vx;
    float noise_wz = curand_normal(&local_state) * sampling_std.wz;
    float noise_vy = is_holonomic ? curand_normal(&local_state) * sampling_std.vy : 0.0f;

    // Match the CPU admissible sample set with distance-deceleration cap.
    float sampled_vx = clampf(control_vx[c] + noise_vx, constraints.vx_min, safe_vx_cap);
    float sampled_wz = clampf(control_wz[c] + noise_wz, -constraints.wz, constraints.wz);
    float sampled_vy = is_holonomic ?
      clampf(control_vy[c] + noise_vy, -constraints.vy, constraints.vy) : 0.0f;

    if (c > 0) {
      rollout_vx = next_vx;
      rollout_vy = next_vy;
      rollout_wz = next_wz;
    }

    float output_cvx = sampled_vx;
    float output_cvy = sampled_vy;
    float output_cwz = sampled_wz;

    if (time_steps > 1 && c + 1 < time_steps) {
      const float lower_vx = rollout_vx > 0.0f ? rollout_vx + min_delta_vx : rollout_vx - max_delta_vx;
      const float upper_vx = rollout_vx > 0.0f ? rollout_vx + max_delta_vx : rollout_vx - min_delta_vx;
      float constrained_vx = clampf(sampled_vx, lower_vx, upper_vx);
      float vx_acc = (constrained_vx - rollout_vx) / model_dt;
      if (c > 0) {
        vx_acc = clampf(vx_acc, prev_vx_acc - max_delta_ax, prev_vx_acc + max_delta_ax);
      }
      vx_acc = clampf(vx_acc, constraints.ax_min, constraints.ax_max);
      next_vx = (c > 0) ? rollout_vx + vx_acc * model_dt : constrained_vx;
      output_cvx = next_vx;
      prev_vx_acc = vx_acc;

      float constrained_wz = clampf(sampled_wz, rollout_wz - max_delta_wz, rollout_wz + max_delta_wz);
      float wz_acc = (constrained_wz - rollout_wz) / model_dt;
      if (c > 0) {
        wz_acc = clampf(wz_acc, prev_wz_acc - max_delta_wz_acc, prev_wz_acc + max_delta_wz_acc);
      }
      wz_acc = clampf(wz_acc, -constraints.az_max, constraints.az_max);
      next_wz = (c > 0) ? rollout_wz + wz_acc * model_dt : constrained_wz;
      output_cwz = next_wz;
      prev_wz_acc = wz_acc;

      if (is_holonomic) {
        const float lower_vy = rollout_vy > 0.0f ? rollout_vy + min_delta_vy : rollout_vy - max_delta_vy;
        const float upper_vy = rollout_vy > 0.0f ? rollout_vy + max_delta_vy : rollout_vy - min_delta_vy;
        float constrained_vy = clampf(sampled_vy, lower_vy, upper_vy);
        float vy_acc = (constrained_vy - rollout_vy) / model_dt;
        if (c > 0) {
          vy_acc = clampf(vy_acc, prev_vy_acc - max_delta_ax, prev_vy_acc + max_delta_ax);
        }
        vy_acc = clampf(vy_acc, constraints.ay_min, constraints.ay_max);
        next_vy = (c > 0) ? rollout_vy + vy_acc * model_dt : constrained_vy;
        output_cvy = next_vy;
        prev_vy_acc = vy_acc;
      }
    }

    float yaw_for_vel = yaw;
    yaw += rollout_wz * model_dt;
    float yaw_cos = cosf(yaw_for_vel);
    float yaw_sin = sinf(yaw_for_vel);

    float dx = rollout_vx * yaw_cos;
    float dy = rollout_vx * yaw_sin;
    if (is_holonomic) {
      dx -= rollout_vy * yaw_sin;
      dy += rollout_vy * yaw_cos;
    }

    x += dx * model_dt;
    y += dy * model_dt;

    size_t idx = r * time_steps + c;
    traj_x[idx] = x;
    traj_y[idx] = y;
    traj_yaws[idx] = yaw;
    if (cvx) cvx[idx] = output_cvx;
    if (cwz) cwz[idx] = output_cwz;
    if (cvy) cvy[idx] = output_cvy;
  }

  curand_states[r] = local_state;
}

// ── CUDA Kernel: Parallel Obstacle Costmap Scoring ──
__global__ void evaluate_obstacles_kernel(
  size_t batch_size,
  size_t time_steps,
  const float * __restrict__ traj_x,
  const float * __restrict__ traj_y,
  const unsigned char * __restrict__ costmap_data,
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
  float * __restrict__ costs)
{
  size_t r = blockIdx.x * blockDim.x + threadIdx.x;
  if (r >= batch_size) return;

  float traj_cost = 0.0f;
  float repulsive_cost = 0.0f;
  bool collided = false;

  constexpr float LOG_253 = 5.5333895f;
  constexpr size_t NEAR_STEPS = 16;

  for (size_t c = 0; c < time_steps; ) {
    size_t step_inc = (c < NEAR_STEPS) ? 1 : 2;
    float weight_factor = (float)step_inc;

    size_t idx = r * time_steps + c;
    float px = traj_x[idx];
    float py = traj_y[idx];

    int mx = __float2int_rd((px - origin_x) / resolution);
    int my = __float2int_rd((py - origin_y) / resolution);

    if (mx < 0 || mx >= (int)size_x || my < 0 || my >= (int)size_y) {
      c += step_inc;
      continue;
    }

    unsigned char cell_cost = costmap_data[my * size_x + mx];
    if (cell_cost >= 254) {
      collided = true;
      break;
    } else if (cell_cost >= 1) {
      if (inflation_radius > 0.0f && inflation_scale_factor > 0.0f && repulsion_weight > 0.0f) {
        float dist_to_obj = (LOG_253 - __logf((float)cell_cost)) / inflation_scale_factor;
        if (dist_to_obj < 0.10f) {
          traj_cost += (0.10f - dist_to_obj) * weight_factor;
        } else if (!near_goal && dist_to_obj < inflation_radius) {
          repulsive_cost += (inflation_radius - dist_to_obj) * weight_factor;
        }
      }
    }
    c += step_inc;
  }

  if (collided) {
    costs[r] = 10000.0f;
  } else {
    float total = (critical_weight * traj_cost) + (repulsion_weight * repulsive_cost / (float)time_steps);
    costs[r] = powf(total, (float)power);
  }
}

// ── Launcher Functions ──

bool cuda_init_device(
  size_t batch_size, size_t time_steps,
  void ** curand_states, void ** d_noises, void ** d_traj_x, void ** d_traj_y, void ** d_traj_yaws, void ** d_costs,
  void ** d_cvx, void ** d_cvy, void ** d_cwz,
  void ** d_ctrl_vx, void ** d_ctrl_vy, void ** d_ctrl_wz, void ** stream)
{
  int device_count = 0;
  cudaError_t err = cudaGetDeviceCount(&device_count);
  if (err != cudaSuccess || device_count == 0) {
    printf("[MPPI CUDA] No CUDA device found: %s\n", cudaGetErrorString(err));
    return false;
  }

  size_t total_points = batch_size * time_steps;
  cudaMalloc(d_noises, total_points * 3 * sizeof(float));
  cudaMalloc(d_traj_x, total_points * sizeof(float));
  cudaMalloc(d_traj_y, total_points * sizeof(float));
  cudaMalloc(d_traj_yaws, total_points * sizeof(float));
  cudaMalloc(d_costs, batch_size * sizeof(float));
  cudaMalloc(d_cvx, total_points * sizeof(float));
  cudaMalloc(d_cvy, total_points * sizeof(float));
  cudaMalloc(d_cwz, total_points * sizeof(float));
  cudaMalloc(d_ctrl_vx, time_steps * sizeof(float));
  cudaMalloc(d_ctrl_vy, time_steps * sizeof(float));
  cudaMalloc(d_ctrl_wz, time_steps * sizeof(float));
  cudaMalloc(curand_states, batch_size * sizeof(curandState));

  if (stream != NULL) {
    cudaStream_t * s = (cudaStream_t *)malloc(sizeof(cudaStream_t));
    cudaStreamCreateWithFlags(s, cudaStreamNonBlocking);
    *stream = (void *)s;
  }

  cudaStream_t custream = (stream && *stream) ? *((cudaStream_t *)*stream) : 0;

  int threadsPerBlock = 256;
  int blocksPerGrid = (int)((batch_size + threadsPerBlock - 1) / threadsPerBlock);
  setup_curand_kernel<<<blocksPerGrid, threadsPerBlock, 0, custream>>>(
    (curandState *)*curand_states, 1337ULL, batch_size);

  if (custream) {
    cudaStreamSynchronize(custream);
  } else {
    cudaDeviceSynchronize();
  }
  printf("[MPPI CUDA] GPU Acceleration initialized on device with Async CUDA Stream (batch=%lu, steps=%lu)\n", batch_size, time_steps);
  return true;
}

void cuda_free_device(
  void * curand_states, void * d_noises, void * d_traj_x, void * d_traj_y, void * d_traj_yaws, void * d_costmap, void * d_costs,
  void * d_cvx, void * d_cvy, void * d_cwz,
  void * d_ctrl_vx, void * d_ctrl_vy, void * d_ctrl_wz, void * stream)
{
  if (stream) {
    cudaStream_t * s = (cudaStream_t *)stream;
    if (s && *s) {
      cudaStreamDestroy(*s);
    }
    free(s);
  }

  if (d_noises) cudaFree(d_noises);
  if (d_traj_x) cudaFree(d_traj_x);
  if (d_traj_y) cudaFree(d_traj_y);
  if (d_traj_yaws) cudaFree(d_traj_yaws);
  if (d_costmap) cudaFree(d_costmap);
  if (d_costs) cudaFree(d_costs);
  if (d_cvx) cudaFree(d_cvx);
  if (d_cvy) cudaFree(d_cvy);
  if (d_cwz) cudaFree(d_cwz);
  if (d_ctrl_vx) cudaFree(d_ctrl_vx);
  if (d_ctrl_vy) cudaFree(d_ctrl_vy);
  if (d_ctrl_wz) cudaFree(d_ctrl_wz);
  if (curand_states) cudaFree(curand_states);
}

bool cuda_synchronize_stream(void * stream)
{
  if (stream) {
    cudaStream_t * s = (cudaStream_t *)stream;
    if (s && *s) {
      cudaError_t err = cudaStreamSynchronize(*s);
      return err == cudaSuccess;
    }
  }
  return true;
}

bool cuda_generate_trajectories(
  size_t batch_size, size_t time_steps, float model_dt, bool is_holonomic,
  const CudaState & init_state,
  const float * control_vx, const float * control_vy, const float * control_wz,
  models::ControlConstraints constraints, models::SamplingStd sampling_std,
  void * curand_states, void * d_traj_x, void * d_traj_y, void * d_traj_yaws,
  void * d_cvx, void * d_cvy, void * d_cwz,
  void * d_ctrl_vx, void * d_ctrl_vy, void * d_ctrl_wz,
  float * host_traj_x, float * host_traj_y, float * host_traj_yaws,
  float * host_cvx, float * host_cvy, float * host_cwz, float goal_distance, void * stream)
{
  cudaStream_t custream = (stream) ? *((cudaStream_t *)stream) : 0;

  cudaMemcpyAsync(d_ctrl_vx, control_vx, time_steps * sizeof(float), cudaMemcpyHostToDevice, custream);
  cudaMemcpyAsync(d_ctrl_wz, control_wz, time_steps * sizeof(float), cudaMemcpyHostToDevice, custream);

  if (is_holonomic && control_vy != NULL && d_ctrl_vy != NULL) {
    cudaMemcpyAsync(d_ctrl_vy, control_vy, time_steps * sizeof(float), cudaMemcpyHostToDevice, custream);
  }

  int threadsPerBlock = 256;
  int blocksPerGrid = (int)((batch_size + threadsPerBlock - 1) / threadsPerBlock);

  generate_trajectories_kernel<<<blocksPerGrid, threadsPerBlock, 0, custream>>>(
    (curandState *)curand_states,
    batch_size, time_steps, model_dt, is_holonomic,
    init_state, (float *)d_ctrl_vx, (float *)(is_holonomic ? d_ctrl_vy : NULL), (float *)d_ctrl_wz, constraints, sampling_std,
    goal_distance,
    (float *)d_traj_x, (float *)d_traj_y, (float *)d_traj_yaws,
    (float *)d_cvx, (float *)d_cvy, (float *)d_cwz);

  cudaError_t err_launch = cudaGetLastError();
  if (err_launch != cudaSuccess) {
    printf("[MPPI CUDA ERROR] generate_trajectories_kernel launch failed: %s\n", cudaGetErrorString(err_launch));
    return false;
  }

  size_t total_points = batch_size * time_steps;
  cudaMemcpyAsync(host_traj_x, d_traj_x, total_points * sizeof(float), cudaMemcpyDeviceToHost, custream);
  cudaMemcpyAsync(host_traj_y, d_traj_y, total_points * sizeof(float), cudaMemcpyDeviceToHost, custream);
  cudaMemcpyAsync(host_traj_yaws, d_traj_yaws, total_points * sizeof(float), cudaMemcpyDeviceToHost, custream);

  if (host_cvx && d_cvx) {
    cudaMemcpyAsync(host_cvx, d_cvx, total_points * sizeof(float), cudaMemcpyDeviceToHost, custream);
  }
  if (host_cwz && d_cwz) {
    cudaMemcpyAsync(host_cwz, d_cwz, total_points * sizeof(float), cudaMemcpyDeviceToHost, custream);
  }
  if (host_cvy && d_cvy) {
    cudaMemcpyAsync(host_cvy, d_cvy, total_points * sizeof(float), cudaMemcpyDeviceToHost, custream);
  }

  if (custream) {
    cudaError_t sync_err = cudaStreamSynchronize(custream);
    if (sync_err != cudaSuccess) {
      printf("[MPPI CUDA ERROR] cudaStreamSynchronize failed: %s\n", cudaGetErrorString(sync_err));
      return false;
    }
  } else {
    cudaError_t sync_err = cudaDeviceSynchronize();
    if (sync_err != cudaSuccess) {
      printf("[MPPI CUDA ERROR] cudaDeviceSynchronize failed: %s\n", cudaGetErrorString(sync_err));
      return false;
    }
  }

  return true;
}

bool cuda_evaluate_obstacles(
  size_t batch_size, size_t time_steps,
  void * d_traj_x, void * d_traj_y, void ** d_costmap, void * d_costs,
  const unsigned char * host_costmap, bool costmap_changed, unsigned int size_x, unsigned int size_y,
  float resolution, float origin_x, float origin_y,
  float circumscribed_cost, float inflation_scale_factor, float inflation_radius,
  float critical_weight, float repulsion_weight, bool near_goal, unsigned int power,
  float * host_costs, void * stream)
{
  cudaStream_t custream = (stream) ? *((cudaStream_t *)stream) : 0;

  size_t map_size = size_x * size_y;
  if (*d_costmap == NULL) {
    cudaMalloc(d_costmap, map_size * sizeof(unsigned char));
    costmap_changed = true;
  }
  
  if (costmap_changed) {
    cudaMemcpyAsync(*d_costmap, host_costmap, map_size * sizeof(unsigned char), cudaMemcpyHostToDevice, custream);
  }

  int threadsPerBlock = 256;
  int blocksPerGrid = (int)((batch_size + threadsPerBlock - 1) / threadsPerBlock);

  evaluate_obstacles_kernel<<<blocksPerGrid, threadsPerBlock, 0, custream>>>(
    batch_size, time_steps,
    (const float *)d_traj_x, (const float *)d_traj_y, (const unsigned char *)*d_costmap,
    size_x, size_y, resolution, origin_x, origin_y,
    circumscribed_cost, inflation_scale_factor, inflation_radius,
    critical_weight, repulsion_weight, near_goal, power,
    (float *)d_costs);

  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess) {
    return false;
  }

  err = cudaMemcpyAsync(
    host_costs, d_costs, batch_size * sizeof(float), cudaMemcpyDeviceToHost, custream);
  if (err != cudaSuccess) {
    return false;
  }

  // The caller consumes host_costs immediately. Returning before this copy completes makes
  // CUDA critic costs appear as their zero-initialized host values.
  return cudaStreamSynchronize(custream) == cudaSuccess;
}

__device__ __forceinline__ float cuda_normalize_angle(float angle) {
  while (angle > (float)M_PI) angle -= 2.0f * (float)M_PI;
  while (angle < -(float)M_PI) angle += 2.0f * (float)M_PI;
  return angle;
}

__device__ __forceinline__ float cuda_shortest_angular_distance(float from, float to) {
  return cuda_normalize_angle(to - from);
}

__global__ void evaluate_path_angle_kernel(
  size_t batch_size, size_t time_steps,
  const float * __restrict__ traj_x,
  const float * __restrict__ traj_y,
  const float * __restrict__ traj_yaws,
  const float * __restrict__ cwz,
  float goal_x, float goal_y,
  bool reversing_allowed, bool forward_preference,
  float direction_deadband, float wrong_way_cost_ratio,
  float current_x, float current_y, float current_yaw, float current_wz,
  float weight, unsigned int power,
  float * __restrict__ d_costs)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= (int)batch_size) return;

  float yaw_error_sum = 0.0f;
  for (size_t t = 0; t < time_steps; ++t) {
    size_t point_idx = idx * time_steps + t;
    float tx = traj_x[point_idx];
    float ty = traj_y[point_idx];
    float tyaw = traj_yaws[point_idx];

    float yaws_between = atan2f(goal_y - ty, goal_x - tx);
    float target_yaw = yaws_between;

    if (reversing_allowed && !forward_preference) {
      float fwd_err = fabsf(cuda_shortest_angular_distance(tyaw, yaws_between));
      if (fwd_err >= (float)M_PI_2) {
        target_yaw = cuda_normalize_angle(yaws_between + (float)M_PI);
      }
    }

    float yaw_err = fabsf(cuda_shortest_angular_distance(tyaw, target_yaw));
    if (wrong_way_cost_ratio != 1.0f && yaw_err > (float)M_PI_2) {
      yaw_err *= wrong_way_cost_ratio;
    }
    yaw_error_sum += yaw_err;
  }

  float total_cost = powf(yaw_error_sum / (float)time_steps * weight, (float)power);

  if (wrong_way_cost_ratio > 0.0f) {
    float current_target_yaw = atan2f(goal_y - current_y, goal_x - current_x);
    float effective_target_yaw = current_target_yaw;
    if (reversing_allowed && !forward_preference) {
      float forward_error = fabsf(cuda_shortest_angular_distance(current_yaw, current_target_yaw));
      float reverse_target_yaw = current_target_yaw + (float)M_PI;
      float reverse_error = fabsf(cuda_shortest_angular_distance(current_yaw, reverse_target_yaw));
      if (reverse_error < forward_error) {
        effective_target_yaw = reverse_target_yaw;
      }
    }

    float current_error = cuda_shortest_angular_distance(current_yaw, effective_target_yaw);
    if (fabsf(current_error) > direction_deadband) {
      float desired_direction = current_error > 0.0f ? 1.0f : -1.0f;
      float wrong_way_sum = 0.0f;
      for (size_t t = 0; t < time_steps; ++t) {
        float state_wz = t == 0 ? current_wz : cwz[idx * time_steps + t - 1];
        wrong_way_sum += fmaxf(-desired_direction * state_wz, 0.0f);
      }
      total_cost += powf(
        wrong_way_sum / (float)time_steps * weight * wrong_way_cost_ratio,
        (float)power);
    }
  }

  d_costs[idx] = total_cost;
}

bool cuda_evaluate_path_angle(
  size_t batch_size, size_t time_steps,
  void * d_traj_x, void * d_traj_y, void * d_traj_yaws, void * d_cwz, void * d_costs,
  float goal_x, float goal_y,
  bool reversing_allowed, bool forward_preference,
  float direction_deadband, float wrong_way_cost_ratio,
  float current_x, float current_y, float current_yaw, float current_wz,
  float weight, unsigned int power,
  float * host_costs, void * stream)
{
  cudaStream_t custream = (stream) ? *((cudaStream_t *)stream) : 0;

  int threadsPerBlock = 256;
  int blocksPerGrid = (int)((batch_size + threadsPerBlock - 1) / threadsPerBlock);

  evaluate_path_angle_kernel<<<blocksPerGrid, threadsPerBlock, 0, custream>>>(
    batch_size, time_steps,
    (const float *)d_traj_x, (const float *)d_traj_y, (const float *)d_traj_yaws,
    (const float *)d_cwz,
    goal_x, goal_y, reversing_allowed, forward_preference,
    direction_deadband, wrong_way_cost_ratio,
    current_x, current_y, current_yaw, current_wz,
    weight, power,
    (float *)d_costs);

  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess) {
    return false;
  }

  err = cudaMemcpyAsync(
    host_costs, d_costs, batch_size * sizeof(float), cudaMemcpyDeviceToHost, custream);
  if (err != cudaSuccess) {
    return false;
  }

  return cudaStreamSynchronize(custream) == cudaSuccess;
}

}  // namespace mppi::cuda
