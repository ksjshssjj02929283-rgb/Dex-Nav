// Copyright (c) 2026 Mobile Navigation Team
// Pure CUDA Device Kernels for MPPI Controller

#include "nav2_mppi_controller/cuda/cuda_kernels.h"
#include <cuda_runtime.h>
#include <curand_kernel.h>
#include <limits>
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

  for (size_t c = 0; c < time_steps; ++c) {
    float noise_vx = curand_normal(&local_state) * sampling_std.vx;
    float noise_wz = curand_normal(&local_state) * sampling_std.wz;
    float noise_vy = is_holonomic ? curand_normal(&local_state) * sampling_std.vy : 0.0f;

    // CPU samples unconstrained Gaussian controls, then MotionModel::predict applies
    // acceleration and jerk constraints to the columns consumed by the rollout.
    float sampled_vx = control_vx[c] + noise_vx;
    float sampled_wz = control_wz[c] + noise_wz;
    float sampled_vy = is_holonomic ? control_vy[c] + noise_vy : 0.0f;

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

__device__ __forceinline__ unsigned int score_sample_weight(
  size_t step, size_t time_steps, bool far_downsample, size_t far_start,
  unsigned int far_stride)
{
  if (!far_downsample || far_stride <= 1 || step < far_start) {
    return 1;
  }
  if ((step - far_start) % far_stride == 0) {
    return min(far_stride, static_cast<unsigned int>(time_steps - step));
  }
  // Always retain the horizon endpoint so terminal behavior remains represented.
  return step + 1 == time_steps ? 1 : 0;
}

__device__ __forceinline__ bool footprint_vertex_to_map(
  float wx, float wy, float origin_x, float origin_y, float resolution,
  unsigned int size_x, unsigned int size_y, int & mx, int & my)
{
  mx = __float2int_rd((wx - origin_x) / resolution);
  my = __float2int_rd((wy - origin_y) / resolution);
  return mx >= 0 && mx < static_cast<int>(size_x) &&
         my >= 0 && my < static_cast<int>(size_y);
}

// Bresenham edge rasterization matching FootprintCollisionChecker::lineCost semantics.
__device__ float footprint_line_cost(
  int x0, int y0, int x1, int y1, const unsigned char * costmap_data,
  unsigned int size_x)
{
  const int delta_x = abs(x1 - x0);
  const int delta_y = abs(y1 - y0);
  int x_inc1 = x1 >= x0 ? 1 : -1;
  int x_inc2 = x_inc1;
  int y_inc1 = y1 >= y0 ? 1 : -1;
  int y_inc2 = y_inc1;
  int denominator;
  int numerator;
  int numerator_add;
  int pixels;
  if (delta_x >= delta_y) {
    x_inc1 = 0;
    y_inc2 = 0;
    denominator = delta_x;
    numerator = delta_x / 2;
    numerator_add = delta_y;
    pixels = delta_x;
  } else {
    x_inc2 = 0;
    y_inc1 = 0;
    denominator = delta_y;
    numerator = delta_y / 2;
    numerator_add = delta_x;
    pixels = delta_y;
  }
  float line_cost = 0.0f;
  for (int pixel = 0; pixel <= pixels; ++pixel) {
    const float point_cost = static_cast<float>(costmap_data[y0 * size_x + x0]);
    if (point_cost == 254.0f) return point_cost;
    line_cost = fmaxf(line_cost, point_cost);
    numerator += numerator_add;
    if (denominator > 0 && numerator >= denominator) {
      numerator -= denominator;
      x0 += x_inc1;
      y0 += y_inc1;
    }
    x0 += x_inc2;
    y0 += y_inc2;
  }
  return line_cost;
}

__device__ float footprint_cost_at_pose(
  float pose_x, float pose_y, float pose_yaw,
  const float * footprint_x, const float * footprint_y, unsigned int footprint_size,
  const unsigned char * costmap_data, unsigned int size_x, unsigned int size_y,
  float resolution, float origin_x, float origin_y)
{
  if (footprint_size < 3) return 254.0f;
  const float cos_yaw = cosf(pose_yaw);
  const float sin_yaw = sinf(pose_yaw);
  int first_x = 0;
  int first_y = 0;
  int previous_x = 0;
  int previous_y = 0;
  float max_cost = 0.0f;
  for (unsigned int vertex = 0; vertex < footprint_size; ++vertex) {
    const float wx = pose_x + footprint_x[vertex] * cos_yaw - footprint_y[vertex] * sin_yaw;
    const float wy = pose_y + footprint_x[vertex] * sin_yaw + footprint_y[vertex] * cos_yaw;
    int current_x = 0;
    int current_y = 0;
    if (!footprint_vertex_to_map(
        wx, wy, origin_x, origin_y, resolution, size_x, size_y, current_x, current_y))
    {
      return 254.0f;
    }
    if (vertex == 0) {
      first_x = previous_x = current_x;
      first_y = previous_y = current_y;
      continue;
    }
    max_cost = fmaxf(
      max_cost, footprint_line_cost(
        previous_x, previous_y, current_x, current_y, costmap_data, size_x));
    if (max_cost == 254.0f) return max_cost;
    previous_x = current_x;
    previous_y = current_y;
  }
  return fmaxf(
    max_cost, footprint_line_cost(
      first_x, first_y, previous_x, previous_y, costmap_data, size_x));
}

// ── CUDA Kernel: Parallel Obstacle Costmap Scoring ──
__global__ void evaluate_obstacles_kernel(
  size_t batch_size,
  size_t time_steps,
  const float * __restrict__ traj_x,
  const float * __restrict__ traj_y,
  const float * __restrict__ traj_yaws,
  const unsigned char * __restrict__ costmap_data,
  const float * __restrict__ footprint_x,
  const float * __restrict__ footprint_y,
  unsigned int footprint_size,
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
  float possibly_inscribed_cost,
  bool far_downsample,
  float far_start_ratio,
  unsigned int far_stride,
  float * __restrict__ costs)
{
  size_t r = blockIdx.x * blockDim.x + threadIdx.x;
  if (r >= batch_size) return;

  float traj_cost = 0.0f;
  float repulsive_cost = 0.0f;
  float evaluated_weight = 0.0f;
  bool collided = false;
  const size_t far_start = min(
    time_steps, static_cast<size_t>(far_start_ratio * static_cast<float>(time_steps)));

  constexpr float LOG_253 = 5.5333895f;
  for (size_t c = 0; c < time_steps; ++c) {
    const unsigned int sample_weight = score_sample_weight(
      c, time_steps, far_downsample, far_start, far_stride);
    if (sample_weight == 0) continue;
    evaluated_weight += static_cast<float>(sample_weight);
    size_t idx = r * time_steps + c;
    float px = traj_x[idx];
    float py = traj_y[idx];

    int mx = __float2int_rd((px - origin_x) / resolution);
    int my = __float2int_rd((py - origin_y) / resolution);

    const bool outside_map = mx < 0 || mx >= (int)size_x || my < 0 || my >= (int)size_y;
    unsigned char cell_cost = outside_map ? 255 : costmap_data[my * size_x + mx];
    bool using_footprint = false;
    if (!outside_map && footprint_size >= 3 &&
      (cell_cost >= possibly_inscribed_cost || possibly_inscribed_cost < 1.0f))
    {
      cell_cost = static_cast<unsigned char>(footprint_cost_at_pose(
          px, py, traj_yaws[idx], footprint_x, footprint_y, footprint_size,
          costmap_data, size_x, size_y, resolution, origin_x, origin_y));
      using_footprint = true;
    }
    if (cell_cost == 254 || (!using_footprint && cell_cost == 253) ||
      (cell_cost == 255 && unknown_is_collision)) {
      collided = true;
      break;
    } else if (cell_cost >= 1 && cell_cost < 253) {
      if (inflation_radius > 0.0f && inflation_scale_factor > 0.0f && repulsion_weight > 0.0f) {
        const float dist_to_obj =
          (inflation_scale_factor * inscribed_radius - __logf((float)cell_cost) + LOG_253) /
          inflation_scale_factor - (using_footprint ? 0.0f : inscribed_radius);
        if (dist_to_obj < collision_margin_distance) {
          traj_cost += sample_weight * (collision_margin_distance - dist_to_obj);
        } else if (!near_goal && dist_to_obj < inflation_radius) {
          repulsive_cost += sample_weight * (inflation_radius - dist_to_obj);
        }
      }
    }
  }

  const float raw_cost = collided ? collision_cost : traj_cost;
  const float total =
    (critical_weight * raw_cost) +
    (repulsion_weight * repulsive_cost / fmaxf(1.0f, evaluated_weight));
  costs[r] = powf(total, (float)power);
}

// ── Launcher Functions ──

#ifdef MPPI_CUDA_TESTING
namespace
{
int allocation_failure_index_for_testing = -1;
}

void cuda_set_allocation_failure_for_testing(int allocation_index)
{
  allocation_failure_index_for_testing = allocation_index;
}
#endif

bool cuda_init_device(
  size_t batch_size, size_t time_steps,
  void ** curand_states, void ** d_noises, void ** d_traj_x, void ** d_traj_y, void ** d_traj_yaws, void ** d_costs,
  void ** d_cvx, void ** d_cvy, void ** d_cwz,
  void ** d_ctrl_vx, void ** d_ctrl_vy, void ** d_ctrl_wz, void ** stream)
{
  if (batch_size == 0 || time_steps == 0 ||
    batch_size > std::numeric_limits<size_t>::max() / time_steps ||
    batch_size * time_steps > std::numeric_limits<size_t>::max() / (3 * sizeof(float)))
  {
    printf(
      "[MPPI CUDA] Invalid allocation dimensions (batch=%zu, steps=%zu)\n",
      batch_size, time_steps);
    return false;
  }

  *curand_states = nullptr;
  *d_noises = nullptr;
  *d_traj_x = nullptr;
  *d_traj_y = nullptr;
  *d_traj_yaws = nullptr;
  *d_costs = nullptr;
  *d_cvx = nullptr;
  *d_cvy = nullptr;
  *d_cwz = nullptr;
  *d_ctrl_vx = nullptr;
  *d_ctrl_vy = nullptr;
  *d_ctrl_wz = nullptr;
  if (stream != nullptr) {
    *stream = nullptr;
  }

  auto cleanup = [&]() {
      if (stream != nullptr && *stream != nullptr) {
        auto * s = static_cast<cudaStream_t *>(*stream);
        if (*s != nullptr) {
          cudaStreamDestroy(*s);
        }
        free(s);
        *stream = nullptr;
      }
      cudaFree(*d_noises);
      cudaFree(*d_traj_x);
      cudaFree(*d_traj_y);
      cudaFree(*d_traj_yaws);
      cudaFree(*d_costs);
      cudaFree(*d_cvx);
      cudaFree(*d_cvy);
      cudaFree(*d_cwz);
      cudaFree(*d_ctrl_vx);
      cudaFree(*d_ctrl_vy);
      cudaFree(*d_ctrl_wz);
      cudaFree(*curand_states);
      *d_noises = *d_traj_x = *d_traj_y = *d_traj_yaws = *d_costs = nullptr;
      *d_cvx = *d_cvy = *d_cwz = nullptr;
      *d_ctrl_vx = *d_ctrl_vy = *d_ctrl_wz = nullptr;
      *curand_states = nullptr;
    };

  int device_count = 0;
  cudaError_t err = cudaGetDeviceCount(&device_count);
  if (err != cudaSuccess || device_count == 0) {
    printf("[MPPI CUDA] No CUDA device found: %s\n", cudaGetErrorString(err));
    return false;
  }

  size_t total_points = batch_size * time_steps;
  auto allocate = [&](void ** pointer, size_t bytes, const char * name) {
#ifdef MPPI_CUDA_TESTING
      if (allocation_failure_index_for_testing == 0) {
        allocation_failure_index_for_testing = -1;
        printf("[MPPI CUDA TEST] Injected cudaMalloc failure for %s (%zu bytes)\n", name, bytes);
        return false;
      }
      if (allocation_failure_index_for_testing > 0) {
        --allocation_failure_index_for_testing;
      }
#endif
      const cudaError_t allocation_error = cudaMalloc(pointer, bytes);
      if (allocation_error != cudaSuccess) {
        printf(
          "[MPPI CUDA] cudaMalloc failed for %s (%zu bytes): %s\n",
          name, bytes, cudaGetErrorString(allocation_error));
        return false;
      }
      return true;
    };

  if (!allocate(d_noises, total_points * 3 * sizeof(float), "noise/workspace") ||
    !allocate(d_traj_x, total_points * sizeof(float), "trajectory x") ||
    !allocate(d_traj_y, total_points * sizeof(float), "trajectory y") ||
    !allocate(d_traj_yaws, total_points * sizeof(float), "trajectory yaw") ||
    !allocate(d_costs, batch_size * sizeof(float), "trajectory costs") ||
    !allocate(d_cvx, total_points * sizeof(float), "sampled vx") ||
    !allocate(d_cvy, total_points * sizeof(float), "sampled vy") ||
    !allocate(d_cwz, total_points * sizeof(float), "sampled wz") ||
    !allocate(d_ctrl_vx, time_steps * sizeof(float), "nominal vx") ||
    !allocate(d_ctrl_vy, time_steps * sizeof(float), "nominal vy") ||
    !allocate(d_ctrl_wz, time_steps * sizeof(float), "nominal wz") ||
    !allocate(curand_states, batch_size * sizeof(curandState), "cuRAND states"))
  {
    cleanup();
    return false;
  }

  if (stream != nullptr) {
    auto * s = static_cast<cudaStream_t *>(malloc(sizeof(cudaStream_t)));
    if (s == nullptr) {
      printf("[MPPI CUDA] Failed to allocate CUDA stream handle\n");
      cleanup();
      return false;
    }
    *s = nullptr;
    err = cudaStreamCreateWithFlags(s, cudaStreamNonBlocking);
    if (err != cudaSuccess) {
      printf("[MPPI CUDA] cudaStreamCreateWithFlags failed: %s\n", cudaGetErrorString(err));
      free(s);
      cleanup();
      return false;
    }
    *stream = static_cast<void *>(s);
  }

  cudaStream_t custream = (stream && *stream) ? *((cudaStream_t *)*stream) : 0;

  int threadsPerBlock = 256;
  int blocksPerGrid = (int)((batch_size + threadsPerBlock - 1) / threadsPerBlock);
  setup_curand_kernel<<<blocksPerGrid, threadsPerBlock, 0, custream>>>(
    (curandState *)*curand_states, 1337ULL, batch_size);
  err = cudaGetLastError();
  if (err != cudaSuccess) {
    printf("[MPPI CUDA] setup_curand_kernel launch failed: %s\n", cudaGetErrorString(err));
    cleanup();
    return false;
  }

  if (custream) {
    err = cudaStreamSynchronize(custream);
  } else {
    err = cudaDeviceSynchronize();
  }
  if (err != cudaSuccess) {
    printf("[MPPI CUDA] cuRAND initialization synchronization failed: %s\n", cudaGetErrorString(err));
    cleanup();
    return false;
  }
  printf("[MPPI CUDA] GPU Acceleration initialized on device with Async CUDA Stream (batch=%zu, steps=%zu)\n", batch_size, time_steps);
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
  float * host_cvx, float * host_cvy, float * host_cwz, void * stream)
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
  void * stream)
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

  float * d_footprint_x = static_cast<float *>(d_footprint_workspace);
  float * d_footprint_y = d_footprint_x + footprint_size;
  if (footprint_size > 0) {
    cudaMemcpyAsync(
      d_footprint_x, footprint_x, footprint_size * sizeof(float),
      cudaMemcpyHostToDevice, custream);
    cudaMemcpyAsync(
      d_footprint_y, footprint_y, footprint_size * sizeof(float),
      cudaMemcpyHostToDevice, custream);
  }

  int threadsPerBlock = 256;
  int blocksPerGrid = (int)((batch_size + threadsPerBlock - 1) / threadsPerBlock);

  evaluate_obstacles_kernel<<<blocksPerGrid, threadsPerBlock, 0, custream>>>(
    batch_size, time_steps,
    (const float *)d_traj_x, (const float *)d_traj_y, (const float *)d_traj_yaws,
    (const unsigned char *)*d_costmap, d_footprint_x, d_footprint_y, footprint_size,
    size_x, size_y, resolution, origin_x, origin_y,
    inscribed_radius, inflation_scale_factor, inflation_radius, collision_margin_distance,
    collision_cost, critical_weight, repulsion_weight, near_goal, unknown_is_collision, power,
    possibly_inscribed_cost,
    far_downsample, far_start_ratio, far_stride,
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

__global__ void evaluate_cost_critic_kernel(
  size_t batch_size, size_t time_steps,
  const float * __restrict__ traj_x,
  const float * __restrict__ traj_y,
  const float * __restrict__ traj_yaws,
  const unsigned char * __restrict__ costmap_data,
  const float * __restrict__ footprint_x,
  const float * __restrict__ footprint_y,
  unsigned int footprint_size,
  unsigned int size_x, unsigned int size_y,
  float resolution, float origin_x, float origin_y,
  float critical_cost, float collision_cost, float weight,
  bool near_goal, bool unknown_is_collision, unsigned int power,
  float possibly_inscribed_cost,
  float * __restrict__ costs)
{
  const size_t trajectory = blockIdx.x * blockDim.x + threadIdx.x;
  if (trajectory >= batch_size) return;

  float accumulated_cost = 0.0f;
  bool collided = false;
  for (size_t step = 0; step < time_steps; ++step) {
    const size_t point = trajectory * time_steps + step;
    const float px = traj_x[point];
    const float py = traj_y[point];
    const int mx = __float2int_rd((px - origin_x) / resolution);
    const int my = __float2int_rd((py - origin_y) / resolution);
    const bool outside = mx < 0 || mx >= static_cast<int>(size_x) ||
      my < 0 || my >= static_cast<int>(size_y);
    const unsigned char center_cost = outside ? 255 : costmap_data[my * size_x + mx];
    if (center_cost < 1) continue;

    unsigned char evaluated_cost = center_cost;
    bool used_footprint = false;
    if (!outside && footprint_size >= 3 &&
      (center_cost >= possibly_inscribed_cost || possibly_inscribed_cost < 1.0f))
    {
      evaluated_cost = static_cast<unsigned char>(footprint_cost_at_pose(
          px, py, traj_yaws[point], footprint_x, footprint_y, footprint_size,
          costmap_data, size_x, size_y, resolution, origin_x, origin_y));
      used_footprint = true;
    }

    const bool collision = evaluated_cost == 254 ||
      (!used_footprint && evaluated_cost == 253) ||
      (evaluated_cost == 255 && unknown_is_collision);
    if (collision) {
      collided = true;
      break;
    }

    // CPU CostCritic always scores the center-point cost, even when footprint was checked.
    if (center_cost >= 253) {
      accumulated_cost += critical_cost;
    } else if (!near_goal) {
      accumulated_cost += static_cast<float>(center_cost);
    }
  }

  if (collided) accumulated_cost = collision_cost;
  costs[trajectory] = powf(
    weight * accumulated_cost / static_cast<float>(time_steps), static_cast<float>(power));
}

bool cuda_evaluate_cost_critic(
  size_t batch_size, size_t time_steps,
  void * d_traj_x, void * d_traj_y, void * d_traj_yaws, void * d_footprint_workspace,
  void ** d_costmap, void * d_costs,
  const unsigned char * host_costmap, unsigned int size_x, unsigned int size_y,
  float resolution, float origin_x, float origin_y,
  float critical_cost, float collision_cost, float weight,
  bool near_goal, bool unknown_is_collision, unsigned int power,
  const float * footprint_x, const float * footprint_y, unsigned int footprint_size,
  float possibly_inscribed_cost, float * host_costs, void * stream)
{
  cudaStream_t custream = stream ? *((cudaStream_t *)stream) : 0;
  const size_t map_size = static_cast<size_t>(size_x) * size_y;
  if (*d_costmap == nullptr) cudaMalloc(d_costmap, map_size * sizeof(unsigned char));
  cudaError_t err = cudaMemcpyAsync(
    *d_costmap, host_costmap, map_size * sizeof(unsigned char), cudaMemcpyHostToDevice, custream);
  if (err != cudaSuccess) return false;

  float * d_footprint_x = static_cast<float *>(d_footprint_workspace);
  float * d_footprint_y = d_footprint_x + footprint_size;
  if (footprint_size > 0) {
    err = cudaMemcpyAsync(
      d_footprint_x, footprint_x, footprint_size * sizeof(float), cudaMemcpyHostToDevice, custream);
    if (err != cudaSuccess) return false;
    err = cudaMemcpyAsync(
      d_footprint_y, footprint_y, footprint_size * sizeof(float), cudaMemcpyHostToDevice, custream);
    if (err != cudaSuccess) return false;
  }

  constexpr int threads_per_block = 256;
  const int blocks = static_cast<int>((batch_size + threads_per_block - 1) / threads_per_block);
  evaluate_cost_critic_kernel<<<blocks, threads_per_block, 0, custream>>>(
    batch_size, time_steps,
    static_cast<const float *>(d_traj_x), static_cast<const float *>(d_traj_y),
    static_cast<const float *>(d_traj_yaws), static_cast<const unsigned char *>(*d_costmap),
    d_footprint_x, d_footprint_y, footprint_size, size_x, size_y,
    resolution, origin_x, origin_y, critical_cost, collision_cost, weight,
    near_goal, unknown_is_collision, power, possibly_inscribed_cost,
    static_cast<float *>(d_costs));
  err = cudaGetLastError();
  if (err != cudaSuccess) return false;
  err = cudaMemcpyAsync(
    host_costs, d_costs, batch_size * sizeof(float), cudaMemcpyDeviceToHost, custream);
  if (err != cudaSuccess) return false;
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
  bool far_downsample, float far_start_ratio, unsigned int far_stride,
  float * __restrict__ d_costs)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= (int)batch_size) return;

  float yaw_error_sum = 0.0f;
  float evaluated_weight = 0.0f;
  const size_t far_start = min(
    time_steps, static_cast<size_t>(far_start_ratio * static_cast<float>(time_steps)));
  for (size_t t = 0; t < time_steps; ++t) {
    const unsigned int sample_weight = score_sample_weight(
      t, time_steps, far_downsample, far_start, far_stride);
    if (sample_weight == 0) continue;
    evaluated_weight += static_cast<float>(sample_weight);
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

    yaw_error_sum += sample_weight * fabsf(cuda_shortest_angular_distance(tyaw, target_yaw));
  }

  float total_cost = powf(
    yaw_error_sum / fmaxf(1.0f, evaluated_weight) * weight, (float)power);

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
      float wrong_way_weight = 0.0f;
      for (size_t t = 0; t < time_steps; ++t) {
        const unsigned int sample_weight = score_sample_weight(
          t, time_steps, far_downsample, far_start, far_stride);
        if (sample_weight == 0) continue;
        float state_wz = t == 0 ? current_wz : cwz[idx * time_steps + t - 1];
        wrong_way_sum += sample_weight * fmaxf(-desired_direction * state_wz, 0.0f);
        wrong_way_weight += static_cast<float>(sample_weight);
      }
      total_cost += powf(
        wrong_way_sum / fmaxf(1.0f, wrong_way_weight) * weight * wrong_way_cost_ratio,
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
  float * host_costs, bool far_downsample, float far_start_ratio, unsigned int far_stride,
  void * stream)
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
    far_downsample, far_start_ratio, far_stride,
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
