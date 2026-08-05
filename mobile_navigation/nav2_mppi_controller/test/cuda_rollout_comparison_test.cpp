// Copyright (c) 2026 Mobile Navigation Team
// CUDA vs CPU Rollout Numerical Consistency Unit Test

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_mppi_controller/cuda/cuda_optimizer.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/trajectories.hpp"
#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/constraints.hpp"
#include "nav2_mppi_controller/motion_models.hpp"
#include "nav2_mppi_controller/tools/noise_generator.hpp"

namespace mppi
{

class CudaRolloutComparisonTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    batch_size_ = 500;
    time_steps_ = 40;
    model_dt_ = 0.05f;

    // 设置基础控制约束
    constraints_.vx_max = 0.5f;
    constraints_.vx_min = -0.35f;
    constraints_.vy = 0.5f;
    constraints_.wz = 1.0f;
    constraints_.ax_max = 3.0f;
    constraints_.ax_min = -3.0f;
    constraints_.ay_max = 3.0f;
    constraints_.ay_min = -3.0f;
    constraints_.az_max = 3.5f;
    constraints_.jx_max = 1000.0f;
    constraints_.jz_max = 1000.0f;

    // 机器人初始状态 (-1.5, 2.0, 0.3rad)
    init_pose_.header.frame_id = "map";
    init_pose_.pose.position.x = -1.5;
    init_pose_.pose.position.y = 2.0;
    init_pose_.pose.position.z = 0.0;
    // yaw = 0.3 rad => quaternion
    init_pose_.pose.orientation.z = std::sin(0.3 / 2.0);
    init_pose_.pose.orientation.w = std::cos(0.3 / 2.0);

    init_speed_.linear.x = 0.2;
    init_speed_.linear.y = 0.0;
    init_speed_.angular.z = 0.1;
  }

  // CPU 侧积分与推导 Helper (完全按 GPU Kernel 的连续时序逻辑对齐)
  void cpuRollout(
    bool is_holonomic,
    models::State & input_state,
    models::Trajectories & output_trajectories,
    float model_dt,
    float initial_yaw)
  {
    for (Eigen::Index r = 0; r < input_state.vx.rows(); ++r) {
      float x = input_state.pose.pose.position.x;
      float y = input_state.pose.pose.position.y;
      float yaw = initial_yaw;

      float rollout_vx = input_state.speed.linear.x;
      float rollout_vy = is_holonomic ? input_state.speed.linear.y : 0.0f;
      float rollout_wz = input_state.speed.angular.z;

      for (Eigen::Index c = 0; c < input_state.vx.cols(); ++c) {
        float yaw_for_vel = yaw;
        yaw += rollout_wz * model_dt;
        output_trajectories.yaws(r, c) = yaw;

        float yaw_cos = std::cos(yaw_for_vel);
        float yaw_sin = std::sin(yaw_for_vel);

        float dx = rollout_vx * yaw_cos;
        float dy = rollout_vx * yaw_sin;
        if (is_holonomic) {
          dx -= rollout_vy * yaw_sin;
          dy += rollout_vy * yaw_cos;
        }

        x += dx * model_dt;
        y += dy * model_dt;
        output_trajectories.x(r, c) = x;
        output_trajectories.y(r, c) = y;

        // 移向下一步的速度状态
        rollout_vx = input_state.cvx(r, c);
        rollout_wz = input_state.cwz(r, c);
        if (is_holonomic) {
          rollout_vy = input_state.cvy(r, c);
        }
      }
    }
  }

  size_t batch_size_;
  size_t time_steps_;
  float model_dt_;
  models::ControlConstraints constraints_;
  geometry_msgs::msg::PoseStamped init_pose_;
  geometry_msgs::msg::Twist init_speed_;
};

// ── TEST 1: CUDA 硬件可用性测试 ──
TEST_F(CudaRolloutComparisonTest, CudaDeviceAvailability)
{
  cuda::CudaOptimizer cuda_opt;
  bool ok = cuda_opt.initialize(batch_size_, time_steps_, model_dt_, false);
  if (ok) {
    EXPECT_TRUE(cuda_opt.isCudaAvailable());
    std::cout << "[TEST INFO] ✅ CUDA GPU is available and initialized successfully." << std::endl;
  } else {
    std::cout << "[TEST INFO] ⚠️ CUDA device is not available in current environment." << std::endl;
  }
}

// ── TEST 2: 差速底盘 (DiffDrive) 零噪声 CPU vs CUDA 数值一致性测试 ──
TEST_F(CudaRolloutComparisonTest, DiffDriveZeroNoiseConsistency)
{
  cuda::CudaOptimizer cuda_opt;
  if (!cuda_opt.initialize(batch_size_, time_steps_, model_dt_, false)) {
    GTEST_SKIP() << "CUDA hardware unavailable, skipping CUDA comparison test.";
  }

  // 设定零采样噪声
  models::SamplingStd zero_std{0.0f, 0.0f, 0.0f};

  // 构建恒定/渐变控制序列
  std::vector<float> control_vx(time_steps_);
  std::vector<float> control_wz(time_steps_);
  std::vector<float> control_vy;
  for (size_t i = 0; i < time_steps_; ++i) {
    control_vx[i] = 0.3f + 0.002f * static_cast<float>(i);
    control_wz[i] = 0.1f + 0.005f * static_cast<float>(i);
  }

  cuda::CudaState cuda_init{
    static_cast<float>(init_pose_.pose.position.x),
    static_cast<float>(init_pose_.pose.position.y),
    0.3f,  // initial yaw
    static_cast<float>(init_speed_.linear.x),
    0.0f,
    static_cast<float>(init_speed_.angular.z)
  };

  models::Trajectories cuda_trajectories;
  cuda_trajectories.reset(batch_size_, time_steps_);
  models::State cuda_state;
  cuda_state.reset(batch_size_, time_steps_);

  // 执行 CUDA 并发 Rollout
  bool ok = cuda_opt.generateNoisedTrajectories(
    cuda_init, control_vx, control_vy, control_wz,
    constraints_, zero_std,
    cuda_trajectories.x.data(),
    cuda_trajectories.y.data(),
    cuda_trajectories.yaws.data(),
    cuda_state.cvx.data(),
    nullptr,
    cuda_state.cwz.data());

  ASSERT_TRUE(ok);
  cuda_opt.synchronizeStream();

  // 构造对应的 CPU 状态并手动填入相同的采样速度（零噪声）
  models::State cpu_state;
  cpu_state.reset(batch_size_, time_steps_);
  cpu_state.pose = init_pose_;
  cpu_state.speed = init_speed_;
  for (size_t i = 0; i < time_steps_; ++i) {
    cpu_state.cvx.col(i).setConstant(cuda_state.cvx(0, i));
    cpu_state.cwz.col(i).setConstant(cuda_state.cwz(0, i));
  }

  models::Trajectories cpu_trajectories;
  cpu_trajectories.reset(batch_size_, time_steps_);
  cpuRollout(false, cpu_state, cpu_trajectories, model_dt_, 0.3f);

  // 比对 CPU 和 CUDA 轨迹上的每个点 (x, y, yaw)
  float max_diff_x = 0.0f;
  float max_diff_y = 0.0f;
  float max_diff_yaw = 0.0f;

  for (size_t r = 0; r < batch_size_; ++r) {
    for (size_t c = 0; c < time_steps_; ++c) {
      float dx = std::abs(cpu_trajectories.x(r, c) - cuda_trajectories.x(r, c));
      float dy = std::abs(cpu_trajectories.y(r, c) - cuda_trajectories.y(r, c));
      float dyaw = std::abs(cpu_trajectories.yaws(r, c) - cuda_trajectories.yaws(r, c));

      max_diff_x = std::max(max_diff_x, dx);
      max_diff_y = std::max(max_diff_y, dy);
      max_diff_yaw = std::max(max_diff_yaw, dyaw);

      EXPECT_NEAR(cpu_trajectories.x(r, c), cuda_trajectories.x(r, c), 2e-2f)
        << "Mismatch at batch " << r << " step " << c;
      EXPECT_NEAR(cpu_trajectories.y(r, c), cuda_trajectories.y(r, c), 2e-2f)
        << "Mismatch at batch " << r << " step " << c;
      EXPECT_NEAR(cpu_trajectories.yaws(r, c), cuda_trajectories.yaws(r, c), 2e-2f)
        << "Mismatch at batch " << r << " step " << c;
    }
  }

  std::cout << "[TEST SUCCESS] DiffDrive Rollout Max Abs Diff -> X: "
            << max_diff_x << " m, Y: " << max_diff_y << " m, Yaw: " << max_diff_yaw << " rad" << std::endl;
}

// ── TEST 3: 全向底盘 (Omni) 零噪声 CPU vs CUDA 数值一致性测试 ──
TEST_F(CudaRolloutComparisonTest, OmniZeroNoiseConsistency)
{
  cuda::CudaOptimizer cuda_opt;
  if (!cuda_opt.initialize(batch_size_, time_steps_, model_dt_, true)) {
    GTEST_SKIP() << "CUDA hardware unavailable, skipping CUDA Omni comparison test.";
  }

  models::SamplingStd zero_std{0.0f, 0.0f, 0.0f};

  std::vector<float> control_vx(time_steps_, 0.25f);
  std::vector<float> control_vy(time_steps_, 0.15f);
  std::vector<float> control_wz(time_steps_, 0.08f);

  cuda::CudaState cuda_init{
    static_cast<float>(init_pose_.pose.position.x),
    static_cast<float>(init_pose_.pose.position.y),
    0.3f,
    static_cast<float>(init_speed_.linear.x),
    0.1f,  // initial vy
    static_cast<float>(init_speed_.angular.z)
  };

  models::Trajectories cuda_trajectories;
  cuda_trajectories.reset(batch_size_, time_steps_);
  models::State cuda_state;
  cuda_state.reset(batch_size_, time_steps_);

  bool ok = cuda_opt.generateNoisedTrajectories(
    cuda_init, control_vx, control_vy, control_wz,
    constraints_, zero_std,
    cuda_trajectories.x.data(),
    cuda_trajectories.y.data(),
    cuda_trajectories.yaws.data(),
    cuda_state.cvx.data(),
    cuda_state.cvy.data(),
    cuda_state.cwz.data());

  ASSERT_TRUE(ok);
  cuda_opt.synchronizeStream();

  models::State cpu_state;
  cpu_state.reset(batch_size_, time_steps_);
  cpu_state.pose = init_pose_;
  cpu_state.speed = init_speed_;
  for (size_t i = 0; i < time_steps_; ++i) {
    cpu_state.cvx.col(i).setConstant(cuda_state.cvx(0, i));
    cpu_state.cvy.col(i).setConstant(cuda_state.cvy(0, i));
    cpu_state.cwz.col(i).setConstant(cuda_state.cwz(0, i));
  }

  models::Trajectories cpu_trajectories;
  cpu_trajectories.reset(batch_size_, time_steps_);
  cpuRollout(true, cpu_state, cpu_trajectories, model_dt_, 0.3f);

  for (size_t r = 0; r < batch_size_; ++r) {
    for (size_t c = 0; c < time_steps_; ++c) {
      EXPECT_NEAR(cpu_trajectories.x(r, c), cuda_trajectories.x(r, c), 2e-2f);
      EXPECT_NEAR(cpu_trajectories.y(r, c), cuda_trajectories.y(r, c), 2e-2f);
      EXPECT_NEAR(cpu_trajectories.yaws(r, c), cuda_trajectories.yaws(r, c), 2e-2f);
    }
  }

  std::cout << "[TEST SUCCESS] ✅ Omni Holonomic CUDA Rollout perfectly matches CPU Rollout." << std::endl;
}

// ── TEST 4: 高斯噪声采样轨迹多样性与物理约束测试 ──
TEST_F(CudaRolloutComparisonTest, TrajectoryDiversityAndConstraints)
{
  cuda::CudaOptimizer cuda_opt;
  if (!cuda_opt.initialize(batch_size_, time_steps_, model_dt_, false)) {
    GTEST_SKIP() << "CUDA hardware unavailable, skipping test.";
  }

  models::SamplingStd sampling_std{0.2f, 0.0f, 0.4f};
  std::vector<float> control_vx(time_steps_, 0.3f);
  std::vector<float> control_wz(time_steps_, 0.0f);

  cuda::CudaState cuda_init{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  models::Trajectories trajs;
  trajs.reset(batch_size_, time_steps_);
  models::State state;
  state.reset(batch_size_, time_steps_);

  bool ok = cuda_opt.generateNoisedTrajectories(
    cuda_init, control_vx, {}, control_wz,
    constraints_, sampling_std,
    trajs.x.data(), trajs.y.data(), trajs.yaws.data(),
    state.cvx.data(), nullptr, state.cwz.data());

  ASSERT_TRUE(ok);
  cuda_opt.synchronizeStream();

  // 1. 验证生成的 500 条轨迹具有高斯随机离散多样性（在 Y 轴和 Yaw 航向角方向呈高斯离散分布）
  EXPECT_GT(trajs.y.col(time_steps_ - 1).maxCoeff() - trajs.y.col(time_steps_ - 1).minCoeff(), 0.1f);
  EXPECT_GT(trajs.yaws.col(time_steps_ - 1).maxCoeff() - trajs.yaws.col(time_steps_ - 1).minCoeff(), 0.2f);
  EXPECT_GT(state.cvx.maxCoeff() - state.cvx.minCoeff(), 0.2f);

  std::cout << "[TEST SUCCESS] ✅ CUDA Trajectory Diversity & Noise Distribution checking passed." << std::endl;
}

}  // namespace mppi

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
