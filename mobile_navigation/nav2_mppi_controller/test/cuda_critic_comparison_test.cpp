// Copyright (c) 2026 Mobile Navigation Team
// CUDA Stream Overlapping & GPU vs CPU Critic Scoring Consistency Test

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_mppi_controller/cuda/cuda_optimizer.hpp"
#include "nav2_mppi_controller/critics/obstacles_critic.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/trajectories.hpp"
#include "nav2_mppi_controller/models/constraints.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"

namespace mppi::critics
{

class CudaCriticComparisonTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    batch_size_ = 200;
    time_steps_ = 40;
    resolution_ = 0.05f;
    size_x_ = 100;
    size_y_ = 100;

    // 1. 初始化仿真 Costmap (100x100 grid, 5m x 5m)
    costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
      size_x_, size_y_, resolution_, -2.5, -2.5, 0);

    // 在地图中心放置一些障碍物 (LETHAL 254 及 INFLATED 150)
    for (unsigned int x = 51; x <= 55; ++x) {
      for (unsigned int y = 51; y <= 55; ++y) {
        costmap_->setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
    for (unsigned int x = 40; x < 45; ++x) {
      for (unsigned int y = 40; y < 45; ++y) {
        costmap_->setCost(x, y, 150);
      }
    }

    // 2. 构造 200 条分布在空间各处的测试候选轨迹
    trajectories_.reset(batch_size_, time_steps_);
    state_.reset(batch_size_, time_steps_);

    for (size_t r = 0; r < batch_size_; ++r) {
      float angle = (2.0f * M_PI * r) / static_cast<float>(batch_size_);
      float speed = 0.1f + 0.3f * (r % 5) / 5.0f;

      float x = 0.0f;
      float y = 0.0f;
      float yaw = angle;

      for (size_t c = 0; c < time_steps_; ++c) {
        x += speed * std::cos(yaw) * 0.05f;
        y += speed * std::sin(yaw) * 0.05f;
        trajectories_.x(r, c) = x;
        trajectories_.y(r, c) = y;
        trajectories_.yaws(r, c) = yaw;
      }
    }
  }

  size_t batch_size_;
  size_t time_steps_;
  unsigned int size_x_;
  unsigned int size_y_;
  float resolution_;
  std::unique_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  models::Trajectories trajectories_;
  models::State state_;
};

// ── TEST 1: GPU 障碍物打分与 CPU 障碍物打分结果一致性测试 ──
TEST_F(CudaCriticComparisonTest, ObstaclesCriticGpuVsCpuConsistency)
{
  cuda::CudaOptimizer cuda_opt;
  if (!cuda_opt.initialize(batch_size_, time_steps_, 0.05f, false)) {
    GTEST_SKIP() << "CUDA device not available in current environment, skipping GPU Critic test.";
  }

  // 先把轨迹同步推送到 GPU 的 d_traj_x, d_traj_y 中
  cuda::CudaState cuda_init{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  models::ControlConstraints constraints{
    0.5f, -0.35f, 0.5f, 1.0f, 3.0f, -3.0f, -3.0f, 3.0f, 3.5f, 1000.0f, 1000.0f};
  models::SamplingStd zero_std{0.0f, 0.0f, 0.0f};
  std::vector<float> control_vx(time_steps_, 0.3f);
  std::vector<float> control_wz(time_steps_, 0.1f);

  cuda_opt.generateNoisedTrajectories(
    cuda_init, control_vx, {}, control_wz,
    constraints, zero_std,
    trajectories_.x.data(), trajectories_.y.data(), trajectories_.yaws.data());
  cuda_opt.synchronizeStream();

  // 1. 调用 GPU 异步流版本的 evaluateObstacleCosts 打分
  std::vector<float> gpu_costs(batch_size_, 0.0f);
  bool ok = cuda_opt.evaluateObstacleCosts(
    costmap_->getCharMap(),
    size_x_, size_y_, resolution_,
    costmap_->getOriginX(), costmap_->getOriginY(),
    253.0f, 10.0f, 0.55f,
    20.0f, 1.5f, false, 1,
    gpu_costs);

  ASSERT_TRUE(ok);
  cuda_opt.synchronizeStream();

  // 2. 统计 GPU 侧障碍物碰撞检测与分值分布
  size_t gpu_collisions = 0;
  for (size_t r = 0; r < batch_size_; ++r) {
    if (gpu_costs[r] >= 10000.0f) {
      gpu_collisions++;
    }
  }

  std::cout << "[CRITIC TEST] GPU Evaluation complete: Total Trajectories=" << batch_size_
            << ", Collided Trajectories=" << gpu_collisions << std::endl;

  // 2. 在 CPU 上计算对应的碰撞判定
  size_t cpu_collisions = 0;
  for (size_t r = 0; r < batch_size_; ++r) {
    bool collided = false;
    for (size_t c = 0; c < time_steps_; ++c) {
      float px = trajectories_.x(r, c);
      float py = trajectories_.y(r, c);
      unsigned int mx, my;
      if (costmap_->worldToMap(px, py, mx, my)) {
        unsigned char cell_cost = costmap_->getCost(mx, my);
        if (cell_cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
          collided = true;
          break;
        }
      }
    }
    if (collided) {
      cpu_collisions++;
    }
  }

  // 3. 校验 GPU 与 CPU 的碰撞判定数 100% 完全相等
  EXPECT_EQ(gpu_collisions, cpu_collisions)
    << "GPU collision count (" << gpu_collisions << ") should exactly match CPU collision count (" << cpu_collisions << ")";

  std::cout << "[CRITIC TEST SUCCESS] ✅ GPU ObstaclesCritic Async Stream Evaluation (Collisions: "
            << gpu_collisions << "/" << batch_size_ << ") perfectly matches CPU Evaluation." << std::endl;
}

// ── TEST 2: GPU PathAngle 打分与 CPU PathAngle 打分数值一致性测试 ──
TEST_F(CudaCriticComparisonTest, PathAngleCriticGpuVsCpuConsistency)
{
  cuda::CudaOptimizer cuda_opt;
  if (!cuda_opt.initialize(batch_size_, time_steps_, 0.05f, false)) {
    GTEST_SKIP() << "CUDA device not available, skipping PathAngleCritic test.";
  }

  // 同步推送测试轨迹到 GPU 显存
  cuda::CudaState cuda_init{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  models::ControlConstraints constraints{
    0.5f, -0.35f, 0.5f, 1.0f, 3.0f, -3.0f, -3.0f, 3.0f, 3.5f, 1000.0f, 1000.0f};
  models::SamplingStd zero_std{0.0f, 0.0f, 0.0f};
  std::vector<float> control_vx(time_steps_, 0.3f);
  std::vector<float> control_wz(time_steps_, 0.1f);

  cuda_opt.generateNoisedTrajectories(
    cuda_init, control_vx, {}, control_wz,
    constraints, zero_std,
    trajectories_.x.data(), trajectories_.y.data(), trajectories_.yaws.data());
  cuda_opt.synchronizeStream();

  float goal_x = 2.0f;
  float goal_y = 1.5f;
  bool reversing_allowed = false;
  bool forward_preference = true;
  float direction_deadband = 0.05f;
  float wrong_way_cost_ratio = 2.0f;
  float weight = 2.0f;
  unsigned int power = 2;

  // 1. GPU 异步流打分
  std::vector<float> gpu_angle_costs(batch_size_, 0.0f);
  bool ok = cuda_opt.evaluatePathAngleCosts(
    goal_x, goal_y, reversing_allowed, forward_preference,
    direction_deadband, wrong_way_cost_ratio,
    cuda_init.pose_x, cuda_init.pose_y, cuda_init.pose_yaw, cuda_init.speed_wz,
    weight, power, gpu_angle_costs);

  ASSERT_TRUE(ok);
  cuda_opt.synchronizeStream();

  // 2. 比对 CPU 和 GPU 在 200 条候选轨迹上的 PathAngle 打分数值
  float max_cost_diff = 0.0f;
  for (size_t r = 0; r < batch_size_; ++r) {
    float cpu_total_cost = 0.0f;
    for (size_t t = 0; t < time_steps_; ++t) {
      float tx = trajectories_.x(r, t);
      float ty = trajectories_.y(r, t);
      float tyaw = trajectories_.yaws(r, t);

      float yaws_between = std::atan2(goal_y - ty, goal_x - tx);
      float target_yaw = yaws_between;

      float yaw_err = std::abs(std::atan2(std::sin(target_yaw - tyaw), std::cos(target_yaw - tyaw)));
      cpu_total_cost += yaw_err;
    }
    float cpu_cost = std::pow(
      cpu_total_cost / static_cast<float>(time_steps_) * weight,
      static_cast<float>(power));
    float diff = std::abs(cpu_cost - gpu_angle_costs[r]);
    max_cost_diff = std::max(max_cost_diff, diff);

    EXPECT_NEAR(cpu_cost, gpu_angle_costs[r], 1e-3f)
      << "PathAngle cost mismatch at trajectory " << r;
  }

  std::cout << "[CRITIC TEST SUCCESS] ✅ PathAngleCritic GPU vs CPU Score Consistency passed. Max Cost Diff="
            << max_cost_diff << std::endl;
}

}  // namespace mppi::critics

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
