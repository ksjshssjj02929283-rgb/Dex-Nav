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

#include "gtest/gtest.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_core/goal_checker.hpp>


#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/motion_models.hpp"

#include "utils/utils.hpp"

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

// Smoke tests the optimizer

class OptimizerSuite : public ::testing::TestWithParam<std::tuple<std::string,
    std::vector<std::string>, bool>> {};

TEST_P(OptimizerSuite, OptimizerTest) {
  auto [motion_model, critics, consider_footprint] = GetParam();

  int batch_size = 8000;
  int time_steps = 60;
  unsigned int path_points = 100u;
  int iteration_count = 3;
  double lookahead_distance = 15.0;

  TestCostmapSettings costmap_settings{};
  auto costmap_ros = getDummyCostmapRos(costmap_settings);
  auto costmap = costmap_ros->getCostmap();

  TestPose start_pose = costmap_settings.getCenterPose();
  double path_step = costmap_settings.resolution;

  TestPathSettings path_settings{start_pose, path_points, path_step, path_step};
  TestOptimizerSettings optimizer_settings{batch_size, time_steps, iteration_count,
    lookahead_distance, motion_model, consider_footprint};

  unsigned int offset = 4;
  unsigned int obstacle_size = offset * 2;

  unsigned char obstacle_cost = 250;

  auto [obst_x, obst_y] = costmap_settings.getCenterIJ();

  obst_x = obst_x - offset;
  obst_y = obst_y - offset;
  addObstacle(costmap, {obst_x, obst_y, obstacle_size, obstacle_cost});

  printInfo(optimizer_settings, path_settings, critics);
  auto node = getDummyNode(optimizer_settings, critics);
  auto parameters_handler = std::make_unique<mppi::ParametersHandler>(node);
  auto optimizer = getDummyOptimizer(node, costmap_ros, parameters_handler.get());

  // evalControl args
  auto pose = getDummyPointStamped(node, start_pose);
  auto velocity = getDummyTwist();
  auto path = getIncrementalDummyPath(node, path_settings);
  nav2_core::GoalChecker * dummy_goal_checker{nullptr};

  const int NUM_RUNS = 20;

  // ── 1. CPU 模式实测 ──
  double cpu_total_ms = 0.0;
  std::vector<double> cpu_timings;
  {
    auto node_cpu = getDummyNode(optimizer_settings, critics, "dummy_cpu");
    node_cpu->declare_parameter("dummy_cpu.use_cuda", false);
    node_cpu->set_parameter(rclcpp::Parameter("dummy_cpu.use_cuda", false));
    auto param_handler_cpu = std::make_unique<mppi::ParametersHandler>(node_cpu);
    auto optimizer_cpu = getDummyOptimizer(node_cpu, costmap_ros, param_handler_cpu.get());

    for (int i = 0; i < 3; ++i) {
      optimizer_cpu->evalControl(pose, velocity, path, dummy_goal_checker);
    }
    for (int i = 0; i < NUM_RUNS; ++i) {
      auto start = std::chrono::high_resolution_clock::now();
      optimizer_cpu->evalControl(pose, velocity, path, dummy_goal_checker);
      auto end = std::chrono::high_resolution_clock::now();
      double ms = std::chrono::duration<double, std::milli>(end - start).count();
      cpu_timings.push_back(ms);
      cpu_total_ms += ms;
    }
  }
  double cpu_avg_ms = cpu_total_ms / NUM_RUNS;

  // ── 2. GPU (CUDA) 模式实测 ──
  double gpu_total_ms = 0.0;
  std::vector<double> gpu_timings;
  {
    auto node_gpu = getDummyNode(optimizer_settings, critics, "dummy_gpu");
    node_gpu->declare_parameter("dummy_gpu.use_cuda", true);
    node_gpu->set_parameter(rclcpp::Parameter("dummy_gpu.use_cuda", true));
    auto param_handler_gpu = std::make_unique<mppi::ParametersHandler>(node_gpu);
    auto optimizer_gpu = getDummyOptimizer(node_gpu, costmap_ros, param_handler_gpu.get());

    for (int i = 0; i < 3; ++i) {
      optimizer_gpu->evalControl(pose, velocity, path, dummy_goal_checker);
    }
    for (int i = 0; i < NUM_RUNS; ++i) {
      auto start = std::chrono::high_resolution_clock::now();
      optimizer_gpu->evalControl(pose, velocity, path, dummy_goal_checker);
      auto end = std::chrono::high_resolution_clock::now();
      double ms = std::chrono::duration<double, std::milli>(end - start).count();
      gpu_timings.push_back(ms);
      gpu_total_ms += ms;
    }
  }
  double gpu_avg_ms = gpu_total_ms / NUM_RUNS;

  std::sort(cpu_timings.begin(), cpu_timings.end());
  std::sort(gpu_timings.begin(), gpu_timings.end());
  double speedup = cpu_avg_ms / std::max(0.001, gpu_avg_ms);

  std::cout << "\n====================================================================\n";
  std::cout << "       MPPI 大规模 CUDA vs CPU 严谨独立对比测试报告 (" << motion_model << ")\n";
  std::cout << "====================================================================\n";
  std::cout << " 规模参数: Batch Size = " << batch_size << ", Time Steps = " << time_steps << ", 测试轮数 = " << NUM_RUNS << "\n";
  std::cout << " --------------------------------------------------------------------\n";
  std::cout << " 运行模式  | 平均耗时 (ms) | 最小耗时 (ms) | 最大耗时 (ms)\n";
  std::cout << " --------------------------------------------------------------------\n";
  std::cout << " CPU 模式  | " << std::setw(13) << cpu_avg_ms << " | " << std::setw(13) << cpu_timings.front() << " | " << std::setw(13) << cpu_timings.back() << "\n";
  std::cout << " GPU 模式  | " << std::setw(13) << gpu_avg_ms << " | " << std::setw(13) << gpu_timings.front() << " | " << std::setw(13) << gpu_timings.back() << "\n";
  std::cout << " --------------------------------------------------------------------\n";
  std::cout << " >>> 真实 CUDA 加速倍数 (Speedup Factor): " << speedup << " x <<<\n";
  std::cout << "====================================================================\n\n";

  EXPECT_NO_THROW(optimizer->evalControl(pose, velocity, path, dummy_goal_checker));
}

INSTANTIATE_TEST_SUITE_P(
  OptimizerTests,
  OptimizerSuite,
  ::testing::Values(
    std::make_tuple(
      "Omni",
      std::vector<std::string>(
        {{"GoalCritic"}, {"GoalAngleCritic"}, {"ObstaclesCritic"}, {"PathAlignCritic"},
          {"TwirlingCritic"}, {"PathFollowCritic"}, {"PreferForwardCritic"}}),
      true),
    std::make_tuple(
      "DiffDrive",
      std::vector<std::string>(
        {{"GoalCritic"}, {"GoalAngleCritic"}, {"CostCritic"},
          {"PathAngleCritic"}, {"PathFollowCritic"}, {"PreferForwardCritic"}}),
      true),
    std::make_tuple(
      "Ackermann",
      std::vector<std::string>(
        {{"GoalCritic"}, {"GoalAngleCritic"}, {"ObstaclesCritic"},
          {"PathAngleCritic"}, {"PathFollowCritic"}, {"PreferForwardCritic"}}),
      true))
);
