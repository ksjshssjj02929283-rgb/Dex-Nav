// Copyright 2026 DexForce
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

#ifndef NAV2_THETA_STAR_PLANNER__SWERVE_STEERING_MODEL_HPP_
#define NAV2_THETA_STAR_PLANNER__SWERVE_STEERING_MODEL_HPP_

#include <array>
#include <cmath>
#include <vector>

#include "nav_msgs/msg/path.hpp"

namespace nav2_theta_star_planner
{

/**
 * @brief 与 mobile_cmd_omni_region.cpp 对齐的舵轮 IK / 连续性参数
 *
 * 规划器用该模型沿路径“预演”四轮目标舵角，避免路径 orientation 突变导致
 * 底盘 Two-Phase（先停轮、转舵、再走）硬逻辑被触发。
 */
struct SwerveSteeringConfig
{
  /// 前后轮距 L（m），对应 CHASSIS_SIZE_L * 0.001
  double wheelbase{0.46};
  /// 左右轮距 W（m），对应 CHASSIS_SIZE_W * 0.001
  double track{0.345};
  /// 舵角物理限位（rad），对应 validA/validB 的 ±π/2
  double steer_limit{M_PI_2};
  /**
   * 任一舵轮目标角相对上一时刻的最大允许跳变（rad）。
   * 对应 mobile_cmd_omni_region.cpp 的 ANGLE_EPS（默认 1.0 rad）。
   * 超过该值时控制器会置零所有轮速，仅转舵。
   */
  double max_wheel_angle_step{1.0};
  /// 沿路径仿真用的标称线速度（m/s），用于把路径点间距换算成 Δt
  double nominal_path_speed{0.5};
  /**
   * 路径上 body yaw 变化率上限（rad/s），用于把起终点转角均匀摊到路程上。
   * 应 ≥ MPPI 实际 wz_max；过小会导致“旋转迟钝、末段堆角”。
   */
  double max_body_yaw_rate{1.2};
  /// 舵轮 IK 微调最大迭代轮次（仅修正仍超限的段，宜小不宜大）
  int max_smoothing_passes{3};
};

/**
 * @brief 沿路径仿真时的四轮舵角记忆（对应控制器 prev_angle_[]）
 */
struct SwerveSteeringState
{
  std::array<float, 4> prev_wheel_angle{{0.f, 0.f, 0.f, 0.f}};

  void reset()
  {
    prev_wheel_angle.fill(0.f);
  }
};

/**
 * @brief 单段路径运动学：由相邻两点 + body yaw 推出 (vx, vy, wz)
 */
struct BodyTwistSample
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

/**
 * @brief 计算四轮目标舵角（仅 Phase 0 几何解算，与底盘控制器一致）
 *
 * @param vx, vy, vw  机体坐标系下的线/角速度（m/s, rad/s）
 * @param config      底盘几何与限位
 * @param prev_wheel_angle  上一时刻四轮舵角（用于 π 等价解 A/B 连续性）
 * @param out_wheel_angle   本时刻四轮目标舵角
 * @return 四轮相对 prev 的最大舵角跳变量（rad）
 */
double computeSwerveWheelTargets(
  double vx, double vy, double vw,
  const SwerveSteeringConfig & config,
  const std::array<float, 4> & prev_wheel_angle,
  std::array<float, 4> & out_wheel_angle);

/**
 * @brief 由路径相邻两点与 body yaw 估计该段对应的机体 twist
 */
BodyTwistSample bodyTwistFromPathSegment(
  double x0, double y0, double yaw0,
  double x1, double y1, double yaw1,
  const SwerveSteeringConfig & config);

/**
 * @brief 检查单段路径在指定 yaws 下是否满足舵角连续性
 *
 * @param in_out_state  传入/更新 prev_wheel_angle（仿真向前推进）
 * @return true  该段未触发舵角跳变约束
 */
bool isPathSegmentSteeringFeasible(
  double x0, double y0, double yaw0,
  double x1, double y1, double yaw1,
  const SwerveSteeringConfig & config,
  SwerveSteeringState & in_out_state);

/**
 * @brief B 层后处理：沿路径调整各点 yaw，使舵轮目标角尽量连续，并锚定终点 goal yaw
 *
 * 在 assignPathOrientations() 之后调用。不修改路径 (x,y)，只调整 orientation。
 *
 * @param path        已插值的几何路径（poses 含 position + orientation）
 * @param goal_yaw    终点目标 yaw（rad），函数结束后 path 末点会对齐该值
 * @param config      仿真参数
 * @return 调整后的路径点 yaw 数量（若 path 过短则返回 0）
 */
size_t enforceSteeringContinuityOnPath(
  nav_msgs::msg::Path & path,
  double goal_yaw,
  const SwerveSteeringConfig & config);

}  // namespace nav2_theta_star_planner

#endif  // NAV2_THETA_STAR_PLANNER__SWERVE_STEERING_MODEL_HPP_
