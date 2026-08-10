// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 障碍约束的公共类型：查询三态、不可评估时的处置策略、产出。
#ifndef NAV2_SPEED_PLANNER__CONSTRAINTS__OBSTACLE_CONSTRAINT_HPP_
#define NAV2_SPEED_PLANNER__CONSTRAINTS__OBSTACLE_CONSTRAINT_HPP_

#include <cstddef>
#include <string>

#include "nav2_speed_planner/core/velocity_envelope.hpp"

namespace nav2_speed_planner
{

enum class ConstraintFailurePolicy
{
  /// 本周期速度规划失败（输出空轨迹），由 ControllerServer 发布零速。生产默认。
  FAIL_CLOSED,
  /// 跳过该约束继续规划。仅允许仿真/明确灰度，且必须有 diagnostic。
  FAIL_OPEN
};

/**
 * @enum PointObstacleStatus
 * @brief 单点障碍判定的三态。
 *
 * ⚠️ `OUT_OF_RANGE` 不是失败：local costmap 是 ±4m 的 rolling window，
 * 路径上超出覆盖范围的点属正常情况（footprint 半对角 0.495m → 约 3.5m 之后即出图）。
 * 若把它当作"不可评估"并触发 FAIL_CLOSED，任何长于约 3.5m 的路径都会让机器人原地不动。
 * 它只计数、只上报，不参与策略判定——但计数本身很重要：R47 的幻影障碍
 * （出图被 nav2 判成 LETHAL）当初若有这个计数，一眼就能看出"3.5m 之后全是出图"。
 */
enum class PointObstacleStatus
{
  FREE,          ///< 查过，无障碍
  OBSTACLE,      ///< 查过，有障碍
  OUT_OF_RANGE   ///< 超出 costmap 覆盖，无信息（正常，不判障、不触发策略）
};

/**
 * @struct ObstacleConstraintOutcome
 * @brief A1 障碍约束的结果——把"有没有障碍"与"能不能评估"分开表达。
 */
struct ObstacleConstraintOutcome
{
  bool has_obstacle{false};        ///< 是否检测到障碍（仅在 evaluable 时有意义）
  bool evaluable{true};            ///< false = 本周期无法评估（TF/costmap 不可用）
  std::string unavailable_reason;  ///< evaluable=false 时的机器可读原因
  size_t scanned_points{0};        ///< 实际完成查询的点数
  size_t out_of_range_points{0};   ///< 超出 costmap 覆盖的点数（观测用）
  VelocityConstraint constraint;   ///< 障碍点及其前方缓冲带的逐点上界
};

}  // namespace nav2_speed_planner

#endif  // NAV2_SPEED_PLANNER__CONSTRAINTS__OBSTACLE_CONSTRAINT_HPP_
