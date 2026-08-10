// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 剖面导数回填：由已生成的 (accel, rel_time) 列统一算出 jerk 列。
#ifndef NAV2_SPEED_PLANNER__CORE__PROFILE_DERIVATIVES_HPP_
#define NAV2_SPEED_PLANNER__CORE__PROFILE_DERIVATIVES_HPP_

#include <vector>

#include "nav2_speed_planner/core/velocity_envelope.hpp"

namespace nav2_speed_planner
{

/// 由加速度列对时间求差分，回填 jerk 列。
///
/// **为什么必须统一在这里算**：jerk 不是任何后端的原生输出。求解型后端以它
/// 为控制输入却不导出逐点值；滤波型后端根本没有这个变量。此前两者都把
/// `TrajectoryPoint::jerk` 留空，实测全 252 行恒 0——于是校验里那条
/// `|jerk| ≤ 限值` 读到的永远是 `0 ≤ 0.5`，**恒真**。整套离线校验从未真正
/// 检查过 jerk，剖面在巡航→制动交界处实际达 −8.6 仍全绿。
///
/// 统一算子还保证不同后端的 jerk 口径可比：同一个差分、同一处采样。
///
/// **取后向差分而非中心差分**：本剖面在节点处是加速度台阶，中心差分会把
/// 一次跳变摊到两个区间上、峰值减半，正好掩盖要找的那类缺陷。后向差分
/// 直接度量"跨过这个节点时加速度变了多少、用了多久"，与滤波器将要施加的
/// 约束 `|Δa| ≤ j·Δt` 是同一个量，改动有没有效果才能对上号。
///
/// 首点无前驱，置 0；`Δt` 退化（零长段 / 未填 rel_time 的后端）同样置 0——
/// 0/0 的差分会污染整列，而列一旦有 NaN，所有 `x ≤ 限值` 形式的断言都会被
/// 静默绕过。
inline void fillJerkFromAccel(std::vector<TrajPointWork> & work)
{
  if (work.empty()) {
    return;
  }
  work.front().jerk = 0.0;
  for (size_t i = 1; i < work.size(); ++i) {
    const double dt = work[i].rel_time - work[i - 1].rel_time;
    work[i].jerk = (dt > 1e-9) ? (work[i].accel - work[i - 1].accel) / dt : 0.0;
  }
}

}  // namespace nav2_speed_planner

#endif  // NAV2_SPEED_PLANNER__CORE__PROFILE_DERIVATIVES_HPP_
