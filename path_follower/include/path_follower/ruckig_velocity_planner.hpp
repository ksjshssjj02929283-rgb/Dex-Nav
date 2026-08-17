#ifndef PATH_FOLLOWER__RUCKIG_VELOCITY_PLANNER_HPP_
#define PATH_FOLLOWER__RUCKIG_VELOCITY_PLANNER_HPP_

#include <vector>

#include "path_follower/trajectory.hpp"

namespace path_follower
{
    /**
 * @brief 沿一维单调"距离"序列（弧长或累计角度）做 jerk 限制速度规划（纯算法，不依赖
 *        rclcpp/节点状态，便于单测与在控制环里反复调用）。
 *
 * 用距离本身作为 Ruckig 的 1-DOF position，一次步进即拿到 (s(t), v(t), a(t)) 原生配对，
 * 按 dist 网格线性插值回填，避免额外做 v(t)->s(t) 积分/反解。
 *
 * @param dist     目标区间的距离序列，单调递增，dist.front() 对应 seed 所在位置（视为 0 基准，
 *                 调用方需自行减去起点偏移）
 * @param seed     起点速度/加速度
 * @param v_max    速度上限（Ruckig max_velocity）
 * @param v_target 终点目标速度
 * @param a_max    最大加速度（正值）
 * @param d_max    最大减速度幅值（正值，作为 min_acceleration=-d_max）
 * @param jerk_max jerk 上限（正值）
 * @param ctrl_dt  在线步进控制周期 s
 * @param v_out    输出：按 dist 插值回填的速度序列，size()==dist.size()
 * @param a_out    输出：按 dist 插值回填的加速度序列，size()==dist.size()
 * @return 是否规划成功（false：Ruckig 报错，v_out/a_out 退化为全 seed.v/seed.a）
 */
    bool planRuckigVelocityProfile(
        const std::vector<double>& dist,
        const RuckigSeed& seed,
        double v_max, double v_target,
        double a_max, double d_max, double jerk_max,
        double ctrl_dt,
        std::vector<double>& v_out,
        std::vector<double>& a_out);
} // namespace path_follower

#endif  // PATH_FOLLOWER__RUCKIG_VELOCITY_PLANNER_HPP_
