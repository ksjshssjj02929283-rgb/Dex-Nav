#ifndef PATH_FOLLOWER__VELOCITY_STEP_MODEL_HPP_
#define PATH_FOLLOWER__VELOCITY_STEP_MODEL_HPP_

namespace path_follower
{
    /// 两阶段速度步进模型参数（线速度/角速度各配一份，量纲自洽即可）。
    struct VelocityStepParams
    {
        double max_accel{0.2}; // 提速时的加速度幅值上限（正值）
        double max_decel{0.2}; // 降速时的减速度幅值上限（正值）
        double accel_decay_time{0.3}; // 加速度衰减时间 T：|a| 由上限均匀衰减到 0 的时长 s
    };

    /**
     * @brief 两阶段速度步进：从 v_prev 朝 v_target 走一个控制周期 dt，返回本帧速度。
     *
     * 完全无状态、闭式求解，只依赖"上一帧速度"，不读轨迹、不关心走了多远，因此不存在
     * "轨迹里烤入的参考速度被反复重规划污染"这类问题（这正是它取代 Ruckig 中途重规划的原因）。
     *
     * jerk 不单独配置，而是由"加速度衰减时间"派生：jerk = max_a / T，即约定衰减段内
     * 加速度幅值从 max_a 均匀降到 0 恰好用掉 T 秒。于是衰减段本身能吃掉的速度差为
     *   delta_v = 0.5 * max_a * T   （a-t 图上的三角形面积，等价于 max_a^2 / (2*jerk)）
     *
     * 两阶段：
     *   阶段A（|v_target - v_prev| > delta_v）：还离得远，直接用最大加/减速度硬顶，尽快逼近；
     *   阶段B（|v_target - v_prev| <= delta_v）：进入衰减段，用能量关系反解当前该有的加速度
     *     a = sqrt(2 * jerk * |diff|)
     *   两段在边界处严格连续（|diff| = delta_v 时 a 恰为 max_a），且 |diff| -> 0 时 a -> 0，
     *   收尾自然平滑——这正是"快速减速 + 快到 0 时才考虑加速度平滑"的设计意图。
     *
     * 注意本函数是通用的加/减速模型，不只用于减速：v_target 高于 v_prev 时按 max_accel 提速，
     * 障碍解除后的"恢复到参考速度"阶段走的就是这条路径。
     *
     * @param v_prev   上一帧输出速度（带符号；倒车/反向旋转场景为负）
     * @param v_target 本帧目标速度（带符号）
     * @param dt       控制周期 s
     * @param params   加/减速度上限与加速度衰减时间
     * @return 本帧速度（带符号），保证不越过 v_target
     */
    double stepVelocityTowards(
        double v_prev, double v_target, double dt, const VelocityStepParams& params);
} // namespace path_follower

#endif  // PATH_FOLLOWER__VELOCITY_STEP_MODEL_HPP_
