#ifndef PATH_FOLLOWER__PID_CONTROLLER_HPP_
#define PATH_FOLLOWER__PID_CONTROLLER_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "path_follower/trajectory.hpp"

namespace path_follower
{
    /// 10 级增益档位数量。
    constexpr std::size_t kGainLevelCount = 10;

    /// 单轴 PID 增益表（10 级，按当前速度选档）。
    struct PidGainTable
    {
        std::array<double, kGainLevelCount> kp{};
        std::array<double, kGainLevelCount> ki{};
        std::array<double, kGainLevelCount> kd{};
    };

    /// 单轴 PID 配置：增益表 + 输出限幅 + 速度分档阈值。
    struct PidAxisConfig
    {
        PidGainTable gains;
        double output_limit{0.0}; // 输出绝对值上限
        std::array<double, kGainLevelCount> vel_levels{}; // 速度分档 m/s
    };

    /// 跟踪误差（轨迹坐标系）。
    struct TrackingError
    {
        double delta_x{0.0}; // 纵向剩余（投影点到前视点的有符号距离，未到为负）m
        double delta_y{0.0}; // 横向偏差（点在线左侧为正）m
        double delta_theta{0.0}; // 航向偏差（左偏为负、右偏为正）rad
    };

    /// 纠偏速度输出（轨迹坐标系）。
    struct CorrectionVelocity
    {
        double vx{0.0};
        double vy{0.0};
        double wz{0.0};
    };

    /**
     * @brief 增量式混合 PID 控制器（3 轴 × 10 级增益调度）。
     *
     * 忠实复现原项目 GetCorrectSpeed 的增量式控制律：
     *   Δout = sign * (Kp·Δe + Kd·Δ²e) + Ki·e
     *   out += Δout ; clamp(out, ±output_limit)
     * - 横向 y: sign = -1
     * - 航向 θ: sign = +1
     * - 纵向 x: 位置式前馈式跟踪（外层在 PathTracker 中以参考速度前馈处理）
     *
     * 切换增益档位时不清零积分（输出累积天然保持积分项）。
     * 差速底盘（无 vy）由 PathTracker 用 FollowPath 非线性律把横向误差折叠到航向。
     */
    class PidController
    {
    public:
        PidController() = default;
        ~PidController() = default;

        /// 配置三轴增益表与限幅。vel_levels 为速度分档（共 10 档）。
        void configure(
            const PidAxisConfig& x_cfg,
            const PidAxisConfig& y_cfg,
            const PidAxisConfig& theta_cfg,
            const std::array<double, kGainLevelCount>& vel_levels,
            double max_correct_vy,
            double max_correct_w);

        /// 复位所有积分状态（新轨迹/到点时调用）。
        void reset();

        /// 根据当前线速度选择档位索引（0..9）。
        std::size_t selectLevel(double current_linear_velocity) const;

        /**
   * @brief 计算纠偏速度（轨迹坐标系）。
   * @param error 跟踪误差
   * @param current_linear_velocity 当前线速度（用于选档）m/s
   * @param reference_linear_v 参考线速度（来自速度规划，前馈）m/s
   * @param reference_w 参考角速度（来自速度规划）rad/s
   * @param have_vy 是否有横向自由度（全向 true / 差速 false）
   * @param diff_lateral_gain 差速模式下横向误差折叠到航向的增益
   * @param max_percentage_vy_to_vx Vy 占 Vx 最大比例
   * @return 纠偏速度（轨迹坐标系）
   */
        CorrectionVelocity compute(
            const TrackingError& error,
            double current_linear_velocity,
            double reference_linear_v,
            double reference_w,
            bool have_vy,
            double diff_lateral_gain,
            double max_percentage_vy_to_vx) const;

    private:
        /// 单轴增量式 PID 运行时状态。
        struct AxisState
        {
            double out{0.0}; // 累积输出（含积分）
            double err0{0.0}; // e(k)
            double err1{0.0}; // e(k-1)
            double err2{0.0}; // e(k-2)
        };

        /// 单轴一步增量式计算。
        static double stepIncrement(
            const AxisState& prev,
            const PidGainTable& gains,
            std::size_t level,
            double error);

        PidAxisConfig x_cfg_{};
        PidAxisConfig y_cfg_{};
        PidAxisConfig theta_cfg_{};
        std::array<double, kGainLevelCount> vel_levels_{};
        double max_correct_vy_{0.4};
        double max_correct_w_{0.3};

        mutable AxisState y_state_{};
        mutable AxisState theta_state_{};
    };
} // namespace path_follower

#endif  // PATH_FOLLOWER__PID_CONTROLLER_HPP_