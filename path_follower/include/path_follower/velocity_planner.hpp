#ifndef PATH_FOLLOWER__VELOCITY_PLANNER_HPP_
#define PATH_FOLLOWER__VELOCITY_PLANNER_HPP_

#include <string>
#include <vector>

#include "path_follower/trajectory.hpp"

namespace path_follower
{
    /// 速度规划参数。
    struct VelocityPlannerParams
    {
        double max_linear_velocity{1.2};
        double min_linear_velocity{0.02}; // 规划起点/终点速度 = 底盘线速度死区
        double max_linear_acceleration{0.2};
        double max_linear_deceleration{0.2}; // 正值
        double max_linear_jerk{1.0};
        double max_angular_velocity{0.8};
        double max_angular_acceleration{0.35};
        double max_angular_deceleration{0.35}; // 正值
        double max_angular_jerk{1.0};
        double min_angular_velocity{0.02}; // 规划起点/终点角速度 = 底盘角速度死区
        double ruckig_control_dt{0.02}; // Ruckig 在线步进周期 s
        bool curvature_deceleration{true};
        // 曲率减速软钳：dec_ratio = max(ratio_min, gain / (w² + gain))，w = linear_v * curvature。
        // gain 是软钳的特征角速度平方：w = sqrt(gain) 处 ratio 恰为 0.5，打折后的角速度
        // 峰值为 sqrt(gain)/2（gain=0.2 -> 0.224 rad/s，gain=0.3 -> 0.274 rad/s），
        // 调大 gain = 允许过弯更快。ratio_min 是打折比例的下限（见 legacyCurvatureClamp）。
        double curvature_dec_gain{0.3};
        double curvature_dec_ratio_min{0.2};
        bool acceleration_envelope{true}; // 曲率限速后的加减速包络（双扫描）

        // 速度包络模式：
        //   "legacy" —— 曲率软钳 + 加减速双扫描（历史行为，逐字保留）
        //   "toppra" —— 物理包络 v <= min(v_ruckig, max_angular_velocity/|kappa|, 角加速度上界)
        //               + 含角加速度约束的前后向扫描。曲率必须来自上游解析切向
        //               （见 PathFollowerNode::inputHeadingUsable），否则角加速度约束
        //               会自动关闭：三点外接圆算出的 kappa 再差分一次已是纯噪声。
        std::string velocity_profile_mode{"legacy"};
        // 角加速度约束的安全余量系数，作用于 max_angular_acceleration。
        // kappa' 在贝塞尔接入/接出点是**跳变**的，任何有限差分都会把阶跃抹成一个 ds 宽的
        // 斜坡，误差约为跳变幅度的一半且**不随加密收敛**（5mm 与 1cm 的 kappa' 误差只差
        // 1.13 倍）。实测 1cm 离散下端到端角加速度峰值超限约 20%，故默认留 0.8 的余量。
        // 要彻底消掉这一项，需要上游把 kappa' 也解析下发。
        double angular_accel_margin{0.8};
        double trajectory_resolution{0.005}; // 插值间距 m
        double angle_resolution{0.0034906585}; // 朝向离散角度间隔 rad（0.2°）
    };

    /**
 * @brief 基于 Ruckig 的 jerk 限制速度规划器（仅做整段全局规划）。
 *
 * 位置与朝向两条轨迹解耦、各自独立规划：
 *   planPosition(raw_points)：计算每点 current_s/切向/曲率，再用 Ruckig 从
 *     (min_linear_velocity, 0) 规划到 (终点, min_linear_velocity)，最后叠曲率减速。
 *   planHeading(start_theta, goal_theta)：按 angle_resolution 离散 [start, goal]，
 *     再用 Ruckig 从 (min_angular_velocity, 0) 规划到 (终点, min_angular_velocity)。
 *
 * 这两个是路径下发时的一次性入口，产出"弧长 -> 参考速度"的映射并烤进 route.points。
 * 规划完成后这条曲线全程只读、不再被改写。
 *
 * 注意：障碍触发的减速/停止**不在这里**处理，也不再对轨迹做中途重规划——
 * 那部分改由 velocity_step_model.hpp 的两阶段步进模型基于上一帧输出速度实时递推
 * （见 VelocityTrackMode 注释）。历史上的 replan*Velocity/replan*VelocityStop 系列
 * 接口已随之删除：它们把"当前速度"寄存在轨迹数组里当种子，导致曲率减速反复复合、
 * Ruckig 边界输入下不收敛、STOP 截断值泄漏进后续规划等一系列问题。
 */
    class VelocityPlanner
    {
    public:
        VelocityPlanner() = default;
        explicit VelocityPlanner(const VelocityPlannerParams& params);

        void setParams(const VelocityPlannerParams& params) { params_ = params; }
        const VelocityPlannerParams& params() const { return params_; }

        /**
   * @brief 位置轨迹规划：原始点 → 弧长/切向/曲率/速度（kInit 整段规划，起点种子固定）。
   * @param raw_points 原始位置点（x,y；trust_input_heading=true 时 heading_angle 已由调用方填好）
   * @param trust_input_heading true = 采信 raw_points 里带来的朝向（上游解析切向），
   *        曲率按 kappa = dtheta/ds 求；false = 从 x,y 自行算切向与三点圆曲率。
   *        判定逻辑在 PathFollowerNode::inputHeadingUsable。
   * @return 规划好的 Route（含 current_s/heading_angle/curvature/linear_v/linear_a/curvature_omega）
   */
        Route planPosition(const std::vector<TrajectoryPoint>& raw_points,
                           bool trust_input_heading = false) const;

        /**
   * @brief 朝向轨迹规划：起点→终点朝向，按 angle_resolution 离散（kInit 整段规划）。
   * @param start_theta 起点朝向 rad
   * @param goal_theta  终点朝向 rad
   * @return 独立 HeadingRoute（含 theta/current_a/heading_omega/heading_alpha）
   */
        HeadingRoute planHeading(double start_theta, double goal_theta) const;

    private:
        /// 计算每点弧长 current_s；切向 heading_angle 按 trust_input_heading 决定来源：
        /// false = 取"指向下一点"的弦向（末点抄倒数第二点）；
        /// true  = 沿用调用方填入的解析切向，仅做 unwrap 使其跨 ±pi 连续
        ///         （后续 kappa = dtheta/ds 的差分依赖这个连续性）。
        void computeArcLengthAndHeading(Route& route, bool trust_input_heading) const;

        /// 计算每点曲率并写入 route。
        /// from_heading=false：三点外接圆法，直接从 x,y 反推。位置扰动的放大系数是
        ///   4/ds^2（1cm 点距下约 4e4），0.1mm 的抖动就能算出量级和真实曲率相当的噪声，
        ///   这也是 legacyCurvatureClamp 需要 ratio_min 兜底的根本原因。
        /// from_heading=true ：kappa = dtheta/ds 中心差分，放大系数只有 1/ds（约 1e2），
        ///   前提是 heading_angle 已被 computeArcLengthAndHeading unwrap 成连续量。
        void computeCurvature(Route& route, bool from_heading) const;

        /// legacy 分支：曲率软钳 + 加减速双扫描（实现见 velocity_envelope.cpp）。
        void applyLegacyProfile(Route& route) const;

        /// toppra 分支：物理包络 + 含角加速度约束的前后向扫描（实现见 velocity_envelope.cpp）。
        /// @param kappa_trustworthy 曲率是否来自解析切向；false 时不施加角加速度约束。
        void applyToppraProfile(Route& route, bool kappa_trustworthy) const;


        /// 整段位置速度填充（planPosition 专用）：Ruckig 从 seed 规划到终点得到"只看弧长"的
        /// 全局速度剖面，再按 velocity_profile_mode 交给 applyLegacyProfile 或
        /// applyToppraProfile 压成最终包络（两者都只会往下压，不会超过 Ruckig 的值）。
        /// 只在路径下发时调用一次，故不需要 from_index/STOP 截断/参数覆盖这些中途重规划的概念。
        void fillPositionVelocityProfile(Route& route, const RuckigSeed& seed,
                                         bool kappa_trustworthy) const;

        /// 整段朝向角速度填充（planHeading 专用），语义同 fillPositionVelocityProfile。
        /// current_a 是有符号累计角度，内部用 sign 转成无符号幅值空间喂给 Ruckig。
        void fillHeadingVelocityProfile(HeadingRoute& route, const RuckigSeed& seed) const;

        VelocityPlannerParams params_;
    };
} // namespace path_follower

#endif  // PATH_FOLLOWER__VELOCITY_PLANNER_HPP_
