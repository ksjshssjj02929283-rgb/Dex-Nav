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
   * @param raw_points 原始位置点（x,y）
   * @return 规划好的 Route（含 current_s/heading_angle/curvature/linear_v/linear_a/curvature_omega）
   */
        Route planPosition(const std::vector<TrajectoryPoint>& raw_points) const;

        /**
   * @brief 朝向轨迹规划：起点→终点朝向，按 angle_resolution 离散（kInit 整段规划）。
   * @param start_theta 起点朝向 rad
   * @param goal_theta  终点朝向 rad
   * @return 独立 HeadingRoute（含 theta/current_a/heading_omega/heading_alpha）
   */
        HeadingRoute planHeading(double start_theta, double goal_theta) const;

    private:
        /// 计算每点弧长 current_s 与切向 heading_angle，写入 route。
        void computeArcLengthAndHeading(Route& route) const;

        /// 计算每点曲率（三点圆法），写入 route。
        void computeCurvature(Route& route) const;

        /// 曲率减速：dec_ratio = max(0.2, 0.2/(w^2+0.2))，仅缩放 [from_index, end) 的
        /// linear_v/curvature_omega/linear_a（linear_a 同比例缩放以保持与 linear_v 数值自洽，
        /// 供后续重规划取种子）。
        void applyCurvatureDeceleration(Route& route, std::size_t from_index) const;

        /// 整段位置速度填充（planPosition 专用）：Ruckig 从 seed 规划到终点，写入所有点的
        /// linear_v/linear_a，再按 curvature_deceleration 开关叠曲率减速 / 直接算 curvature_omega。
        /// 只在路径下发时调用一次，故不需要 from_index/STOP 截断/参数覆盖这些中途重规划的概念。
        void fillPositionVelocityProfile(Route& route, const RuckigSeed& seed) const;

        /// 整段朝向角速度填充（planHeading 专用），语义同 fillPositionVelocityProfile。
        /// current_a 是有符号累计角度，内部用 sign 转成无符号幅值空间喂给 Ruckig。
        void fillHeadingVelocityProfile(HeadingRoute& route, const RuckigSeed& seed) const;

        VelocityPlannerParams params_;
    };
} // namespace path_follower

#endif  // PATH_FOLLOWER__VELOCITY_PLANNER_HPP_
