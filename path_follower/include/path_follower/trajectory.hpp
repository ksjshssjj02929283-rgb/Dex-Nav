#ifndef PATH_FOLLOWER__TRAJECTORY_HPP_
#define PATH_FOLLOWER__TRAJECTORY_HPP_

#include <cstddef>
#include <limits>
#include <vector>

namespace path_follower
{
    /// 路径模式。原项目支持自旋与离散曲线；简化后跟踪统一为离散曲线，
/// 但保留枚举以兼容 spin 到点场景。
    enum class RouteMode
    {
        kDispersedCurve = 0, // 离散点轨迹（一般直线/曲线）
        kSpin = 1, // 原地自旋到点
    };

    /// 三阶段状态机阶段（diff 与 omni 平移共用）。
    /// - kAlignToPath: 阶段一，原地旋转对准起始目标（diff=轨迹起始切向; omni平移=holdDirection）。
    /// - kFollowTrack: 阶段二，沿轨迹跟随。
    /// - kAlignToGoal: 阶段三，到点后旋转到 target_yaw（无 target_yaw 则跳过）。
    /// omni 平移+旋转（mode 3）不用此状态机，单阶段双路并行。
    enum class TrackPhase
    {
        kAlignToPath = 0, // 阶段一：原地旋转对准轨迹起始切向
        kFollowTrack = 1, // 阶段二：沿轨迹跟随，仅输出 vx（vy≡0）
        kAlignToGoal = 2, // 阶段三：到点后原地旋转到目标朝向
    };

    /// 轨迹点（位置轨迹坐标系）。仅承载位置与切向/线速度信息；
    /// 朝向跟踪量（theta/heading_omega）已解耦到独立 HeadingRoute。
    struct TrajectoryPoint
    {
        double x{0.0}; // 位姿 m
        double y{0.0}; // 位姿 m
        double heading_angle{0.0}; // 段航向 rad（切向，用于轨迹系→车体系旋转）
        double linear_v{0.0}; // 参考线速度 m/s（速度规划烤入）
        double linear_a{0.0}; // 参考线加速度 m/s²（Ruckig 规划烤入；障碍触发重规划时作为 current_acceleration 种子）
        double curvature_omega{0.0}; // 曲率角速度 rad/s（= linear_v × curvature，规划烤入）
        double curvature{0.0}; // 曲率 1/m
        double current_s{0.0}; // 距起点弧长 m
    };

    /// 一条轨迹。对应原项目 A_Model_Route。
    struct Route
    {
        std::vector<TrajectoryPoint> points;
        RouteMode mode{RouteMode::kDispersedCurve};
        double total_s{0.0}; // 总弧长 m

        bool empty() const { return points.empty(); }
        std::size_t size() const { return points.size(); }
    };

    /// 朝向轨迹点（独立于位置轨迹，按角度间隔离散）。
    struct HeadingPoint
    {
        double theta{0.0}; // 目标朝向 rad（unwrap 单调，跨 ±π 不跳变）
        double current_a{0.0}; // 距起点累计角度 rad（有符号，梯形“距离”）
        double heading_omega{0.0}; // 参考角速度 rad/s（速度规划烤入）
        double heading_alpha{0.0}; // 参考角加速度 rad/s²（Ruckig 规划烤入；障碍触发重规划时作为 current_acceleration 种子）
    };

    /// 独立朝向轨迹：起点朝向 → 终点朝向，按 angle_resolution 离散。
    struct HeadingRoute
    {
        std::vector<HeadingPoint> points;
        double total_a{0.0}; // 总角度跨度 rad（有符号）

        bool empty() const { return points.empty(); }
        std::size_t size() const { return points.size(); }
    };

    /// 一次规划结果：位置轨迹 + 朝向轨迹 + 目标朝向。
    ///
    /// 朝向轨迹按跟踪模式语义不同：
    /// - diff：heading_align = 阶段一旋转轨迹（current_theta -> path_start_heading）；
    ///   heading_final = 阶段三旋转轨迹（path_end_heading -> goal_theta，onPath 预规划）；
    ///   阶段二航向由位置轨迹切向给出，不查朝向轨迹。
    /// - omni 平移（mode 2）：heading_align = 阶段一旋转轨迹（start_theta -> hold_direction）；
    ///   heading_final = 阶段三旋转轨迹（hold_direction -> target_yaw，有 target_yaw 时预规划）；
    ///   阶段二朝向锁定 hold_direction，不查朝向轨迹。
    /// - omni 平移+旋转（mode 3）：仅用 heading_align（start_theta -> goal_theta 并行跟踪），
    ///   heading_final 不用，位置与朝向并行，单阶段。
    struct PlannedRoute
    {
        Route position; // 5mm 位置轨迹（无朝向，但含切向 heading_angle）
        HeadingRoute heading_align; // omni: start→goal 并行朝向；diff: 阶段一旋转
        HeadingRoute heading_final; // diff 阶段三旋转（omni 不用，留空）
        double goal_theta{0.0}; // 末点目标朝向 rad（到点判定用）
        double path_start_heading{0.0}; // 轨迹起始切向 rad（diff 阶段一对准目标）
        double path_end_heading{0.0}; // 轨迹末点切向 rad（diff 阶段三规划锚点）
    };

    /// 二维位姿（用于当前机器人状态）。
    struct Pose2D
    {
        double x{0.0};
        double y{0.0};
        double theta{0.0};
    };

    /// 跟踪方式（对应 nav_interfaces/TrackingPath.mode）。
    enum class TrackingMode
    {
        kDiff = 1,               // 差速：vy 折入 w（三阶段状态机）
        kOmniTranslation = 2,     // 全向平移：保持车体朝向不变，纯平移
        kOmniTransRotation = 3,   // 全向一边平移一边旋转
    };

    /// 段级跟踪参数：随路径一起下发，控制环据此分派与限幅。
    /// onPath 由节点参数（base_type/enable_reverse）填充；
    /// onTrackingPath 由 nav_interfaces/TrackingPath 消息字段填充。
    /// 控制环统一从本结构读取，避免节点级成员与段级覆盖混淆。
    struct SegmentProfile
    {
        TrackingMode mode{TrackingMode::kOmniTransRotation};
        bool enable_reverse{false};        // 仅 kDiff：倒车（对准切线反向、vx 取负）
        bool has_target_yaw{true};         // false=到点不做末段旋转（diff 跳过阶段三）
        double target_yaw{0.0};            // 到点目标朝向 rad（has_target_yaw=true 时有效）
        double hold_direction{std::numeric_limits<double>::quiet_NaN()}; // kOmniTranslation 保持的朝向 rad；NaN=保持当前朝向
        double max_speed{std::numeric_limits<double>::quiet_NaN()};      // 段限速 m/s；NaN=默认上限
        double padding{std::numeric_limits<double>::quiet_NaN()};        // 车体膨胀 m；仅记录，path_follower 暂不消费（留给 collision_monitor）
    };

    /// 速度（车体坐标系 vx, vy, w）。
    struct Velocity
    {
        double vx{0.0};
        double vy{0.0};
        double wz{0.0};
    };

    /// 跟踪结果码（对应原 RETURNVALUE）。
    enum class TrackResult
    {
        kNormalNotDone = 0, // 正常跟踪中，未完成
        kReached = 1, // 到点
        kTrackError = -1, // 跟踪异常（误差超限等）
        kVelNan = -2, // 速度非法
    };

    /// 速度跟踪状态机（与 nav_interfaces/ObstacleMonitorStatus 对应）。
    ///
    /// 决定"这一帧的参考速度从哪来"：kFollowRef 直接读轨迹里烤入的全局规划速度；
    /// 其余三态都不读轨迹速度，改由两阶段步进模型（velocity_step_model.hpp）基于
    /// **上一帧输出速度** 实时递推，只是目标速度不同。
    /// - kFollowRef：对应 FREE 稳态，直接跟踪全局速度规划的参考速度，模型不介入。
    /// - kSlowdown：对应 SLOWDOWN，目标速度 = min(参考速度, 最大速度/2)（取上限语义，
    ///   已经比 max/2 慢就不会反过来把车加速）。
    /// - kStop：对应 STOP，目标速度 = 0。
    /// - kResume：障碍解除后的恢复段，目标速度 = 当前参考速度；模型输出收敛到参考速度
    ///   附近后自动退回 kFollowRef。缺了这一态，解除瞬间会从"模型输出的低速"直接跳回
    ///   "轨迹参考速度"，产生速度断崖。
    enum class VelocityTrackMode
    {
        kFollowRef = 0,
        kSlowdown = 1,
        kStop = 2,
        kResume = 3,
    };

    /// Ruckig 规划种子（起点速度+起点加速度），整段速度规划（planPosition/planHeading）用。
    struct RuckigSeed
    {
        double v{0.0};
        double a{0.0};
    };
} // namespace path_follower

#endif  // PATH_FOLLOWER__TRAJECTORY_HPP_