#ifndef PATH_FOLLOWER__PATH_FOLLOWER_HPP_
#define PATH_FOLLOWER__PATH_FOLLOWER_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_interfaces/msg/navigation_status.hpp>
#include <nav_interfaces/msg/obstacle_monitor_status.hpp>
#include <nav_interfaces/msg/tracking_path.hpp>
#include <nav_interfaces/action/steering_angle_control.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "path_follower/pid_controller.hpp"
#include "path_follower/trajectory.hpp"
#include "path_follower/velocity_planner.hpp"
#include "path_follower/velocity_step_model.hpp"

namespace path_follower
{
    /// 底盘类型 / 默认跟踪方式（onPath 据此设段 mode；onTrackingPath 由消息 mode 覆盖）。
    enum class BaseType
    {
        kDiff = 0,              // 差速：vy 折入 w（三阶段状态机）
        kOmniTranslation = 1,   // 全向平移：保持车体朝向（三阶段，阶段一/三目标=hold/target）
        kOmniTransRotation = 2, // 全向平移+旋转：位置与朝向并行（单阶段双路）
    };

    /**
     * @brief PID 轨迹跟踪节点（全向为主，预留差速）。
     *
     * 控制环（每周期）：
     *   1) 取最新里程计位姿/速度；
     *   2) 在轨迹中找最近点 i（窗口搜索、禁后退）；
     *   3) 前视点 = OffsetPointIndex（至少前移 tracking_num 个点 + |v|·dt）；
     *   4) 对 i→前视点 线段投影，得 Δx/Δy/Δθ（CalcLinearVectorDelta）；
     *   5) 判到点（Δx ≥ −reach_distance）；
     *   6) VelProjection 取参考 linear_v/curvature_omega，叠曲率减速；
     *   7) PidController 增量式纠偏（10 级增益按速度选档）；
     *   8) 合成 traj_v = {v_ref, vy_corr, w_corr + w_ref}；
     *   9) 低速 Vy/Vx 比例限幅；
     *  10) trajVelToAgvVel 旋到车体系，发布 cmd_vel。
    */
    class PathFollowerNode : public rclcpp::Node
    {
    public:
        explicit PathFollowerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~PathFollowerNode() override = default;

        /// 控制周期更新（由 wall_timer 驱动）。
        void update();

    private:
        // ---------------- 控制环分派（按底盘类型）----------------
        /// omni 平移+旋转（mode 3）：位置与朝向并行跟踪（单阶段双路）。
        void updateOmni(
            const Pose2D& pose, const Velocity& vel, const PlannedRoute& route,
            const SegmentProfile& segment,
            const std::chrono::steady_clock::time_point& now_steady);

        /// omni 平移（mode 2）：三阶段状态机（与 diff 同构，阶段一/三目标不同）。
        void updateOmniTranslation(
            const Pose2D& pose, const Velocity& vel, const PlannedRoute& route,
            const SegmentProfile& segment,
            const std::chrono::steady_clock::time_point& now_steady);

        /// diff 三阶段状态机分派。
        void updateDiff(
            const Pose2D& pose, const Velocity& vel, const PlannedRoute& route,
            const SegmentProfile& segment,
            const std::chrono::steady_clock::time_point& now_steady);

        /// diff 阶段一：原地旋转对准轨迹起始切向。返回 cmd_vel。
        Velocity diffPhaseAlignToPath(
            const Pose2D& pose, const PlannedRoute& route, const SegmentProfile& segment);

        /// diff 阶段二：沿轨迹跟随，仅输出 vx。返回 cmd_vel。
        Velocity diffPhaseFollowTrack(
            const Pose2D& pose, const Velocity& vel, const PlannedRoute& route,
            const SegmentProfile& segment);

        /// diff 阶段三：到点后原地旋转到目标朝向。返回 cmd_vel。
        /// omni 平移阶段三复用本函数（目标同为 target_yaw）。
        Velocity diffPhaseAlignToGoal(
            const Pose2D& pose, const PlannedRoute& route, const SegmentProfile& segment);

        /// omni 平移阶段一：原地旋转到 hold_direction。返回 cmd_vel。
        Velocity omniTransPhaseAlignHold(
            const Pose2D& pose, const PlannedRoute& route, const SegmentProfile& segment);

        /// omni 平移阶段二：沿轨迹平移，车体朝向锁定 hold_direction。返回 cmd_vel。
        Velocity omniTransPhaseFollowTrack(
            const Pose2D& pose, const Velocity& vel, const PlannedRoute& route,
            const SegmentProfile& segment);

        // ---------------- 参数声明与回调 ----------------
        void declareAllParameters();
        /// add_on_set_parameters_callback：校验后即时生效。
        rcl_interfaces::msg::SetParametersResult onParametersChanged(
            const std::vector<rclcpp::Parameter>& parameters);

        // ---------------- 订阅回调 ----------------
        void onPath(const nav_msgs::msg::Path::SharedPtr msg);
        void onTrackingPath(const nav_interfaces::msg::TrackingPath::SharedPtr msg);
        void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
        /// collision_monitor 实时障碍状态：只存最新值，判定放到控制环里做。
        void onObstacleStatus(const nav_interfaces::msg::ObstacleMonitorStatus::SharedPtr msg);

        /// 控制环每周期调用：把最新障碍状态映射成速度跟踪状态 vel_track_mode_
        /// （含 STOP/SLOWDOWN 解除的 1s 迟滞）。只切状态，不碰轨迹——障碍减速/停止/恢复
        /// 全部由 stepTrackedLinearVelocity/stepTrackedAngularVelocity 逐帧递推实现。
        void updateVelocityTrackMode();

        /// 线速度通道：按 vel_track_mode_ 决定本帧参考线速度。
        /// kFollowRef 直接返回轨迹参考速度；其余三态用两阶段步进模型从 last_track_linear_v_
        /// 递推（目标分别是 min(ref, max/2) / 0 / ref）。kResume 下输出收敛到 route_ref_v
        /// 附近时，顺带把 vel_track_mode_ 退回 kFollowRef（模式切换统一由本通道驱动）。
        /// @param route_ref_v 轨迹给出的参考线速度（已含倒车取负、段限速处理）
        double stepTrackedLinearVelocity(double route_ref_v);

        /// 角速度通道（独立朝向规划，仅 updateOmni 的朝向维度用）：语义同上，
        /// 但不参与 kResume->kFollowRef 的模式切换判定，只跟随当前模式。
        double stepTrackedAngularVelocity(double route_ref_w);

        /// 每条新路线/阶段切换时复位速度跟踪状态与上一帧速度记忆。
        /// 刻意不触碰障碍状态看门狗的基准时间——原因见实现处注释。
        void resetVelocityTrack();

        /// 障碍状态距今多久没更新（s）。节点启动后从未收到过，则返回自启动以来的时长，
        /// 即"从未收到"等价于"一直陈旧"，这正是需要的语义。
        double obstacleStatusAge() const;

        /// 把 [block_begin, now] 这段"执行器被同步等待独占、自己的订阅回调压根跑不到"的
        /// 窗口从各看门狗的时间轴上剪掉（相关时间戳整体后移该时长），避免自阻塞被误判成
        /// 链路静默。覆盖障碍状态看门狗与 odom 接收年龄看门狗——两者基准都是"本地接收
        /// 时刻"，自阻塞对它们的污染方式完全一样。
        /// 必须在阻塞结束、回到事件循环之前调用。详见实现处注释。reason 仅用于日志。
        void excludeBlockedWindowFromWatchdogs(
            const rclcpp::Time& block_begin, const char* reason);

        /// odom 接收年龄看门狗（控制环每帧第一件事）。
        ///
        /// 判的是"距上一帧 odom 到达已经过去多久"，而不是相邻两帧的间隔——间隔只有等下一帧
        /// 真正到达才算得出来，那时车已经用过期位姿跑完了整个盲区，是事后补救而非拦截。
        ///
        /// 超限时不进 getCurrentState / PID / 坐标变换（位姿本来就过期了，拿它纠偏只会算出
        /// 错误的修正量），而是把上一帧真正发出去的**车体系**指令按两阶段步进模型等比衰减：
        /// 方向与转弯半径保持不变，模长按可用减速度收敛到 0。等比缩放而不是各维独立衰减，
        /// 是因为全向底盘直线运动时 vy 可能才是主分量，单独归零会让车横向甩一下。
        ///
        /// @return true 表示本帧已处理完毕（已发指令），控制环应直接返回
        bool guardOdomStale();

        /// 查 path_frame -> odom_frame 变换（定位跳变判据的旁路输入）。
        /// 与主路取位姿的 path_frame -> base_link 查询相互独立：判据支路将来整体摘除时，
        /// 主路一行都不用动。
        /// @param[out] map_odom 查得的变换（失败时其值无意义）
        /// @param[out] stamp 该变换的时间戳，用于定位链路静默看门狗
        /// @return 本帧是否查到
        bool lookupMapToOdom(Pose2D& map_odom, rclcpp::Time& stamp);

        // ---------------- 控制环子步骤 ----------------
        /// 从最新里程计/TF 取位姿/速度（线程安全）。
        /// @param allow_localization_switch 是否允许本次调用触发/使用末端 map->odom 定位切换；
        ///   buildAndLoadRoute 里为取新路线起点朝向而调用时传 false，避免误用上一路线目标点的锚定状态。
        std::optional<std::pair<Pose2D, Velocity>> getCurrentState(bool allow_localization_switch = true);

        /// 用锚定时刻的 (map_pose, odom_pose) 位姿对 + 当前 odom 位姿，估计当前 map 系位姿：
        /// 视 odom->map 为锚定时刻标定的恒定刚体变换，此后纯靠 odom 平滑递推。
        /// 末端定位切换与定位跳变兜底共用这一个递推公式，只是锚定对的来源不同。
        static Pose2D propagateFromAnchor(
            const Pose2D& anchor_map, const Pose2D& anchor_odom, const Pose2D& odom_now);

        /// propagateFromAnchor 的末端定位切换特化：锚定对取 odom_anchor_*。
        Pose2D applyOdomAnchor(const Pose2D& odom_now) const;

        /// 定位跳变兜底：判定 map 定位是否发生跳变，并决定本帧交给控制的位姿。
        ///
        /// 判据（旁路）——把 map->odom 的变化量投影到车当前位置：
        ///   pose_cur = T_cur * odom_pose , pose_ref = T_ref * odom_pose
        ///   disagree = hypot(位置差, 归一化朝向差) > threshold  =>  跳变
        /// 两项乘的是**同一帧 odom**，车的真实运动被完全抵消（T_cur==T_ref 时 disagree 恒为 0，
        /// 无论车在哪、跑多快），odom 话题的传输延迟同样被完全抵消——这正是换掉"TF 位姿 vs
        /// odom 递推位姿"老判据的原因：那两项分别走 TF 树和 /odom 话题两条独立通路，话题堵
        /// 200ms 就会凭空造出 v*0.2 的假偏差，0.35m/s 下恰好顶穿 0.07 阈值。
        ///
        /// 位置与朝向都要判、且同权合成（1 rad 记作 1 m）：投影项 (R(dphi)-I)*p 只覆盖"旋转
        /// 导致的位置跳变"，车恰好在 odom 原点附近时 ||p||~0，纯旋转跳变的位移趋近于 0，
        /// 位置分量看不见——而朝向确实跳了，它会直接进 delta_theta 前馈和航向对准。
        /// 朝向差必须先归一化到 (-pi, pi]：两个估计各自累积，完全可能一个 +3.13 一个 -3.13，
        /// 裸相减得到 6.26 的假跳变。
        ///
        /// 位姿（主路）——正常时每帧滚动刷新锚定对并直接返回 map 位姿；一旦判定跳变，冻结
        /// 跳变前那一帧的锚定对并改用 odom 递推位姿，使控制输入保持连续（跳变不会污染
        /// last_index_ / 航向误差 / 横向折叠 / 到点判定）。递推位姿刻意取自主路锚定对而不是
        /// 旁路的 T_ref：两者来源不同（TF 插值 vs odom 话题），混用会在交还时多叠一层来源差。
        /// 两套基准在同一帧一起刷新、一起冻结。
        ///
        /// 触发后直接进强制停车等待 stop_wait_duration（控制环只发 0 速），不设一级冻结窗口：
        /// 那个窗口原本是为了给老判据争取"等车停稳再判"的时间（递推项在刹车过程中持续累积
        /// 漂移，判据不可信），新判据与车动不动无关，从触发那一刻起就一直有效。
        /// 等满为止，期间恢复与否一概不管——刻意不做提前退出，避免"刚停下就走、走两步又跳"
        /// 的反复横跳，也让上层看到的行为是确定的。等满后判一次：恢复则置 jump_resume_pending_
        /// 让控制环走 kResume 复速，否则置 jump_abort_pending_ 判导航失败。
        ///
        /// @param map_pose 主路 TF 查得的 map 位姿；have_map_pose=false 时其值无意义
        /// @param have_map_pose 本帧主路 TF 是否可用（可用递推撑住若干帧，见 tf_fail_count_）
        /// @param odom_pose 当前 odom 位姿
        /// @return 本帧供控制使用的位姿
        Pose2D guardLocalizationJump(
            const Pose2D& map_pose, bool have_map_pose, const Pose2D& odom_pose);

        /// 清空定位跳变兜底状态（新路线、stop 服务、以及切到不经过 TF 的取位姿分支时）。
        /// 必须清 jump_stop_active_：否则下次回到 TF 分支时会带着上次的截止时刻，
        /// 可能立刻撞上判定点而误判导航失败。
        void resetLocalizationJumpGuard();

        /// 窗口搜索最近点索引（禁后退）。
        std::size_t findNearestIndex(const Pose2D& pose, std::size_t last_index) const;

        /// 向后跟踪：从当前点沿弧长前移，但至少前移 tracking_num 个点。
        std::size_t offsetPointIndex(
            const Velocity& v, double dt, std::size_t min_points,
            const Route& route, std::size_t current_index) const;

        /// 计算误差（对 start→aim 线段投影）。
        TrackingError calcDelta(
            const Pose2D& current, const Pose2D& start, const Pose2D& aim) const;

        /// 速度投影：当前点投影到 i-1→i+1 段得比例 K，插值参考 linear_v/curvature_omega。
        void velProjection(
            const Pose2D& current, std::size_t index, const Route& route,
            double& target_linear_v, double& target_w) const;

        /// 朝向最近点搜索（允许有限后退窗口）：找当前朝向最接近的轨迹点索引。
        std::size_t findHeadingIndex(
            double current_theta, std::size_t last_index, const HeadingRoute& route) const;

        /// 朝向速度投影：当前朝向在 hi→hi+1 段插值得参考 heading_omega 与目标 theta。
        void headingProjection(
            double current_theta, std::size_t index, const HeadingRoute& route,
            double& target_heading_omega, double& target_theta) const;

        /// 偏差检测：超限则发零速、清路线、置失败标志，返回 true 表示已失败。
        /// 全流程生效：阶段二传当前跟踪索引；阶段一/三（原地旋转）传固定的起点/终点索引 0/last。
        bool checkTrackDeviation(const Pose2D& pose, const Route& pos, std::size_t nearest_idx);

        /// 到达目标点时打印里程计位姿与目标点的残差：
        /// 残差投影到目标位姿坐标系（fwd/left/yaw），并附带欧氏距离。
        void logArrivalPose(const char* phase_tag, const Pose2D& pose,
                            double target_x, double target_y, double target_theta) const;

        /// 差速 FollowPath 非线性律：把横向误差折叠成附加航向修正。
        double followPath(
            const TrackingError& error, double v, double kpy, double kvy) const;

        /// 轨迹坐标系速度 → 车体坐标系速度。
        Velocity trajVelToAgvVel(
            const Velocity& traj_vel, double heading_angle, const Pose2D& current) const;

        /// 发布 cmd_vel。
        void publishCmdVel(const Velocity& v);
        /// 发布层死区抬升：等比缩放 (vx,vy,wz) 以维持曲率约束。
        /// 仅当 speed=hypot(vx,vy) 与 |wz| 都 <各自 min 时才缩放；单维时把该维抬到 min。
        void applyMinCommandFloor(Velocity& v) const;
        void publishDebugPath();

        /// 发布跟踪状态（NAVIGATING/FINISHED/FAILED/IDLE），供 schedule_planner 推进段序列。
        /// message 附带过程信息，FAILED 时应说明失败原因（schedule_planner 会透传给 action 结果）。
        void publishNavigationStatus(int32_t status, const std::string& message = "");

        /// ~/stop 服务回调：复位路线、发零速、发 IDLE，供上层取消。
        void onStopService(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
            std::shared_ptr<std_srvs::srv::Trigger::Response> res);

        // ---------------- 工具 ----------------
        /// 把 nav_msgs/Path 转为内部原始轨迹点（含每点朝向，写入 heading_angle）。
        std::vector<TrajectoryPoint> pathToTrajectoryPoints(
            const nav_msgs::msg::Path& path) const;
        /// 把 TrackingPath.poses（geometry_msgs/Pose[]）转为内部原始轨迹点（含每点朝向）。
        std::vector<TrajectoryPoint> posesToTrajectoryPoints(
            const std::vector<geometry_msgs::msg::Pose>& poses) const;

        /**
         * @brief 判断输入轨迹点自带的朝向能否直接采信（决定曲率从朝向差分还是从 x,y 反推）。
         *
         * 判据一——是否填了：只要存在任一非零 heading_angle 即认为上游填了朝向。
         *   未填时 geometry_msgs/Quaternion 默认是单位四元数（w=1），yaw 恰为 0；
         *   唯一会误判的场景是"整段沿 +x 的直线"，而那种情况自行计算出来同样是 0，
         *   两条路结果一致，误判无害。
         *
         * 判据二——填得对不对：抽样比较朝向与相邻点弦向的夹角，超过容差就整段回退自算。
         *   挡住"部分点填了部分没填"和"填了车体朝向而非行进切向"这两类上游错误。
         *
         * @return true = 采信输入朝向；false = 由 VelocityPlanner 自行从位置计算
         */
        bool inputHeadingUsable(const std::vector<TrajectoryPoint>& pts) const;

        /**
         * @brief 公共规划入口：把离散原始轨迹点 + 段级参数构建成 PlannedRoute 并装载。
         *
         * onPath 与 onTrackingPath 共用：两者仅段级参数来源不同（节点参数 vs 消息字段），
         * 规划/截断/朝向轨迹/复位/发布状态 完全一致。onPath 的可观察行为保持不变。
         *
         * @param raw 已离散的原始轨迹点（仅 x/y）
         * @param frame_id 输入路径坐标系
         * @param profile 段级跟踪参数（mode/enable_reverse/has_target_yaw/target_yaw/...）
         */
        void buildAndLoadRoute(
            std::vector<TrajectoryPoint> raw,
            const std::string& frame_id,
            SegmentProfile profile);

        /// 加载并应用一档 PID 配置到 PidController。
        void applyPidConfig();

        // ---------------- ROS 接口 ----------------
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Subscription<nav_interfaces::msg::TrackingPath>::SharedPtr tracking_path_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<nav_interfaces::msg::ObstacleMonitorStatus>::SharedPtr obstacle_status_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_path_pub_;
        rclcpp::Publisher<nav_interfaces::msg::NavigationStatus>::SharedPtr
            navigation_status_pub_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
        // [实验功能] 舵角限位分析可行时，用于试调用 mobile_cmd_omni 的 /steering_angle_control action
        // 摆到 theta0；注意 SteeringAngleControl 语义是"按轮位分配符号形成 X 锁"，不是"全轮同向对齐"，
        // 这里只是复用现成 action 接口做调用链路试验，不代表语义完全匹配。
        // steer_helper_node_ 专门承载该 action client：主节点（this）已经被 main() 里的
        // rclcpp::spin(node) 加入了执行器，若同步等待时直接对 this 调用
        // rclcpp::spin_until_future_complete，会因该函数内部尝试把 this 再加入一个临时执行器
        // 而抛 "has already been added to an executor"（实测炸过一次）。用一个独立、从未加入
        // 任何执行器的辅助节点承载 client，同步等待时才把它临时加入一次性执行器、用完即移出。
        rclcpp::Node::SharedPtr steer_helper_node_;
        rclcpp_action::Client<nav_interfaces::action::SteeringAngleControl>::SharedPtr steering_angle_control_client_;
        rclcpp::TimerBase::SharedPtr control_timer_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        param_callback_;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // ---------------- 核心组件 ----------------
        PidController pid_;
        VelocityPlanner velocity_planner_;

        // ---------------- 参数（缓存） ----------------
        BaseType base_type_{BaseType::kOmniTransRotation};
        double control_rate_{30.0};
        double update_timeout_{1.0};
        std::string path_topic_{"~/path"};
        std::string tracking_path_topic_{"~/tracking_path"};
        std::string odom_topic_{"~/odom"};
        std::string cmd_vel_topic_{"~/cmd_vel"};
        std::string debug_path_topic_{"~/debug/reference_path"};
        std::string navigation_status_topic_{"~/navigation_status"};
        std::string obstacle_status_topic_{"/obstacle_status"}; // 绝对话题：collision_monitor 是独立节点
        std::string stop_service_name_{"~/stop"};
        std::string odom_frame_{"odom"};
        std::string base_frame_{"base_link"};
        double trajectory_resolution_{0.005};
        double angle_resolution_{0.0034906585}; // 朝向离散角度间隔 rad（0.2°）
        std::size_t tracking_num_{5};
        std::size_t max_search_number_{100};
        double reach_distance_{0.01};
        double reach_angle_{0.03};
        double end_extended_distance_{0.01};
        std::size_t heading_back_window_{8}; // 朝向索引允许的后退窗口点数
        bool enable_heading_lock_{false}; // 起步阶段锁定朝向，避免平移一开始就同步旋转
        double heading_lock_distance_{0.5}; // current_s 小于该距离时禁用朝向跟踪
        bool enable_path_truncate_{true}; // 收到路径时若车已在路径中段，从最近点起截断
        bool enable_reverse_{false}; // 倒车模式：diff 阶段二航向对准切线反向，vx 取负
        bool enable_track_deviation_check_{true}; // 跟踪中超偏报失败开关
        double track_deviation_limit_{0.1}; // 超出该距离（m）判定为跟踪失败
        bool enable_steer_range_analysis_{true}; // [实验功能] kOmniTransRotation 舵角限位可行性分析开关
        double experimental_steer_limit_deg_{135.0}; // [实验功能] 舵角限位可行性分析用的 ±限位角度

        // 末端定位切换：距目标点剩余距离/角度 <= 阈值时，位姿参考由 map(TF) 切换到 odom 锚定估计，
        // 消除到点阶段 map 定位抖动导致的速度忽正忽负；0=关闭（保持原 map/TF 逻辑不变）。
        double end_localization_switch_distance_{0.1};
        double end_localization_switch_angle_{0.15};

        // 速度规划参数
        VelocityPlannerParams planner_params_;

        // 10 级速度分档
        std::array<double, kGainLevelCount> vel_levels_{};

        // PID 增益表
        PidGainTable x_gains_{};
        PidGainTable y_gains_{};
        PidGainTable theta_gains_{};
        double max_correct_vy_{0.4};
        double max_correct_w_{0.3};
        double max_percentage_vy_to_vx_{0.2};
        double diff_lateral_gain_{1.5};
        double kp_heading_{1.0}; // 朝向比例纠偏增益（wz = 梯形前馈 + kp·err）

        // ---------------- 运行时状态 ----------------
        mutable std::mutex route_mutex_;
        std::optional<PlannedRoute> route_; // 当前规划结果（位置 + 朝向）
        SegmentProfile segment_;            // 当前段级跟踪参数（与 route_ 同生命周期，同锁）
        std::size_t nearest_index_{0}; // 最近点索引
        std::size_t last_index_{0}; // 上次索引（禁后退）
        std::size_t track_index_{0}; // 前视点索引
        std::size_t heading_index_{0}; // 当前朝向索引（朝向进度维度）
        std::size_t last_heading_index_{0}; // 上次朝向索引（搜索起点，允许后退）
        TrackPhase track_phase_{TrackPhase::kAlignToPath}; // 三阶段状态机当前阶段（diff / omni平移共用）
        bool route_finished_{false};
        bool route_failed_{false}; // 轨迹跟踪异常中止（与正常到点区分）
        bool is_need_reset_{true};

        // ---------------- 速度跟踪状态（两阶段步进模型） ----------------
        VelocityTrackMode vel_track_mode_{VelocityTrackMode::kFollowRef};
        double last_track_linear_v_{0.0};  // 上一帧线速度通道输出（模型递推的唯一依据）
        double last_track_angular_w_{0.0}; // 上一帧角速度通道输出
        // 加速度衰减时间 T：|a| 由上限均匀衰减到 0 的时长，jerk = |a|_max / T 由此派生。
        double linear_accel_decay_time_{0.3};
        double angular_accel_decay_time_{0.3};
        // kResume 退回 kFollowRef 的收敛阈值（线速度通道判定）。
        double resume_converge_linear_velocity_{0.02};
        // SLOWDOWN/STOP 相对常规最大减速度的放大倍数（越紧急压得越狠；jerk 随之等比放大）。
        double slowdown_decel_scale_{1.5};
        double stop_decel_scale_{2.0};

        mutable std::mutex obstacle_status_mutex_;
        uint8_t latest_obstacle_status_{nav_interfaces::msg::ObstacleMonitorStatus::FREE};
        // 障碍状态看门狗：最近一次收到 obstacle_status 的时刻。障碍检测模块异常静默时，
        // 上一帧的 FREE 会被一直沿用下去（等于失去停障能力），必须主动判超时。
        rclcpp::Time last_obstacle_status_time_;
        double obstacle_status_timeout_{1.0}; // 障碍状态超时 s，<=0 关闭看门狗
        // STOP/SLOWDOWN 解除迟滞：记录最近一次处于该状态的时间，1s 内不允许降级。
        rclcpp::Time last_obstacle_stop_time_;
        rclcpp::Time last_obstacle_slowdown_time_;

        mutable std::mutex odom_mutex_;
        nav_msgs::msg::Odometry latest_odom_;
        rclcpp::Time last_odom_time_;   // 上一帧 odom 的本地接收时刻（年龄看门狗基准）
        rclcpp::Time last_odom_stamp_;  // 上一帧 odom 的 msg->header.stamp（诊断用）
        bool have_odom_{false};
        // odom 接收年龄超限而正在强制停车。必须是独立标志，不能只看 vel_track_mode_==kStop
        // ——那个也可能是障碍触发的，混用会把障碍停车错误地当成 odom 恢复而提前复速。
        bool odom_stale_stop_active_{false};
        // 强制停车截止时刻，触发那一帧设定，之后不续期。复速要求"窗口已满"且"odom 已新鲜"。
        rclcpp::Time odom_stale_stop_deadline_;
        // 上一帧真正发布出去的车体系指令。odom 卡顿时的平滑刹停要从它等比衰减，
        // 因为那条路径上没有位姿、做不了轨迹系->车体系的旋转。
        Velocity last_published_cmd_{};

        rclcpp::Time last_path_time_;
        std::chrono::steady_clock::time_point last_control_update_time_;
        bool have_control_update_time_{false};
        std::string path_frame_{"odom"};

        // 末端定位切换状态（单线程控制环内使用，随每条新路线重置）。
        bool odom_anchor_active_{false}; // 是否已切换到末端 odom 锚定位姿模式
        Pose2D odom_anchor_map_pose_{};  // 切换时刻的 map(TF) 位姿
        Pose2D odom_anchor_odom_pose_{}; // 切换时刻的 odom 位姿

        // 定位跳变兜底状态（同样只在控制环内使用；随每条新路线重置）。
        // 两套基准，同一帧一起刷新、一起冻结：
        //   旁路 jump_ref_map_odom_ 只参与判据；主路锚定对只参与位姿递推。
        bool jump_ref_valid_{false};      // 两套基准是否已建立
        Pose2D jump_ref_map_odom_{};      // 基准时刻的 path_frame->odom 变换（判据用）
        Pose2D jump_anchor_map_pose_{};   // 基准时刻的 map(TF) 位姿（递推用）
        Pose2D jump_anchor_odom_pose_{};  // 基准时刻的 odom 位姿（递推用）
        rclcpp::Time last_map_odom_stamp_; // 最近一次查到的 map->odom 变换时间戳（静默看门狗）
        bool jump_stop_active_{false};    // 是否处于跳变强制停车等待期（控制环只发 0 速）
        rclcpp::Time jump_stop_deadline_; // 强制停车的截止时刻，等满为止，期间不提前退出
        // 主路 TF 连续失败帧数。5ms 超时下偶发失败是常态，靠锚定对递推撑住几帧即可，
        // 撑久了递推会漂，所以超过上限仍按原有行为放弃本帧。
        int tf_fail_count_{0};
        // 下面两个是跨函数的一次性事件，由 guardLocalizationJump 置位、控制环消费。
        // 之所以不在 guardLocalizationJump 里就地处理：它跑在取位姿的路径上，而
        // buildAndLoadRoute 也会调 getCurrentState，在那里反手去动 route_ 锁序上不安全。
        bool jump_abort_pending_{false};  // 停车等满仍未恢复 -> 判导航失败
        bool jump_resume_pending_{false}; // 停车等满且已恢复 -> 控制环走 kResume 复速

        bool enable_localization_jump_recovery_{true};      // 功能总开关（关闭=完全恢复原行为）
        double localization_jump_threshold_{0.07};          // 跳变判据 m，恢复判据复用同一阈值
        double localization_jump_stop_wait_duration_{30.0}; // 强制停车等待时长 s
        double map_to_odom_timeout_{1.0};                   // map->odom 静默告警阈值 s，<=0 关闭
        int max_tf_fallback_frames_{5};                     // 主路 TF 连续失败时允许递推撑住的帧数

        // odom 接收看门狗。停车与日志用两个阈值：想只看诊断不改变车的运动时，
        // 把 odom_stale_stop_timeout 设 0 关掉停车、留着日志即可。
        double odom_stale_stop_timeout_{0.2};       // 接收年龄超过该值触发强制停车 s，<=0 关闭
        double odom_stale_stop_hold_duration_{5.0}; // 强制停车最短保持时长 s
        double odom_gap_log_threshold_{0.2};        // 相邻两帧接收间隔超过该值打印诊断 s，<=0 关闭
        double tf_lookup_timeout_{0.005};      // TF lookup 等待上限 s
    };
} // namespace path_follower

#endif  // PATH_FOLLOWER__PATH_FOLLOWER_HPP_