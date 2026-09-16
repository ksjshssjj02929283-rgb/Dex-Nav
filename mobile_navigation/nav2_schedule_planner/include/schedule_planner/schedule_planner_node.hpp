#ifndef SCHEDULE_PLANNER__SCHEDULE_PLANNER_NODE_HPP_
#define SCHEDULE_PLANNER__SCHEDULE_PLANNER_NODE_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav_interfaces/action/navigate_through_graph_points.hpp>
#include <nav_interfaces/msg/navigation_status.hpp>
#include <nav_interfaces/msg/path_segment.hpp>
#include <nav_interfaces/msg/tracking_path.hpp>
#include <nav_interfaces/srv/resolve_nav_points.hpp>
#include <nav_interfaces/srv/set_polygon_padding.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "schedule_planner/path_segment_sampler.hpp"
#include "schedule_planner/segment_merger.hpp"

namespace schedule_planner
{
    /**
     * @brief 多段路径执行器：托管 NavigateThroughGraphPoints action。
     *
     * 数据流：
     *   上层 ──NavigateThroughGraphPoints(nav_point_names)──▶ schedule_planner
     *   schedule_planner ──ResolveNavPoints 服务──▶ map_asset_manager（反查→PathSegment[]）
     *   schedule_planner ──/path(逐段 5mm)──▶ path_follower ──cmd_vel──▶ 底盘
     *                ◀── ~/navigation_status(NAVIGATING/FINISHED/FAILED) ──
     *                ── ~/stop(取消) ──▶
     *
     * 执行模型：handle_accepted 起一条专用线程跑 execute()，逐段离散发 /path，
     * 通过条件变量等待 path_follower 回报 FINISHED/FAILED 推进；取消/超时则调 ~/stop。
     * 主执行器用 SingleThreadedExecutor 即可（execute() 不占执行器线程）。
     *
     * v1：逐段离散为 nav_interfaces/TrackingPath 发 /tracking_path，携带段级跟踪参数
     *     （mode/backward/hold_direction/max_speed/padding/target_yaw），path_follower 据此分派。
     *
     * v2：新增 mode 参数二选一，节点启动时固定，不支持运行时热切换。
     *   - "independence"（默认）：上述 v1 流程，逐字节不变。
     *   - "controller"：逐段离散后不再发 /tracking_path，而是把同一份离散点转成
     *     nav_msgs/Path（中间点朝向取相邻点切向，末点取 target_yaw，NaN 则延续切向），
     *     经 rclcpp_action 客户端向 controller_server 发 FollowPath goal（controller_id/
     *     goal_checker_id 为固定参数，不透传段级属性），用 action 的 succeed/abort/
     *     canceled 结果驱动 runSegmentViaController() 的返回码，语义与 runSegment()
     *     完全一致（0=完成 1=失败 2=取消 3=超时 4=shutdown），execute() 的上层循环无需
     *     区分模式。此模式绕开 bt_navigator，因此没有 BT 里的全局重规划/清 costmap/
     *     后退等恢复行为，仅有 controller_server 自身的 progress_checker/failure_tolerance
     *     兜底。失败原因同样取自 navigation_status（两种模式下 status_sub_ 都订阅），
     *     因为 FollowPath 的 action 结果只有状态码、没有 message。
     */
    class SchedulePlannerNode : public rclcpp::Node
    {
    public:
        explicit SchedulePlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~SchedulePlannerNode() override;

    private:
        using NavigateThroughGraphPoints = nav_interfaces::action::NavigateThroughGraphPoints;
        using GoalHandleT = rclcpp_action::ServerGoalHandle<NavigateThroughGraphPoints>;
        using PathSegment = nav_interfaces::msg::PathSegment;
        using ResolveNavPoints = nav_interfaces::srv::ResolveNavPoints;
        using SetPolygonPadding = nav_interfaces::srv::SetPolygonPadding;
        using FollowPath = nav2_msgs::action::FollowPath;
        using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

        // ---------------- action 回调 ----------------
        rclcpp_action::GoalResponse handleGoal(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const NavigateThroughGraphPoints::Goal> goal);
        rclcpp_action::CancelResponse handleCancel(
            const std::shared_ptr<GoalHandleT> goal_handle);
        void handleAccepted(const std::shared_ptr<GoalHandleT> goal_handle);

        /// 主执行（运行在 exec_thread_ 上）：executeGoal() 的异常兜底外壳。
        /// exec_thread_ 是裸 std::thread，任何异常逃逸出线程函数都会直接 std::terminate
        /// 掉整个节点，所以这一层必须存在，不能把 try/catch 分散到各调用点。
        void execute(const std::shared_ptr<GoalHandleT> goal_handle);
        /// 主执行实体（解析 -> 逐段执行 -> 出结果）。
        void executeGoal(const std::shared_ptr<GoalHandleT> goal_handle);
        /// execute() 捕获异常后的收尾：goal 还活着就 abort 掉，并释放 current_goal_。
        void abortGoalOnException(
            const std::shared_ptr<GoalHandleT>& goal_handle, const std::string& what);

        /// 发送一条（可能由多段合并而来的）轨迹，等待 path_follower 回报。
        /// 返回：0=完成 1=失败 2=取消 3=超时 4=shutdown。
        /// out_msg 携带该结果的过程/失败信息（失败时来自 NavigationStatus.message），供
        /// execute() 拼进 action 结果，失败时不再是笼统的 "segment N failed"。
        /// 离散与 target_yaw 的处理已上提到 executeGoal——合并后一组对应多段，
        /// 「是不是末段」这个判断只有调用方知道。
        /// @param seg_count 本组包含的原始段数，用于按比例放宽段超时。
        int runSegment(
            nav_interfaces::msg::TrackingPath tp, std::size_t idx, std::size_t total,
            std::size_t seg_count,
            const std::shared_ptr<GoalHandleT>& goal_handle, std::string& out_msg);

        /// controller 模式：离散一段后转 nav_msgs/Path，发 FollowPath goal 给 controller_server
        /// 并等待其结果。返回值语义与 runSegment 一致：0=完成 1=失败 2=取消 3=超时 4=shutdown。
        /// out_msg 语义同 runSegment：FollowPath 结果本身不带错误文本，故失败/拒绝/超时时
        /// 由 waitFailureMessage() 去 navigation_status 取 controller 上报的失败原因拼在后面，
        /// 取不到才退回通用说明。
        int runSegmentViaController(
            const PathSegment& seg, std::size_t idx, std::size_t total,
            const std::shared_ptr<GoalHandleT>& goal_handle, std::string& out_msg);

        /// controller 模式下取失败原因文本：FollowPath 的 action 结果只有 abort/cancel 码、
        /// 不带 message，真正的失败原因由 controller 侧发在 navigation_status 上。
        /// 结果回调与状态话题是两条独立链路，结果可能先于状态到达，故最多再等
        /// status_message_timeout_ 秒让 FAILED 状态落地；期间没等到 FAILED 则返回空串，
        /// 由调用方保留通用说明（不返回 NAVIGATING/FINISHED 的文本，避免把非错误信息当失败原因）。
        std::string waitFailureMessage();

        /// 把 TrackingPath（poses 朝向暂无意义）转成 nav_msgs/Path：
        /// 中间点朝向取相邻点切向，末点优先取 target_yaw（NaN 则延续上一点切向）。
        nav_msgs::msg::Path buildNav2Path(const nav_interfaces::msg::TrackingPath& tp) const;

        /// 取消当前 controller 模式下正在执行的 FollowPath goal（若有）。
        void cancelControllerGoal();

        /// 向 mobile_bag_recorder 上报一次异常，请它留下故障前后一段时间的 bag。
        /// 走 std_msgs/String 话题而不是它的 TriggerRecord 服务：发布不阻塞（这里是段级
        /// 失败路径，后面还要停车/收尾，不能等应答），且只依赖 std_msgs，本包不必 depend
        /// 录包节点——导航执行器反过来依赖录包工具是架构倒置。
        /// reason 会被录包节点写进 bag 目录名（只保留字母数字/_/-，超 40 字符截断）。
        /// 录包节点没起时这就是一次没有订阅者的 publish，无副作用。
        void triggerBagRecord(const std::string& reason);

        /// 终端测试用取消服务：无论有没有 active goal 都直接触发一次停车兜底；
        /// 有 active goal 时另置 force_cancel_requested_ 唤醒等待循环，走正常取消流程。
        void onCancelService(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
            std::shared_ptr<std_srvs::srv::Trigger::Response> res);

        // ---------------- 外部交互 ----------------
        /// 调 map_asset_manager 解析 nav_point_names -> PathSegment[]。
        bool resolveSegments(
            const std::vector<std::string>& nav_point_names,
            std::vector<PathSegment>& segments, std::string& err);

        /// 按 move_mode_ 表达的底盘能力，就地归一化段级 mode / hold_direction。
        ///
        /// 与 default_mode_ 的区别：default_mode_ 只在 PathSegment.mode==0 时兜底，段里显式
        /// 写了 mode=2/3 会**绕过**它；move_mode_ 是能力上限，无条件覆盖。
        ///
        /// 放在输入端归一而不是塞进 PathSegmentSampler::resolveMode，是因为后者是 static 且被
        /// sampler(x2) 与 segment_merger(x1) 共用，加一个参数会让签名一路扩散到 5 个文件；
        /// 在这里归一之后，resolveMode / 合并判定 / 离散全部自然走对应分支，那两个类一行不用改。
        ///
        /// 由 resolveSegments 在拿到结果后立即调用——那里是 segments 的唯一入口，
        /// 后续 planMergeGroups / discretize / discretizeGroup / runSegmentViaController 都取自它。
        void applyMoveModeConstraint(std::vector<PathSegment>& segments);
        /// 调 path_follower ~/stop 停车（阻塞等响应，仅供 exec_thread_ 上的调用点使用：
        /// 该线程独立于节点 executor，阻塞等待不会自锁死）。
        void callStop();
        /// callStop() 的非阻塞版：发出请求后立即返回，响应在回调里异步处理（仅记日志）。
        /// 供节点 executor 自身回调（如 onCancelService）使用——那些回调本身就是靠
        /// executor spin 触发的，若在其中阻塞等响应，会等到 executor 转不动自己，永远超时。
        void callStopAsync();
        /// 把本段的 padding 下发给 collision_monitor 的停障检测器（阻塞等响应，
        /// 仅供 exec_thread_ 上的调用点使用，理由同 callStop()）。
        /// padding 为 NaN 时透传 NaN，collision_monitor 侧即恢复其 default_padding。
        /// 返回 false 表示服务不可用/超时/失败，调用方据此决定是否继续发路径。
        bool callSetStopPadding(float padding, std::string& err);
        void onNavigationStatus(const nav_interfaces::msg::NavigationStatus::SharedPtr msg);
        void publishFeedback(
            const std::shared_ptr<GoalHandleT>& gh,
            const std::string& status, std::size_t idx, std::size_t total);
        void finishGoal();

        // ---------------- 状态 ----------------
        enum class TrackState
        {
            kIdle = 0,
            kNavigating = 1,
            kFinished = 2,
            kFailed = 3
        };

        std::mutex state_mutex_;
        std::condition_variable state_cv_;
        TrackState track_state_{TrackState::kIdle};
        /// path_follower 最近一次 NavigationStatus.message（FAILED 时是失败原因），
        /// 与 track_state_ 同锁更新，runSegment() 在读到终态时一并取出。
        std::string last_status_message_;

        /// controller 模式下 FollowPath goal 的执行状态（与 TrackState 分开，避免两条
        /// 状态机互相干扰；handleCancel 会同时唤醒 state_cv_ 和 controller_state_cv_）。
        enum class ControllerGoalState
        {
            kAwaitingAcceptance = 0,
            kRejected = 1,
            kExecuting = 2,
            kSucceeded = 3,
            kAborted = 4,
            kCanceled = 5
        };
        std::mutex controller_state_mutex_;
        std::condition_variable controller_state_cv_;
        ControllerGoalState controller_goal_state_{ControllerGoalState::kAwaitingAcceptance};
        GoalHandleFollowPath::SharedPtr controller_goal_handle_;

        std::mutex current_goal_mutex_;
        std::shared_ptr<GoalHandleT> current_goal_;
        std::thread exec_thread_;
        std::atomic<bool> shutting_down_{false};
        /// 终端 /schedule_planner/force_cancel 服务触发的取消请求；execute() 每次新 goal 开始时复位，
        /// runSegment()/runSegmentViaController() 的等待循环与 is_canceling() 并列判断。
        std::atomic<bool> force_cancel_requested_{false};

        // ---------------- ROS 接口 ----------------
        rclcpp_action::Server<NavigateThroughGraphPoints>::SharedPtr action_server_;
        rclcpp::Publisher<nav_interfaces::msg::TrackingPath>::SharedPtr path_pub_;
        rclcpp::Subscription<nav_interfaces::msg::NavigationStatus>::SharedPtr status_sub_;
        rclcpp::Client<ResolveNavPoints>::SharedPtr resolve_client_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
        rclcpp::Client<SetPolygonPadding>::SharedPtr padding_client_;
        rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
        /// 段级失败时向录包节点上报的话题；bag_trigger_topic_ 为空时不创建（=关闭该功能）
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bag_trigger_pub_;

        PathSegmentSampler sampler_;
        /// 相邻段合并平滑的参数（见 segment_merger.hpp）。
        MergeParams merge_params_;

        // ---------------- 参数 ----------------
        std::string action_name_{"navigate_through_graph_points"};
        std::string tracking_path_topic_{"/tracking_path"};
        std::string navigation_status_topic_{"/navigation_status"};
        std::string stop_service_{"/stop"};
        std::string resolve_service_{"/map_asset_manager/resolve_nav_points"};
        std::string cancel_service_name_{"/schedule_planner/force_cancel"};
        /// collision_monitor 的膨胀设置服务
        std::string padding_service_{"/collision_monitor/set_polygon_padding"};
        /// 停障检测器名（只改这一个，减速区保持配置里的 default_padding）
        std::string stop_polygon_name_{"PolygonStop"};
        double padding_timeout_{2.0};
        /// true = padding 服务不可用/失败时中止该段（停障距离未知，保守处理）；
        /// false = 只告警并按上一段的 padding 继续（collision_monitor 未部署的场景）
        bool require_padding_service_{true};
        std::string frame_id_{"map"};
        double resolution_{0.005};
        double resolve_timeout_{5.0};
        double segment_timeout_{300.0};
        /// controller 模式下 FollowPath 失败后，等 navigation_status 上的失败原因落地的上限 s。
        double status_message_timeout_{0.3};
        /// mobile_bag_recorder 的触发话题；设为空串则完全不上报（不创建 publisher）
        std::string bag_trigger_topic_{"/mobile_bag_recorder_node/trigger_event"};
        /// 默认跟踪方式：PathSegment 未显式指定 mode 时使用。
        /// 1=MODE_DIFF（差速），3=MODE_OMNI_TRANS_ROTATION（全向平移+旋转）。
        uint8_t default_mode_{1};

        /// 底盘运动能力（对应 path_follower 的 base_type），决定生成路径段时允许的跟踪方式：
        ///   1 = MODE_DIFF：只能差速。所有段强制 DIFF，hold_direction 一律清 NaN。
        ///   2 = MODE_OMNI_TRANSLATION：所有段强制全向平移（保持朝向）。
        ///   3 = MODE_OMNI_TRANS_ROTATION：全能力，三种模式都允许，按段自身 mode 走（默认）。
        /// 与 default_mode_ 的关系见 applyMoveModeConstraint。取 1/2 时 default_mode_ 自动失效
        /// （段 mode 已被归一为具体值，resolveMode 不再走兜底分支）。
        uint8_t move_mode_{3};

        /// true = controller 模式（发 FollowPath 给 controller_server），
        /// false = independence 模式（v1 原有 /tracking_path 流程）。节点启动时由
        /// "mode" 参数（"independence"/"controller"）固定，不支持运行时热切换。
        bool controller_mode_{false};
        std::string follow_path_action_name_{"/follow_path"};
        std::string controller_id_{""};
        std::string goal_checker_id_{""};
    };
} // namespace schedule_planner

#endif // SCHEDULE_PLANNER__SCHEDULE_PLANNER_NODE_HPP_
