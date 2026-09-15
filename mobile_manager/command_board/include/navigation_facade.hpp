// 导航门面节点（节点名 navigation，宿主进程 command_board，DEC-035/DEC-037 命名规则）。
// 对外 L1：~/navigate（统一 action，DEC-042）+ 过渡期保留的 ~/navigate_to_point、~/navigate_route；~/state、~/global_path、~/local_path、~/nav_command。
// 附：nav_command SET_SPEED_LIMIT 透传 nav2 speed_limit；订阅急停位 → BLOCKED(1312)；CHECK_REACHABLE 预留（1311）。
// 只做契约面：受理 / 目标解析（订阅 map_asset_manager/assets）/ 转发 nav2 / 统一状态机 / 路径转发；不做规划与控制。
// 形态：独立共享库 navigation_facade + rclcpp_components 组件，宿主可以是 command_board 进程（当前）、mission_manager 进程或任意容器，接口名不随宿主变。
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/msg/speed_limit.hpp>
#include <nav_interfaces/action/navigate_route.hpp>
#include <nav_interfaces/action/navigate.hpp>
#include <nav_interfaces/action/navigate_to_point.hpp>
#include <nav_interfaces/msg/asset_snapshot.hpp>
#include <nav_interfaces/msg/localization_state.hpp>
#include <nav_interfaces/msg/navigation_path.hpp>
#include <nav_interfaces/msg/navigation_state.hpp>
#include <nav_interfaces/msg/navigation_status.hpp>
#include <nav_interfaces/msg/route_plan.hpp>
#include <nav_interfaces/srv/nav_command.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>

namespace command_board {

class NavigationFacade : public rclcpp::Node {
public:
    // options 供 rclcpp_components 容器加载；无论宿主进程用 launch 的 name= 怎么改名，本节点名固定为 navigation（见 .cpp 守卫）
    explicit NavigationFacade(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    using NavToPoint = nav_interfaces::action::NavigateToPoint;
    using NavRoute   = nav_interfaces::action::NavigateRoute;
    using NavAct     = nav_interfaces::action::Navigate;
    using N2Pose     = nav2_msgs::action::NavigateToPose;
    using N2Poses    = nav2_msgs::action::NavigateThroughPoses;
    using State      = nav_interfaces::msg::NavigationState;
    using GHPoint    = rclcpp_action::ServerGoalHandle<NavToPoint>;
    using GHRoute    = rclcpp_action::ServerGoalHandle<NavRoute>;
    using GHNav      = rclcpp_action::ServerGoalHandle<NavAct>;
    using N2PoseGH   = rclcpp_action::ClientGoalHandle<N2Pose>;
    using N2PosesGH  = rclcpp_action::ClientGoalHandle<N2Poses>;

    enum class CancelReason { NONE, PAUSE, CANCEL, PREEMPT, TIMEOUT };

    struct Task {
        std::string id;
        uint8_t target_type{State::TARGET_NONE};
        std::string target_name;
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;   // 终点 = back()
        bool route_mode{false};
        uint8_t state{State::IDLE};
        int32_t error_code{State::E_NONE};
        std::string message;
        uint64_t plan_seq{0};
        float distance_remaining{0.f};
        float eta_sec{0.f};
        uint32_t poses_remaining{0};
        uint32_t recoveries{0};
        uint32_t current_index{0};
        bool path_received{false};
        std::shared_ptr<GHPoint> gh_point;
        std::shared_ptr<GHRoute> gh_route;
        std::shared_ptr<GHNav>   gh_nav;
        N2PoseGH::SharedPtr n2_pose;
        N2PosesGH::SharedPtr n2_poses;
        rclcpp_action::GoalUUID n2_goal_id{};
        CancelReason cancel_reason{CancelReason::NONE};
        rclcpp::Time last_feedback;
        rclcpp::Time blocked_since;
        std::string last_nav_status_fail;
    };

    // ── 对外 action ──
    rclcpp_action::GoalResponse onPointGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const NavToPoint::Goal>);
    rclcpp_action::CancelResponse onPointCancel(const std::shared_ptr<GHPoint>);
    void onPointAccepted(const std::shared_ptr<GHPoint>);
    rclcpp_action::GoalResponse onRouteGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const NavRoute::Goal>);
    rclcpp_action::CancelResponse onRouteCancel(const std::shared_ptr<GHRoute>);
    void onRouteAccepted(const std::shared_ptr<GHRoute>);
    rclcpp_action::GoalResponse onNavGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const NavAct::Goal>);
    rclcpp_action::CancelResponse onNavCancel(const std::shared_ptr<GHNav>);
    void onNavAccepted(const std::shared_ptr<GHNav>);
    void admitLocked(Task&& t);                                   // 三个 action 共用的受理尾段
    bool resolveTargetLocked(Task& t, std::string& why);          // 点名 / 路线名 → 位姿序列
    void finalizeHandlesLocked(Task& t, bool ok, int32_t code, const std::string& text);
    void publishActionFeedbackLocked();

    // ── 任务生命周期（调用方持锁）──
    bool preemptOrRejectLocked(std::string& reason);
    void startTaskLocked(Task&& task);
    void sendToNav2Locked();
    void cancelNav2Locked(CancelReason reason);
    void finishLocked(uint8_t state, int32_t code, const std::string& msg);
    void publishStateLocked();
    void publishEmptyPathLocked();
    bool localizationReady(std::string& why) const;
    std::string newTaskId();
    static bool taskActive(uint8_t s) { return s == State::PENDING || s == State::RUNNING || s == State::PAUSED || s == State::BLOCKED; }

    // ── nav2 回调 ──
    void onN2PoseResponse(N2PoseGH::SharedPtr);
    void onN2PoseFeedback(N2PoseGH::SharedPtr, const std::shared_ptr<const N2Pose::Feedback>);
    void onN2PoseResult(const N2PoseGH::WrappedResult&);
    void onN2PosesResponse(N2PosesGH::SharedPtr);
    void onN2PosesFeedback(N2PosesGH::SharedPtr, const std::shared_ptr<const N2Poses::Feedback>);
    void onN2PosesResult(const N2PosesGH::WrappedResult&);
    void handleNav2Result(const rclcpp_action::GoalUUID& id, rclcpp_action::ResultCode code);
    int32_t mapAbortCodeLocked(std::string& why) const;

    // ── 订阅回调 ──
    void onAssets(const nav_interfaces::msg::AssetSnapshot::SharedPtr);
    void onPlan(const nav_msgs::msg::Path::SharedPtr);
    void onRoutePlan(const nav_interfaces::msg::RoutePlan::SharedPtr);
    void onLocalSource(const nav_msgs::msg::Path::SharedPtr);
    void onLocState(const nav_interfaces::msg::LocalizationState::SharedPtr);
    void onNavStatus(const nav_interfaces::msg::NavigationStatus::SharedPtr);
    void onDiag(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr);
    void onEstop(const std_msgs::msg::Bool::SharedPtr);
    void onNavCommand(const std::shared_ptr<nav_interfaces::srv::NavCommand::Request>, std::shared_ptr<nav_interfaces::srv::NavCommand::Response>);
    void onTimer();

    // ── 成员 ──
    std::recursive_mutex mu_;
    Task task_;
    uint32_t task_counter_{0};
    std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> points_;
    std::unordered_map<std::string, std::vector<std::string>> routes_;
    std::vector<nav_interfaces::msg::RoutePlanSegment> route_segments_;
    nav_interfaces::msg::LocalizationState loc_state_;
    bool loc_state_seen_{false};
    bool estop_active_{false};
    float speed_limit_{0.f};
    bool speed_percentage_{false};
    rclcpp::Time last_local_pub_;

    // 参数
    std::string global_frame_;
    bool require_localization_{true};
    bool preempt_on_new_goal_{true};
    double blocked_hold_sec_{3.0};
    double progress_timeout_sec_{0.0};
    double local_path_rate_hz_{5.0};
    double nav2_wait_sec_{1.0};

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp_action::Server<NavToPoint>::SharedPtr point_server_;
    rclcpp_action::Server<NavRoute>::SharedPtr route_server_;
    rclcpp_action::Server<NavAct>::SharedPtr nav_server_;
    rclcpp_action::Client<N2Pose>::SharedPtr n2_pose_client_;
    rclcpp_action::Client<N2Poses>::SharedPtr n2_poses_client_;
    rclcpp::Publisher<State>::SharedPtr state_pub_;
    rclcpp::Publisher<nav_interfaces::msg::NavigationPath>::SharedPtr global_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;
    rclcpp::Subscription<nav_interfaces::msg::AssetSnapshot>::SharedPtr assets_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
    rclcpp::Subscription<nav_interfaces::msg::RoutePlan>::SharedPtr route_plan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_src_sub_;
    rclcpp::Subscription<nav_interfaces::msg::LocalizationState>::SharedPtr loc_sub_;
    rclcpp::Subscription<nav_interfaces::msg::NavigationStatus>::SharedPtr nav_status_sub_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
    rclcpp::Service<nav_interfaces::srv::NavCommand>::SharedPtr cmd_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace command_board
