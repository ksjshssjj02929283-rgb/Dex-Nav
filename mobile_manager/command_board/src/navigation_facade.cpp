#include "navigation_facade.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

#include <tf2/LinearMath/Quaternion.h>

namespace command_board {

using namespace std::chrono_literals;

namespace {
geometry_msgs::msg::PoseStamped makePose(const std::string& frame, double x, double y, double yaw) {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = frame;
    p.pose.position.x = x;
    p.pose.position.y = y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    p.pose.orientation.x = q.x(); p.pose.orientation.y = q.y(); p.pose.orientation.z = q.z(); p.pose.orientation.w = q.w();
    return p;
}
std::string codeMsg(int32_t code, const std::string& text) { return "E" + std::to_string(code) + ": " + text; }
}  // namespace

NavigationFacade::NavigationFacade()
    : rclcpp::Node("navigation"), last_local_pub_(0, 0, RCL_ROS_TIME) {
    // ── 参数（全部有默认值，yaml 只覆盖需要现场调的）──
    global_frame_          = declare_parameter<std::string>("global_frame", "map");
    require_localization_  = declare_parameter<bool>("require_localization", true);
    preempt_on_new_goal_   = declare_parameter<bool>("preempt_on_new_goal", true);
    blocked_hold_sec_      = declare_parameter<double>("blocked_hold_sec", 3.0);
    progress_timeout_sec_  = declare_parameter<double>("progress_timeout_sec", 0.0);
    local_path_rate_hz_    = declare_parameter<double>("local_path_rate_hz", 5.0);
    nav2_wait_sec_         = declare_parameter<double>("nav2_wait_sec", 1.0);
    set_goal_tolerance_    = declare_parameter<bool>("set_goal_tolerance", true);
    default_goal_tolerance_ = declare_parameter<double>("default_goal_tolerance", 0.05);
    const auto controller_name   = declare_parameter<std::string>("controller_server_name", "/controller_server");
    const auto nav2_pose_action  = declare_parameter<std::string>("nav2_pose_action", "/navigate_to_pose");
    const auto nav2_poses_action = declare_parameter<std::string>("nav2_poses_action", "/navigate_through_poses");
    const auto plan_topic        = declare_parameter<std::string>("plan_topic", "/plan");
    const auto route_plan_topic  = declare_parameter<std::string>("route_plan_topic", "/planner_server/RouteGridPlanner/route_plan");
    const auto local_src_topic   = declare_parameter<std::string>("local_path_source_topic", "/local_plan");
    const auto assets_topic      = declare_parameter<std::string>("assets_topic", "/map_asset_manager/assets");
    const auto loc_topic         = declare_parameter<std::string>("localization_state_topic", "/localization/state");
    const auto nav_status_topic  = declare_parameter<std::string>("nav2_status_topic", "/navigation_status");
    const auto diag_topic        = declare_parameter<std::string>("speed_plan_diag_topic", "/navigation/speed_plan_diagnostics");
    const double heartbeat_hz    = declare_parameter<double>("heartbeat_hz", 1.0);

    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    const auto latched = rclcpp::QoS(1).transient_local().reliable();

    state_pub_       = create_publisher<State>("~/state", latched);
    global_path_pub_ = create_publisher<nav_interfaces::msg::NavigationPath>("~/global_path", latched);
    local_path_pub_  = create_publisher<nav_msgs::msg::Path>("~/local_path", rclcpp::QoS(1));

    rclcpp::SubscriptionOptions so; so.callback_group = cb_group_;
    assets_sub_     = create_subscription<nav_interfaces::msg::AssetSnapshot>(assets_topic, latched, std::bind(&NavigationFacade::onAssets, this, std::placeholders::_1), so);
    plan_sub_       = create_subscription<nav_msgs::msg::Path>(plan_topic, rclcpp::QoS(10), std::bind(&NavigationFacade::onPlan, this, std::placeholders::_1), so);
    route_plan_sub_ = create_subscription<nav_interfaces::msg::RoutePlan>(route_plan_topic, latched, std::bind(&NavigationFacade::onRoutePlan, this, std::placeholders::_1), so);
    local_src_sub_  = create_subscription<nav_msgs::msg::Path>(local_src_topic, rclcpp::QoS(1), std::bind(&NavigationFacade::onLocalSource, this, std::placeholders::_1), so);
    loc_sub_        = create_subscription<nav_interfaces::msg::LocalizationState>(loc_topic, rclcpp::QoS(10), std::bind(&NavigationFacade::onLocState, this, std::placeholders::_1), so);
    nav_status_sub_ = create_subscription<nav_interfaces::msg::NavigationStatus>(nav_status_topic, rclcpp::QoS(10), std::bind(&NavigationFacade::onNavStatus, this, std::placeholders::_1), so);
    diag_sub_       = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(diag_topic, rclcpp::QoS(10), std::bind(&NavigationFacade::onDiag, this, std::placeholders::_1), so);

    n2_pose_client_  = rclcpp_action::create_client<N2Pose>(this, nav2_pose_action, cb_group_);
    n2_poses_client_ = rclcpp_action::create_client<N2Poses>(this, nav2_poses_action, cb_group_);
    param_client_    = create_client<rcl_interfaces::srv::SetParameters>(controller_name + "/set_parameters", rmw_qos_profile_services_default, cb_group_);

    point_server_ = rclcpp_action::create_server<NavToPoint>(
        this, "~/navigate_to_point",
        std::bind(&NavigationFacade::onPointGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&NavigationFacade::onPointCancel, this, std::placeholders::_1),
        std::bind(&NavigationFacade::onPointAccepted, this, std::placeholders::_1),
        rcl_action_server_get_default_options(), cb_group_);
    route_server_ = rclcpp_action::create_server<NavRoute>(
        this, "~/navigate_route",
        std::bind(&NavigationFacade::onRouteGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&NavigationFacade::onRouteCancel, this, std::placeholders::_1),
        std::bind(&NavigationFacade::onRouteAccepted, this, std::placeholders::_1),
        rcl_action_server_get_default_options(), cb_group_);
    cmd_srv_ = create_service<nav_interfaces::srv::NavCommand>("~/nav_command",
        std::bind(&NavigationFacade::onNavCommand, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, cb_group_);

    const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, heartbeat_hz));
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period), std::bind(&NavigationFacade::onTimer, this), cb_group_);

    task_.last_feedback = now();
    task_.blocked_since = rclcpp::Time(0, 0, RCL_ROS_TIME);
    {
        std::lock_guard<std::recursive_mutex> lk(mu_);
        publishStateLocked();       // 启动即发 IDLE（latched）
        publishEmptyPathLocked();
    }
    RCLCPP_INFO(get_logger(), "navigation facade up: navigate_to_point / navigate_route / state / global_path / local_path / nav_command (nav2 %s, %s)",
                nav2_pose_action.c_str(), nav2_poses_action.c_str());
}

// ──────────────────────────────────────────────────────────────────────
// 对外 action：受理即 ACCEPT，校验与错误码在 accepted 里给（REJECT 无法携带错误码）
// ──────────────────────────────────────────────────────────────────────

rclcpp_action::GoalResponse NavigationFacade::onPointGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const NavToPoint::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::GoalResponse NavigationFacade::onRouteGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const NavRoute::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse NavigationFacade::onPointCancel(const std::shared_ptr<GHPoint> gh) {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    if (task_.gh_point == gh && taskActive(task_.state)) cancelNav2Locked(CancelReason::CANCEL);
    return rclcpp_action::CancelResponse::ACCEPT;
}
rclcpp_action::CancelResponse NavigationFacade::onRouteCancel(const std::shared_ptr<GHRoute> gh) {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    if (task_.gh_route == gh && taskActive(task_.state)) cancelNav2Locked(CancelReason::CANCEL);
    return rclcpp_action::CancelResponse::ACCEPT;
}

void NavigationFacade::onPointAccepted(const std::shared_ptr<GHPoint> gh) {
    const auto goal = gh->get_goal();
    std::lock_guard<std::recursive_mutex> lk(mu_);
    std::string why;
    if (!preemptOrRejectLocked(why)) {
        auto r = std::make_shared<NavToPoint::Result>(); r->success = false; r->message = codeMsg(State::E_NAV_BUSY, why);
        gh->abort(r); return;
    }
    Task t; t.gh_point = gh;
    if (!goal->point_name.empty()) {
        auto it = points_.find(goal->point_name);
        if (it == points_.end()) {
            Task f; f.id = newTaskId(); f.target_type = State::TARGET_POINT; f.target_name = goal->point_name; f.gh_point = gh; task_ = std::move(f);
            finishLocked(State::FAILED, State::E_NAV_TARGET_NOT_FOUND, "point not found: " + goal->point_name); return;
        }
        t.target_type = State::TARGET_POINT; t.target_name = goal->point_name; t.waypoints = {it->second.first};
        t.tolerance = goal->tolerance > 0.0 ? goal->tolerance : it->second.second;
    } else if (goal->use_pose) {
        t.target_type = State::TARGET_POSE; t.waypoints = {goal->pose}; t.tolerance = goal->tolerance;
        if (t.waypoints.back().header.frame_id.empty()) t.waypoints.back().header.frame_id = global_frame_;
    } else {
        Task f; f.id = newTaskId(); f.gh_point = gh; task_ = std::move(f);
        finishLocked(State::FAILED, State::E_NAV_TARGET_NOT_FOUND, "goal has neither point_name nor use_pose"); return;
    }
    startTaskLocked(std::move(t));
}

void NavigationFacade::onRouteAccepted(const std::shared_ptr<GHRoute> gh) {
    const auto goal = gh->get_goal();
    std::lock_guard<std::recursive_mutex> lk(mu_);
    std::string why;
    if (!preemptOrRejectLocked(why)) {
        auto r = std::make_shared<NavRoute::Result>(); r->success = false; r->message = codeMsg(State::E_NAV_BUSY, why);
        gh->abort(r); return;
    }
    Task t; t.gh_route = gh; t.route_mode = true; t.target_type = State::TARGET_ROUTE; t.target_name = goal->route_name;
    auto it = routes_.find(goal->route_name);
    if (it == routes_.end() || it->second.empty()) {
        Task f = std::move(t); f.id = newTaskId(); task_ = std::move(f);
        finishLocked(State::FAILED, State::E_NAV_TARGET_NOT_FOUND, "route not found or empty: " + goal->route_name); return;
    }
    for (const auto& n : it->second) {
        auto p = points_.find(n);
        if (p == points_.end()) {
            Task f = std::move(t); f.id = newTaskId(); task_ = std::move(f);
            finishLocked(State::FAILED, State::E_NAV_TARGET_NOT_FOUND, "route '" + goal->route_name + "' references missing point: " + n); return;
        }
        t.waypoints.push_back(p->second.first);
    }
    startTaskLocked(std::move(t));
}

// ──────────────────────────────────────────────────────────────────────
// 任务生命周期
// ──────────────────────────────────────────────────────────────────────

std::string NavigationFacade::newTaskId() {
    std::ostringstream os;
    os << "nav-" << now().nanoseconds() / 1000000 << "-" << (++task_counter_);
    return os.str();
}

bool NavigationFacade::preemptOrRejectLocked(std::string& why) {
    if (!taskActive(task_.state)) return true;
    if (!preempt_on_new_goal_) { why = "task " + task_.id + " is active (state " + std::to_string(task_.state) + ")"; return false; }
    // 抢占：当前任务取消并终态化，新任务接手
    cancelNav2Locked(CancelReason::PREEMPT);
    finishLocked(State::CANCELLED, State::E_NAV_CANCELLED, "preempted by new goal");
    return true;
}

void NavigationFacade::startTaskLocked(Task&& t) {
    t.id = newTaskId();
    t.state = State::PENDING;
    t.last_feedback = now();
    t.blocked_since = rclcpp::Time(0, 0, RCL_ROS_TIME);
    task_ = std::move(t);
    std::string why;
    if (require_localization_ && !localizationReady(why)) { finishLocked(State::FAILED, State::E_NAV_SENSOR_LOST, why); return; }
    publishStateLocked();
    publishEmptyPathLocked();
    sendToNav2Locked();
}

void NavigationFacade::sendToNav2Locked() {
    const auto wait = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(nav2_wait_sec_));
    task_.cancel_reason = CancelReason::NONE;
    task_.state = State::PENDING;
    if (task_.route_mode) {
        if (!n2_poses_client_->wait_for_action_server(wait)) { finishLocked(State::FAILED, State::E_NAV_NOT_READY, "nav2 navigate_through_poses server not available"); return; }
        N2Poses::Goal g; g.poses = task_.waypoints;
        for (auto& p : g.poses) { if (p.header.frame_id.empty()) p.header.frame_id = global_frame_; p.header.stamp = now(); }
        auto opts = rclcpp_action::Client<N2Poses>::SendGoalOptions();
        opts.goal_response_callback = std::bind(&NavigationFacade::onN2PosesResponse, this, std::placeholders::_1);
        opts.feedback_callback      = std::bind(&NavigationFacade::onN2PosesFeedback, this, std::placeholders::_1, std::placeholders::_2);
        opts.result_callback        = std::bind(&NavigationFacade::onN2PosesResult, this, std::placeholders::_1);
        n2_poses_client_->async_send_goal(g, opts);
    } else {
        if (!n2_pose_client_->wait_for_action_server(wait)) { finishLocked(State::FAILED, State::E_NAV_NOT_READY, "nav2 navigate_to_pose server not available"); return; }
        if (set_goal_tolerance_) applyGoalToleranceLocked(task_.tolerance > 0.0 ? task_.tolerance : default_goal_tolerance_);
        N2Pose::Goal g; g.pose = task_.waypoints.back();
        if (g.pose.header.frame_id.empty()) g.pose.header.frame_id = global_frame_;
        g.pose.header.stamp = now();
        auto opts = rclcpp_action::Client<N2Pose>::SendGoalOptions();
        opts.goal_response_callback = std::bind(&NavigationFacade::onN2PoseResponse, this, std::placeholders::_1);
        opts.feedback_callback      = std::bind(&NavigationFacade::onN2PoseFeedback, this, std::placeholders::_1, std::placeholders::_2);
        opts.result_callback        = std::bind(&NavigationFacade::onN2PoseResult, this, std::placeholders::_1);
        n2_pose_client_->async_send_goal(g, opts);
    }
    publishStateLocked();
}

// 原 map_asset_manager::NavigationExecutor 的功能：发单点 goal 前把控制器 goal_checker 容差设为该点位的 tolerance
void NavigationFacade::applyGoalToleranceLocked(double tol) {
    using namespace std::chrono_literals;
    if (!param_client_->wait_for_service(300ms)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "controller set_parameters unavailable; goal tolerance %.3f not applied", tol);
        return;
    }
    auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    for (const char* name : {"goal_checker.lon_goal_tolerance", "goal_checker.lat_goal_tolerance"}) {
        rcl_interfaces::msg::Parameter p; p.name = name; p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE; p.value.double_value = tol;
        req->parameters.push_back(p);
    }
    param_client_->async_send_request(req, [this, tol](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture f) {
        try { for (const auto& r : f.get()->results) if (!r.successful) RCLCPP_WARN(get_logger(), "goal tolerance %.3f rejected: %s", tol, r.reason.c_str()); }
        catch (const std::exception& e) { RCLCPP_WARN(get_logger(), "set goal tolerance failed: %s", e.what()); }
    });
}

void NavigationFacade::cancelNav2Locked(CancelReason reason) {
    task_.cancel_reason = reason;
    if (task_.n2_pose)  n2_pose_client_->async_cancel_goal(task_.n2_pose);
    if (task_.n2_poses) n2_poses_client_->async_cancel_goal(task_.n2_poses);
}

void NavigationFacade::finishLocked(uint8_t state, int32_t code, const std::string& msg) {
    task_.state = state; task_.error_code = code; task_.message = msg;
    task_.n2_pose.reset(); task_.n2_poses.reset(); task_.cancel_reason = CancelReason::NONE;
    const bool ok = (state == State::SUCCEEDED);
    const std::string text = ok ? "succeeded" : codeMsg(code, msg);
    if (task_.gh_point) {
        auto r = std::make_shared<NavToPoint::Result>(); r->success = ok; r->message = text;
        if (task_.gh_point->is_active()) { if (task_.gh_point->is_canceling()) task_.gh_point->canceled(r); else if (ok) task_.gh_point->succeed(r); else task_.gh_point->abort(r); }
        task_.gh_point.reset();
    }
    if (task_.gh_route) {
        auto r = std::make_shared<NavRoute::Result>(); r->success = ok; r->message = text;
        if (task_.gh_route->is_active()) { if (task_.gh_route->is_canceling()) task_.gh_route->canceled(r); else if (ok) task_.gh_route->succeed(r); else task_.gh_route->abort(r); }
        task_.gh_route.reset();
    }
    if (ok) RCLCPP_INFO(get_logger(), "task %s %s", task_.id.c_str(), text.c_str());
    else    RCLCPP_WARN(get_logger(), "task %s %s", task_.id.c_str(), text.c_str());
    publishStateLocked();
    publishEmptyPathLocked();
}

bool NavigationFacade::localizationReady(std::string& why) const {
    using L = nav_interfaces::msg::LocalizationState;
    if (!loc_state_seen_) { why = "no localization state received yet"; return false; }
    if (loc_state_.status != L::LOC_SUCCESS) { why = "localization status " + std::to_string(loc_state_.status) + " != LOC_SUCCESS"; return false; }
    return true;
}

// ──────────────────────────────────────────────────────────────────────
// nav2 回调
// ──────────────────────────────────────────────────────────────────────

void NavigationFacade::onN2PoseResponse(N2PoseGH::SharedPtr gh) {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    if (!gh) { finishLocked(State::FAILED, State::E_NAV_NOT_READY, "nav2 rejected the goal"); return; }
    if (task_.state != State::PENDING) return;
    task_.n2_pose = gh; task_.n2_goal_id = gh->get_goal_id(); task_.state = State::RUNNING; task_.last_feedback = now();
    publishStateLocked();
}
void NavigationFacade::onN2PosesResponse(N2PosesGH::SharedPtr gh) {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    if (!gh) { finishLocked(State::FAILED, State::E_NAV_NOT_READY, "nav2 rejected the goal"); return; }
    if (task_.state != State::PENDING) return;
    task_.n2_poses = gh; task_.n2_goal_id = gh->get_goal_id(); task_.state = State::RUNNING; task_.last_feedback = now();
    publishStateLocked();
}
void NavigationFacade::onN2PoseFeedback(N2PoseGH::SharedPtr gh, const std::shared_ptr<const N2Pose::Feedback> fb) {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    if (!gh || gh->get_goal_id() != task_.n2_goal_id) return;
    task_.distance_remaining = fb->distance_remaining;
    task_.eta_sec = static_cast<float>(rclcpp::Duration(fb->estimated_time_remaining).seconds());
    task_.recoveries = static_cast<uint32_t>(std::max<int16_t>(0, fb->number_of_recoveries));
    task_.poses_remaining = 1; task_.current_index = 0; task_.last_feedback = now();
    if (task_.gh_point) { auto f = std::make_shared<NavToPoint::Feedback>(); f->current_status = "RUNNING"; task_.gh_point->publish_feedback(f); }
}
void NavigationFacade::onN2PosesFeedback(N2PosesGH::SharedPtr gh, const std::shared_ptr<const N2Poses::Feedback> fb) {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    if (!gh || gh->get_goal_id() != task_.n2_goal_id) return;
    task_.distance_remaining = fb->distance_remaining;
    task_.eta_sec = static_cast<float>(rclcpp::Duration(fb->estimated_time_remaining).seconds());
    task_.recoveries = static_cast<uint32_t>(std::max<int16_t>(0, fb->number_of_recoveries));
    task_.poses_remaining = static_cast<uint32_t>(std::max<int16_t>(0, fb->number_of_poses_remaining));
    const uint32_t total = static_cast<uint32_t>(task_.waypoints.size());
    task_.current_index = task_.poses_remaining >= total ? 0 : total - task_.poses_remaining;
    task_.last_feedback = now();
    if (task_.gh_route) {
        auto f = std::make_shared<NavRoute::Feedback>(); f->current_status = "RUNNING"; f->current_point_index = task_.current_index; f->total_points = total;
        task_.gh_route->publish_feedback(f);
    }
}
void NavigationFacade::onN2PoseResult(const N2PoseGH::WrappedResult& r)  { handleNav2Result(r.goal_id, r.code); }
void NavigationFacade::onN2PosesResult(const N2PosesGH::WrappedResult& r) { handleNav2Result(r.goal_id, r.code); }

void NavigationFacade::handleNav2Result(const rclcpp_action::GoalUUID& id, rclcpp_action::ResultCode code) {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    if (id != task_.n2_goal_id || !taskActive(task_.state)) return;   // 过期 goal（抢占 / 上一任务）
    switch (code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            task_.distance_remaining = 0.f; task_.eta_sec = 0.f; task_.poses_remaining = 0;
            task_.current_index = static_cast<uint32_t>(task_.waypoints.size());
            finishLocked(State::SUCCEEDED, State::E_NONE, "");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            switch (task_.cancel_reason) {
                case CancelReason::PAUSE:   task_.n2_pose.reset(); task_.n2_poses.reset(); task_.cancel_reason = CancelReason::NONE; task_.state = State::PAUSED; publishStateLocked(); break;
                case CancelReason::TIMEOUT: finishLocked(State::FAILED, State::E_NAV_TIMEOUT, "no progress for " + std::to_string(progress_timeout_sec_) + " s"); break;
                case CancelReason::PREEMPT: finishLocked(State::CANCELLED, State::E_NAV_CANCELLED, "preempted by new goal"); break;
                default:                    finishLocked(State::CANCELLED, State::E_NAV_CANCELLED, "cancelled"); break;
            }
            break;
        default: {
            std::string why; const int32_t c = mapAbortCodeLocked(why);
            finishLocked(State::FAILED, c, why);
        }
    }
}

int32_t NavigationFacade::mapAbortCodeLocked(std::string& why) const {
    if (!task_.path_received) { why = "nav2 aborted before any global path was produced (goal unreachable / no plan)"; return State::E_NAV_GOAL_UNREACHABLE; }
    std::string lower = task_.last_nav_status_fail; std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
    if (lower.find("collision") != std::string::npos) { why = "nav2 aborted on collision: " + task_.last_nav_status_fail; return State::E_NAV_PATH_BLOCKED; }
    std::string lw;
    if (require_localization_ && !localizationReady(lw)) { why = "nav2 aborted while localization not ready: " + lw; return State::E_NAV_SENSOR_LOST; }
    why = task_.last_nav_status_fail.empty() ? "nav2 aborted the goal" : ("nav2 aborted: " + task_.last_nav_status_fail);
    return State::E_NAV_CONTROLLER_FAILED;
}

// ──────────────────────────────────────────────────────────────────────
// 订阅 / 服务 / 定时器
// ──────────────────────────────────────────────────────────────────────

void NavigationFacade::onAssets(const nav_interfaces::msg::AssetSnapshot::SharedPtr m) {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    points_.clear(); routes_.clear();
    for (const auto& p : m->points) points_[p.name] = {makePose(global_frame_, p.x, p.y, p.theta), static_cast<double>(p.tolerance)};
    for (const auto& r : m->routes) routes_[r.name] = r.point_names;
}

void NavigationFacade::onPlan(const nav_msgs::msg::Path::SharedPtr m) {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    if (!taskActive(task_.state) || m->poses.empty()) return;
    task_.plan_seq++; task_.path_received = true;
    nav_interfaces::msg::NavigationPath out;
    out.header = m->header; if (out.header.frame_id.empty()) out.header.frame_id = global_frame_;
    out.task_id = task_.id; out.plan_seq = task_.plan_seq; out.path = *m;
    if (task_.route_mode) out.segments = route_segments_;
    global_path_pub_->publish(out);
    publishStateLocked();
}

void NavigationFacade::onRoutePlan(const nav_interfaces::msg::RoutePlan::SharedPtr m) {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    route_segments_ = m->segments;
}

void NavigationFacade::onLocalSource(const nav_msgs::msg::Path::SharedPtr m) {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    if (task_.state != State::RUNNING && task_.state != State::BLOCKED) return;
    const auto t = now();
    if (local_path_rate_hz_ > 0.0 && (t - last_local_pub_).seconds() < 1.0 / local_path_rate_hz_) return;
    last_local_pub_ = t;
    local_path_pub_->publish(*m);
}

void NavigationFacade::onLocState(const nav_interfaces::msg::LocalizationState::SharedPtr m) {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    loc_state_ = *m; loc_state_seen_ = true;
}

void NavigationFacade::onNavStatus(const nav_interfaces::msg::NavigationStatus::SharedPtr m) {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    if (m->status == nav_interfaces::msg::NavigationStatus::FAILED && taskActive(task_.state)) task_.last_nav_status_fail = m->message;
}

void NavigationFacade::onDiag(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    bool waiting = false, seen = false;
    for (const auto& s : m->status) {
        if (s.name != "navigation_speed_plan") continue;
        seen = true; waiting = (s.message == "collision_stop_wait");
    }
    if (!seen) return;
    const auto t = now();
    if (waiting) {
        if (task_.blocked_since.nanoseconds() == 0) task_.blocked_since = t;
        if (task_.state == State::RUNNING && (t - task_.blocked_since).seconds() >= blocked_hold_sec_) {
            task_.state = State::BLOCKED; task_.message = "collision stop wait > " + std::to_string(blocked_hold_sec_) + " s"; publishStateLocked();
        }
    } else {
        task_.blocked_since = rclcpp::Time(0, 0, RCL_ROS_TIME);
        if (task_.state == State::BLOCKED) { task_.state = State::RUNNING; task_.message.clear(); publishStateLocked(); }
    }
}

void NavigationFacade::onNavCommand(const std::shared_ptr<nav_interfaces::srv::NavCommand::Request> req, std::shared_ptr<nav_interfaces::srv::NavCommand::Response> res) {
    using Cmd = nav_interfaces::srv::NavCommand;
    std::lock_guard<std::recursive_mutex> lk(mu_);
    auto fail = [&](const std::string& m) { res->success = false; res->error_code = State::E_NAV_OP_INVALID; res->message = m; };
    if (!req->task_id.empty() && req->task_id != task_.id) { fail("task_id mismatch: current is " + task_.id); return; }
    switch (req->op) {
        case Cmd::Request::OP_PAUSE:
            if (task_.state != State::RUNNING && task_.state != State::BLOCKED) { fail("PAUSE only allowed in RUNNING/BLOCKED (state " + std::to_string(task_.state) + ")"); return; }
            cancelNav2Locked(CancelReason::PAUSE);
            task_.state = State::PAUSED; task_.message = "paused"; publishStateLocked();
            res->success = true; res->error_code = State::E_NONE; res->message = "paused " + task_.id; return;
        case Cmd::Request::OP_RESUME:
            if (task_.state != State::PAUSED) { fail("RESUME only allowed in PAUSED (state " + std::to_string(task_.state) + ")"); return; }
            task_.message.clear(); sendToNav2Locked();
            res->success = true; res->error_code = State::E_NONE; res->message = "resumed " + task_.id; return;
        case Cmd::Request::OP_CANCEL:
            if (!taskActive(task_.state)) { fail("CANCEL: no active task (state " + std::to_string(task_.state) + ")"); return; }
            if (task_.state == State::PAUSED) { finishLocked(State::CANCELLED, State::E_NAV_CANCELLED, "cancelled while paused"); }
            else { cancelNav2Locked(CancelReason::CANCEL); }
            res->success = true; res->error_code = State::E_NONE; res->message = "cancel requested for " + task_.id; return;
        default: fail("invalid op " + std::to_string(req->op)); return;
    }
}

void NavigationFacade::onTimer() {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    if (task_.state == State::RUNNING && progress_timeout_sec_ > 0.0 && (now() - task_.last_feedback).seconds() > progress_timeout_sec_) {
        RCLCPP_WARN(get_logger(), "task %s: no nav2 feedback for %.1f s, cancelling", task_.id.c_str(), progress_timeout_sec_);
        cancelNav2Locked(CancelReason::TIMEOUT);
    }
    publishStateLocked();   // 1 Hz 心跳
}

// ──────────────────────────────────────────────────────────────────────
// 发布
// ──────────────────────────────────────────────────────────────────────

void NavigationFacade::publishStateLocked() {
    State s;
    s.header.stamp = now(); s.header.frame_id = global_frame_;
    s.task_id = task_.id; s.state = task_.state; s.error_code = task_.error_code; s.target_type = task_.target_type; s.target_name = task_.target_name;
    if (!task_.waypoints.empty()) { s.target_pose = task_.waypoints.back(); s.waypoints = task_.waypoints; }
    s.current_index = task_.current_index; s.distance_remaining = task_.distance_remaining; s.eta_sec = task_.eta_sec;
    s.poses_remaining = task_.poses_remaining; s.recoveries = task_.recoveries; s.plan_seq = task_.plan_seq; s.message = task_.message;
    state_pub_->publish(s);
}

void NavigationFacade::publishEmptyPathLocked() {
    nav_interfaces::msg::NavigationPath p;
    p.header.stamp = now(); p.header.frame_id = global_frame_;
    p.task_id = task_.id; p.plan_seq = task_.plan_seq; p.path.header = p.header;
    global_path_pub_->publish(p);
}

}  // namespace command_board
