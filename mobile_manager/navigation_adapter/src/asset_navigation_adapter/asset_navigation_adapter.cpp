#include "navigation_adapter/asset_navigation_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <thread>
#include <utility>

namespace navigation_adapter {

namespace {

constexpr const char* kStatusNavigating = "NAVIGATING";
constexpr const char* kStatusCanceled = "CANCELED";
constexpr const char* kStatusFailed = "FAILED";

template <typename ResultT>
void abortGoal(const ResultT& result, const std::string& message) {
    result->success = false;
    result->message = message;
}

}  // namespace

// ──────────────────────────────────────────────────────────────────────
// 构造与初始化
// ──────────────────────────────────────────────────────────────────────

AssetNavigationAdapter::AssetNavigationAdapter(rclcpp::Node* node) : node_(node) {
    node_->declare_parameter<std::string>("get_point_service", "/map_asset_manager/get_point");
    node_->declare_parameter<std::string>("get_all_points_service", "/map_asset_manager/get_all_nav_points");
    node_->declare_parameter<std::string>("get_route_service", "/map_asset_manager/get_route");
    node_->declare_parameter<std::string>("navigate_to_pose_action", "/navigate_to_pose");
    node_->declare_parameter<std::string>("navigate_through_poses_action", "/navigate_through_poses");
    node_->declare_parameter<std::string>("navigate_to_point_action", "~/navigate_to_point");
    node_->declare_parameter<std::string>("navigate_route_action", "~/navigate_route");
    node_->declare_parameter<std::string>("global_frame_id", "map");
    node_->declare_parameter<double>("service_timeout_sec", 2.0);
    node_->declare_parameter<double>("action_server_timeout_sec", 2.0);

    get_point_service_ = node_->get_parameter("get_point_service").as_string();
    get_all_points_service_ = node_->get_parameter("get_all_points_service").as_string();
    get_route_service_ = node_->get_parameter("get_route_service").as_string();
    navigate_to_pose_action_ = node_->get_parameter("navigate_to_pose_action").as_string();
    navigate_through_poses_action_ = node_->get_parameter("navigate_through_poses_action").as_string();
    navigate_to_point_action_ = node_->get_parameter("navigate_to_point_action").as_string();
    navigate_route_action_ = node_->get_parameter("navigate_route_action").as_string();
    global_frame_id_ = node_->get_parameter("global_frame_id").as_string();

    const double service_timeout_sec = node_->get_parameter("service_timeout_sec").as_double();
    const double action_server_timeout_sec = node_->get_parameter("action_server_timeout_sec").as_double();
    service_timeout_ =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(service_timeout_sec));
    action_server_timeout_ =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(action_server_timeout_sec));

    get_point_client_ = node_->create_client<nav_interfaces::srv::GetNavPoint>(get_point_service_);
    get_all_points_client_ = node_->create_client<nav_interfaces::srv::GetAllNavPoints>(get_all_points_service_);
    get_route_client_ = node_->create_client<nav_interfaces::srv::GetRoute>(get_route_service_);
    navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(node_, navigate_to_pose_action_);
    navigate_through_poses_client_ =
        rclcpp_action::create_client<NavigateThroughPoses>(node_, navigate_through_poses_action_);

    navigate_to_point_server_ = rclcpp_action::create_server<NavigateToPoint>(
        node_, navigate_to_point_action_,
        std::bind(&AssetNavigationAdapter::handleNavigateToPointGoal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&AssetNavigationAdapter::handleNavigateToPointCancel, this, std::placeholders::_1),
        std::bind(&AssetNavigationAdapter::handleNavigateToPointAccepted, this, std::placeholders::_1));

    navigate_route_server_ = rclcpp_action::create_server<NavigateRoute>(
        node_, navigate_route_action_,
        std::bind(&AssetNavigationAdapter::handleNavigateRouteGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&AssetNavigationAdapter::handleNavigateRouteCancel, this, std::placeholders::_1),
        std::bind(&AssetNavigationAdapter::handleNavigateRouteAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "AssetNavigationAdapter started");
    RCLCPP_INFO(node_->get_logger(), "  get_point_service: %s", get_point_service_.c_str());
    RCLCPP_INFO(node_->get_logger(), "  get_all_points_service: %s", get_all_points_service_.c_str());
    RCLCPP_INFO(node_->get_logger(), "  get_route_service: %s", get_route_service_.c_str());
    RCLCPP_INFO(node_->get_logger(), "  navigate_to_pose_action: %s", navigate_to_pose_action_.c_str());
    RCLCPP_INFO(node_->get_logger(), "  navigate_through_poses_action: %s", navigate_through_poses_action_.c_str());
}

// shutdown 时等待 action worker 结束，避免 detached thread 继续访问已析构模块。
// worker 内部会感知 shutting_down_ 并尽量向 Nav2 转发取消。
AssetNavigationAdapter::~AssetNavigationAdapter() {
    shutting_down_.store(true);

    std::vector<std::thread> threads;
    {
        std::lock_guard<std::mutex> lock(worker_threads_mutex_);
        threads.swap(worker_threads_);
    }

    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
}

// ──────────────────────────────────────────────────────────────────────
// Action Server 回调
// ──────────────────────────────────────────────────────────────────────

rclcpp_action::GoalResponse AssetNavigationAdapter::handleNavigateToPointGoal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const NavigateToPoint::Goal> goal) {
    (void)uuid;

    if (!goal->use_pose && goal->point_name.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Reject navigate_to_point goal: point_name is empty");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (shutting_down_.load() || !rclcpp::ok()) {
        RCLCPP_WARN(node_->get_logger(), "Reject navigate_to_point goal: node is shutting down");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (!tryStartNavigation()) {
        RCLCPP_WARN(node_->get_logger(), "Reject navigate_to_point goal: another navigation is active");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(node_->get_logger(), "Accept navigate_to_point goal: point='%s' use_pose=%s", goal->point_name.c_str(),
                goal->use_pose ? "true" : "false");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AssetNavigationAdapter::handleNavigateToPointCancel(
    const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle) {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void AssetNavigationAdapter::handleNavigateToPointAccepted(
    const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle) {
    startNavigateToPointWorker(goal_handle);
}

rclcpp_action::GoalResponse AssetNavigationAdapter::handleNavigateRouteGoal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const NavigateRoute::Goal> goal) {
    (void)uuid;

    if (goal->route_name.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Reject navigate_route goal: route_name is empty");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (shutting_down_.load() || !rclcpp::ok()) {
        RCLCPP_WARN(node_->get_logger(), "Reject navigate_route goal: node is shutting down");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (!tryStartNavigation()) {
        RCLCPP_WARN(node_->get_logger(), "Reject navigate_route goal: another navigation is active");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(node_->get_logger(), "Accept navigate_route goal: route='%s'", goal->route_name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AssetNavigationAdapter::handleNavigateRouteCancel(
    const std::shared_ptr<GoalHandleNavigateRoute> goal_handle) {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void AssetNavigationAdapter::handleNavigateRouteAccepted(const std::shared_ptr<GoalHandleNavigateRoute> goal_handle) {
    startNavigateRouteWorker(goal_handle);
}

void AssetNavigationAdapter::startNavigateToPointWorker(const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle) {
    std::lock_guard<std::mutex> lock(worker_threads_mutex_);
    worker_threads_.emplace_back(&AssetNavigationAdapter::executeNavigateToPoint, this, goal_handle);
}

void AssetNavigationAdapter::startNavigateRouteWorker(const std::shared_ptr<GoalHandleNavigateRoute> goal_handle) {
    std::lock_guard<std::mutex> lock(worker_threads_mutex_);
    worker_threads_.emplace_back(&AssetNavigationAdapter::executeNavigateRoute, this, goal_handle);
}

// ──────────────────────────────────────────────────────────────────────
// 导航执行
// ──────────────────────────────────────────────────────────────────────

// 将点位名称或自定义 pose 转成标准 NavigateToPose goal。
// 失败时终止本层 action；取消时向已下发的 Nav2 goal 继续转发取消请求。
void AssetNavigationAdapter::executeNavigateToPoint(const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle) {
    auto result = std::make_shared<NavigateToPoint::Result>();
    const auto goal = goal_handle->get_goal();

    auto feedback = std::make_shared<NavigateToPoint::Feedback>();
    feedback->current_status = kStatusNavigating;
    goal_handle->publish_feedback(feedback);

    geometry_msgs::msg::PoseStamped pose;
    std::string target_label;
    if (goal->use_pose) {
        pose = normalizePose(goal->pose);
        target_label = goal->point_name.empty() ? "custom_pose" : goal->point_name;
        if (goal->tolerance > 0.0) {
            RCLCPP_WARN(node_->get_logger(),
                        "NavigateToPoint tolerance %.3f is not applied by standard NavigateToPose in V1",
                        goal->tolerance);
        }
    } else {
        nav_interfaces::msg::NavPoint point;
        std::string message;
        if (!getPoint(goal->point_name, &point, &message)) {
            abortGoal(result, message);
            goal_handle->abort(result);
            finishNavigation();
            return;
        }
        pose = toPoseStamped(point);
        target_label = point.name;
        if (point.tolerance > 0.0f) {
            RCLCPP_WARN(node_->get_logger(), "NavPoint tolerance %.3f is not applied by standard NavigateToPose in V1",
                        point.tolerance);
        }
    }

    if (shutting_down_.load() || !rclcpp::ok()) {
        result->success = false;
        result->message = kStatusCanceled;
        goal_handle->canceled(result);
        finishNavigation();
        return;
    }

    if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = kStatusCanceled;
        goal_handle->canceled(result);
        finishNavigation();
        return;
    }

    if (!navigate_to_pose_client_->wait_for_action_server(action_server_timeout_)) {
        abortGoal(result, "Nav2 navigate_to_pose action server is not available");
        goal_handle->abort(result);
        finishNavigation();
        return;
    }

    NavigateToPose::Goal nav2_goal;
    nav2_goal.pose = pose;

    RCLCPP_INFO(node_->get_logger(), "Dispatch NavigateToPose: target='%s' x=%.3f y=%.3f", target_label.c_str(),
                pose.pose.position.x, pose.pose.position.y);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.feedback_callback = [goal_handle](GoalHandleNavigateToPose::SharedPtr,
                                                        const std::shared_ptr<const NavigateToPose::Feedback>) {
        auto fb = std::make_shared<NavigateToPoint::Feedback>();
        fb->current_status = kStatusNavigating;
        goal_handle->publish_feedback(fb);
    };

    auto goal_future = navigate_to_pose_client_->async_send_goal(nav2_goal, send_goal_options);
    if (goal_future.wait_for(action_server_timeout_) != std::future_status::ready) {
        abortGoal(result, "Timed out while sending goal to Nav2 navigate_to_pose");
        goal_handle->abort(result);
        finishNavigation();
        return;
    }

    auto nav2_goal_handle = goal_future.get();
    if (!nav2_goal_handle) {
        abortGoal(result, "Nav2 navigate_to_pose rejected goal");
        goal_handle->abort(result);
        finishNavigation();
        return;
    }

    std::string message;
    const auto result_future = navigate_to_pose_client_->async_get_result(nav2_goal_handle);
    if (!waitForNav2NavigateToPoseResult(goal_handle, nav2_goal_handle, result_future, &message)) {
        result->success = false;
        result->message = message;
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
        } else {
            goal_handle->abort(result);
        }
        finishNavigation();
        return;
    }

    result->success = true;
    result->message = "Navigation to point '" + target_label + "' succeeded";
    RCLCPP_INFO(node_->get_logger(), "NavigateToPoint succeeded: target='%s'", target_label.c_str());
    goal_handle->succeed(result);
    finishNavigation();
}

// 将路线名称按启动时点位快照展开，再一次性下发 NavigateThroughPoses。
// 导航执行过程中不跟随资产变化，避免过程点被修改导致任务不可复现。
void AssetNavigationAdapter::executeNavigateRoute(const std::shared_ptr<GoalHandleNavigateRoute> goal_handle) {
    auto result = std::make_shared<NavigateRoute::Result>();
    const auto goal = goal_handle->get_goal();

    std::vector<std::string> point_names;
    std::string message;
    if (!getRoutePointNames(goal->route_name, &point_names, &message)) {
        abortGoal(result, message);
        goal_handle->abort(result);
        finishNavigation();
        return;
    }

    if (point_names.empty()) {
        abortGoal(result, "Route '" + goal->route_name + "' has no points");
        goal_handle->abort(result);
        finishNavigation();
        return;
    }

    std::unordered_map<std::string, nav_interfaces::msg::NavPoint> points_snapshot;
    if (!getAllPointsSnapshot(&points_snapshot, &message)) {
        abortGoal(result, message);
        goal_handle->abort(result);
        finishNavigation();
        return;
    }

    std::vector<geometry_msgs::msg::PoseStamped> poses;
    poses.reserve(point_names.size());
    for (const auto& point_name : point_names) {
        const auto it = points_snapshot.find(point_name);
        if (it == points_snapshot.end()) {
            abortGoal(result, "Route point '" + point_name + "' is missing in point snapshot");
            goal_handle->abort(result);
            finishNavigation();
            return;
        }
        if (shutting_down_.load() || !rclcpp::ok()) {
            result->success = false;
            result->message = kStatusCanceled;
            goal_handle->canceled(result);
            finishNavigation();
            return;
        }
        if (goal_handle->is_canceling()) {
            result->success = false;
            result->message = kStatusCanceled;
            goal_handle->canceled(result);
            finishNavigation();
            return;
        }
        poses.push_back(toPoseStamped(it->second));
    }

    if (!navigate_through_poses_client_->wait_for_action_server(action_server_timeout_)) {
        abortGoal(result, "Nav2 navigate_through_poses action server is not available");
        goal_handle->abort(result);
        finishNavigation();
        return;
    }

    auto initial_feedback = std::make_shared<NavigateRoute::Feedback>();
    initial_feedback->current_status = kStatusNavigating;
    initial_feedback->current_point_index = 0;
    initial_feedback->total_points = static_cast<uint32_t>(poses.size());
    goal_handle->publish_feedback(initial_feedback);

    NavigateThroughPoses::Goal nav2_goal;
    nav2_goal.poses = poses;

    RCLCPP_INFO(node_->get_logger(), "Dispatch NavigateThroughPoses: route='%s' poses=%zu snapshot_points=%zu",
                goal->route_name.c_str(), poses.size(), points_snapshot.size());

    auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.feedback_callback =
        [goal_handle, total_points = poses.size()](
            GoalHandleNavigateThroughPoses::SharedPtr,
            const std::shared_ptr<const NavigateThroughPoses::Feedback> nav2_feedback) {
            auto fb = std::make_shared<NavigateRoute::Feedback>();
            fb->current_status = kStatusNavigating;
            fb->total_points = static_cast<uint32_t>(total_points);

            const auto remaining = std::max<int16_t>(nav2_feedback->number_of_poses_remaining, 0);
            const size_t done =
                total_points > static_cast<size_t>(remaining) ? total_points - static_cast<size_t>(remaining) : 0U;
            fb->current_point_index =
                static_cast<uint32_t>(std::min(done, total_points == 0U ? 0U : total_points - 1U));
            goal_handle->publish_feedback(fb);
        };

    auto goal_future = navigate_through_poses_client_->async_send_goal(nav2_goal, send_goal_options);
    if (goal_future.wait_for(action_server_timeout_) != std::future_status::ready) {
        abortGoal(result, "Timed out while sending goal to Nav2 navigate_through_poses");
        goal_handle->abort(result);
        finishNavigation();
        return;
    }

    auto nav2_goal_handle = goal_future.get();
    if (!nav2_goal_handle) {
        abortGoal(result, "Nav2 navigate_through_poses rejected goal");
        goal_handle->abort(result);
        finishNavigation();
        return;
    }

    const auto result_future = navigate_through_poses_client_->async_get_result(nav2_goal_handle);
    if (!waitForNav2NavigateThroughPosesResult(goal_handle, nav2_goal_handle, result_future, &message)) {
        result->success = false;
        result->message = message;
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
        } else {
            goal_handle->abort(result);
        }
        finishNavigation();
        return;
    }

    result->success = true;
    result->message = "Route '" + goal->route_name + "' succeeded";
    RCLCPP_INFO(node_->get_logger(), "NavigateRoute succeeded: route='%s'", goal->route_name.c_str());
    goal_handle->succeed(result);
    finishNavigation();
}

// ──────────────────────────────────────────────────────────────────────
// 资产查询
// ──────────────────────────────────────────────────────────────────────

// 查询单个点位资产，返回用于下发 Nav2 的原始点位信息。
// 失败时通过 message 给出面向 action result 的可读原因。
bool AssetNavigationAdapter::getPoint(const std::string& name,
                                      nav_interfaces::msg::NavPoint* point,
                                      std::string* message) {
    if (!get_point_client_->wait_for_service(service_timeout_)) {
        *message = "get_point service is not available: " + get_point_service_;
        return false;
    }

    auto request = std::make_shared<nav_interfaces::srv::GetNavPoint::Request>();
    request->name = name;
    auto future = get_point_client_->async_send_request(request);
    if (future.wait_for(service_timeout_) != std::future_status::ready) {
        get_point_client_->remove_pending_request(future);
        *message = "get_point timed out for '" + name + "'";
        return false;
    }

    auto response = future.get();
    if (!response->success) {
        *message = "point '" + name + "' not found";
        return false;
    }

    *point = response->point;
    return true;
}

// 获取路线启动时的点位快照，用于一次性解析 NavigateThroughPoses。
// 本次导航后续不跟随资产变化，保持路线执行语义稳定。
bool AssetNavigationAdapter::getAllPointsSnapshot(
    std::unordered_map<std::string, nav_interfaces::msg::NavPoint>* points, std::string* message) {
    if (!get_all_points_client_->wait_for_service(service_timeout_)) {
        *message = "get_all_points service is not available: " + get_all_points_service_;
        return false;
    }

    auto request = std::make_shared<nav_interfaces::srv::GetAllNavPoints::Request>();
    auto future = get_all_points_client_->async_send_request(request);
    if (future.wait_for(service_timeout_) != std::future_status::ready) {
        get_all_points_client_->remove_pending_request(future);
        *message = "get_all_points timed out";
        return false;
    }

    auto response = future.get();
    if (!response->success) {
        *message = "get_all_points failed";
        return false;
    }

    points->clear();
    points->reserve(response->points.size());
    for (const auto& point : response->points) {
        (*points)[point.name] = point;
    }
    RCLCPP_INFO(node_->get_logger(), "Loaded point snapshot for route navigation: points=%zu", points->size());
    return true;
}

// 查询路线资产，只返回有序点名列表；点位坐标由启动时快照统一解析。
// 这样 route 仍只表达资产关系，不直接耦合导航 pose。
bool AssetNavigationAdapter::getRoutePointNames(const std::string& name,
                                                std::vector<std::string>* point_names,
                                                std::string* message) {
    if (!get_route_client_->wait_for_service(service_timeout_)) {
        *message = "get_route service is not available: " + get_route_service_;
        return false;
    }

    auto request = std::make_shared<nav_interfaces::srv::GetRoute::Request>();
    request->name = name;
    auto future = get_route_client_->async_send_request(request);
    if (future.wait_for(service_timeout_) != std::future_status::ready) {
        get_route_client_->remove_pending_request(future);
        *message = "get_route timed out for '" + name + "'";
        return false;
    }

    auto response = future.get();
    if (!response->success) {
        *message = "route '" + name + "' not found";
        return false;
    }

    *point_names = response->point_names;
    return true;
}

// ──────────────────────────────────────────────────────────────────────
// 消息转换
// ──────────────────────────────────────────────────────────────────────

geometry_msgs::msg::PoseStamped AssetNavigationAdapter::toPoseStamped(
    const nav_interfaces::msg::NavPoint& point) const {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_id_;
    pose.header.stamp = node_->now();
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.z = std::sin(static_cast<double>(point.theta) * 0.5);
    pose.pose.orientation.w = std::cos(static_cast<double>(point.theta) * 0.5);
    return pose;
}

geometry_msgs::msg::PoseStamped AssetNavigationAdapter::normalizePose(
    const geometry_msgs::msg::PoseStamped& pose) const {
    auto normalized = pose;
    if (normalized.header.frame_id.empty()) {
        normalized.header.frame_id = global_frame_id_;
    }
    normalized.header.stamp = node_->now();

    const auto& q = normalized.pose.orientation;
    if (q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 0.0) {
        normalized.pose.orientation.w = 1.0;
    }
    return normalized;
}

// ──────────────────────────────────────────────────────────────────────
// Nav2 结果等待与取消转发
// ──────────────────────────────────────────────────────────────────────

// 等待 Nav2 NavigateToPose 结束，并在上层 action 取消时转发 cancel。
// 返回 false 时 message 表示取消或失败原因，由调用方决定 abort/canceled。
bool AssetNavigationAdapter::waitForNav2NavigateToPoseResult(
    const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle,
    const GoalHandleNavigateToPose::SharedPtr& nav2_goal_handle,
    std::shared_future<GoalHandleNavigateToPose::WrappedResult> result_future,
    std::string* message) {
    while (result_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
        if (shutting_down_.load() || !rclcpp::ok()) {
            if (rclcpp::ok()) {
                navigate_to_pose_client_->async_cancel_goal(nav2_goal_handle);
            }
            *message = kStatusCanceled;
            return false;
        }
        if (goal_handle->is_canceling()) {
            navigate_to_pose_client_->async_cancel_goal(nav2_goal_handle);
            *message = kStatusCanceled;
            return false;
        }
    }

    const auto wrapped_result = result_future.get();
    switch (wrapped_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            return true;
        case rclcpp_action::ResultCode::CANCELED:
            *message = kStatusCanceled;
            return false;
        default:
            *message = kStatusFailed;
            return false;
    }
}

// 等待 Nav2 NavigateThroughPoses 结束，并在上层 action 取消时转发 cancel。
// 返回 false 时 message 表示取消或失败原因，由调用方决定 abort/canceled。
bool AssetNavigationAdapter::waitForNav2NavigateThroughPosesResult(
    const std::shared_ptr<GoalHandleNavigateRoute> goal_handle,
    const GoalHandleNavigateThroughPoses::SharedPtr& nav2_goal_handle,
    std::shared_future<GoalHandleNavigateThroughPoses::WrappedResult> result_future,
    std::string* message) {
    while (result_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
        if (shutting_down_.load() || !rclcpp::ok()) {
            if (rclcpp::ok()) {
                navigate_through_poses_client_->async_cancel_goal(nav2_goal_handle);
            }
            *message = kStatusCanceled;
            return false;
        }
        if (goal_handle->is_canceling()) {
            navigate_through_poses_client_->async_cancel_goal(nav2_goal_handle);
            *message = kStatusCanceled;
            return false;
        }
    }

    const auto wrapped_result = result_future.get();
    switch (wrapped_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            return true;
        case rclcpp_action::ResultCode::CANCELED:
            *message = kStatusCanceled;
            return false;
        default:
            *message = kStatusFailed;
            return false;
    }
}

// ──────────────────────────────────────────────────────────────────────
// 运行状态
// ──────────────────────────────────────────────────────────────────────

bool AssetNavigationAdapter::tryStartNavigation() {
    std::lock_guard<std::mutex> lock(navigation_mutex_);
    if (navigation_active_) {
        return false;
    }
    navigation_active_ = true;
    return true;
}

void AssetNavigationAdapter::finishNavigation() {
    std::lock_guard<std::mutex> lock(navigation_mutex_);
    navigation_active_ = false;
}

}  // namespace navigation_adapter
