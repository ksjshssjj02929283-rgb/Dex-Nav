#include "nav_point_manager/navigation_executor.hpp"

#include <rcl_interfaces/srv/set_parameters.hpp>

namespace nav_point_manager {

NavigationExecutor::NavigationExecutor(rclcpp::Node::SharedPtr node, std::shared_ptr<PointDatabase> database)
    : node_(node), database_(database), current_state_(NavState::IDLE) {
    // 获取参数
    node_->declare_parameter<std::string>("controller_server_name", "/controller_server");
    node_->declare_parameter<double>("default_tolerance", 0.05);

    controller_server_name_ = node_->get_parameter("controller_server_name").as_string();
    default_tolerance_ = node_->get_parameter("default_tolerance").as_double();

    // 创建 Action clients
    nav2_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");
    nav2_through_poses_client_ = rclcpp_action::create_client<NavigateThroughPoses>(node_, "/navigate_through_poses");

    // 创建参数设置客户端
    set_param_client_ =
        node_->create_client<rcl_interfaces::srv::SetParameters>(controller_server_name_ + "/set_parameters");

    RCLCPP_INFO(node_->get_logger(), "NavigationExecutor initialized");
}

bool NavigationExecutor::navigateToPoint(const std::string& point_name) {
    auto point_opt = database_->getPoint(point_name);
    if (!point_opt.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "Point '%s' not found", point_name.c_str());
        return false;
    }

    const auto& point = point_opt.value();

    // 设置容差
    double tolerance = (point.tolerance > 0.0) ? point.tolerance : default_tolerance_;
    if (!setNav2GoalTolerance(tolerance)) {
        RCLCPP_WARN(node_->get_logger(), "Failed to set tolerance, using Nav2 default");
    }

    // 创建目标姿态
    auto goal_pose = createPoseStamped(point.x, point.y, point.yaw);

    // 检查 action server
    if (!nav2_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "Nav2 action server not available");
        return false;
    }

    // 发送目标
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal_pose;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&NavigationExecutor::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&NavigationExecutor::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&NavigationExecutor::resultCallback, this, std::placeholders::_1);

    auto goal_handle_future = nav2_client_->async_send_goal(goal_msg, send_goal_options);

    if (goal_handle_future.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to send goal");
        return false;
    }

    current_goal_handle_ = goal_handle_future.get();
    if (!current_goal_handle_) {
        RCLCPP_ERROR(node_->get_logger(), "Goal rejected by Nav2");
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_ = NavState::NAVIGATING;
        current_target_ = point_name;
    }

    RCLCPP_INFO(node_->get_logger(), "Navigating to point: %s", point_name.c_str());
    return true;
}

bool NavigationExecutor::navigateToPose(const geometry_msgs::msg::PoseStamped& pose, double tolerance) {
    if (!setNav2GoalTolerance(tolerance)) {
        RCLCPP_WARN(node_->get_logger(), "Failed to set tolerance");
    }

    if (!nav2_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "Nav2 action server not available");
        return false;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = pose;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&NavigationExecutor::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&NavigationExecutor::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&NavigationExecutor::resultCallback, this, std::placeholders::_1);

    auto goal_handle_future = nav2_client_->async_send_goal(goal_msg, send_goal_options);

    if (goal_handle_future.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
        return false;
    }

    current_goal_handle_ = goal_handle_future.get();
    if (!current_goal_handle_) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_ = NavState::NAVIGATING;
        current_target_ = "custom_pose";
    }

    return true;
}
bool NavigationExecutor::navigateRoute(const std::string& route_name) {
    auto route_opt = database_->getRoute(route_name);
    if (!route_opt.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "Route '%s' not found", route_name.c_str());
        return false;
    }

    const auto& route = route_opt.value();

    // 构建路径点列表
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    for (const auto& point_name : route.point_names) {
        auto point_opt = database_->getPoint(point_name);
        if (!point_opt.has_value()) {
            RCLCPP_ERROR(node_->get_logger(), "Point '%s' in route not found", point_name.c_str());
            return false;
        }

        const auto& point = point_opt.value();
        poses.push_back(createPoseStamped(point.x, point.y, point.yaw));
    }

    if (poses.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Route '%s' has no valid points", route_name.c_str());
        return false;
    }

    // 检查 action server
    if (!nav2_through_poses_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "NavigateThroughPoses action server not available");
        return false;
    }

    // 发送目标
    auto goal_msg = NavigateThroughPoses::Goal();
    goal_msg.poses = poses;

    auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&NavigationExecutor::goalResponseCallbackThroughPoses, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&NavigationExecutor::feedbackCallbackThroughPoses, this,
                                                    std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&NavigationExecutor::resultCallbackThroughPoses, this, std::placeholders::_1);

    auto goal_handle_future = nav2_through_poses_client_->async_send_goal(goal_msg, send_goal_options);

    if (goal_handle_future.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to send route goal");
        return false;
    }

    current_goal_handle_through_poses_ = goal_handle_future.get();
    if (!current_goal_handle_through_poses_) {
        RCLCPP_ERROR(node_->get_logger(), "Route goal rejected by Nav2");
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_ = NavState::NAVIGATING;
        current_target_ = route_name;
    }

    RCLCPP_INFO(node_->get_logger(), "Navigating route: %s with %zu points", route_name.c_str(), poses.size());
    return true;
}

bool NavigationExecutor::cancelNavigation() {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (current_state_ != NavState::NAVIGATING) {
        RCLCPP_WARN(node_->get_logger(), "No active navigation to cancel");
        return false;
    }

    if (current_goal_handle_) {
        nav2_client_->async_cancel_goal(current_goal_handle_);
        current_goal_handle_ = nullptr;
    }

    if (current_goal_handle_through_poses_) {
        nav2_through_poses_client_->async_cancel_goal(current_goal_handle_through_poses_);
        current_goal_handle_through_poses_ = nullptr;
    }

    current_state_ = NavState::CANCELED;
    RCLCPP_INFO(node_->get_logger(), "Navigation canceled");
    return true;
}

NavState NavigationExecutor::getCurrentState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_state_;
}

std::string NavigationExecutor::getCurrentTarget() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_target_;
}

void NavigationExecutor::goalResponseCallback(const GoalHandleNavigateToPose::SharedPtr& goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "Goal rejected by Nav2");
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_ = NavState::FAILED;
    }
}

void NavigationExecutor::feedbackCallback(GoalHandleNavigateToPose::SharedPtr,
                                          const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
    (void)feedback;  // 未使用的参数
    // 可以在这里处理反馈信息
}

void NavigationExecutor::resultCallback(const GoalHandleNavigateToPose::WrappedResult& result) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_goal_handle_ = nullptr;

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            current_state_ = NavState::SUCCESS;
            RCLCPP_INFO(node_->get_logger(), "Navigation succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            current_state_ = NavState::FAILED;
            RCLCPP_ERROR(node_->get_logger(), "Navigation aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            current_state_ = NavState::CANCELED;
            RCLCPP_WARN(node_->get_logger(), "Navigation canceled");
            break;
        default:
            current_state_ = NavState::FAILED;
            RCLCPP_ERROR(node_->get_logger(), "Navigation failed with unknown result");
            break;
    }
}

void NavigationExecutor::goalResponseCallbackThroughPoses(
    const GoalHandleNavigateThroughPoses::SharedPtr& goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "Route goal rejected by Nav2");
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_ = NavState::FAILED;
    }
}

void NavigationExecutor::feedbackCallbackThroughPoses(
    GoalHandleNavigateThroughPoses::SharedPtr, const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback) {
    RCLCPP_DEBUG(node_->get_logger(), "Route navigation: %d poses remaining", feedback->number_of_poses_remaining);
}

void NavigationExecutor::resultCallbackThroughPoses(const GoalHandleNavigateThroughPoses::WrappedResult& result) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_goal_handle_through_poses_ = nullptr;

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            current_state_ = NavState::SUCCESS;
            RCLCPP_INFO(node_->get_logger(), "Route navigation succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            current_state_ = NavState::FAILED;
            RCLCPP_ERROR(node_->get_logger(), "Route navigation aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            current_state_ = NavState::CANCELED;
            RCLCPP_WARN(node_->get_logger(), "Route navigation canceled");
            break;
        default:
            current_state_ = NavState::FAILED;
            break;
    }
}

geometry_msgs::msg::PoseStamped NavigationExecutor::createPoseStamped(double x, double y, double yaw) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = node_->now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    return pose;
}

bool NavigationExecutor::setNav2GoalTolerance(double tolerance) {
    if (!set_param_client_->wait_for_service(std::chrono::seconds(1))) {
        return false;
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

    rcl_interfaces::msg::Parameter param_lon;
    param_lon.name = "goal_checker.lon_goal_tolerance";
    param_lon.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    param_lon.value.double_value = tolerance;
    request->parameters.push_back(param_lon);

    rcl_interfaces::msg::Parameter param_lat;
    param_lat.name = "goal_checker.lat_goal_tolerance";
    param_lat.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    param_lat.value.double_value = tolerance;
    request->parameters.push_back(param_lat);

    auto future = set_param_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
        return false;
    }

    auto result = future.get();
    return result->results.size() >= 2 && result->results[0].successful && result->results[1].successful;
}

}  // namespace nav_point_manager
