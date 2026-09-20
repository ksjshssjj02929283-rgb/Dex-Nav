#include "map_asset_manager/navigation_executor.hpp"

namespace map_asset_manager {

NavigationExecutor::NavigationExecutor(rclcpp::Node::SharedPtr node)
    : node_(node), current_state_(NavState::IDLE) {
    node_->declare_parameter<std::string>("controller_server_name", "/controller_server");
    node_->declare_parameter<double>("default_tolerance", 0.05);

    controller_server_name_ = node_->get_parameter("controller_server_name").as_string();
    default_tolerance_       = node_->get_parameter("default_tolerance").as_double();

    nav2_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");
    nav2_through_poses_client_ =
        rclcpp_action::create_client<NavigateThroughPoses>(node_, "/navigate_through_poses");
    set_param_client_ =
        node_->create_client<rcl_interfaces::srv::SetParameters>(controller_server_name_ + "/set_parameters");

    RCLCPP_INFO(node_->get_logger(), "NavigationExecutor initialized (no database dependency)");
}

// ──────────────────────────────────────────────────────────────────────
// 公共接口
// ──────────────────────────────────────────────────────────────────────

bool NavigationExecutor::navigateToPose(const std::string& label,
                                        const geometry_msgs::msg::PoseStamped& pose,
                                        double tolerance) {
    if (!setNav2GoalTolerance(tolerance > 0.0 ? tolerance : default_tolerance_)) {
        RCLCPP_WARN(node_->get_logger(), "Failed to set tolerance, using Nav2 default");
    }

    if (!nav2_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "Nav2 navigate_to_pose server not available");
        return false;
    }

    auto goal_msg        = NavigateToPose::Goal();
    goal_msg.pose        = pose;

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    opts.goal_response_callback =
        std::bind(&NavigationExecutor::goalResponseCallback, this, std::placeholders::_1);
    opts.feedback_callback =
        std::bind(&NavigationExecutor::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    opts.result_callback = std::bind(&NavigationExecutor::resultCallback, this, std::placeholders::_1);

    auto future = nav2_client_->async_send_goal(goal_msg, opts);
    if (future.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to send goal to Nav2");
        return false;
    }

    current_goal_handle_ = future.get();
    if (!current_goal_handle_) {
        RCLCPP_ERROR(node_->get_logger(), "Goal rejected by Nav2");
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_  = NavState::NAVIGATING;
        current_target_ = label;
    }

    RCLCPP_INFO(node_->get_logger(), "Navigating to: %s", label.c_str());
    return true;
}

bool NavigationExecutor::navigateThroughPoses(const std::string& label,
                                              const std::vector<geometry_msgs::msg::PoseStamped>& poses) {
    if (poses.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "No poses provided for route navigation");
        return false;
    }

    if (!nav2_through_poses_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "Nav2 navigate_through_poses server not available");
        return false;
    }

    auto goal_msg  = NavigateThroughPoses::Goal();
    goal_msg.poses = poses;

    auto opts = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    opts.goal_response_callback =
        std::bind(&NavigationExecutor::goalResponseCallbackThroughPoses, this, std::placeholders::_1);
    opts.feedback_callback = std::bind(&NavigationExecutor::feedbackCallbackThroughPoses,
                                       this, std::placeholders::_1, std::placeholders::_2);
    opts.result_callback =
        std::bind(&NavigationExecutor::resultCallbackThroughPoses, this, std::placeholders::_1);

    auto future = nav2_through_poses_client_->async_send_goal(goal_msg, opts);
    if (future.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to send route goal");
        return false;
    }

    current_goal_handle_through_poses_ = future.get();
    if (!current_goal_handle_through_poses_) {
        RCLCPP_ERROR(node_->get_logger(), "Route goal rejected by Nav2");
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_  = NavState::NAVIGATING;
        current_target_ = label;
    }

    RCLCPP_INFO(node_->get_logger(), "Navigating route '%s' with %zu poses",
                label.c_str(), poses.size());
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

geometry_msgs::msg::PoseStamped NavigationExecutor::createPoseStamped(double x, double y, double yaw) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp    = node_->now();
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

// ──────────────────────────────────────────────────────────────────────
// 私有回调
// ──────────────────────────────────────────────────────────────────────

void NavigationExecutor::goalResponseCallback(const GoalHandleNavigateToPose::SharedPtr& gh) {
    if (!gh) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_ = NavState::FAILED;
        RCLCPP_ERROR(node_->get_logger(), "Goal rejected by Nav2");
    }
}

void NavigationExecutor::feedbackCallback(GoalHandleNavigateToPose::SharedPtr,
                                          const std::shared_ptr<const NavigateToPose::Feedback>) {}

void NavigationExecutor::resultCallback(const GoalHandleNavigateToPose::WrappedResult& result) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_goal_handle_ = nullptr;
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            current_state_ = NavState::SUCCESS;
            RCLCPP_INFO(node_->get_logger(), "Navigation succeeded");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            current_state_ = NavState::CANCELED;
            break;
        default:
            current_state_ = NavState::FAILED;
            RCLCPP_ERROR(node_->get_logger(), "Navigation aborted/failed");
            break;
    }
}

void NavigationExecutor::goalResponseCallbackThroughPoses(const GoalHandleNavigateThroughPoses::SharedPtr& gh) {
    if (!gh) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_ = NavState::FAILED;
    }
}

void NavigationExecutor::feedbackCallbackThroughPoses(
    GoalHandleNavigateThroughPoses::SharedPtr,
    const std::shared_ptr<const NavigateThroughPoses::Feedback> fb) {
    RCLCPP_DEBUG(node_->get_logger(), "Route navigation: %d poses remaining",
                 fb->number_of_poses_remaining);
}

void NavigationExecutor::resultCallbackThroughPoses(const GoalHandleNavigateThroughPoses::WrappedResult& result) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_goal_handle_through_poses_ = nullptr;
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            current_state_ = NavState::SUCCESS;
            RCLCPP_INFO(node_->get_logger(), "Route navigation succeeded");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            current_state_ = NavState::CANCELED;
            break;
        default:
            current_state_ = NavState::FAILED;
            RCLCPP_ERROR(node_->get_logger(), "Route navigation aborted/failed");
            break;
    }
}

bool NavigationExecutor::setNav2GoalTolerance(double tolerance) {
    if (!set_param_client_->wait_for_service(std::chrono::seconds(1))) return false;

    auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    for (const auto& param_name : {"goal_checker.lon_goal_tolerance", "goal_checker.lat_goal_tolerance"}) {
        rcl_interfaces::msg::Parameter p;
        p.name                = param_name;
        p.value.type          = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        p.value.double_value  = tolerance;
        req->parameters.push_back(p);
    }

    auto future = set_param_client_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) return false;
    auto res = future.get();
    return res->results.size() >= 2 && res->results[0].successful && res->results[1].successful;
}

}  // namespace map_asset_manager
