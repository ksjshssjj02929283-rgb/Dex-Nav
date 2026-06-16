#include "loop_navigation_bt/loop_navigation_bt.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <future>
#include <limits>

#include "nlohmann/json.hpp"

namespace loop_navigation_bt {

// Helper function to parse JSON file for navigation points
std::vector<geometry_msgs::msg::PoseStamped> parseNavPoints(const std::string& json_path, rclcpp::Logger logger) {
    std::vector<geometry_msgs::msg::PoseStamped> points;
    std::ifstream file(json_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(logger, "Failed to open navigation points file: %s", json_path.c_str());
        return points;
    }
    nlohmann::json j;
    file >> j;
    std::string frame = j.value("frame_id", std::string("map"));
    for (const auto& item : j["points"]) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame;
        pose.header.stamp = rclcpp::Clock().now();
        pose.pose.position.x = item["position"].value("x", 0.0);
        pose.pose.position.y = item["position"].value("y", 0.0);
        pose.pose.position.z = item["position"].value("z", 0.0);
        pose.pose.orientation.x = item["orientation"].value("x", 0.0);
        pose.pose.orientation.y = item["orientation"].value("y", 0.0);
        pose.pose.orientation.z = item["orientation"].value("z", 0.0);
        pose.pose.orientation.w = item["orientation"].value("w", 1.0);
        points.push_back(pose);
    }
    RCLCPP_INFO(logger, "Successfully parsed %zu navigation points.", points.size());
    return points;
}

//--- Implementation for LoopNavigateFourPoints ---
LoopNavigateFourPoints::LoopNavigateFourPoints(const std::string& name,
                                               const BT::NodeConfiguration& config,
                                               rclcpp::Node::SharedPtr node_ptr)
    : BtActionNode(name, config, node_ptr) {
    route_client_ = rclcpp_action::create_client<NavigateThroughPoses>(node_, "/navigate_through_poses");
    point_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");

    // Initialize TF2 buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

LoopNavigateFourPoints::~LoopNavigateFourPoints() = default;

BT::PortsList LoopNavigateFourPoints::providedPorts() {
    return {BT::InputPort<std::string>("points", "Path to the JSON file with navigation points")};
}

BT::NodeStatus LoopNavigateFourPoints::onStart() {
    // If navigation is paused (e.g. during undocking/charging), don't dispatch a new goal
    try {
        bool paused = config().blackboard->get<bool>("nav_paused");
        if (paused) {
            RCLCPP_DEBUG(node_->get_logger(), "Navigation paused -> skip goal dispatch");
            return BT::NodeStatus::SUCCESS;
        }
    } catch (...) {
    }

    // Check if we should stop due to low battery (for no-charging mode)
    try {
        bool should_stop = config().blackboard->get<bool>("should_stop_navigation");
        if (should_stop) {
            RCLCPP_INFO(node_->get_logger(), "Stop flag detected -> skip goal dispatch");
            return BT::NodeStatus::SUCCESS;
        }
    } catch (...) {
    }

    // Load points once
    if (goals_.empty()) {
        auto json_path = getInput<std::string>("points");
        if (!json_path) {
            RCLCPP_ERROR(node_->get_logger(), "'points' port is missing.");
            return BT::NodeStatus::FAILURE;
        }
        goals_ = parseNavPoints(json_path.value(), node_->get_logger());
        if (goals_.empty()) {
            return BT::NodeStatus::FAILURE;
        }
    }

    if (!route_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "Action server /navigate_through_poses not available.");
        return BT::NodeStatus::FAILURE;
    }
    if (!point_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "Action server /navigate_to_pose not available.");
        return BT::NodeStatus::FAILURE;
    }

    if (!initial_alignment_done_ || recovery_after_failure_) {
        Pose current_pose{};
        if (!getCurrentPose(current_pose)) {
            RCLCPP_WARN(node_->get_logger(),
                        "Failed to get current pose within timeout. Will retry nearest-point alignment on next tick.");
            return BT::NodeStatus::FAILURE;
        }
        partial_start_index_ = findNearestGoalIndex(current_pose);
        active_point_from_recovery_ = recovery_after_failure_;
        partial_forward_ = recovery_after_failure_ ? recovery_forward_ : chooseInitialForward(partial_start_index_);
        pending_partial_route_ = true;
        recovery_after_failure_ = false;

        RCLCPP_INFO(node_->get_logger(), "%s nearest route point[%zu] (%.2f, %.2f), partial direction: %s",
                    initial_alignment_done_ ? "Recovering from failed route at" : "Starting route from",
                    partial_start_index_, goals_[partial_start_index_].pose.position.x,
                    goals_[partial_start_index_].pose.position.y, partial_forward_ ? "FORWARD" : "REVERSE");
        return sendNavigateToPoint(partial_start_index_);
    }

    if (pending_partial_route_) {
        const auto partial_route = buildRouteFromIndex(partial_start_index_, partial_forward_);
        if (partial_route.empty()) {
            RCLCPP_INFO(node_->get_logger(),
                        "Nearest point is already at the %s end of the route. Skipping empty partial route.",
                        partial_forward_ ? "forward" : "reverse");
            pending_partial_route_ = false;
            forward_ = !partial_forward_;
            return BT::NodeStatus::SUCCESS;
        }
        return sendRoute(partial_route, partial_forward_, true);
    }

    auto whole_route = goals_;
    if (!forward_) {
        std::reverse(whole_route.begin(), whole_route.end());
    }
    return sendRoute(whole_route, forward_, false);
}

BT::NodeStatus LoopNavigateFourPoints::onRunning() {
    // If paused mid-execution, cancel current goal and yield SUCCESS
    try {
        bool paused = config().blackboard->get<bool>("nav_paused");
        if (paused) {
            if (route_goal_handle_) {
                RCLCPP_INFO(node_->get_logger(), "Navigation paused: cancel current goal");
                route_client_->async_cancel_goal(route_goal_handle_);
            }
            if (point_goal_handle_) {
                RCLCPP_INFO(node_->get_logger(), "Navigation paused: cancel current point goal");
                point_client_->async_cancel_goal(point_goal_handle_);
            }
            route_goal_handle_.reset();
            point_goal_handle_.reset();
            route_result_ready_ = false;
            point_result_ready_ = false;
            running_action_ = RunningAction::NONE;
            return BT::NodeStatus::SUCCESS;
        }
    } catch (...) {
    }

    if (running_action_ == RunningAction::NAVIGATE_TO_POINT) {
        if (!point_goal_handle_) {
            if (point_future_goal_handle_.valid() &&
                point_future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                auto tmp = point_future_goal_handle_.get();
                if (!tmp) {
                    RCLCPP_ERROR(node_->get_logger(), "Nearest point goal rejected by server.");
                    running_action_ = RunningAction::NONE;
                    return BT::NodeStatus::FAILURE;
                }
            }
            return BT::NodeStatus::RUNNING;
        }

        if (!point_result_ready_) {
            return BT::NodeStatus::RUNNING;
        }

        if (point_result_code_ == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(node_->get_logger(), "Reached nearest route point[%zu].", active_point_index_);
            initial_alignment_done_ = true;
        } else if (point_result_code_ == rclcpp_action::ResultCode::ABORTED) {
            RCLCPP_WARN(node_->get_logger(), "Navigation to nearest route point[%zu] aborted. Retrying alignment.",
                        active_point_index_);
            if (active_point_from_recovery_) {
                recovery_after_failure_ = true;
                recovery_forward_ = partial_forward_;
            } else {
                initial_alignment_done_ = false;
            }
            pending_partial_route_ = false;
        } else if (point_result_code_ == rclcpp_action::ResultCode::CANCELED) {
            RCLCPP_WARN(node_->get_logger(), "Navigation to nearest route point[%zu] canceled. Return FAILURE.",
                        active_point_index_);
            point_goal_handle_.reset();
            point_result_ready_ = false;
            running_action_ = RunningAction::NONE;
            return BT::NodeStatus::FAILURE;
        }

        point_goal_handle_.reset();
        point_result_ready_ = false;
        running_action_ = RunningAction::NONE;
        return BT::NodeStatus::SUCCESS;
    }

    if (running_action_ != RunningAction::NAVIGATE_ROUTE) {
        return BT::NodeStatus::RUNNING;
    }

    // Wait until route goal is accepted
    if (!route_goal_handle_) {
        if (route_future_goal_handle_.valid() &&
            route_future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            auto tmp = route_future_goal_handle_.get();
            if (!tmp) {
                RCLCPP_ERROR(node_->get_logger(), "Route rejected by server.");
                running_action_ = RunningAction::NONE;
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::RUNNING;
    }

    // Wait for route result
    if (!route_result_ready_) {
        return BT::NodeStatus::RUNNING;
    }

    // Handle route result
    if (route_result_code_ == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node_->get_logger(), "%s %s route completed successfully.",
                    active_route_forward_ ? "FORWARD" : "REVERSE", active_route_partial_ ? "partial" : "whole");

        // Let TF stabilize
        rclcpp::sleep_for(std::chrono::seconds(2));

        if (active_route_partial_) {
            pending_partial_route_ = false;
            forward_ = !active_route_forward_;
        } else {
            forward_ = !forward_;
        }
        route_goal_handle_.reset();
        route_result_ready_ = false;
        running_action_ = RunningAction::NONE;
        return BT::NodeStatus::SUCCESS;
    } else if (route_result_code_ == rclcpp_action::ResultCode::ABORTED) {
        RCLCPP_WARN(node_->get_logger(),
                    "%s %s route aborted. Re-entering from nearest point and keeping failed direction.",
                    active_route_forward_ ? "FORWARD" : "REVERSE", active_route_partial_ ? "partial" : "whole");
        recovery_after_failure_ = true;
        recovery_forward_ = active_route_forward_;
        pending_partial_route_ = false;
        route_goal_handle_.reset();
        route_result_ready_ = false;
        running_action_ = RunningAction::NONE;
        return BT::NodeStatus::SUCCESS;
    } else if (route_result_code_ == rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_WARN(node_->get_logger(), "%s route canceled. Return FAILURE.",
                    active_route_forward_ ? "FORWARD" : "REVERSE");
        route_goal_handle_.reset();
        route_result_ready_ = false;
        running_action_ = RunningAction::NONE;
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

size_t LoopNavigateFourPoints::findNearestGoalIndex(const Pose& current) const {
    size_t nearest_index = 0;
    double nearest_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < goals_.size(); ++i) {
        const auto& position = goals_[i].pose.position;
        const double dist = std::hypot(current.x - position.x, current.y - position.y);
        if (dist < nearest_dist) {
            nearest_dist = dist;
            nearest_index = i;
        }
    }

    return nearest_index;
}

bool LoopNavigateFourPoints::chooseInitialForward(size_t start_index) const {
    const size_t forward_remaining = goals_.empty() ? 0 : goals_.size() - 1 - start_index;
    const size_t reverse_remaining = start_index;
    return forward_remaining < reverse_remaining;
}

std::vector<geometry_msgs::msg::PoseStamped> LoopNavigateFourPoints::buildRouteFromIndex(size_t start_index,
                                                                                         bool forward) const {
    std::vector<geometry_msgs::msg::PoseStamped> route;
    if (goals_.empty() || start_index >= goals_.size()) {
        return route;
    }

    if (forward) {
        for (size_t i = start_index + 1; i < goals_.size(); ++i) {
            route.push_back(goals_[i]);
        }
    } else {
        for (size_t i = start_index; i > 0; --i) {
            route.push_back(goals_[i - 1]);
        }
    }

    return route;
}

BT::NodeStatus LoopNavigateFourPoints::sendNavigateToPoint(size_t goal_index) {
    if (goal_index >= goals_.size()) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid nearest point index: %zu", goal_index);
        return BT::NodeStatus::FAILURE;
    }

    point_result_ready_ = false;
    point_result_code_ = rclcpp_action::ResultCode::UNKNOWN;
    point_goal_handle_.reset();
    active_point_index_ = goal_index;
    running_action_ = RunningAction::NAVIGATE_TO_POINT;

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goals_[goal_index];

    RCLCPP_INFO(node_->get_logger(), "Sending nearest point[%zu] goal via navigate_to_pose: (%.2f, %.2f)", goal_index,
                goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](GoalHandleNavigateToPose::SharedPtr gh) {
        if (!gh) {
            RCLCPP_ERROR(node_->get_logger(), "Nearest point goal rejected by server");
        } else {
            point_goal_handle_ = gh;
        }
    };
    send_goal_options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult& result) {
        point_result_code_ = result.code;
        point_result_ready_ = true;
    };

    point_future_goal_handle_ = point_client_->async_send_goal(goal_msg, send_goal_options);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LoopNavigateFourPoints::sendRoute(const std::vector<geometry_msgs::msg::PoseStamped>& route,
                                                 bool forward,
                                                 bool partial) {
    if (route.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Refusing to send an empty route.");
        return BT::NodeStatus::FAILURE;
    }

    route_result_ready_ = false;
    route_result_code_ = rclcpp_action::ResultCode::UNKNOWN;
    route_goal_handle_.reset();
    active_route_partial_ = partial;
    active_route_forward_ = forward;
    active_route_target_index_ = forward ? goals_.size() - 1 : 0;
    running_action_ = RunningAction::NAVIGATE_ROUTE;

    auto goal_msg = NavigateThroughPoses::Goal();
    goal_msg.poses = route;

    RCLCPP_INFO(node_->get_logger(), "Sending %s %s route with %zu poses via navigate_through_poses",
                forward ? "FORWARD" : "REVERSE", partial ? "partial" : "whole", route.size());

    auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](GoalHandleNavigateThroughPoses::SharedPtr gh) {
        if (!gh) {
            RCLCPP_ERROR(node_->get_logger(), "Route goal rejected by server");
        } else {
            route_goal_handle_ = gh;
        }
    };
    send_goal_options.result_callback = [this](const GoalHandleNavigateThroughPoses::WrappedResult& result) {
        route_result_code_ = result.code;
        route_result_ready_ = true;
    };

    route_future_goal_handle_ = route_client_->async_send_goal(goal_msg, send_goal_options);
    return BT::NodeStatus::RUNNING;
}

bool LoopNavigateFourPoints::getCurrentPose(Pose& current_pose,
                                            std::chrono::milliseconds timeout,
                                            std::chrono::milliseconds retry_interval) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    bool warned_once = false;

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
        try {
            const auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

            current_pose.x = transform.transform.translation.x;
            current_pose.y = transform.transform.translation.y;

            tf2::Quaternion q;
            tf2::fromMsg(transform.transform.rotation, q);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            current_pose.yaw = yaw;
            return true;
        } catch (const tf2::TransformException& ex) {
            if (!warned_once) {
                RCLCPP_WARN(node_->get_logger(), "Could not get robot pose immediately: %s. Retrying for %.1f seconds.",
                            ex.what(), timeout.count() / 1000.0);
                warned_once = true;
            }
        }

        rclcpp::sleep_for(retry_interval);
    }

    RCLCPP_WARN(node_->get_logger(), "Could not get robot pose within %.1f seconds.", timeout.count() / 1000.0);
    return false;
}

void LoopNavigateFourPoints::onHalted() {
    if (route_goal_handle_) {
        RCLCPP_WARN(node_->get_logger(), "LoopNavigateFourPoints halted, canceling route.");
        route_client_->async_cancel_goal(route_goal_handle_);
        route_goal_handle_.reset();
    }
    if (point_goal_handle_) {
        RCLCPP_WARN(node_->get_logger(), "LoopNavigateFourPoints halted, canceling point goal.");
        point_client_->async_cancel_goal(point_goal_handle_);
        point_goal_handle_.reset();
    }
    route_result_ready_ = false;
    point_result_ready_ = false;
    running_action_ = RunningAction::NONE;
}

//--- Implementation for CheckBatteryAboveThreshold ---
CheckBatteryAboveThreshold::CheckBatteryAboveThreshold(const std::string& name,
                                                       const BT::NodeConfiguration& config,
                                                       rclcpp::Node::SharedPtr node_ptr)
    : BT::ConditionNode(name, config), node_(node_ptr) {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    battery_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics_agg", qos, std::bind(&CheckBatteryAboveThreshold::battery_callback, this, std::placeholders::_1));
}

BT::PortsList CheckBatteryAboveThreshold::providedPorts() {
    return {BT::InputPort<double>("threshold", "Minimum battery percentage (0.0 to 100.0)")};
}

void CheckBatteryAboveThreshold::battery_callback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
    // Default to invalid until we find the field
    float parsed = -1.0f;
    for (const auto& status : msg->status) {
        if (status.name == "/Devices/System/PowerBoard/collector_node: PowerBoard") {
            for (const auto& kv : status.values) {
                if (kv.key == "battery_percentage") {
                    try {
                        parsed = static_cast<float>(std::stod(kv.value));
                    } catch (...) {
                        parsed = -1.0f;
                    }
                    break;
                }
            }
            break;
        }
    }
    RCLCPP_INFO(node_->get_logger(), "BATTERY: %0.3f", parsed);
    latest_battery_percentage_ = parsed;
}

BT::NodeStatus CheckBatteryAboveThreshold::tick() {
    auto threshold = getInput<double>("threshold");
    if (!threshold) {
        RCLCPP_ERROR(node_->get_logger(), "'threshold' port is missing.");
        return BT::NodeStatus::FAILURE;
    }

    // Process pending diagnostics callbacks to refresh latest_battery_percentage_
    rclcpp::spin_some(node_);

    // If charging process is active, force FAILURE only for low-threshold checks (e.g. 0.35 at top-level)
    try {
        bool charging = config().blackboard->get<bool>("charging_active");
        if (charging && threshold.value() < 66) {
            RCLCPP_DEBUG(node_->get_logger(), "Charging active -> low-threshold battery check forced to FAILURE");
            return BT::NodeStatus::FAILURE;
        }
    } catch (...) {
    }

    if (latest_battery_percentage_ < 0) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Battery state not yet received.");
        return BT::NodeStatus::FAILURE;  // Keep retrying
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Checking battery: %.2f%% against threshold > %.2f%%", latest_battery_percentage_ * 100,
                         threshold.value() * 100);

    bool battery_ok = latest_battery_percentage_ >= threshold.value();

    // If battery is low, set stop flag to prevent new navigation goals
    if (!battery_ok) {
        try {
            config().blackboard->set("should_stop_navigation", true);
            RCLCPP_WARN(node_->get_logger(), "Battery low! Setting stop flag.");
        } catch (...) {
        }
    }

    return battery_ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

//--- Implementation for CancelNavigation ---
CancelNavigation::CancelNavigation(const std::string& name,
                                   const BT::NodeConfiguration& config,
                                   rclcpp::Node::SharedPtr node_ptr)
    : BtActionNode(name, config, node_ptr) {
    action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");
}

BT::NodeStatus CancelNavigation::onStart() {
    RCLCPP_INFO(node_->get_logger(), "Canceling all navigation goals.");
    action_client_->async_cancel_all_goals();
    return BT::NodeStatus::SUCCESS;
}

//--- Implementation for Undocking ---
Undocking::Undocking(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node_ptr)
    : BtActionNode(name, config, node_ptr) {
    vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("control/cmd_vel", rclcpp::SystemDefaultsQoS());
}

BT::PortsList Undocking::providedPorts() {
    return {BT::InputPort<double>("duration", 3.0, "Undocking duration in seconds")};
}

void Undocking::publish_cmd(double linear_x) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_x;
    cmd.angular.z = 0.0;
    vel_pub_->publish(cmd);
}

BT::NodeStatus Undocking::onStart() {
    // Signal navigation loop to pause while undocking
    try {
        config().blackboard->set("nav_paused", true);
    } catch (...) {
        RCLCPP_WARN(node_->get_logger(), "Undocking: failed to set 'nav_paused' on blackboard");
    }

    // Read duration from port; default to 3.0 if not provided
    if (auto dur = getInput<double>("duration")) {
        duration_sec_ = dur.value();
    } else {
        RCLCPP_WARN(node_->get_logger(), "Undocking: no 'duration' provided, using default %.2f s", duration_sec_);
    }

    if (duration_sec_ <= 0.0) {
        RCLCPP_WARN(node_->get_logger(), "Undocking: non-positive duration (%.2f), stopping immediately",
                    duration_sec_);
        publish_cmd(0.0);
        // Clear pause flag immediately
        try {
            config().blackboard->set("nav_paused", false);
        } catch (...) {
        }
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_INFO(node_->get_logger(), "Starting undocking for %.2f seconds at %.2f m/s", duration_sec_, linear_speed_);
    start_time_ = std::chrono::steady_clock::now();
    publish_cmd(linear_speed_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Undocking::onRunning() {
    auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
    if (elapsed >= duration_sec_) {
        publish_cmd(0.0);
        // Clear flags so navigation can resume and charging mode ends
        try {
            config().blackboard->set("nav_paused", false);
            config().blackboard->set("charging_active", false);
        } catch (...) {
        }
        RCLCPP_INFO(node_->get_logger(), "Undocking finished (ran %.2f / %.2f s)", elapsed, duration_sec_);
        return BT::NodeStatus::SUCCESS;
    }

    // keep moving forward
    publish_cmd(linear_speed_);
    return BT::NodeStatus::RUNNING;
}

void Undocking::onHalted() {
    RCLCPP_WARN(node_->get_logger(), "Undocking halted. Stopping robot.");
    publish_cmd(0.0);
}

//--- Implementation for NavigateToFirstPointAndStop ---
NavigateToFirstPointAndStop::NavigateToFirstPointAndStop(const std::string& name,
                                                         const BT::NodeConfiguration& config,
                                                         rclcpp::Node::SharedPtr node_ptr)
    : BtActionNode(name, config, node_ptr) {
    action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");
}

BT::PortsList NavigateToFirstPointAndStop::providedPorts() {
    return {BT::InputPort<std::string>("points", "Path to the JSON file with navigation points")};
}

BT::NodeStatus NavigateToFirstPointAndStop::onStart() {
    if (goals_.empty()) {
        auto json_path = getInput<std::string>("points");
        if (!json_path) {
            RCLCPP_ERROR(node_->get_logger(), "[LowBattery] 'points' port is missing.");
            return BT::NodeStatus::FAILURE;
        }
        goals_ = parseNavPoints(json_path.value(), node_->get_logger());
        if (goals_.empty()) {
            return BT::NodeStatus::FAILURE;
        }
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "[LowBattery] Action server /navigate_to_pose not available.");
        return BT::NodeStatus::FAILURE;
    }

    result_ready_ = false;
    result_code_ = rclcpp_action::ResultCode::UNKNOWN;
    goal_handle_.reset();

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goals_[0];  // Always navigate to the first point

    RCLCPP_WARN(node_->get_logger(), "[LowBattery] Battery low! Navigating to first point (%.2f, %.2f) then stopping.",
                goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](GoalHandleNavigateToPose::SharedPtr gh) {
        if (!gh) {
            RCLCPP_ERROR(node_->get_logger(), "[LowBattery] Goal rejected by server.");
        } else {
            goal_handle_ = gh;
        }
    };
    send_goal_options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult& result) {
        result_code_ = result.code;
        result_ready_ = true;
    };

    future_goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToFirstPointAndStop::onRunning() {
    // Wait until goal is accepted
    if (!goal_handle_) {
        if (future_goal_handle_.valid() &&
            future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            auto tmp = future_goal_handle_.get();
            if (!tmp) {
                RCLCPP_ERROR(node_->get_logger(), "[LowBattery] Goal was rejected by server.");
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::RUNNING;
    }

    if (!result_ready_) {
        return BT::NodeStatus::RUNNING;
    }

    if (result_code_ == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node_->get_logger(), "[LowBattery] Reached first point. Signaling program to stop.");
    } else if (result_code_ == rclcpp_action::ResultCode::ABORTED) {
        RCLCPP_WARN(node_->get_logger(), "[LowBattery] Navigation to first point aborted. Stopping anyway.");
    } else {
        RCLCPP_WARN(node_->get_logger(), "[LowBattery] Navigation canceled or unknown result. Stopping anyway.");
    }

    // Regardless of result, signal the main loop to exit
    try {
        config().blackboard->set("low_battery_shutdown", true);
    } catch (...) {
        RCLCPP_ERROR(node_->get_logger(), "[LowBattery] Failed to set 'low_battery_shutdown' on blackboard.");
    }
    return BT::NodeStatus::SUCCESS;
}

void NavigateToFirstPointAndStop::onHalted() {
    if (goal_handle_) {
        RCLCPP_WARN(node_->get_logger(), "[LowBattery] NavigateToFirstPointAndStop halted, canceling goal.");
        action_client_->async_cancel_goal(goal_handle_);
        goal_handle_.reset();
    }
}

}  // namespace loop_navigation_bt
