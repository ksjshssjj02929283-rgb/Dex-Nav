#ifndef LOOP_NAVIGATION_BT_HPP_
#define LOOP_NAVIGATION_BT_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace loop_navigation_bt {

// Forward declaration
class BtActionNode : public BT::StatefulActionNode {
public:
    BtActionNode(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node_ptr)
        : BT::StatefulActionNode(name, config), node_(node_ptr) {}

protected:
    rclcpp::Node::SharedPtr node_;
};

// 1. LoopNavigateFourPoints: Cycles through a list of navigation goals.
class LoopNavigateFourPoints : public BtActionNode {
public:
    LoopNavigateFourPoints(const std::string& name,
                           const BT::NodeConfiguration& config,
                           rclcpp::Node::SharedPtr node_ptr);
    ~LoopNavigateFourPoints();

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    enum class RunningAction {
        NONE,
        NAVIGATE_TO_POINT,
        NAVIGATE_ROUTE,
    };

    struct Pose {
        double x, y, yaw;
    };

    bool getCurrentPose(Pose& current_pose,
                        std::chrono::milliseconds timeout = std::chrono::milliseconds(3000),
                        std::chrono::milliseconds retry_interval = std::chrono::milliseconds(200));
    size_t findNearestGoalIndex(const Pose& current) const;
    bool chooseInitialForward(size_t start_index) const;
    std::vector<geometry_msgs::msg::PoseStamped> buildRouteFromIndex(size_t start_index, bool forward) const;
    BT::NodeStatus sendNavigateToPoint(size_t goal_index);
    BT::NodeStatus sendRoute(const std::vector<geometry_msgs::msg::PoseStamped>& route, bool forward, bool partial);

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr route_client_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr point_client_;
    std::vector<geometry_msgs::msg::PoseStamped> goals_;

    bool initial_alignment_done_ = false;
    bool pending_partial_route_ = false;
    bool recovery_after_failure_ = false;
    bool forward_ = true;  // Direction for the next whole-route navigation.
    bool partial_forward_ = true;
    bool recovery_forward_ = true;
    size_t partial_start_index_ = 0;

    RunningAction running_action_ = RunningAction::NONE;
    bool active_route_partial_ = false;
    bool active_route_forward_ = true;
    bool active_point_from_recovery_ = false;
    size_t active_route_target_index_ = 0;
    size_t active_point_index_ = 0;

    std::shared_future<GoalHandleNavigateThroughPoses::SharedPtr> route_future_goal_handle_;
    GoalHandleNavigateThroughPoses::SharedPtr route_goal_handle_;
    std::atomic<bool> route_result_ready_ = false;
    rclcpp_action::ResultCode route_result_code_ = rclcpp_action::ResultCode::UNKNOWN;

    std::shared_future<GoalHandleNavigateToPose::SharedPtr> point_future_goal_handle_;
    GoalHandleNavigateToPose::SharedPtr point_goal_handle_;
    std::atomic<bool> point_result_ready_ = false;
    rclcpp_action::ResultCode point_result_code_ = rclcpp_action::ResultCode::UNKNOWN;
};

// 2. CheckBatteryAboveThreshold: Checks if battery level is above a certain threshold.
class CheckBatteryAboveThreshold : public BT::ConditionNode {
public:
    CheckBatteryAboveThreshold(const std::string& name,
                               const BT::NodeConfiguration& config,
                               rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    void battery_callback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr battery_sub_;
    float latest_battery_percentage_ = -1.0f;  // Initialize with an invalid value
};

// 3. CancelNavigation: Cancels the current navigation goal.
class CancelNavigation : public BtActionNode {
public:
    CancelNavigation(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override { return BT::NodeStatus::SUCCESS; }  // Should be quick
    void onHalted() override {}

private:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
};

// 4. Undocking: Performs the undocking procedure.
class Undocking : public BtActionNode {
public:
    Undocking(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::chrono::time_point<std::chrono::steady_clock> start_time_;
    double duration_sec_ = 3.0;         // default duration if not provided
    const double linear_speed_ = 0.15;  // m/s as requested
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

    void publish_cmd(double linear_x);
};

// 5. NavigateToFirstPointAndStop: On low battery, navigate to point[0] then signal shutdown.
class NavigateToFirstPointAndStop : public BtActionNode {
public:
    NavigateToFirstPointAndStop(const std::string& name,
                                const BT::NodeConfiguration& config,
                                rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    std::vector<geometry_msgs::msg::PoseStamped> goals_;
    std::shared_future<GoalHandleNavigateToPose::SharedPtr> future_goal_handle_;
    GoalHandleNavigateToPose::SharedPtr goal_handle_;
    std::atomic<bool> result_ready_ = false;
    rclcpp_action::ResultCode result_code_ = rclcpp_action::ResultCode::UNKNOWN;
};

}  // namespace loop_navigation_bt

#endif  // LOOP_NAVIGATION_BT_HPP_
