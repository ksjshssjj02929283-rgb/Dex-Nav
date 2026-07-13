#ifndef LOOP_NAVIGATION_BT_HPP_
#define LOOP_NAVIGATION_BT_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <chrono>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
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
    struct Pose {
        double x, y, yaw;
    };

    struct MetricStats {
        double mean = 0.0;
        double std_dev = 0.0;
        double sigma3_probability = 0.0;
    };

    struct NavigationErrorSample {
        Pose current = {0.0, 0.0, 0.0};
        double lon_error = 0.0;
        double lat_error = 0.0;
        double yaw_error = 0.0;
        double dist_error = 0.0;
        bool within_desired_tolerance = false;
        bool within_accept_tolerance = false;
        bool from_goal_checker = false;
        bool valid = false;
    };

    struct GoalErrorStats {
        std::string id;
        int succeeded = 0;
        int failed = 0;
        std::vector<double> elapsed_times;
        std::vector<double> success_elapsed_times;
        std::vector<double> failed_elapsed_times;
        std::vector<double> success_dist_errors;
        std::vector<double> success_lon_errors;
        std::vector<double> success_lat_errors;
        std::vector<double> success_yaw_errors;
        std::vector<double> failed_dist_errors;
        std::vector<double> failed_lon_errors;
        std::vector<double> failed_lat_errors;
        std::vector<double> failed_yaw_errors;
        int success_within_1cm = 0;
        int success_within_2cm = 0;
        int failed_within_1cm = 0;
        int failed_within_2cm = 0;
    };

    NavigationErrorSample getNavigationErrorSample(const Pose& goal);
    bool getLatestGoalCheckerErrorSample(const Pose& goal, NavigationErrorSample& sample);
    void spinNodeForDuration(double duration_sec);
    void logNavigationError(const Pose& goal,
                            const NavigationErrorSample& error_sample,
                            double elapsed_sec,
                            bool succeeded,
                            std::size_t goal_index);
    bool isWithinPositionTolerance(double lon_error, double lat_error, double tolerance) const;
    MetricStats calculateMetricStats(const std::vector<double>& data) const;
    void calculateAndLogStats(bool force = false);

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr goal_error_sub_;
    diagnostic_msgs::msg::DiagnosticArray latest_goal_error_;
    std::mutex latest_goal_error_mutex_;
    bool has_latest_goal_error_ = false;
    bool use_goal_checker_error_ = true;

    // Logging and stats
    std::ofstream log_file_;
    int succeeded_nav_ = 0;
    int failed_nav_ = 0;
    bool stats_logged_ = false;
    std::vector<double> dist_errors_;
    std::vector<double> lon_errors_;
    std::vector<double> lat_errors_;
    std::vector<double> yaw_errors_;
    std::vector<double> success_dist_errors_;
    std::vector<double> success_lon_errors_;
    std::vector<double> success_lat_errors_;
    std::vector<double> success_yaw_errors_;
    std::vector<double> failed_dist_errors_;
    std::vector<double> failed_lon_errors_;
    std::vector<double> failed_lat_errors_;
    std::vector<double> failed_yaw_errors_;
    int within_1cm_count_ = 0;
    int within_2cm_count_ = 0;
    int success_within_1cm_count_ = 0;
    int success_within_2cm_count_ = 0;
    int failed_within_1cm_count_ = 0;
    int failed_within_2cm_count_ = 0;
    std::vector<GoalErrorStats> per_goal_stats_;
    std::size_t stats_log_interval_ = 20;
    std::size_t last_stats_logged_sample_count_ = 0;
    // 位置误差统计阈值：默认对应 goal_checker 的期望精度和最终接受精度。
    double desired_position_tolerance_ = 0.01;
    double accept_position_tolerance_ = 0.02;

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    std::vector<geometry_msgs::msg::PoseStamped> goals_;
    std::vector<std::string> goal_ids_;
    unsigned int current_goal_index_ = 0;
    int total_rounds_ = 0;
    int completed_rounds_ = 0;
    bool navigation_complete_ = false;
    rclcpp::Time goal_ros_start_time_;
    std::chrono::time_point<std::chrono::steady_clock> goal_start_time_;
    std::shared_future<GoalHandleNavigateToPose::SharedPtr> future_goal_handle_;
    GoalHandleNavigateToPose::SharedPtr goal_handle_;
    std::atomic<bool> result_ready_ = false;
    rclcpp_action::ResultCode result_code_ = rclcpp_action::ResultCode::UNKNOWN;
    // 误差日志采样窗口：默认按 10Hz 连续采 10 次，对误差本身求平均。
    std::size_t pose_sample_count_ = 10;
    double pose_sample_interval_sec_ = 0.1;
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
