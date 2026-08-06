#ifndef LOOP_NAVIGATION_BT_HPP_
#define LOOP_NAVIGATION_BT_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <chrono>
#include <cstdint>
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
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace loop_navigation_bt {

// 行为树 ROS Action 节点的公共基类，统一保存 ROS 2 节点指针。
class BtActionNode : public BT::StatefulActionNode {
public:
    BtActionNode(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node_ptr)
        : BT::StatefulActionNode(name, config), node_(node_ptr) {}

protected:
    rclcpp::Node::SharedPtr node_;
};

// 按 JSON 中的点位顺序循环发送 Nav2 目标，并记录每次导航误差。
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

    struct DistanceSnapshot {
        double goal_distance_m = 0.0;        // 当前导航目标执行期间的实际轨迹长度。
        double cumulative_distance_m = 0.0;  // 程序启动后所有导航目标的累计轨迹长度。
        bool active = false;                 // 停止统计前是否正在跟踪导航目标。
        bool valid = false;                  // 当前目标期间是否收到过有效里程计。
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
    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void startDistanceTracking();
    DistanceSnapshot stopDistanceTracking();
    void spinNodeForDuration(double duration_sec, const char* reason = nullptr);
    void logNavigationError(const Pose& goal,
                            const NavigationErrorSample& error_sample,
                            const DistanceSnapshot& distance_snapshot,
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

    // 仅在 Nav2 目标执行期间积分里程计，不统计自动回充和退桩运动。
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::mutex distance_mutex_;
    std::string odom_topic_ = "/feedback/odom";
    double odom_max_increment_m_ = 0.5;
    double current_goal_distance_m_ = 0.0;
    double cumulative_navigation_distance_m_ = 0.0;
    double last_odom_x_ = 0.0;
    double last_odom_y_ = 0.0;
    bool distance_tracking_active_ = false;
    bool has_last_odom_pose_ = false;
    bool received_odom_for_goal_ = false;

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
    std::vector<double> goal_wait_times_sec_;
    unsigned int current_goal_index_ = 0;
    int total_rounds_ = 0;
    int completed_rounds_ = 0;
    bool navigation_complete_ = false;
    bool stop_flag_logged_ = false;  // 每次停止标志置位期间只打印一次提示。
    rclcpp::Time goal_ros_start_time_;
    std::chrono::time_point<std::chrono::steady_clock> goal_start_time_;
    std::shared_future<GoalHandleNavigateToPose::SharedPtr> future_goal_handle_;
    GoalHandleNavigateToPose::SharedPtr goal_handle_;
    std::atomic<bool> result_ready_ = false;
    rclcpp_action::ResultCode result_code_ = rclcpp_action::ResultCode::UNKNOWN;
    // 每次发送目标都会递增；旧目标的异步回调晚到时会因代次不匹配而被忽略。
    std::atomic<std::uint64_t> goal_generation_{0};
    // 点位未单独配置时使用的停留时间；成功到点和导航失败共用。
    double wait_time_sec_ = 2.0;
    // 等待 Nav2 Action server 上线的最长时间。
    double action_server_wait_timeout_sec_ = 2.0;
    // 误差日志采样窗口：默认按 10Hz 连续采 10 次，对误差本身求平均。
    std::size_t pose_sample_count_ = 10;
    double pose_sample_interval_sec_ = 0.1;
};

// 从底盘诊断信息中读取电量，并判断是否达到行为树传入的阈值。
class CheckBatteryAboveThreshold : public BT::ConditionNode {
public:
    CheckBatteryAboveThreshold(const std::string& name,
                               const BT::NodeConfiguration& config,
                               rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    void battery_callback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg);
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr battery_sub_;
    float latest_battery_percentage_ = -1.0f;  // -1 表示尚未收到有效电量。
};

// 等待收到有效电量并达到指定阈值；未达到时保持 RUNNING，不进行同步忙重试。
class WaitForBatteryAboveThreshold : public BtActionNode {
public:
    WaitForBatteryAboveThreshold(const std::string& name,
                                 const BT::NodeConfiguration& config,
                                 rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override {}

private:
    void battery_callback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg);
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr battery_sub_;
    float latest_battery_percentage_ = -1.0f;  // -1 表示尚未收到有效电量。
    bool battery_updated_ = false;  // 每条有效电量消息最多输出一次条件日志。
};

// 取消当前所有 Nav2 导航目标，避免导航和回充流程同时控制机器人。
class CancelNavigation : public BtActionNode {
public:
    CancelNavigation(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override { return BT::NodeStatus::SUCCESS; }  // 取消请求发送后即可继续。
    void onHalted() override {}

private:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
};

// 充电完成后按指定速度和时长前进，使机器人退出充电桩。
class Undocking : public BtActionNode {
public:
    Undocking(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::chrono::time_point<std::chrono::steady_clock> start_time_;
    double duration_sec_ = 3.0;      // 退桩持续时间，单位：秒。
    double linear_speed_ = 0.15;     // 退桩线速度，单位：m/s，正值表示向前。
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

    void publish_cmd(double linear_x);
};

// 无回充模式低电量时返回第一个点，完成后通知主循环退出。
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
    double action_server_wait_timeout_sec_ = 2.0;  // 等待 Nav2 Action server 的最长时间。
};

}  // namespace loop_navigation_bt

#endif  // LOOP_NAVIGATION_BT_HPP_
