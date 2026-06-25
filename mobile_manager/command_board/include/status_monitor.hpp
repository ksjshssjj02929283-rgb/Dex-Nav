// nav2_command_board / StatusMonitor (Phase 3.2 + 3.4)
//
// Module ① — Aggregates state from upstream nodes into a thread-safe snapshot,
// and additionally publishes:
//   - /battery_info        (BatteryInfo)      parsed from /diagnostics_agg PowerBoard
//   - ~/nav_system_status  (NavSystemStatus)  aggregated battery/safety/localization/
//                                             pose+velocity/task, at 1 Hz.
// Snapshot fields feed AutoMapSaver's save conditions.

#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "nav_interfaces/msg/battery_info.hpp"
#include "nav_interfaces/msg/localization_state.hpp"
#include "nav_interfaces/msg/nav_system_status.hpp"

namespace command_board {

class StatusMonitor {
public:
    struct Snapshot {
        rclcpp::Time              stamp;
        geometry_msgs::msg::Pose2D pose;            // map -> base_link via TF (AutoMapSaver 位移检测)
        geometry_msgs::msg::Twist velocity;         // from /feedback/odom
        geometry_msgs::msg::Pose  odom_pose;        // from /feedback/odom (NavSystemStatus)
        uint16_t                  battery_soc = 0;       // 0-100
        float                     battery_voltage = 0.0f;
        float                     battery_current = 0.0f;
        bool                      battery_charging = false;
        bool                      battery_valid = false;
        bool                      is_estop_active = false;
        bool                      bumper_front_active = false;
        bool                      bumper_rear_active = false;
        nav_interfaces::msg::LocalizationState localization;  // from /localization_node/state
        bool                      localization_valid = false;
        std::string               task_status = "UNKNOWN";    // from mission_manager/status
        bool                      pose_valid = false;
        bool                      velocity_valid = false;
        bool                      odom_pose_valid = false;
        bool                      task_status_valid = false;
        std::string               recover_state = "NORMAL";  // SafetyReactor 急停丢位恢复状态机
        std::string               safety_alarm;              // SafetyReactor 告警（ALARM 时非空）
    };

    explicit StatusMonitor(rclcpp::Node* node);
    ~StatusMonitor() = default;

    StatusMonitor(const StatusMonitor&) = delete;
    StatusMonitor& operator=(const StatusMonitor&) = delete;

    // Thread-safe accessor — returns a copy of the latest aggregated snapshot.
    Snapshot GetSnapshot() const;

    // Thread-safe setter — SafetyReactor pushes its recovery state / alarm into
    // the aggregated NavSystemStatus.
    void SetRecoverStatus(const std::string& state, const std::string& alarm);

private:
    void OnOdom(nav_msgs::msg::Odometry::SharedPtr msg);
    void OnTaskStatus(std_msgs::msg::String::SharedPtr msg);
    void OnDiagnosticsAgg(diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
    void OnLocalizationState(nav_interfaces::msg::LocalizationState::SharedPtr msg);
    void UpdatePoseFromTF();   // called whenever GetSnapshot() reads; cheap lookup
    void PublishNavStatus();   // 1 Hz timer

    rclcpp::Node* node_;
    mutable std::mutex mutex_;
    Snapshot snapshot_;

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_status_sub_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
    rclcpp::Subscription<nav_interfaces::msg::LocalizationState>::SharedPtr localization_sub_;

    rclcpp::Publisher<nav_interfaces::msg::BatteryInfo>::SharedPtr battery_pub_;
    rclcpp::Publisher<nav_interfaces::msg::NavSystemStatus>::SharedPtr nav_status_pub_;
    rclcpp::TimerBase::SharedPtr nav_status_timer_;
};

}  // namespace command_board
