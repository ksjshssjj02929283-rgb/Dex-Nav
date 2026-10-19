#include "status_monitor.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace command_board {

using namespace std::chrono_literals;

namespace {

bool ParseBoolField(const std::string& value) {
    std::string lower = value;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return lower == "true";
}

// diagnostics_agg status 段名（PowerBoard 电池 + Safety 安全）。
constexpr const char* kPowerBoardName  = "/Devices/System/PowerBoard/collector_node: PowerBoard";
constexpr const char* kSafetyName      = "/Devices/System/PowerBoard/collector_node: PowerBoard Safety";

}  // namespace

StatusMonitor::StatusMonitor(rclcpp::Node* node) : node_(node) {
    snapshot_.stamp = node_->now();

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/localization/odometry/filtered", qos_reliable,
        std::bind(&StatusMonitor::OnOdom, this, std::placeholders::_1));

    task_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "mission_manager/status", qos_reliable,
        std::bind(&StatusMonitor::OnTaskStatus, this, std::placeholders::_1));

    diagnostics_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics_agg", qos_reliable,
        std::bind(&StatusMonitor::OnDiagnosticsAgg, this, std::placeholders::_1));

    localization_sub_ = node_->create_subscription<nav_interfaces::msg::LocalizationState>(
        "/localization/state", qos_reliable,
        std::bind(&StatusMonitor::OnLocalizationState, this, std::placeholders::_1));

    map_state_sub_ = node_->create_subscription<nav_interfaces::msg::MapState>(
        "map_manager/map_state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&StatusMonitor::OnMapState, this, std::placeholders::_1));

    battery_pub_ = node_->create_publisher<nav_interfaces::msg::BatteryInfo>(
        "/battery_info", qos_reliable);
    nav_status_pub_ = node_->create_publisher<nav_interfaces::msg::NavSystemStatus>(
        "~/nav_system_status", qos_reliable);

    nav_status_timer_ = node_->create_wall_timer(
        1s, std::bind(&StatusMonitor::PublishNavStatus, this));

    RCLCPP_INFO(node_->get_logger(),
                "StatusMonitor ready. Subscribed: /feedback/odom, mission_manager/status, "
                "/diagnostics_agg, /localization/state; pose via TF map->base_link. "
                "Publishing: /battery_info, ~/nav_system_status (1 Hz).");
}

StatusMonitor::Snapshot StatusMonitor::GetSnapshot() const {
    // Refresh pose on read (cheap TF lookup; avoids needing a /tf subscription).
    const_cast<StatusMonitor*>(this)->UpdatePoseFromTF();

    std::lock_guard<std::mutex> lock(mutex_);
    Snapshot copy = snapshot_;
    copy.stamp = node_->now();
    return copy;
}

void StatusMonitor::SetRecoverStatus(const std::string& state, const std::string& alarm) {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.recover_state = state;
    snapshot_.safety_alarm  = alarm;
}

void StatusMonitor::SetNavFault(
    const int32_t code, const std::string& detail,
    const std::vector<nav_interfaces::msg::SensorState>& sensors) {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.nav_fault_code   = code;
    snapshot_.nav_fault_detail = detail;
    snapshot_.sensors          = sensors;
}

void StatusMonitor::PublishNow() {
    PublishNavStatus();
}

bool StatusMonitor::IsEstopActive() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_.is_estop_active;
}

void StatusMonitor::OnMapState(nav_interfaces::msg::MapState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.current_map_name = msg->map_name;
    RCLCPP_INFO(node_->get_logger(), "StatusMonitor: map_name = '%s'", msg->map_name.c_str());
}

void StatusMonitor::UpdatePoseFromTF() {
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException&) {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.pose_valid = false;
        return;
    }
    tf2::Quaternion q;
    tf2::fromMsg(tf.transform.rotation, q);
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.pose.x     = tf.transform.translation.x;
    snapshot_.pose.y     = tf.transform.translation.y;
    snapshot_.pose.theta = tf2::getYaw(q);
    snapshot_.pose_valid = true;
}

void StatusMonitor::OnOdom(nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.velocity        = msg->twist.twist;
    snapshot_.velocity_valid  = true;
    snapshot_.odom_pose       = msg->pose.pose;
    snapshot_.odom_pose_valid = true;
}

void StatusMonitor::OnTaskStatus(std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.task_status       = msg->data;
    snapshot_.task_status_valid = true;
}

void StatusMonitor::OnLocalizationState(nav_interfaces::msg::LocalizationState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.localization       = *msg;
    snapshot_.localization_valid = true;
}

void StatusMonitor::OnDiagnosticsAgg(diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
    for (const auto& status : msg->status) {
        // ── 安全输入：estop / bumper（PowerBoard Safety 段）──
        if (status.name == kSafetyName) {
            bool estop = false, bumper_front = false, bumper_rear = false;
            for (const auto& kv : status.values) {
                if (kv.key == "estop_state")                   estop = ParseBoolField(kv.value);
                else if (kv.key == "bumper_strip_front_state") bumper_front = ParseBoolField(kv.value);
                else if (kv.key == "bumper_strip_rear_state")  bumper_rear = ParseBoolField(kv.value);
            }
            std::lock_guard<std::mutex> lock(mutex_);
            snapshot_.is_estop_active     = estop;
            snapshot_.bumper_front_active = bumper_front;
            snapshot_.bumper_rear_active  = bumper_rear;
        }
        // ── 电池：voltage / current / temperature / percentage / charging（PowerBoard 段）──
        else if (status.name == kPowerBoardName) {
            float voltage = 0.0f, current = 0.0f, temperature = 0.0f;
            uint16_t soc = 0;
            bool charging = false, got = false;
            for (const auto& kv : status.values) {
                if (kv.key == "battery_voltage")          { voltage = std::strtof(kv.value.c_str(), nullptr); got = true; }
                else if (kv.key == "battery_current")     { current = std::strtof(kv.value.c_str(), nullptr); }
                else if (kv.key == "battery_temperature") { temperature = std::strtof(kv.value.c_str(), nullptr); }
                else if (kv.key == "battery_percentage")  { soc = static_cast<uint16_t>(std::strtol(kv.value.c_str(), nullptr, 10)); }
                else if (kv.key == "battery_charging")    { charging = ParseBoolField(kv.value); }
            }
            if (got) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    snapshot_.battery_voltage  = voltage;
                    snapshot_.battery_current  = current;
                    snapshot_.battery_soc      = soc;
                    snapshot_.battery_charging = charging;
                    snapshot_.battery_valid    = true;
                }
                // 发布标准 BatteryInfo（command_board 从 diagnostics 解析后成为电池信息来源）。
                nav_interfaces::msg::BatteryInfo bat;
                bat.header.stamp = node_->now();
                bat.voltage      = voltage;
                bat.current      = current;
                bat.temperature  = temperature;
                bat.soc          = soc;
                bat.charge_state = charging;
                battery_pub_->publish(bat);
            }
        }
    }
}

void StatusMonitor::PublishNavStatus() {
    const auto snap = GetSnapshot();

    nav_interfaces::msg::NavSystemStatus msg;
    msg.header.stamp = node_->now();

    msg.battery_percentage = snap.battery_soc;
    msg.battery_charging   = snap.battery_charging;
    msg.battery_voltage    = snap.battery_voltage;
    msg.battery_current    = snap.battery_current;

    msg.estop_active        = snap.is_estop_active;
    msg.bumper_front_active = snap.bumper_front_active;
    msg.bumper_rear_active  = snap.bumper_rear_active;

    msg.localization = snap.localization;

    msg.pose     = snap.odom_pose;
    msg.velocity = snap.velocity;

    msg.task_status = snap.task_status;
    msg.current_map_name = snap.current_map_name;

    msg.recover_state = snap.recover_state;
    msg.safety_alarm  = snap.safety_alarm;

    msg.nav_fault_code   = snap.nav_fault_code;
    msg.nav_fault_detail = snap.nav_fault_detail;
    msg.sensors          = snap.sensors;

    nav_status_pub_->publish(msg);
}

}  // namespace command_board
