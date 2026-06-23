#include "auto_map_saver.hpp"

#include <chrono>
#include <cmath>
#include <ctime>
#include <sstream>

#include "status_monitor.hpp"

namespace command_board {

using namespace std::chrono_literals;

namespace {

double NormalizeAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

}  // namespace

AutoMapSaver::AutoMapSaver(rclcpp::Node* node, StatusMonitor* status_monitor)
    : node_(node), status_monitor_(status_monitor) {
    auto declare = [this](const std::string& name, auto& var) {
        var = node_->declare_parameter(name, var);
    };
    declare("auto_map_saver.enable",                          enable_);
    declare("auto_map_saver.daily_save_hour",                 daily_save_hour_);
    declare("auto_map_saver.daily_save_minute",               daily_save_minute_);
    declare("auto_map_saver.check_poll_interval_sec",         check_poll_interval_sec_);
    declare("auto_map_saver.save_service",                    save_service_);
    declare("auto_map_saver.save_service_timeout_sec",        save_service_timeout_sec_);
    declare("auto_map_saver.pose_change_threshold_m",         pose_change_threshold_m_);
    declare("auto_map_saver.yaw_change_threshold_rad",        yaw_change_threshold_rad_);
    declare("auto_map_saver.velocity_threshold_mps",          velocity_threshold_mps_);
    declare("auto_map_saver.angular_velocity_threshold_rps",  angular_velocity_threshold_rps_);
    declare("auto_map_saver.battery_soc_skip_below",          battery_soc_skip_below_);
    declare("auto_map_saver.skip_when_task_not_idle",         skip_when_task_not_idle_);

    // Sanity-clamp daily target.
    if (daily_save_hour_   < 0 || daily_save_hour_   > 23) daily_save_hour_   = 20;
    if (daily_save_minute_ < 0 || daily_save_minute_ > 59) daily_save_minute_ = 0;

    // If we start up after today's target time, treat today as already fired so
    // we don't fire immediately on launch.
    if (LocalTimeReached(daily_save_hour_, daily_save_minute_)) {
        last_fired_ymd_ = CurrentLocalYmd();
        RCLCPP_INFO(node_->get_logger(),
                    "[AutoMapSaver] Startup past today's target %02d:%02d — "
                    "next trigger tomorrow.",
                    daily_save_hour_, daily_save_minute_);
    }

    callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    save_map_cli_ = node_->create_client<nav_interfaces::srv::SaveMap>(save_service_);

    save_srv_ = node_->create_service<nav_interfaces::srv::SaveMap>(
        "~/save_map",
        std::bind(&AutoMapSaver::OnSaveMap, this,
                  std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_);

    status_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "~/auto_map_saver_status", rclcpp::QoS(1).transient_local());

    if (enable_) {
        const auto period = std::chrono::duration<double>(check_poll_interval_sec_);
        timer_ = node_->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&AutoMapSaver::OnTimer, this), callback_group_);
    }

    status_timer_ = node_->create_wall_timer(
        1s, std::bind(&AutoMapSaver::OnStatusTimer, this));

    RCLCPP_INFO(node_->get_logger(),
                "AutoMapSaver ready. enable=%d daily_target=%02d:%02d "
                "poll_interval=%.1fs save_service=%s. Service: ~/save_map.",
                enable_, daily_save_hour_, daily_save_minute_,
                check_poll_interval_sec_, save_service_.c_str());
}

int AutoMapSaver::CurrentLocalYmd() {
    const auto t  = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{};
    localtime_r(&t, &tm);
    return (tm.tm_year + 1900) * 10000 + (tm.tm_mon + 1) * 100 + tm.tm_mday;
}

bool AutoMapSaver::LocalTimeReached(int target_hour, int target_minute) {
    const auto t  = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{};
    localtime_r(&t, &tm);
    if (tm.tm_hour > target_hour) return true;
    if (tm.tm_hour < target_hour) return false;
    return tm.tm_min >= target_minute;
}

void AutoMapSaver::OnTimer() {
    // Daily gate: fire only when wall clock crosses today's target AND we
    // haven't fired today yet. Polling cadence is check_poll_interval_sec.
    const int today_ymd = CurrentLocalYmd();
    if (today_ymd == last_fired_ymd_) return;  // already attempted today
    if (!LocalTimeReached(daily_save_hour_, daily_save_minute_)) return;

    last_fired_ymd_   = today_ymd;  // mark attempted (even if skipped by conditions)
    any_attempt_made_ = true;

    const auto reason = CheckPreconditions();
    last_skip_reason_ = reason;

    if (reason != SkipReason::kNone) {
        RCLCPP_INFO(node_->get_logger(),
                    "[AutoMapSaver] Skipped (daily %02d:%02d on %d): reason=%s",
                    daily_save_hour_, daily_save_minute_, today_ymd,
                    SkipReasonToString(reason));
        return;
    }

    std::string srv_msg, map_path, backup_ts;
    if (!CallSaveMapService("", srv_msg, map_path, backup_ts)) {
        last_skip_reason_ = SkipReason::kSaveFailed;
        RCLCPP_ERROR(node_->get_logger(),
                     "[AutoMapSaver] save_map call failed: %s", srv_msg.c_str());
        return;
    }
    auto snap = status_monitor_->GetSnapshot();
    if (snap.pose_valid) {
        last_saved_pose_ = snap.pose;
        last_saved_pose_valid_ = true;
    }
    last_saved_time_ = node_->now();
    RCLCPP_INFO(node_->get_logger(),
                "[AutoMapSaver] Saved (daily %02d:%02d on %d): %s; pose=(%.3f, %.3f, %.4f)",
                daily_save_hour_, daily_save_minute_, today_ymd, srv_msg.c_str(),
                last_saved_pose_.x, last_saved_pose_.y, last_saved_pose_.theta);
}

void AutoMapSaver::OnSaveMap(
    const std::shared_ptr<nav_interfaces::srv::SaveMap::Request> req,
    std::shared_ptr<nav_interfaces::srv::SaveMap::Response> res) {
    any_attempt_made_ = true;
    const auto reason = CheckPreconditions();
    last_skip_reason_ = reason;
    if (reason != SkipReason::kNone) {
        // State not savable → skip, do not save.
        res->success = false;
        res->message = std::string("Skipped: ") + SkipReasonToString(reason);
        RCLCPP_INFO(node_->get_logger(), "[AutoMapSaver] %s", res->message.c_str());
        return;
    }
    std::string srv_msg, map_path, backup_ts;
    res->success   = CallSaveMapService(req->map_name, srv_msg, map_path, backup_ts);
    res->message   = srv_msg;
    res->map_path  = map_path;
    res->backup_ts = backup_ts;
    if (res->success) {
        auto snap = status_monitor_->GetSnapshot();
        if (snap.pose_valid) {
            last_saved_pose_ = snap.pose;
            last_saved_pose_valid_ = true;
        }
        last_saved_time_ = node_->now();
        RCLCPP_INFO(node_->get_logger(),
                    "[AutoMapSaver] Saved (~/save_map): %s (backup_ts=%s)",
                    srv_msg.c_str(), backup_ts.c_str());
    } else {
        last_skip_reason_ = SkipReason::kSaveFailed;
        RCLCPP_ERROR(node_->get_logger(),
                     "[AutoMapSaver] save_map failed: %s", srv_msg.c_str());
    }
}

AutoMapSaver::SkipReason AutoMapSaver::CheckPreconditions() {
    if (!enable_) return SkipReason::kDisabled;
    if (!status_monitor_) return SkipReason::kSnapshotInvalid;

    const auto snap = status_monitor_->GetSnapshot();

    // ① estop — hard safety gate.
    if (snap.is_estop_active) return SkipReason::kEstop;

    // ② low battery.
    if (snap.battery_valid &&
        snap.battery_soc < static_cast<uint16_t>(battery_soc_skip_below_)) {
        return SkipReason::kLowBattery;
    }

    // ③ task not IDLE (string compare; mission_manager publishes raw status strings).
    if (skip_when_task_not_idle_ && snap.task_status_valid) {
        if (snap.task_status != "IDLE") return SkipReason::kTaskNotIdle;
    }

    // ④ robot moving (linear or angular speed above threshold).
    if (snap.velocity_valid) {
        const double v = std::hypot(snap.velocity.linear.x, snap.velocity.linear.y);
        const double w = std::abs(snap.velocity.angular.z);
        if (v > velocity_threshold_mps_ || w > angular_velocity_threshold_rps_) {
            return SkipReason::kRobotMoving;
        }
    }

    // ⑤ no movement since last save (only if we have a baseline).
    if (last_saved_pose_valid_ && snap.pose_valid) {
        const double dx = snap.pose.x - last_saved_pose_.x;
        const double dy = snap.pose.y - last_saved_pose_.y;
        const double dist = std::hypot(dx, dy);
        const double dyaw = std::abs(NormalizeAngle(snap.pose.theta - last_saved_pose_.theta));
        if (dist < pose_change_threshold_m_ && dyaw < yaw_change_threshold_rad_) {
            return SkipReason::kNoMovement;
        }
    }

    return SkipReason::kNone;
}

bool AutoMapSaver::CallSaveMapService(const std::string& map_name, std::string& out_msg,
                                      std::string& out_map_path, std::string& out_backup_ts) {
    if (!save_map_cli_->service_is_ready()) {
        const auto timeout = std::chrono::duration<double>(3.0);
        if (!save_map_cli_->wait_for_service(
                std::chrono::duration_cast<std::chrono::seconds>(timeout))) {
            out_msg = "save_map service not available: " + save_service_;
            return false;
        }
    }
    auto req = std::make_shared<nav_interfaces::srv::SaveMap::Request>();
    req->map_name = map_name;  // 空 = 当前已加载地图
    auto fut = save_map_cli_->async_send_request(req);
    const auto timeout = std::chrono::duration<double>(save_service_timeout_sec_);
    if (fut.wait_for(std::chrono::duration_cast<std::chrono::milliseconds>(timeout))
        != std::future_status::ready) {
        out_msg = "Timeout waiting for save_map (>" +
                  std::to_string(save_service_timeout_sec_) + "s)";
        return false;
    }
    auto resp = fut.get();
    out_msg       = resp->message;
    out_map_path  = resp->map_path;
    out_backup_ts = resp->backup_ts;
    return resp->success;
}

void AutoMapSaver::OnStatusTimer() {
    if (!status_pub_) return;
    std_msgs::msg::String msg;
    std::ostringstream oss;
    oss << "enable=" << (enable_ ? "true" : "false")
        << " daily_target=" << daily_save_hour_ << ":"
        << (daily_save_minute_ < 10 ? "0" : "") << daily_save_minute_
        << " last_fired_ymd=" << last_fired_ymd_
        << " any_attempt=" << (any_attempt_made_ ? "true" : "false")
        << " last_skip=" << SkipReasonToString(last_skip_reason_);
    if (last_saved_pose_valid_) {
        oss << " last_saved_at=" << last_saved_time_.seconds()
            << " last_saved_pose=(" << last_saved_pose_.x
            << "," << last_saved_pose_.y
            << "," << last_saved_pose_.theta << ")";
    }
    msg.data = oss.str();
    status_pub_->publish(msg);
}

const char* AutoMapSaver::SkipReasonToString(SkipReason reason) {
    switch (reason) {
        case SkipReason::kNone:            return "none";
        case SkipReason::kDisabled:        return "disabled";
        case SkipReason::kEstop:           return "estop_active";
        case SkipReason::kLowBattery:      return "low_battery";
        case SkipReason::kTaskNotIdle:     return "task_not_idle";
        case SkipReason::kRobotMoving:     return "robot_moving";
        case SkipReason::kNoMovement:      return "no_movement_since_last_save";
        case SkipReason::kSaveFailed:      return "save_service_failed";
        case SkipReason::kSnapshotInvalid: return "snapshot_invalid";
    }
    return "unknown";
}

}  // namespace command_board
