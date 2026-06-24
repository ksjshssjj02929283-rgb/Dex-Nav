#include "auto_map_saver.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
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
    declare("auto_map_saver.save_window_start_hour",          save_window_start_hour_);
    declare("auto_map_saver.save_window_start_minute",        save_window_start_minute_);
    declare("auto_map_saver.save_window_end_hour",            save_window_end_hour_);
    declare("auto_map_saver.save_window_end_minute",          save_window_end_minute_);
    declare("auto_map_saver.check_poll_interval_sec",         check_poll_interval_sec_);
    declare("auto_map_saver.state_file_path",                 state_file_path_);
    declare("auto_map_saver.save_service",                    save_service_);
    declare("auto_map_saver.save_service_timeout_sec",        save_service_timeout_sec_);
    declare("auto_map_saver.pose_change_threshold_m",         pose_change_threshold_m_);
    declare("auto_map_saver.yaw_change_threshold_rad",        yaw_change_threshold_rad_);
    declare("auto_map_saver.velocity_threshold_mps",          velocity_threshold_mps_);
    declare("auto_map_saver.angular_velocity_threshold_rps",  angular_velocity_threshold_rps_);
    declare("auto_map_saver.battery_soc_skip_below",          battery_soc_skip_below_);
    declare("auto_map_saver.skip_when_task_not_idle",         skip_when_task_not_idle_);

    // Sanity-clamp window endpoints.
    auto clamp_hm = [](int& h, int& m) {
        if (h < 0 || h > 23) h = 0;
        if (m < 0 || m > 59) m = 0;
    };
    clamp_hm(save_window_start_hour_, save_window_start_minute_);
    clamp_hm(save_window_end_hour_,   save_window_end_minute_);

    // Restore last successful-save day from disk so a restart does not re-save
    // (or miss) today (DEC-016). -1 if never saved / file absent.
    last_saved_ymd_ = LoadPersistedState();

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
                "AutoMapSaver ready. enable=%d window=%02d:%02d~%02d:%02d "
                "poll=%.1fs save_service=%s state_file=%s last_saved_ymd=%d. Service: ~/save_map.",
                enable_, save_window_start_hour_, save_window_start_minute_,
                save_window_end_hour_, save_window_end_minute_,
                check_poll_interval_sec_, save_service_.c_str(),
                state_file_path_.c_str(), last_saved_ymd_);
}

int AutoMapSaver::CurrentLocalYmd() {
    const auto t  = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{};
    localtime_r(&t, &tm);
    return (tm.tm_year + 1900) * 10000 + (tm.tm_mon + 1) * 100 + tm.tm_mday;
}

bool AutoMapSaver::InSaveWindow() const {
    const auto t  = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{};
    localtime_r(&t, &tm);
    const int now_min   = tm.tm_hour * 60 + tm.tm_min;
    const int start_min = save_window_start_hour_ * 60 + save_window_start_minute_;
    const int end_min   = save_window_end_hour_   * 60 + save_window_end_minute_;
    // One-day window only (start <= end); cross-midnight not supported (see 方案 §11.6).
    return now_min >= start_min && now_min <= end_min;
}

std::string AutoMapSaver::ExpandUser(const std::string& path) {
    if (!path.empty() && path[0] == '~') {
        const char* home = std::getenv("HOME");
        if (home) return std::string(home) + path.substr(1);
    }
    return path;
}

int AutoMapSaver::LoadPersistedState() const {
    const std::string path = ExpandUser(state_file_path_);
    std::ifstream ifs(path);
    if (!ifs.is_open()) return -1;  // absent = never saved
    int ymd = -1;
    ifs >> ymd;
    if (!ifs || ymd < 19700101 || ymd > 99991231) {
        RCLCPP_WARN(node_->get_logger(),
                    "[AutoMapSaver] state file '%s' invalid; treating as never-saved.",
                    path.c_str());
        return -1;
    }
    return ymd;
}

void AutoMapSaver::PersistState(int ymd) const {
    const std::string path = ExpandUser(state_file_path_);
    std::ofstream ofs(path, std::ios::trunc);
    if (!ofs.is_open()) {
        RCLCPP_WARN(node_->get_logger(),
                    "[AutoMapSaver] cannot write state file '%s'; in-memory idempotency only "
                    "(may re-save after restart).", path.c_str());
        return;
    }
    ofs << ymd << std::endl;
}

void AutoMapSaver::OnTimer() {
    // DEC-016 gate: time WINDOW + persisted success-day idempotency.
    //   ① skip if today already saved (survives restart via state file)
    //   ② skip if outside [start, end] window
    //   ③ on skip/failure DO NOT mark today → retry on next poll within window
    //   ④ only a successful save marks today + persists to disk
    const int today_ymd = CurrentLocalYmd();
    if (today_ymd == last_saved_ymd_) return;  // already saved today (idempotent)
    if (!InSaveWindow()) return;               // not in the save window yet

    any_attempt_made_ = true;

    const auto reason = CheckPreconditions();
    last_skip_reason_ = reason;
    if (reason != SkipReason::kNone) {
        RCLCPP_INFO(node_->get_logger(),
                    "[AutoMapSaver] Skipped in window (%02d:%02d~%02d:%02d on %d), will retry: reason=%s",
                    save_window_start_hour_, save_window_start_minute_,
                    save_window_end_hour_, save_window_end_minute_, today_ymd,
                    SkipReasonToString(reason));
        return;  // not marked → retried next poll
    }

    std::string srv_msg, map_path, backup_ts;
    if (!CallSaveMapService("", srv_msg, map_path, backup_ts)) {
        last_skip_reason_ = SkipReason::kSaveFailed;
        RCLCPP_ERROR(node_->get_logger(),
                     "[AutoMapSaver] save_map failed in window, will retry: %s", srv_msg.c_str());
        return;  // not marked → retried next poll
    }

    last_saved_ymd_ = today_ymd;     // mark + persist ONLY on success
    PersistState(today_ymd);

    auto snap = status_monitor_->GetSnapshot();
    if (snap.pose_valid) {
        last_saved_pose_ = snap.pose;
        last_saved_pose_valid_ = true;
    }
    last_saved_time_ = node_->now();
    RCLCPP_INFO(node_->get_logger(),
                "[AutoMapSaver] Saved (window on %d): %s; pose=(%.3f, %.3f, %.4f)",
                today_ymd, srv_msg.c_str(),
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
        last_saved_ymd_ = CurrentLocalYmd();  // manual save also satisfies today's idempotency
        PersistState(last_saved_ymd_);
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
        << " window=" << save_window_start_hour_ << ":"
        << (save_window_start_minute_ < 10 ? "0" : "") << save_window_start_minute_
        << "~" << save_window_end_hour_ << ":"
        << (save_window_end_minute_ < 10 ? "0" : "") << save_window_end_minute_
        << " in_window=" << (InSaveWindow() ? "true" : "false")
        << " last_saved_ymd=" << last_saved_ymd_
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
