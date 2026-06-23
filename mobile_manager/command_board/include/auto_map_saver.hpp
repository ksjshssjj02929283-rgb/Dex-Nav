// nav2_command_board / AutoMapSaver (Phase 3.5)
//
// Module ⑤ — Daily map-save scheduler (DEC-006). Polls every
// check_poll_interval_sec; when wall-clock crosses the configured
// daily_save_hour:daily_save_minute AND no save has fired today, evaluates the
// skip conditions in short-circuit order. If all pass, calls the SLAM
// node's save_map service. A single manual service (~/save_map) runs the same
// conditional save on demand: if the state is not savable, it skips (no save).
//
// Configuration: see params/nav2_command_board_params.yaml :: auto_map_saver.

#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_interfaces/srv/save_map.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

namespace command_board {

class StatusMonitor;  // forward decl; injected by Nav2CommandBoardROS

class AutoMapSaver {
public:
    AutoMapSaver(rclcpp::Node* node, StatusMonitor* status_monitor);
    ~AutoMapSaver() = default;

    AutoMapSaver(const AutoMapSaver&) = delete;
    AutoMapSaver& operator=(const AutoMapSaver&) = delete;

private:
    enum class SkipReason {
        kNone,
        kDisabled,
        kEstop,
        kLowBattery,
        kTaskNotIdle,
        kRobotMoving,
        kNoMovement,
        kSaveFailed,
        kSnapshotInvalid,
    };

    void OnTimer();
    void OnSaveMap(const std::shared_ptr<nav_interfaces::srv::SaveMap::Request> req,
                   std::shared_ptr<nav_interfaces::srv::SaveMap::Response> res);
    void OnStatusTimer();

    // Returns kNone if all conditions pass (state is savable).
    SkipReason CheckPreconditions();
    bool       CallSaveMapService(const std::string& map_name, std::string& out_msg,
                                  std::string& out_map_path, std::string& out_backup_ts);
    static const char* SkipReasonToString(SkipReason reason);

    rclcpp::Node*  node_;
    StatusMonitor* status_monitor_;  // non-owning

    // Parameters (DEC-006: switched from hourly to daily trigger).
    bool        enable_ = true;
    int         daily_save_hour_ = 20;          // 0-23, local time
    int         daily_save_minute_ = 0;         // 0-59, local time
    double      check_poll_interval_sec_ = 60.0;  // how often to check the wall clock
    std::string save_service_ = "/localization_node/save_map";
    double      save_service_timeout_sec_ = 30.0;
    double      pose_change_threshold_m_ = 0.10;
    double      yaw_change_threshold_rad_ = 0.05;
    double      velocity_threshold_mps_ = 0.02;
    double      angular_velocity_threshold_rps_ = 0.02;
    int         battery_soc_skip_below_ = 10;
    bool        skip_when_task_not_idle_ = true;

    // Compact YYYYMMDD form (e.g. 20260522); -1 = never fired yet.
    static int CurrentLocalYmd();
    static bool LocalTimeReached(int target_hour, int target_minute);

    // State.
    geometry_msgs::msg::Pose2D last_saved_pose_{};
    bool                       last_saved_pose_valid_ = false;
    rclcpp::Time               last_saved_time_;
    SkipReason                 last_skip_reason_ = SkipReason::kNone;
    bool                       any_attempt_made_ = false;
    int                        last_fired_ymd_ = -1;  // last calendar day a save was attempted

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::Client<nav_interfaces::srv::SaveMap>::SharedPtr save_map_cli_;
    rclcpp::Service<nav_interfaces::srv::SaveMap>::SharedPtr save_srv_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
};

}  // namespace command_board
