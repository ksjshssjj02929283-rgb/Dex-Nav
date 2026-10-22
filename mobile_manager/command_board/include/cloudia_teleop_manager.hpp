#pragma once

#include <condition_variable>
#include <map>
#include <mutex>
#include <string>
#include <thread>

#include <cloudia_control_interfaces/srv/report_task_submission.hpp>
#include <cloudia_control_interfaces/srv/request_task_submission.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>

#include "cloudia_teleop_state.hpp"

namespace command_board {
class CloudiaTeleopManager {
public:
    explicit CloudiaTeleopManager(rclcpp::Node* node);
    ~CloudiaTeleopManager();
    CloudiaTeleopManager(const CloudiaTeleopManager&) = delete;
    CloudiaTeleopManager& operator=(const CloudiaTeleopManager&) = delete;

    // Called by the existing safety monitor; a missing/stale heartbeat also stops.
    void UpdateSafety(bool fault);
    void RequestStop(const std::string& reason);

private:
    using Acquire = cloudia_control_interfaces::srv::RequestTaskSubmission;
    using Report = cloudia_control_interfaces::srv::ReportTaskSubmission;
    struct Submission {
        std::string coordinator_id;
        uint8_t outcome{Report::Request::UNCERTAIN};
        bool reported{false};
        int64_t task_id{0};
        double created{0.0};
    };

    void OnAcquire(const Acquire::Request::SharedPtr request, Acquire::Response::SharedPtr response);
    void OnReport(const Report::Request::SharedPtr request, Report::Response::SharedPtr response);
    void Tick();
    void Worker();
    nlohmann::json Http(const std::string& method, int port, const std::string& path, bool authenticated);
    CloudiaTeleopState::Remote ReadRemote();
    void StopTask();
    bool PersistLocked();
    void LoadJournal();
    bool SubmissionUncertainLocked(double now) const;
    void PublishZeroLocked();
    static double Now();

    rclcpp::Node* node_;
    CloudiaTeleopState state_;
    std::mutex mutex_;
    std::condition_variable wake_;
    bool shutdown_{false};
    bool urgent_stop_{false};
    bool journal_fault_{false};
    std::thread worker_;
    std::string host_, token_, coordinator_id_, journal_path_;
    int journal_lock_fd_{-1};
    int task_port_{8002}, state_port_{8852};
    double http_timeout_{1.0}, poll_period_{0.2}, submission_timeout_{10.0};
    double linear_still_{0.02}, angular_still_{0.02}, stop_repeat_{0.5};
    std::map<std::string, Submission> pending_;
    nlohmann::json completed_ = nlohmann::json::object();
    int64_t awaiting_task_id_{0};
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::Service<Acquire>::SharedPtr acquire_service_;
    rclcpp::Service<Report>::SharedPtr report_service_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace command_board
