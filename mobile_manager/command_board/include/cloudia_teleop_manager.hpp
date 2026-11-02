#pragma once

#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

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

    //安全状态上报
    void UpdateSafety(bool fault);
    //停车信号上报
    void RequestStop(const std::string& reason);

private:
    void Tick();
    void Worker();
    nlohmann::json Http(const std::string& method, int port, const std::string& path, bool authenticated);
    CloudiaTeleopState::Remote ReadRemote();
    void StopTask();
    void PublishZeroLocked();
    static double Now();

    rclcpp::Node* node_;
    CloudiaTeleopState state_;
    std::mutex mutex_;
    std::condition_variable wake_;
    bool shutdown_{false};
    bool urgent_stop_{false};
    std::thread worker_;
    std::string host_, token_;
    int task_port_{8002}, state_port_{8852};
    double http_timeout_{1.0}, poll_period_{0.2};
    double linear_still_{0.02}, angular_still_{0.02}, stop_repeat_{0.5};
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace command_board
