#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "chassis_interfaces/msg/chassis_motor_control.hpp"
#include "chassis_interfaces/msg/chassis_motor_feedback.hpp"
#include "nav_interfaces/action/steering_angle_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// 整车 Twist 由 RobotServer 解算；本节点仅负责静止对舵 Action。
// 所有回调使用默认 mutually-exclusive callback group 和单线程 spin。
class MobileOmniController : public rclcpp::Node {
public:
    using Steering = nav_interfaces::action::SteeringAngleControl;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Steering>;
    using Clock = std::chrono::steady_clock;

    MobileOmniController() : Node("mobile_cmd_omni") {
        motor_pub_ = create_publisher<chassis_interfaces::msg::ChassisMotorControl>(
            "/control/omni_chassis_motors", 10);
        feedback_sub_ = create_subscription<chassis_interfaces::msg::ChassisMotorFeedback>(
            "/feedback/chassis_motor", 1,
            std::bind(&MobileOmniController::onFeedback, this, std::placeholders::_1));
        action_server_ = rclcpp_action::create_server<Steering>(
            this, "/steering_angle_control",
            std::bind(&MobileOmniController::onGoal, this, std::placeholders::_1, std::placeholders::_2),
            [](const std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
            std::bind(&MobileOmniController::onAccepted, this, std::placeholders::_1));
        timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                  std::bind(&MobileOmniController::onTimer, this));
    }

private:
    static constexpr float kPi = 3.14159265358979323846f;
    static constexpr float kSteerLimit = kPi * 0.8f;
    static constexpr double kFeedbackTimeout = 0.1;

    rclcpp_action::GoalResponse onGoal(
        const rclcpp_action::GoalUUID&, std::shared_ptr<const Steering::Goal> goal) {
        if (goal_reserved_ || active_goal_) {
            return rclcpp_action::GoalResponse::REJECT;
        }
        for (const float angle : goal->target_steer_angles_rad) {
            if (!std::isfinite(angle) || std::abs(angle) > kSteerLimit) {
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        if (!std::isfinite(goal->timeout_sec) || !std::isfinite(goal->settle_tolerance_rad)) {
            return rclcpp_action::GoalResponse::REJECT;
        }
        goal_reserved_ = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    void onAccepted(const std::shared_ptr<GoalHandle> goal) {
        active_goal_ = goal;
        goal_reserved_ = false;
        targets_ = goal->get_goal()->target_steer_angles_rad;
        tolerance_ = goal->get_goal()->settle_tolerance_rad > 0.0f
                         ? goal->get_goal()->settle_tolerance_rad : 0.08f;
        timeout_ = goal->get_goal()->timeout_sec > 0.0f ? goal->get_goal()->timeout_sec : 3.0f;
        started_ = Clock::now();
        started_ros_ = now();
        feedback_seq_at_start_ = feedback_seq_;
    }

    void onFeedback(const chassis_interfaces::msg::ChassisMotorFeedback::SharedPtr msg) {
        if (msg->chassis_type != chassis_interfaces::msg::ChassisMotorFeedback::CHASSIS_OMNI ||
            msg->position.size() != targets_.size()) {
            feedback_valid_ = false;
            return;
        }
        for (const double angle : msg->position) {
            if (!std::isfinite(angle)) {
                feedback_valid_ = false;
                return;
            }
        }
        // 避免将队列里的历史反馈当成 Action 接受后的新量测。
        const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
        const double age = (now() - stamp).seconds();
        if (age < 0.0 || age > kFeedbackTimeout || (active_goal_ && stamp < started_ros_)) {
            feedback_valid_ = false;
            return;
        }
        std::copy(msg->position.begin(), msg->position.end(), actual_angles_.begin());
        feedback_received_ = Clock::now();
        feedback_valid_ = true;
        ++feedback_seq_;
        // Action 仅判断舵角；实际 speed_rpm 由 RobotServer 用于行驶解算。
    }

    void finish(bool success, const std::string& message) {
        auto result = std::make_shared<Steering::Result>();
        result->success = success;
        result->message = message;
        result->total_elapsed_time = rclcpp::Duration::from_seconds(
            std::chrono::duration<double>(Clock::now() - started_).count());
        if (active_goal_->is_canceling()) {
            active_goal_->canceled(result);
        } else if (success) {
            active_goal_->succeed(result);
        } else {
            active_goal_->abort(result);
        }
        active_goal_.reset();
        // 不再持续发布：server 在电机指令超时后进入 None，等待新的 Twist 接管。
        // 若继续发布保持指令，会不断刷新 ParkingAction 的控制权，阻止导航恢复。
    }

    void onTimer() {
        if (!active_goal_) return;

        const auto current = Clock::now();
        const double elapsed = std::chrono::duration<double>(current - started_).count();
        const bool fresh_feedback = feedback_valid_ && feedback_seq_ > feedback_seq_at_start_ &&
            std::chrono::duration<double>(current - feedback_received_).count() <= kFeedbackTimeout;
        const bool canceled = active_goal_->is_canceling();
        const bool timed_out = elapsed >= timeout_;

        chassis_interfaces::msg::ChassisMotorControl output;
        output.header.stamp = now();
        output.header.frame_id = "base_link";
        output.position.assign(targets_.begin(), targets_.end());
        output.speed.assign(targets_.size(), 0.0);
        output.estop = 0;
        output.neutral = 0;
        if ((canceled || timed_out) && fresh_feedback) {
            output.position.assign(actual_angles_.begin(), actual_angles_.end());
        }
        motor_pub_->publish(output);

        if (canceled) {
            finish(false, "静止对舵任务已取消");
            return;
        }
        if (timed_out) {
            finish(false, "等待新鲜舵角反馈到位超时");
            return;
        }

        float max_error = fresh_feedback ? 0.0f : std::numeric_limits<float>::infinity();
        if (fresh_feedback) {
            for (std::size_t i = 0; i < targets_.size(); ++i) {
                const float error = std::abs(std::remainder(targets_[i] - actual_angles_[i], 2.0f * kPi));
                max_error = std::max(max_error, error);
            }
        }
        auto feedback = std::make_shared<Steering::Feedback>();
        feedback->max_steer_error = max_error;
        feedback->time_left = rclcpp::Duration::from_seconds(std::max(0.0, timeout_ - elapsed));
        active_goal_->publish_feedback(feedback);
        if (fresh_feedback && max_error <= tolerance_) {
            finish(true, "四轮舵角已到位，停止发布并交还 server 控制权");
        }
    }

    rclcpp::Publisher<chassis_interfaces::msg::ChassisMotorControl>::SharedPtr motor_pub_;
    rclcpp::Subscription<chassis_interfaces::msg::ChassisMotorFeedback>::SharedPtr feedback_sub_;
    rclcpp_action::Server<Steering>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<GoalHandle> active_goal_;
    std::array<float, 4> targets_{};
    std::array<float, 4> actual_angles_{};
    float tolerance_{0.08f};
    double timeout_{3.0};
    bool goal_reserved_{false};
    bool feedback_valid_{false};
    uint64_t feedback_seq_{0};
    uint64_t feedback_seq_at_start_{0};
    Clock::time_point started_{};
    Clock::time_point feedback_received_{};
    rclcpp::Time started_ros_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MobileOmniController>());
    rclcpp::shutdown();
    return 0;
}
