#include <chrono>
#include <future>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

class ClientA : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    explicit ClientA() : Node("client_a") {
        this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        
        // 延时 1 秒发送，以确保 server 已经启动
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ClientA::send_goal, this)
        );
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void send_goal() {
        this->timer_->cancel(); // 仅执行一次

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action Server 未就绪！");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position.x = 1.0; // Client A 的目标坐标为 1.0

        RCLCPP_INFO(this->get_logger(), "【Client A】正在发送导航目标 (x=1.0)...");

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
            [this](std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "【Client A】目标被 Server 拒绝！");
                } else {
                    std::stringstream ss;
                    for (auto byte : goal_handle->get_goal_id()) ss << std::hex << (int)byte;
                    RCLCPP_INFO(this->get_logger(), "【Client A】目标已被 Server 接受，分配 UUID: %s", ss.str().c_str());
                }
            };

        send_goal_options.result_callback =
            [this](const GoalHandleNavigateToPose::WrappedResult & result) {
                std::stringstream ss;
                for (auto byte : result.goal_id) ss << std::hex << (int)byte;

                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "【Client A】收到结果: 导航成功！UUID: %s", ss.str().c_str());
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_WARN(this->get_logger(), "【Client A】收到结果: 导航被中止 (ABORTED，可能被抢占)！UUID: %s", ss.str().c_str());
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(this->get_logger(), "【Client A】收到结果: 导航被取消 (CANCELED)！UUID: %s", ss.str().c_str());
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "【Client A】收到未知状态码！UUID: %s", ss.str().c_str());
                        break;
                }
            };

        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClientA>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
