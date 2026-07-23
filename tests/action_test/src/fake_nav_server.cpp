#include <functional>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

class FakeNavServer : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

    explicit FakeNavServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("fake_nav_server", options) {
        
        this->action_server_ = rclcpp_action::create_server<NavigateToPose>(
            this,
            "navigate_to_pose",
            std::bind(&FakeNavServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&FakeNavServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&FakeNavServer::handle_accepted, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Fake NavigateToPose Action Server 已启动！");
    }

private:
    rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
    std::shared_ptr<GoalHandleNavigateToPose> current_goal_handle_;
    std::mutex goal_mutex_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigateToPose::Goal> goal) {
        (void)goal;
        std::stringstream ss;
        for (auto byte : uuid) ss << std::hex << (int)byte;
        RCLCPP_INFO(this->get_logger(), "收到新 Goal 请求，UUID: %s", ss.str().c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
        std::stringstream ss;
        for (auto byte : goal_handle->get_goal_id()) ss << std::hex << (int)byte;
        RCLCPP_INFO(this->get_logger(), "收到取消请求，UUID: %s", ss.str().c_str());
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        
        // 模拟 Nav2 的抢占机制：如果当前有正在执行的 Goal，先将其取消 (Preempt)
        if (current_goal_handle_ && current_goal_handle_->is_active()) {
            std::stringstream ss_old;
            for (auto byte : current_goal_handle_->get_goal_id()) ss_old << std::hex << (int)byte;
            RCLCPP_INFO(this->get_logger(), "【抢占检测】存在活跃 Goal %s，正在将其取消...", ss_old.str().c_str());
            
            auto result = std::make_shared<NavigateToPose::Result>();
            current_goal_handle_->abort(result); // 或者 canceled
        }

        current_goal_handle_ = goal_handle;
        std::thread{std::bind(&FakeNavServer::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
        std::stringstream ss;
        for (auto byte : goal_handle->get_goal_id()) ss << std::hex << (int)byte;
        
        RCLCPP_INFO(this->get_logger(), "开始执行 Goal: %s，模拟导航中 (3秒)...", ss.str().c_str());
        
        // 模拟 3 秒钟的导航执行过程，期间每隔 100ms 检查是否被抢占/取消
        for (int i = 0; i < 30; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (goal_handle->is_canceling()) {
                auto result = std::make_shared<NavigateToPose::Result>();
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal: %s 已被客户端主动 Cancel", ss.str().c_str());
                return;
            }
            if (!goal_handle->is_active()) {
                RCLCPP_INFO(this->get_logger(), "Goal: %s 已不再活跃（已被抢占/Abort）", ss.str().c_str());
                return;
            }
        }

        if (rclcpp::ok()) {
            auto result = std::make_shared<NavigateToPose::Result>();
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal: %s 执行成功并已返回 SUCCEEDED 结果！", ss.str().c_str());
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FakeNavServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
