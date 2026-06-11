#ifndef MAP_ASSET_MANAGER__NAVIGATION_EXECUTOR_HPP_
#define MAP_ASSET_MANAGER__NAVIGATION_EXECUTOR_HPP_

#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <mutex>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <vector>

#include "map_asset_manager/types.hpp"

namespace map_asset_manager {

// 导航执行器 —— 只负责向 Nav2 发送 goal，不持有任何数据库引用
// 坐标解析由调用方（SubManager）完成后以 PoseStamped 传入
class NavigationExecutor {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

    explicit NavigationExecutor(rclcpp::Node::SharedPtr node);

    // 导航到单个姿态
    // label: 用于日志和状态上报的目标名称
    bool navigateToPose(const std::string& label, const geometry_msgs::msg::PoseStamped& pose, double tolerance);

    // 路径导航：调用方已将所有路径点解析为 PoseStamped 列表
    bool navigateThroughPoses(const std::string& label, const std::vector<geometry_msgs::msg::PoseStamped>& poses);

    bool cancelNavigation();

    NavState getCurrentState() const;
    std::string getCurrentTarget() const;

    // 工具函数：外部也可以用
    geometry_msgs::msg::PoseStamped createPoseStamped(double x, double y, double yaw);

private:
    void goalResponseCallback(const GoalHandleNavigateToPose::SharedPtr& goal_handle);
    void feedbackCallback(GoalHandleNavigateToPose::SharedPtr,
                          const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void resultCallback(const GoalHandleNavigateToPose::WrappedResult& result);

    void goalResponseCallbackThroughPoses(const GoalHandleNavigateThroughPoses::SharedPtr& goal_handle);
    void feedbackCallbackThroughPoses(GoalHandleNavigateThroughPoses::SharedPtr,
                                      const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback);
    void resultCallbackThroughPoses(const GoalHandleNavigateThroughPoses::WrappedResult& result);

    bool setNav2GoalTolerance(double tolerance);

    rclcpp::Node::SharedPtr node_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr nav2_through_poses_client_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_param_client_;

    mutable std::mutex state_mutex_;
    NavState current_state_{NavState::IDLE};
    std::string current_target_;
    GoalHandleNavigateToPose::SharedPtr current_goal_handle_;
    GoalHandleNavigateThroughPoses::SharedPtr current_goal_handle_through_poses_;

    std::string controller_server_name_;
    double default_tolerance_{0.05};
};

}  // namespace map_asset_manager

#endif  // MAP_ASSET_MANAGER__NAVIGATION_EXECUTOR_HPP_
