#ifndef NAV_POINT_MANAGER__NAVIGATION_EXECUTOR_HPP_
#define NAV_POINT_MANAGER__NAVIGATION_EXECUTOR_HPP_

#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <mutex>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>

#include "nav_point_manager/point_database.hpp"
#include "nav_point_manager/types.hpp"

namespace nav_point_manager {

class NavigationExecutor {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

    NavigationExecutor(rclcpp::Node::SharedPtr node, std::shared_ptr<PointDatabase> database);
    ~NavigationExecutor() = default;

    // 导航到指定点位
    bool navigateToPoint(const std::string& point_name);

    // 导航到指定姿态
    bool navigateToPose(const geometry_msgs::msg::PoseStamped& pose, double tolerance);

    // 路网导航
    bool navigateRoute(const std::string& route_name);

    // 取消导航
    bool cancelNavigation();

    // 获取当前状态
    NavState getCurrentState() const;
    std::string getCurrentTarget() const;

private:
    // Nav2 回调
    void goalResponseCallback(const GoalHandleNavigateToPose::SharedPtr& goal_handle);
    void feedbackCallback(GoalHandleNavigateToPose::SharedPtr,
                          const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void resultCallback(const GoalHandleNavigateToPose::WrappedResult& result);

    void goalResponseCallbackThroughPoses(const GoalHandleNavigateThroughPoses::SharedPtr& goal_handle);
    void feedbackCallbackThroughPoses(GoalHandleNavigateThroughPoses::SharedPtr,
                                      const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback);
    void resultCallbackThroughPoses(const GoalHandleNavigateThroughPoses::WrappedResult& result);

    // 辅助函数
    geometry_msgs::msg::PoseStamped createPoseStamped(double x, double y, double yaw);
    bool setNav2GoalTolerance(double tolerance);

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<PointDatabase> database_;

    // Action clients
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr nav2_through_poses_client_;

    // Service client for setting tolerance
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_param_client_;

    // 状态管理
    mutable std::mutex state_mutex_;
    NavState current_state_;
    std::string current_target_;
    GoalHandleNavigateToPose::SharedPtr current_goal_handle_;
    GoalHandleNavigateThroughPoses::SharedPtr current_goal_handle_through_poses_;

    // 参数
    std::string controller_server_name_;
    double default_tolerance_;
};

}  // namespace nav_point_manager

#endif  // NAV_POINT_MANAGER__NAVIGATION_EXECUTOR_HPP_
