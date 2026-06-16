#ifndef NAVIGATION_ADAPTER__ASSET_NAVIGATION_ADAPTER_HPP_
#define NAVIGATION_ADAPTER__ASSET_NAVIGATION_ADAPTER_HPP_

#include <atomic>
#include <chrono>
#include <future>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <mutex>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_interfaces/action/navigate_route.hpp>
#include <nav_interfaces/action/navigate_to_point.hpp>
#include <nav_interfaces/msg/nav_point.hpp>
#include <nav_interfaces/srv/get_all_nav_points.hpp>
#include <nav_interfaces/srv/get_nav_point.hpp>
#include <nav_interfaces/srv/get_route.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace navigation_adapter {

class AssetNavigationAdapter {
public:
    using NavigateToPoint = nav_interfaces::action::NavigateToPoint;
    using NavigateRoute = nav_interfaces::action::NavigateRoute;
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;

    using GoalHandleNavigateToPoint = rclcpp_action::ServerGoalHandle<NavigateToPoint>;
    using GoalHandleNavigateRoute = rclcpp_action::ServerGoalHandle<NavigateRoute>;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

    explicit AssetNavigationAdapter(rclcpp::Node* node);
    ~AssetNavigationAdapter();

private:
    rclcpp_action::GoalResponse handleNavigateToPointGoal(const rclcpp_action::GoalUUID& uuid,
                                                          std::shared_ptr<const NavigateToPoint::Goal> goal);
    rclcpp_action::CancelResponse handleNavigateToPointCancel(
        const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle);
    void handleNavigateToPointAccepted(const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle);

    rclcpp_action::GoalResponse handleNavigateRouteGoal(const rclcpp_action::GoalUUID& uuid,
                                                        std::shared_ptr<const NavigateRoute::Goal> goal);
    rclcpp_action::CancelResponse handleNavigateRouteCancel(const std::shared_ptr<GoalHandleNavigateRoute> goal_handle);
    void handleNavigateRouteAccepted(const std::shared_ptr<GoalHandleNavigateRoute> goal_handle);
    void startNavigateToPointWorker(const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle);
    void startNavigateRouteWorker(const std::shared_ptr<GoalHandleNavigateRoute> goal_handle);

    void executeNavigateToPoint(const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle);
    void executeNavigateRoute(const std::shared_ptr<GoalHandleNavigateRoute> goal_handle);

    bool getPoint(const std::string& name, nav_interfaces::msg::NavPoint* point, std::string* message);
    bool getAllPointsSnapshot(std::unordered_map<std::string, nav_interfaces::msg::NavPoint>* points,
                              std::string* message);
    bool getRoutePointNames(const std::string& name, std::vector<std::string>* point_names, std::string* message);
    geometry_msgs::msg::PoseStamped toPoseStamped(const nav_interfaces::msg::NavPoint& point) const;
    geometry_msgs::msg::PoseStamped normalizePose(const geometry_msgs::msg::PoseStamped& pose) const;

    bool waitForNav2NavigateToPoseResult(const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle,
                                         const GoalHandleNavigateToPose::SharedPtr& nav2_goal_handle,
                                         std::shared_future<GoalHandleNavigateToPose::WrappedResult> result_future,
                                         std::string* message);
    bool waitForNav2NavigateThroughPosesResult(
        const std::shared_ptr<GoalHandleNavigateRoute> goal_handle,
        const GoalHandleNavigateThroughPoses::SharedPtr& nav2_goal_handle,
        std::shared_future<GoalHandleNavigateThroughPoses::WrappedResult> result_future,
        std::string* message);

    bool tryStartNavigation();
    void finishNavigation();

    rclcpp::Node* node_;

    // ── 资产查询与 Nav2 Action Client ───────────────────────────
    rclcpp::Client<nav_interfaces::srv::GetNavPoint>::SharedPtr get_point_client_;
    rclcpp::Client<nav_interfaces::srv::GetAllNavPoints>::SharedPtr get_all_points_client_;
    rclcpp::Client<nav_interfaces::srv::GetRoute>::SharedPtr get_route_client_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr navigate_through_poses_client_;

    // ── 对外 Action Server ──────────────────────────────────────
    rclcpp_action::Server<NavigateToPoint>::SharedPtr navigate_to_point_server_;
    rclcpp_action::Server<NavigateRoute>::SharedPtr navigate_route_server_;

    // ── 运行状态 ────────────────────────────────────────────────
    std::mutex navigation_mutex_;
    std::mutex worker_threads_mutex_;
    std::vector<std::thread> worker_threads_;
    std::atomic_bool shutting_down_{false};
    bool navigation_active_{false};

    // ── 参数 ────────────────────────────────────────────────────
    std::string get_point_service_;
    std::string get_all_points_service_;
    std::string get_route_service_;
    std::string navigate_to_pose_action_;
    std::string navigate_through_poses_action_;
    std::string navigate_to_point_action_;
    std::string navigate_route_action_;
    std::string global_frame_id_;
    std::chrono::milliseconds service_timeout_{2000};
    std::chrono::milliseconds action_server_timeout_{2000};
};

}  // namespace navigation_adapter

#endif  // NAVIGATION_ADAPTER__ASSET_NAVIGATION_ADAPTER_HPP_
