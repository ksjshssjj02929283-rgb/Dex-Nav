#ifndef NAV_POINT_MANAGER__NAV_POINT_MANAGER_NODE_HPP_
#define NAV_POINT_MANAGER__NAV_POINT_MANAGER_NODE_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>

// 使用 nav_interfaces 中的自定义消息
#include "nav_interfaces/action/navigate_route.hpp"
#include "nav_interfaces/action/navigate_to_point.hpp"
#include "nav_interfaces/srv/add_nav_point.hpp"
#include "nav_interfaces/srv/add_route.hpp"
#include "nav_interfaces/srv/get_all_nav_points.hpp"
#include "nav_interfaces/srv/get_nav_point.hpp"
#include "nav_interfaces/srv/get_route.hpp"
#include "nav_interfaces/srv/get_status.hpp"
#include "nav_interfaces/srv/load_point.hpp"
#include "nav_interfaces/srv/remove_nav_point.hpp"
#include "nav_interfaces/srv/remove_route.hpp"
#include "nav_interfaces/srv/update_nav_point.hpp"
#include "nav_point_manager/file_manager.hpp"
#include "nav_point_manager/navigation_executor.hpp"
#include "nav_point_manager/point_database.hpp"
#include "nav_point_manager/types.hpp"

namespace nav_point_manager {

class NavPointManagerNode : public rclcpp::Node {
public:
    using NavigateToPointAction = nav_interfaces::action::NavigateToPoint;
    using NavigateRouteAction = nav_interfaces::action::NavigateRoute;
    using GoalHandleNavigateToPoint = rclcpp_action::ServerGoalHandle<NavigateToPointAction>;
    using GoalHandleNavigateRoute = rclcpp_action::ServerGoalHandle<NavigateRouteAction>;

    NavPointManagerNode();
    ~NavPointManagerNode() = default;

    // 初始化方法，必须在构造后调用
    void initializeActionServers();

private:
    void initialize();
    void loadData();
    void publishMarkers();
    void statusTimerCallback();

    // Service callbacks - 点位管理
    void addPointCallback(const std::shared_ptr<nav_interfaces::srv::AddNavPoint::Request> request,
                          std::shared_ptr<nav_interfaces::srv::AddNavPoint::Response> response);

    void removePointCallback(const std::shared_ptr<nav_interfaces::srv::RemoveNavPoint::Request> request,
                             std::shared_ptr<nav_interfaces::srv::RemoveNavPoint::Response> response);

    void updatePointCallback(const std::shared_ptr<nav_interfaces::srv::UpdateNavPoint::Request> request,
                             std::shared_ptr<nav_interfaces::srv::UpdateNavPoint::Response> response);

    void getPointCallback(const std::shared_ptr<nav_interfaces::srv::GetNavPoint::Request> request,
                          std::shared_ptr<nav_interfaces::srv::GetNavPoint::Response> response);

    void getAllPointsCallback(const std::shared_ptr<nav_interfaces::srv::GetAllNavPoints::Request> request,
                              std::shared_ptr<nav_interfaces::srv::GetAllNavPoints::Response> response);

    // Service callbacks - 路网管理
    void addRouteCallback(const std::shared_ptr<nav_interfaces::srv::AddRoute::Request> request,
                          std::shared_ptr<nav_interfaces::srv::AddRoute::Response> response);

    void removeRouteCallback(const std::shared_ptr<nav_interfaces::srv::RemoveRoute::Request> request,
                             std::shared_ptr<nav_interfaces::srv::RemoveRoute::Response> response);

    void getRouteCallback(const std::shared_ptr<nav_interfaces::srv::GetRoute::Request> request,
                          std::shared_ptr<nav_interfaces::srv::GetRoute::Response> response);

    // Service callbacks - 其他
    void reloadPointsCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void getStatusCallback(const std::shared_ptr<nav_interfaces::srv::GetStatus::Request> request,
                           std::shared_ptr<nav_interfaces::srv::GetStatus::Response> response);

    void loadPointCallback(const std::shared_ptr<nav_interfaces::srv::LoadPoint::Request> request,
                           std::shared_ptr<nav_interfaces::srv::LoadPoint::Response> response);

    // Action callbacks - 导航到点位
    rclcpp_action::GoalResponse handleNavigateToPointGoal(const rclcpp_action::GoalUUID& uuid,
                                                          std::shared_ptr<const NavigateToPointAction::Goal> goal);

    rclcpp_action::CancelResponse handleNavigateToPointCancel(
        const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle);

    void handleNavigateToPointAccepted(const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle);

    void executeNavigateToPoint(const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle);

    // Action callbacks - 路网导航
    rclcpp_action::GoalResponse handleNavigateRouteGoal(const rclcpp_action::GoalUUID& uuid,
                                                        std::shared_ptr<const NavigateRouteAction::Goal> goal);

    rclcpp_action::CancelResponse handleNavigateRouteCancel(const std::shared_ptr<GoalHandleNavigateRoute> goal_handle);

    void handleNavigateRouteAccepted(const std::shared_ptr<GoalHandleNavigateRoute> goal_handle);

    void executeNavigateRoute(const std::shared_ptr<GoalHandleNavigateRoute> goal_handle);

    // 辅助函数
    visualization_msgs::msg::Marker createPointMarker(const NavPoint& point, int id);
    std_msgs::msg::ColorRGBA getColorForType(const std::string& type);
    void savePointsToFile();
    void saveRoutesToFile();
    void publishRoutePath(const std::vector<NavPoint>& route_points);

    // 核心组件
    std::shared_ptr<FileManager> file_manager_;
    std::shared_ptr<PointDatabase> database_;
    std::shared_ptr<NavigationExecutor> nav_executor_;

    // ROS 接口 - Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr route_marker_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr marker_timer_;

    // ROS 接口 - Services (点位管理)
    rclcpp::Service<nav_interfaces::srv::AddNavPoint>::SharedPtr add_point_srv_;
    rclcpp::Service<nav_interfaces::srv::RemoveNavPoint>::SharedPtr remove_point_srv_;
    rclcpp::Service<nav_interfaces::srv::UpdateNavPoint>::SharedPtr update_point_srv_;
    rclcpp::Service<nav_interfaces::srv::GetNavPoint>::SharedPtr get_point_srv_;
    rclcpp::Service<nav_interfaces::srv::GetAllNavPoints>::SharedPtr get_all_points_srv_;

    // ROS 接口 - Services (路网管理)
    rclcpp::Service<nav_interfaces::srv::AddRoute>::SharedPtr add_route_srv_;
    rclcpp::Service<nav_interfaces::srv::RemoveRoute>::SharedPtr remove_route_srv_;
    rclcpp::Service<nav_interfaces::srv::GetRoute>::SharedPtr get_route_srv_;

    // ROS 接口 - Services (其他)
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_points_srv_;
    rclcpp::Service<nav_interfaces::srv::GetStatus>::SharedPtr get_status_srv_;
    rclcpp::Service<nav_interfaces::srv::LoadPoint>::SharedPtr load_point_srv_;

    // ROS 接口 - Actions
    rclcpp_action::Server<NavigateToPointAction>::SharedPtr navigate_to_point_action_;
    rclcpp_action::Server<NavigateRouteAction>::SharedPtr navigate_route_action_;

    // 参数
    std::string data_base_path_;
    std::string default_map_folder_;
    std::string points_file_;
    std::string routes_file_;
    bool auto_save_;
};

}  // namespace nav_point_manager

#endif  // NAV_POINT_MANAGER__NAV_POINT_MANAGER_NODE_HPP_
