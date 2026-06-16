#ifndef MAP_ASSET_MANAGER__NAV_POINT_MANAGER_HPP_
#define MAP_ASSET_MANAGER__NAV_POINT_MANAGER_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>

#include "map_asset_manager/data_event_bus.hpp"
#include "map_asset_manager/navigation_executor.hpp"
#include "map_asset_manager/point_manager/point_store.hpp"
#include "map_asset_manager/types.hpp"
#include "nav_interfaces/action/navigate_to_point.hpp"
#include "nav_interfaces/srv/add_nav_point.hpp"
#include "nav_interfaces/srv/get_all_nav_points.hpp"
#include "nav_interfaces/srv/get_nav_point.hpp"
#include "nav_interfaces/srv/get_status.hpp"
#include "nav_interfaces/srv/remove_nav_point.hpp"
#include "nav_interfaces/srv/sample_nav_point.hpp"
#include "nav_interfaces/srv/update_nav_point.hpp"

namespace map_asset_manager {

// 导航点子管理器：注册所有 NavPoint 相关的 Service 和 Action
class NavPointManager {
public:
    using NavigateToPointAction = nav_interfaces::action::NavigateToPoint;
    using GoalHandleNavigateToPoint = rclcpp_action::ServerGoalHandle<NavigateToPointAction>;

    // save_fn 由 MapAssetManagerNode 提供，保存全局统一资产文件
    NavPointManager(rclcpp::Node* node,
                    std::shared_ptr<PointStore> point_store,
                    std::shared_ptr<DataEventBus> event_bus,
                    std::function<void()> save_fn);

    // 必须在节点构造完成后、调用方拿到 shared_ptr 时再调用
    void initializeActionServers(rclcpp::Node::SharedPtr node, std::shared_ptr<NavigationExecutor> nav_executor);

    // 供主节点访问：发布点位可视化 Marker
    void publishMarkers();

private:
    // Service 回调
    void addPointCallback(const std::shared_ptr<nav_interfaces::srv::AddNavPoint::Request> req,
                          std::shared_ptr<nav_interfaces::srv::AddNavPoint::Response> res);
    void samplePointCallback(const std::shared_ptr<nav_interfaces::srv::SampleNavPoint::Request> req,
                             std::shared_ptr<nav_interfaces::srv::SampleNavPoint::Response> res);
    void removePointCallback(const std::shared_ptr<nav_interfaces::srv::RemoveNavPoint::Request> req,
                             std::shared_ptr<nav_interfaces::srv::RemoveNavPoint::Response> res);
    void updatePointCallback(const std::shared_ptr<nav_interfaces::srv::UpdateNavPoint::Request> req,
                             std::shared_ptr<nav_interfaces::srv::UpdateNavPoint::Response> res);
    void getPointCallback(const std::shared_ptr<nav_interfaces::srv::GetNavPoint::Request> req,
                          std::shared_ptr<nav_interfaces::srv::GetNavPoint::Response> res);
    void getAllPointsCallback(const std::shared_ptr<nav_interfaces::srv::GetAllNavPoints::Request> req,
                              std::shared_ptr<nav_interfaces::srv::GetAllNavPoints::Response> res);
    void getStatusCallback(const std::shared_ptr<nav_interfaces::srv::GetStatus::Request> req,
                           std::shared_ptr<nav_interfaces::srv::GetStatus::Response> res);
    // Action 回调
    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid,
                                           std::shared_ptr<const NavigateToPointAction::Goal> goal);
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle);
    void handleAccepted(const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle);
    void executeNavigateToPoint(const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle);

    // 辅助
    void saveToFile();
    visualization_msgs::msg::Marker createPointMarker(const NavPoint& point, int id);
    std_msgs::msg::ColorRGBA getColorForType(const std::string& type);
    static std::string pointTypeToString(uint32_t type);

    rclcpp::Node* node_;
    std::shared_ptr<PointStore> point_store_;
    std::shared_ptr<DataEventBus> event_bus_;
    std::shared_ptr<NavigationExecutor> nav_executor_;
    std::function<void()> save_fn_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    // Services
    rclcpp::Service<nav_interfaces::srv::AddNavPoint>::SharedPtr add_point_srv_;
    rclcpp::Service<nav_interfaces::srv::SampleNavPoint>::SharedPtr sample_point_srv_;
    rclcpp::Service<nav_interfaces::srv::RemoveNavPoint>::SharedPtr remove_point_srv_;
    rclcpp::Service<nav_interfaces::srv::UpdateNavPoint>::SharedPtr update_point_srv_;
    rclcpp::Service<nav_interfaces::srv::GetNavPoint>::SharedPtr get_point_srv_;
    rclcpp::Service<nav_interfaces::srv::GetAllNavPoints>::SharedPtr get_all_points_srv_;
    rclcpp::Service<nav_interfaces::srv::GetStatus>::SharedPtr get_status_srv_;

    // Action
    rclcpp_action::Server<NavigateToPointAction>::SharedPtr navigate_to_point_action_;

    // Publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    int prev_marker_count_{0};
};

}  // namespace map_asset_manager

#endif  // MAP_ASSET_MANAGER__NAV_POINT_MANAGER_HPP_
