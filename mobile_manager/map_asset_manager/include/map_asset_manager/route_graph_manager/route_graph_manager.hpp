#ifndef MAP_ASSET_MANAGER__ROUTE_GRAPH_MANAGER_HPP_
#define MAP_ASSET_MANAGER__ROUTE_GRAPH_MANAGER_HPP_

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nav_interfaces/srv/add_route_junction.hpp"
#include "nav_interfaces/srv/remove_route_junction.hpp"
#include "nav_interfaces/srv/add_route_edge.hpp"
#include "nav_interfaces/srv/remove_route_edge.hpp"
#include "nav_interfaces/srv/update_route_edge.hpp"
#include "nav_interfaces/srv/get_route_graph.hpp"

#include "map_asset_manager/data_event_bus.hpp"
#include "map_asset_manager/point_manager/point_store.hpp"
#include "map_asset_manager/route_graph_manager/junction_store.hpp"
#include "map_asset_manager/route_graph_manager/route_edge_store.hpp"
#include "map_asset_manager/types.hpp"

namespace map_asset_manager {

// 路网拓扑图子管理器
// 管理 Junction（中间节点）和 RouteEdge（路网边）
// 端点校验：type="nav" 查 PointStore；type="junction" 查 JunctionStore
class RouteGraphManager {
public:
    RouteGraphManager(rclcpp::Node* node,
                      std::shared_ptr<JunctionStore>  junction_store,
                      std::shared_ptr<RouteEdgeStore> edge_store,
                      std::shared_ptr<PointStore>     point_store,
                      std::shared_ptr<DataEventBus>   event_bus,
                      std::function<void()>           save_fn);

    // EventBus 响应：NavPoint 被删除时级联清理路网边
    void onNavPointRemoved(const std::string& nav_name);

    void publishMarkers();

private:
    // ── Service 回调 ──────────────────────────────────────────────
    void addJunctionCallback(
        const std::shared_ptr<nav_interfaces::srv::AddRouteJunction::Request> req,
        std::shared_ptr<nav_interfaces::srv::AddRouteJunction::Response> res);

    void removeJunctionCallback(
        const std::shared_ptr<nav_interfaces::srv::RemoveRouteJunction::Request> req,
        std::shared_ptr<nav_interfaces::srv::RemoveRouteJunction::Response> res);

    void addEdgeCallback(
        const std::shared_ptr<nav_interfaces::srv::AddRouteEdge::Request> req,
        std::shared_ptr<nav_interfaces::srv::AddRouteEdge::Response> res);

    void removeEdgeCallback(
        const std::shared_ptr<nav_interfaces::srv::RemoveRouteEdge::Request> req,
        std::shared_ptr<nav_interfaces::srv::RemoveRouteEdge::Response> res);

    void updateEdgeCallback(
        const std::shared_ptr<nav_interfaces::srv::UpdateRouteEdge::Request> req,
        std::shared_ptr<nav_interfaces::srv::UpdateRouteEdge::Response> res);

    void getRouteGraphCallback(
        const std::shared_ptr<nav_interfaces::srv::GetRouteGraph::Request> req,
        std::shared_ptr<nav_interfaces::srv::GetRouteGraph::Response> res);

    // ── 辅助 ──────────────────────────────────────────────────────
    bool endpointExists(const RouteEndpoint& ep) const;
    bool endpointPosition(
        const RouteEndpoint& ep, std::pair<float, float>& position) const;
    void snapEdgeEndpoints(RouteEdge& edge) const;

    // ── 成员 ──────────────────────────────────────────────────────
    rclcpp::Node*                    node_;
    std::shared_ptr<JunctionStore>   junction_store_;
    std::shared_ptr<RouteEdgeStore>  edge_store_;
    std::shared_ptr<PointStore>      point_store_;
    std::shared_ptr<DataEventBus>    event_bus_;
    std::function<void()>            save_fn_;

    // Services
    rclcpp::Service<nav_interfaces::srv::AddRouteJunction>::SharedPtr    add_junction_srv_;
    rclcpp::Service<nav_interfaces::srv::RemoveRouteJunction>::SharedPtr remove_junction_srv_;
    rclcpp::Service<nav_interfaces::srv::AddRouteEdge>::SharedPtr        add_edge_srv_;
    rclcpp::Service<nav_interfaces::srv::RemoveRouteEdge>::SharedPtr     remove_edge_srv_;
    rclcpp::Service<nav_interfaces::srv::UpdateRouteEdge>::SharedPtr     update_edge_srv_;
    rclcpp::Service<nav_interfaces::srv::GetRouteGraph>::SharedPtr       get_graph_srv_;

    // Publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace map_asset_manager

#endif  // MAP_ASSET_MANAGER__ROUTE_GRAPH_MANAGER_HPP_
