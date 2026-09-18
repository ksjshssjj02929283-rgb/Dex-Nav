#ifndef MAP_ASSET_MANAGER__ROUTE_GRAPH_MANAGER_HPP_
#define MAP_ASSET_MANAGER__ROUTE_GRAPH_MANAGER_HPP_

#include <functional>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <utility>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nav_interfaces/srv/add_route_junction.hpp"
#include "nav_interfaces/srv/remove_route_junction.hpp"
#include "nav_interfaces/srv/add_route_edge.hpp"
#include "nav_interfaces/srv/remove_route_edge.hpp"
#include "nav_interfaces/srv/update_route_edge.hpp"
#include "nav_interfaces/srv/get_route_graph.hpp"
#include "nav_interfaces/msg/path_segment.hpp"
#include "nav_interfaces/srv/resolve_nav_points.hpp"

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

    // nav_point_ids → PathSegment[]：逐对在路网图上求最短路，每条边离散成一段
    void resolveNavPointsCallback(
        const std::shared_ptr<nav_interfaces::srv::ResolveNavPoints::Request> req,
        std::shared_ptr<nav_interfaces::srv::ResolveNavPoints::Response> res);

    // ── 辅助 ──────────────────────────────────────────────────────
    bool endpointExists(const RouteEndpoint& ep) const;
    bool endpointPosition(
        const RouteEndpoint& ep, std::pair<float, float>& position) const;
    void snapEdgeEndpoints(RouteEdge& edge) const;

    /// 端点 → 坐标（nav 查 PointStore，junction 查 JunctionStore）。
    std::optional<std::pair<float, float>> endpointCoord(const RouteEndpoint& ep) const;

    /// 找 a→b 的直接边（相邻 nav 点对必为一条边，不做图搜索）。
    /// 命中 from=a,to=b → reversed=false；命中双向边 from=b,to=a → reversed=true。
    std::optional<RouteEdge> findDirectEdge(
        const RouteEndpoint& a, const RouteEndpoint& b, bool& reversed) const;

    /// 找终点为 target 的边（命中即返回，任意一条）：
    /// 直达 to==target（reversed=false），或双向边 from==target 反向（reversed=true）。
    /// 仅作 TF 不可用时 findNearestEdgeTo() 的退化兜底。
    std::optional<RouteEdge> findEdgeTo(
        const RouteEndpoint& target, bool& reversed) const;

    /// 找终点为 target、且几何上离 robot 最近的边（"参考输入边"）。
    /// 候选与 findEdgeTo 同（直达 to==target，或双向边 from==target 反向），
    /// 但遍历全部候选按点到边折线的最近距离取最小者，避免命中任意一条。
    /// distance 输出车体到该边的最近距离（无候选时为 +inf）。
    std::optional<RouteEdge> findNearestEdgeTo(
        const RouteEndpoint& target, const std::pair<double, double>& robot,
        bool& reversed, double& distance) const;

    /// 当前车体在 map 系的 (x, y)；TF 不可用时返回 nullopt。
    std::optional<std::pair<double, double>> robotXY() const;

    /// 边的几何折线（LINE=两端点，BEZIER=按控制点 Bernstein 采样）。
    /// 只用于距离计算，故与行进方向无关；端点坐标缺失时返回空。
    std::vector<std::pair<double, double>> edgePolyline(const RouteEdge& e) const;

    /// 点到折线的最近距离（逐段取点到线段距离的最小值）；折线为空时返回 +inf。
    static double distanceToPolyline(
        const std::pair<double, double>& p,
        const std::vector<std::pair<double, double>>& poly);

    /// 识别「窄道点位」：目标点 P 是一条单向路径的终点，同时又是一条**倒车**单向路径的起点，
    /// 且那条倒车路径的终点正是进来那条路径的起点 Q。也就是 P 处只有一进一出、且出去靠倒车：
    ///
    ///     Q ──(in : from=Q, to=P, 单向)──▶ P
    ///     Q ◀──(out: from=P, to=Q, 单向, backward=true)── P
    ///
    /// 这样配路网本身就说明 P 附近没有原地掉头的空间，所以车停在 P 上再被要求去 P 时，
    /// 正确的做法是「先沿 out 倒车退到 Q，再沿 in 正向开回 P」，而不是就地转向。
    /// 命中返回 {out, in}（即该按顺序下发的两段）；不命中返回 nullopt。
    ///
    /// 硬性前提：P 关联的边**恰好只有这两条**。多于两条说明 P 不是死胡同端点，
    /// 退出方向不唯一，此时打 WARN 并返回 nullopt，交回常规逻辑。
    std::optional<std::pair<RouteEdge, RouteEdge>> findNarrowTurnaround(
        const RouteEndpoint& p) const;

    /// 单条边 → PathSegment（按行进方向序给 control_points，末点切向作 target_yaw 缺省值；
    /// 请求的最终目标点会在 resolveNavPointsCallback 中改用其自身配置的 yaw 覆盖，见该函数注释）。
    nav_interfaces::msg::PathSegment edgeToSegment(const RouteEdge& e, bool reversed) const;

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
    rclcpp::Service<nav_interfaces::srv::ResolveNavPoints>::SharedPtr    resolve_nav_points_srv_;

    // Publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // TF：ResolveNavPoints 的起始段改造需要当前车体坐标（map → base_link）
    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    /// 起点吸附/归位判定阈值 m：车体离规划起点小于它就把起点挪到车体位置；
    /// 车体离参考输入边小于它就先跑一段参考输入边归位（参数 route_start_tolerance）。
    double start_tolerance_{0.1};

    /// 窄道点位「倒车退出再开回」判定的距离阈值 m（参数 narrow_turnaround_tolerance）：
    /// 只有车体离目标点小于它，才认为车确实停在窄道端点上、需要这一进一出。
    /// 与 start_tolerance_ 分开配：后者是起点吸附的几何容差（cm 级），
    /// 这里要覆盖的是「车停在 P」的实际停靠误差（dm 级），两者量纲不同。
    double narrow_turnaround_tolerance_{0.3};
};

}  // namespace map_asset_manager

#endif  // MAP_ASSET_MANAGER__ROUTE_GRAPH_MANAGER_HPP_
