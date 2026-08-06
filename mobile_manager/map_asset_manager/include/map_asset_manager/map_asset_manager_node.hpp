#ifndef MAP_ASSET_MANAGER__MAP_ASSET_MANAGER_NODE_HPP_
#define MAP_ASSET_MANAGER__MAP_ASSET_MANAGER_NODE_HPP_

#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_interfaces/msg/map_state.hpp>
#include <nav_interfaces/srv/transform_assets.hpp>
#include <nav_interfaces/srv/save_assets.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "map_asset_manager/area_manager/area_store.hpp"
#include "map_asset_manager/area_manager/area_manager.hpp"
#include "map_asset_manager/data_event_bus.hpp"
#include "map_asset_manager/file_manager.hpp"
#include "map_asset_manager/point_manager/nav_point_manager.hpp"
#include "map_asset_manager/navigation_executor.hpp"
#include "map_asset_manager/point_manager/point_store.hpp"
#include "map_asset_manager/route_manager/route_store.hpp"
#include "map_asset_manager/route_manager/route_manager.hpp"
#include "map_asset_manager/room_manager/room_manager.hpp"
#include "map_asset_manager/room_manager/room_store.hpp"
#include "map_asset_manager/route_graph_manager/junction_store.hpp"
#include "map_asset_manager/route_graph_manager/route_edge_store.hpp"
#include "map_asset_manager/route_graph_manager/route_graph_manager.hpp"
#include "map_asset_manager/spatial_indexer.hpp"
#include "map_asset_manager/types.hpp"

namespace map_asset_manager {

class MapAssetManagerNode : public rclcpp::Node {
public:
    MapAssetManagerNode();
    ~MapAssetManagerNode() = default;

    // 必须在构造完成后调用（shared_from_this 依赖）
    void initializeActionServers();

    // 统一保存所有资产到 map_assets.json，供子管理器通过 save_fn 回调
    void saveAllAssets();

private:
    void initialize();
    void loadData();
    void publishStatus();

    // 全局通用服务回调
    void reloadDataCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void saveAssetsCallback(const std::shared_ptr<nav_interfaces::srv::SaveAssets::Request> req,
                            std::shared_ptr<nav_interfaces::srv::SaveAssets::Response> res);
    // 注（DEC-009）：资产回退不在引擎侧提供——回退唯一入口在 command_board（直接从 assets/backups/ 恢复 + 重下发）。
    void transformAssetsCallback(
        const std::shared_ptr<nav_interfaces::srv::TransformAssets::Request> req,
        std::shared_ptr<nav_interfaces::srv::TransformAssets::Response> res);

    // map_manager/map_state 回调：按 epoch 比对触发重载（不依赖名字，根治同名漏检）
    void onMapState(const nav_interfaces::msg::MapState::SharedPtr msg);

    // folder 和 name 都就绪后执行地图切换
    void switchToMap(const std::string& map_name);

    // EventBus 订阅
    void registerEventHandlers();

    // ── 核心共享组件 ─────────────────────────────────────────────
    std::shared_ptr<DataEventBus>       event_bus_;
    std::shared_ptr<FileManager>        file_manager_;
    std::shared_ptr<PointStore>         point_store_;
    std::shared_ptr<RouteStore>         route_store_;
    std::shared_ptr<AreaStore>          area_store_;
    std::shared_ptr<RoomStore>          room_store_;
    std::shared_ptr<JunctionStore>      junction_store_;
    std::shared_ptr<RouteEdgeStore>     edge_store_;
    std::shared_ptr<NavigationExecutor> nav_executor_;

    // ── 子管理器 ─────────────────────────────────────────────────
    std::shared_ptr<NavPointManager>    point_mgr_;
    std::shared_ptr<RouteManager>       route_mgr_;
    std::shared_ptr<AreaManager>        area_mgr_;
    std::shared_ptr<RoomManager>        room_mgr_;
    std::shared_ptr<RouteGraphManager>  route_graph_mgr_;

    // ── 空间归属索引引擎（事件驱动自动维护 Area/Room 的 NavPoint+Junction）──
    std::shared_ptr<SpatialIndexer>     spatial_indexer_;

    // ── ROS 接口 ─────────────────────────────────────────────────
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_data_srv_;
    rclcpp::Service<nav_interfaces::srv::SaveAssets>::SharedPtr save_assets_srv_;
    rclcpp::Service<nav_interfaces::srv::TransformAssets>::SharedPtr transform_assets_srv_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<nav_interfaces::msg::MapState>::SharedPtr map_state_sub_;

    // ── 状态 ─────────────────────────────────────────────
    std::string map_base_folder_;     // 地图根目录（来自 map_manager/map_state）
    std::string current_map_name_;    // 当前地图名称（来自 map_manager/map_state）
    std::string assets_file_;         // 当前 map_assets.json 完整路径
    uint64_t last_epoch_{0};          // 上次处理的 map_state epoch；比对它判断是否重载
    bool auto_save_{true};
};

}  // namespace map_asset_manager

#endif  // MAP_ASSET_MANAGER__MAP_ASSET_MANAGER_NODE_HPP_
