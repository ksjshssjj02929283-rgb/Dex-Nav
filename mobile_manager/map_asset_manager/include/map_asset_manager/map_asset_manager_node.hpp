#ifndef MAP_ASSET_MANAGER__MAP_ASSET_MANAGER_NODE_HPP_
#define MAP_ASSET_MANAGER__MAP_ASSET_MANAGER_NODE_HPP_

#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_interfaces/msg/map_state.hpp>
#include <nav_interfaces/msg/asset_change.hpp>
#include <nav_interfaces/msg/asset_snapshot.hpp>
#include <nav_interfaces/msg/asset_snapshot.hpp>
#include <nav_interfaces/srv/get_assets.hpp>
#include <nav_interfaces/srv/set_assets.hpp>
#include <nav_interfaces/srv/transform_assets.hpp>
#include <nav_interfaces/srv/save_assets.hpp>
#include <nav_interfaces/srv/rollback_assets.hpp>
#include <nav_interfaces/msg/map_version.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "map_asset_manager/area_filter_publisher.hpp"
#include "map_asset_manager/area_manager/area_store.hpp"
#include "map_asset_manager/asset_geometry.hpp"
#include "map_asset_manager/asset_set.hpp"
#include "map_asset_manager/area_manager/area_manager.hpp"
#include "map_asset_manager/data_event_bus.hpp"
#include "map_asset_manager/file_manager.hpp"
#include "map_asset_manager/point_manager/nav_point_manager.hpp"
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
    void publishStatus();          // 发 ~/assets（AssetSnapshot：六类数组 + loaded + error_code，快照即状态，latched）
    void refreshAllOutputs();      // 各管理器 markers/arrays + status（set_assets / loadData 后统一调用）
    nav_interfaces::msg::AssetSnapshot buildSnapshot() const;
    // X-Wiz 批量接口
    void getAssetsCallback(const std::shared_ptr<nav_interfaces::srv::GetAssets::Request> req,
                           std::shared_ptr<nav_interfaces::srv::GetAssets::Response> res);
    void setAssetsCallback(const std::shared_ptr<nav_interfaces::srv::SetAssets::Request> req,
                           std::shared_ptr<nav_interfaces::srv::SetAssets::Response> res);

    // 全局通用服务回调
    void reloadDataCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void saveAssetsCallback(const std::shared_ptr<nav_interfaces::srv::SaveAssets::Request> req,
                            std::shared_ptr<nav_interfaces::srv::SaveAssets::Response> res);
    // 任意地图的资产回滚；command_board 的 rollback_map 资产部分委托这里
    void rollbackAssetsCallback(const std::shared_ptr<nav_interfaces::srv::RollbackAssets::Request> req,
                                std::shared_ptr<nav_interfaces::srv::RollbackAssets::Response> res);
    void transformAssetsCallback(
        const std::shared_ptr<nav_interfaces::srv::TransformAssets::Request> req,
        std::shared_ptr<nav_interfaces::srv::TransformAssets::Response> res);

    // map_manager/map_state 回调：按 epoch 比对触发重载（不依赖名字，根治同名漏检）
    void onMapState(const nav_interfaces::msg::MapState::SharedPtr msg);

    // 【旧机器兼容】旧 map_manager 无 map_state，只有 map_folder / map_name 两个 String topic。
    // folder / name 各自到达一个，齐备且有变化时切图（无 epoch，退化为按名字比对，同名重载检测不到）。

    // folder 和 name 都就绪后执行地图切换
    void switchToMap(const std::string& map_name);

    // 资产文件路径解析：新分层布局 <map>/assets/map_assets.json；
    // 旧机器扁平布局（无 slam/ 子目录）退回 <map>/map_assets.json
    std::string resolveAssetsFile(const std::string& map_name) const;

    // EventBus 订阅
    void registerEventHandlers();

    // ── 按图寻址 ─────────────────────────────────────────────────
    struct Target {
        bool is_current{false};
        std::string map_name;
        std::string map_dir;
        std::string assets_file;
    };
    // map_name 空或等于当前图 → 当前地图；否则校验地图目录存在
    bool resolveTarget(const std::string& map_name, Target& t, int32_t& code, std::string& msg) const;
    AssetSet currentSet() const;
    // 把当前图内存写入资产文件并记下指纹（显式保存、回滚 / 变换前落盘都走这里）
    bool persistCurrent();
    void applyToStores(const AssetSet& set);
    void reloadCurrentFromFile();
    std::shared_ptr<const GridMap> gridFor(const std::string& map_dir);
    std::vector<nav_interfaces::msg::AssetIssue> issuesFor(const std::string& map_dir, const AssetSet& set);
    std::vector<nav_interfaces::msg::MapVersion> versionsFor(const std::string& assets_file) const;
    // 发 ~/asset_changes；revision 为空时现算
    void publishChange(const std::string& map_name, bool is_current, const std::string& op,
                       const std::string& version_ts = "", std::string revision = "");

    // ── 核心共享组件 ─────────────────────────────────────────────
    std::shared_ptr<DataEventBus>       event_bus_;
    std::shared_ptr<FileManager>        file_manager_;
    std::shared_ptr<PointStore>         point_store_;
    std::shared_ptr<RouteStore>         route_store_;
    std::shared_ptr<AreaStore>          area_store_;
    std::shared_ptr<RoomStore>          room_store_;
    std::shared_ptr<JunctionStore>      junction_store_;
    std::shared_ptr<RouteEdgeStore>     edge_store_;

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
    rclcpp::Service<nav_interfaces::srv::GetAssets>::SharedPtr get_assets_srv_;
    rclcpp::Service<nav_interfaces::srv::SetAssets>::SharedPtr set_assets_srv_;
    rclcpp::Service<nav_interfaces::srv::RollbackAssets>::SharedPtr rollback_assets_srv_;
    std::unique_ptr<AreaFilterPublisher> area_filter_;
    rclcpp::Publisher<nav_interfaces::msg::AssetChange>::SharedPtr changes_pub_;
    rclcpp::TimerBase::SharedPtr single_instance_timer_;
    std::string file_revision_;  // 当前图资产文件上的内容指纹（加载 / 本节点写入后更新）
    GridMapCache grid_cache_;
    GeometryParams geometry_params_;
    rclcpp::Publisher<nav_interfaces::msg::AssetSnapshot>::SharedPtr assets_pub_;
    int32_t last_error_code_{0};      // 最近一次 set_assets / save 结果，随 state 发布
    rclcpp::Subscription<nav_interfaces::msg::MapState>::SharedPtr map_state_sub_;

    // ── 状态 ─────────────────────────────────────────────
    std::string map_base_folder_;     // 地图根目录（来自 map_manager/map_state）
    std::string current_map_name_;    // 当前地图名称（来自 map_manager/map_state）
    std::string assets_file_;         // 当前 map_assets.json 完整路径
    uint64_t last_epoch_{0};          // 上次处理的 map_state epoch；比对它判断是否重载
    std::string legacy_folder_;       // 【旧机器兼容】map_manager/map_folder 暂存（等 name 齐备）
    std::string legacy_name_;         // 【旧机器兼容】map_manager/map_name 暂存（等 folder 齐备）
    bool auto_save_{true};
};

}  // namespace map_asset_manager

#endif  // MAP_ASSET_MANAGER__MAP_ASSET_MANAGER_NODE_HPP_
