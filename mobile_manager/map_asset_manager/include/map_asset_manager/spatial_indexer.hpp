#ifndef MAP_ASSET_MANAGER__SPATIAL_INDEXER_HPP_
#define MAP_ASSET_MANAGER__SPATIAL_INDEXER_HPP_

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "map_asset_manager/area_manager/area_store.hpp"
#include "map_asset_manager/data_event_bus.hpp"
#include "map_asset_manager/point_manager/point_store.hpp"
#include "map_asset_manager/room_manager/room_store.hpp"
#include "map_asset_manager/route_graph_manager/junction_store.hpp"

namespace map_asset_manager {

// 自动空间归属计算引擎
//
// 职责：监听 POINT/JUNCTION/AREA/ROOM/DATA_RELOADED 事件，自动维护
//   - Area.related_nav_point_names / Area.related_junction_ids
//   - Room.related_nav_point_names / Room.related_junction_ids
//
// 算法：polygon_utils::isPointInPolygon 判断 NavPoint / RouteJunction
// 是否落在每个 Area/Room polygon 内。
//
// 设计权衡：所有事件触发全量重算（30 Room × 30 Area × 200 点 = 12000 次
// point_in_polygon，< 100ms），避免增量逻辑复杂度和漏更新风险。
class SpatialIndexer {
public:
    SpatialIndexer(rclcpp::Node*                       node,
                   std::shared_ptr<PointStore>         point_store,
                   std::shared_ptr<JunctionStore>      junction_store,
                   std::shared_ptr<AreaStore>          area_store,
                   std::shared_ptr<RoomStore>          room_store,
                   std::shared_ptr<DataEventBus>       event_bus,
                   std::function<void()>               on_areas_changed,
                   std::function<void()>               on_rooms_changed);

    // 全量重算（启动时由外部调用，事件触发时内部调用）
    void rebuildAll();

private:
    // 事件回调
    void onPointEvent(const AssetEvent& e);
    void onJunctionEvent(const AssetEvent& e);
    void onAreaEvent(const AssetEvent& e);
    void onRoomEvent(const AssetEvent& e);
    void onDataReloaded(const AssetEvent& e);

    // 核心算法
    void rebuildAllAreas();
    void rebuildAllRooms();

    rclcpp::Node*                  node_;
    std::shared_ptr<PointStore>    point_store_;
    std::shared_ptr<JunctionStore> junction_store_;
    std::shared_ptr<AreaStore>     area_store_;
    std::shared_ptr<RoomStore>     room_store_;
    std::shared_ptr<DataEventBus>  event_bus_;

    // 重算完成后回调（触发快照推送：saveAllAssets → publishStatus）
    std::function<void()>          on_areas_changed_;
    std::function<void()>          on_rooms_changed_;

    // 防递归（事件中再发事件时）
    bool in_rebuild_{false};
};

}  // namespace map_asset_manager

#endif  // MAP_ASSET_MANAGER__SPATIAL_INDEXER_HPP_
