#include "map_asset_manager/spatial_indexer.hpp"

#include <algorithm>

#include "map_asset_manager/polygon_utils.hpp"

namespace map_asset_manager {

SpatialIndexer::SpatialIndexer(
    rclcpp::Node*                       node,
    std::shared_ptr<PointStore>         point_store,
    std::shared_ptr<JunctionStore>      junction_store,
    std::shared_ptr<AreaStore>          area_store,
    std::shared_ptr<RoomStore>          room_store,
    std::shared_ptr<DataEventBus>       event_bus,
    std::function<void()>               on_areas_changed,
    std::function<void()>               on_rooms_changed)
    : node_(node),
      point_store_(point_store),
      junction_store_(junction_store),
      area_store_(area_store),
      room_store_(room_store),
      event_bus_(event_bus),
      on_areas_changed_(std::move(on_areas_changed)),
      on_rooms_changed_(std::move(on_rooms_changed))
{
    auto self_point = [this](const AssetEvent& e) { onPointEvent(e); };
    event_bus_->subscribe(AssetEventType::POINT_ADDED,    self_point);
    event_bus_->subscribe(AssetEventType::POINT_UPDATED,  self_point);
    event_bus_->subscribe(AssetEventType::POINT_REMOVED,  self_point);

    auto self_junction = [this](const AssetEvent& e) { onJunctionEvent(e); };
    event_bus_->subscribe(AssetEventType::JUNCTION_ADDED,    self_junction);
    event_bus_->subscribe(AssetEventType::JUNCTION_UPDATED,  self_junction);
    event_bus_->subscribe(AssetEventType::JUNCTION_REMOVED,  self_junction);

    auto self_area = [this](const AssetEvent& e) { onAreaEvent(e); };
    event_bus_->subscribe(AssetEventType::AREA_ADDED,    self_area);
    event_bus_->subscribe(AssetEventType::AREA_UPDATED,  self_area);
    event_bus_->subscribe(AssetEventType::AREA_REMOVED,  self_area);

    auto self_room = [this](const AssetEvent& e) { onRoomEvent(e); };
    event_bus_->subscribe(AssetEventType::ROOM_ADDED,    self_room);
    event_bus_->subscribe(AssetEventType::ROOM_UPDATED,  self_room);
    event_bus_->subscribe(AssetEventType::ROOM_REMOVED,  self_room);

    event_bus_->subscribe(AssetEventType::DATA_RELOADED,
                          [this](const AssetEvent& e) { onDataReloaded(e); });

    RCLCPP_INFO(node_->get_logger(),
                "SpatialIndexer initialized: auto-binding NavPoints + RouteJunctions to Areas + Rooms");
}

// ──────────────────────────────────────────────────────────────────────
// 事件回调
// ──────────────────────────────────────────────────────────────────────

void SpatialIndexer::onPointEvent(const AssetEvent& /*e*/) {
    if (in_rebuild_) return;
    rebuildAll();
}

void SpatialIndexer::onJunctionEvent(const AssetEvent& /*e*/) {
    if (in_rebuild_) return;
    rebuildAll();
}

void SpatialIndexer::onAreaEvent(const AssetEvent& /*e*/) {
    if (in_rebuild_) return;
    in_rebuild_ = true;
    rebuildAllAreas();
    in_rebuild_ = false;
    if (on_areas_changed_) on_areas_changed_();
}

void SpatialIndexer::onRoomEvent(const AssetEvent& /*e*/) {
    if (in_rebuild_) return;
    in_rebuild_ = true;
    rebuildAllRooms();
    in_rebuild_ = false;
    if (on_rooms_changed_) on_rooms_changed_();
}

void SpatialIndexer::onDataReloaded(const AssetEvent& /*e*/) {
    rebuildAll();
}

// ──────────────────────────────────────────────────────────────────────
// 全量重算
// ──────────────────────────────────────────────────────────────────────

void SpatialIndexer::rebuildAll() {
    if (in_rebuild_) return;
    in_rebuild_ = true;
    rebuildAllAreas();
    rebuildAllRooms();
    in_rebuild_ = false;

    if (on_areas_changed_) on_areas_changed_();
    if (on_rooms_changed_) on_rooms_changed_();
}

void SpatialIndexer::rebuildAllAreas() {
    const auto areas = area_store_->getAllAreas();
    const auto points = point_store_->getAllPoints();
    const auto junctions = junction_store_->getAll();

    for (auto area : areas) {
        if (area.polygon.size() < 3) {
            if (!area.related_nav_point_names.empty() || !area.related_junction_ids.empty()) {
                area.related_nav_point_names.clear();
                area.related_junction_ids.clear();
                area_store_->updateArea(area);
            }
            continue;
        }

        std::vector<std::string> nav_names;
        nav_names.reserve(points.size());
        for (const auto& p : points) {
            if (polygon_utils::isPointInPolygon(p.x, p.y, area.polygon)) {
                nav_names.push_back(p.name);
            }
        }
        std::sort(nav_names.begin(), nav_names.end());

        std::vector<std::string> jct_ids;
        jct_ids.reserve(junctions.size());
        for (const auto& j : junctions) {
            if (polygon_utils::isPointInPolygon(j.x, j.y, area.polygon)) {
                jct_ids.push_back(j.id);
            }
        }
        std::sort(jct_ids.begin(), jct_ids.end());

        if (nav_names != area.related_nav_point_names ||
            jct_ids   != area.related_junction_ids) {
            area.related_nav_point_names = std::move(nav_names);
            area.related_junction_ids    = std::move(jct_ids);
            area_store_->updateArea(area);
        }
    }
}

void SpatialIndexer::rebuildAllRooms() {
    const auto rooms = room_store_->getAllRooms();
    const auto points = point_store_->getAllPoints();
    const auto junctions = junction_store_->getAll();

    for (auto room : rooms) {
        if (room.polygon.size() < 3) {
            if (!room.related_nav_point_names.empty() || !room.related_junction_ids.empty()) {
                room.related_nav_point_names.clear();
                room.related_junction_ids.clear();
                room_store_->updateRoom(room);
            }
            continue;
        }

        std::vector<std::string> nav_names;
        nav_names.reserve(points.size());
        for (const auto& p : points) {
            if (polygon_utils::isPointInPolygon(p.x, p.y, room.polygon)) {
                nav_names.push_back(p.name);
            }
        }
        std::sort(nav_names.begin(), nav_names.end());

        std::vector<std::string> jct_ids;
        jct_ids.reserve(junctions.size());
        for (const auto& j : junctions) {
            if (polygon_utils::isPointInPolygon(j.x, j.y, room.polygon)) {
                jct_ids.push_back(j.id);
            }
        }
        std::sort(jct_ids.begin(), jct_ids.end());

        if (nav_names != room.related_nav_point_names ||
            jct_ids   != room.related_junction_ids) {
            room.related_nav_point_names = std::move(nav_names);
            room.related_junction_ids    = std::move(jct_ids);
            room_store_->updateRoom(room);
        }
    }
}

}  // namespace map_asset_manager
