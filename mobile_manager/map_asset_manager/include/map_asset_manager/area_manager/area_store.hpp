#ifndef MAP_ASSET_MANAGER__AREA_STORE_HPP_
#define MAP_ASSET_MANAGER__AREA_STORE_HPP_

#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "map_asset_manager/types.hpp"

namespace map_asset_manager {

// 区域内存数据库（只管 Area）
// TODO: EventBus 响应接口待实现（onPointRemoved / onPointRenamed）
class AreaStore {
public:
    explicit AreaStore(rclcpp::Logger logger);

    bool addArea(const Area& area);
    bool removeArea(const std::string& name);
    bool updateArea(const Area& area);
    std::optional<Area> getArea(const std::string& name) const;
    std::vector<Area> getAllAreas() const;
    bool hasArea(const std::string& name) const;

    // 批量操作
    void setAreas(const std::unordered_map<std::string, Area>& areas);
    std::unordered_map<std::string, Area> getAreasMap() const;

    // EventBus 响应接口（TODO: 实现区域与导航点的联动）
    // std::vector<std::string> onPointRemoved(const std::string& point_name);
    // std::vector<std::string> onPointRenamed(const std::string& old_name, const std::string& new_name);

    void clear();

private:
    mutable std::mutex mutex_;
    std::unordered_map<std::string, Area> areas_map_;
    rclcpp::Logger logger_;
};

}  // namespace map_asset_manager

#endif  // MAP_ASSET_MANAGER__AREA_STORE_HPP_
