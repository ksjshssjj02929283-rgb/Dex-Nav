#ifndef MAP_ASSET_MANAGER__POINT_STORE_HPP_
#define MAP_ASSET_MANAGER__POINT_STORE_HPP_

#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "map_asset_manager/types.hpp"

namespace map_asset_manager {

// 导航点内存数据库（只管 NavPoint，不含 Route）
class PointStore {
public:
    explicit PointStore(rclcpp::Logger logger);

    // addPoint 返回值：0=失败, 1=新增, 2=覆盖
    int addPoint(const NavPoint& point);
    bool removePoint(const std::string& name);
    bool updatePoint(const NavPoint& point);
    std::optional<NavPoint> getPoint(const std::string& name) const;
    std::vector<NavPoint> getAllPoints() const;
    bool hasPoint(const std::string& name) const;

    // 批量操作（热重载时使用）
    void setPoints(const std::unordered_map<std::string, NavPoint>& points);
    std::unordered_map<std::string, NavPoint> getPointsMap() const;

    void clear();

private:
    mutable std::mutex mutex_;
    std::unordered_map<std::string, NavPoint> points_map_;
    rclcpp::Logger logger_;
};

}  // namespace map_asset_manager

#endif  // MAP_ASSET_MANAGER__POINT_STORE_HPP_
