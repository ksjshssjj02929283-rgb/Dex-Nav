#ifndef NAV_POINT_MANAGER__POINT_DATABASE_HPP_
#define NAV_POINT_MANAGER__POINT_DATABASE_HPP_

#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "nav_point_manager/types.hpp"

namespace nav_point_manager {

class PointDatabase {
public:
    explicit PointDatabase(rclcpp::Logger logger);
    ~PointDatabase() = default;

    // 点位操作
    // addPoint 返回值: 0=失败, 1=新增, 2=覆盖
    int addPoint(const NavPoint& point);
    bool removePoint(const std::string& name);
    bool updatePoint(const NavPoint& point);
    std::optional<NavPoint> getPoint(const std::string& name) const;
    std::vector<NavPoint> getAllPoints() const;
    bool hasPoint(const std::string& name) const;

    // 路网操作
    bool addRoute(const Route& route);
    bool removeRoute(const std::string& name);
    std::optional<Route> getRoute(const std::string& name) const;
    std::vector<Route> getAllRoutes() const;
    bool hasRoute(const std::string& name) const;

    // 批量操作
    void setPoints(const std::unordered_map<std::string, NavPoint>& points);
    void setRoutes(const std::unordered_map<std::string, Route>& routes);
    std::unordered_map<std::string, NavPoint> getPointsMap() const;
    std::unordered_map<std::string, Route> getRoutesMap() const;

    // 清空
    void clear();

private:
    mutable std::mutex data_mutex_;
    std::unordered_map<std::string, NavPoint> points_map_;
    std::unordered_map<std::string, Route> routes_map_;
    rclcpp::Logger logger_;
};

}  // namespace nav_point_manager

#endif  // NAV_POINT_MANAGER__POINT_DATABASE_HPP_
