#ifndef NAV_POINT_MANAGER__TYPES_HPP_
#define NAV_POINT_MANAGER__TYPES_HPP_

#include <string>
#include <vector>

namespace nav_point_manager {

// 导航点结构
struct NavPoint {
    std::string name;
    std::string type;  // CHARGING, STATION, PICKUP, PARKING
    double x;
    double y;
    double yaw;
    double tolerance;
};

// 路网结构
struct Route {
    std::string name;
    std::vector<std::string> point_names;
};

// 导航状态枚举
enum class NavState { IDLE, NAVIGATING, SUCCESS, FAILED, CANCELED };

}  // namespace nav_point_manager

#endif  // NAV_POINT_MANAGER__TYPES_HPP_
