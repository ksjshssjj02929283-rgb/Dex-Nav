#ifndef MAP_ASSET_MANAGER__TYPES_HPP_
#define MAP_ASSET_MANAGER__TYPES_HPP_

#include <cstdint>
#include <geometry_msgs/msg/point.hpp>
#include <string>
#include <vector>

namespace map_asset_manager {

// 导航点
struct NavPoint {
    std::string name;
    std::string type;  // CHARGING, STATION, DOORWAIT, PARKING， LIFT 等
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double tolerance{0.0};
};

// 路径（按点位名称序列引用导航点）
struct Route {
    std::string name;
    std::vector<std::string> point_names;
};

// 区域类型常量（与 nav_interfaces/AreaInfo.msg 保持一致）
namespace AreaType {
constexpr uint32_t VIRTUAL_WALL = 1;        // 虚拟墙
constexpr uint32_t RESTRICTED_AREA = 2;     // 禁行区
constexpr uint32_t SPEED_LIMIT_AREA = 3;    // 减速区
constexpr uint32_t DETOUR_AREA = 4;         // 绕障区
constexpr uint32_t STOP_OBSTACLE_AREA = 5;  // 停障区
constexpr uint32_t PARKING_AREA = 6;        // 停车区
constexpr uint32_t CHARGING_AREA = 7;       // 充电区
constexpr uint32_t ELEVATOR_AREA = 8;       // 电梯区
constexpr uint32_t BUMPY_AREA = 9;          // 颠簸区（降速 + 关闭深度相机）

inline std::string toString(uint32_t t) {
    switch (t) {
        case VIRTUAL_WALL:
            return "VIRTUAL_WALL";
        case RESTRICTED_AREA:
            return "RESTRICTED_AREA";
        case SPEED_LIMIT_AREA:
            return "SPEED_LIMIT_AREA";
        case DETOUR_AREA:
            return "DETOUR_AREA";
        case STOP_OBSTACLE_AREA:
            return "STOP_OBSTACLE_AREA";
        case PARKING_AREA:
            return "PARKING_AREA";
        case CHARGING_AREA:
            return "CHARGING_AREA";
        case ELEVATOR_AREA:
            return "ELEVATOR_AREA";
        case BUMPY_AREA:
            return "BUMPY_AREA";
        default:
            return "UNKNOWN";
    }
}

inline uint32_t fromString(const std::string& s) {
    if (s == "VIRTUAL_WALL") return VIRTUAL_WALL;
    if (s == "RESTRICTED_AREA") return RESTRICTED_AREA;
    if (s == "SPEED_LIMIT_AREA") return SPEED_LIMIT_AREA;
    if (s == "DETOUR_AREA") return DETOUR_AREA;
    if (s == "STOP_OBSTACLE_AREA") return STOP_OBSTACLE_AREA;
    if (s == "PARKING_AREA") return PARKING_AREA;
    if (s == "CHARGING_AREA") return CHARGING_AREA;
    if (s == "ELEVATOR_AREA") return ELEVATOR_AREA;
    if (s == "BUMPY_AREA") return BUMPY_AREA;
    return 0;
}
}  // namespace AreaType

// 区域（多边形，地图坐标系）
struct Area {
    std::string name;
    uint32_t type{0};  // AreaType 常量

    std::vector<geometry_msgs::msg::Point> polygon;  // 多边形顶点（x, y）

    // SPEED_LIMIT_AREA
    float max_speed{0.0f};  // 最大速度 (m/s)；0 = 系统默认

    // STOP_OBSTACLE_AREA
    float stop_distance{0.0f};  // 停止距离 (m)；0 = 系统默认

    // PARKING_AREA / CHARGING_AREA
    std::string linked_point_name;  // 关联导航点名称

    // CHARGING_AREA
    std::string charger_type;  // WIRELESS / CONTACT

    // BUMPY_AREA
    float depth_min_height{0.1f};  // 深度相机最小有效检测高度 (m)

    // 通用扩展（向后兼容）
    std::string attribute;
};

// 导航状态
enum class NavState { IDLE, NAVIGATING, SUCCESS, FAILED, CANCELED };

// ── RouteGraph 数据结构 ──────────────────────────────────────────────

// 路网中间拓扑节点（RouteGraphManager 私有，不进入 NavPointManager）
struct RouteJunction {
    std::string id;
    float x{0.0f};
    float y{0.0f};
    float yaw{0.0f};
};

// 路网边端点（可引用 NavPoint 或 RouteJunction）
struct RouteEndpoint {
    std::string id;    // NavPoint.name 或 RouteJunction.id
    std::string type;  // "nav" | "junction"
};

// 路网边
struct RouteEdge {
    std::string id;
    RouteEndpoint from;
    RouteEndpoint to;
    std::string edge_type;  // "LINE" | "BEZIER"
    bool bidirectional{false};
    float max_speed{0.0f};
    float xy_tolerance{0.3f};
    float th_tolerance{0.5f};
    float lane_width{0.0f};
    std::string obs_policy{"STOP"};  // "STOP" | "DETOUR"
    std::vector<std::pair<float, float>> control_points;
};

}  // namespace map_asset_manager

#endif  // MAP_ASSET_MANAGER__TYPES_HPP_
