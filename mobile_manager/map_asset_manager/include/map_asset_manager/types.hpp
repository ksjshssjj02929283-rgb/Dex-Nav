#ifndef MAP_ASSET_MANAGER__TYPES_HPP_
#define MAP_ASSET_MANAGER__TYPES_HPP_

#include <array>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <cstdint>
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
    constexpr uint32_t VIRTUAL_WALL       = 1;  // 虚拟墙
    constexpr uint32_t RESTRICTED_AREA    = 2;  // 禁行区
    constexpr uint32_t SPEED_LIMIT_AREA   = 3;  // 减速区
    constexpr uint32_t DETOUR_AREA        = 4;  // 绕障区
    constexpr uint32_t STOP_OBSTACLE_AREA = 5;  // 停障区
    constexpr uint32_t PARKING_AREA       = 6;  // 停车区
    constexpr uint32_t CHARGING_AREA      = 7;  // 充电区
    constexpr uint32_t ELEVATOR_AREA      = 8;  // 电梯区
    constexpr uint32_t BUMPY_AREA         = 9;  // 颠簸区（降速 + 关闭深度相机）

    inline std::string toString(uint32_t t) {
        switch (t) {
            case VIRTUAL_WALL:       return "VIRTUAL_WALL";
            case RESTRICTED_AREA:    return "RESTRICTED_AREA";
            case SPEED_LIMIT_AREA:   return "SPEED_LIMIT_AREA";
            case DETOUR_AREA:        return "DETOUR_AREA";
            case STOP_OBSTACLE_AREA: return "STOP_OBSTACLE_AREA";
            case PARKING_AREA:       return "PARKING_AREA";
            case CHARGING_AREA:      return "CHARGING_AREA";
            case ELEVATOR_AREA:      return "ELEVATOR_AREA";
            case BUMPY_AREA:         return "BUMPY_AREA";
            default:                 return "UNKNOWN";
        }
    }

    inline uint32_t fromString(const std::string& s) {
        if (s == "VIRTUAL_WALL")       return VIRTUAL_WALL;
        if (s == "RESTRICTED_AREA")    return RESTRICTED_AREA;
        if (s == "SPEED_LIMIT_AREA")   return SPEED_LIMIT_AREA;
        if (s == "DETOUR_AREA")        return DETOUR_AREA;
        if (s == "STOP_OBSTACLE_AREA") return STOP_OBSTACLE_AREA;
        if (s == "PARKING_AREA")       return PARKING_AREA;
        if (s == "CHARGING_AREA")      return CHARGING_AREA;
        if (s == "ELEVATOR_AREA")      return ELEVATOR_AREA;
        if (s == "BUMPY_AREA")         return BUMPY_AREA;
        return 0;
    }
}  // namespace AreaType

// 区域（多边形，地图坐标系）
struct Area {
    std::string name;
    uint32_t type{0};                             // AreaType 常量

    std::vector<geometry_msgs::msg::Point> polygon;  // 多边形顶点（x, y）

    // SPEED_LIMIT_AREA
    float max_speed{0.0f};         // 最大速度 (m/s)；0 = 系统默认

    // STOP_OBSTACLE_AREA
    float stop_distance{0.0f};     // 停止距离 (m)；0 = 系统默认

    // PARKING_AREA / CHARGING_AREA
    std::vector<std::string> manual_linked_point_names;  // 手工指定的关联导航点（支持多个）

    // CHARGING_AREA
    std::string charger_type;      // WIRELESS / CONTACT

    // BUMPY_AREA
    float depth_min_height{0.1f};       // 深度相机最小有效检测高度 (m)

    // 通用扩展（向后兼容）
    std::string attribute;

    // ── 自动空间归属（SpatialIndexer 维护，业务侧只读，持久化到 JSON）────
    // 由 polygon 内点检测自动计算
    std::vector<std::string> related_nav_point_names;  // polygon 内的 NavPoint 名
    std::vector<std::string> related_junction_ids;     // polygon 内的 RouteJunction id
};

// 导航状态
enum class NavState { IDLE, NAVIGATING, SUCCESS, FAILED, CANCELED };

// ── DSG 节点来源（用于双源融合标识）─────────────────────────────────
namespace DSGSource {
    constexpr const char* MANUAL        = "manual";
    constexpr const char* HYDRA         = "hydra";
    constexpr const char* HOV_SG        = "hov_sg";
    constexpr const char* CLIO          = "clio";
    constexpr const char* USER_OVERRIDE = "user_override";
}  // namespace DSGSource

// ── DSG 层级（spark_dsg 对齐）──────────────────────────────────────
namespace DSGLayer {
    constexpr uint32_t OBJECTS   = 1;
    constexpr uint32_t PLACES    = 2;
    constexpr uint32_t ROOMS     = 3;
    constexpr uint32_t FLOORS    = 4;
    constexpr uint32_t BUILDINGS = 5;
}  // namespace DSGLayer

// 房间类型常量（与 nav_interfaces/RoomInfo.msg 保持一致）
namespace RoomType {
    constexpr uint32_t UNKNOWN       = 0;
    constexpr uint32_t MEETING_ROOM  = 1;  // 会议室
    constexpr uint32_t PANTRY        = 2;  // 茶水间
    constexpr uint32_t CAFE          = 3;  // 咖啡厅
    constexpr uint32_t CORRIDOR      = 4;  // 走廊
    constexpr uint32_t OFFICE        = 5;  // 办公室
    constexpr uint32_t LOBBY         = 6;  // 大堂
    constexpr uint32_t RESTROOM      = 7;  // 卫生间
    constexpr uint32_t STORAGE       = 8;  // 储藏室

    inline std::string toString(uint32_t t) {
        switch (t) {
            case MEETING_ROOM: return "MEETING_ROOM";
            case PANTRY:       return "PANTRY";
            case CAFE:         return "CAFE";
            case CORRIDOR:     return "CORRIDOR";
            case OFFICE:       return "OFFICE";
            case LOBBY:        return "LOBBY";
            case RESTROOM:     return "RESTROOM";
            case STORAGE:      return "STORAGE";
            default:           return "UNKNOWN";
        }
    }

    inline uint32_t fromString(const std::string& s) {
        if (s == "MEETING_ROOM") return MEETING_ROOM;
        if (s == "PANTRY")       return PANTRY;
        if (s == "CAFE")         return CAFE;
        if (s == "CORRIDOR")     return CORRIDOR;
        if (s == "OFFICE")       return OFFICE;
        if (s == "LOBBY")        return LOBBY;
        if (s == "RESTROOM")     return RESTROOM;
        if (s == "STORAGE")      return STORAGE;
        return UNKNOWN;
    }
}  // namespace RoomType

// Room 的可选行为约束（替代独立 Area 在房间内的约束作用）
// 避免"会议室限速"场景被迫维护双 polygon（Room + 同位置 Area）
struct RoomBehavior {
    // 速度限制（对应 Area SPEED_LIMIT_AREA）
    float max_speed_m_s{0.0f};        // 0 = 不限速；> 0 = 业务侧自动降速

    // 通行约束（对应 Area RESTRICTED_AREA / DETOUR_AREA）
    bool  restricted{false};          // 禁入
    bool  detour_enabled{false};      // 遇障绕行

    // 障碍策略（对应 Area STOP_OBSTACLE_AREA）
    float stop_distance_m{0.0f};      // 遇障停止距离 (m)

    // 颠簸 / 低矮障碍（对应 Area BUMPY_AREA）
    float depth_min_height_m{0.0f};   // 深度相机最小有效检测高度

    // 关联额外 Area（如有跨房间或门口减速带等补充约束）
    std::vector<std::string> additional_area_names;
};

// 房间（DSG ROOMS 层节点的 Phase 1 实现）
struct Room {
    // ── DSG 节点标识（room_manager 自动填充）─────────────────
    std::string id;                  // "room:<name>:<source>"
    std::string source;              // DSGSource 常量
    int64_t     created_at_ns{0};    // 纳秒 epoch（rclcpp::Time 兼容）
    int64_t     updated_at_ns{0};

    // ── 基本信息 ────────────────────────────────────────────
    std::string name;
    uint32_t    type{RoomType::UNKNOWN};
    std::string semantic_label;      // 开放词汇（VLN/Clio 兼容）
    std::string display_label;

    // ── 空间几何 ────────────────────────────────────────────
    geometry_msgs::msg::Pose pose;                       // 3D 位姿（从 polygon 推导默认值）
    std::vector<geometry_msgs::msg::Point> polygon;      // 2D 边界（map 坐标系）
    std::array<double, 6> bounding_box{0,0,0,0,0,0};     // AABB

    // ── 行为约束（可选，替代独立 Area 在房间内的约束作用）──
    RoomBehavior behavior;

    // ── 关联（手工指定）─────────────────────────────────────
    std::vector<std::string> manual_anchor_point_names;  // 手工指定的关联 NavPoint（多入口/多工位/多锚点）

    // ── 扩展 ────────────────────────────────────────────────
    std::string attribute;           // JSON 扩展
    std::string external_dsg_node_id;  // 跨 DSG 系统关联 ID

    // ── 自动空间归属（SpatialIndexer 维护，业务侧只读，持久化到 JSON）────
    // 由 polygon 内点检测自动计算
    std::vector<std::string> related_nav_point_names;
    std::vector<std::string> related_junction_ids;
};

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
    std::string   id;
    RouteEndpoint from;
    RouteEndpoint to;
    std::string   edge_type;      // "LINE" | "BEZIER"
    bool          bidirectional{false};
    float         max_speed{0.0f};
    float         xy_tolerance{0.3f};
    float         th_tolerance{0.5f};
    float         lane_width{0.0f};
    std::string   obs_policy{"STOP"};  // "STOP" | "DETOUR"
    std::vector<std::pair<float, float>> control_points;
};

}  // namespace map_asset_manager

#endif  // MAP_ASSET_MANAGER__TYPES_HPP_
