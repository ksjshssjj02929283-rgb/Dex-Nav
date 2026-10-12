#pragma once

// 资产几何校验与失效判定：资产与底图、禁行区的关系。
//   保存时：本次新增 / 修改的点位、路网节点、路网边、区域不合格 → 拒绝保存
//   任何时候：已有的点位、边、路线进入不可通行区 → 保留但列入失效清单（AssetIssue）
// 底图直接读该图的 map.png + map.yaml（当前图与非当前图同一来源），按文件修改时间缓存。

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <nav_interfaces/msg/asset_issue.hpp>

#include "map_asset_manager/asset_set.hpp"

namespace map_asset_manager {

// 原因码（与 AssetIssue.code、SetAssets.rejected 第三段一致）
namespace GeoCode {
constexpr const char* OUT_OF_MAP = "out_of_map";
constexpr const char* IN_UNKNOWN = "in_unknown";
constexpr const char* IN_OBSTACLE = "in_obstacle";
constexpr const char* IN_RESTRICTED = "in_restricted";
constexpr const char* TOO_CLOSE_BOUNDARY = "too_close_boundary";
constexpr const char* TOO_CLOSE_UNKNOWN = "too_close_unknown";
constexpr const char* TOO_CLOSE_OBSTACLE = "too_close_obstacle";
constexpr const char* TOO_CLOSE_RESTRICTED = "too_close_restricted";
constexpr const char* CROSSES_UNKNOWN = "crosses_unknown";
constexpr const char* CROSSES_OBSTACLE = "crosses_obstacle";
constexpr const char* CROSSES_RESTRICTED = "crosses_restricted";
constexpr const char* ENDPOINT_UNAVAILABLE = "endpoint_unavailable";
constexpr const char* MISSING_CONTROL_POINT = "missing_control_point";
constexpr const char* WIDTH_TOO_SMALL = "width_too_small";
constexpr const char* POINT_UNAVAILABLE = "point_unavailable";
constexpr const char* TOO_FEW_VERTICES = "too_few_vertices";
constexpr const char* SELF_INTERSECTING = "self_intersecting";
}  // namespace GeoCode

// 一张底图的三值栅格 + 到障碍 / 未知格边缘的距离（米）。行 0 = 地图 y 最小（与 OccupancyGrid 一致）
struct GridMap {
    enum Cell : uint8_t { FREE = 0, UNKNOWN = 1, OCCUPIED = 2 };
    int width{0};
    int height{0};
    double resolution{0.0};
    double origin_x{0.0};
    double origin_y{0.0};
    double origin_yaw{0.0};
    std::vector<uint8_t> cells;
    std::vector<float> dist_occupied;
    std::vector<float> dist_unknown;

    // 世界坐标 → 栅格；越界返回 false
    bool toCell(double x, double y, int& col, int& row) const;
    // 到地图外边界的距离（米），图外为负
    double distToBoundary(double x, double y) const;
};

// 底图来源：分层布局 <map>/slam/map.yaml，旧扁平布局 <map>/map.yaml
std::string ResolveGridYaml(const std::string& map_dir);
// 读不到（无底图、格式不认识）返回 nullptr，err 给原因
std::shared_ptr<const GridMap> LoadGridMap(const std::string& yaml_path, std::string& err);

// 按 yaml / png 的修改时间与大小缓存，底图被编辑后自动重读
class GridMapCache {
public:
    std::shared_ptr<const GridMap> get(const std::string& map_dir, std::string& err);

private:
    struct Entry {
        std::string stamp;
        std::shared_ptr<const GridMap> grid;
    };
    std::mutex mutex_;
    std::unordered_map<std::string, Entry> entries_;
};

struct GeometryParams {
    double clearance_m{0.35};  // 资产到障碍 / 未知 / 禁行区 / 地图边界的最小距离（机器人半宽）
};

class GeometryChecker {
public:
    // grid 可为空：没有底图时只做禁行区与多边形自身的检查
    GeometryChecker(std::shared_ptr<const GridMap> grid, const AssetSet& set, GeometryParams params);

    // 空串 = 合格
    std::string checkPose(double x, double y) const;
    std::string checkArea(const Area& area) const;
    // 路网边：端点可用性、曲线途经点、宽度、沿线（宽度范围内）可通行
    std::string checkEdge(const RouteEdge& edge, const AssetSet& set) const;

    // 全部点位 / 边 / 路线的失效清单
    std::vector<nav_interfaces::msg::AssetIssue> computeIssues(const AssetSet& set) const;

    bool hasGrid() const { return grid_ != nullptr; }
    double clearance() const { return params_.clearance_m; }

private:
    std::string checkSample(double x, double y, double clearance, bool path) const;
    bool endpointPose(const RouteEndpoint& ep, const AssetSet& set, double& x, double& y) const;

    std::shared_ptr<const GridMap> grid_;
    GeometryParams params_;
    std::vector<std::vector<geometry_msgs::msg::Point>> restricted_;
};

// 路网边的几何折线：直线取两端点；贝塞尔按控制点（首尾即端点）求值，按 step 采样
std::vector<std::pair<double, double>> SampleEdgePath(const RouteEdge& edge, double ax, double ay, double bx,
                                                      double by, double step);

}  // namespace map_asset_manager
