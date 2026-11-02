#pragma once

// 资产坐标变换核心：地图原点改变（重设原点 / 对齐旋转）后，把资产换算到新坐标系。
// 纯文件操作、不依赖任何节点，供离线工具 transform_assets_tool 与 map_asset_manager 的
// transform_assets 服务共用，保证两条入口换算结果一致。

#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/logger.hpp>

#include "map_asset_manager/asset_versions.hpp"
#include "map_asset_manager/file_manager.hpp"
#include "map_asset_manager/types.hpp"

namespace map_asset_manager {

// 新原点在旧坐标系下的位姿：新坐标 = 旧坐标在 (x, y, yaw) 处坐标系下的表示
struct OriginChange {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

struct AssetTransformStats {
    int points{0};
    int areas{0};
    int junctions{0};
    int edges{0};   // 带控制点或平移朝向、实际被改动的边
    int rooms{0};
};

// 原地变换全部带坐标 / 朝向的资产：
//   导航点 xy + yaw；区域多边形；路网节点 xy + yaw；边控制点 + 平移朝向 hold_direction（地图系朝向）；
//   房间多边形 + 位姿（位置与朝向）+ 外包框重算。路线只引用点名，无坐标。
void TransformAssetsInPlace(const OriginChange& t,
                            std::unordered_map<std::string, NavPoint>& points,
                            std::unordered_map<std::string, Area>& areas,
                            std::vector<RouteJunction>& junctions,
                            std::vector<RouteEdge>& edges,
                            std::unordered_map<std::string, Room>& rooms,
                            AssetTransformStats* stats = nullptr);

// 资产文件位置：分层布局 <map>/assets/map_assets.json；
// 旧扁平布局（无 slam/ 子目录且根下有 map_assets.json 或 map.yaml）为 <map>/map_assets.json
std::string ResolveAssetsFile(const std::string& map_dir);

struct AssetFileTransformResult {
    bool ok{false};
    bool no_assets{false};      // 资产文件不存在：无资产可转，视为成功
    std::string message;
    std::string assets_file;
    std::string version_ts;     // 变换前存下的资产版本
    AssetTransformStats stats;
};

// 文件级变换：存版本 → 读 → 变换 → 原子写回。不触碰任何节点内存。
AssetFileTransformResult TransformAssetsFile(const std::string& assets_file, const OriginChange& t,
                                             FileManager& file_manager, const rclcpp::Logger& logger);

}  // namespace map_asset_manager
