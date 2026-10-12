#pragma once

// 一张地图的全部资产（六类），与节点内存无关。
// 当前地图与非当前地图共用这一份数据形态：当前地图从各 store 取出 / 写回，非当前地图从文件读 / 写回，
// 编辑、校验、指纹、失效判定都只面向它，两条路径不分叉。

#include <string>
#include <unordered_map>
#include <vector>

#include "map_asset_manager/file_manager.hpp"
#include "map_asset_manager/types.hpp"

namespace map_asset_manager {

struct AssetSet {
    std::unordered_map<std::string, NavPoint> points;
    std::unordered_map<std::string, Route> routes;
    std::unordered_map<std::string, Area> areas;
    std::unordered_map<std::string, Room> rooms;
    std::unordered_map<std::string, RouteJunction> junctions;
    std::unordered_map<std::string, RouteEdge> edges;

    std::vector<RouteJunction> junctionList() const;
    std::vector<RouteEdge> edgeList() const;
};

// 文件不存在 → 空集合且返回 true（新图没有资产是正常情况）
bool LoadAssetSet(FileManager& fm, const std::string& assets_file, AssetSet& out);
// 只在内容真的变了时写盘，并把文件顶层 last_edited_at 记为现在；内容与盘上相同（含「没有文件也没有资产」）
// 则什么都不做、返回 true —— 资产最后编辑时间因此只随编辑推进，加载 / 重复保存不动它
bool SaveAssetSet(FileManager& fm, const std::string& assets_file, const AssetSet& set);

// 内容指纹：按文件格式序列化、数组按名字 / id 排序后取 64 位 FNV-1a。
// 与空白、键序、unordered_map 遍历顺序无关；文件与内存只要内容相同指纹就相同
std::string ComputeAssetsRevision(const AssetSet& set);

// 区域 / 房间的 related_*（内含的点位与路网节点），与 SpatialIndexer 同一规则
void RecomputeRelated(AssetSet& set);

}  // namespace map_asset_manager
