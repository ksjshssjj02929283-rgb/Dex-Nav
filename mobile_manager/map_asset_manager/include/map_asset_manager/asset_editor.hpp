#pragma once

// set_assets 的处理核心：对一份 AssetSet 应用一次提交。纯函数，不碰节点内存、不读写文件，
// 当前地图（取自各 store）与非当前地图（取自文件）共用，两条路径的模式语义、级联、校验完全一致。

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <nav_interfaces/srv/set_assets.hpp>
#include <rclcpp/time.hpp>

#include "map_asset_manager/asset_geometry.hpp"
#include "map_asset_manager/asset_set.hpp"

namespace map_asset_manager {

struct ApplyOutcome {
    bool ok{false};
    int32_t error_code{0};
    std::string message;
    std::vector<std::string> rejected;  // "<kind>:<name|id>:<reason>"
    int moved_curves{0};                // 随端点移动而整体变换的曲线路网条数
};

// 成功时 set 被替换为应用后的集合（含 related_* 重算）；失败时 set 不变。
// geometry 为空 = 不做几何校验（无底图且无禁行区时等价）
ApplyOutcome ApplySetAssets(const nav_interfaces::srv::SetAssets::Request& req, AssetSet& set, const rclcpp::Time& now,
                            const GeometryParams* geometry, std::shared_ptr<const GridMap> grid);

// 曲线路网随端点移动：以旧起终点 → 新起终点求平移 + 旋转 + 等比缩放，作用于全部控制点（形状不变）
void FollowEndpoints(RouteEdge& edge, double old_ax, double old_ay, double old_bx, double old_by, double new_ax,
                     double new_ay, double new_bx, double new_by);

}  // namespace map_asset_manager
