#pragma once

// 区域 → Nav2 costmap filter 掩膜：禁行区 / 虚拟墙 → keepout 掩膜，减速区 → speed 掩膜。
// 资产节点自己持有区域、订阅 /map 取栅格几何，区域或底图一变就重画并发布（latched），
// 禁行区保存后立即作用于导航。掩膜与 filter info 话题名与导航参数里 keepout_filter / speed_filter 一致。

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <nav2_msgs/msg/costmap_filter_info.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include "map_asset_manager/types.hpp"

namespace map_asset_manager {

class AreaFilterPublisher {
public:
    explicit AreaFilterPublisher(rclcpp::Node* node);

    // 当前地图的区域整体更新（加载 / 编辑 / 回滚后调用）
    void setAreas(const std::vector<Area>& areas);

    struct Stats {
        size_t total{0};
        size_t keepout{0};
        size_t speed{0};
        size_t ignored{0};
        bool published{false};
    };
    Stats lastStats() const { return last_stats_; }

private:
    void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void renderAndPublish();
    void fillPolygonCells(const std::vector<geometry_msgs::msg::Point>& polygon,
                          const nav_msgs::msg::MapMetaData& info, const std::function<void(int, int)>& fill) const;
    bool worldToMap(double wx, double wy, int* mx, int* my) const;
    void setKeepoutCell(int mx, int my);
    void inflateKeepoutCell(int mx, int my, double radius);
    void rasterizeVirtualWall(const std::vector<geometry_msgs::msg::Point>& points);
    int8_t encodeSpeedLimit(float speed_limit) const;
    size_t renderHash() const;

    rclcpp::Node* node_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr keepout_mask_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr speed_mask_pub_;
    rclcpp::Publisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr keepout_info_pub_;
    rclcpp::Publisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr speed_info_pub_;

    nav_msgs::msg::OccupancyGrid map_;
    bool has_map_{false};
    bool has_areas_{false};
    std::vector<Area> areas_;
    nav_msgs::msg::MapMetaData local_info_;  // 地图局部系下的栅格信息（原点 0、无旋转），栅格化用
    nav_msgs::msg::OccupancyGrid keepout_mask_;
    nav_msgs::msg::OccupancyGrid speed_mask_;
    std::vector<float> speed_limits_;
    size_t last_hash_{0};
    bool has_last_hash_{false};
    Stats last_stats_;

    std::string keepout_mask_topic_;
    std::string speed_mask_topic_;
    double speed_base_{0.0};
    double speed_multiplier_{0.01};
    double virtual_wall_width_{0.10};
};

}  // namespace map_asset_manager
