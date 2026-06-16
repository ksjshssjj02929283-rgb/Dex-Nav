#ifndef NAVIGATION_ADAPTER__AREA_FILTER_ADAPTER_HPP_
#define NAVIGATION_ADAPTER__AREA_FILTER_ADAPTER_HPP_

#include <array>
#include <cstddef>
#include <functional>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav2_msgs/msg/costmap_filter_info.hpp>
#include <nav_interfaces/msg/area_info.hpp>
#include <nav_interfaces/msg/area_info_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

namespace navigation_adapter {

class AreaFilterAdapter {
public:
    explicit AreaFilterAdapter(rclcpp::Node* node);

private:
    enum class LifecycleState {
        WAITING_MAP,
        WAITING_AREAS,
        STALE,
        ACTIVE,
    };

    // AreaInfo.type 常量目前覆盖 1..9，预留 0 给未知值，预留若干余量给未来扩展。
    static constexpr size_t kIgnoredByTypeSize = 16U;

    struct RenderStats {
        size_t total_areas{0};
        size_t keepout_areas{0};
        size_t speed_areas{0};
        size_t ignored_areas{0};
        // ignored_by_type[t] 记录 AreaInfo.type==t 的区域未成功转换为 mask 的数量。
        // index 0 用于上游新增类型时的兜底（落入 default 分支）。
        std::array<size_t, kIgnoredByTypeSize> ignored_by_type{};
    };

    void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void onAreas(const nav_interfaces::msg::AreaInfoArray::SharedPtr msg);
    void onMapName(const std_msgs::msg::String::SharedPtr msg);
    void onStatusTimer();

    void renderAndPublish();
    void publishFilterInfo();
    bool readyToRender() const;

    void clearMasksFromMap(const nav_msgs::msg::OccupancyGrid& map);
    RenderStats renderAreas();
    bool renderKeepoutArea(const nav_interfaces::msg::AreaInfo& area);
    bool renderSpeedArea(const nav_interfaces::msg::AreaInfo& area);
    bool rasterizeRestrictedPolygon(const std::vector<geometry_msgs::msg::Point>& polygon);
    bool rasterizeVirtualWall(const std::vector<geometry_msgs::msg::Point>& points);
    bool rasterizeSpeedPolygon(const std::vector<geometry_msgs::msg::Point>& polygon, float speed_limit);

    bool computeBounds(const std::vector<geometry_msgs::msg::Point>& points,
                       double* min_x,
                       double* min_y,
                       double* max_x,
                       double* max_y) const;
    int8_t encodeSpeedLimit(float speed_limit) const;
    bool worldToMap(double wx, double wy, int* mx, int* my) const;
    void setKeepoutCell(int mx, int my);
    void setSpeedCell(int mx, int my, float speed_limit, int8_t encoded);
    void fillPolygonCells(const std::vector<geometry_msgs::msg::Point>& polygon,
                          const nav_msgs::msg::MapMetaData& info,
                          const std::function<void(int, int)>& fill_cell) const;
    void inflateKeepoutCell(int center_mx, int center_my, double radius);
    std::vector<geometry_msgs::msg::Point> toPoints(const std::vector<geometry_msgs::msg::Pose2D>& polygon) const;
    size_t buildRenderHash() const;
    std::string stateToString() const;
    std::string formatIgnoredBreakdown(const RenderStats& stats) const;

    rclcpp::Node* node_;

    // ── ROS 接口 ─────────────────────────────────────────────────
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_interfaces::msg::AreaInfoArray>::SharedPtr areas_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_name_sub_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr keepout_mask_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr speed_mask_pub_;
    rclcpp::Publisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr keepout_filter_info_pub_;
    rclcpp::Publisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr speed_filter_info_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // ── 输入缓存与渲染结果 ───────────────────────────────────────
    nav_msgs::msg::OccupancyGrid latest_map_;
    nav_interfaces::msg::AreaInfoArray latest_areas_;
    nav_msgs::msg::OccupancyGrid keepout_mask_;
    nav_msgs::msg::OccupancyGrid speed_mask_;
    std::vector<float> speed_limits_grid_;

    bool has_map_{false};
    bool has_areas_{false};

    std::string current_map_name_;
    size_t last_render_hash_{0U};
    bool has_last_render_hash_{false};
    LifecycleState state_{LifecycleState::WAITING_MAP};
    RenderStats last_stats_;

    // ── 参数 ─────────────────────────────────────────────────────
    std::string map_topic_;
    std::string areas_topic_;
    std::string map_name_topic_;
    std::string keepout_mask_topic_;
    std::string speed_mask_topic_;
    std::string keepout_filter_info_topic_;
    std::string speed_filter_info_topic_;

    double speed_base_{0.0};
    double speed_multiplier_{0.01};
    double virtual_wall_width_{0.10};
};

}  // namespace navigation_adapter

#endif  // NAVIGATION_ADAPTER__AREA_FILTER_ADAPTER_HPP_
