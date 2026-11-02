#ifndef MAP_ASSET_RVIZ_PLUGIN__MAP_ASSET_MANAGER_PANEL_HPP_
#define MAP_ASSET_RVIZ_PLUGIN__MAP_ASSET_MANAGER_PANEL_HPP_

#include <QCheckBox>
#include <QComboBox>
#include <QTimer>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QTabWidget>
#include <QTableWidget>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <nav_interfaces/msg/asset_change.hpp>
#include <nav_interfaces/msg/asset_snapshot.hpp>
#include <nav_interfaces/msg/map_state.hpp>
#include <nav_interfaces/srv/get_assets.hpp>
#include <nav_interfaces/srv/get_maps.hpp>
#include <nav_interfaces/srv/rollback_assets.hpp>
#include <nav_interfaces/srv/save_assets.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace map_asset_rviz_plugin {

class NavPointTab;
class RouteTab;
class AreaTab;
class RoomTab;
class RouteGraphTab;

// 地图资产管理面板：编辑目标（当前地图或任意一张地图）+ 资产工具 + 分类页签 + 版本 / 回滚 + 失效清单。
// 选了非当前地图时，所有页签与工具都按该图读写（map_asset_manager 按图寻址），
// 并在 ~/target_map、~/target_assets 发布该图的底图与资产预览（RViz 里加对应显示）。
class MapAssetManagerPanel : public rviz_common::Panel {
    Q_OBJECT

public:
    explicit MapAssetManagerPanel(QWidget* parent = nullptr);
    ~MapAssetManagerPanel() override = default;

    void onInitialize() override;
    void save(rviz_common::Config config) const override;
    void load(const rviz_common::Config& config) override;

private Q_SLOTS:
    void onPrefixChanged();
    void refreshMapList();
    void onTargetChanged(int index);
    void refreshTargetInfo();        // 指纹、数量、版本列表、失效清单 + 非当前图预览
    void saveVersion();
    void rollbackToVersion();
    void onAddNavPointTool();
    void onAddAreaTool();
    void onAddRoomTool();
    void onAddJunctionTool();
    void onAddEdgeTool();
    void onAddCurveEdgeTool();
    void onRouteGraphDeleteTool();
    void onRouteGraphStartTool();
    void onRouteGraphSaveTool();

private:
    void activateTool(const QString& class_id);
    void recreateClients();
    void refreshTabs();
    void applyTarget(const std::string& map_name);  // 空 = 当前地图
    void setStatus(const QString& text, bool error = false);
    void showTargetInfo(const nav_interfaces::srv::GetAssets::Response& r);
    void publishPreview(const nav_interfaces::srv::GetAssets::Response& r);
    void clearPreview();
    // 「只看编辑目标」：编辑非当前图时隐藏当前加载图的底图 / 资产 / 掩膜显示，切回时恢复原状态
    void applyDisplayFocus();
    void scheduleRefresh();
    bool loadTargetGrid(const std::string& map_name, nav_msgs::msg::OccupancyGrid& grid, QString& err) const;

    QLineEdit* prefix_edit_;
    QLineEdit* mm_prefix_edit_;
    QComboBox* target_combo_;
    QLabel* target_info_label_;
    QLabel* status_label_;
    QTabWidget* tabs_;
    NavPointTab* nav_point_tab_;
    RouteTab* route_tab_;
    AreaTab* area_tab_;
    RoomTab* room_tab_;
    RouteGraphTab* route_graph_tab_;
    QTableWidget* versions_table_;
    QListWidget* issues_list_;
    QCheckBox* focus_check_;
    QTimer* refresh_timer_;
    std::map<void*, bool> hidden_displays_;  // 被本面板隐藏的显示 → 隐藏前是否启用

    std::string current_map_;
    std::string map_folder_;
    std::string pending_target_;  // load() 读到的目标，地图列表到达后再选中
    std::string last_revision_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<nav_interfaces::srv::GetMaps>::SharedPtr get_maps_cli_;
    rclcpp::Client<nav_interfaces::srv::GetAssets>::SharedPtr get_assets_cli_;
    rclcpp::Client<nav_interfaces::srv::SaveAssets>::SharedPtr save_assets_cli_;
    rclcpp::Client<nav_interfaces::srv::RollbackAssets>::SharedPtr rollback_cli_;
    rclcpp::Subscription<nav_interfaces::msg::MapState>::SharedPtr map_state_sub_;
    rclcpp::Subscription<nav_interfaces::msg::AssetSnapshot>::SharedPtr assets_sub_;
    rclcpp::Subscription<nav_interfaces::msg::AssetChange>::SharedPtr changes_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr preview_map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr preview_assets_pub_;
};

}  // namespace map_asset_rviz_plugin

#endif  // MAP_ASSET_RVIZ_PLUGIN__MAP_ASSET_MANAGER_PANEL_HPP_
