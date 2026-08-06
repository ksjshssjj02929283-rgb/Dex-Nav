#ifndef MAP_ASSET_RVIZ_PLUGIN__MAP_ASSET_PANEL_HPP_
#define MAP_ASSET_RVIZ_PLUGIN__MAP_ASSET_PANEL_HPP_

#include <QLineEdit>
#include <QPushButton>
#include <QTabWidget>

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

namespace map_asset_rviz_plugin {

class NavPointTab;
class RouteTab;
class AreaTab;
class RoomTab;
class RouteGraphTab;

class MapAssetPanel : public rviz_common::Panel {
    Q_OBJECT

public:
    explicit MapAssetPanel(QWidget* parent = nullptr);
    ~MapAssetPanel() override = default;

    void onInitialize() override;
    void save(rviz_common::Config config) const override;
    void load(const rviz_common::Config& config) override;

private Q_SLOTS:
    void onPrefixChanged();
    void onAddNavPointTool();
    void onAddAreaTool();
    void onAddRoomTool();          ///< 添加房间（多边形）[M]
    void onAddJunctionTool();
    void onAddEdgeTool();           ///< 直线路网边 [L]
    void onAddCurveEdgeTool();      ///< 贝塞尔曲线路网边 [B]
    void onRouteGraphDeleteTool();  ///< 统一删除 Junction/Edge [D]
    void onRouteGraphStartTool();   ///< 重载路网（start）[R]
    void onRouteGraphSaveTool();    ///< 保存路网 [S]

private:
    void activateTool(const QString& class_id);

    QLineEdit*       prefix_edit_;
    QPushButton*     add_point_btn_;
    QPushButton*     add_area_btn_;
    QPushButton*     add_room_btn_;
    QPushButton*     add_junction_btn_;
    QPushButton*     add_edge_btn_;
    QPushButton*     add_curve_edge_btn_;
    QPushButton*     route_graph_delete_btn_;
    QPushButton*     route_graph_start_btn_;
    QPushButton*     route_graph_save_btn_;
    QTabWidget*      tabs_;
    NavPointTab*     nav_point_tab_;
    RouteTab*        route_tab_;
    AreaTab*         area_tab_;
    RoomTab*         room_tab_;
    RouteGraphTab*   route_graph_tab_;

    rclcpp::Node::SharedPtr node_;
};

}  // namespace map_asset_rviz_plugin

#endif  // MAP_ASSET_RVIZ_PLUGIN__MAP_ASSET_PANEL_HPP_
