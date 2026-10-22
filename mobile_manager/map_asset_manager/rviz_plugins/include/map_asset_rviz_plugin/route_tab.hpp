#ifndef MAP_ASSET_RVIZ_PLUGIN__ROUTE_TAB_HPP_
#define MAP_ASSET_RVIZ_PLUGIN__ROUTE_TAB_HPP_

#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QWidget>

#include <memory>
#include <string>

#include <nav_interfaces/srv/set_assets.hpp>
#include <nav_interfaces/srv/get_assets.hpp>
#include <rclcpp/rclcpp.hpp>

namespace map_asset_rviz_plugin {

class RouteTab : public QWidget {
    Q_OBJECT

public:
    explicit RouteTab(QWidget* parent = nullptr);

    void setNode(rclcpp::Node::SharedPtr node);
    void setServicePrefix(const std::string& prefix);

    // 外部（NavPointTab 刷新后）更新可选点列表
    void setAvailablePoints(const QStringList& names);

Q_SIGNALS:
    void statusChanged(QString msg);
    void refreshDone(QStringList routes);
    void operationFinished(bool success, QString msg);
    void pointListRefreshDone(QStringList points);

private Q_SLOTS:
    void onRefreshRoutes();
    void onRefreshPoints();
    void onCreate();
    void onRemove();
    void onRouteSelected(QListWidgetItem* item);
    void applyRefreshRoutes(QStringList routes);
    void applyOperationResult(bool success, QString msg);
    void applyPointListRefresh(QStringList points);

private:
    void rebuildClients();

    QListWidget* route_list_;     // 已有路线
    QListWidget* point_list_;     // 可选点位（多选）
    QListWidget* selected_list_;  // 已选点顺序
    QLineEdit*   name_edit_;
    QPushButton* add_point_btn_;  // >> 添加到已选
    QPushButton* remove_point_btn_;// << 从已选移除
    QPushButton* up_btn_;         // 上移
    QPushButton* down_btn_;       // 下移
    QPushButton* refresh_routes_btn_;
    QPushButton* refresh_points_btn_;
    QPushButton* create_btn_;
    QPushButton* remove_route_btn_;
    QLabel*      status_label_;
    QLabel*      route_detail_label_;

    rclcpp::Node::SharedPtr node_;
    std::string prefix_{"/map_asset_manager"};

    rclcpp::Client<nav_interfaces::srv::SetAssets>::SharedPtr        add_client_;
    rclcpp::Client<nav_interfaces::srv::SetAssets>::SharedPtr     remove_client_;
    rclcpp::Client<nav_interfaces::srv::GetAssets>::SharedPtr       get_client_;
    rclcpp::Client<nav_interfaces::srv::GetAssets>::SharedPtr    get_all_routes_client_;
    rclcpp::Client<nav_interfaces::srv::GetAssets>::SharedPtr       get_points_client_;
};

}  // namespace map_asset_rviz_plugin

#endif  // MAP_ASSET_RVIZ_PLUGIN__ROUTE_TAB_HPP_
