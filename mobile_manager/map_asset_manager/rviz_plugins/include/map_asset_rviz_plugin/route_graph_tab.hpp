#ifndef MAP_ASSET_RVIZ_PLUGIN__ROUTE_GRAPH_TAB_HPP_
#define MAP_ASSET_RVIZ_PLUGIN__ROUTE_GRAPH_TAB_HPP_

#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QWidget>
#include <memory>
#include <nav_interfaces/srv/get_route_graph.hpp>
#include <nav_interfaces/srv/remove_route_edge.hpp>
#include <nav_interfaces/srv/remove_route_junction.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace map_asset_rviz_plugin {

class RouteGraphTab : public QWidget {
    Q_OBJECT

public:
    explicit RouteGraphTab(QWidget* parent = nullptr);

    void setNode(rclcpp::Node::SharedPtr node);
    void setServicePrefix(const std::string& prefix);

Q_SIGNALS:
    void statusChanged(QString msg);

private Q_SLOTS:
    void onRefresh();
    void onRemoveEdge();
    void onRemoveJunction();
    void applyRefresh(QStringList junctions, QStringList edges);
    void applyResult(bool success, QString msg);

private:
    void rebuildClients();

    QListWidget* junction_list_;
    QListWidget* edge_list_;
    QPushButton* refresh_btn_;
    QPushButton* remove_edge_btn_;
    QPushButton* remove_junction_btn_;
    QLabel* status_label_;

    rclcpp::Node::SharedPtr node_;
    std::string prefix_{"/map_asset_manager"};

    rclcpp::Client<nav_interfaces::srv::GetRouteGraph>::SharedPtr get_graph_client_;
    rclcpp::Client<nav_interfaces::srv::RemoveRouteEdge>::SharedPtr remove_edge_client_;
    rclcpp::Client<nav_interfaces::srv::RemoveRouteJunction>::SharedPtr remove_junction_client_;
};

}  // namespace map_asset_rviz_plugin

#endif  // MAP_ASSET_RVIZ_PLUGIN__ROUTE_GRAPH_TAB_HPP_
