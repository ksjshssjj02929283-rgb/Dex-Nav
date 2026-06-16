#ifndef MAP_ASSET_RVIZ_PLUGIN__AREA_TAB_HPP_
#define MAP_ASSET_RVIZ_PLUGIN__AREA_TAB_HPP_

#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QWidget>
#include <memory>
#include <nav_interfaces/srv/get_all_areas.hpp>
#include <nav_interfaces/srv/remove_area_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace map_asset_rviz_plugin {

class AreaTab : public QWidget {
    Q_OBJECT

public:
    explicit AreaTab(QWidget* parent = nullptr);

    void setNode(rclcpp::Node::SharedPtr node);
    void setServicePrefix(const std::string& prefix);

Q_SIGNALS:
    void operationFinished(bool success, QString msg);

private Q_SLOTS:
    void onRefresh();
    void onRemove();
    void applyOperationResult(bool success, QString msg);

private:
    void rebuildClients();
    static QString typeLabel(uint32_t type);

    QListWidget* list_;
    QPushButton* refresh_btn_;
    QPushButton* remove_btn_;
    QLabel* status_label_;
    QLabel* detail_label_;

    rclcpp::Node::SharedPtr node_;
    std::string prefix_{"/map_asset_manager"};

    rclcpp::Client<nav_interfaces::srv::GetAllAreas>::SharedPtr get_client_;
    rclcpp::Client<nav_interfaces::srv::RemoveAreaInfo>::SharedPtr remove_client_;
};

}  // namespace map_asset_rviz_plugin

#endif  // MAP_ASSET_RVIZ_PLUGIN__AREA_TAB_HPP_
