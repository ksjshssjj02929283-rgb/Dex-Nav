#ifndef MAP_ASSET_RVIZ_PLUGIN__NAV_POINT_TAB_HPP_
#define MAP_ASSET_RVIZ_PLUGIN__NAV_POINT_TAB_HPP_

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QVariantList>
#include <QWidget>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <nav_interfaces/action/navigate_to_point.hpp>
#include <nav_interfaces/srv/set_nav_point.hpp>
#include <nav_interfaces/srv/get_all_nav_points.hpp>
#include <nav_interfaces/srv/sample_nav_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace map_asset_rviz_plugin {

class NavPointTab : public QWidget {
    Q_OBJECT

public:
    explicit NavPointTab(QWidget* parent = nullptr);

    void setNode(rclcpp::Node::SharedPtr node);
    void setServicePrefix(const std::string& prefix);

    // 由 NavPointDraftBridge 信号调用：将工具选点结果填入编辑框
    void prefillDraft(double x, double y, double yaw_rad);

Q_SIGNALS:
    void statusChanged(QString msg);
    // 跨线程信号
    void refreshDone(QVariantList points);   // 每项为 QVariantMap{name, type, x, y, theta, tolerance}
    void operationFinished(bool success, QString msg);
    void poseFilled(double x, double y, double yaw_rad);
    void navigationStatus(QString msg);
    void navigationActiveChanged(bool active);

private Q_SLOTS:
    void onRefresh();
    void onSample();
    void onAdd();
    void onRemove();
    void onEdit();
    void onNavTo();
    void onCancelNav();
    void applyRefresh(QVariantList points);
    void applyOperationResult(bool success, QString msg);
    void applyPoseFilled(double x, double y, double yaw_rad);
    void applyNavigationActive(bool active);

private:
    using NavigateToPointAction = nav_interfaces::action::NavigateToPoint;
    using GoalHandleNavigateToPoint =
        rclcpp_action::ClientGoalHandle<NavigateToPointAction>;

    void rebuildClients();

    QListWidget*   list_;
    QLineEdit*     name_edit_;
    QComboBox*     type_combo_;
    QDoubleSpinBox* x_spin_;
    QDoubleSpinBox* y_spin_;
    QDoubleSpinBox* theta_spin_;
    QPushButton*   refresh_btn_;
    QPushButton*   sample_btn_;
    QPushButton*   add_btn_;
    QPushButton*   edit_btn_;
    QPushButton*   remove_btn_;
    QPushButton*   nav_btn_;
    QPushButton*   cancel_nav_btn_;
    QLabel*        status_label_;

    rclcpp::Node::SharedPtr node_;
    std::string prefix_{"/map_asset_manager"};

    rclcpp::Client<nav_interfaces::srv::GetAllNavPoints>::SharedPtr get_all_client_;
    rclcpp::Client<nav_interfaces::srv::SetNavPoint>::SharedPtr     add_client_;
    rclcpp::Client<nav_interfaces::srv::SetNavPoint>::SharedPtr  remove_client_;
    rclcpp::Client<nav_interfaces::srv::SampleNavPoint>::SharedPtr  sample_client_;
    rclcpp::Client<nav_interfaces::srv::SetNavPoint>::SharedPtr  update_client_;
    rclcpp_action::Client<NavigateToPointAction>::SharedPtr          nav_client_;

    std::mutex navigation_goal_mutex_;
    GoalHandleNavigateToPoint::SharedPtr navigation_goal_handle_;
    bool navigation_active_{false};
    uint64_t navigation_generation_{0};
};

}  // namespace map_asset_rviz_plugin

#endif  // MAP_ASSET_RVIZ_PLUGIN__NAV_POINT_TAB_HPP_
