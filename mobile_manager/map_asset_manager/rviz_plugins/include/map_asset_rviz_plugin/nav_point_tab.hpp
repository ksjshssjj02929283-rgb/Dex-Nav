#ifndef MAP_ASSET_RVIZ_PLUGIN__NAV_POINT_TAB_HPP_
#define MAP_ASSET_RVIZ_PLUGIN__NAV_POINT_TAB_HPP_

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QTimer>
#include <QVariantList>
#include <QWidget>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <nav_interfaces/action/navigate_to_point.hpp>
#include <nav_interfaces/srv/set_assets.hpp>
#include <nav_interfaces/srv/get_assets.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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
    void onSampleTick();
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

    rclcpp::Client<nav_interfaces::srv::GetAssets>::SharedPtr get_all_client_;
    rclcpp::Client<nav_interfaces::srv::SetAssets>::SharedPtr     add_client_;
    rclcpp::Client<nav_interfaces::srv::SetAssets>::SharedPtr  remove_client_;
    // 采样机器人当前位姿（map → base_link）用的 TF，随节点创建一次
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    QTimer* sample_timer_{nullptr};
    int sample_count_{0};
    double sample_x_{0.0}, sample_y_{0.0}, sample_sin_{0.0}, sample_cos_{0.0};
    rclcpp::Client<nav_interfaces::srv::SetAssets>::SharedPtr  update_client_;
    rclcpp_action::Client<NavigateToPointAction>::SharedPtr          nav_client_;

    std::mutex navigation_goal_mutex_;
    GoalHandleNavigateToPoint::SharedPtr navigation_goal_handle_;
    bool navigation_active_{false};
    uint64_t navigation_generation_{0};
};

}  // namespace map_asset_rviz_plugin

#endif  // MAP_ASSET_RVIZ_PLUGIN__NAV_POINT_TAB_HPP_
