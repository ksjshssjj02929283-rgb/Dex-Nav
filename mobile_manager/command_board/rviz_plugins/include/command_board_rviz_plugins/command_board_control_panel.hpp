// command_board_rviz_plugins / CommandBoardControlPanel
//
// 独立 RViz 控制 panel：把 command_board 对外服务做成按钮（保存导航地图 / 备份地图与资产 /
// 回退地图 / 重置原点）+ localization 全局重定位。订阅 NavSystemStatus 拿当前地图名
// （save / reset 默认作用于当前地图）。
//
// 线程模型：服务响应回调（executor 线程）只写缓存；QTimer(GUI 线程)读缓存刷 widget。

#ifndef COMMAND_BOARD_RVIZ_PLUGINS__COMMAND_BOARD_CONTROL_PANEL_HPP_
#define COMMAND_BOARD_RVIZ_PLUGINS__COMMAND_BOARD_CONTROL_PANEL_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <QLabel>
#include <QLineEdit>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_interfaces/msg/nav_system_status.hpp>
#include <nav_interfaces/srv/save_map.hpp>
#include <nav_interfaces/srv/rollback_map.hpp>
#include <nav_interfaces/srv/reset_map_origin_by_xy_yaw.hpp>

class QTimer;

namespace command_board_rviz_plugins {

class CommandBoardControlPanel : public rviz_common::Panel {
    Q_OBJECT

public:
    explicit CommandBoardControlPanel(QWidget* parent = nullptr);
    ~CommandBoardControlPanel() override = default;

    void onInitialize() override;
    void save(rviz_common::Config config) const override;
    void load(const rviz_common::Config& config) override;

private Q_SLOTS:
    void onTopicChanged();   // 改前缀重建 clients + 重订阅
    void refreshUi();        // QTimer：刷当前地图 + 操作结果
    void onSaveMap();        // ~/save_map（空=当前地图）
    void onBackup();         // ~/backup_unified
    void onRollback();       // ~/rollback_map（确认）
    void onResetByPose();    // ~/reset_map_origin_by_robot_pose（确认）
    void onResetByXYYaw();   // ~/reset_map_origin_by_xyyaw（当前地图，确认）
    void onGlobalReloc();    // /localization/trigger_global_relocalization（确认）

private:
    void setupUi();
    void makeClients();
    void subscribeStatus();
    void onStatus(nav_interfaces::msg::NavSystemStatus::SharedPtr msg);   // 回调：存当前地图名
    void setOpPending(const std::string& op);
    void storeOpResult(const std::string& op, bool ok, const std::string& msg);

    QLineEdit* topic_edit_{nullptr};
    QLabel*    map_label_{nullptr};
    QLineEdit* rollback_mapver_edit_{nullptr};
    QLineEdit* rollback_assetver_edit_{nullptr};
    QLineEdit* reset_x_edit_{nullptr};
    QLineEdit* reset_y_edit_{nullptr};
    QLineEdit* reset_yaw_edit_{nullptr};
    QLabel*    op_result_label_{nullptr};
    QTimer*    ui_timer_{nullptr};

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<nav_interfaces::msg::NavSystemStatus>::SharedPtr status_sub_;
    rclcpp::Client<nav_interfaces::srv::SaveMap>::SharedPtr save_map_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr backup_cli_;
    rclcpp::Client<nav_interfaces::srv::RollbackMap>::SharedPtr rollback_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_pose_cli_;
    rclcpp::Client<nav_interfaces::srv::ResetMapOriginByXYYaw>::SharedPtr reset_xyyaw_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr global_reloc_cli_;

    mutable std::mutex cache_mutex_;
    std::string current_map_name_;
    std::string last_op_{"--"};
    bool        last_op_ok_{true};
};

}  // namespace command_board_rviz_plugins

#endif  // COMMAND_BOARD_RVIZ_PLUGINS__COMMAND_BOARD_CONTROL_PANEL_HPP_
