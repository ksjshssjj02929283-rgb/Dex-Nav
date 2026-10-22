// command_board_rviz_plugins / CommandBoardControlPanel
//
// 独立 RViz 控制 panel：只放依赖实时定位、作用于当前地图的 command_board 操作——
// 保存导航地图 / 备份地图与资产 / 按当前位姿重设原点 / 全局重定位（localization 服务）。
// 作用于任意地图的对齐 / 替换 / 按坐标重设原点 / 底图版本回滚在 Map Manager 面板，资产回滚在 Map Asset Manager 面板。
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
    void onResetByPose();    // ~/reset_map_origin_by_robot_pose（确认）
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
    QLabel*    op_result_label_{nullptr};
    QTimer*    ui_timer_{nullptr};

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<nav_interfaces::msg::NavSystemStatus>::SharedPtr status_sub_;
    rclcpp::Client<nav_interfaces::srv::SaveMap>::SharedPtr save_map_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr backup_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_pose_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr global_reloc_cli_;

    mutable std::mutex cache_mutex_;
    std::string current_map_name_;
    std::string last_op_{"--"};
    bool        last_op_ok_{true};
};

}  // namespace command_board_rviz_plugins

#endif  // COMMAND_BOARD_RVIZ_PLUGINS__COMMAND_BOARD_CONTROL_PANEL_HPP_
