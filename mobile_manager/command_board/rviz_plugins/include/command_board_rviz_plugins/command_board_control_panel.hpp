// command_board_rviz_plugins / CommandBoardControlPanel
//
// 独立 RViz 控制 panel：把 command_board 对外服务做成按钮 + localization 全局重定位。
// 按「是否依赖实时定位」分两区：
//   - 当前地图：保存导航地图 / 备份地图与资产 / 按当前位姿重设原点 / 全局重定位
//   - 目标地图（从 map_manager get_maps 的列表里选，任意一张）：回退（底图版本取自 MapInfo.versions）/
//     按坐标重设原点 / 对齐 / 替换。目标是当前地图时 command_board 会额外停启定位并重新加载。
//
// 线程模型：服务响应回调（executor 线程）只写缓存；QTimer(GUI 线程)读缓存刷 widget。

#ifndef COMMAND_BOARD_RVIZ_PLUGINS__COMMAND_BOARD_CONTROL_PANEL_HPP_
#define COMMAND_BOARD_RVIZ_PLUGINS__COMMAND_BOARD_CONTROL_PANEL_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QLineEdit>

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_interfaces/msg/nav_system_status.hpp>
#include <nav_interfaces/msg/map_info.hpp>
#include <nav_interfaces/srv/align_map.hpp>
#include <nav_interfaces/srv/get_assets.hpp>
#include <nav_interfaces/srv/get_maps.hpp>
#include <nav_interfaces/srv/replace_map.hpp>
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
    void onResetByXYYaw();   // ~/reset_map_origin_by_xyyaw（目标地图，确认）
    void onGlobalReloc();    // /localization/trigger_global_relocalization（确认）
    void onRefreshMaps();    // map_manager ~/get_maps → 目标地图下拉框
    void onTargetChanged();  // 目标地图切换：刷新版本下拉框与替换源图候选
    void onAlignDryRun();    // ~/align_map dry_run（目标地图）
    void onAlignApply();     // ~/align_map（目标地图，确认）
    void onReplaceDryRun();  // ~/replace_map dry_run（目标地图 ← 源图）
    void onReplaceApply();   // ~/replace_map（确认）

private:
    void setupUi();
    void makeClients();
    void subscribeStatus();
    void onStatus(nav_interfaces::msg::NavSystemStatus::SharedPtr msg);   // 回调：存当前地图名
    void setOpPending(const std::string& op);
    void storeOpResult(const std::string& op, bool ok, const std::string& msg);
    std::string targetMap() const;          // 目标地图下拉框选中的地图名
    void applyMaps(const std::vector<nav_interfaces::msg::MapInfo>& maps, const std::string& current);
    void callAlign(bool dry_run);
    void refreshAssetVersions(const std::string& target);
    void callReplace(bool dry_run);

    QLineEdit* topic_edit_{nullptr};
    QLineEdit* mm_prefix_edit_{nullptr};      // map_manager 前缀（取地图列表）
    QComboBox* target_combo_{nullptr};        // 目标地图（itemData = 地图名）
    QComboBox* mapver_combo_{nullptr};        // 底图版本（itemData = version_ts，"" = 最新）
    QComboBox* source_combo_{nullptr};        // 替换源图
    QCheckBox* keep_source_check_{nullptr};
    QCheckBox* accept_degraded_check_{nullptr};
    QCheckBox* manual_check_{nullptr};
    QLineEdit* manual_x_edit_{nullptr};
    QLineEdit* manual_y_edit_{nullptr};
    QLineEdit* manual_yaw_edit_{nullptr};
    QLabel*    map_label_{nullptr};
    QComboBox* assetver_combo_{nullptr};      // 资产版本（itemData = version_ts，"" = 最新，"none" = 不回退）
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
    rclcpp::Client<nav_interfaces::srv::GetMaps>::SharedPtr get_maps_cli_;
    rclcpp::Client<nav_interfaces::srv::GetAssets>::SharedPtr get_assets_cli_;   // 目标地图的资产版本列表
    rclcpp::Client<nav_interfaces::srv::AlignMap>::SharedPtr align_cli_;
    rclcpp::Client<nav_interfaces::srv::ReplaceMap>::SharedPtr replace_cli_;

    std::vector<nav_interfaces::msg::MapInfo> maps_;   // 只在 GUI 线程读写

    mutable std::mutex cache_mutex_;
    std::string current_map_name_;
    std::string last_op_{"--"};
    bool        last_op_ok_{true};
};

}  // namespace command_board_rviz_plugins

#endif  // COMMAND_BOARD_RVIZ_PLUGINS__COMMAND_BOARD_CONTROL_PANEL_HPP_
