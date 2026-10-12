#ifndef MAP_MANAGER_RVIZ_PLUGIN__MAP_MANAGER_PANEL_HPP_
#define MAP_MANAGER_RVIZ_PLUGIN__MAP_MANAGER_PANEL_HPP_

#include <QButtonGroup>
#include <QGroupBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QTabWidget>

#include <string>
#include <vector>

#include <nav_interfaces/msg/map_info.hpp>
#include <nav_interfaces/msg/map_state.hpp>
#include <nav_interfaces/msg/map_version.hpp>
#include <nav_interfaces/srv/get_maps.hpp>
#include <nav_interfaces/srv/get_trashed_maps.hpp>
#include <nav_interfaces/srv/map_manager_command.hpp>
#include <nav_interfaces/srv/rollback_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

namespace Ogre {
class ManualObject;
class SceneNode;
}  // namespace Ogre

namespace map_manager_rviz_plugin {

// 地图管理面板：地图列表与地图级操作（加载 / 重命名 / 回收站 / 删除 / 显示朝向 / 版本回滚）
// + 底图编辑（配合 MapEditTool 画笔，map_command OP_EDIT_GRID 一次提交）+ 回收站恢复。
// 服务调用全部异步，结果经 Qt 主线程回到界面。
class MapManagerPanel : public rviz_common::Panel {
    Q_OBJECT

public:
    explicit MapManagerPanel(QWidget* parent = nullptr);
    ~MapManagerPanel() override;

    void onInitialize() override;
    void save(rviz_common::Config config) const override;
    void load(const rviz_common::Config& config) override;

private Q_SLOTS:
    void recreateClients();
    // 地图页
    void refreshMaps();
    void loadMap();
    void renameMap();
    void trashMap();
    void removeMap();
    void setDisplayYaw();
    void fillVersions();  // 选中地图（未选 = 当前地图）的版本，取自 get_maps 的 MapInfo.versions
    void rollbackToVersion();
    void onMapSelectionChanged();
    // 底图编辑页
    void startEdit();
    void activateBrush();
    void undoStroke();
    void clearStrokes();
    void saveEdit();
    void cancelEdit();
    void reloadEditedMap();
    void onBrushSettingsChanged();
    void onDraftChanged();
    // 回收站页
    void refreshTrash();
    void restoreTrash();

private:
    QWidget* buildMapsTab();
    std::string targetMap() const;  // 选中的地图，未选则为当前地图
    QWidget* buildEditTab();
    QWidget* buildTrashTab();

    std::string selectedMap() const;
    void sendMapCommand(uint8_t op, const std::string& map_name, const std::string& new_name,
                        const QString& what, bool refresh_after = true);
    void setStatus(const QString& text, bool error = false);
    void setEditingUi(bool editing);
    // 当前地图切换或重新加载（map_state 的名字 / epoch 变化）：编辑状态整体重置，未保存笔画丢弃
    void onMapStateChanged(const std::string& name, uint64_t epoch);
    void resetEdit(const QString& reason);
    void rebuildPreview();

    // 界面
    QLineEdit* mm_prefix_edit_;
    QLineEdit* cb_prefix_edit_;
    QTabWidget* tabs_;
    QLabel* status_label_;

    QTableWidget* maps_table_;
    QDoubleSpinBox* yaw_spin_;
    QTableWidget* versions_table_;
    QGroupBox* versions_box_;

    QLabel* edit_target_label_;
    QPushButton* start_edit_btn_;
    QPushButton* brush_btn_;
    QPushButton* undo_btn_;
    QPushButton* clear_btn_;
    QPushButton* save_btn_;
    QPushButton* cancel_btn_;
    QPushButton* reload_btn_;
    QButtonGroup* tool_group_;
    QComboBox* mode_combo_;
    QDoubleSpinBox* width_spin_;
    QLabel* draft_label_;
    QLabel* edit_result_label_;

    QTableWidget* trash_table_;

    // 状态
    std::vector<nav_interfaces::msg::MapInfo> maps_;
    std::vector<nav_interfaces::msg::MapVersion> versions_;  // 当前地图的底图版本，新的在前
    std::string current_map_;
    uint64_t map_epoch_ = 0;
    std::string edit_map_;        // 正在编辑的地图 = 开始编辑时的当前地图
    std::string edit_revision_;   // 开始编辑时取到的底图指纹，保存成功后换成新值

    // ROS
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<nav_interfaces::srv::GetMaps>::SharedPtr get_maps_cli_;
    rclcpp::Client<nav_interfaces::srv::MapManagerCommand>::SharedPtr cmd_cli_;
    rclcpp::Client<nav_interfaces::srv::GetTrashedMaps>::SharedPtr trash_cli_;
    rclcpp::Client<nav_interfaces::srv::RollbackMap>::SharedPtr rollback_cli_;
    rclcpp::Subscription<nav_interfaces::msg::MapState>::SharedPtr map_state_sub_;

    // 预览
    Ogre::ManualObject* preview_obj_ = nullptr;
    Ogre::SceneNode* preview_node_ = nullptr;
};

}  // namespace map_manager_rviz_plugin

#endif  // MAP_MANAGER_RVIZ_PLUGIN__MAP_MANAGER_PANEL_HPP_
