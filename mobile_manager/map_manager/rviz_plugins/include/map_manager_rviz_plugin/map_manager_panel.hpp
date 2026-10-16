#ifndef MAP_MANAGER_RVIZ_PLUGIN__MAP_MANAGER_PANEL_HPP_
#define MAP_MANAGER_RVIZ_PLUGIN__MAP_MANAGER_PANEL_HPP_

#include <QButtonGroup>
#include <QCheckBox>
#include <QGroupBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QTabWidget>

#include <map>
#include <string>
#include <vector>

#include <nav_interfaces/msg/map_info.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_interfaces/msg/map_state.hpp>
#include <nav_interfaces/msg/map_version.hpp>
#include <nav_interfaces/srv/align_map.hpp>
#include <nav_interfaces/srv/get_maps.hpp>
#include <nav_interfaces/srv/get_trashed_maps.hpp>
#include <nav_interfaces/srv/map_manager_command.hpp>
#include <nav_interfaces/srv/map_package.hpp>
#include <nav_interfaces/srv/replace_map.hpp>
#include <nav_interfaces/srv/reset_map_origin_by_xy_yaw.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

class QTimer;

namespace rviz_common {
class Display;
}  // namespace rviz_common

namespace Ogre {
class ManualObject;
class SceneNode;
}  // namespace Ogre

namespace map_manager_rviz_plugin {

// 地图管理面板：地图列表与地图级操作（加载 / 重命名 / 回收站 / 删除 / 显示朝向 / 版本回滚）
// + 对齐 / 替换 / 按坐标重设原点（作用于选中的任意地图）
// + 底图编辑（配合 MapEditTool 画笔，map_command OP_EDIT_GRID 一次提交；可编辑任意地图，非当前图读本机文件发预览）
// + 回收站恢复。
// 依赖：除对齐 / 替换 / 重设原点外只调 map_manager；这三项要同时变换资产、当前地图还要停启定位，
// 由 command_board 编排，command_board 不在线时对应按钮置灰。
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
    // 对齐 / 替换 / 原点页（经 command_board）
    void alignDryRun();
    void alignApply();
    void replaceDryRun();
    void replaceApply();
    void resetOriginByXYYaw();
    void refreshCommandBoardState();  // 定时检查 command_board 服务是否在线，决定按钮是否可用
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
    // 导出 / 导入（~/map_package）：地图表可多选，逐张调用，一次只跑一批
    void exportMaps();
    void importMaps();

    void refreshTrash();
    void restoreTrash();

private:
    QWidget* buildMapsTab();
    std::string targetMap() const;  // 选中的地图，未选则为当前地图
    QWidget* buildMapOpsTab();
    QWidget* buildEditTab();
    QWidget* buildTrashTab();

    std::string selectedMap() const;
    std::vector<std::string> selectedMaps() const;  // 「地图」页选中的全部地图（多选），按表格顺序
    struct PackageJob {
        bool is_export = true;
        std::string map_name;   // 导出：要导出的图
        std::string file;       // 导入：本机上的包
    };
    void startPackageJobs(std::vector<PackageJob> jobs);
    void runNextPackageJob();
    void finishPackageJob(const QString& line, bool ok);
    void sendMapCommand(uint8_t op, const std::string& map_name, const std::string& new_name,
                        const QString& what, bool refresh_after = true);
    void setStatus(const QString& text, bool error = false);
    void setEditingUi(bool editing);
    // 当前地图切换或重新加载（map_state 的名字 / epoch 变化）：结束本次编辑，未保存笔画丢弃
    void onMapStateChanged(const std::string& name, uint64_t epoch);
    void resetEdit(const QString& reason);
    void rebuildPreview();
    void updateReloadState();  // 当前地图文件指纹与加载时不同 → 「重新加载使其生效」可用
    void fillMapOpsTarget();  // 对齐 / 替换 / 原点页：目标地图说明与替换源图候选
    void fillEditTarget();    // 底图编辑页：编辑对象下拉框与说明（与「地图」页选中同步）
    void selectMapInTable(const std::string& name);  // 程序内选中「地图」页某行（空 = 清除选择）
    // 编辑中编辑对象换成别的图：有未保存笔画先确认，确认后结束本次编辑；用户取消返回 false
    bool confirmTargetChange(const std::string& target);
    // 非当前图编辑：读本机文件发预览，隐藏当前图的显示（结束时恢复）
    bool publishEditPreview(const std::string& map_name, QString& err);
    void clearEditPreview();
    void applyEditFocus(bool focus);
    void callAlign(bool dry_run);
    void callReplace(bool dry_run);
    void setOpsResult(const QString& text, bool error);

    // 界面
    QLineEdit* mm_prefix_edit_;
    QLineEdit* cb_prefix_edit_;
    QTabWidget* tabs_;
    QLabel* status_label_;

    QTableWidget* maps_table_;
    QDoubleSpinBox* yaw_spin_;
    QTableWidget* versions_table_;
    QGroupBox* versions_box_;

    QLabel* ops_target_label_;
    QLabel* cb_state_label_;
    QLabel* ops_result_label_;
    QComboBox* source_combo_;
    QCheckBox* keep_source_check_;
    QCheckBox* accept_degraded_check_;
    QCheckBox* manual_check_;
    QLineEdit* manual_x_edit_;
    QLineEdit* manual_y_edit_;
    QLineEdit* manual_yaw_edit_;
    QLineEdit* reset_x_edit_;
    QLineEdit* reset_y_edit_;
    QLineEdit* reset_yaw_edit_;
    std::vector<QPushButton*> cb_buttons_;  // 依赖 command_board 的按钮
    QTimer* cb_timer_ = nullptr;

    QLabel* edit_target_label_;
    QComboBox* edit_target_combo_;
    bool syncing_selection_ = false;  // 程序内改选择 / 重建列表时置位，不触发「换编辑对象」确认
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
    QPushButton* export_btn_ = nullptr;
    QPushButton* import_btn_ = nullptr;

    // 状态
    std::vector<nav_interfaces::msg::MapInfo> maps_;
    std::vector<nav_interfaces::msg::MapVersion> versions_;  // 当前地图的底图版本，新的在前
    std::string current_map_;
    uint64_t map_epoch_ = 0;
    std::string edit_map_;        // 正在编辑的地图 = 开始编辑时的当前地图
    std::string edit_revision_;   // 开始编辑时取到的底图指纹，保存成功后换成新值
    bool edit_is_current_ = true; // 开始编辑时目标是否是当前地图
    std::string map_folder_;      // get_maps 给出的地图根目录（非当前图预览读文件用）
    // 导出 / 导入批次
    std::vector<PackageJob> package_jobs_;
    size_t package_index_ = 0;
    uint64_t package_seq_ = 0;          // 每发一次请求 +1；超时与迟到的响应按它对号
    std::string package_dest_dir_;      // 导出：复制到本机的目录
    uint8_t package_content_ = 0;
    uint8_t package_conflict_ = 0;
    QStringList package_report_;
    int package_ok_ = 0;
    std::string package_last_imported_;
    QString package_last_dir_;
    std::map<rviz_common::Display*, bool> focus_hidden_;  // 编辑非当前图时隐藏的显示及其原启用状态
    std::string loaded_map_;      // 下面两项对应的地图
    uint64_t loaded_epoch_ = 0;   // 记录 loaded_revision_ 时的 map_state epoch
    std::string loaded_revision_; // 当前地图加载后第一次取到的底图指纹（近似「导航正在用的那份」）

    // ROS
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<nav_interfaces::srv::GetMaps>::SharedPtr get_maps_cli_;
    rclcpp::Client<nav_interfaces::srv::MapManagerCommand>::SharedPtr cmd_cli_;
    rclcpp::Client<nav_interfaces::srv::GetTrashedMaps>::SharedPtr trash_cli_;
    rclcpp::Client<nav_interfaces::srv::AlignMap>::SharedPtr align_cli_;
    rclcpp::Client<nav_interfaces::srv::ReplaceMap>::SharedPtr replace_cli_;
    rclcpp::Client<nav_interfaces::srv::ResetMapOriginByXYYaw>::SharedPtr reset_xyyaw_cli_;
    rclcpp::Client<nav_interfaces::srv::MapPackage>::SharedPtr package_cli_;
    rclcpp::Subscription<nav_interfaces::msg::MapState>::SharedPtr map_state_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr edit_preview_pub_;

    // 预览
    Ogre::ManualObject* preview_obj_ = nullptr;
    Ogre::SceneNode* preview_node_ = nullptr;
};

}  // namespace map_manager_rviz_plugin

#endif  // MAP_MANAGER_RVIZ_PLUGIN__MAP_MANAGER_PANEL_HPP_
