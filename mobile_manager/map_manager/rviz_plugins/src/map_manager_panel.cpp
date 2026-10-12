#include "map_manager_rviz_plugin/map_manager_panel.hpp"

#include <QDateTime>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QInputDialog>
#include <QMessageBox>
#include <QPointer>
#include <QRadioButton>
#include <QVBoxLayout>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>

#include <cmath>
#include <map>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/tool_manager.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <nav_interfaces/msg/grid_stroke.hpp>
#include <nav_interfaces/msg/map_state.hpp>

#include "map_manager_rviz_plugin/grid_edit_draft.hpp"

namespace map_manager_rviz_plugin {

namespace {

using GetMaps = nav_interfaces::srv::GetMaps;
using MapCommand = nav_interfaces::srv::MapManagerCommand;
using GetTrashedMaps = nav_interfaces::srv::GetTrashedMaps;
using RollbackMap = nav_interfaces::srv::RollbackMap;

constexpr char kMapEditToolId[] = "map_manager_rviz_plugin/MapEditTool";
constexpr char kPreviewMaterial[] = "MapManagerPanelEditPreview";

// 异步调服务，结果回到 Qt 主线程；服务不可用时 handler 收到 nullptr。
template <typename SrvT, typename Handler>
void CallAsync(QObject* ctx, const typename rclcpp::Client<SrvT>::SharedPtr& cli,
               std::shared_ptr<typename SrvT::Request> req, Handler handler) {
    if (!cli || !cli->service_is_ready()) {
        QMetaObject::invokeMethod(ctx, [handler]() { handler(nullptr); }, Qt::QueuedConnection);
        return;
    }
    QPointer<QObject> guard(ctx);
    cli->async_send_request(req, [guard, handler](typename rclcpp::Client<SrvT>::SharedFuture fut) {
        auto resp = fut.get();
        if (!guard) return;
        QMetaObject::invokeMethod(guard.data(), [handler, resp]() { handler(resp); }, Qt::QueuedConnection);
    });
}

QString TimeText(const builtin_interfaces::msg::Time& t) {
    if (t.sec <= 0) return "-";
    return QDateTime::fromSecsSinceEpoch(t.sec).toString("yyyy-MM-dd HH:mm:ss");
}

QTableWidgetItem* Item(const QString& text) {
    auto* it = new QTableWidgetItem(text);
    it->setFlags(it->flags() & ~Qt::ItemIsEditable);
    return it;
}

QTableWidget* MakeTable(const QStringList& headers) {
    auto* t = new QTableWidget(0, headers.size());
    t->setHorizontalHeaderLabels(headers);
    t->setSelectionBehavior(QAbstractItemView::SelectRows);
    t->setSelectionMode(QAbstractItemView::SingleSelection);
    t->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    t->horizontalHeader()->setStretchLastSection(true);
    t->verticalHeader()->setVisible(false);
    return t;
}

}  // namespace

// ── 构造与界面 ──────────────────────────────────────────────────────────

MapManagerPanel::MapManagerPanel(QWidget* parent) : rviz_common::Panel(parent) {
    auto* layout = new QVBoxLayout(this);

    auto* prefix_row = new QHBoxLayout();
    mm_prefix_edit_ = new QLineEdit("/map_manager");
    cb_prefix_edit_ = new QLineEdit("/command_board");
    mm_prefix_edit_->setToolTip("map_manager 节点名（get_maps / map_command / get_trashed_maps）");
    cb_prefix_edit_->setToolTip("command_board 节点名（rollback_map）；单独 ros2 run 起的默认名是 /command_board_manager");
    prefix_row->addWidget(new QLabel("map_manager:"));
    prefix_row->addWidget(mm_prefix_edit_);
    prefix_row->addWidget(new QLabel("command_board:"));
    prefix_row->addWidget(cb_prefix_edit_);
    layout->addLayout(prefix_row);

    tabs_ = new QTabWidget();
    tabs_->addTab(buildMapsTab(), "地图");
    tabs_->addTab(buildEditTab(), "底图编辑");
    tabs_->addTab(buildTrashTab(), "回收站");
    layout->addWidget(tabs_);

    status_label_ = new QLabel("就绪");
    status_label_->setWordWrap(true);
    layout->addWidget(status_label_);

    connect(mm_prefix_edit_, &QLineEdit::editingFinished, this, &MapManagerPanel::recreateClients);
    connect(cb_prefix_edit_, &QLineEdit::editingFinished, this, &MapManagerPanel::recreateClients);
    connect(&GridEditDraft::instance(), &GridEditDraft::changed, this, &MapManagerPanel::onDraftChanged);
}

MapManagerPanel::~MapManagerPanel() {
    GridEditDraft::instance().editing = false;
    GridEditDraft::instance().clear();
    if (preview_node_ && getDisplayContext()) {
        auto* scene = getDisplayContext()->getSceneManager();
        scene->destroyManualObject(preview_obj_);
        scene->destroySceneNode(preview_node_);
    }
}

QWidget* MapManagerPanel::buildMapsTab() {
    auto* w = new QWidget();
    auto* v = new QVBoxLayout(w);

    maps_table_ = MakeTable({"名称", "当前", "有效面积 m²", "总面积 m²", "显示朝向 °", "底图最后编辑", "创建时间", "数据版本"});
    v->addWidget(maps_table_);
    connect(maps_table_, &QTableWidget::itemSelectionChanged, this, &MapManagerPanel::onMapSelectionChanged);

    auto* ops = new QHBoxLayout();
    auto add = [&](const QString& text, void (MapManagerPanel::*slot)()) {
        auto* b = new QPushButton(text);
        connect(b, &QPushButton::clicked, this, slot);
        ops->addWidget(b);
    };
    add("刷新", &MapManagerPanel::refreshMaps);
    add("加载", &MapManagerPanel::loadMap);
    add("重命名", &MapManagerPanel::renameMap);
    add("移入回收站", &MapManagerPanel::trashMap);
    add("删除", &MapManagerPanel::removeMap);
    v->addLayout(ops);
    // 对齐 / 替换要同时变换资产、当前地图还要停启定位，属跨节点编排，入口只放在 Command Board 面板
    auto* align_hint = new QLabel("地图对齐、地图替换、按坐标重设原点：请在 Command Board 面板「目标地图」区操作（可作用于任意地图）。"
                                  "这些操作前存下的底图版本会出现在下方版本列表（来源「对齐前」「替换前」「重设原点前」），可在此回滚。");
    align_hint->setWordWrap(true);
    align_hint->setStyleSheet("color: gray;");
    v->addWidget(align_hint);

    auto* yaw_box = new QGroupBox("显示朝向（只影响前端显示，坐标与定位不变）");
    auto* yaw_row = new QHBoxLayout(yaw_box);
    yaw_spin_ = new QDoubleSpinBox();
    yaw_spin_->setRange(-180.0, 180.0);
    yaw_spin_->setDecimals(1);
    yaw_spin_->setSingleStep(1.0);
    yaw_spin_->setSuffix(" °");
    auto* yaw_btn = new QPushButton("设置显示朝向");
    connect(yaw_btn, &QPushButton::clicked, this, &MapManagerPanel::setDisplayYaw);
    yaw_row->addWidget(yaw_spin_);
    yaw_row->addWidget(yaw_btn);
    v->addWidget(yaw_box);

    versions_box_ = new QGroupBox("底图版本");
    auto* rb_box = versions_box_;
    auto* rb_v = new QVBoxLayout(rb_box);
    versions_table_ = MakeTable({"版本", "保存时间", "来源", "三件齐全"});
    versions_table_->setMinimumHeight(140);
    rb_v->addWidget(versions_table_);
    auto* rb_row = new QHBoxLayout();
    auto* rb_refresh = new QPushButton("刷新");
    auto* rb_btn = new QPushButton("回滚到此版本");
    connect(rb_refresh, &QPushButton::clicked, this, &MapManagerPanel::refreshMaps);
    connect(rb_btn, &QPushButton::clicked, this, &MapManagerPanel::rollbackToVersion);
    rb_row->addWidget(rb_refresh);
    rb_row->addWidget(rb_btn);
    rb_v->addLayout(rb_row);
    v->addWidget(rb_box);
    return w;
}

QWidget* MapManagerPanel::buildEditTab() {
    auto* w = new QWidget();
    auto* v = new QVBoxLayout(w);

    edit_target_label_ = new QLabel("编辑对象：当前地图（RViz 显示的那张；切换或重新加载地图后需重新开始编辑）");
    edit_target_label_->setWordWrap(true);
    v->addWidget(edit_target_label_);

    auto* brush_box = new QGroupBox("画笔");
    auto* form = new QFormLayout(brush_box);
    auto* tool_row = new QHBoxLayout();
    tool_group_ = new QButtonGroup(this);
    const std::pair<int, const char*> tools[] = {{kToolWall, "墙体"}, {kToolErase, "擦除噪点"}, {kToolUnknown, "未知区域"}};
    for (const auto& t : tools) {
        auto* rb = new QRadioButton(t.second);
        tool_group_->addButton(rb, t.first);
        tool_row->addWidget(rb);
    }
    tool_group_->button(kToolWall)->setChecked(true);
    form->addRow("工具:", tool_row);
    mode_combo_ = new QComboBox();
    mode_combo_->addItem("自由绘制", static_cast<int>(DrawMode::kFree));
    mode_combo_->addItem("直线（按住 Shift 吸附 45°）", static_cast<int>(DrawMode::kLine));
    mode_combo_->addItem("矩形", static_cast<int>(DrawMode::kRect));
    form->addRow("方式:", mode_combo_);
    width_spin_ = new QDoubleSpinBox();
    width_spin_->setRange(0.05, 3.00);
    width_spin_->setSingleStep(0.05);
    width_spin_->setDecimals(2);
    width_spin_->setValue(0.20);
    width_spin_->setSuffix(" m");
    form->addRow("宽度（笔刷直径）:", width_spin_);
    v->addWidget(brush_box);
    connect(tool_group_, &QButtonGroup::idClicked, this,
            [this](int) { onBrushSettingsChanged(); });
    connect(mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            [this](int) { onBrushSettingsChanged(); });
    connect(width_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            [this](double) { onBrushSettingsChanged(); });

    auto* row1 = new QHBoxLayout();
    start_edit_btn_ = new QPushButton("开始编辑");
    brush_btn_ = new QPushButton("画笔 [E]");
    undo_btn_ = new QPushButton("撤销一笔");
    clear_btn_ = new QPushButton("清空");
    row1->addWidget(start_edit_btn_);
    row1->addWidget(brush_btn_);
    row1->addWidget(undo_btn_);
    row1->addWidget(clear_btn_);
    v->addLayout(row1);
    auto* row2 = new QHBoxLayout();
    save_btn_ = new QPushButton("保存修改");
    cancel_btn_ = new QPushButton("取消编辑");
    reload_btn_ = new QPushButton("重新加载使其生效");
    row2->addWidget(save_btn_);
    row2->addWidget(cancel_btn_);
    row2->addWidget(reload_btn_);
    v->addLayout(row2);
    connect(start_edit_btn_, &QPushButton::clicked, this, &MapManagerPanel::startEdit);
    connect(brush_btn_, &QPushButton::clicked, this, &MapManagerPanel::activateBrush);
    connect(undo_btn_, &QPushButton::clicked, this, &MapManagerPanel::undoStroke);
    connect(clear_btn_, &QPushButton::clicked, this, &MapManagerPanel::clearStrokes);
    connect(save_btn_, &QPushButton::clicked, this, &MapManagerPanel::saveEdit);
    connect(cancel_btn_, &QPushButton::clicked, this, &MapManagerPanel::cancelEdit);
    connect(reload_btn_, &QPushButton::clicked, this, &MapManagerPanel::reloadEditedMap);

    draft_label_ = new QLabel("待保存 0 笔");
    edit_result_label_ = new QLabel();
    edit_result_label_->setWordWrap(true);
    v->addWidget(draft_label_);
    v->addWidget(edit_result_label_);
    v->addWidget(new QLabel("提示：Fixed Frame 须为 map；笔画按 map 坐标落盘，红 = 墙体、绿 = 擦除、灰 = 未知。\n"
                            "擦除只把障碍变为可通行，未知区域保持未知。保存前可随时撤销；保存后可在「地图」页回滚。"));
    v->addStretch();
    setEditingUi(false);
    reload_btn_->setEnabled(false);
    return w;
}

QWidget* MapManagerPanel::buildTrashTab() {
    auto* w = new QWidget();
    auto* v = new QVBoxLayout(w);
    trash_table_ = MakeTable({"条目名", "原地图名", "归档时间", "有效面积 m²"});
    v->addWidget(trash_table_);
    auto* row = new QHBoxLayout();
    auto* refresh = new QPushButton("刷新");
    auto* restore = new QPushButton("恢复选中条目");
    connect(refresh, &QPushButton::clicked, this, &MapManagerPanel::refreshTrash);
    connect(restore, &QPushButton::clicked, this, &MapManagerPanel::restoreTrash);
    row->addWidget(refresh);
    row->addWidget(restore);
    v->addLayout(row);
    return w;
}

// ── 初始化与配置 ────────────────────────────────────────────────────────

void MapManagerPanel::onInitialize() {
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    recreateClients();

    auto* scene = getDisplayContext()->getSceneManager();
    auto& mm = Ogre::MaterialManager::getSingleton();
    if (!mm.resourceExists(kPreviewMaterial, "rviz_rendering")) {
        auto mat = mm.create(kPreviewMaterial, "rviz_rendering");
        auto* pass = mat->getTechnique(0)->getPass(0);
        pass->setLightingEnabled(false);
        pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        pass->setDepthWriteEnabled(false);
        pass->setCullingMode(Ogre::CULL_NONE);
        pass->setVertexColourTracking(Ogre::TVC_DIFFUSE);
    }
    preview_obj_ = scene->createManualObject();
    preview_obj_->setDynamic(true);
    preview_node_ = scene->getRootSceneNode()->createChildSceneNode();
    preview_node_->attachObject(preview_obj_);
    refreshMaps();
}

void MapManagerPanel::recreateClients() {
    if (!node_) return;
    const std::string mm = mm_prefix_edit_->text().toStdString();
    const std::string cb = cb_prefix_edit_->text().toStdString();
    get_maps_cli_ = node_->create_client<GetMaps>(mm + "/get_maps");
    cmd_cli_ = node_->create_client<MapCommand>(mm + "/map_command");
    trash_cli_ = node_->create_client<GetTrashedMaps>(mm + "/get_trashed_maps");
    rollback_cli_ = node_->create_client<RollbackMap>(cb + "/rollback_map");
    QPointer<MapManagerPanel> guard(this);
    map_state_sub_ = node_->create_subscription<nav_interfaces::msg::MapState>(
        mm + "/map_state", rclcpp::QoS(1).transient_local().reliable(),
        [guard](nav_interfaces::msg::MapState::ConstSharedPtr msg) {
            if (!guard) return;
            const std::string name = msg->map_name;
            const uint64_t epoch = msg->epoch;
            QMetaObject::invokeMethod(guard.data(), [guard, name, epoch]() {
                if (guard) guard->onMapStateChanged(name, epoch);
            }, Qt::QueuedConnection);
        });
    Q_EMIT configChanged();
}

void MapManagerPanel::save(rviz_common::Config config) const {
    rviz_common::Panel::save(config);
    config.mapSetValue("MapManagerPrefix", mm_prefix_edit_->text());
    config.mapSetValue("CommandBoardPrefix", cb_prefix_edit_->text());
}

void MapManagerPanel::load(const rviz_common::Config& config) {
    rviz_common::Panel::load(config);
    QString v;
    if (config.mapGetString("MapManagerPrefix", &v)) mm_prefix_edit_->setText(v);
    if (config.mapGetString("CommandBoardPrefix", &v)) cb_prefix_edit_->setText(v);
    recreateClients();
}

void MapManagerPanel::setStatus(const QString& text, bool error) {
    status_label_->setStyleSheet(error ? "color: #c0392b;" : "");
    status_label_->setText(text);
}

// ── 地图页 ─────────────────────────────────────────────────────────────

std::string MapManagerPanel::selectedMap() const {
    const int row = maps_table_->currentRow();
    if (row < 0 || row >= static_cast<int>(maps_.size())) return "";
    return maps_[static_cast<size_t>(row)].name;
}

void MapManagerPanel::refreshMaps() {
    CallAsync<GetMaps>(this, get_maps_cli_, std::make_shared<GetMaps::Request>(),
                       [this](GetMaps::Response::SharedPtr r) {
        if (!r) { setStatus("get_maps 服务不可用，检查 map_manager 前缀", true); return; }
        if (!r->success) { setStatus(QString("get_maps 失败 [%1]: %2").arg(r->error_code).arg(QString::fromStdString(r->message)), true); return; }
        const std::string keep = selectedMap();
        maps_ = r->maps;
        current_map_ = r->current_map_name;
        maps_table_->setRowCount(static_cast<int>(maps_.size()));
        int select_row = -1;
        for (size_t i = 0; i < maps_.size(); ++i) {
            const auto& m = maps_[i];
            const int row = static_cast<int>(i);
            maps_table_->setItem(row, 0, Item(QString::fromStdString(m.name)));
            maps_table_->setItem(row, 1, Item(m.name == current_map_ ? "●" : ""));
            maps_table_->setItem(row, 2, Item(QString::number(m.effective_area_m2, 'f', 2)));
            maps_table_->setItem(row, 3, Item(QString::number(m.total_area_m2, 'f', 2)));
            maps_table_->setItem(row, 4, Item(QString::number(m.display_yaw * 180.0 / M_PI, 'f', 1)));
            maps_table_->setItem(row, 5, Item(TimeText(m.last_edited_at)));
            maps_table_->setItem(row, 6, Item(m.created_at.empty() ? "-" : QString::fromStdString(m.created_at)));
            maps_table_->setItem(row, 7, Item(m.data_version.empty() ? "-" : QString::fromStdString(m.data_version)));
            if (m.name == keep) select_row = row;
        }
        if (select_row >= 0) maps_table_->selectRow(select_row);
        setStatus(QString("%1 张地图，当前：%2").arg(maps_.size())
                      .arg(current_map_.empty() ? "（未加载）" : QString::fromStdString(current_map_)));
        fillVersions();
    });
}

void MapManagerPanel::onMapSelectionChanged() {
    const int row = maps_table_->currentRow();
    if (row >= 0 && row < static_cast<int>(maps_.size())) {
        yaw_spin_->setValue(maps_[static_cast<size_t>(row)].display_yaw * 180.0 / M_PI);
    }
    fillVersions();
    if (!GridEditDraft::instance().editing) {
        edit_target_label_->setText(QString("编辑对象：当前地图 %1")
                                        .arg(current_map_.empty() ? "（未加载）" : QString::fromStdString(current_map_)));
    }
}

void MapManagerPanel::sendMapCommand(uint8_t op, const std::string& map_name, const std::string& new_name,
                                     const QString& what, bool refresh_after) {
    auto req = std::make_shared<MapCommand::Request>();
    req->op = op;
    req->map_name = map_name;
    req->new_name = new_name;
    setStatus(what + " ...");
    CallAsync<MapCommand>(this, cmd_cli_, req, [this, what, refresh_after](MapCommand::Response::SharedPtr r) {
        if (!r) { setStatus("map_command 服务不可用", true); return; }
        setStatus(QString("%1%2 [%3] %4").arg(what).arg(r->success ? "成功" : "失败").arg(r->error_code)
                      .arg(QString::fromStdString(r->message)), !r->success);
        if (refresh_after) { refreshMaps(); refreshTrash(); }
    });
}

void MapManagerPanel::loadMap() {
    const std::string name = selectedMap();
    if (name.empty()) { setStatus("先在列表中选中一张地图", true); return; }
    if (QMessageBox::question(this, "加载地图",
                              QString("加载「%1」为当前地图？\n整机运行时定位与导航会随之重新加载，需要重新定位。")
                                  .arg(QString::fromStdString(name))) != QMessageBox::Yes) return;
    sendMapCommand(MapCommand::Request::OP_LOAD, name, "", "加载");
}

void MapManagerPanel::renameMap() {
    const std::string name = selectedMap();
    if (name.empty()) { setStatus("先在列表中选中一张地图", true); return; }
    bool ok = false;
    const QString new_name = QInputDialog::getText(this, "重命名地图", "新名称:", QLineEdit::Normal,
                                                   QString::fromStdString(name), &ok).trimmed();
    if (!ok || new_name.isEmpty() || new_name.toStdString() == name) return;
    sendMapCommand(MapCommand::Request::OP_RENAME, name, new_name.toStdString(), "重命名");
}

void MapManagerPanel::trashMap() {
    const std::string name = selectedMap();
    if (name.empty()) { setStatus("先在列表中选中一张地图", true); return; }
    if (QMessageBox::question(this, "移入回收站",
                              QString("把「%1」移入回收站？可在「回收站」页恢复。").arg(QString::fromStdString(name)))
        != QMessageBox::Yes) return;
    sendMapCommand(MapCommand::Request::OP_TRASH, name, "", "移入回收站");
}

void MapManagerPanel::removeMap() {
    const std::string name = selectedMap();
    if (name.empty()) { setStatus("先在列表中选中一张地图", true); return; }
    if (QMessageBox::warning(this, "永久删除",
                             QString("永久删除「%1」？此操作不可恢复（移入回收站可恢复）。").arg(QString::fromStdString(name)),
                             QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) return;
    sendMapCommand(MapCommand::Request::OP_REMOVE, name, "", "删除");
}

void MapManagerPanel::setDisplayYaw() {
    const std::string name = selectedMap();
    if (name.empty()) { setStatus("先在列表中选中一张地图", true); return; }
    auto req = std::make_shared<MapCommand::Request>();
    req->op = MapCommand::Request::OP_SET_DISPLAY_YAW;
    req->map_name = name;
    req->display_yaw = static_cast<float>(yaw_spin_->value() * M_PI / 180.0);
    CallAsync<MapCommand>(this, cmd_cli_, req, [this](MapCommand::Response::SharedPtr r) {
        if (!r) { setStatus("map_command 服务不可用", true); return; }
        setStatus(QString("设置显示朝向%1：%2").arg(r->success ? "成功" : "失败").arg(QString::fromStdString(r->message)),
                  !r->success);
        refreshMaps();
    });
}

std::string MapManagerPanel::targetMap() const {
    const std::string s = selectedMap();
    return s.empty() ? current_map_ : s;
}

void MapManagerPanel::fillVersions() {
    const std::string target = targetMap();
    versions_.clear();
    for (const auto& m : maps_) {
        if (m.name == target) versions_ = m.versions;  // 新的在前
    }
    versions_box_->setTitle(QString("底图版本：%1%2（选中一条回滚：只回滚底图、资产不动）")
                                .arg(target.empty() ? "（未选地图）" : QString::fromStdString(target))
                                .arg(target == current_map_ ? "，当前地图，回滚后重新下发" : "，非当前地图，只恢复文件"));
    static const std::map<std::string, QString> kSource = {
        {"localization_save", "定位保存"}, {"grid_edit", "底图编辑前"}, {"rollback", "回滚前"},
        {"align", "对齐前"}, {"replace", "替换前"}, {"reset_origin", "重设原点前"}};
    versions_table_->setRowCount(static_cast<int>(versions_.size()));
    for (size_t i = 0; i < versions_.size(); ++i) {
        const auto& v = versions_[i];
        const int row = static_cast<int>(i);
        const auto it = kSource.find(v.source);
        versions_table_->setItem(row, 0, Item(QString::fromStdString(v.version_ts)));
        versions_table_->setItem(row, 1, Item(TimeText(v.saved_at)));
        auto* src = Item(it != kSource.end() ? it->second : "未标记（早期版本）");
        src->setToolTip("版本是哪个操作在改动地图前自动存下的（版本目录里的 .source；定位侧保存另带 .complete）。\n"
                        "未标记 = 加来源标记之前产生的版本，来源无法判断，内容可正常回滚");
        versions_table_->setItem(row, 2, src);
        versions_table_->setItem(row, 3, Item(v.complete ? "✓" : "✗ 不可回滚"));
    }
}

void MapManagerPanel::rollbackToVersion() {
    const std::string target = targetMap();
    if (target.empty()) { setStatus("没有选中或加载地图", true); return; }
    const int row = versions_table_->currentRow();
    if (row < 0 || row >= static_cast<int>(versions_.size())) { setStatus("先在版本列表中选中一个版本", true); return; }
    const auto v = versions_[static_cast<size_t>(row)];
    if (!v.complete) { setStatus("该版本三件套不齐，不能回滚", true); return; }
    const bool is_current = (target == current_map_);
    if (QMessageBox::question(this, "回滚底图",
                              QString("把「%1」的底图回滚到版本 %2（%3）？\n"
                                      "资产不动；回滚前的状态会自动存成一个新版本，可再回滚撤销。\n%4")
                                  .arg(QString::fromStdString(target)).arg(QString::fromStdString(v.version_ts))
                                  .arg(TimeText(v.saved_at))
                                  .arg(is_current ? "这是当前地图：回滚后重新下发，整机运行时定位会重新加载地图，需要重新定位。"
                                                  : "这不是当前地图：只恢复文件，不影响正在运行的定位与导航。"))
        != QMessageBox::Yes) return;
    auto req = std::make_shared<RollbackMap::Request>();
    req->map_name = target;
    req->map_version_ts = v.version_ts;
    req->assets_version_ts = "none";  // 只回滚底图，资产保持当前
    setStatus("回滚中 ...");
    CallAsync<RollbackMap>(this, rollback_cli_, req, [this](RollbackMap::Response::SharedPtr r) {
        if (!r) { setStatus("rollback_map 服务不可用，检查 command_board 前缀", true); return; }
        setStatus(QString("回滚%1：%2").arg(r->success ? "成功" : "失败").arg(QString::fromStdString(r->message)),
                  !r->success);
        refreshMaps();
    });
}

// ── 底图编辑页 ──────────────────────────────────────────────────────────

void MapManagerPanel::setEditingUi(bool editing) {
    start_edit_btn_->setEnabled(!editing);
    brush_btn_->setEnabled(editing);
    undo_btn_->setEnabled(editing);
    clear_btn_->setEnabled(editing);
    save_btn_->setEnabled(editing);
    cancel_btn_->setEnabled(editing);
}

void MapManagerPanel::onBrushSettingsChanged() {
    auto& d = GridEditDraft::instance();
    d.tool = static_cast<uint8_t>(tool_group_->checkedId());
    d.mode = static_cast<DrawMode>(mode_combo_->currentData().toInt());
    d.width_m = width_spin_->value();
}

void MapManagerPanel::startEdit() {
    const std::string target = current_map_;
    if (target.empty()) { setStatus("没有加载地图，无法编辑：先在「地图」页加载一张", true); return; }
    const QString frame = getDisplayContext()->getFixedFrame();
    if (frame != "map") {
        setStatus(QString("Fixed Frame 是「%1」，请改为 map 再编辑（笔画按 map 坐标落盘）").arg(frame), true);
        return;
    }
    CallAsync<GetMaps>(this, get_maps_cli_, std::make_shared<GetMaps::Request>(),
                       [this, target](GetMaps::Response::SharedPtr r) {
        if (!r || !r->success) { setStatus("get_maps 失败，无法开始编辑", true); return; }
        std::string revision;
        bool found = false;
        for (const auto& m : r->maps) {
            if (m.name == target) { revision = m.grid_revision; found = true; }
        }
        if (!found) { setStatus("地图不存在：" + QString::fromStdString(target), true); return; }
        current_map_ = r->current_map_name;
        edit_map_ = target;
        edit_revision_ = revision;
        auto& d = GridEditDraft::instance();
        d.clear();
        d.editing = true;
        onBrushSettingsChanged();
        setEditingUi(true);
        reload_btn_->setEnabled(false);
        if (target != current_map_) {  // 取指纹期间地图被切换
            resetEdit("地图已切换，请重新开始编辑");
            return;
        }
        edit_target_label_->setText(QString("编辑对象：当前地图 %1（编辑中）").arg(QString::fromStdString(target)));
        edit_result_label_->clear();
        setStatus("已开始编辑，底图指纹 " + QString::fromStdString(revision));
        activateBrush();
    });
}

void MapManagerPanel::activateBrush() {
    auto* tm = getDisplayContext()->getToolManager();
    for (int i = 0; i < tm->numTools(); ++i) {
        if (tm->getTool(i)->getClassId() == kMapEditToolId) {
            tm->setCurrentTool(tm->getTool(i));
            return;
        }
    }
    if (auto* tool = tm->addTool(kMapEditToolId)) tm->setCurrentTool(tool);
}

void MapManagerPanel::undoStroke() { GridEditDraft::instance().undo(); }

void MapManagerPanel::clearStrokes() { GridEditDraft::instance().clear(); }

void MapManagerPanel::saveEdit() {
    auto& d = GridEditDraft::instance();
    d.endStroke();
    if (d.strokes().empty()) { setStatus("没有待保存的笔画", true); return; }
    if (edit_map_ != current_map_) { resetEdit("当前地图已变化，本轮编辑作废，请重新开始"); return; }
    auto req = std::make_shared<MapCommand::Request>();
    req->op = MapCommand::Request::OP_EDIT_GRID;
    req->map_name = edit_map_;
    req->expected_revision = edit_revision_;
    for (const auto& s : d.strokes()) {
        nav_interfaces::msg::GridStroke gs;
        gs.tool = s.tool;
        gs.shape = s.shape;
        gs.width_m = static_cast<float>(s.width_m);
        for (const auto& p : s.points) {
            geometry_msgs::msg::Point pt;
            pt.x = p.first;
            pt.y = p.second;
            gs.points.push_back(pt);
        }
        req->strokes.push_back(gs);
    }
    save_btn_->setEnabled(false);
    setStatus(QString("保存 %1 笔 ...").arg(req->strokes.size()));
    CallAsync<MapCommand>(this, cmd_cli_, req, [this](MapCommand::Response::SharedPtr r) {
        save_btn_->setEnabled(GridEditDraft::instance().editing);
        if (!r) { setStatus("map_command 服务不可用", true); return; }
        if (!r->success) {
            const bool stale = (r->error_code == nav_interfaces::msg::MapState::E_MAP_GRID_REVISION_MISMATCH);
            setStatus(stale ? "地图已被其他操作修改，本轮修改无法保存：请「取消编辑」后重新开始"
                            : QString("保存失败 [%1] %2").arg(r->error_code).arg(QString::fromStdString(r->message)),
                      true);
            return;
        }
        GridEditDraft::instance().clear();
        if (!r->revision.empty()) edit_revision_ = r->revision;
        reload_btn_->setEnabled(r->requires_reload);
        edit_result_label_->setText(
            QString("已保存：%1\n编辑前的底图存为版本 %2（可在「地图」页版本列表回滚）%3")
                .arg(QString::fromStdString(r->message))
                .arg(r->backup_ts.empty() ? "无（没有像素变化）" : QString::fromStdString(r->backup_ts))
                .arg(r->requires_reload ? "\n这是当前地图，需「重新加载使其生效」导航才会用新底图" : ""));
        setStatus("保存成功");
        refreshMaps();
    });
}

void MapManagerPanel::cancelEdit() {
    auto& d = GridEditDraft::instance();
    if (!d.strokes().empty() &&
        QMessageBox::question(this, "取消编辑", QString("丢弃未保存的 %1 笔？").arg(d.strokes().size()))
            != QMessageBox::Yes) return;
    resetEdit("已退出编辑");
}

void MapManagerPanel::resetEdit(const QString& reason) {
    auto& d = GridEditDraft::instance();
    const bool was_editing = d.editing;
    d.editing = false;
    d.clear();
    edit_map_.clear();
    edit_revision_.clear();
    setEditingUi(false);
    reload_btn_->setEnabled(false);
    edit_result_label_->clear();
    onMapSelectionChanged();
    if (was_editing || !reason.isEmpty()) setStatus(reason, false);
}

void MapManagerPanel::onMapStateChanged(const std::string& name, uint64_t epoch) {
    const bool changed = (name != current_map_) || (epoch != map_epoch_);
    current_map_ = name;
    map_epoch_ = epoch;
    if (!changed) return;
    if (GridEditDraft::instance().editing) {
        // 切换地图 / 重新加载（含回滚后重新下发）后，RViz 显示的底图已换，旧草稿与旧指纹都不再成立
        resetEdit(QString("当前地图已%1（%2），底图编辑已重置，未保存的笔画已丢弃；如需继续请重新「开始编辑」")
                      .arg(edit_map_ == name ? "重新加载" : "切换")
                      .arg(QString::fromStdString(name)));
    }
    onMapSelectionChanged();
    refreshMaps();
}

void MapManagerPanel::reloadEditedMap() {
    const std::string name = edit_map_.empty() ? current_map_ : edit_map_;
    if (QMessageBox::question(this, "重新加载",
                              QString("重新加载「%1」使底图修改生效？\n整机运行时定位会重新加载地图，需要重新定位。")
                                  .arg(QString::fromStdString(name))) != QMessageBox::Yes) return;
    reload_btn_->setEnabled(false);
    sendMapCommand(MapCommand::Request::OP_LOAD, name, "", "重新加载");
}

void MapManagerPanel::onDraftChanged() {
    const auto& d = GridEditDraft::instance();
    draft_label_->setText(QString("待保存 %1 笔%2").arg(d.strokes().size()).arg(d.hasActive() ? "（正在画）" : ""));
    rebuildPreview();
}

// 笔画预览：线段画成宽度等于笔刷直径的色块，端点与折点画圆片，矩形画填充。
void MapManagerPanel::rebuildPreview() {
    if (!preview_obj_) return;
    preview_obj_->clear();
    const auto& d = GridEditDraft::instance();
    std::vector<DraftStroke> items = d.strokes();
    if (d.hasActive()) items.push_back(d.active());
    if (items.empty()) return;

    Ogre::Vector3 pos;
    Ogre::Quaternion ori;
    if (getDisplayContext()->getFrameManager()->getTransform(std::string("map"), pos, ori)) {
        preview_node_->setPosition(pos);
        preview_node_->setOrientation(ori);
    }
    constexpr float z = 0.03f;
    preview_obj_->begin(kPreviewMaterial, Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");
    for (const auto& s : items) {
        Ogre::ColourValue c = s.tool == kToolWall ? Ogre::ColourValue(0.9f, 0.1f, 0.1f, 0.55f)
                            : s.tool == kToolErase ? Ogre::ColourValue(0.1f, 0.85f, 0.2f, 0.55f)
                                                   : Ogre::ColourValue(0.55f, 0.55f, 0.55f, 0.6f);
        auto tri = [&](double ax, double ay, double bx, double by, double cx, double cy) {
            for (const auto& p : {std::make_pair(ax, ay), std::make_pair(bx, by), std::make_pair(cx, cy)}) {
                preview_obj_->position(static_cast<float>(p.first), static_cast<float>(p.second), z);
                preview_obj_->colour(c);
            }
        };
        if (s.shape == kShapeRect && s.points.size() == 2) {
            const auto& a = s.points[0];
            const auto& b = s.points[1];
            tri(a.first, a.second, b.first, a.second, b.first, b.second);
            tri(a.first, a.second, b.first, b.second, a.first, b.second);
            continue;
        }
        const double hw = s.width_m / 2.0;
        for (const auto& p : s.points) {  // 圆片（端点 / 折点 / 单点）
            constexpr int kSeg = 12;
            for (int k = 0; k < kSeg; ++k) {
                const double a0 = 2 * M_PI * k / kSeg, a1 = 2 * M_PI * (k + 1) / kSeg;
                tri(p.first, p.second, p.first + hw * std::cos(a0), p.second + hw * std::sin(a0),
                    p.first + hw * std::cos(a1), p.second + hw * std::sin(a1));
            }
        }
        for (size_t i = 0; i + 1 < s.points.size(); ++i) {  // 线段色块
            const auto& a = s.points[i];
            const auto& b = s.points[i + 1];
            const double dx = b.first - a.first, dy = b.second - a.second;
            const double len = std::hypot(dx, dy);
            if (len < 1e-9) continue;
            const double nx = -dy / len * hw, ny = dx / len * hw;
            tri(a.first + nx, a.second + ny, a.first - nx, a.second - ny, b.first - nx, b.second - ny);
            tri(a.first + nx, a.second + ny, b.first - nx, b.second - ny, b.first + nx, b.second + ny);
        }
    }
    preview_obj_->end();
}

// ── 回收站页 ────────────────────────────────────────────────────────────

void MapManagerPanel::refreshTrash() {
    CallAsync<GetTrashedMaps>(this, trash_cli_, std::make_shared<GetTrashedMaps::Request>(),
                              [this](GetTrashedMaps::Response::SharedPtr r) {
        if (!r) return;  // 回收站页静默：服务不可用时地图页已有提示
        trash_table_->setRowCount(0);
        if (!r->success) return;
        trash_table_->setRowCount(static_cast<int>(r->entries.size()));
        for (size_t i = 0; i < r->entries.size(); ++i) {
            const auto& e = r->entries[i];
            const int row = static_cast<int>(i);
            trash_table_->setItem(row, 0, Item(QString::fromStdString(e.entry_name)));
            trash_table_->setItem(row, 1, Item(QString::fromStdString(e.original_name)));
            trash_table_->setItem(row, 2, Item(TimeText(e.archived_at)));
            trash_table_->setItem(row, 3, Item(QString::number(e.map_info.effective_area_m2, 'f', 2)));
        }
    });
}

void MapManagerPanel::restoreTrash() {
    const int row = trash_table_->currentRow();
    if (row < 0) { setStatus("先在回收站列表中选中一个条目", true); return; }
    const std::string entry = trash_table_->item(row, 0)->text().toStdString();
    const QString original = trash_table_->item(row, 1)->text();
    bool ok = false;
    const QString name = QInputDialog::getText(this, "恢复地图", "恢复为（同名已存在时需改名）:", QLineEdit::Normal,
                                               original, &ok).trimmed();
    if (!ok) return;
    sendMapCommand(MapCommand::Request::OP_RESTORE, entry,
                   name == original ? std::string() : name.toStdString(), "恢复");
}

}  // namespace map_manager_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(map_manager_rviz_plugin::MapManagerPanel, rviz_common::Panel)
