#include "map_manager_rviz_plugin/map_manager_panel.hpp"

#include <QCheckBox>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QImage>
#include <QTextStream>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QInputDialog>
#include <QMessageBox>
#include <QPointer>
#include <QRadioButton>
#include <QTimer>
#include <QVBoxLayout>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <map>
#include <sstream>

#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/visualization_manager.hpp>
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
using AlignMap = nav_interfaces::srv::AlignMap;
using ReplaceMap = nav_interfaces::srv::ReplaceMap;
using ResetXYYaw = nav_interfaces::srv::ResetMapOriginByXYYaw;
using MapPackage = nav_interfaces::srv::MapPackage;

constexpr char kMapEditToolId[] = "map_manager_rviz_plugin/MapEditTool";
constexpr char kPreviewMaterial[] = "MapManagerPanelEditPreview";
// 编辑非当前图时的底图预览（latched）
constexpr char kEditPreviewTopic[] = "/map_manager_panel/edit_target_map";

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

std::string Trim(const std::string& s) {
    const auto b = s.find_first_not_of(" \t\r\"'");
    const auto e = s.find_last_not_of(" \t\r\"'");
    return b == std::string::npos ? std::string() : s.substr(b, e - b + 1);
}

// 读 <dir>/map.yaml + 图像，按 map_manager 发 /map 的同一规则（image_loader TRINARY）转成栅格：
// occ > occupied_thresh → 100；occ < free_thresh → 0；其余灰度 126~128 → −1，其它 → 1 + 98·occ。
bool LoadTrinaryGrid(const QString& yaml, nav_msgs::msg::OccupancyGrid& grid, QString& err) {
    QFile f(yaml);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) { err = "读不到 " + yaml + "（RViz 不在机器人本机？）"; return false; }
    std::string image = "map.png";
    double res = 0.05, ox = 0, oy = 0, oyaw = 0, occ_th = 0.65, free_th = 0.196;
    bool negate = false;
    QTextStream in(&f);
    while (!in.atEnd()) {
        std::string line = in.readLine().toStdString();
        const auto hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        const auto colon = line.find(':');
        if (colon == std::string::npos) continue;
        const std::string key = Trim(line.substr(0, colon)), val = Trim(line.substr(colon + 1));
        try {
            if (key == "image") image = val;
            else if (key == "resolution") res = std::stod(val);
            else if (key == "negate") negate = (val == "1" || val == "true");
            else if (key == "occupied_thresh") occ_th = std::stod(val);
            else if (key == "free_thresh") free_th = std::stod(val);
            else if (key == "origin") {
                std::string v = val;
                for (auto& c : v) if (c == '[' || c == ']' || c == ',') c = ' ';
                std::istringstream ss(v);
                ss >> ox >> oy >> oyaw;
            }
        } catch (const std::exception&) {
        }
    }
    const QString png = QFileInfo(yaml).dir().filePath(QString::fromStdString(image));
    QImage img(png);
    if (img.isNull()) { err = "读不到 " + png; return false; }
    img = img.convertToFormat(QImage::Format_Grayscale8);
    grid.header.frame_id = "map";
    grid.info.resolution = static_cast<float>(res);
    grid.info.width = img.width();
    grid.info.height = img.height();
    grid.info.origin.position.x = ox;
    grid.info.origin.position.y = oy;
    grid.info.origin.orientation.z = std::sin(oyaw / 2.0);
    grid.info.origin.orientation.w = std::cos(oyaw / 2.0);
    grid.data.assign(static_cast<size_t>(img.width()) * img.height(), -1);
    for (int r = 0; r < img.height(); ++r) {
        const uchar* line = img.constScanLine(r);
        for (int c = 0; c < img.width(); ++c) {
            const double p = negate ? 255.0 - line[c] : line[c];
            const double occ = (255.0 - p) / 255.0;
            int8_t v;
            if (occ > occ_th) v = 100;
            else if (occ < free_th) v = 0;
            else if (p >= 126.0 && p <= 128.0) v = -1;
            else v = static_cast<int8_t>(static_cast<int>(1 + 98 * occ));
            grid.data[static_cast<size_t>(img.height() - 1 - r) * img.width() + c] = v;
        }
    }
    return true;
}

}  // namespace

// ── 构造与界面 ──────────────────────────────────────────────────────────

MapManagerPanel::MapManagerPanel(QWidget* parent) : rviz_common::Panel(parent) {
    auto* layout = new QVBoxLayout(this);

    auto* prefix_row = new QHBoxLayout();
    mm_prefix_edit_ = new QLineEdit("/map_manager");
    cb_prefix_edit_ = new QLineEdit("/command_board");
    mm_prefix_edit_->setToolTip("map_manager 节点名（get_maps / map_command / get_trashed_maps）");
    cb_prefix_edit_->setToolTip("command_board 节点名（对齐 / 替换 / 按坐标重设原点）；单独 ros2 run 起的默认名是 /command_board_manager");
    prefix_row->addWidget(new QLabel("map_manager:"));
    prefix_row->addWidget(mm_prefix_edit_);
    prefix_row->addWidget(new QLabel("command_board:"));
    prefix_row->addWidget(cb_prefix_edit_);
    layout->addLayout(prefix_row);

    tabs_ = new QTabWidget();
    tabs_->addTab(buildMapsTab(), "地图");
    tabs_->addTab(buildEditTab(), "底图编辑");
    tabs_->addTab(buildMapOpsTab(), "对齐 / 替换 / 原点");
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
    // 可多选（导出 / 导入用）；其它按钮仍作用于当前行
    maps_table_->setSelectionMode(QAbstractItemView::ExtendedSelection);
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
    auto* pkg_row = new QHBoxLayout();
    export_btn_ = new QPushButton("导出…");
    export_btn_->setToolTip("把选中的地图（可多选）各打成一个 zip 包，复制到本机目录");
    import_btn_ = new QPushButton("导入…");
    import_btn_->setToolTip("从本机选一个或多个地图包导入（不自动加载）");
    connect(export_btn_, &QPushButton::clicked, this, &MapManagerPanel::exportMaps);
    connect(import_btn_, &QPushButton::clicked, this, &MapManagerPanel::importMaps);
    pkg_row->addWidget(export_btn_);
    pkg_row->addWidget(import_btn_);
    pkg_row->addStretch();
    v->addLayout(pkg_row);
    auto* align_hint = new QLabel("地图对齐、地图替换、按坐标重设原点在「对齐 / 替换 / 原点」页，作用于这里选中的地图；"
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

QWidget* MapManagerPanel::buildMapOpsTab() {
    auto* w = new QWidget();
    auto* v = new QVBoxLayout(w);
    ops_target_label_ = new QLabel();
    ops_target_label_->setWordWrap(true);
    v->addWidget(ops_target_label_);
    cb_state_label_ = new QLabel();
    cb_state_label_->setWordWrap(true);
    v->addWidget(cb_state_label_);
    auto cb_button = [this](const QString& text, void (MapManagerPanel::*slot)()) {
        auto* b = new QPushButton(text);
        connect(b, &QPushButton::clicked, this, slot);
        cb_buttons_.push_back(b);
        return b;
    };

    // 对齐：按墙面方向估角，底图与资产同角度旋转，失败联合回滚
    auto* align_box = new QGroupBox("地图对齐（把墙面摆正）");
    auto* align_row = new QHBoxLayout(align_box);
    align_row->addWidget(cb_button("试算（只估角，不改文件）", &MapManagerPanel::alignDryRun));
    align_row->addWidget(cb_button("执行对齐", &MapManagerPanel::alignApply));
    v->addWidget(align_box);

    // 替换：用源图内容替换目标底图，坐标系沿用目标，资产不动
    auto* rep_box = new QGroupBox("地图替换（用重新建的图替换选中地图的底图，坐标系与资产保持不变）");
    auto* rep_v = new QVBoxLayout(rep_box);
    auto* rep_row1 = new QHBoxLayout();
    source_combo_ = new QComboBox();
    source_combo_->setToolTip("提供新内容的地图（整场重走新建的图），不能与选中地图相同");
    keep_source_check_ = new QCheckBox("保留源图");
    accept_degraded_check_ = new QCheckBox("未收敛也执行");
    rep_row1->addWidget(new QLabel("源图:"));
    rep_row1->addWidget(source_combo_, 1);
    rep_row1->addWidget(keep_source_check_);
    rep_row1->addWidget(accept_degraded_check_);
    rep_v->addLayout(rep_row1);
    auto* rep_row2 = new QHBoxLayout();
    manual_check_ = new QCheckBox("手动变换");
    manual_check_->setToolTip("不做自动匹配，直接用给定的 源图原点在目标坐标系中的 x / y / yaw");
    manual_x_edit_ = new QLineEdit();
    manual_x_edit_->setPlaceholderText("x");
    manual_y_edit_ = new QLineEdit();
    manual_y_edit_->setPlaceholderText("y");
    manual_yaw_edit_ = new QLineEdit();
    manual_yaw_edit_->setPlaceholderText("yaw(rad)");
    rep_row2->addWidget(manual_check_);
    rep_row2->addWidget(manual_x_edit_);
    rep_row2->addWidget(manual_y_edit_);
    rep_row2->addWidget(manual_yaw_edit_);
    rep_v->addLayout(rep_row2);
    auto* rep_row3 = new QHBoxLayout();
    rep_row3->addWidget(cb_button("试算（只匹配，不改文件）", &MapManagerPanel::replaceDryRun));
    rep_row3->addWidget(cb_button("执行替换", &MapManagerPanel::replaceApply));
    rep_v->addLayout(rep_row3);
    v->addWidget(rep_box);

    // 按坐标重设原点：底图与资产一起变换
    auto* reset_box = new QGroupBox("按坐标重设原点（新原点在旧坐标系中的位姿）");
    auto* reset_row = new QHBoxLayout(reset_box);
    reset_x_edit_ = new QLineEdit();
    reset_x_edit_->setPlaceholderText("x");
    reset_y_edit_ = new QLineEdit();
    reset_y_edit_->setPlaceholderText("y");
    reset_yaw_edit_ = new QLineEdit();
    reset_yaw_edit_->setPlaceholderText("yaw(rad)");
    reset_row->addWidget(reset_x_edit_);
    reset_row->addWidget(reset_y_edit_);
    reset_row->addWidget(reset_yaw_edit_);
    reset_row->addWidget(cb_button("执行重设", &MapManagerPanel::resetOriginByXYYaw));
    v->addWidget(reset_box);

    auto* hint = new QLabel("三项都会在改动前自动存底图版本，可在「地图」页版本列表回滚（资产版本在 Map Asset Manager 面板回滚）。"
                            "按机器人当前位姿重设原点依赖实时定位，在 Command Board 面板操作。");
    hint->setWordWrap(true);
    hint->setStyleSheet("color: gray;");
    v->addWidget(hint);
    ops_result_label_ = new QLabel();
    ops_result_label_->setWordWrap(true);
    v->addWidget(ops_result_label_);
    v->addStretch();
    return w;
}

QWidget* MapManagerPanel::buildEditTab() {
    auto* w = new QWidget();
    auto* v = new QVBoxLayout(w);

    auto* target_row = new QHBoxLayout();
    target_row->addWidget(new QLabel("编辑对象:"));
    edit_target_combo_ = new QComboBox();
    edit_target_combo_->setToolTip("与「地图」页选中的地图同步；编辑中更换会结束本次编辑");
    target_row->addWidget(edit_target_combo_, 1);
    v->addLayout(target_row);
    // 只响应用户操作（activated），程序内重填下拉框不触发
    connect(edit_target_combo_, QOverload<int>::of(&QComboBox::activated), this, [this](int idx) {
        selectMapInTable(edit_target_combo_->itemData(idx).toString().toStdString());
    });
    edit_target_label_ = new QLabel();
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
                            "每种工具一笔到位：墙体覆盖处全变障碍；擦除把障碍与灰色噪点变为可通行，未知区域保持未知；"
                            "未知区域覆盖处全变未知。保存前可随时撤销；保存后可在「地图」页回滚。"));
    v->addStretch();
    setEditingUi(false);
    reload_btn_->setEnabled(false);
    reload_btn_->setToolTip("当前地图文件里有已保存、但导航还没用上的修改时可用；退出编辑后仍然有效");
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
    edit_preview_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
        kEditPreviewTopic, rclcpp::QoS(1).transient_local().reliable());

    cb_timer_ = new QTimer(this);
    connect(cb_timer_, &QTimer::timeout, this, &MapManagerPanel::refreshCommandBoardState);
    cb_timer_->start(1000);
    refreshCommandBoardState();
    refreshMaps();
}

void MapManagerPanel::recreateClients() {
    if (!node_) return;
    const std::string mm = mm_prefix_edit_->text().toStdString();
    const std::string cb = cb_prefix_edit_->text().toStdString();
    get_maps_cli_ = node_->create_client<GetMaps>(mm + "/get_maps");
    cmd_cli_ = node_->create_client<MapCommand>(mm + "/map_command");
    trash_cli_ = node_->create_client<GetTrashedMaps>(mm + "/get_trashed_maps");
    align_cli_ = node_->create_client<AlignMap>(cb + "/align_map");
    replace_cli_ = node_->create_client<ReplaceMap>(cb + "/replace_map");
    reset_xyyaw_cli_ = node_->create_client<ResetXYYaw>(cb + "/reset_map_origin_by_xyyaw");
    package_cli_ = node_->create_client<MapPackage>(mm + "/map_package");
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

// ── 导出 / 导入（~/map_package）────────────────────────────────────────
// 地图表可多选；导出 = 每张图一个包（不外套一层），导入 = 逐个包导入。请求一个接一个发：
// 节点里本来就串行处理，并发只会排队。复制文件用 std::filesystem（UTF-8 字节路径），
// RViz 跑在 C locale 下中文路径也能用。

std::vector<std::string> MapManagerPanel::selectedMaps() const {
    std::vector<int> rows;
    for (const auto& idx : maps_table_->selectionModel()->selectedRows()) rows.push_back(idx.row());
    std::sort(rows.begin(), rows.end());
    std::vector<std::string> out;
    for (const int r : rows) {
        if (r >= 0 && r < static_cast<int>(maps_.size())) out.push_back(maps_[static_cast<size_t>(r)].name);
    }
    return out;
}

void MapManagerPanel::exportMaps() {
    const auto names = selectedMaps();
    if (names.empty()) { setStatus("先在列表中选中要导出的地图（Ctrl / Shift 可多选）", true); return; }
    const QString dir = QFileDialog::getExistingDirectory(
        this, QString("把 %1 张地图导出到").arg(names.size()), package_last_dir_.isEmpty() ? QDir::homePath() : package_last_dir_);
    if (dir.isEmpty()) return;
    QMessageBox box(QMessageBox::Question, "导出地图",
                    QString("导出 %1 张地图，每张一个 zip 包，保存到：\n%2").arg(names.size()).arg(dir),
                    QMessageBox::Ok | QMessageBox::Cancel, this);
    auto* full = new QCheckBox("含历史版本与附带文件（排障用，包更大）");
    box.setCheckBox(full);
    if (box.exec() != QMessageBox::Ok) return;
    package_last_dir_ = dir;
    package_dest_dir_ = dir.toStdString();
    package_content_ = full->isChecked() ? MapPackage::Request::CONTENT_FULL : MapPackage::Request::CONTENT_STANDARD;
    std::vector<PackageJob> jobs;
    for (const auto& n : names) jobs.push_back({true, n, ""});
    startPackageJobs(std::move(jobs));
}

void MapManagerPanel::importMaps() {
    if (map_folder_.empty()) { setStatus("还没拿到地图目录，先点「刷新」", true); return; }
    std::error_code ec;
    if (!std::filesystem::is_directory(map_folder_, ec)) {
        setStatus(QString("本机没有地图目录 %1：RViz 与 map_manager 不在同一台机器，不能从本机导入")
                      .arg(QString::fromStdString(map_folder_)), true);
        return;
    }
    const QStringList files = QFileDialog::getOpenFileNames(
        this, "选择要导入的地图包（可多选）", package_last_dir_.isEmpty() ? QDir::homePath() : package_last_dir_,
        "地图包 (*.zip)");
    if (files.isEmpty()) return;
    const QStringList policies = {"自动改名（重名时加 _copy1、_copy2 …）", "重名时拒绝", "覆盖（旧图移入回收站）"};
    bool ok = false;
    const QString policy = QInputDialog::getItem(this, "导入地图", "已有同名地图时：", policies, 0, false, &ok);
    if (!ok) return;
    package_conflict_ = static_cast<uint8_t>(policies.indexOf(policy));
    if (package_conflict_ == MapPackage::Request::ON_CONFLICT_OVERWRITE &&
        QMessageBox::warning(this, "覆盖导入",
                             "同名的旧图会整张移入回收站（可在「回收站」页恢复）；当前已加载的地图不能被覆盖。继续？",
                             QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) {
        return;
    }
    package_last_dir_ = QFileInfo(files.front()).absolutePath();
    std::vector<PackageJob> jobs;
    for (const auto& f : files) jobs.push_back({false, "", f.toStdString()});
    startPackageJobs(std::move(jobs));
}

void MapManagerPanel::startPackageJobs(std::vector<PackageJob> jobs) {
    package_jobs_ = std::move(jobs);
    package_index_ = 0;
    package_ok_ = 0;
    package_report_.clear();
    package_last_imported_.clear();
    export_btn_->setEnabled(false);
    import_btn_->setEnabled(false);
    runNextPackageJob();
}

void MapManagerPanel::runNextPackageJob() {
    namespace fs = std::filesystem;
    if (package_index_ >= package_jobs_.size()) {
        export_btn_->setEnabled(true);
        import_btn_->setEnabled(true);
        const bool is_export = !package_jobs_.empty() && package_jobs_.front().is_export;
        const int total = static_cast<int>(package_jobs_.size());
        const QString title = QString("%1完成：成功 %2 / 共 %3").arg(is_export ? "导出" : "导入").arg(package_ok_).arg(total);
        setStatus(title, package_ok_ != total);
        QMessageBox::information(this, is_export ? "导出地图" : "导入地图", title + "\n\n" + package_report_.join("\n"));
        if (!is_export) {
            refreshMaps();
            if (!package_last_imported_.empty()) {
                const std::string name = package_last_imported_;
                QTimer::singleShot(800, this, [this, name]() { selectMapInTable(name); });
            }
        }
        return;
    }
    const PackageJob& job = package_jobs_[package_index_];
    auto req = std::make_shared<MapPackage::Request>();
    QString label;
    if (job.is_export) {
        req->op = MapPackage::Request::OP_EXPORT;
        req->map_name = job.map_name;
        req->content = package_content_;
        label = QString::fromStdString(job.map_name);
    } else {
        // 先把本机的包复制进 <map_folder>/.imports/，服务只接受那里的文件
        const fs::path src(job.file);
        label = QString::fromStdString(src.filename().string());
        const fs::path imports = fs::path(map_folder_) / ".imports";
        std::error_code ec;
        fs::create_directories(imports, ec);
        fs::path dst = imports / src.filename();
        for (int n = 2; fs::exists(dst, ec); ++n) {
            dst = imports / (src.stem().string() + "_" + std::to_string(n) + src.extension().string());
        }
        fs::copy_file(src, dst, ec);
        if (ec) {
            finishPackageJob(QString("❌ %1：复制到 %2 失败：%3").arg(label)
                                 .arg(QString::fromStdString(imports.string()))
                                 .arg(QString::fromStdString(ec.message())), false);
            return;
        }
        req->op = MapPackage::Request::OP_IMPORT;
        req->file_path = dst.string();
        req->on_conflict = package_conflict_;
    }
    setStatus(QString("正在%1 %2/%3：%4").arg(job.is_export ? "导出" : "导入").arg(package_index_ + 1)
                  .arg(package_jobs_.size()).arg(label));
    const uint64_t seq = ++package_seq_;
    // 节点中途退出时响应永远不会来：超时就记失败、继续下一个，迟到的响应按 seq 丢弃
    QTimer::singleShot(180000, this, [this, seq, label]() {
        if (seq != package_seq_ || package_index_ >= package_jobs_.size()) return;
        ++package_seq_;
        finishPackageJob(QString("❌ %1：等待 map_manager 超时（180 s）").arg(label), false);
    });
    const bool is_export = job.is_export;
    CallAsync<MapPackage>(this, package_cli_, req, [this, seq, label, is_export](MapPackage::Response::SharedPtr r) {
        if (seq != package_seq_) return;
        ++package_seq_;
        if (!r) { finishPackageJob(QString("❌ %1：map_package 服务不可用（map_manager 没启动？）").arg(label), false); return; }
        if (!r->success) {
            finishPackageJob(QString("❌ %1：[%2] %3").arg(label).arg(r->error_code).arg(QString::fromStdString(r->message)),
                             false);
            return;
        }
        if (!is_export) {
            package_last_imported_ = r->map_name;
            QString line = QString("✅ %1 → 地图「%2」").arg(label).arg(QString::fromStdString(r->map_name));
            if (!r->trashed_entry.empty()) {
                line += QString("（旧图已移入回收站：%1）").arg(QString::fromStdString(r->trashed_entry));
            }
            finishPackageJob(line, true);
            return;
        }
        namespace fs = std::filesystem;
        const fs::path src(r->file_path);
        const fs::path dst = fs::path(package_dest_dir_) / src.filename();
        std::error_code ec;
        if (!fs::exists(src, ec)) {
            // RViz 不在机器人上：包留在机器人里，给出路径
            finishPackageJob(QString("✅ %1：包在机器人上（本机读不到）：%2").arg(label)
                                 .arg(QString::fromStdString(r->file_path)), true);
            return;
        }
        fs::copy_file(src, dst, fs::copy_options::overwrite_existing, ec);
        if (ec) {
            finishPackageJob(QString("❌ %1：已打包但复制到所选目录失败：%2（包在 %3）").arg(label)
                                 .arg(QString::fromStdString(ec.message())).arg(QString::fromStdString(r->file_path)),
                             false);
            return;
        }
        finishPackageJob(QString("✅ %1 → %2（%3 KB）").arg(label).arg(QString::fromStdString(dst.string()))
                             .arg(r->size_bytes / 1024), true);
    });
}

void MapManagerPanel::finishPackageJob(const QString& line, bool ok) {
    package_report_ << line;
    if (ok) ++package_ok_;
    ++package_index_;
    runNextPackageJob();
}

void MapManagerPanel::refreshMaps() {
    CallAsync<GetMaps>(this, get_maps_cli_, std::make_shared<GetMaps::Request>(),
                       [this](GetMaps::Response::SharedPtr r) {
        if (!r) { setStatus("get_maps 服务不可用，检查 map_manager 前缀", true); return; }
        if (!r->success) { setStatus(QString("get_maps 失败 [%1]: %2").arg(r->error_code).arg(QString::fromStdString(r->message)), true); return; }
        const std::string keep = selectedMap();
        maps_ = r->maps;
        current_map_ = r->current_map_name;
        map_folder_ = r->map_folder;
        syncing_selection_ = true;
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
        syncing_selection_ = false;
        setStatus(QString("%1 张地图，当前：%2").arg(maps_.size())
                      .arg(current_map_.empty() ? "（未加载）" : QString::fromStdString(current_map_)));
        fillVersions();
        fillMapOpsTarget();
        fillEditTarget();
        updateReloadState();
    });
}

void MapManagerPanel::onMapSelectionChanged() {
    if (!syncing_selection_ && !confirmTargetChange(targetMap())) {
        // 用户取消：选择退回正在编辑的图
        syncing_selection_ = true;
        selectMapInTable(edit_map_);
        syncing_selection_ = false;
    }
    const int row = maps_table_->currentRow();
    if (row >= 0 && row < static_cast<int>(maps_.size())) {
        yaw_spin_->setValue(maps_[static_cast<size_t>(row)].display_yaw * 180.0 / M_PI);
    }
    fillVersions();
    fillMapOpsTarget();
    fillEditTarget();
}

void MapManagerPanel::fillEditTarget() {
    const bool editing = GridEditDraft::instance().editing;
    const std::string target = editing ? edit_map_ : targetMap();
    edit_target_combo_->blockSignals(true);
    edit_target_combo_->clear();
    for (const auto& m : maps_) {
        const QString name = QString::fromStdString(m.name);
        edit_target_combo_->addItem(m.name == current_map_ ? name + "（当前）" : name, name);
    }
    edit_target_combo_->setCurrentIndex(edit_target_combo_->findData(QString::fromStdString(target)));
    edit_target_combo_->blockSignals(false);
    if (target.empty()) {
        edit_target_label_->setText("未选地图：在上面的下拉框或「地图」页选中一张");
    } else if (target == current_map_) {
        edit_target_label_->setText(editing ? "当前地图，编辑中" : "当前地图；保存后需「重新加载使其生效」");
    } else {
        edit_target_label_->setText(editing ? "未加载地图，编辑中：RViz 显示的是它的预览，当前地图的显示已暂时隐藏"
                                            : "未加载地图；编辑时显示它的预览，保存只改文件，加载这张图时生效");
    }
}

void MapManagerPanel::selectMapInTable(const std::string& name) {
    for (size_t i = 0; i < maps_.size(); ++i) {
        if (maps_[i].name == name) { maps_table_->selectRow(static_cast<int>(i)); return; }
    }
    maps_table_->clearSelection();
    maps_table_->setCurrentCell(-1, -1);
}

bool MapManagerPanel::confirmTargetChange(const std::string& target) {
    const auto& d = GridEditDraft::instance();
    if (!d.editing || target == edit_map_) return true;
    if (!d.strokes().empty() &&
        QMessageBox::question(this, "更换编辑对象",
                              QString("正在编辑「%1」，还有 %2 笔未保存。\n更换编辑对象会结束本次编辑并丢弃这些笔画，继续？")
                                  .arg(QString::fromStdString(edit_map_)).arg(d.strokes().size()))
            != QMessageBox::Yes) {
        return false;
    }
    syncing_selection_ = true;  // resetEdit 内部会刷新选择相关界面，避免重入
    resetEdit(QString("编辑对象已改为「%1」，本次编辑结束；如需编辑请重新「开始编辑」").arg(QString::fromStdString(target)));
    syncing_selection_ = false;
    return true;
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
    // 只回滚底图：map_manager 恢复文件（回滚前自动存版本）；是当前地图时再重新加载，下游按 epoch 重载
    auto req = std::make_shared<MapCommand::Request>();
    req->op = MapCommand::Request::OP_ROLLBACK;
    req->map_name = target;
    req->version_ts = v.version_ts;
    setStatus("回滚中 ...");
    CallAsync<MapCommand>(this, cmd_cli_, req, [this, target](MapCommand::Response::SharedPtr r) {
        if (!r) { setStatus("map_command 服务不可用", true); return; }
        if (!r->success) {
            setStatus(QString("回滚失败 [%1] %2").arg(r->error_code).arg(QString::fromStdString(r->message)), true);
            refreshMaps();
            return;
        }
        const QString saved = r->backup_ts.empty() ? QString("无") : QString::fromStdString(r->backup_ts);
        if (r->requires_reload) {
            sendMapCommand(MapCommand::Request::OP_LOAD, target, "",
                           QString("回滚完成（回滚前的底图存为版本 %1），重新加载").arg(saved));
            return;
        }
        setStatus(QString("回滚成功，回滚前的底图存为版本 %1（非当前地图，只恢复文件）").arg(saved));
        refreshMaps();
    });
}

// ── 对齐 / 替换 / 原点页 ────────────────────────────────────────────────

void MapManagerPanel::fillMapOpsTarget() {
    const std::string target = targetMap();
    ops_target_label_->setText(
        QString("目标地图：%1%2（在「地图」页选中，未选 = 当前地图）")
            .arg(target.empty() ? "（未选地图）" : QString::fromStdString(target))
            .arg(target.empty() ? "" : (target == current_map_ ? "，当前地图：会停导航与定位，完成后重新加载，需要重新定位"
                                                              : "，非当前地图：只改文件")));
    const QString keep = source_combo_->currentText();
    source_combo_->clear();
    for (const auto& m : maps_) {
        if (m.name != target) source_combo_->addItem(QString::fromStdString(m.name));
    }
    const int idx = source_combo_->findText(keep);
    if (idx >= 0) source_combo_->setCurrentIndex(idx);
}

void MapManagerPanel::refreshCommandBoardState() {
    const bool ok = align_cli_ && align_cli_->service_is_ready();
    for (auto* b : cb_buttons_) b->setEnabled(ok);
    cb_state_label_->setText(ok ? QString("command_board：在线")
                                : QString("command_board：未在线（检查上方前缀）——对齐 / 替换 / 重设原点要由它编排，暂不可用；"
                                          "本面板其余功能不受影响"));
    cb_state_label_->setStyleSheet(ok ? "color: green;" : "color: #c0392b;");
}

void MapManagerPanel::setOpsResult(const QString& text, bool error) {
    ops_result_label_->setStyleSheet(error ? "color: #c0392b;" : "color: green;");
    ops_result_label_->setText(text);
    setStatus(text, error);
}

void MapManagerPanel::alignDryRun() { callAlign(true); }

void MapManagerPanel::alignApply() {
    const std::string target = targetMap();
    if (target.empty()) { setOpsResult("先在「地图」页选中一张地图", true); return; }
    if (QMessageBox::question(this, "确认对齐",
                              QString("对齐「%1」：按墙面方向估角，底图与资产同角度旋转，失败联合回滚。\n"
                                      "改动前自动存底图版本（对齐前）。\n%2")
                                  .arg(QString::fromStdString(target))
                                  .arg(target == current_map_ ? "这是当前地图：会停导航与定位，完成后重新加载，需要重新定位。"
                                                              : "这不是当前地图：只改文件。"))
        != QMessageBox::Yes) return;
    callAlign(false);
}

void MapManagerPanel::callAlign(bool dry_run) {
    const std::string target = targetMap();
    if (target.empty()) { setOpsResult("先在「地图」页选中一张地图", true); return; }
    auto req = std::make_shared<AlignMap::Request>();
    req->map_name = target;
    req->dry_run = dry_run;
    const QString what = dry_run ? "对齐试算" : "对齐";
    setOpsResult(what + " 调用中 ...", false);
    CallAsync<AlignMap>(this, align_cli_, req, [this, what, dry_run](AlignMap::Response::SharedPtr r) {
        if (!r) { setOpsResult("align_map 服务不可用，检查 command_board 前缀", true); return; }
        std::ostringstream s;
        s << std::fixed << std::setprecision(2) << "估计对齐角 " << r->estimated_yaw_deg << "°，正交误差 "
          << r->orthogonality_error_deg << "°";
        if (r->applied) {
            s << "，执行后残余 " << r->residual_yaw_deg << "°，资产点 / 区域 / 路网节点 " << r->transformed_points << " / "
              << r->transformed_areas << " / " << r->transformed_junctions;
        }
        s << "；" << r->message;
        setOpsResult(QString("%1%2：%3").arg(what).arg(r->success ? "成功" : "失败").arg(QString::fromStdString(s.str())),
                     !r->success);
        if (!dry_run) refreshMaps();
    });
}

void MapManagerPanel::replaceDryRun() { callReplace(true); }

void MapManagerPanel::replaceApply() {
    const std::string target = targetMap();
    const std::string source = source_combo_->currentText().toStdString();
    if (target.empty() || source.empty()) { setOpsResult("先选中目标地图，并选择源图", true); return; }
    if (QMessageBox::question(this, "确认替换",
                              QString("用「%1」替换「%2」的底图（坐标系沿用目标，资产不动）。\n改动前自动存底图版本（替换前）。\n%3%4")
                                  .arg(QString::fromStdString(source)).arg(QString::fromStdString(target))
                                  .arg(keep_source_check_->isChecked() ? "" : "完成后源图移入回收站。\n")
                                  .arg(target == current_map_ ? "目标是当前地图：会停导航与定位，完成后重新加载，需要重新定位。" : ""))
        != QMessageBox::Yes) return;
    callReplace(false);
}

void MapManagerPanel::callReplace(bool dry_run) {
    const std::string target = targetMap();
    const std::string source = source_combo_->currentText().toStdString();
    if (target.empty() || source.empty()) { setOpsResult("先选中目标地图，并选择源图", true); return; }
    auto req = std::make_shared<ReplaceMap::Request>();
    req->target_map_name = target;
    req->source_map_name = source;
    req->dry_run = dry_run;
    req->keep_source = keep_source_check_->isChecked();
    req->accept_degraded = accept_degraded_check_->isChecked();
    if (manual_check_->isChecked()) {
        bool okx = false, oky = false, okyaw = false;
        req->use_manual_transform = true;
        req->manual_x = manual_x_edit_->text().toDouble(&okx);
        req->manual_y = manual_y_edit_->text().toDouble(&oky);
        req->manual_yaw = manual_yaw_edit_->text().toDouble(&okyaw);
        if (!okx || !oky || !okyaw) { setOpsResult("手动变换的 x / y / yaw 需为数字", true); return; }
    }
    const QString what = dry_run ? "替换试算" : "替换";
    setOpsResult(what + " 调用中 ...", false);
    CallAsync<ReplaceMap>(this, replace_cli_, req, [this, what, dry_run](ReplaceMap::Response::SharedPtr r) {
        if (!r) { setOpsResult("replace_map 服务不可用，检查 command_board 前缀", true); return; }
        std::ostringstream s;
        s << std::fixed << std::setprecision(3) << "变换 x=" << r->x << " y=" << r->y << " yaw=" << std::setprecision(2)
          << r->yaw_deg << "°，内点 " << r->inliers << "，"
          << (r->converged ? "已收敛" : (r->degraded ? "未收敛（质量存疑）" : "未收敛"));
        if (r->coverage_warning) s << "，源图覆盖不足目标的 80%";
        if (!r->source_trashed_to.empty()) s << "，源图已移入回收站";
        s << "；" << r->message;
        setOpsResult(QString("%1%2：%3").arg(what).arg(r->success ? "成功" : "失败").arg(QString::fromStdString(s.str())),
                     !r->success);
        if (!dry_run) { refreshMaps(); refreshTrash(); }
    });
}

void MapManagerPanel::resetOriginByXYYaw() {
    const std::string target = targetMap();
    if (target.empty()) { setOpsResult("先在「地图」页选中一张地图", true); return; }
    bool okx = false, oky = false, okyaw = false;
    const double x = reset_x_edit_->text().toDouble(&okx);
    const double y = reset_y_edit_->text().toDouble(&oky);
    const double yaw = reset_yaw_edit_->text().toDouble(&okyaw);
    if (!okx || !oky || !okyaw) { setOpsResult("x / y / yaw 需为数字", true); return; }
    if (QMessageBox::question(this, "确认重设原点",
                              QString("把「%1」的原点设为 x=%2  y=%3  yaw=%4 ？\n底图与资产一起变换，改动前自动存底图版本（重设原点前）。\n%5")
                                  .arg(QString::fromStdString(target)).arg(x).arg(y).arg(yaw)
                                  .arg(target == current_map_ ? "这是当前地图：会停导航与定位，完成后重新加载，需要重新定位。"
                                                              : "这不是当前地图：只改文件。"))
        != QMessageBox::Yes) return;
    auto req = std::make_shared<ResetXYYaw::Request>();
    req->map_name = target;
    req->x = x;
    req->y = y;
    req->yaw = yaw;
    setOpsResult("重设原点 调用中 ...", false);
    CallAsync<ResetXYYaw>(this, reset_xyyaw_cli_, req, [this](ResetXYYaw::Response::SharedPtr r) {
        if (!r) { setOpsResult("reset_map_origin_by_xyyaw 服务不可用，检查 command_board 前缀", true); return; }
        setOpsResult(QString("重设原点%1：%2").arg(r->success ? "成功" : "失败").arg(QString::fromStdString(r->message)),
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
    const std::string target = targetMap();
    if (target.empty()) { setStatus("没有可编辑的地图：先在「地图」页选中一张", true); return; }
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
        map_folder_ = r->map_folder;
        const bool is_current = (target == current_map_);
        if (!is_current) {
            QString err;
            if (!publishEditPreview(target, err)) { setStatus("无法显示要编辑的地图：" + err, true); return; }
        }
        edit_map_ = target;
        edit_revision_ = revision;
        edit_is_current_ = is_current;
        auto& d = GridEditDraft::instance();
        d.clear();
        d.editing = true;
        onBrushSettingsChanged();
        setEditingUi(true);
        if (!is_current) applyEditFocus(true);
        fillEditTarget();
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
    if (edit_is_current_ != (edit_map_ == current_map_)) {
        resetEdit("编辑对象是否为当前地图已变化，本轮编辑作废，请重新开始");
        return;
    }
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
        if (!edit_is_current_) {
            QString err;
            if (!publishEditPreview(edit_map_, err)) setStatus("刷新预览失败：" + err, true);
        }
        edit_result_label_->setText(
            QString("已保存：%1\n编辑前的底图存为版本 %2（可在「地图」页版本列表回滚）%3")
                .arg(QString::fromStdString(r->message))
                .arg(r->backup_ts.empty() ? "无（没有像素变化）" : QString::fromStdString(r->backup_ts))
                .arg(r->requires_reload ? "\n这是当前地图，需「重新加载使其生效」导航才会用新底图"
                                        : (edit_is_current_ ? "" : "\n未加载地图：只改了文件，加载这张图时生效")));
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
    if (!edit_is_current_) {
        clearEditPreview();
        applyEditFocus(false);
    }
    edit_map_.clear();
    edit_revision_.clear();
    edit_is_current_ = true;
    setEditingUi(false);
    edit_result_label_->setText(reason);  // 状态栏随后会被地图列表刷新覆盖，原因留在本页
    onMapSelectionChanged();
    if (was_editing || !reason.isEmpty()) setStatus(reason, false);
}

void MapManagerPanel::onMapStateChanged(const std::string& name, uint64_t epoch) {
    const bool changed = (name != current_map_) || (epoch != map_epoch_);
    current_map_ = name;
    map_epoch_ = epoch;
    if (!changed) return;
    if (GridEditDraft::instance().editing) {
        // 编辑当前地图：切换或重新加载都结束本次编辑（重新加载 = 本轮修改已生效，下次重新开始）。
        // 编辑未加载地图：只有它被加载成当前地图时结束（显示换成了它本身）；别的图切换 / 重载与它无关
        const size_t unsaved = GridEditDraft::instance().strokes().size();
        const QString dropped = unsaved > 0 ? QString("，未保存的 %1 笔已丢弃").arg(unsaved) : QString();
        if (!edit_is_current_) {
            if (name == edit_map_) {
                resetEdit(QString("正在编辑的「%1」已被加载为当前地图，本次编辑结束%2；如需继续请重新「开始编辑」")
                              .arg(QString::fromStdString(name)).arg(dropped));
            }
        } else if (edit_map_ != name) {
            resetEdit(QString("当前地图已切换（%1），本次编辑结束%2；如需编辑请重新「开始编辑」")
                          .arg(QString::fromStdString(name)).arg(dropped));
        } else {
            resetEdit(QString("已重新加载「%1」，修改已生效，本次编辑结束%2；如需继续请重新「开始编辑」")
                          .arg(QString::fromStdString(name)).arg(dropped));
        }
    }
    onMapSelectionChanged();
    refreshMaps();
}

// ── 编辑未加载地图：预览与显示聚焦 ──────────────────────────────────────

bool MapManagerPanel::publishEditPreview(const std::string& map_name, QString& err) {
    if (map_folder_.empty()) { err = "get_maps 没有给出地图目录"; return false; }
    QString yaml = QString::fromStdString(map_folder_ + "/" + map_name + "/slam/map.yaml");
    if (!QFile::exists(yaml)) yaml = QString::fromStdString(map_folder_ + "/" + map_name + "/map.yaml");
    nav_msgs::msg::OccupancyGrid grid;
    if (!LoadTrinaryGrid(yaml, grid, err)) return false;
    grid.header.stamp = node_->now();
    edit_preview_pub_->publish(grid);
    return true;
}

void MapManagerPanel::clearEditPreview() {
    if (!edit_preview_pub_) return;
    // 发 1×1 未知格占位而不是空图：空图会让 RViz 留下旧贴图
    nav_msgs::msg::OccupancyGrid g;
    g.header.frame_id = "map";
    g.header.stamp = node_->now();
    g.info.resolution = 0.05f;
    g.info.width = g.info.height = 1;
    g.info.origin.orientation.w = 1.0;
    g.data = {-1};
    edit_preview_pub_->publish(g);
}

// 编辑未加载地图时只看它：关掉当前地图的底图 / 掩膜 / 资产显示与资产面板的预览，打开本面板的预览（没有就新建）；
// 结束时恢复原启用状态
void MapManagerPanel::applyEditFocus(bool focus) {
    auto* ctx = getDisplayContext();
    auto* root = ctx ? ctx->getRootDisplayGroup() : nullptr;
    if (!root) return;
    auto is_loaded_map_display = [](const QString& topic) {
        return topic == "/map" || topic.startsWith("/map_asset_manager/") || topic.startsWith("/map_asset_manager_panel/") ||
               topic == "/keepout_filter_mask" || topic == "/speed_filter_mask";
    };
    rviz_common::Display* preview = nullptr;
    std::function<void(rviz_common::DisplayGroup*)> walk = [&](rviz_common::DisplayGroup* g) {
        for (int i = 0; i < g->numDisplays(); ++i) {
            rviz_common::Display* d = g->getDisplayAt(i);
            if (auto* sub = dynamic_cast<rviz_common::DisplayGroup*>(d)) { walk(sub); continue; }
            auto* topic_prop = d->subProp("Topic");
            if (!topic_prop) continue;
            const QString topic = topic_prop->getValue().toString();
            if (topic == kEditPreviewTopic) { preview = d; continue; }
            if (!is_loaded_map_display(topic)) continue;
            if (focus) {
                if (!focus_hidden_.count(d)) focus_hidden_[d] = d->isEnabled();
                d->setEnabled(false);
            } else if (auto it = focus_hidden_.find(d); it != focus_hidden_.end()) {
                d->setEnabled(it->second);
            }
        }
    };
    walk(root);
    if (!focus) focus_hidden_.clear();
    if (!preview && focus) {
        if (auto* vm = dynamic_cast<rviz_common::VisualizationManager*>(ctx)) {
            preview = vm->createDisplay("rviz_default_plugins/Map", "GridEditTarget", false);
            if (preview) {
                if (auto* t = preview->subProp("Topic")) {
                    t->setValue(kEditPreviewTopic);
                    if (auto* dur = t->subProp("Durability Policy")) dur->setValue("Transient Local");
                }
            }
        }
    }
    if (preview) preview->setEnabled(focus);
}

// 「重新加载使其生效」是否可用以文件为准：记下当前地图加载时（map_state epoch 变化后第一次取到）的底图指纹，
// 之后当前地图文件的指纹与它不同 = 有已保存但未生效的修改。不跟编辑会话走，取消 / 重新开始编辑、
// 空保存、别的客户端改了当前图都不影响判断；面板启动前就存在的未生效修改无从得知。
void MapManagerPanel::updateReloadState() {
    std::string file_revision;
    for (const auto& m : maps_) {
        if (m.name == current_map_) file_revision = m.grid_revision;
    }
    if (current_map_.empty() || file_revision.empty()) {
        reload_btn_->setEnabled(false);
        return;
    }
    if (loaded_map_ != current_map_ || loaded_epoch_ != map_epoch_ || loaded_revision_.empty()) {
        loaded_map_ = current_map_;
        loaded_epoch_ = map_epoch_;
        loaded_revision_ = file_revision;
    }
    const bool pending = (file_revision != loaded_revision_);
    reload_btn_->setEnabled(pending);
    reload_btn_->setText(pending ? "重新加载使其生效（有未生效的修改）" : "重新加载使其生效");
}

void MapManagerPanel::reloadEditedMap() {
    const std::string name = current_map_;  // 未生效的修改总是针对当前地图
    if (name.empty()) return;
    const auto& d = GridEditDraft::instance();
    const QString unsaved = (d.editing && !d.strokes().empty())
        ? QString("\n还有 %1 笔未保存，重新加载后会被丢弃。").arg(d.strokes().size()) : QString();
    if (QMessageBox::question(this, "重新加载",
                              QString("重新加载「%1」使底图修改生效？本次编辑随之结束。\n整机运行时定位会重新加载地图，需要重新定位。%2")
                                  .arg(QString::fromStdString(name)).arg(unsaved)) != QMessageBox::Yes) return;
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
