#include "map_asset_rviz_plugin/map_asset_panel.hpp"

#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/tool_manager.hpp>

#include "map_asset_rviz_plugin/area_tab.hpp"
#include "map_asset_rviz_plugin/nav_point_tab.hpp"
#include "map_asset_rviz_plugin/room_tab.hpp"
#include "map_asset_rviz_plugin/route_tab.hpp"
#include "map_asset_rviz_plugin/route_graph_tab.hpp"

namespace map_asset_rviz_plugin {

static constexpr char kAddNavPointToolId[]    = "map_asset_rviz_plugin/AddNavPointTool";
static constexpr char kAddAreaToolId[]        = "map_asset_rviz_plugin/AddAreaTool";
static constexpr char kAddRoomToolId[]        = "map_asset_rviz_plugin/AddRoomTool";
static constexpr char kAddJunctionToolId[]    = "map_asset_rviz_plugin/AddJunctionTool";
static constexpr char kAddEdgeToolId[]        = "map_asset_rviz_plugin/AddEdgeTool";
static constexpr char kAddCurveEdgeToolId[]   = "map_asset_rviz_plugin/AddCurveEdgeTool";

MapAssetPanel::MapAssetPanel(QWidget* parent)
    : rviz_common::Panel(parent) {
    // ── 服务前缀 ──────────────────────────────────────────────────
    prefix_edit_ = new QLineEdit("/map_asset_manager");

    auto* prefix_row = new QFormLayout();
    prefix_row->addRow("Service prefix:", prefix_edit_);

    // ── 工具快捷按钮 ──────────────────────────────────────────────
    add_point_btn_         = new QPushButton("选取导航点  [N]");
    add_area_btn_          = new QPushButton("绘制区域  [A]");
    add_room_btn_          = new QPushButton("绘制房间  [M]");
    add_junction_btn_      = new QPushButton("添加 Junction  [J]");
    add_edge_btn_          = new QPushButton("直线路网边  [L]");
    add_curve_edge_btn_    = new QPushButton("曲线路网边  [B]");
    route_graph_delete_btn_ = new QPushButton("删除节点/边  [D]");
    route_graph_start_btn_  = new QPushButton("开始修改路网  [R]");
    route_graph_save_btn_   = new QPushButton("保存路网  [S]");
    add_point_btn_->setToolTip("激活「选取导航点」工具（快捷键 N）：在地图上拖拽选取位置和朝向，松开后填写名称再点「添加点位」");
    add_area_btn_->setToolTip("激活「绘制区域」工具（快捷键 A）：先填写属性，再左键画顶点，右键完成");
    add_room_btn_->setToolTip("激活「绘制房间」工具（快捷键 M）：先填写名称/类型/关联点，再左键画顶点，右键完成（DSG ROOMS 层节点）");
    add_junction_btn_->setToolTip("激活「添加 Junction」工具（快捷键 J）：左键点击放置路网中间节点，自动命名 jct_N，右键退出");
    add_edge_btn_->setToolTip("激活「直线路网边」工具（快捷键 L）：左键 snap 选起点（蓝色小圈），移动到终点后右键完成");
    add_curve_edge_btn_->setToolTip("激活「贝塞尔曲线路网边」工具（快捷键 B）：左键 snap 选起点，左键添加控制点，右键 snap 选终点完成");
    route_graph_delete_btn_->setToolTip("激活「统一删除」工具（快捷键 D）：左键高亮 Junction/边，Del 键确认删除，右键退出");
    route_graph_start_btn_->setToolTip("激活「开始修改路网」工具（快捷键 R）：从文件加载路网并进入编辑模式，保存前可任意编辑");
    route_graph_save_btn_->setToolTip("激活「保存路网」工具（快捷键 S）：将当前内存路网持久化到文件");

    auto* tool_row1 = new QHBoxLayout();
    tool_row1->addWidget(add_point_btn_);
    tool_row1->addWidget(add_area_btn_);
    tool_row1->addWidget(add_room_btn_);

    auto* tool_row2 = new QHBoxLayout();
    tool_row2->addWidget(add_junction_btn_);
    tool_row2->addWidget(route_graph_delete_btn_);

    auto* tool_row3 = new QHBoxLayout();
    tool_row3->addWidget(add_edge_btn_);
    tool_row3->addWidget(add_curve_edge_btn_);

    auto* tool_row4 = new QHBoxLayout();
    tool_row4->addWidget(route_graph_start_btn_);
    tool_row4->addWidget(route_graph_save_btn_);

    auto* tool_vbox = new QVBoxLayout();
    tool_vbox->addLayout(tool_row1);
    tool_vbox->addLayout(tool_row2);
    tool_vbox->addLayout(tool_row3);
    tool_vbox->addLayout(tool_row4);

    auto* tool_box = new QGroupBox("工具");
    tool_box->setLayout(tool_vbox);

    // ── 标签页 ────────────────────────────────────────────────────
    nav_point_tab_    = new NavPointTab();
    route_tab_        = new RouteTab();
    area_tab_         = new AreaTab();
    room_tab_         = new RoomTab();
    route_graph_tab_  = new RouteGraphTab();

    tabs_ = new QTabWidget();
    tabs_->addTab(nav_point_tab_,   "导航点");
    tabs_->addTab(route_tab_,       "路线");
    tabs_->addTab(area_tab_,        "区域");
    tabs_->addTab(room_tab_,        "房间");
    tabs_->addTab(route_graph_tab_, "路网");

    // ── 总布局 ────────────────────────────────────────────────────
    auto* root = new QVBoxLayout(this);
    root->addLayout(prefix_row);
    root->addWidget(tool_box);
    root->addWidget(tabs_);

    connect(prefix_edit_,    &QLineEdit::editingFinished,
            this, &MapAssetPanel::onPrefixChanged);
    connect(add_point_btn_,  &QPushButton::clicked,
            this, &MapAssetPanel::onAddNavPointTool);
    connect(add_area_btn_,   &QPushButton::clicked,
            this, &MapAssetPanel::onAddAreaTool);
    connect(add_room_btn_,   &QPushButton::clicked,
            this, &MapAssetPanel::onAddRoomTool);
    connect(add_junction_btn_,       &QPushButton::clicked,
            this, &MapAssetPanel::onAddJunctionTool);
    connect(add_edge_btn_,           &QPushButton::clicked,
            this, &MapAssetPanel::onAddEdgeTool);
    connect(add_curve_edge_btn_,     &QPushButton::clicked,
            this, &MapAssetPanel::onAddCurveEdgeTool);
    connect(route_graph_delete_btn_, &QPushButton::clicked,
            this, &MapAssetPanel::onRouteGraphDeleteTool);
    connect(route_graph_start_btn_,  &QPushButton::clicked,
            this, &MapAssetPanel::onRouteGraphStartTool);
    connect(route_graph_save_btn_,   &QPushButton::clicked,
            this, &MapAssetPanel::onRouteGraphSaveTool);
}

// ──────────────────────────────────────────────────────────────────────
// 初始化
// ──────────────────────────────────────────────────────────────────────

void MapAssetPanel::onInitialize() {
    node_ = getDisplayContext()
                ->getRosNodeAbstraction()
                .lock()
                ->get_raw_node();

    const std::string prefix = prefix_edit_->text().toStdString();
    nav_point_tab_->setNode(node_);
    nav_point_tab_->setServicePrefix(prefix);

    route_tab_->setNode(node_);
    route_tab_->setServicePrefix(prefix);

    area_tab_->setNode(node_);
    area_tab_->setServicePrefix(prefix);

    room_tab_->setNode(node_);
    room_tab_->setServicePrefix(prefix);

    route_graph_tab_->setNode(node_);
    route_graph_tab_->setServicePrefix(prefix);
}

// ──────────────────────────────────────────────────────────────────────
// 工具激活
// ──────────────────────────────────────────────────────────────────────

void MapAssetPanel::activateTool(const QString& class_id) {
    auto* tm = getDisplayContext()->getToolManager();

    // 先在已有工具中查找
    for (int i = 0; i < tm->numTools(); ++i) {
        auto* tool = tm->getTool(i);
        if (tool->getClassId() == class_id) {
            tm->setCurrentTool(tool);
            return;
        }
    }

    // 工具栏中还没有该工具，动态添加后激活
    auto* tool = tm->addTool(class_id);
    if (tool) {
        tm->setCurrentTool(tool);
    }
}

void MapAssetPanel::onAddNavPointTool() {
    activateTool(kAddNavPointToolId);
    // 切换到「导航点」标签页方便查看结果
    tabs_->setCurrentWidget(nav_point_tab_);
}

void MapAssetPanel::onAddAreaTool() {
    activateTool(kAddAreaToolId);
    tabs_->setCurrentWidget(area_tab_);
}

void MapAssetPanel::onAddRoomTool() {
    activateTool(kAddRoomToolId);
    tabs_->setCurrentWidget(room_tab_);
}

void MapAssetPanel::onAddJunctionTool() {
    activateTool(kAddJunctionToolId);
    tabs_->setCurrentWidget(route_graph_tab_);
}

void MapAssetPanel::onAddEdgeTool() {
    activateTool(kAddEdgeToolId);
    tabs_->setCurrentWidget(route_graph_tab_);
}

void MapAssetPanel::onAddCurveEdgeTool() {
    activateTool(kAddCurveEdgeToolId);
    tabs_->setCurrentWidget(route_graph_tab_);
}

void MapAssetPanel::onRouteGraphDeleteTool() {
    activateTool("map_asset_rviz_plugin/RouteGraphDeleteTool");
    tabs_->setCurrentWidget(route_graph_tab_);
}

void MapAssetPanel::onRouteGraphStartTool() {
    activateTool("map_asset_rviz_plugin/RouteGraphStartTool");
}

void MapAssetPanel::onRouteGraphSaveTool() {
    activateTool("map_asset_rviz_plugin/RouteGraphSaveTool");
}

// ──────────────────────────────────────────────────────────────────────
// 其他
// ──────────────────────────────────────────────────────────────────────

void MapAssetPanel::onPrefixChanged() {
    if (!node_) return;
    const std::string prefix = prefix_edit_->text().trimmed().toStdString();
    nav_point_tab_->setServicePrefix(prefix);
    route_tab_->setServicePrefix(prefix);
    area_tab_->setServicePrefix(prefix);
    room_tab_->setServicePrefix(prefix);
    route_graph_tab_->setServicePrefix(prefix);
}

void MapAssetPanel::save(rviz_common::Config config) const {
    Panel::save(config);
    config.mapSetValue("service_prefix", prefix_edit_->text());
}

void MapAssetPanel::load(const rviz_common::Config& config) {
    Panel::load(config);
    QString prefix;
    if (config.mapGetString("service_prefix", &prefix)) {
        prefix_edit_->setText(prefix);
        if (node_) onPrefixChanged();
    }
}

}  // namespace map_asset_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(map_asset_rviz_plugin::MapAssetPanel, rviz_common::Panel)
