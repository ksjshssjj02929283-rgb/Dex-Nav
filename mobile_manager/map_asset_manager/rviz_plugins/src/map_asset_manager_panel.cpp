#include "map_asset_rviz_plugin/map_asset_manager_panel.hpp"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QImage>
#include <QMessageBox>
#include <QPointer>
#include <QTextStream>
#include <QVBoxLayout>

#include <cmath>
#include <functional>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/tool_manager.hpp>

#include "map_asset_rviz_plugin/area_tab.hpp"
#include "map_asset_rviz_plugin/asset_target.hpp"
#include "map_asset_rviz_plugin/nav_point_tab.hpp"
#include "map_asset_rviz_plugin/room_tab.hpp"
#include "map_asset_manager/types.hpp"
#include "map_asset_rviz_plugin/route_graph_tab.hpp"
#include "map_asset_rviz_plugin/route_tab.hpp"

namespace map_asset_rviz_plugin {

namespace {

constexpr char kAddNavPointToolId[] = "map_asset_rviz_plugin/AddNavPointTool";
constexpr char kAddAreaToolId[] = "map_asset_rviz_plugin/AddAreaTool";
constexpr char kAddRoomToolId[] = "map_asset_rviz_plugin/AddRoomTool";
constexpr char kAddJunctionToolId[] = "map_asset_rviz_plugin/AddJunctionTool";
constexpr char kAddEdgeToolId[] = "map_asset_rviz_plugin/AddEdgeTool";
constexpr char kAddCurveEdgeToolId[] = "map_asset_rviz_plugin/AddCurveEdgeTool";
constexpr char kPreviewMapTopic[] = "/map_asset_manager_panel/target_map";
constexpr char kPreviewAssetsTopic[] = "/map_asset_manager_panel/target_assets";

QString SourceLabel(const std::string& s) {
    if (s == "save") return "显式保存";
    if (s == "transform") return "坐标变换前";
    if (s == "rollback") return "回滚前";
    return "未标记";
}

std::string Trim(std::string s) {
    const auto b = s.find_first_not_of(" \t\r\"'");
    if (b == std::string::npos) return "";
    const auto e = s.find_last_not_of(" \t\r\"'");
    return s.substr(b, e - b + 1);
}

std_msgs::msg::ColorRGBA Color(float r, float g, float b, float a = 0.9f) {
    std_msgs::msg::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

geometry_msgs::msg::Point P(double x, double y, double z = 0.05) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

}  // namespace

MapAssetManagerPanel::MapAssetManagerPanel(QWidget* parent) : rviz_common::Panel(parent) {
    prefix_edit_ = new QLineEdit("/map_asset_manager");
    mm_prefix_edit_ = new QLineEdit("/map_manager");
    auto* prefix_row = new QFormLayout();
    prefix_row->addRow("资产服务前缀:", prefix_edit_);
    prefix_row->addRow("地图管理前缀:", mm_prefix_edit_);

    // ── 编辑目标 ──────────────────────────────────────────────────
    target_combo_ = new QComboBox();
    target_combo_->setToolTip("要编辑哪张地图的资产。选非当前地图时，页签与地图上的工具都改该图的文件，"
                              "不影响当前地图与导航；RViz 里加 Map（" + QString(kPreviewMapTopic) +
                              "）与 MarkerArray（" + QString(kPreviewAssetsTopic) + "）可看到该图");
    auto* refresh_maps_btn = new QPushButton("刷新地图");
    target_info_label_ = new QLabel("—");
    target_info_label_->setWordWrap(true);
    auto* target_row = new QHBoxLayout();
    target_row->addWidget(new QLabel("编辑目标:"));
    target_row->addWidget(target_combo_, 1);
    target_row->addWidget(refresh_maps_btn);
    auto* target_box = new QGroupBox("编辑目标");
    auto* target_vbox = new QVBoxLayout(target_box);
    target_vbox->addLayout(target_row);
    target_vbox->addWidget(target_info_label_);
    focus_check_ = new QCheckBox("只看编辑目标（编辑非当前地图时隐藏当前地图的底图 / 资产 / 掩膜显示）");
    focus_check_->setChecked(true);
    target_vbox->addWidget(focus_check_);
    refresh_timer_ = new QTimer(this);
    refresh_timer_->setSingleShot(true);
    refresh_timer_->setInterval(200);
    connect(refresh_timer_, &QTimer::timeout, this, [this]() {
        AssetTarget::bumpGeneration();  // 目标图数据变了：地图工具下次操作前重载端点 / 节点
        refreshTabs();
        refreshTargetInfo();
    });
    connect(focus_check_, &QCheckBox::toggled, this, [this](bool) { applyDisplayFocus(); });

    // ── 工具 ──────────────────────────────────────────────────────
    auto* add_point_btn = new QPushButton("选取导航点  [N]");
    auto* add_area_btn = new QPushButton("绘制区域  [A]");
    auto* add_room_btn = new QPushButton("绘制房间  [M]");
    auto* add_junction_btn = new QPushButton("添加 Junction  [J]");
    auto* add_edge_btn = new QPushButton("直线路网边  [L]");
    auto* add_curve_edge_btn = new QPushButton("曲线路网边  [B]");
    auto* graph_delete_btn = new QPushButton("删除节点/边  [D]");
    auto* graph_start_btn = new QPushButton("开始修改路网  [R]");
    auto* graph_save_btn = new QPushButton("保存路网  [S]");
    add_point_btn->setToolTip("在地图上拖拽选取位置和朝向，松开后填写名称再点「添加点位」");
    add_area_btn->setToolTip("先填写属性，再左键画顶点，右键完成");
    add_room_btn->setToolTip("先填写名称/类型/关联点，再左键画顶点，右键完成");
    add_junction_btn->setToolTip("左键放置路网中间节点，自动命名 jct_N，右键退出");
    add_edge_btn->setToolTip("左键 snap 选起点，移动到终点后右键完成");
    add_curve_edge_btn->setToolTip("左键 snap 选起点，左键添加控制点，右键 snap 选终点完成");
    graph_delete_btn->setToolTip("左键高亮 Junction/边，Del 键确认删除，右键退出");
    graph_start_btn->setToolTip("当前地图：从文件重载路网并进入编辑；非当前地图：直接读写文件");
    graph_save_btn->setToolTip("把编辑目标的资产存成一份版本");
    auto* grid = new QGridLayout();
    grid->addWidget(add_point_btn, 0, 0);
    grid->addWidget(add_area_btn, 0, 1);
    grid->addWidget(add_room_btn, 0, 2);
    grid->addWidget(add_junction_btn, 1, 0);
    grid->addWidget(add_edge_btn, 1, 1);
    grid->addWidget(add_curve_edge_btn, 1, 2);
    grid->addWidget(graph_delete_btn, 2, 0);
    grid->addWidget(graph_start_btn, 2, 1);
    grid->addWidget(graph_save_btn, 2, 2);
    auto* tool_box = new QGroupBox("工具（作用于编辑目标）");
    tool_box->setLayout(grid);

    // ── 页签 ──────────────────────────────────────────────────────
    nav_point_tab_ = new NavPointTab();
    route_tab_ = new RouteTab();
    area_tab_ = new AreaTab();
    room_tab_ = new RoomTab();
    route_graph_tab_ = new RouteGraphTab();

    auto* versions_page = new QWidget();
    versions_table_ = new QTableWidget(0, 3);
    versions_table_->setHorizontalHeaderLabels({"版本", "时间", "来源"});
    versions_table_->horizontalHeader()->setStretchLastSection(true);
    versions_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    versions_table_->setSelectionMode(QAbstractItemView::SingleSelection);
    versions_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    auto* save_version_btn = new QPushButton("存一份版本");
    auto* rollback_btn = new QPushButton("回滚到选中版本");
    rollback_btn->setToolTip("回滚前自动把当前资产存成「回滚前」版本，可再回滚撤销；当前地图回滚后立即生效");
    auto* version_btns = new QHBoxLayout();
    version_btns->addWidget(save_version_btn);
    version_btns->addWidget(rollback_btn);
    auto* versions_layout = new QVBoxLayout(versions_page);
    versions_layout->addWidget(versions_table_);
    versions_layout->addLayout(version_btns);

    auto* issues_page = new QWidget();
    issues_list_ = new QListWidget();
    auto* issues_layout = new QVBoxLayout(issues_page);
    issues_layout->addWidget(new QLabel("底图或禁行区变化后不可用的点位 / 路网（资产保留，修正后自动消失）"));
    issues_layout->addWidget(issues_list_);

    tabs_ = new QTabWidget();
    tabs_->addTab(nav_point_tab_, "导航点");
    tabs_->addTab(route_tab_, "路线");
    tabs_->addTab(area_tab_, "区域");
    tabs_->addTab(room_tab_, "房间");
    tabs_->addTab(route_graph_tab_, "路网");
    tabs_->addTab(versions_page, "版本");
    tabs_->addTab(issues_page, "失效");

    status_label_ = new QLabel();
    status_label_->setWordWrap(true);

    auto* root = new QVBoxLayout(this);
    root->addLayout(prefix_row);
    root->addWidget(target_box);
    root->addWidget(tool_box);
    root->addWidget(tabs_, 1);
    root->addWidget(status_label_);

    connect(prefix_edit_, &QLineEdit::editingFinished, this, &MapAssetManagerPanel::onPrefixChanged);
    connect(mm_prefix_edit_, &QLineEdit::editingFinished, this, &MapAssetManagerPanel::onPrefixChanged);
    connect(refresh_maps_btn, &QPushButton::clicked, this, &MapAssetManagerPanel::refreshMapList);
    connect(target_combo_, QOverload<int>::of(&QComboBox::activated), this, &MapAssetManagerPanel::onTargetChanged);
    connect(save_version_btn, &QPushButton::clicked, this, &MapAssetManagerPanel::saveVersion);
    connect(rollback_btn, &QPushButton::clicked, this, &MapAssetManagerPanel::rollbackToVersion);
    connect(add_point_btn, &QPushButton::clicked, this, &MapAssetManagerPanel::onAddNavPointTool);
    connect(add_area_btn, &QPushButton::clicked, this, &MapAssetManagerPanel::onAddAreaTool);
    connect(add_room_btn, &QPushButton::clicked, this, &MapAssetManagerPanel::onAddRoomTool);
    connect(add_junction_btn, &QPushButton::clicked, this, &MapAssetManagerPanel::onAddJunctionTool);
    connect(add_edge_btn, &QPushButton::clicked, this, &MapAssetManagerPanel::onAddEdgeTool);
    connect(add_curve_edge_btn, &QPushButton::clicked, this, &MapAssetManagerPanel::onAddCurveEdgeTool);
    connect(graph_delete_btn, &QPushButton::clicked, this, &MapAssetManagerPanel::onRouteGraphDeleteTool);
    connect(graph_start_btn, &QPushButton::clicked, this, &MapAssetManagerPanel::onRouteGraphStartTool);
    connect(graph_save_btn, &QPushButton::clicked, this, &MapAssetManagerPanel::onRouteGraphSaveTool);
}

void MapAssetManagerPanel::onInitialize() {
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    // 地图工具的执行结果（成功 / 被拒原因）显示到本面板状态栏
    QPointer<MapAssetManagerPanel> self(this);
    AssetTarget::setReporter([self](const QString& text, bool ok) {
        if (!self) return;
        QMetaObject::invokeMethod(self.data(), [self, text, ok]() {
            if (self) self->setStatus(text, !ok);
        }, Qt::QueuedConnection);
    });
    auto latched = rclcpp::QoS(1).transient_local().reliable();
    preview_map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(kPreviewMapTopic, latched);
    preview_assets_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(kPreviewAssetsTopic, latched);
    recreateClients();
    QMetaObject::invokeMethod(this, "refreshMapList", Qt::QueuedConnection);
}

void MapAssetManagerPanel::recreateClients() {
    if (!node_) return;
    const std::string prefix = prefix_edit_->text().trimmed().toStdString();
    const std::string mm = mm_prefix_edit_->text().trimmed().toStdString();
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

    get_maps_cli_ = node_->create_client<nav_interfaces::srv::GetMaps>(mm + "/get_maps");
    get_assets_cli_ = node_->create_client<nav_interfaces::srv::GetAssets>(prefix + "/get_assets");
    save_assets_cli_ = node_->create_client<nav_interfaces::srv::SaveAssets>(prefix + "/save_assets");
    rollback_cli_ = node_->create_client<nav_interfaces::srv::RollbackAssets>(prefix + "/rollback_assets");

    auto latched = rclcpp::QoS(1).transient_local().reliable();
    map_state_sub_ = node_->create_subscription<nav_interfaces::msg::MapState>(
        mm + "/map_state", latched, [this](nav_interfaces::msg::MapState::ConstSharedPtr msg) {
            const std::string name = msg->map_name, folder = msg->map_folder;
            QMetaObject::invokeMethod(this, [this, name, folder]() {
                const bool changed = (name != current_map_);
                current_map_ = name;
                map_folder_ = folder;
                if (changed) {
                    // 编辑目标恰好成为当前地图 → 按当前地图处理
                    if (AssetTarget::mapName() == name) applyTarget("");
                    refreshMapList();
                }
            }, Qt::QueuedConnection);
        });
    // 任意地图的资产变更事件：是编辑目标就刷新页签、信息与预览（地图上的工具写非当前图后靠它更新显示）
    changes_sub_ = node_->create_subscription<nav_interfaces::msg::AssetChange>(
        prefix + "/asset_changes", rclcpp::QoS(10).reliable(),
        [this](nav_interfaces::msg::AssetChange::ConstSharedPtr m) {
            const std::string name = m->map_name;
            const bool is_current = m->is_current;
            const std::string revision = m->revision;
            QMetaObject::invokeMethod(this, [this, name, is_current, revision]() {
                const std::string target = AssetTarget::mapName();
                const bool hit = target.empty() ? is_current : (name == target);
                if (hit && revision != last_revision_) scheduleRefresh();
            }, Qt::QueuedConnection);
        });
    assets_sub_ = node_->create_subscription<nav_interfaces::msg::AssetSnapshot>(
        prefix + "/assets", latched, [this](nav_interfaces::msg::AssetSnapshot::ConstSharedPtr) {
            // 当前地图资产变化（任何编辑入口）→ 刷新版本与失效清单
            QMetaObject::invokeMethod(this, [this]() {
                if (AssetTarget::isCurrent()) scheduleRefresh();
            }, Qt::QueuedConnection);
        });
}

void MapAssetManagerPanel::onPrefixChanged() { recreateClients(); }

void MapAssetManagerPanel::setStatus(const QString& text, bool error) {
    status_label_->setStyleSheet(error ? "color: #c0392b;" : "");
    status_label_->setText(text);
}

void MapAssetManagerPanel::refreshMapList() {
    if (!get_maps_cli_) return;
    if (!get_maps_cli_->service_is_ready()) {
        setStatus("地图列表服务不可用（" + mm_prefix_edit_->text() + "/get_maps）", true);
        return;
    }
    get_maps_cli_->async_send_request(
        std::make_shared<nav_interfaces::srv::GetMaps::Request>(),
        [this](rclcpp::Client<nav_interfaces::srv::GetMaps>::SharedFuture fut) {
            auto resp = fut.get();
            std::vector<std::string> names;
            for (const auto& m : resp->maps) names.push_back(m.name);
            QMetaObject::invokeMethod(this, [this, names]() {
                const std::string keep = pending_target_.empty() ? AssetTarget::mapName() : pending_target_;
                pending_target_.clear();
                target_combo_->clear();
                target_combo_->addItem(QString("当前地图（%1）").arg(QString::fromStdString(current_map_.empty() ? "未加载" : current_map_)), QString());
                int select = 0;
                for (const auto& n : names) {
                    if (n == current_map_) continue;
                    target_combo_->addItem(QString::fromStdString(n), QString::fromStdString(n));
                    if (n == keep) select = target_combo_->count() - 1;
                }
                target_combo_->setCurrentIndex(select);
                applyTarget(select == 0 ? std::string() : keep);
            }, Qt::QueuedConnection);
        });
}

void MapAssetManagerPanel::onTargetChanged(int index) {
    applyTarget(target_combo_->itemData(index).toString().toStdString());
}

void MapAssetManagerPanel::applyTarget(const std::string& map_name) {
    const bool changed = (map_name != AssetTarget::mapName());
    AssetTarget::setMapName(map_name);
    AssetTarget::bumpGeneration();
    if (map_name.empty()) clearPreview();
    if (changed) {
        last_revision_.clear();
        setStatus(QString("编辑目标：%1").arg(QString::fromStdString(AssetTarget::label())));
    }
    applyDisplayFocus();
    refreshTabs();
    refreshTargetInfo();
}

void MapAssetManagerPanel::scheduleRefresh() { refresh_timer_->start(); }

void MapAssetManagerPanel::applyDisplayFocus() {
    auto* root = getDisplayContext() ? getDisplayContext()->getRootDisplayGroup() : nullptr;
    if (!root) return;
    const bool hide_loaded = focus_check_->isChecked() && !AssetTarget::isCurrent();
    const QString prefix = prefix_edit_->text().trimmed();
    // 当前加载地图的显示：底图 /map、资产节点的标记话题、costmap filter 掩膜
    auto is_loaded_map_display = [&](const QString& topic) {
        return topic == "/map" || topic.startsWith(prefix + "/") || topic == "/keepout_filter_mask" ||
               topic == "/speed_filter_mask";
    };
    std::function<void(rviz_common::DisplayGroup*)> walk = [&](rviz_common::DisplayGroup* g) {
        for (int i = 0; i < g->numDisplays(); ++i) {
            rviz_common::Display* d = g->getDisplayAt(i);
            if (auto* sub = dynamic_cast<rviz_common::DisplayGroup*>(d)) { walk(sub); continue; }
            auto* topic_prop = d->subProp("Topic");
            if (!topic_prop) continue;
            const QString topic = topic_prop->getValue().toString();
            if (topic.startsWith("/map_asset_manager_panel/")) {
                // 预览只服务于非当前地图：目标是当前地图时关掉，避免残留贴图与当前底图叠影
                const bool want = !AssetTarget::isCurrent();
                if (d->isEnabled() != want) d->setEnabled(want);
                continue;
            }
            if (!is_loaded_map_display(topic)) continue;
            if (hide_loaded) {
                if (!hidden_displays_.count(d)) hidden_displays_[d] = d->isEnabled();
                d->setEnabled(false);
            } else if (auto it = hidden_displays_.find(d); it != hidden_displays_.end()) {
                d->setEnabled(it->second);
                hidden_displays_.erase(it);
            }
        }
    };
    walk(root);
    if (!hide_loaded) hidden_displays_.clear();
}

void MapAssetManagerPanel::refreshTabs() {
    for (QObject* tab : {static_cast<QObject*>(nav_point_tab_), static_cast<QObject*>(area_tab_),
                         static_cast<QObject*>(room_tab_), static_cast<QObject*>(route_graph_tab_)}) {
        QMetaObject::invokeMethod(tab, "onRefresh", Qt::QueuedConnection);
    }
    QMetaObject::invokeMethod(route_tab_, "onRefreshRoutes", Qt::QueuedConnection);
    QMetaObject::invokeMethod(route_tab_, "onRefreshPoints", Qt::QueuedConnection);
}

void MapAssetManagerPanel::refreshTargetInfo() {
    if (!get_assets_cli_ || !get_assets_cli_->service_is_ready()) {
        target_info_label_->setText("资产服务不可用");
        return;
    }
    auto req = std::make_shared<nav_interfaces::srv::GetAssets::Request>();
    map_asset_rviz_plugin::applyTarget(req);
    const std::string asked = req->map_name;
    get_assets_cli_->async_send_request(req, [this, asked](rclcpp::Client<nav_interfaces::srv::GetAssets>::SharedFuture fut) {
        auto resp = fut.get();
        QMetaObject::invokeMethod(this, [this, resp, asked]() {
            if (asked != AssetTarget::mapName()) return;  // 目标已切换，丢弃旧结果
            if (!resp->success) {
                target_info_label_->setText(QString("读取失败 [%1] %2").arg(resp->error_code).arg(QString::fromStdString(resp->message)));
                return;
            }
            showTargetInfo(*resp);
            if (!resp->is_current) publishPreview(*resp);
        }, Qt::QueuedConnection);
    });
}

void MapAssetManagerPanel::showTargetInfo(const nav_interfaces::srv::GetAssets::Response& r) {
    last_revision_ = r.revision;
    const QString edited = r.last_edited_at.sec > 0
        ? QDateTime::fromSecsSinceEpoch(r.last_edited_at.sec).toString("yyyy-MM-dd hh:mm:ss")
        : QString("—");
    target_info_label_->setText(QString("%1%2 · 点位 %3 · 区域 %4 · 房间 %5 · 路线 %6 · 节点 %7 · 边 %8 · 失效 %9 · 指纹 %10")
                                    .arg(QString::fromStdString(r.map_name))
                                    .arg(r.is_current ? "（当前地图）" : "（未加载，直接读写文件）")
                                    .arg(r.points.size()).arg(r.areas.size()).arg(r.rooms.size()).arg(r.routes.size())
                                    .arg(r.junctions.size()).arg(r.edges.size()).arg(r.issues.size())
                                    .arg(QString::fromStdString(r.revision)) +
                                QString(" · 资产最后编辑 %1").arg(edited));
    versions_table_->setRowCount(0);
    for (const auto& v : r.versions) {
        const int row = versions_table_->rowCount();
        versions_table_->insertRow(row);
        versions_table_->setItem(row, 0, new QTableWidgetItem(QString::fromStdString(v.version_ts)));
        versions_table_->setItem(row, 1, new QTableWidgetItem(
            QDateTime::fromSecsSinceEpoch(v.saved_at.sec).toString("yyyy-MM-dd hh:mm:ss")));
        versions_table_->setItem(row, 2, new QTableWidgetItem(SourceLabel(v.source)));
    }
    issues_list_->clear();
    for (const auto& i : r.issues) {
        issues_list_->addItem(QString("[%1] %2 — %3").arg(QString::fromStdString(i.kind))
                                  .arg(QString::fromStdString(i.key)).arg(QString::fromStdString(i.code)));
    }
    tabs_->setTabText(6, r.issues.empty() ? QString("失效") : QString("失效 (%1)").arg(r.issues.size()));
}

void MapAssetManagerPanel::saveVersion() {
    if (!save_assets_cli_ || !save_assets_cli_->service_is_ready()) { setStatus("save_assets 不可用", true); return; }
    auto req = std::make_shared<nav_interfaces::srv::SaveAssets::Request>();
    map_asset_rviz_plugin::applyTarget(req);
    save_assets_cli_->async_send_request(req, [this](rclcpp::Client<nav_interfaces::srv::SaveAssets>::SharedFuture fut) {
        auto resp = fut.get();
        QMetaObject::invokeMethod(this, [this, resp]() {
            setStatus(QString::fromStdString(resp->message), !resp->success);
            refreshTargetInfo();
        }, Qt::QueuedConnection);
    });
}

void MapAssetManagerPanel::rollbackToVersion() {
    const auto sel = versions_table_->selectionModel()->selectedRows();
    if (sel.isEmpty()) { setStatus("先在版本列表里选一个版本", true); return; }
    const QString ts = versions_table_->item(sel.first().row(), 0)->text();
    if (QMessageBox::question(this, "回滚资产",
                              QString("把 %1 的资产回滚到版本 %2？\n回滚前的状态会存成「回滚前」版本，可再回滚撤销。")
                                  .arg(QString::fromStdString(AssetTarget::label())).arg(ts)) != QMessageBox::Yes) {
        return;
    }
    if (!rollback_cli_ || !rollback_cli_->service_is_ready()) { setStatus("rollback_assets 不可用", true); return; }
    auto req = std::make_shared<nav_interfaces::srv::RollbackAssets::Request>();
    req->map_name = AssetTarget::mapName();
    req->version_ts = ts.toStdString();
    req->expected_revision = last_revision_;
    rollback_cli_->async_send_request(req, [this](rclcpp::Client<nav_interfaces::srv::RollbackAssets>::SharedFuture fut) {
        auto resp = fut.get();
        QMetaObject::invokeMethod(this, [this, resp]() {
            setStatus(QString("[%1] %2").arg(resp->error_code).arg(QString::fromStdString(resp->message)), !resp->success);
            refreshTabs();
            refreshTargetInfo();
        }, Qt::QueuedConnection);
    });
}

// ── 非当前地图预览 ─────────────────────────────────────────────────

bool MapAssetManagerPanel::loadTargetGrid(const std::string& map_name, nav_msgs::msg::OccupancyGrid& grid,
                                          QString& err) const {
    if (map_folder_.empty()) { err = "未收到 map_state，不知道地图目录"; return false; }
    QString yaml = QString::fromStdString(map_folder_ + "/" + map_name + "/slam/map.yaml");
    if (!QFile::exists(yaml)) yaml = QString::fromStdString(map_folder_ + "/" + map_name + "/map.yaml");
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
    grid.header.stamp = node_->now();
    grid.info.resolution = static_cast<float>(res);
    grid.info.width = img.width();
    grid.info.height = img.height();
    grid.info.origin.position.x = ox;
    grid.info.origin.position.y = oy;
    // 与 map_manager 发 /map 同一规则：原点 yaw 写进 origin 朝向，同一张图两者逐格重合
    grid.info.origin.orientation.z = std::sin(oyaw / 2.0);
    grid.info.origin.orientation.w = std::cos(oyaw / 2.0);
    grid.data.assign(static_cast<size_t>(img.width()) * img.height(), -1);
    for (int r = 0; r < img.height(); ++r) {
        const uchar* line = img.constScanLine(r);
        for (int c = 0; c < img.width(); ++c) {
            const double p = negate ? line[c] / 255.0 : (255.0 - line[c]) / 255.0;
            int8_t v = -1;
            if (p > occ_th) v = 100;
            else if (p < free_th) v = 0;
            grid.data[static_cast<size_t>(img.height() - 1 - r) * img.width() + c] = v;
        }
    }
    return true;
}

void MapAssetManagerPanel::publishPreview(const nav_interfaces::srv::GetAssets::Response& r) {
    nav_msgs::msg::OccupancyGrid grid;
    QString err;
    if (loadTargetGrid(r.map_name, grid, err)) preview_map_pub_->publish(grid);
    else setStatus("预览底图：" + err, true);

    std::unordered_set<std::string> bad_points, bad_edges;
    for (const auto& i : r.issues) {
        if (i.kind == "point") bad_points.insert(i.key);
        if (i.kind == "edge") bad_edges.insert(i.key);
    }
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);
    int id = 0;
    auto base = [&](const char* ns, int type) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = node_->now();
        m.ns = ns;
        m.id = id++;
        m.type = type;
        m.pose.orientation.w = 1.0;
        return m;
    };
    std::unordered_map<std::string, std::pair<double, double>> pos;
    for (const auto& p : r.points) {
        pos[p.name] = {p.x, p.y};
        auto m = base("points", visualization_msgs::msg::Marker::ARROW);
        m.pose.position = P(p.x, p.y);
        m.pose.orientation.z = std::sin(p.theta / 2.0);
        m.pose.orientation.w = std::cos(p.theta / 2.0);
        m.scale.x = 0.5;
        m.scale.y = 0.08;
        m.scale.z = 0.08;
        m.color = bad_points.count(p.name) ? Color(0.9f, 0.1f, 0.1f) : Color(0.1f, 0.7f, 0.2f);
        arr.markers.push_back(m);
        auto t = base("point_names", visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
        t.pose.position = P(p.x, p.y, 0.4);
        t.scale.z = 0.25;
        t.color = Color(1, 1, 1);
        t.text = p.name;
        arr.markers.push_back(t);
    }
    for (const auto& j : r.junctions) {
        pos[j.id] = {j.x, j.y};
        auto m = base("junctions", visualization_msgs::msg::Marker::SPHERE);
        m.pose.position = P(j.x, j.y);
        m.scale.x = m.scale.y = m.scale.z = 0.2;
        m.color = Color(0.2f, 0.5f, 1.0f);
        arr.markers.push_back(m);
    }
    for (const auto& e : r.edges) {
        auto a = pos.find(e.from_ep.id), b = pos.find(e.to_ep.id);
        if (a == pos.end() || b == pos.end()) continue;
        auto m = base("edges", visualization_msgs::msg::Marker::LINE_STRIP);
        m.scale.x = e.lane_width > 0.0f ? e.lane_width : 0.05;
        m.color = bad_edges.count(e.id) ? Color(0.9f, 0.1f, 0.1f, 0.6f) : Color(1.0f, 0.8f, 0.1f, 0.6f);
        std::vector<std::pair<double, double>> ctrl;
        if (e.edge_type == "BEZIER" && e.control_points.size() >= 2) {
            for (const auto& c : e.control_points) ctrl.emplace_back(c.x, c.y);
            ctrl.front() = a->second;
            ctrl.back() = b->second;
        } else {
            ctrl = {a->second, b->second};
        }
        const int n = ctrl.size() > 2 ? 40 : 1;
        for (int i = 0; i <= n; ++i) {
            auto tmp = ctrl;
            const double t = static_cast<double>(i) / n;
            for (size_t k = tmp.size() - 1; k > 0; --k)
                for (size_t q = 0; q < k; ++q)
                    tmp[q] = {tmp[q].first + (tmp[q + 1].first - tmp[q].first) * t,
                              tmp[q].second + (tmp[q + 1].second - tmp[q].second) * t};
            m.points.push_back(P(tmp[0].first, tmp[0].second, 0.02));
        }
        arr.markers.push_back(m);
    }
    // 轮廓 + 名称标签（与当前地图上 map_asset_manager 发布的区域 / 房间标签同格式：名称换行类型）
    auto polygon = [&](const char* ns, const char* label_ns, const std::vector<geometry_msgs::msg::Pose2D>& poly,
                       std_msgs::msg::ColorRGBA c, bool closed, const std::string& label, double label_z) {
        if (poly.size() < 2) return;
        auto m = base(ns, visualization_msgs::msg::Marker::LINE_STRIP);
        m.scale.x = 0.05;
        m.color = c;
        double cx = 0.0, cy = 0.0;
        for (const auto& v : poly) {
            m.points.push_back(P(v.x, v.y, 0.03));
            cx += v.x;
            cy += v.y;
        }
        if (closed) m.points.push_back(P(poly.front().x, poly.front().y, 0.03));
        arr.markers.push_back(m);
        auto t = base(label_ns, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
        t.pose.position = P(cx / poly.size(), cy / poly.size(), label_z);
        t.scale.z = 0.25;
        t.color = c;
        t.color.a = 1.0f;
        t.text = label;
        arr.markers.push_back(t);
    };
    for (const auto& a : r.areas) {
        polygon("areas", "area_names", a.polygon,
                a.type == nav_interfaces::msg::AreaInfo::RESTRICTED_AREA ? Color(0.9f, 0.1f, 0.1f) : Color(0.2f, 0.6f, 1.0f),
                a.type != nav_interfaces::msg::AreaInfo::VIRTUAL_WALL,  // 虚拟墙是折线，不闭合
                a.name + "\n" + map_asset_manager::AreaType::toString(a.type), 0.3);
    }
    for (const auto& rm : r.rooms) {
        polygon("rooms", "room_names", rm.polygon, Color(0.6f, 0.3f, 0.9f), true,
                (rm.display_label.empty() ? rm.name : rm.display_label) + "\n" +
                    map_asset_manager::RoomType::toString(rm.type), 0.5);
    }
    preview_assets_pub_->publish(arr);
}

void MapAssetManagerPanel::clearPreview() {
    if (!preview_assets_pub_) return;
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);
    preview_assets_pub_->publish(arr);
    // 底图预览不能发 0×0 空图：RViz 的 Map 显示收到空图不清旧贴图，却会按空图的原点 (0,0) 重新摆放，
    // 旧贴图就以错位的「重影」留在画面上。改发一张 1×1 的未知格占位（透明度下几乎不可见），
    // 并且编辑目标是当前地图时预览显示本身也关掉（applyDisplayFocus）
    nav_msgs::msg::OccupancyGrid placeholder;
    placeholder.header.frame_id = "map";
    placeholder.header.stamp = node_->now();
    placeholder.info.resolution = 0.05f;
    placeholder.info.width = 1;
    placeholder.info.height = 1;
    placeholder.info.origin.orientation.w = 1.0;
    placeholder.data = {-1};
    preview_map_pub_->publish(placeholder);
}

// ── 工具激活 ───────────────────────────────────────────────────────

void MapAssetManagerPanel::activateTool(const QString& class_id) {
    auto* tm = getDisplayContext()->getToolManager();
    for (int i = 0; i < tm->numTools(); ++i) {
        auto* tool = tm->getTool(i);
        if (tool->getClassId() == class_id) {
            tm->setCurrentTool(tool);
            return;
        }
    }
    if (auto* tool = tm->addTool(class_id)) tm->setCurrentTool(tool);
}

void MapAssetManagerPanel::onAddNavPointTool() { activateTool(kAddNavPointToolId); tabs_->setCurrentWidget(nav_point_tab_); }
void MapAssetManagerPanel::onAddAreaTool() { activateTool(kAddAreaToolId); tabs_->setCurrentWidget(area_tab_); }
void MapAssetManagerPanel::onAddRoomTool() { activateTool(kAddRoomToolId); tabs_->setCurrentWidget(room_tab_); }
void MapAssetManagerPanel::onAddJunctionTool() { activateTool(kAddJunctionToolId); tabs_->setCurrentWidget(route_graph_tab_); }
void MapAssetManagerPanel::onAddEdgeTool() { activateTool(kAddEdgeToolId); tabs_->setCurrentWidget(route_graph_tab_); }
void MapAssetManagerPanel::onAddCurveEdgeTool() { activateTool(kAddCurveEdgeToolId); tabs_->setCurrentWidget(route_graph_tab_); }
void MapAssetManagerPanel::onRouteGraphDeleteTool() { activateTool("map_asset_rviz_plugin/RouteGraphDeleteTool"); tabs_->setCurrentWidget(route_graph_tab_); }
void MapAssetManagerPanel::onRouteGraphStartTool() { activateTool("map_asset_rviz_plugin/RouteGraphStartTool"); }
void MapAssetManagerPanel::onRouteGraphSaveTool() { activateTool("map_asset_rviz_plugin/RouteGraphSaveTool"); }

// ── 配置 ───────────────────────────────────────────────────────────

void MapAssetManagerPanel::save(rviz_common::Config config) const {
    Panel::save(config);
    config.mapSetValue("service_prefix", prefix_edit_->text());
    config.mapSetValue("map_manager_prefix", mm_prefix_edit_->text());
    config.mapSetValue("target_map", QString::fromStdString(AssetTarget::mapName()));
    config.mapSetValue("focus_target", focus_check_->isChecked());
}

void MapAssetManagerPanel::load(const rviz_common::Config& config) {
    Panel::load(config);
    QString v;
    if (config.mapGetString("service_prefix", &v)) prefix_edit_->setText(v);
    if (config.mapGetString("map_manager_prefix", &v)) mm_prefix_edit_->setText(v);
    if (config.mapGetString("target_map", &v)) pending_target_ = v.toStdString();
    bool focus = true;
    if (config.mapGetBool("focus_target", &focus)) focus_check_->setChecked(focus);
    if (node_) {
        recreateClients();
        refreshMapList();
    }
}

}  // namespace map_asset_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(map_asset_rviz_plugin::MapAssetManagerPanel, rviz_common::Panel)
