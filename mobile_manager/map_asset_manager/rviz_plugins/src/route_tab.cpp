#include "map_asset_rviz_plugin/route_tab.hpp"

#include <algorithm>
#include "map_asset_rviz_plugin/asset_target.hpp"

#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QVBoxLayout>

namespace map_asset_rviz_plugin {

RouteTab::RouteTab(QWidget* parent) : QWidget(parent) {
    // ── 已有路线列表 ──────────────────────────────────────────────
    auto* routes_box = new QGroupBox("已有路线");
    route_list_ = new QListWidget();
    route_list_->setSelectionMode(QAbstractItemView::SingleSelection);
    route_detail_label_ = new QLabel();
    route_detail_label_->setWordWrap(true);
    auto* routes_vbox = new QVBoxLayout(routes_box);
    routes_vbox->addWidget(route_list_);
    routes_vbox->addWidget(route_detail_label_);

    // ── 路线编辑区 ────────────────────────────────────────────────
    auto* edit_box = new QGroupBox("创建新路线");

    name_edit_ = new QLineEdit();
    name_edit_->setPlaceholderText("路线名称");

    // 可选点列表（左）
    point_list_ = new QListWidget();
    point_list_->setSelectionMode(QAbstractItemView::ExtendedSelection);
    auto* points_label = new QLabel("可用点位：");

    // 已选顺序列表（右）
    selected_list_ = new QListWidget();
    selected_list_->setSelectionMode(QAbstractItemView::SingleSelection);
    auto* selected_label = new QLabel("已选顺序：");

    // 中间操作按钮
    add_point_btn_    = new QPushButton(">>");
    remove_point_btn_ = new QPushButton("<<");
    up_btn_           = new QPushButton("↑");
    down_btn_         = new QPushButton("↓");
    auto* mid_vbox    = new QVBoxLayout();
    mid_vbox->addStretch();
    mid_vbox->addWidget(add_point_btn_);
    mid_vbox->addWidget(remove_point_btn_);
    mid_vbox->addSpacing(10);
    mid_vbox->addWidget(up_btn_);
    mid_vbox->addWidget(down_btn_);
    mid_vbox->addStretch();

    auto* list_left  = new QVBoxLayout();
    list_left->addWidget(points_label);
    list_left->addWidget(point_list_);

    auto* list_right = new QVBoxLayout();
    list_right->addWidget(selected_label);
    list_right->addWidget(selected_list_);

    auto* lists_hbox = new QHBoxLayout();
    lists_hbox->addLayout(list_left);
    lists_hbox->addLayout(mid_vbox);
    lists_hbox->addLayout(list_right);

    auto* edit_vbox = new QVBoxLayout(edit_box);
    edit_vbox->addWidget(new QLabel("路线名称:"));
    edit_vbox->addWidget(name_edit_);
    edit_vbox->addLayout(lists_hbox);

    // ── 按钮行 ────────────────────────────────────────────────────
    refresh_routes_btn_ = new QPushButton("刷新路线");
    refresh_points_btn_ = new QPushButton("刷新点位");
    create_btn_         = new QPushButton("创建路线");
    remove_route_btn_   = new QPushButton("删除选中路线");

    auto* btn_row = new QHBoxLayout();
    btn_row->addWidget(refresh_routes_btn_);
    btn_row->addWidget(refresh_points_btn_);
    btn_row->addWidget(create_btn_);
    btn_row->addWidget(remove_route_btn_);

    // ── 状态 ──────────────────────────────────────────────────────
    status_label_ = new QLabel("就绪");
    status_label_->setWordWrap(true);

    // ── 总布局 ────────────────────────────────────────────────────
    auto* root = new QVBoxLayout(this);
    root->addWidget(routes_box);
    root->addWidget(edit_box);
    root->addLayout(btn_row);
    root->addWidget(status_label_);

    // ── 信号连接 ──────────────────────────────────────────────────
    connect(refresh_routes_btn_, &QPushButton::clicked, this, &RouteTab::onRefreshRoutes);
    connect(refresh_points_btn_, &QPushButton::clicked, this, &RouteTab::onRefreshPoints);
    connect(create_btn_,         &QPushButton::clicked, this, &RouteTab::onCreate);
    connect(remove_route_btn_,   &QPushButton::clicked, this, &RouteTab::onRemove);

    connect(route_list_, &QListWidget::itemClicked, this, &RouteTab::onRouteSelected);

    connect(add_point_btn_, &QPushButton::clicked, this, [this]() {
        for (auto* item : point_list_->selectedItems()) {
            // 避免重复
            auto found = selected_list_->findItems(item->text(), Qt::MatchExactly);
            if (found.isEmpty()) {
                selected_list_->addItem(item->text());
            }
        }
    });
    connect(remove_point_btn_, &QPushButton::clicked, this, [this]() {
        qDeleteAll(selected_list_->selectedItems());
    });
    connect(up_btn_, &QPushButton::clicked, this, [this]() {
        int row = selected_list_->currentRow();
        if (row <= 0) return;
        auto* item = selected_list_->takeItem(row);
        selected_list_->insertItem(row - 1, item);
        selected_list_->setCurrentRow(row - 1);
    });
    connect(down_btn_, &QPushButton::clicked, this, [this]() {
        int row = selected_list_->currentRow();
        if (row < 0 || row >= selected_list_->count() - 1) return;
        auto* item = selected_list_->takeItem(row);
        selected_list_->insertItem(row + 1, item);
        selected_list_->setCurrentRow(row + 1);
    });

    connect(this, &RouteTab::refreshDone,          this, &RouteTab::applyRefreshRoutes,  Qt::QueuedConnection);
    connect(this, &RouteTab::operationFinished,    this, &RouteTab::applyOperationResult, Qt::QueuedConnection);
    connect(this, &RouteTab::pointListRefreshDone, this, &RouteTab::applyPointListRefresh, Qt::QueuedConnection);
}

void RouteTab::setNode(rclcpp::Node::SharedPtr node) {
    node_ = node;
    rebuildClients();
}

void RouteTab::setServicePrefix(const std::string& prefix) {
    prefix_ = prefix;
    if (node_) rebuildClients();
}

void RouteTab::setAvailablePoints(const QStringList& names) {
    point_list_->clear();
    point_list_->addItems(names);
}

void RouteTab::rebuildClients() {
    add_client_              = node_->create_client<nav_interfaces::srv::SetAssets>(prefix_ + "/set_assets");
    remove_client_           = node_->create_client<nav_interfaces::srv::SetAssets>(prefix_ + "/set_assets");
    get_client_              = node_->create_client<nav_interfaces::srv::GetAssets>(prefix_ + "/get_assets");
    get_all_routes_client_   = node_->create_client<nav_interfaces::srv::GetAssets>(prefix_ + "/get_assets");
    get_points_client_       = node_->create_client<nav_interfaces::srv::GetAssets>(prefix_ + "/get_assets");
}

// ──────────────────────────────────────────────────────────────────────

void RouteTab::onRefreshRoutes() {
    if (!get_all_routes_client_) { status_label_->setText("节点未初始化"); return; }
    refresh_routes_btn_->setEnabled(false);
    status_label_->setText("刷新路线...");

    auto req = std::make_shared<nav_interfaces::srv::GetAssets::Request>(); applyTarget(req);
    req->kinds = nav_interfaces::srv::GetAssets::Request::KIND_ROUTES;
    get_all_routes_client_->async_send_request(
        req,
        [this](rclcpp::Client<nav_interfaces::srv::GetAssets>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                QStringList items;
                for (const auto& r : resp->routes) {
                    QString pts;
                    for (size_t i = 0; i < r.point_names.size(); ++i) {
                        if (i) pts += " → ";
                        pts += QString::fromStdString(r.point_names[i]);
                    }
                    items << QString("%1  [%2]")
                                 .arg(QString::fromStdString(r.name))
                                 .arg(pts);
                }
                Q_EMIT refreshDone(items);
            } catch (const std::exception& ex) {
                Q_EMIT operationFinished(false, QString("刷新路线失败: %1").arg(ex.what()));
            }
        });
}

void RouteTab::onRefreshPoints() {
    if (!get_points_client_) { status_label_->setText("节点未初始化"); return; }
    status_label_->setText("刷新点位...");
    auto req = std::make_shared<nav_interfaces::srv::GetAssets::Request>(); applyTarget(req);
    req->kinds = nav_interfaces::srv::GetAssets::Request::KIND_POINTS;
    get_points_client_->async_send_request(
        req,
        [this](rclcpp::Client<nav_interfaces::srv::GetAssets>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                QStringList names;
                for (const auto& pt : resp->points) {
                    names << QString::fromStdString(pt.name);
                }
                Q_EMIT pointListRefreshDone(names);
            } catch (const std::exception& ex) {
                Q_EMIT operationFinished(false, QString("刷新点位失败: %1").arg(ex.what()));
            }
        });
}

void RouteTab::onCreate() {
    if (!add_client_) { status_label_->setText("节点未初始化"); return; }
    const std::string name = name_edit_->text().trimmed().toStdString();
    if (name.empty()) { status_label_->setText("错误：请填写路线名称"); return; }
    if (selected_list_->count() == 0) { status_label_->setText("错误：请至少选择一个点位"); return; }

    auto req = std::make_shared<nav_interfaces::srv::SetAssets::Request>(); applyTarget(req); req->routes.resize(1);
    req->mode = nav_interfaces::srv::SetAssets::Request::MODE_ADD_ONLY;
    req->routes[0].name = name;
    for (int i = 0; i < selected_list_->count(); ++i) {
        req->routes[0].point_names.push_back(selected_list_->item(i)->text().toStdString());
    }

    create_btn_->setEnabled(false);
    status_label_->setText("创建路线中...");

    add_client_->async_send_request(
        req,
        [this](rclcpp::Client<nav_interfaces::srv::SetAssets>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                Q_EMIT operationFinished(resp->success,
                    resp->success ? "路线创建成功" : QString("路线创建失败: %1").arg(
                        assetMessage(*resp)));
            } catch (const std::exception& ex) {
                Q_EMIT operationFinished(false, QString("创建异常: %1").arg(ex.what()));
            }
        });
}

void RouteTab::onRemove() {
    if (!remove_client_) { status_label_->setText("节点未初始化"); return; }
    auto* sel = route_list_->currentItem();
    if (!sel) { status_label_->setText("请先选择要删除的路线"); return; }

    // 条目格式: "route_name  [点1 → 点2]"，提取名称部分
    const QString full_text = sel->text();
    const int sep = full_text.indexOf("  [");
    const QString name = (sep >= 0) ? full_text.left(sep) : full_text;

    if (QMessageBox::question(this, "确认删除",
            QString("确定删除路线 \"%1\"？").arg(name),
            QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes) {
        return;
    }

    auto req = std::make_shared<nav_interfaces::srv::SetAssets::Request>(); applyTarget(req); req->routes.resize(1);
    req->mode = nav_interfaces::srv::SetAssets::Request::MODE_REMOVE;
    req->routes[0].name = name.toStdString();

    remove_route_btn_->setEnabled(false);
    status_label_->setText("删除路线中...");

    remove_client_->async_send_request(
        req,
        [this](rclcpp::Client<nav_interfaces::srv::SetAssets>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                Q_EMIT operationFinished(resp->success,
                    resp->success ? "路线删除成功" : QString("路线删除失败: %1").arg(
                        assetMessage(*resp)));
            } catch (const std::exception& ex) {
                Q_EMIT operationFinished(false, QString("删除异常: %1").arg(ex.what()));
            }
        });
}

void RouteTab::onRouteSelected(QListWidgetItem* item) {
    if (!get_client_ || !item) return;

    const QString full_text = item->text();
    const int sep = full_text.indexOf("  [");
    const QString name = (sep >= 0) ? full_text.left(sep) : full_text;

    auto req = std::make_shared<nav_interfaces::srv::GetAssets::Request>(); applyTarget(req);
    req->kinds = nav_interfaces::srv::GetAssets::Request::KIND_ROUTES;

    get_client_->async_send_request(
        req,
        [this, route = name.toStdString()](rclcpp::Client<nav_interfaces::srv::GetAssets>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                const auto it = std::find_if(resp->routes.begin(), resp->routes.end(),
                                             [&](const auto& r) { return r.name == route; });
                if (resp->success && it != resp->routes.end()) {
                    QStringList pts;
                    for (const auto& p : it->point_names) {
                        pts << QString::fromStdString(p);
                    }
                    QString detail = QString("路线点位: %1").arg(pts.join(" → "));
                    QMetaObject::invokeMethod(route_detail_label_, "setText",
                        Qt::QueuedConnection, Q_ARG(QString, detail));
                }
            } catch (...) {}
        });
}

void RouteTab::applyRefreshRoutes(QStringList routes) {
    route_list_->clear();
    route_list_->addItems(routes);
    refresh_routes_btn_->setEnabled(true);
    status_label_->setText(QString("共 %1 条路线").arg(routes.size()));
}

void RouteTab::applyOperationResult(bool /*success*/, QString msg) {
    status_label_->setText(msg);
    refresh_routes_btn_->setEnabled(true);
    create_btn_->setEnabled(true);
    remove_route_btn_->setEnabled(true);
}

void RouteTab::applyPointListRefresh(QStringList points) {
    point_list_->clear();
    point_list_->addItems(points);
    status_label_->setText(QString("已加载 %1 个可用点位").arg(points.size()));
}

}  // namespace map_asset_rviz_plugin
