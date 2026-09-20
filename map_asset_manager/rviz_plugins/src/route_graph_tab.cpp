#include "map_asset_rviz_plugin/route_graph_tab.hpp"

#include <QGroupBox>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QMessageBox>
#include <QVBoxLayout>

namespace map_asset_rviz_plugin {

RouteGraphTab::RouteGraphTab(QWidget* parent) : QWidget(parent) {
    // ── Junction 列表 ──────────────────────────────────────────────
    auto* jct_box = new QGroupBox("Junction 节点");
    junction_list_ = new QListWidget();
    junction_list_->setSelectionMode(QAbstractItemView::SingleSelection);
    remove_junction_btn_ = new QPushButton("删除选中 Junction");
    auto* jct_vbox = new QVBoxLayout(jct_box);
    jct_vbox->addWidget(junction_list_);
    jct_vbox->addWidget(remove_junction_btn_);

    // ── Edge 列表 ──────────────────────────────────────────────────
    auto* edge_box = new QGroupBox("路网边");
    edge_list_ = new QListWidget();
    edge_list_->setSelectionMode(QAbstractItemView::SingleSelection);
    remove_edge_btn_ = new QPushButton("删除选中边");
    auto* edge_vbox = new QVBoxLayout(edge_box);
    edge_vbox->addWidget(edge_list_);
    edge_vbox->addWidget(remove_edge_btn_);

    // ── 按钮行 ────────────────────────────────────────────────────
    refresh_btn_ = new QPushButton("刷新路网");

    // ── 状态 ──────────────────────────────────────────────────────
    status_label_ = new QLabel("就绪");
    status_label_->setWordWrap(true);

    // ── 总布局 ────────────────────────────────────────────────────
    auto* root = new QVBoxLayout(this);
    root->addWidget(jct_box);
    root->addWidget(edge_box);
    root->addWidget(refresh_btn_);
    root->addWidget(status_label_);

    // ── 信号连接 ──────────────────────────────────────────────────
    connect(refresh_btn_,        &QPushButton::clicked, this, &RouteGraphTab::onRefresh);
    connect(remove_edge_btn_,    &QPushButton::clicked, this, &RouteGraphTab::onRemoveEdge);
    connect(remove_junction_btn_,&QPushButton::clicked, this, &RouteGraphTab::onRemoveJunction);

    connect(this, &RouteGraphTab::statusChanged, status_label_, &QLabel::setText, Qt::QueuedConnection);
}

void RouteGraphTab::setNode(rclcpp::Node::SharedPtr node) {
    node_ = node;
    rebuildClients();
}

void RouteGraphTab::setServicePrefix(const std::string& prefix) {
    prefix_ = prefix;
    if (node_) rebuildClients();
}

void RouteGraphTab::rebuildClients() {
    get_graph_client_      = node_->create_client<nav_interfaces::srv::GetRouteGraph>(
        prefix_ + "/get_route_graph");
    remove_edge_client_    = node_->create_client<nav_interfaces::srv::RemoveRouteEdge>(
        prefix_ + "/remove_route_edge");
    remove_junction_client_= node_->create_client<nav_interfaces::srv::RemoveRouteJunction>(
        prefix_ + "/remove_route_junction");
}

void RouteGraphTab::onRefresh() {
    if (!get_graph_client_) { Q_EMIT statusChanged("节点未初始化"); return; }
    refresh_btn_->setEnabled(false);
    Q_EMIT statusChanged("刷新路网...");

    auto req = std::make_shared<nav_interfaces::srv::GetRouteGraph::Request>();
    get_graph_client_->async_send_request(
        req,
        [this](rclcpp::Client<nav_interfaces::srv::GetRouteGraph>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                QStringList jcts, edges;
                for (const auto& j : resp->junctions) {
                    jcts << QString("[%1]  x=%2 y=%3 yaw=%4")
                                .arg(QString::fromStdString(j.id))
                                .arg(j.x, 0, 'f', 2)
                                .arg(j.y, 0, 'f', 2)
                                .arg(j.yaw, 0, 'f', 2);
                }
                for (const auto& e : resp->edges) {
                    QString line = QString("[%1]  %2:%3 → %4:%5  [%6%7]")
                        .arg(QString::fromStdString(e.id))
                        .arg(QString::fromStdString(e.from_ep.type))
                        .arg(QString::fromStdString(e.from_ep.id))
                        .arg(QString::fromStdString(e.to_ep.type))
                        .arg(QString::fromStdString(e.to_ep.id))
                        .arg(QString::fromStdString(e.edge_type))
                        .arg(e.bidirectional ? " ↔" : "");
                    edges << line;
                }
                Q_EMIT statusChanged(QString("共 %1 junctions, %2 edges")
                    .arg(jcts.size()).arg(edges.size()));
                QMetaObject::invokeMethod(this, [this, jcts, edges]() {
                    junction_list_->clear();
                    junction_list_->addItems(jcts);
                    edge_list_->clear();
                    edge_list_->addItems(edges);
                    refresh_btn_->setEnabled(true);
                }, Qt::QueuedConnection);
            } catch (const std::exception& ex) {
                Q_EMIT statusChanged(QString("刷新失败: %1").arg(ex.what()));
                QMetaObject::invokeMethod(refresh_btn_, [this](){ refresh_btn_->setEnabled(true); }, Qt::QueuedConnection);
            }
        });
}

void RouteGraphTab::onRemoveEdge() {
    if (!remove_edge_client_) { Q_EMIT statusChanged("节点未初始化"); return; }
    auto* sel = edge_list_->currentItem();
    if (!sel) { Q_EMIT statusChanged("请先选择要删除的边"); return; }

    // 条目格式: "edge_id  from_type:from_id → ..."
    const QString full = sel->text();
    const QString id = full.left(full.indexOf("  "));

    if (QMessageBox::question(this, "确认删除",
            QString("确定删除边 \"%1\"？").arg(id),
            QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes) {
        return;
    }

    auto req = std::make_shared<nav_interfaces::srv::RemoveRouteEdge::Request>();
    req->id = id.toStdString();
    remove_edge_client_->async_send_request(req,
        [this](rclcpp::Client<nav_interfaces::srv::RemoveRouteEdge>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                Q_EMIT statusChanged(resp->success
                    ? "边删除成功"
                    : QString("删除失败: %1").arg(QString::fromStdString(resp->message)));
                if (resp->success) {
                    QMetaObject::invokeMethod(this, &RouteGraphTab::onRefresh, Qt::QueuedConnection);
                }
            } catch (const std::exception& ex) {
                Q_EMIT statusChanged(QString("删除异常: %1").arg(ex.what()));
            }
        });
}

void RouteGraphTab::onRemoveJunction() {
    if (!remove_junction_client_) { Q_EMIT statusChanged("节点未初始化"); return; }
    auto* sel = junction_list_->currentItem();
    if (!sel) { Q_EMIT statusChanged("请先选择要删除的 Junction"); return; }

    // 条目格式: "[jct_id]  x=... y=..."
    const QString full = sel->text();
    const QString id = full.mid(1, full.indexOf("]") - 1);

    if (QMessageBox::question(this, "确认删除",
            QString("确定删除 Junction \"%1\"（及关联的路网边）？").arg(id),
            QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes) {
        return;
    }

    auto req = std::make_shared<nav_interfaces::srv::RemoveRouteJunction::Request>();
    req->id = id.toStdString();
    remove_junction_client_->async_send_request(req,
        [this](rclcpp::Client<nav_interfaces::srv::RemoveRouteJunction>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                Q_EMIT statusChanged(resp->success
                    ? QString("Junction 删除成功: %1").arg(QString::fromStdString(resp->message))
                    : QString("删除失败: %1").arg(QString::fromStdString(resp->message)));
                if (resp->success) {
                    QMetaObject::invokeMethod(this, &RouteGraphTab::onRefresh, Qt::QueuedConnection);
                }
            } catch (const std::exception& ex) {
                Q_EMIT statusChanged(QString("删除异常: %1").arg(ex.what()));
            }
        });
}

void RouteGraphTab::applyRefresh(QStringList junctions, QStringList edges) {
    junction_list_->clear();
    junction_list_->addItems(junctions);
    edge_list_->clear();
    edge_list_->addItems(edges);
    refresh_btn_->setEnabled(true);
}

void RouteGraphTab::applyResult(bool success, QString msg) {
    Q_EMIT statusChanged(msg);
    if (success) onRefresh();
}

}  // namespace map_asset_rviz_plugin
