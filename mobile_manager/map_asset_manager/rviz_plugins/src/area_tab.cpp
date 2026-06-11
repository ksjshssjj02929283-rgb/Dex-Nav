#include "map_asset_rviz_plugin/area_tab.hpp"

#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QVBoxLayout>

namespace map_asset_rviz_plugin {

// 类型标签（与后端 AreaType 命名空间保持一致）
QString AreaTab::typeLabel(uint32_t type) {
    switch (type) {
        case 1:
            return "虚拟墙";
        case 2:
            return "禁行区";
        case 3:
            return "减速区";
        case 4:
            return "绕障区";
        case 5:
            return "停障区";
        case 6:
            return "停车区";
        case 7:
            return "充电区";
        case 8:
            return "电梯区";
        case 9:
            return "颠簸区";
        default:
            return "未知";
    }
}

// ──────────────────────────────────────────────────────────────────────

AreaTab::AreaTab(QWidget* parent) : QWidget(parent) {
    list_ = new QListWidget();
    list_->setSelectionMode(QAbstractItemView::SingleSelection);

    detail_label_ = new QLabel();
    detail_label_->setWordWrap(true);
    detail_label_->setStyleSheet("background:#f8f8f8; padding:4px; border-radius:3px;");

    refresh_btn_ = new QPushButton("刷新列表");
    remove_btn_ = new QPushButton("删除选中");
    remove_btn_->setEnabled(false);

    auto* btn_row = new QHBoxLayout();
    btn_row->addWidget(refresh_btn_);
    btn_row->addWidget(remove_btn_);

    auto* hint = new QLabel(
        "提示：使用工具栏「添加区域」工具 (A) 在地图上绘制区域。\n"
        "左键添加顶点，右键完成绘制。");
    hint->setWordWrap(true);
    hint->setStyleSheet("color: gray; font-style: italic;");

    status_label_ = new QLabel("就绪");
    status_label_->setWordWrap(true);

    auto* root = new QVBoxLayout(this);
    root->addWidget(list_);
    root->addWidget(detail_label_);
    root->addLayout(btn_row);
    root->addWidget(hint);
    root->addWidget(status_label_);

    connect(refresh_btn_, &QPushButton::clicked, this, &AreaTab::onRefresh);
    connect(remove_btn_, &QPushButton::clicked, this, &AreaTab::onRemove);

    connect(list_, &QListWidget::currentItemChanged, this, [this](QListWidgetItem* current, QListWidgetItem*) {
        remove_btn_->setEnabled(current != nullptr);
        if (!current) {
            detail_label_->clear();
            return;
        }
        detail_label_->setText(current->data(Qt::UserRole).toString());
    });

    connect(this, &AreaTab::operationFinished, this, &AreaTab::applyOperationResult, Qt::QueuedConnection);
}

// ──────────────────────────────────────────────────────────────────────

void AreaTab::setNode(rclcpp::Node::SharedPtr node) {
    node_ = node;
    rebuildClients();
}

void AreaTab::setServicePrefix(const std::string& prefix) {
    prefix_ = prefix;
    if (node_) rebuildClients();
}

void AreaTab::rebuildClients() {
    get_client_ = node_->create_client<nav_interfaces::srv::GetAllAreas>(prefix_ + "/get_all_areas");
    remove_client_ = node_->create_client<nav_interfaces::srv::RemoveAreaInfo>(prefix_ + "/remove_area");
}

// ──────────────────────────────────────────────────────────────────────
// 刷新
// ──────────────────────────────────────────────────────────────────────

void AreaTab::onRefresh() {
    if (!get_client_) {
        status_label_->setText("节点未初始化");
        return;
    }
    refresh_btn_->setEnabled(false);
    remove_btn_->setEnabled(false);
    status_label_->setText("刷新中...");

    auto req = std::make_shared<nav_interfaces::srv::GetAllAreas::Request>();
    get_client_->async_send_request(req, [this](rclcpp::Client<nav_interfaces::srv::GetAllAreas>::SharedFuture fut) {
        try {
            auto resp = fut.get();
            QMetaObject::invokeMethod(
                this,
                [this, resp]() {
                    list_->clear();
                    detail_label_->clear();

                    for (const auto& area : resp->area_info_list) {
                        QString label = typeLabel(area.type);

                        // 构建详情文本
                        QStringList details;
                        details
                            << QString("类型: %1  顶点数: %2").arg(label).arg(static_cast<int>(area.polygon.size()));
                        if (area.type == 3) details << QString("最大速度: %1 m/s").arg(area.max_speed);
                        if (area.type == 5) details << QString("停止距离: %1 m").arg(area.stop_distance);
                        if (area.type == 6 || area.type == 7) {
                            if (!area.linked_point_name.empty())
                                details << QString("关联点: %1").arg(QString::fromStdString(area.linked_point_name));
                        }
                        if (area.type == 7) {
                            details << QString("充电方式: %1").arg(QString::fromStdString(area.charger_type));
                        }
                        if (!area.attribute.empty())
                            details << QString("属性: %1").arg(QString::fromStdString(area.attribute));

                        auto* item =
                            new QListWidgetItem(QString("[%1] %2").arg(label).arg(QString::fromStdString(area.name)));
                        item->setData(Qt::UserRole, details.join("\n"));
                        list_->addItem(item);
                    }

                    refresh_btn_->setEnabled(true);
                    status_label_->setText(QString("共 %1 个区域").arg(resp->area_info_list.size()));
                },
                Qt::QueuedConnection);
        } catch (const std::exception& ex) {
            Q_EMIT operationFinished(false, QString("刷新失败: %1").arg(ex.what()));
        }
    });
}

// ──────────────────────────────────────────────────────────────────────
// 删除
// ──────────────────────────────────────────────────────────────────────

void AreaTab::onRemove() {
    auto* item = list_->currentItem();
    if (!item) {
        status_label_->setText("请先选中一个区域");
        return;
    }

    // 从列表项文本中提取区域名称：格式为 "[类型] 名称"
    QString text = item->text();
    int bracket_end = text.indexOf("] ");
    QString area_name = (bracket_end >= 0) ? text.mid(bracket_end + 2) : text;

    if (QMessageBox::question(this, "确认删除", QString("删除区域「%1」？").arg(area_name),
                              QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes) {
        return;
    }

    if (!remove_client_) {
        status_label_->setText("节点未初始化");
        return;
    }
    remove_btn_->setEnabled(false);
    status_label_->setText("删除中...");

    auto req = std::make_shared<nav_interfaces::srv::RemoveAreaInfo::Request>();
    req->name = area_name.toStdString();

    remove_client_->async_send_request(
        req, [this](rclcpp::Client<nav_interfaces::srv::RemoveAreaInfo>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                if (resp->success) {
                    Q_EMIT operationFinished(true, "删除成功，自动刷新列表...");
                    QMetaObject::invokeMethod(this, &AreaTab::onRefresh, Qt::QueuedConnection);
                } else {
                    Q_EMIT operationFinished(false, QString("删除失败: %1").arg(QString::fromStdString(resp->message)));
                }
            } catch (const std::exception& ex) {
                Q_EMIT operationFinished(false, QString("删除异常: %1").arg(ex.what()));
            }
        });
}

// ──────────────────────────────────────────────────────────────────────

void AreaTab::applyOperationResult(bool /*success*/, QString msg) {
    status_label_->setText(msg);
    refresh_btn_->setEnabled(true);
    remove_btn_->setEnabled(list_->currentItem() != nullptr);
}

}  // namespace map_asset_rviz_plugin
