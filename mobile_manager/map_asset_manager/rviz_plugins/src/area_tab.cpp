#include "map_asset_rviz_plugin/area_tab.hpp"

#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QRegExp>
#include <QVBoxLayout>

namespace map_asset_rviz_plugin {

// 类型标签（与后端 AreaType 命名空间保持一致）
QString AreaTab::typeLabel(uint32_t type) {
    switch (type) {
        case 1: return "虚拟墙";
        case 2: return "禁行区";
        case 3: return "减速区";
        case 4: return "绕障区";
        case 5: return "停障区";
        case 6: return "停车区";
        case 7: return "充电区";
        case 8: return "电梯区";
        case 9: return "颠簸区";
        default: return "未知";
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
    edit_btn_    = new QPushButton("修改属性");
    edit_btn_->setEnabled(false);
    remove_btn_  = new QPushButton("删除选中");
    remove_btn_->setEnabled(false);

    auto* btn_row = new QHBoxLayout();
    btn_row->addWidget(refresh_btn_);
    btn_row->addWidget(edit_btn_);
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
    connect(edit_btn_,    &QPushButton::clicked, this, &AreaTab::onEdit);
    connect(remove_btn_,  &QPushButton::clicked, this, &AreaTab::onRemove);

    connect(list_, &QListWidget::currentItemChanged, this,
        [this](QListWidgetItem* current, QListWidgetItem*) {
            const bool has_sel = (current != nullptr);
            remove_btn_->setEnabled(has_sel);
            edit_btn_->setEnabled(has_sel);
            if (!current) { detail_label_->clear(); return; }
            detail_label_->setText(current->data(Qt::UserRole + 1).toString());
        });

    connect(this, &AreaTab::operationFinished, this, &AreaTab::applyOperationResult,
            Qt::QueuedConnection);
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
    get_client_    = node_->create_client<nav_interfaces::srv::GetAllAreas>(prefix_ + "/get_all_areas");
    remove_client_ = node_->create_client<nav_interfaces::srv::RemoveAreaInfo>(prefix_ + "/remove_area");
    update_client_ = node_->create_client<nav_interfaces::srv::UpdateAreaInfo>(prefix_ + "/update_area");
}

// ──────────────────────────────────────────────────────────────────────
// 刷新
// ──────────────────────────────────────────────────────────────────────

void AreaTab::onRefresh() {
    if (!get_client_) { status_label_->setText("节点未初始化"); return; }
    refresh_btn_->setEnabled(false);
    remove_btn_->setEnabled(false);
    status_label_->setText("刷新中...");

    auto req = std::make_shared<nav_interfaces::srv::GetAllAreas::Request>();
    get_client_->async_send_request(
        req,
        [this](rclcpp::Client<nav_interfaces::srv::GetAllAreas>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                QMetaObject::invokeMethod(this, [this, resp]() {
                    list_->clear();
                    detail_label_->clear();

                    for (const auto& area : resp->area_info_list) {
                        QString label = typeLabel(area.type);

                        // 构建详情文本
                        QStringList details;
                        details << QString("类型: %1  顶点数: %2")
                                       .arg(label)
                                       .arg(static_cast<int>(area.polygon.size()));
                        if (area.type == 3)
                            details << QString("最大速度: %1 m/s").arg(area.max_speed);
                        if (area.type == 5)
                            details << QString("停止距离: %1 m").arg(area.stop_distance);
                        if (area.type == 6 || area.type == 7) {
                            if (!area.manual_linked_point_names.empty()) {
                                QStringList linked;
                                for (const auto& n : area.manual_linked_point_names)
                                    linked << QString::fromStdString(n);
                                details << QString("手工关联点: %1").arg(linked.join(", "));
                            }
                        }
                        if (area.type == 7) {
                            details << QString("充电方式: %1").arg(
                                QString::fromStdString(area.charger_type));
                        }
                        if (!area.attribute.empty())
                            details << QString("属性: %1").arg(
                                QString::fromStdString(area.attribute));

                        // 自动空间归属（SpatialIndexer 维护）
                        if (!area.related_nav_point_names.empty()) {
                            QStringList items;
                            for (const auto& n : area.related_nav_point_names)
                                items << QString::fromStdString(n);
                            details << QString("自动归属 NavPoint (%1): %2")
                                            .arg(items.size())
                                            .arg(items.join(", "));
                        }
                        if (!area.related_junction_ids.empty()) {
                            QStringList items;
                            for (const auto& n : area.related_junction_ids)
                                items << QString::fromStdString(n);
                            details << QString("自动归属 Junction (%1): %2")
                                            .arg(items.size())
                                            .arg(items.join(", "));
                        }

                        // 完整数据快照，供 onEdit 使用（UserRole）
                        QVariantMap snapshot;
                        snapshot["name"]             = QString::fromStdString(area.name);
                        snapshot["type"]             = area.type;
                        snapshot["max_speed"]        = static_cast<double>(area.max_speed);
                        snapshot["stop_distance"]    = static_cast<double>(area.stop_distance);
                        snapshot["depth_min_height"] = static_cast<double>(area.depth_min_height);
                        snapshot["charger_type"]     = QString::fromStdString(area.charger_type);
                        snapshot["attribute"]        = QString::fromStdString(area.attribute);
                        QStringList linked;
                        for (const auto& n : area.manual_linked_point_names)
                            linked << QString::fromStdString(n);
                        snapshot["manual_linked"]    = linked;
                        QVariantList poly;
                        for (const auto& pose : area.polygon) {
                            QVariantMap p;
                            p["x"]     = static_cast<double>(pose.x);
                            p["y"]     = static_cast<double>(pose.y);
                            p["theta"] = static_cast<double>(pose.theta);
                            poly.append(p);
                        }
                        snapshot["polygon"]          = poly;

                        auto* item = new QListWidgetItem(
                            QString("[%1] %2").arg(label).arg(
                                QString::fromStdString(area.name)));
                        item->setData(Qt::UserRole,     snapshot);             // 完整快照
                        item->setData(Qt::UserRole + 1, details.join("\n"));   // 详情字符串
                        list_->addItem(item);
                    }

                    refresh_btn_->setEnabled(true);
                    status_label_->setText(
                        QString("共 %1 个区域").arg(resp->area_info_list.size()));
                }, Qt::QueuedConnection);
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
    if (!item) { status_label_->setText("请先选中一个区域"); return; }

    // 从列表项文本中提取区域名称：格式为 "[类型] 名称"
    QString text = item->text();
    int bracket_end = text.indexOf("] ");
    QString area_name = (bracket_end >= 0) ? text.mid(bracket_end + 2) : text;

    if (QMessageBox::question(this, "确认删除",
                              QString("删除区域「%1」？").arg(area_name),
                              QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes) {
        return;
    }

    if (!remove_client_) { status_label_->setText("节点未初始化"); return; }
    remove_btn_->setEnabled(false);
    status_label_->setText("删除中...");

    auto req = std::make_shared<nav_interfaces::srv::RemoveAreaInfo::Request>();
    req->name = area_name.toStdString();

    remove_client_->async_send_request(
        req,
        [this](rclcpp::Client<nav_interfaces::srv::RemoveAreaInfo>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                if (resp->success) {
                    Q_EMIT operationFinished(true, "删除成功，自动刷新列表...");
                    QMetaObject::invokeMethod(this, &AreaTab::onRefresh, Qt::QueuedConnection);
                } else {
                    Q_EMIT operationFinished(false,
                        QString("删除失败: %1").arg(QString::fromStdString(resp->message)));
                }
            } catch (const std::exception& ex) {
                Q_EMIT operationFinished(false, QString("删除异常: %1").arg(ex.what()));
            }
        });
}

// ──────────────────────────────────────────────────────────────────────
// 修改属性
// ──────────────────────────────────────────────────────────────────────

void AreaTab::onEdit() {
    if (!update_client_) { status_label_->setText("节点未初始化"); return; }
    auto* sel = list_->currentItem();
    if (!sel) { status_label_->setText("请先选中一个区域"); return; }

    QVariantMap snap = sel->data(Qt::UserRole).toMap();
    const QString name = snap["name"].toString();
    if (name.isEmpty()) { status_label_->setText("选中项缺少原始数据，请先「刷新列表」"); return; }

    QDialog dlg(this);
    dlg.setWindowTitle(QString("修改区域属性 — %1").arg(name));
    dlg.setMinimumWidth(420);

    auto* name_label = new QLabel(name);
    name_label->setStyleSheet("font-weight: bold; color: #555;");

    auto* type_combo = new QComboBox();
    static const struct { uint32_t value; const char* label; } kTypes[] = {
        {1, "虚拟墙 (VIRTUAL_WALL)"},
        {2, "禁行区 (RESTRICTED_AREA)"},
        {3, "减速区 (SPEED_LIMIT_AREA)"},
        {4, "绕障区 (DETOUR_AREA)"},
        {5, "停障区 (STOP_OBSTACLE_AREA)"},
        {6, "停车区 (PARKING_AREA)"},
        {7, "充电区 (CHARGING_AREA)"},
        {8, "电梯区 (ELEVATOR_AREA)"},
        {9, "颠簸区 (BUMPY_AREA)"},
    };
    for (const auto& t : kTypes) {
        type_combo->addItem(t.label, t.value);
    }
    const uint32_t cur_type = snap["type"].toUInt();
    for (int i = 0; i < type_combo->count(); ++i) {
        if (type_combo->itemData(i).toUInt() == cur_type) {
            type_combo->setCurrentIndex(i); break;
        }
    }

    auto* max_speed_spin = new QDoubleSpinBox();
    max_speed_spin->setRange(0.0, 10.0);
    max_speed_spin->setSingleStep(0.1);
    max_speed_spin->setDecimals(2);
    max_speed_spin->setSuffix(" m/s");
    max_speed_spin->setSpecialValueText("默认");
    max_speed_spin->setValue(snap["max_speed"].toDouble());

    auto* stop_dist_spin = new QDoubleSpinBox();
    stop_dist_spin->setRange(0.0, 5.0);
    stop_dist_spin->setSingleStep(0.05);
    stop_dist_spin->setDecimals(2);
    stop_dist_spin->setSuffix(" m");
    stop_dist_spin->setSpecialValueText("默认");
    stop_dist_spin->setValue(snap["stop_distance"].toDouble());

    auto* depth_min_spin = new QDoubleSpinBox();
    depth_min_spin->setRange(0.0, 2.0);
    depth_min_spin->setSingleStep(0.01);
    depth_min_spin->setDecimals(3);
    depth_min_spin->setSuffix(" m");
    depth_min_spin->setSpecialValueText("默认");
    depth_min_spin->setValue(snap["depth_min_height"].toDouble());

    auto* charger_combo = new QComboBox();
    charger_combo->addItem("(空)", QString(""));
    charger_combo->addItem("WIRELESS（无线）", QString("WIRELESS"));
    charger_combo->addItem("CONTACT（接触式）", QString("CONTACT"));
    const QString cur_charger = snap["charger_type"].toString();
    for (int i = 0; i < charger_combo->count(); ++i) {
        if (charger_combo->itemData(i).toString() == cur_charger) {
            charger_combo->setCurrentIndex(i); break;
        }
    }

    auto* linked_edit = new QLineEdit();
    linked_edit->setPlaceholderText("关联 NavPoint 名称，多个用逗号或分号分隔（停车/充电区有效）");
    linked_edit->setText(snap["manual_linked"].toStringList().join(", "));

    auto* attribute_edit = new QLineEdit();
    attribute_edit->setPlaceholderText("JSON 扩展属性（可选）");
    attribute_edit->setText(snap["attribute"].toString());

    auto* form = new QFormLayout();
    form->addRow("名称 (只读):",       name_label);
    form->addRow("类型:",              type_combo);
    form->addRow("限速 max_speed:",    max_speed_spin);
    form->addRow("停障距离 stop_dist:", stop_dist_spin);
    form->addRow("深度最小高 depth_min:", depth_min_spin);
    form->addRow("充电方式:",          charger_combo);
    form->addRow("关联 NavPoint:",     linked_edit);
    form->addRow("属性 JSON:",         attribute_edit);

    auto* tip = new QLabel(
        "<i>名称作为主键不可修改；polygon 顶点保留原值（如需重绘请删除后用工具栏 (A) 重新添加）。"
        "字段仅在对应类型下生效（限速→减速区 / 停障距离→停障区 / 深度高→颠簸区 / 充电方式→充电区 / 关联点→停车+充电区）。</i>");
    tip->setStyleSheet("color: gray;");
    tip->setWordWrap(true);

    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    auto* root = new QVBoxLayout(&dlg);
    root->addLayout(form);
    root->addWidget(tip);
    root->addWidget(btns);

    if (dlg.exec() != QDialog::Accepted) return;

    // 组装 request：保留原 polygon + related_*（only-read），其它字段用 UI 值
    auto req = std::make_shared<nav_interfaces::srv::UpdateAreaInfo::Request>();
    req->area_info.name             = name.toStdString();
    req->area_info.type             = type_combo->currentData().toUInt();
    req->area_info.max_speed        = static_cast<float>(max_speed_spin->value());
    req->area_info.stop_distance    = static_cast<float>(stop_dist_spin->value());
    req->area_info.depth_min_height = static_cast<float>(depth_min_spin->value());
    req->area_info.charger_type     = charger_combo->currentData().toString().toStdString();
    req->area_info.attribute        = attribute_edit->text().trimmed().toStdString();

    // 关联 NavPoint：逗号 / 分号分隔
    QStringList linked_parts = linked_edit->text().split(QRegExp("[,;，；]"), Qt::SkipEmptyParts);
    for (const auto& s : linked_parts) {
        QString trimmed = s.trimmed();
        if (!trimmed.isEmpty()) req->area_info.manual_linked_point_names.push_back(trimmed.toStdString());
    }

    // 保留原 polygon
    QVariantList poly = snap["polygon"].toList();
    for (const auto& v : poly) {
        QVariantMap pm = v.toMap();
        geometry_msgs::msg::Pose2D pose;
        pose.x     = pm["x"].toDouble();
        pose.y     = pm["y"].toDouble();
        pose.theta = pm["theta"].toDouble();
        req->area_info.polygon.push_back(pose);
    }

    edit_btn_->setEnabled(false);
    status_label_->setText(QString("更新区域 %1 中...").arg(name));

    update_client_->async_send_request(
        req,
        [this, name](rclcpp::Client<nav_interfaces::srv::UpdateAreaInfo>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                if (resp->success) {
                    Q_EMIT operationFinished(true, QString("区域 %1 修改成功，刷新列表中...").arg(name));
                    QMetaObject::invokeMethod(this, &AreaTab::onRefresh, Qt::QueuedConnection);
                } else {
                    Q_EMIT operationFinished(false,
                        QString("修改失败: %1").arg(QString::fromStdString(resp->message)));
                }
            } catch (const std::exception& ex) {
                Q_EMIT operationFinished(false, QString("更新异常: %1").arg(ex.what()));
            }
        });
}

// ──────────────────────────────────────────────────────────────────────

void AreaTab::applyOperationResult(bool /*success*/, QString msg) {
    status_label_->setText(msg);
    refresh_btn_->setEnabled(true);
    const bool has_sel = (list_->currentItem() != nullptr);
    remove_btn_->setEnabled(has_sel);
    edit_btn_->setEnabled(has_sel);
}

}  // namespace map_asset_rviz_plugin
