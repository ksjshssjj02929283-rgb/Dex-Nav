#include "map_asset_rviz_plugin/room_tab.hpp"
#include "map_asset_rviz_plugin/asset_target.hpp"

#include <QCheckBox>
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
#include <QStringList>
#include <QVBoxLayout>

namespace map_asset_rviz_plugin {

// 类型标签（与后端 RoomType 命名空间保持一致）
QString RoomTab::typeLabel(uint32_t type) {
    switch (type) {
        case 1: return "会议室";
        case 2: return "茶水间";
        case 3: return "咖啡厅";
        case 4: return "走廊";
        case 5: return "办公室";
        case 6: return "大堂";
        case 7: return "卫生间";
        case 8: return "储藏室";
        default: return "未分类";
    }
}

// ──────────────────────────────────────────────────────────────────────

RoomTab::RoomTab(QWidget* parent) : QWidget(parent) {
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
        "提示：使用工具栏「添加房间」工具 (M) 在地图上绘制房间。\n"
        "左键添加顶点，右键完成绘制。DSG ROOMS 层节点（layer=3）。");
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

    connect(refresh_btn_, &QPushButton::clicked, this, &RoomTab::onRefresh);
    connect(edit_btn_,    &QPushButton::clicked, this, &RoomTab::onEdit);
    connect(remove_btn_,  &QPushButton::clicked, this, &RoomTab::onRemove);

    connect(list_, &QListWidget::currentItemChanged, this,
        [this](QListWidgetItem* current, QListWidgetItem*) {
            const bool has_sel = (current != nullptr);
            remove_btn_->setEnabled(has_sel);
            edit_btn_->setEnabled(has_sel);
            if (!current) { detail_label_->clear(); return; }
            detail_label_->setText(current->data(Qt::UserRole + 1).toString());
        });

    connect(this, &RoomTab::operationFinished, this, &RoomTab::applyOperationResult,
            Qt::QueuedConnection);
}

// ──────────────────────────────────────────────────────────────────────

void RoomTab::setNode(rclcpp::Node::SharedPtr node) {
    node_ = node;
    rebuildClients();
}

void RoomTab::setServicePrefix(const std::string& prefix) {
    prefix_ = prefix;
    if (node_) rebuildClients();
}

void RoomTab::rebuildClients() {
    get_client_    = node_->create_client<nav_interfaces::srv::GetAssets>(prefix_ + "/get_assets");
    remove_client_ = node_->create_client<nav_interfaces::srv::SetAssets>(prefix_ + "/set_assets");
    update_client_ = node_->create_client<nav_interfaces::srv::SetAssets>(prefix_ + "/set_assets");
}

// ──────────────────────────────────────────────────────────────────────
// 刷新
// ──────────────────────────────────────────────────────────────────────

void RoomTab::onRefresh() {
    if (!get_client_) { status_label_->setText("节点未初始化"); return; }
    refresh_btn_->setEnabled(false);
    remove_btn_->setEnabled(false);
    status_label_->setText("刷新中...");

    auto req = std::make_shared<nav_interfaces::srv::GetAssets::Request>(); applyTarget(req);
    req->kinds = nav_interfaces::srv::GetAssets::Request::KIND_ROOMS;
    get_client_->async_send_request(
        req,
        [this](rclcpp::Client<nav_interfaces::srv::GetAssets>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                QMetaObject::invokeMethod(this, [this, resp]() {
                    list_->clear();
                    detail_label_->clear();

                    for (const auto& room : resp->rooms) {
                        const QString label = typeLabel(room.type);

                        // 详情文本
                        QStringList details;
                        details << QString("类型: %1  顶点数: %2")
                                       .arg(label)
                                       .arg(static_cast<int>(room.polygon.size()));
                        if (!room.id.empty())
                            details << QString("ID: %1").arg(QString::fromStdString(room.id));
                        if (!room.source.empty())
                            details << QString("来源: %1").arg(QString::fromStdString(room.source));
                        if (!room.semantic_label.empty())
                            details << QString("开放词汇: %1")
                                        .arg(QString::fromStdString(room.semantic_label));

                        // 行为约束（方案 D：内嵌约束子集）
                        {
                            QStringList behavior_items;
                            if (room.behavior.max_speed_m_s > 0.0f)
                                behavior_items << QString("限速 %1 m/s").arg(room.behavior.max_speed_m_s);
                            if (room.behavior.restricted)
                                behavior_items << "禁入";
                            if (room.behavior.detour_enabled)
                                behavior_items << "允许绕障";
                            if (room.behavior.stop_distance_m > 0.0f)
                                behavior_items << QString("停障距离 %1 m").arg(room.behavior.stop_distance_m);
                            if (room.behavior.depth_min_height_m > 0.0f)
                                behavior_items << QString("深度最小高 %1 m").arg(room.behavior.depth_min_height_m);
                            if (!room.behavior.additional_area_names.empty()) {
                                QStringList extras;
                                for (const auto& n : room.behavior.additional_area_names)
                                    extras << QString::fromStdString(n);
                                behavior_items << QString("附加 Area: %1").arg(extras.join(", "));
                            }
                            if (!behavior_items.empty()) {
                                details << QString("行为约束: %1").arg(behavior_items.join(" | "));
                            }
                        }

                        // 手工指定的关联 NavPoint
                        if (!room.manual_anchor_point_names.empty()) {
                            QStringList anchors;
                            for (const auto& n : room.manual_anchor_point_names)
                                anchors << QString::fromStdString(n);
                            details << QString("手工关联 NavPoint: %1").arg(anchors.join(", "));
                        }

                        if (!room.attribute.empty())
                            details << QString("属性: %1")
                                        .arg(QString::fromStdString(room.attribute));
                        if (!room.external_dsg_node_id.empty())
                            details << QString("外部 DSG ID: %1")
                                        .arg(QString::fromStdString(room.external_dsg_node_id));

                        // 自动空间归属（SpatialIndexer 维护）
                        if (!room.related_nav_point_names.empty()) {
                            QStringList items;
                            for (const auto& n : room.related_nav_point_names)
                                items << QString::fromStdString(n);
                            details << QString("自动归属 NavPoint (%1): %2")
                                            .arg(items.size())
                                            .arg(items.join(", "));
                        }
                        if (!room.related_junction_ids.empty()) {
                            QStringList items;
                            for (const auto& n : room.related_junction_ids)
                                items << QString::fromStdString(n);
                            details << QString("自动归属 Junction (%1): %2")
                                            .arg(items.size())
                                            .arg(items.join(", "));
                        }

                        // 完整数据快照，供 onEdit 使用（UserRole）
                        QVariantMap snapshot;
                        snapshot["name"]           = QString::fromStdString(room.name);
                        snapshot["type"]           = room.type;
                        snapshot["semantic_label"] = QString::fromStdString(room.semantic_label);
                        snapshot["display_label"]  = QString::fromStdString(room.display_label);
                        snapshot["attribute"]      = QString::fromStdString(room.attribute);
                        snapshot["source"]         = QString::fromStdString(room.source);
                        QStringList anchors;
                        for (const auto& n : room.manual_anchor_point_names)
                            anchors << QString::fromStdString(n);
                        snapshot["manual_anchors"] = anchors;
                        // behavior
                        QVariantMap b;
                        b["max_speed_m_s"]      = static_cast<double>(room.behavior.max_speed_m_s);
                        b["restricted"]         = room.behavior.restricted;
                        b["detour_enabled"]     = room.behavior.detour_enabled;
                        b["stop_distance_m"]    = static_cast<double>(room.behavior.stop_distance_m);
                        b["depth_min_height_m"] = static_cast<double>(room.behavior.depth_min_height_m);
                        QStringList extras;
                        for (const auto& n : room.behavior.additional_area_names)
                            extras << QString::fromStdString(n);
                        b["additional_area_names"] = extras;
                        snapshot["behavior"]    = b;
                        // polygon
                        QVariantList poly;
                        for (const auto& pose : room.polygon) {
                            QVariantMap p;
                            p["x"]     = static_cast<double>(pose.x);
                            p["y"]     = static_cast<double>(pose.y);
                            p["theta"] = static_cast<double>(pose.theta);
                            poly.append(p);
                        }
                        snapshot["polygon"]     = poly;

                        auto* item = new QListWidgetItem(
                            QString("[%1] %2").arg(label).arg(
                                QString::fromStdString(room.name)));
                        item->setData(Qt::UserRole,     snapshot);             // 完整快照
                        item->setData(Qt::UserRole + 1, details.join("\n"));   // 详情字符串
                        list_->addItem(item);
                    }

                    refresh_btn_->setEnabled(true);
                    status_label_->setText(
                        QString("共 %1 个房间").arg(resp->rooms.size()));
                }, Qt::QueuedConnection);
            } catch (const std::exception& ex) {
                Q_EMIT operationFinished(false, QString("刷新失败: %1").arg(ex.what()));
            }
        });
}

// ──────────────────────────────────────────────────────────────────────
// 删除
// ──────────────────────────────────────────────────────────────────────

void RoomTab::onRemove() {
    auto* item = list_->currentItem();
    if (!item) { status_label_->setText("请先选中一个房间"); return; }

    // 从列表项文本中提取房间名称：格式为 "[类型] 名称"
    QString text = item->text();
    int bracket_end = text.indexOf("] ");
    QString room_name = (bracket_end >= 0) ? text.mid(bracket_end + 2) : text;

    if (QMessageBox::question(this, "确认删除",
                              QString("删除房间「%1」？").arg(room_name),
                              QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes) {
        return;
    }

    if (!remove_client_) { status_label_->setText("节点未初始化"); return; }
    remove_btn_->setEnabled(false);
    status_label_->setText("删除中...");

    auto req = std::make_shared<nav_interfaces::srv::SetAssets::Request>(); applyTarget(req); req->rooms.resize(1);
    req->mode = nav_interfaces::srv::SetAssets::Request::MODE_REMOVE;
    req->rooms[0].name = room_name.toStdString();

    remove_client_->async_send_request(
        req,
        [this](rclcpp::Client<nav_interfaces::srv::SetAssets>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                if (resp->success) {
                    Q_EMIT operationFinished(true, "删除成功，自动刷新列表...");
                    QMetaObject::invokeMethod(this, &RoomTab::onRefresh, Qt::QueuedConnection);
                } else {
                    Q_EMIT operationFinished(false,
                        QString("删除失败: %1").arg(assetMessage(*resp)));
                }
            } catch (const std::exception& ex) {
                Q_EMIT operationFinished(false, QString("删除异常: %1").arg(ex.what()));
            }
        });
}

// ──────────────────────────────────────────────────────────────────────
// 修改属性
// ──────────────────────────────────────────────────────────────────────

void RoomTab::onEdit() {
    if (!update_client_) { status_label_->setText("节点未初始化"); return; }
    auto* sel = list_->currentItem();
    if (!sel) { status_label_->setText("请先选中一个房间"); return; }

    QVariantMap snap = sel->data(Qt::UserRole).toMap();
    const QString name = snap["name"].toString();
    if (name.isEmpty()) { status_label_->setText("选中项缺少原始数据，请先「刷新列表」"); return; }

    QDialog dlg(this);
    dlg.setWindowTitle(QString("修改房间属性 — %1").arg(name));
    dlg.setMinimumWidth(440);

    auto* name_label = new QLabel(name);
    name_label->setStyleSheet("font-weight: bold; color: #555;");

    auto* type_combo = new QComboBox();
    static const struct { uint32_t value; const char* label; } kRoomTypes[] = {
        {1, "会议室 (MEETING_ROOM)"},
        {2, "茶水间 (PANTRY)"},
        {3, "咖啡厅 (CAFE)"},
        {4, "走廊 (CORRIDOR)"},
        {5, "办公室 (OFFICE)"},
        {6, "大堂 (LOBBY)"},
        {7, "卫生间 (RESTROOM)"},
        {8, "储藏室 (STORAGE)"},
        {0, "未分类 (UNKNOWN)"},
    };
    for (const auto& t : kRoomTypes) type_combo->addItem(t.label, t.value);
    const uint32_t cur_type = snap["type"].toUInt();
    for (int i = 0; i < type_combo->count(); ++i) {
        if (type_combo->itemData(i).toUInt() == cur_type) {
            type_combo->setCurrentIndex(i); break;
        }
    }

    auto* semantic_edit = new QLineEdit();
    semantic_edit->setPlaceholderText("VLN 开放词汇标签（可选）");
    semantic_edit->setText(snap["semantic_label"].toString());

    auto* display_edit = new QLineEdit();
    display_edit->setPlaceholderText("RViz 显示标签（留空则用 name）");
    display_edit->setText(snap["display_label"].toString());

    auto* anchor_edit = new QLineEdit();
    anchor_edit->setPlaceholderText("关联 NavPoint 名称，多个用逗号或分号分隔");
    anchor_edit->setText(snap["manual_anchors"].toStringList().join(", "));

    auto* attribute_edit = new QLineEdit();
    attribute_edit->setPlaceholderText("JSON 扩展属性（可选）");
    attribute_edit->setText(snap["attribute"].toString());

    // 区域属性（RoomBehavior，与 add_room_tool 对齐）
    QVariantMap b = snap["behavior"].toMap();

    auto* max_speed_spin = new QDoubleSpinBox();
    max_speed_spin->setRange(0.0, 10.0);
    max_speed_spin->setSingleStep(0.1);
    max_speed_spin->setDecimals(2);
    max_speed_spin->setSuffix(" m/s");
    max_speed_spin->setSpecialValueText("不限");
    max_speed_spin->setValue(b["max_speed_m_s"].toDouble());

    auto* restricted_check = new QCheckBox("禁入（机器人不进入此房间）");
    restricted_check->setChecked(b["restricted"].toBool());

    auto* detour_check = new QCheckBox("允许绕障（房间内允许绕过障碍）");
    detour_check->setChecked(b["detour_enabled"].toBool());

    auto* stop_dist_spin = new QDoubleSpinBox();
    stop_dist_spin->setRange(0.0, 5.0);
    stop_dist_spin->setSingleStep(0.05);
    stop_dist_spin->setDecimals(2);
    stop_dist_spin->setSuffix(" m");
    stop_dist_spin->setSpecialValueText("默认");
    stop_dist_spin->setValue(b["stop_distance_m"].toDouble());

    auto* depth_min_spin = new QDoubleSpinBox();
    depth_min_spin->setRange(0.0, 2.0);
    depth_min_spin->setSingleStep(0.01);
    depth_min_spin->setDecimals(3);
    depth_min_spin->setSuffix(" m");
    depth_min_spin->setSpecialValueText("默认");
    depth_min_spin->setValue(b["depth_min_height_m"].toDouble());

    auto* additional_areas_edit = new QLineEdit();
    additional_areas_edit->setPlaceholderText("额外关联 Area 名称，多个用逗号或分号分隔（可选）");
    additional_areas_edit->setText(b["additional_area_names"].toStringList().join(", "));

    auto* behavior_form = new QFormLayout();
    behavior_form->addRow("限速 max_speed:", max_speed_spin);
    behavior_form->addRow(restricted_check);
    behavior_form->addRow(detour_check);
    behavior_form->addRow("停障距离 stop_dist:", stop_dist_spin);
    behavior_form->addRow("深度最小高 depth_min:", depth_min_spin);
    behavior_form->addRow("额外关联 Area:", additional_areas_edit);

    auto* behavior_group = new QGroupBox("区域属性（RoomBehavior，方案 D）");
    behavior_group->setLayout(behavior_form);

    auto* form = new QFormLayout();
    form->addRow("名称 (只读):",  name_label);
    form->addRow("类型:",         type_combo);
    form->addRow("开放词汇:",     semantic_edit);
    form->addRow("显示标签:",     display_edit);
    form->addRow("关联 NavPoint:", anchor_edit);
    form->addRow("属性 JSON:",    attribute_edit);

    auto* tip = new QLabel(
        "<i>名称作为主键不可修改；polygon 顶点保留原值（如需重绘请删除后用工具栏 (M) 重新添加）。</i>");
    tip->setStyleSheet("color: gray;");
    tip->setWordWrap(true);

    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    auto* root = new QVBoxLayout(&dlg);
    root->addLayout(form);
    root->addWidget(behavior_group);
    root->addWidget(tip);
    root->addWidget(btns);

    if (dlg.exec() != QDialog::Accepted) return;

    auto split_names = [](const QString& text, std::vector<std::string>& out) {
        QStringList parts = text.split(QRegExp("[,;，；]"), Qt::SkipEmptyParts);
        for (const auto& s : parts) {
            QString trimmed = s.trimmed();
            if (!trimmed.isEmpty()) out.push_back(trimmed.toStdString());
        }
    };

    auto req = std::make_shared<nav_interfaces::srv::SetAssets::Request>(); applyTarget(req); req->rooms.resize(1);
    req->mode = nav_interfaces::srv::SetAssets::Request::MODE_UPDATE_ONLY;
    req->rooms[0].name           = name.toStdString();
    req->rooms[0].type           = type_combo->currentData().toUInt();
    req->rooms[0].semantic_label = semantic_edit->text().trimmed().toStdString();
    req->rooms[0].display_label  = display_edit->text().trimmed().toStdString();
    req->rooms[0].attribute      = attribute_edit->text().trimmed().toStdString();
    req->rooms[0].source         = snap["source"].toString().toStdString();  // 保留原 source
    split_names(anchor_edit->text(), req->rooms[0].manual_anchor_point_names);

    // 区域属性（RoomBehavior）
    req->rooms[0].behavior.max_speed_m_s      = static_cast<float>(max_speed_spin->value());
    req->rooms[0].behavior.restricted         = restricted_check->isChecked();
    req->rooms[0].behavior.detour_enabled     = detour_check->isChecked();
    req->rooms[0].behavior.stop_distance_m    = static_cast<float>(stop_dist_spin->value());
    req->rooms[0].behavior.depth_min_height_m = static_cast<float>(depth_min_spin->value());
    split_names(additional_areas_edit->text(), req->rooms[0].behavior.additional_area_names);

    // 保留原 polygon
    QVariantList poly = snap["polygon"].toList();
    for (const auto& v : poly) {
        QVariantMap pm = v.toMap();
        geometry_msgs::msg::Pose2D pose;
        pose.x     = pm["x"].toDouble();
        pose.y     = pm["y"].toDouble();
        pose.theta = pm["theta"].toDouble();
        req->rooms[0].polygon.push_back(pose);
    }

    edit_btn_->setEnabled(false);
    status_label_->setText(QString("更新房间 %1 中...").arg(name));

    update_client_->async_send_request(
        req,
        [this, name](rclcpp::Client<nav_interfaces::srv::SetAssets>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                if (resp->success) {
                    Q_EMIT operationFinished(true, QString("房间 %1 修改成功，刷新列表中...").arg(name));
                    QMetaObject::invokeMethod(this, &RoomTab::onRefresh, Qt::QueuedConnection);
                } else {
                    Q_EMIT operationFinished(false,
                        QString("修改失败: %1").arg(assetMessage(*resp)));
                }
            } catch (const std::exception& ex) {
                Q_EMIT operationFinished(false, QString("更新异常: %1").arg(ex.what()));
            }
        });
}

// ──────────────────────────────────────────────────────────────────────

void RoomTab::applyOperationResult(bool /*success*/, QString msg) {
    status_label_->setText(msg);
    refresh_btn_->setEnabled(true);
    const bool has_sel = (list_->currentItem() != nullptr);
    remove_btn_->setEnabled(has_sel);
    edit_btn_->setEnabled(has_sel);
}

}  // namespace map_asset_rviz_plugin
