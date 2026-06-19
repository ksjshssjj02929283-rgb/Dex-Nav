#include "map_asset_rviz_plugin/add_room_tool.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QTimer>
#include <QVBoxLayout>

#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreVector3.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/interaction/view_picker_iface.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/render_window.hpp>

namespace map_asset_rviz_plugin {

// ──────────────────────────────────────────────────────────────────────
// 房间类型定义（与后端 RoomType / RoomInfo.msg 一致）
// ──────────────────────────────────────────────────────────────────────

struct RoomTypeDef {
    uint32_t    value;
    const char* label;
};

static const RoomTypeDef kRoomTypes[] = {
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
static constexpr int kRoomTypeCount =
    static_cast<int>(sizeof(kRoomTypes) / sizeof(kRoomTypes[0]));

// ──────────────────────────────────────────────────────────────────────

AddRoomTool::AddRoomTool() {
    shortcut_key_ = 'm';  // M for "rooM"（与 A 区分开）
}

AddRoomTool::~AddRoomTool() {
    if (preview_node_ && context_) {
        context_->getSceneManager()->destroyManualObject(preview_line_);
        context_->getSceneManager()->destroySceneNode(preview_node_);
    }
}

void AddRoomTool::onInitialize() {
    node_   = context_->getRosNodeAbstraction().lock()->get_raw_node();
    client_ = node_->create_client<nav_interfaces::srv::AddRoom>(service_name_);

    auto* scene = context_->getSceneManager();
    preview_line_ = scene->createManualObject("map_asset_room_preview");
    preview_line_->setDynamic(true);
    preview_node_ = scene->getRootSceneNode()->createChildSceneNode();
    preview_node_->attachObject(preview_line_);
    preview_node_->setVisible(false);
}

// ──────────────────────────────────────────────────────────────────────
// 颜色映射（与后端 RoomManager::typeColor 一致）
// ──────────────────────────────────────────────────────────────────────

AddRoomTool::Color AddRoomTool::typeColor(uint32_t type) {
    switch (type) {
        case 1: return {0.20f, 0.55f, 0.95f};  // 会议室 - 蓝
        case 2: return {0.95f, 0.75f, 0.25f};  // 茶水间 - 黄
        case 3: return {0.65f, 0.45f, 0.25f};  // 咖啡厅 - 棕
        case 4: return {0.70f, 0.70f, 0.70f};  // 走廊 - 灰
        case 5: return {0.30f, 0.80f, 0.40f};  // 办公室 - 绿
        case 6: return {0.95f, 0.55f, 0.30f};  // 大堂 - 橙
        case 7: return {0.80f, 0.30f, 0.60f};  // 卫生间 - 粉紫
        case 8: return {0.50f, 0.50f, 0.30f};  // 储藏室 - 橄榄
        default: return {0.60f, 0.60f, 0.60f};
    }
}

// ──────────────────────────────────────────────────────────────────────
// 预弹窗：在绘制前获取名称、类型、可选关联点
// ──────────────────────────────────────────────────────────────────────

bool AddRoomTool::showPreDialog() {
    QDialog dlg;
    dlg.setWindowTitle("创建房间 — 设置基本属性");
    dlg.setMinimumWidth(360);

    auto* name_edit          = new QLineEdit();
    name_edit->setPlaceholderText("如 会议室3");

    auto* type_combo         = new QComboBox();
    for (int i = 0; i < kRoomTypeCount; ++i) {
        type_combo->addItem(kRoomTypes[i].label,
                            static_cast<unsigned int>(kRoomTypes[i].value));
    }

    auto* semantic_edit      = new QLineEdit();
    semantic_edit->setPlaceholderText("如 meeting room（VLN 开放词汇标签，可选）");

    auto* display_edit       = new QLineEdit();
    display_edit->setPlaceholderText("rviz 标签（留空则使用 name）");

    auto* anchor_edit        = new QLineEdit();
    anchor_edit->setPlaceholderText("关联 NavPoint 名称，多个用逗号或分号分隔（可选）");

    // ── 区域属性（RoomBehavior，方案 D）──────────────────────────
    auto* max_speed_spin = new QDoubleSpinBox();
    max_speed_spin->setRange(0.0, 10.0);
    max_speed_spin->setSingleStep(0.1);
    max_speed_spin->setDecimals(2);
    max_speed_spin->setSuffix(" m/s");
    max_speed_spin->setSpecialValueText("不限");
    max_speed_spin->setValue(0.0);

    auto* restricted_check    = new QCheckBox("禁入（机器人不进入此房间）");
    auto* detour_check        = new QCheckBox("允许绕障（房间内允许绕过障碍）");

    auto* stop_dist_spin = new QDoubleSpinBox();
    stop_dist_spin->setRange(0.0, 5.0);
    stop_dist_spin->setSingleStep(0.05);
    stop_dist_spin->setDecimals(2);
    stop_dist_spin->setSuffix(" m");
    stop_dist_spin->setSpecialValueText("默认");
    stop_dist_spin->setValue(0.0);

    auto* depth_min_spin = new QDoubleSpinBox();
    depth_min_spin->setRange(0.0, 2.0);
    depth_min_spin->setSingleStep(0.01);
    depth_min_spin->setDecimals(3);
    depth_min_spin->setSuffix(" m");
    depth_min_spin->setSpecialValueText("默认");
    depth_min_spin->setValue(0.0);

    auto* additional_areas_edit = new QLineEdit();
    additional_areas_edit->setPlaceholderText("额外关联 Area 名称，多个用逗号或分号分隔（可选）");

    auto* behavior_form = new QFormLayout();
    behavior_form->addRow("限速 max_speed:", max_speed_spin);
    behavior_form->addRow(restricted_check);
    behavior_form->addRow(detour_check);
    behavior_form->addRow("停障距离 stop_dist:", stop_dist_spin);
    behavior_form->addRow("深度最小高 depth_min:", depth_min_spin);
    behavior_form->addRow("额外关联 Area:", additional_areas_edit);

    auto* behavior_group = new QGroupBox("区域属性（RoomBehavior，可选）");
    behavior_group->setLayout(behavior_form);

    auto* tip_label = new QLabel(
        "<i>提示：DSG ROOMS 层节点。默认多边形类型，激活后左键添加顶点，右键完成。"
        "区域属性内嵌 Area 约束子集（方案 D），0 / 未勾选 = 不启用；与独立 Area 取最严格值。</i>");
    tip_label->setWordWrap(true);
    tip_label->setStyleSheet("color: gray;");

    auto* form = new QFormLayout();
    form->addRow("房间名称 *:", name_edit);
    form->addRow("房间类型 *:", type_combo);
    form->addRow("开放词汇:", semantic_edit);
    form->addRow("显示标签:", display_edit);
    form->addRow("关联 NavPoint:", anchor_edit);

    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    QObject::connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    QObject::connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    auto* root = new QVBoxLayout(&dlg);
    root->addLayout(form);
    root->addWidget(behavior_group);
    root->addWidget(tip_label);
    root->addWidget(btns);

    if (dlg.exec() != QDialog::Accepted) return false;

    room_name_ = name_edit->text().trimmed().toStdString();
    if (room_name_.empty()) return false;

    room_type_           = type_combo->currentData().toUInt();
    room_semantic_label_ = semantic_edit->text().trimmed().toStdString();
    room_display_label_  = display_edit->text().trimmed().toStdString();

    // 关联点：逗号 / 分号 分隔
    auto split_names = [](const QString& text, std::vector<std::string>& out) {
        out.clear();
        QStringList parts = text.split(QRegExp("[,;，；]"), Qt::SkipEmptyParts);
        for (const auto& s : parts) {
            QString trimmed = s.trimmed();
            if (!trimmed.isEmpty()) {
                out.push_back(trimmed.toStdString());
            }
        }
    };
    split_names(anchor_edit->text(), room_anchor_point_names_);

    // 区域属性（RoomBehavior）—— 0 / false 表示不启用
    behavior_max_speed_m_s_       = static_cast<float>(max_speed_spin->value());
    behavior_restricted_          = restricted_check->isChecked();
    behavior_detour_enabled_      = detour_check->isChecked();
    behavior_stop_distance_m_     = static_cast<float>(stop_dist_spin->value());
    behavior_depth_min_height_m_  = static_cast<float>(depth_min_spin->value());
    split_names(additional_areas_edit->text(), behavior_additional_area_names_);

    preview_color_ = typeColor(room_type_);
    return true;
}

// ──────────────────────────────────────────────────────────────────────
// 工具生命周期
// ──────────────────────────────────────────────────────────────────────

void AddRoomTool::activate() {
    cancelled_ = false;
    vertices_.clear();

    if (!showPreDialog()) {
        cancelled_ = true;
        setStatus("已取消");
        QTimer::singleShot(0, [this]() {
            if (context_ && context_->getToolManager()) {
                context_->getToolManager()->setCurrentTool(
                    context_->getToolManager()->getDefaultTool());
            }
        });
        return;
    }

    if (preview_node_) preview_node_->setVisible(true);
    setStatus(QString("【%1】左键添加顶点，右键完成（需 ≥3 个顶点）")
                  .arg(QString::fromStdString(room_name_)));
}

void AddRoomTool::deactivate() {
    resetPolygon();
    if (preview_node_) preview_node_->setVisible(false);
}

// ──────────────────────────────────────────────────────────────────────
// 鼠标事件
// ──────────────────────────────────────────────────────────────────────

int AddRoomTool::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
    if (cancelled_) return Render | Finished;

    if (event.rightDown()) {
        finishPolygon();
        return Render | Finished;
    }

    if (event.leftDown()) {
        Ogre::Vector3 pos;
        if (!context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, pos)) {
            setStatus("无法获取点击位置（请点击有效地图区域）");
            return Render;
        }

        geometry_msgs::msg::Pose2D v;
        v.x     = static_cast<double>(pos.x);
        v.y     = static_cast<double>(pos.y);
        v.theta = 0.0;
        vertices_.push_back(v);
        updatePreviewLine();

        setStatus(QString("【%1】已添加 %2 个顶点，右键完成（需 ≥3）")
                      .arg(QString::fromStdString(room_name_))
                      .arg(vertices_.size()));
    }

    return Render;
}

// ──────────────────────────────────────────────────────────────────────
// 辅助方法
// ──────────────────────────────────────────────────────────────────────

void AddRoomTool::finishPolygon() {
    if (vertices_.size() < 3) {
        setStatus(QString("顶点不足（当前 %1 个，需 ≥3）").arg(vertices_.size()));
        resetPolygon();
        return;
    }

    auto verts = vertices_;
    resetPolygon();

    if (!client_->service_is_ready()) {
        setStatus(QString("服务不可用: %1").arg(QString::fromStdString(service_name_)));
        return;
    }

    auto req = std::make_shared<nav_interfaces::srv::AddRoom::Request>();
    req->room.name           = room_name_;
    req->room.type           = room_type_;
    req->room.semantic_label = room_semantic_label_;
    req->room.display_label  = room_display_label_;
    req->room.polygon        = verts;
    // source 留空 → 后端默认 manual
    req->room.manual_anchor_point_names.assign(room_anchor_point_names_.begin(),
                                        room_anchor_point_names_.end());

    // 区域属性（RoomBehavior，方案 D 内嵌约束子集）
    req->room.behavior.max_speed_m_s      = behavior_max_speed_m_s_;
    req->room.behavior.restricted         = behavior_restricted_;
    req->room.behavior.detour_enabled     = behavior_detour_enabled_;
    req->room.behavior.stop_distance_m    = behavior_stop_distance_m_;
    req->room.behavior.depth_min_height_m = behavior_depth_min_height_m_;
    req->room.behavior.additional_area_names.assign(
        behavior_additional_area_names_.begin(),
        behavior_additional_area_names_.end());

    setStatus(QString("正在添加房间 【%1】...").arg(QString::fromStdString(room_name_)));

    client_->async_send_request(
        req,
        [this](rclcpp::Client<nav_interfaces::srv::AddRoom>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                setStatus(resp->success
                              ? QString("房间 【%1】 添加成功")
                                    .arg(QString::fromStdString(room_name_))
                              : QString("房间 【%1】 添加失败: %2")
                                    .arg(QString::fromStdString(room_name_))
                                    .arg(QString::fromStdString(resp->message)));
            } catch (const std::exception& ex) {
                setStatus(QString("添加异常: %1").arg(ex.what()));
            }
        });
}

void AddRoomTool::resetPolygon() {
    vertices_.clear();
    if (preview_line_) preview_line_->clear();
}

void AddRoomTool::updatePreviewLine() {
    if (!preview_line_ || vertices_.empty()) return;

    preview_line_->clear();
    preview_line_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

    const auto& col = preview_color_;
    for (const auto& v : vertices_) {
        preview_line_->colour(col.r, col.g, col.b, 1.0f);
        preview_line_->position(
            static_cast<float>(v.x),
            static_cast<float>(v.y),
            0.05f);
    }
    // 多边形闭合预览
    if (vertices_.size() >= 2) {
        preview_line_->colour(col.r, col.g, col.b, 0.5f);
        preview_line_->position(
            static_cast<float>(vertices_.front().x),
            static_cast<float>(vertices_.front().y),
            0.05f);
    }

    preview_line_->end();
}

// ──────────────────────────────────────────────────────────────────────
// 配置持久化
// ──────────────────────────────────────────────────────────────────────

void AddRoomTool::load(const rviz_common::Config& config) {
    Tool::load(config);
}

void AddRoomTool::save(rviz_common::Config config) const {
    Tool::save(config);
}

}  // namespace map_asset_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(map_asset_rviz_plugin::AddRoomTool, rviz_common::Tool)
