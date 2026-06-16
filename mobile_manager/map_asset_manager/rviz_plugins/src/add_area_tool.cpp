#include "map_asset_rviz_plugin/add_area_tool.hpp"

#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreVector3.h>

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
#include <rviz_common/display_context.hpp>
#include <rviz_common/interaction/view_picker_iface.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/render_window.hpp>

namespace map_asset_rviz_plugin {

// ──────────────────────────────────────────────────────────────────────
// 区域类型定义
// ──────────────────────────────────────────────────────────────────────

struct AreaTypeDef {
    uint32_t value;
    const char* label;
};

static const AreaTypeDef kAreaTypes[] = {
    {1, "虚拟墙 (VIRTUAL_WALL)"},  {2, "禁行区 (RESTRICTED_AREA)"},    {3, "减速区 (SPEED_LIMIT_AREA)"},
    {4, "绕障区 (DETOUR_AREA)"},   {5, "停障区 (STOP_OBSTACLE_AREA)"}, {6, "停车区 (PARKING_AREA)"},
    {7, "充电区 (CHARGING_AREA)"}, {8, "电梯区 (ELEVATOR_AREA)"},      {9, "颠簸区 (BUMPY_AREA)"},
};
static constexpr int kAreaTypeCount = static_cast<int>(sizeof(kAreaTypes) / sizeof(kAreaTypes[0]));

// ──────────────────────────────────────────────────────────────────────

AddAreaTool::AddAreaTool() { shortcut_key_ = 'a'; }

AddAreaTool::~AddAreaTool() {
    if (preview_node_ && context_) {
        context_->getSceneManager()->destroyManualObject(preview_line_);
        context_->getSceneManager()->destroySceneNode(preview_node_);
    }
}

void AddAreaTool::onInitialize() {
    node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
    client_ = node_->create_client<nav_interfaces::srv::AddAreaInfo>(service_name_);

    auto* scene = context_->getSceneManager();
    preview_line_ = scene->createManualObject("map_asset_area_preview");
    preview_line_->setDynamic(true);
    preview_node_ = scene->getRootSceneNode()->createChildSceneNode();
    preview_node_->attachObject(preview_line_);
    preview_node_->setVisible(false);
}

// ──────────────────────────────────────────────────────────────────────
// 颜色映射（与后端 AreaManager 保持一致）
// ──────────────────────────────────────────────────────────────────────

AddAreaTool::Color AddAreaTool::typeColor(uint32_t type) {
    switch (type) {
        case 1:
            return {0.9f, 0.9f, 0.9f};  // 虚拟墙 - 白灰
        case 2:
            return {1.0f, 0.0f, 0.0f};  // 禁行 - 红
        case 3:
            return {1.0f, 0.8f, 0.0f};  // 减速 - 黄
        case 4:
            return {0.0f, 0.9f, 0.9f};  // 绕障 - 青
        case 5:
            return {1.0f, 0.5f, 0.0f};  // 停障 - 橙
        case 6:
            return {0.0f, 0.8f, 0.0f};  // 停车 - 绿
        case 7:
            return {0.2f, 0.5f, 1.0f};  // 充电 - 蓝
        case 8:
            return {0.7f, 0.3f, 1.0f};  // 电梯 - 紫
        default:
            return {0.6f, 0.6f, 0.6f};
    }
}

// ──────────────────────────────────────────────────────────────────────
// 预弹窗：在绘制前获取名称和类型
// ──────────────────────────────────────────────────────────────────────

bool AddAreaTool::showPreDialog() {
    QDialog dlg;
    dlg.setWindowTitle("创建区域 — 设置基本属性");
    dlg.setMinimumWidth(360);

    // ── 基础字段 ──────────────────────────────────────────────────
    auto* name_edit = new QLineEdit();
    auto* type_combo = new QComboBox();
    for (int i = 0; i < kAreaTypeCount; ++i) {
        type_combo->addItem(kAreaTypes[i].label, static_cast<unsigned int>(kAreaTypes[i].value));
    }

    // ── 类型专属字段 ──────────────────────────────────────────────

    // 减速区 (type=3)
    auto* speed_box = new QGroupBox("减速区属性");
    auto* speed_spin = new QDoubleSpinBox();
    speed_spin->setRange(0.0, 10.0);
    speed_spin->setValue(0.5);
    speed_spin->setSuffix(" m/s");
    {
        auto* fl = new QFormLayout(speed_box);
        fl->addRow("最大速度:", speed_spin);
    }

    // 停障区 (type=5)
    auto* stop_box = new QGroupBox("停障区属性");
    auto* stop_spin = new QDoubleSpinBox();
    stop_spin->setRange(0.0, 5.0);
    stop_spin->setValue(0.5);
    stop_spin->setSuffix(" m");
    {
        auto* fl = new QFormLayout(stop_box);
        fl->addRow("停止距离:", stop_spin);
    }

    // 停车区 (type=6) & 充电区 (type=7)
    auto* linked_box = new QGroupBox("停车 / 充电区属性");
    auto* linked_edit = new QLineEdit();
    linked_edit->setPlaceholderText("关联导航点名称");
    auto* charger_type_combo = new QComboBox();
    charger_type_combo->addItem("无线 (WIRELESS)");
    charger_type_combo->addItem("接触式 (CONTACT)");
    auto* charger_type_label = new QLabel("充电方式:");
    {
        auto* fl = new QFormLayout(linked_box);
        fl->addRow("关联导航点:", linked_edit);
        fl->addRow(charger_type_label, charger_type_combo);
    }

    // 颠簸区 (type=9)
    auto* bumpy_box = new QGroupBox("颠簸区属性");
    auto* bumpy_speed_spin = new QDoubleSpinBox();
    bumpy_speed_spin->setRange(0.0, 5.0);
    bumpy_speed_spin->setValue(0.3);
    bumpy_speed_spin->setSuffix(" m/s");
    auto* depth_min_height_spin = new QDoubleSpinBox();
    depth_min_height_spin->setRange(0.0, 1.0);
    depth_min_height_spin->setValue(0.1);
    depth_min_height_spin->setSingleStep(0.05);
    depth_min_height_spin->setSuffix(" m");
    depth_min_height_spin->setToolTip(
        "深度相机最小有效检测高度，高于此值的物体才触发避障，\n用于忽略地面颠簸产生的噪点");
    {
        auto* fl = new QFormLayout(bumpy_box);
        fl->addRow("限速:", bumpy_speed_spin);
        fl->addRow("避障最小高度:", depth_min_height_spin);
    }

    // ── 布局 ──────────────────────────────────────────────────────
    auto* form = new QFormLayout();
    form->addRow("区域名称 *:", name_edit);
    form->addRow("区域类型 *:", type_combo);

    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    QObject::connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    QObject::connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    auto* root = new QVBoxLayout(&dlg);
    root->addLayout(form);
    root->addWidget(speed_box);
    root->addWidget(stop_box);
    root->addWidget(linked_box);
    root->addWidget(bumpy_box);
    root->addWidget(btns);

    // 初始化可见性
    auto updateVisibility = [&](int idx) {
        uint32_t t = type_combo->itemData(idx).toUInt();
        speed_box->setVisible(t == 3);
        stop_box->setVisible(t == 5);
        linked_box->setVisible(t == 6 || t == 7);
        charger_type_label->setVisible(t == 7);
        charger_type_combo->setVisible(t == 7);
        bumpy_box->setVisible(t == 9);
        dlg.adjustSize();
    };
    updateVisibility(type_combo->currentIndex());
    QObject::connect(type_combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
                     [&](int idx) { updateVisibility(idx); });

    if (dlg.exec() != QDialog::Accepted) return false;

    area_name_ = name_edit->text().trimmed().toStdString();
    if (area_name_.empty()) return false;

    area_type_ = type_combo->currentData().toUInt();
    area_max_speed_ = static_cast<float>(speed_spin->value());
    area_stop_distance_ = static_cast<float>(stop_spin->value());
    area_linked_point_ = linked_edit->text().toStdString();
    area_charger_type_ = (charger_type_combo->currentIndex() == 0) ? "WIRELESS" : "CONTACT";
    area_max_speed_ = static_cast<float>(bumpy_speed_spin->value());
    area_depth_min_height_ = static_cast<float>(depth_min_height_spin->value());

    preview_color_ = typeColor(area_type_);
    return true;
}

// ──────────────────────────────────────────────────────────────────────
// 工具生命周期
// ──────────────────────────────────────────────────────────────────────

void AddAreaTool::activate() {
    cancelled_ = false;
    vertices_.clear();

    if (!showPreDialog()) {
        cancelled_ = true;
        setStatus("已取消");
        // 延迟切回默认工具，避免在 activate() 执行中途切换
        QTimer::singleShot(0, [this]() {
            if (context_ && context_->getToolManager()) {
                context_->getToolManager()->setCurrentTool(context_->getToolManager()->getDefaultTool());
            }
        });
        return;
    }

    if (preview_node_) preview_node_->setVisible(true);
    if (area_type_ == 1) {
        setStatus(
            QString("【虚拟墙 %1】左键添加顶点，右键完成（需 ≥2 个顶点）").arg(QString::fromStdString(area_name_)));
    } else {
        setStatus(QString("【%1】左键添加顶点，右键完成（需 ≥3 个顶点）").arg(QString::fromStdString(area_name_)));
    }
}

void AddAreaTool::deactivate() {
    resetPolygon();
    if (preview_node_) preview_node_->setVisible(false);
}

// ──────────────────────────────────────────────────────────────────────
// 鼠标事件
// ──────────────────────────────────────────────────────────────────────

int AddAreaTool::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
    if (cancelled_) return Render | Finished;

    if (event.rightDown()) {
        // 右键：完成绘制
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
        v.x = static_cast<double>(pos.x);
        v.y = static_cast<double>(pos.y);
        v.theta = 0.0;
        vertices_.push_back(v);
        updatePreviewLine();

        const int min_verts = (area_type_ == 1) ? 2 : 3;
        setStatus(QString("【%1】已添加 %2 个顶点，右键完成（需 ≥%3）")
                      .arg(QString::fromStdString(area_name_))
                      .arg(vertices_.size())
                      .arg(min_verts));
    }

    return Render;
}

// ──────────────────────────────────────────────────────────────────────
// 辅助方法
// ──────────────────────────────────────────────────────────────────────

void AddAreaTool::finishPolygon() {
    const size_t min_verts = (area_type_ == 1) ? 2u : 3u;
    if (vertices_.size() < min_verts) {
        setStatus(QString("顶点不足（当前 %1 个，需 ≥%2）").arg(vertices_.size()).arg(min_verts));
        resetPolygon();
        return;
    }

    auto verts = vertices_;
    resetPolygon();

    if (!client_->service_is_ready()) {
        setStatus(QString("服务不可用: %1").arg(QString::fromStdString(service_name_)));
        return;
    }

    auto req = std::make_shared<nav_interfaces::srv::AddAreaInfo::Request>();
    req->area_info.name = area_name_;
    req->area_info.type = area_type_;
    req->area_info.max_speed = area_max_speed_;
    req->area_info.stop_distance = area_stop_distance_;
    req->area_info.linked_point_name = area_linked_point_;
    req->area_info.charger_type = area_charger_type_;
    req->area_info.depth_min_height = area_depth_min_height_;
    req->area_info.polygon = verts;

    setStatus(QString("正在添加区域 【%1】...").arg(QString::fromStdString(area_name_)));

    client_->async_send_request(req, [this](rclcpp::Client<nav_interfaces::srv::AddAreaInfo>::SharedFuture fut) {
        try {
            auto resp = fut.get();
            setStatus(resp->success ? QString("区域 【%1】 添加成功").arg(QString::fromStdString(area_name_))
                                    : QString("区域 【%1】 添加失败").arg(QString::fromStdString(area_name_)));
        } catch (const std::exception& ex) {
            setStatus(QString("添加异常: %1").arg(ex.what()));
        }
    });
}

void AddAreaTool::resetPolygon() {
    vertices_.clear();
    if (preview_line_) preview_line_->clear();
}

void AddAreaTool::updatePreviewLine() {
    if (!preview_line_ || vertices_.empty()) return;

    preview_line_->clear();
    preview_line_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

    const auto& col = preview_color_;
    for (const auto& v : vertices_) {
        preview_line_->colour(col.r, col.g, col.b, 1.0f);
        preview_line_->position(static_cast<float>(v.x), static_cast<float>(v.y), 0.05f);
    }
    // 多边形闭合回第一个顶点（虚拟墙 type=1 不闭合）
    if (area_type_ != 1 && vertices_.size() >= 2) {
        preview_line_->colour(col.r, col.g, col.b, 0.5f);
        preview_line_->position(static_cast<float>(vertices_.front().x), static_cast<float>(vertices_.front().y),
                                0.05f);
    }

    preview_line_->end();
}

// ──────────────────────────────────────────────────────────────────────
// 配置持久化
// ──────────────────────────────────────────────────────────────────────

void AddAreaTool::load(const rviz_common::Config& config) { Tool::load(config); }

void AddAreaTool::save(rviz_common::Config config) const { Tool::save(config); }

}  // namespace map_asset_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(map_asset_rviz_plugin::AddAreaTool, rviz_common::Tool)
