#include "map_asset_rviz_plugin/add_nav_point_tool.hpp"
#include "map_asset_rviz_plugin/nav_point_draft_bridge.hpp"

#include <cmath>

#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreVector3.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/interaction/view_picker_iface.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/viewport_mouse_event.hpp>

namespace map_asset_rviz_plugin {

AddNavPointTool::AddNavPointTool() {
    shortcut_key_ = 'n';
}

AddNavPointTool::~AddNavPointTool() {
    if (context_) {
        auto* scene = context_->getSceneManager();
        if (arrow_node_) {
            scene->destroySceneNode(arrow_node_);
            arrow_node_ = nullptr;
        }
        if (arrow_obj_) {
            scene->destroyManualObject(arrow_obj_);
            arrow_obj_ = nullptr;
        }
    }
}

void AddNavPointTool::onInitialize() {
    node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();

    auto* scene = context_->getSceneManager();
    arrow_obj_  = scene->createManualObject("map_asset_nav_point_arrow");
    arrow_obj_->setDynamic(true);
    arrow_node_ = scene->getRootSceneNode()->createChildSceneNode();
    arrow_node_->attachObject(arrow_obj_);
    arrow_node_->setVisible(false);
}

void AddNavPointTool::activate() {
    dragging_ = false;
    setStatus("按住左键拖拽确定位置和朝向，松开后在面板中编辑点位属性");
}

void AddNavPointTool::deactivate() {
    dragging_ = false;
    clearArrowPreview();
}

// ──────────────────────────────────────────────────────────────────────
// 鼠标事件
// ──────────────────────────────────────────────────────────────────────

int AddNavPointTool::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
    if (event.leftDown()) {
        Ogre::Vector3 pos;
        if (!context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, pos)) {
            setStatus("无法获取点击位置（请点击有效地图区域）");
            return Render;
        }
        start_x_  = static_cast<double>(pos.x);
        start_y_  = static_cast<double>(pos.y);
        dragging_ = true;
        if (arrow_node_) arrow_node_->setVisible(true);
        updateArrowPreview(start_x_, start_y_, start_x_, start_y_);
        setStatus("拖拽确定朝向，松开完成选点");
        return Render;
    }

    if (event.type == QEvent::MouseMove && dragging_) {
        Ogre::Vector3 pos;
        if (context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, pos)) {
            updateArrowPreview(start_x_, start_y_,
                               static_cast<double>(pos.x),
                               static_cast<double>(pos.y));
        }
        return Render;
    }

    if (event.leftUp() && dragging_) {
        dragging_ = false;
        clearArrowPreview();

        Ogre::Vector3 pos;
        double end_x = start_x_, end_y = start_y_;
        if (context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, pos)) {
            end_x = static_cast<double>(pos.x);
            end_y = static_cast<double>(pos.y);
        }

        const double dx  = end_x - start_x_;
        const double dy  = end_y - start_y_;
        const double yaw = std::atan2(dy, dx);

        NavPointDraftBridge::instance()->notifyDraft(start_x_, start_y_, yaw);
        setStatus(QString("位置 (%.3f, %.3f) 朝向 %.3f rad — 在面板中填写名称后确认")
                      .arg(start_x_).arg(start_y_).arg(yaw));
        return Render | Finished;
    }

    return Render;
}

// ──────────────────────────────────────────────────────────────────────
// 箭头预览
// ──────────────────────────────────────────────────────────────────────

void AddNavPointTool::updateArrowPreview(double x0, double y0, double x1, double y1) {
    if (!arrow_obj_) return;

    const double dx  = x1 - x0;
    const double dy  = y1 - y0;
    const double len = std::sqrt(dx * dx + dy * dy);

    // 箭头主轴长度至少 0.3 m，最多 2.0 m
    const double shaft = std::max(0.3, std::min(len, 2.0));
    const double head  = shaft * 0.3;
    const double yaw   = std::atan2(dy, dx);
    const double cx    = std::cos(yaw);
    const double cy    = std::sin(yaw);
    const double px    = -cy;  // 垂直方向
    const double py    =  cx;

    const float z = 0.05f;
    const float tip_x = static_cast<float>(x0 + cx * shaft);
    const float tip_y = static_cast<float>(y0 + cy * shaft);

    arrow_obj_->clear();
    arrow_obj_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);

    // 轴
    arrow_obj_->colour(0.0f, 1.0f, 0.0f, 1.0f);
    arrow_obj_->position(static_cast<float>(x0), static_cast<float>(y0), z);
    arrow_obj_->colour(0.0f, 1.0f, 0.0f, 1.0f);
    arrow_obj_->position(tip_x, tip_y, z);

    // 左翼
    arrow_obj_->colour(0.0f, 1.0f, 0.0f, 1.0f);
    arrow_obj_->position(tip_x, tip_y, z);
    arrow_obj_->colour(0.0f, 1.0f, 0.0f, 1.0f);
    arrow_obj_->position(
        static_cast<float>(x0 + cx * (shaft - head) + px * head * 0.5),
        static_cast<float>(y0 + cy * (shaft - head) + py * head * 0.5), z);

    // 右翼
    arrow_obj_->colour(0.0f, 1.0f, 0.0f, 1.0f);
    arrow_obj_->position(tip_x, tip_y, z);
    arrow_obj_->colour(0.0f, 1.0f, 0.0f, 1.0f);
    arrow_obj_->position(
        static_cast<float>(x0 + cx * (shaft - head) - px * head * 0.5),
        static_cast<float>(y0 + cy * (shaft - head) - py * head * 0.5), z);

    arrow_obj_->end();
}

void AddNavPointTool::clearArrowPreview() {
    if (arrow_obj_) arrow_obj_->clear();
    if (arrow_node_) arrow_node_->setVisible(false);
}

// ──────────────────────────────────────────────────────────────────────
// 配置持久化
// ──────────────────────────────────────────────────────────────────────

void AddNavPointTool::load(const rviz_common::Config& config) {
    Tool::load(config);
}

void AddNavPointTool::save(rviz_common::Config config) const {
    Tool::save(config);
}

}  // namespace map_asset_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(map_asset_rviz_plugin::AddNavPointTool, rviz_common::Tool)
