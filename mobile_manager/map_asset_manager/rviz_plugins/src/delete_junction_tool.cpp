// DeleteJunctionTool — 在地图上点击删除路网 Junction 节点
// 悬停时红色高亮最近的 Junction；左键确认删除；右键退出。

#include "map_asset_rviz_plugin/delete_junction_tool.hpp"

#include <QMessageBox>

#include <OgreManualObject.h>
#include <OgreSceneManager.h>

#include <nav_interfaces/srv/set_assets.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/interaction/view_picker_iface.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/viewport_mouse_event.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace map_asset_rviz_plugin {

DeleteJunctionTool::DeleteJunctionTool() {
    shortcut_key_ = 'k';  // [K] 删除 Junction
}

DeleteJunctionTool::~DeleteJunctionTool() {
    if (context_) {
        auto* scene = context_->getSceneManager();
        if (preview_node_) { scene->destroySceneNode(preview_node_); preview_node_ = nullptr; }
        if (preview_obj_)  { scene->destroyManualObject(preview_obj_); preview_obj_ = nullptr; }
    }
}

void DeleteJunctionTool::onInitialize() {
    node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
    auto* scene = context_->getSceneManager();

    const std::string name =
        "map_asset_deljct_preview_" + std::to_string(reinterpret_cast<uintptr_t>(this));
    preview_obj_ = scene->createManualObject(name);
    preview_obj_->setDynamic(true);
    preview_node_ = scene->getRootSceneNode()->createChildSceneNode();
    preview_node_->attachObject(preview_obj_);
    preview_node_->setVisible(false);
}

void DeleteJunctionTool::activate() {
    reloadJunctions();
    hovered_ = nullptr;
    setStatus(junctions_.empty()
        ? "当前没有 Junction（请先添加）"
        : "悬停到 Junction 上，左键删除；右键退出");
}

void DeleteJunctionTool::deactivate() {
    clearPreview();
    hovered_ = nullptr;
}

int DeleteJunctionTool::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
    Ogre::Vector3 pos;

    // ----- 鼠标移动：高亮最近 Junction -----
    if (event.type == QEvent::MouseMove) {
        if (context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, pos)) {
            updateHighlight(pos.x, pos.y);
        }
        return Render;
    }

    // ----- 左键：删除 -----
    if (event.leftDown()) {
        if (!context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, pos)) {
            setStatus("无法获取点击位置");
            return Render;
        }
        updateHighlight(pos.x, pos.y);

        if (!hovered_) {
            setStatus("附近没有 Junction（需在 0.5m 范围内点击）");
            return Render;
        }

        // 二次确认
        const auto ans = QMessageBox::question(
            nullptr, "确认删除",
            QString("删除 Junction '%1' (%.3f, %.3f)？\n（关联的路网边也将一并删除）")
                .arg(QString::fromStdString(hovered_->id))
                .arg(hovered_->x)
                .arg(hovered_->y),
            QMessageBox::Yes | QMessageBox::No);
        if (ans != QMessageBox::Yes) {
            setStatus("取消删除");
            return Render;
        }

        doDelete(*hovered_);
        // 删除后刷新列表
        reloadJunctions();
        hovered_ = nullptr;
        clearPreview();
        setStatus("已删除，继续点击或右键退出");
        return Render;
    }

    // ----- 右键：退出 -----
    if (event.rightDown()) {
        clearPreview();
        return Render | Finished;
    }

    return Render;
}

// ---------------------------------------------------------------------------

void DeleteJunctionTool::reloadJunctions() {
    junctions_ = loadJunctions(node_, service_prefix_);
}

void DeleteJunctionTool::updateHighlight(double mx, double my) {
    hovered_ = findNearest(mx, my, junctions_, kSnapRadius);

    preview_obj_->clear();
    if (!hovered_) {
        preview_node_->setVisible(false);
        setStatus("悬停到 Junction 上，左键删除；右键退出");
        return;
    }

    preview_node_->setVisible(true);
    // 外圈（红色高亮）
    drawRing(preview_obj_, hovered_->x, hovered_->y, kPreviewZ, 0.22f,
             1.0f, 0.2f, 0.2f);
    // 内圈
    drawRing(preview_obj_, hovered_->x, hovered_->y, kPreviewZ, 0.08f,
             1.0f, 0.5f, 0.5f);

    setStatus(QString("找到 Junction: %1，左键删除")
                  .arg(QString::fromStdString(hovered_->id)));
}

void DeleteJunctionTool::clearPreview() {
    if (preview_obj_)  preview_obj_->clear();
    if (preview_node_) preview_node_->setVisible(false);
}

void DeleteJunctionTool::doDelete(const EndpointInfo& ep) {
    auto client = node_->create_client<nav_interfaces::srv::SetAssets>(
        service_prefix_ + "/set_assets");
    auto req = std::make_shared<nav_interfaces::srv::SetAssets::Request>(); req->junctions.resize(1);
    req->mode = nav_interfaces::srv::SetAssets::Request::MODE_REMOVE;
    req->junctions[0].id = ep.id;

    const QString display_id = QString::fromStdString(ep.id);
    client->async_send_request(req,
        [this, display_id](
            rclcpp::Client<nav_interfaces::srv::SetAssets>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                setStatus(resp->success
                    ? QString("Junction '%1' 已删除").arg(display_id)
                    : QString("删除失败: %1").arg(
                          QString::fromStdString(resp->message)));
            } catch (const std::exception& ex) {
                setStatus(QString("异常: %1").arg(ex.what()));
            }
        });
}

void DeleteJunctionTool::load(const rviz_common::Config& config) { Tool::load(config); }
void DeleteJunctionTool::save(rviz_common::Config config) const  { Tool::save(config); }

}  // namespace map_asset_rviz_plugin

PLUGINLIB_EXPORT_CLASS(map_asset_rviz_plugin::DeleteJunctionTool, rviz_common::Tool)
