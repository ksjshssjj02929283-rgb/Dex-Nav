// AddJunctionTool — 与 point_tool 交互逻辑一致
// 左键立即放置（无对话框，自动命名 jct_N），鼠标移动实时预览圆，右键退出

#include "map_asset_rviz_plugin/add_junction_tool.hpp"
#include "map_asset_rviz_plugin/route_graph_tools_common.hpp"

#include <OgreManualObject.h>
#include <OgreSceneManager.h>

#include <nav_interfaces/srv/set_assets.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/interaction/view_picker_iface.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/viewport_mouse_event.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace map_asset_rviz_plugin {

AddJunctionTool::AddJunctionTool() {
    shortcut_key_ = 'j';
}

AddJunctionTool::~AddJunctionTool() {
    if (context_) {
        auto* scene = context_->getSceneManager();
        if (preview_node_) { scene->destroySceneNode(preview_node_); preview_node_ = nullptr; }
        if (preview_obj_)  { scene->destroyManualObject(preview_obj_); preview_obj_ = nullptr; }
    }
}

void AddJunctionTool::onInitialize() {
    node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
    auto* scene = context_->getSceneManager();

    const std::string name =
        "map_asset_jct_preview_" + std::to_string(reinterpret_cast<uintptr_t>(this));
    preview_obj_ = scene->createManualObject(name);
    preview_obj_->setDynamic(true);
    preview_node_ = scene->getRootSceneNode()->createChildSceneNode();
    preview_node_->attachObject(preview_obj_);
    preview_node_->setVisible(false);
}

void AddJunctionTool::activate() {
    if (!s_route_graph_editing) {
        setStatus("请先点击「开始修改路网 [R]」再操作");
        return;
    }
    setStatus("左键放置 Junction（自动命名），右键退出");
}

void AddJunctionTool::deactivate() {
    clearPreview();
}

int AddJunctionTool::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
    if (!s_route_graph_editing) return Render;
    Ogre::Vector3 pos;

    // ----- 鼠标移动：绘制预览圆（同 point_tool 的 mouse_pos 预览）-----
    if (event.type == QEvent::MouseMove) {
        if (context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, pos)) {
            updatePreview(pos.x, pos.y);
        }
        return Render;
    }

    // ----- 左键：立即放置（同 point_tool 的 is_stop=true）-----
    if (event.leftDown()) {
        if (!context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, pos)) {
            setStatus("无法获取点击位置（请点击地图有效区域）");
            return Render;
        }

        const float px = pos.x;
        const float py = pos.y;
        const std::string auto_id = "jct_" + std::to_string(junction_counter_++);

        auto client = node_->create_client<nav_interfaces::srv::SetAssets>(
            service_prefix_ + "/set_assets");
        auto req = std::make_shared<nav_interfaces::srv::SetAssets::Request>(); req->junctions.resize(1);
        req->mode         = nav_interfaces::srv::SetAssets::Request::MODE_ADD_ONLY;
        req->junctions[0].id  = auto_id;
        req->junctions[0].x   = px;
        req->junctions[0].y   = py;
        req->junctions[0].yaw = 0.0f;

        const QString qid = QString::fromStdString(auto_id);
        client->async_send_request(req,
            [this, qid](rclcpp::Client<nav_interfaces::srv::SetAssets>::SharedFuture fut) {
                try {
                    auto resp = fut.get();
                    setStatus(resp->success
                        ? QString("Junction '%1' 已添加，继续点击放置").arg(qid)
                        : QString("添加失败: %1").arg(QString::fromStdString(resp->message)));
                } catch (const std::exception& ex) {
                    setStatus(QString("异常: %1").arg(ex.what()));
                }
            });

        setStatus(QString("放置 '%1' 中…").arg(qid));
        return Render;  // 保持工具激活，可继续放置
    }

    // ----- 右键：退出 -----
    if (event.rightDown()) {
        clearPreview();
        return Render | Finished;
    }

    return Render;
}

void AddJunctionTool::updatePreview(double x, double y) {
    if (!preview_obj_) return;
    preview_node_->setVisible(true);
    preview_obj_->clear();
    drawRing(preview_obj_,
             static_cast<float>(x), static_cast<float>(y), kPreviewZ,
             0.18f, 0.7f, 0.7f, 0.7f);
    drawRing(preview_obj_,
             static_cast<float>(x), static_cast<float>(y), kPreviewZ,
             0.06f, 0.5f, 0.5f, 0.5f);
}

void AddJunctionTool::clearPreview() {
    if (preview_obj_)  preview_obj_->clear();
    if (preview_node_) preview_node_->setVisible(false);
}

void AddJunctionTool::load(const rviz_common::Config& config) { Tool::load(config); }
void AddJunctionTool::save(rviz_common::Config config) const  { Tool::save(config); }

}  // namespace map_asset_rviz_plugin

PLUGINLIB_EXPORT_CLASS(map_asset_rviz_plugin::AddJunctionTool, rviz_common::Tool)
