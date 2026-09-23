#include "map_asset_rviz_plugin/delete_edge_tool.hpp"

#include <QInputDialog>
#include <QMessageBox>

#include <rviz_common/display_context.hpp>
#include <rviz_common/interaction/view_picker_iface.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/viewport_mouse_event.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace map_asset_rviz_plugin {

DeleteEdgeTool::DeleteEdgeTool() {
    shortcut_key_ = 'x';
}

void DeleteEdgeTool::onInitialize() {
    node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
    get_graph_client_   = node_->create_client<nav_interfaces::srv::GetRouteGraph>(
        service_prefix_ + "/get_route_graph");
    remove_edge_client_ = node_->create_client<nav_interfaces::srv::SetAssets>(
        service_prefix_ + "/set_assets");
}

void DeleteEdgeTool::activate() {
    setStatus("左键点击弹出边列表选择删除，右键退出");
}

void DeleteEdgeTool::deactivate() {}

int DeleteEdgeTool::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
    if (event.leftDown()) {
        // 获取当前所有边
        if (!get_graph_client_->wait_for_service(std::chrono::milliseconds(500))) {
            setStatus("路网服务不可用");
            return 0;
        }
        auto req = std::make_shared<nav_interfaces::srv::GetRouteGraph::Request>();
        auto fut = get_graph_client_->async_send_request(req);
        if (fut.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
            setStatus("获取路网超时");
            return 0;
        }
        auto resp = fut.get();
        if (resp->edges.empty()) {
            setStatus("当前路网无边");
            return 0;
        }

        QStringList edge_list;
        for (const auto& e : resp->edges) {
            edge_list << QString("%1  (%2:%3 → %4:%5)")
                .arg(QString::fromStdString(e.id))
                .arg(QString::fromStdString(e.from_ep.type))
                .arg(QString::fromStdString(e.from_ep.id))
                .arg(QString::fromStdString(e.to_ep.type))
                .arg(QString::fromStdString(e.to_ep.id));
        }

        bool ok = false;
        const QString sel = QInputDialog::getItem(
            nullptr, "删除路网边", "选择要删除的边:", edge_list, 0, false, &ok);
        if (!ok || sel.isEmpty()) return 0;

        const QString edge_id = sel.left(sel.indexOf("  ("));
        if (QMessageBox::question(nullptr, "确认删除",
                QString("确定删除边 \"%1\"？").arg(edge_id),
                QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes) {
            return 0;
        }

        auto del_req = std::make_shared<nav_interfaces::srv::SetAssets::Request>(); del_req->edges.resize(1);
        del_req->mode = nav_interfaces::srv::SetAssets::Request::MODE_REMOVE;
        del_req->edges[0].id = edge_id.toStdString();
        remove_edge_client_->async_send_request(del_req,
            [this, edge_id](rclcpp::Client<nav_interfaces::srv::SetAssets>::SharedFuture f) {
                try {
                    auto r = f.get();
                    setStatus(r->success
                        ? QString("边 '%1' 删除成功").arg(edge_id)
                        : QString("删除失败: %1").arg(QString::fromStdString(r->message)));
                } catch (const std::exception& ex) {
                    setStatus(QString("删除异常: %1").arg(ex.what()));
                }
            });
        return 0;
    }

    if (event.rightDown()) {
        return Finished;
    }
    return 0;
}

void DeleteEdgeTool::load(const rviz_common::Config& config) { Tool::load(config); }
void DeleteEdgeTool::save(rviz_common::Config config) const { Tool::save(config); }

}  // namespace map_asset_rviz_plugin

PLUGINLIB_EXPORT_CLASS(map_asset_rviz_plugin::DeleteEdgeTool, rviz_common::Tool)
