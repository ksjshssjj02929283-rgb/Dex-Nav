// RouteGraphDeleteTool — 统一路网删除工具（快捷键 D）
// 对标 del_tool 交互：
//   左键点击：高亮最近 Junction 或 Edge
//   Del 键：确认删除
//   右键：退出工具

#include "map_asset_rviz_plugin/route_graph_delete_tool.hpp"
#include "map_asset_rviz_plugin/route_graph_tools_common.hpp"

#include <QKeyEvent>
#include <QMetaObject>

#include <OgreManualObject.h>
#include <OgreSceneManager.h>

#include <nav_interfaces/srv/set_assets.hpp>
#include <nav_interfaces/srv/get_route_graph.hpp>
#include <nav_interfaces/srv/get_all_nav_points.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/interaction/view_picker_iface.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/viewport_mouse_event.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <cmath>

namespace map_asset_rviz_plugin {

RouteGraphDeleteTool::RouteGraphDeleteTool() {
    shortcut_key_ = 'd';
    access_all_keys_ = true;
}

RouteGraphDeleteTool::~RouteGraphDeleteTool() {
    if (context_) {
        auto* scene = context_->getSceneManager();
        if (preview_node_) { scene->destroySceneNode(preview_node_); preview_node_ = nullptr; }
        if (preview_obj_)  { scene->destroyManualObject(preview_obj_); preview_obj_ = nullptr; }
    }
}

void RouteGraphDeleteTool::onInitialize() {
    node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
    auto* scene = context_->getSceneManager();

    const std::string name =
        "map_asset_del_preview_" + std::to_string(reinterpret_cast<uintptr_t>(this));
    preview_obj_ = scene->createManualObject(name);
    preview_obj_->setDynamic(true);
    preview_node_ = scene->getRootSceneNode()->createChildSceneNode();
    preview_node_->attachObject(preview_obj_);
    preview_node_->setVisible(false);
}

void RouteGraphDeleteTool::activate() {
    if (!s_route_graph_editing) {
        setStatus("请先点击「开始修改路网 [R]」再操作");
        return;
    }
    reloadGraph();
    sel_type_ = SelType::NONE;
    setStatus("悬停到节点或边上，按 Del 删除；右键退出");
}

void RouteGraphDeleteTool::deactivate() {
    sel_type_ = SelType::NONE;
    clearPreview();
}

int RouteGraphDeleteTool::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
    if (!s_route_graph_editing) return Render;

    Ogre::Vector3 pos;
    const bool got3d =
        context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, pos);
    const double mx = got3d ? pos.x : 0.0;
    const double my = got3d ? pos.y : 0.0;

    // ----- 鼠标移动：实时更新悬停高亮 -----
    if (event.type == QEvent::MouseMove && got3d) {
        last_mx_ = mx;
        last_my_ = my;
        updateHighlight(mx, my);
        if (sel_type_ != SelType::NONE) {
            setStatus(QString("悬停：%1 「%2」，按 Del 删除；右键退出")
                          .arg(sel_type_ == SelType::JUNCTION ? "节点" : "边")
                          .arg(QString::fromStdString(sel_id_)));
        } else {
            setStatus("悬停到节点或边上，按 Del 删除；右键退出");
        }
        return Render;
    }

    // ----- 右键：退出 -----
    if (event.rightDown()) {
        clearPreview();
        return Render | Finished;
    }

    return Render;
}

int RouteGraphDeleteTool::processKeyEvent(QKeyEvent* event,
                                          rviz_common::RenderPanel* /*panel*/) {
    // Del 键：删除当前悬停高亮的项（同 del_tool 的 is_stop=true）
    if (event->key() == Qt::Key_Delete) {
        if (sel_type_ != SelType::NONE) {
            doDelete();
        } else {
            setStatus("无可删除项（请先将鼠标悬停到节点或边上）");
        }
    }
    return Render;
}

// ---------------------------------------------------------------------------

void RouteGraphDeleteTool::reloadGraph() {
    // 使用 loadEndpointsAsync 并行获取 nav 点 + junction（含坐标），
    // 再异步获取边列表 — 修复 nav 端坐标为 0 的问题
    setStatus("正在加载路网…");

    loadEndpointsAsync(node_, service_prefix_,
        [this](std::vector<EndpointInfo> eps) {
            auto cli = node_->create_client<nav_interfaces::srv::GetRouteGraph>(
                service_prefix_ + "/get_route_graph");
            if (!cli->service_is_ready()) {
                QMetaObject::invokeMethod(this, [this]() {
                    setStatus("路网服务不可用");
                }, Qt::QueuedConnection);
                return;
            }
            auto req = std::make_shared<nav_interfaces::srv::GetRouteGraph::Request>();
            cli->async_send_request(req,
                [this, cli, eps](
                    rclcpp::Client<nav_interfaces::srv::GetRouteGraph>::SharedFuture fut) {
                    try {
                        auto resp = fut.get();

                        std::vector<EndpointInfo> new_junctions;
                        std::vector<EdgeInfo>     new_edges;

                        for (const auto& ep : eps) {
                            if (ep.type == "junction") new_junctions.push_back(ep);
                        }

                        // 从全量 eps（含 nav + junction）查坐标
                        for (const auto& edge : resp->edges) {
                            EdgeInfo ei;
                            ei.id = edge.id;
                            for (const auto& ep : eps) {
                                if (ep.id == edge.from_ep.id) { ei.x0 = ep.x; ei.y0 = ep.y; }
                                if (ep.id == edge.to_ep.id)   { ei.x1 = ep.x; ei.y1 = ep.y; }
                            }
                            new_edges.push_back(ei);
                        }

                        {
                            std::lock_guard<std::mutex> lk(data_mutex_);
                            junctions_ = std::move(new_junctions);
                            edges_     = std::move(new_edges);
                            all_eps_   = eps;
                        }

                        QMetaObject::invokeMethod(this, [this]() {
                            if (preview_obj_ && preview_node_ && context_) {
                                refreshPreview();
                                context_->queueRender();
                            }
                        }, Qt::QueuedConnection);

                        setStatus("路网已加载，悬停选中后按 Del 删除；右键退出");
                    } catch (const std::exception& ex) {
                        setStatus(QString("路网加载失败: %1").arg(ex.what()));
                    }
                });
        });
}

void RouteGraphDeleteTool::updateHighlight(double mx, double my) {
    // 快照数据，避免在渲染期间与 reloadGraph 回调产生竞争
    std::vector<EndpointInfo> junctions_snap;
    std::vector<EdgeInfo>     edges_snap;
    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        junctions_snap = junctions_;
        edges_snap     = edges_;
    }

    sel_type_ = SelType::NONE;
    sel_id_.clear();

    preview_obj_->clear();
    preview_node_->setVisible(true);

    // 1. 始终显示所有节点（蓝色小圆圈）
    for (const auto& jct : junctions_snap) {
        drawRing(preview_obj_, jct.x, jct.y, kPreviewZ, 0.18f, 0.3f, 0.6f, 1.0f);
    }

    // 2. 始终显示所有边（绿色细线）
    for (const auto& e : edges_snap) {
        drawSegment(preview_obj_, e.x0, e.y0, e.x1, e.y1, kPreviewZ, 0.2f, 0.7f, 0.2f);
    }

    // 3. 寻找最近 Junction
    const EndpointInfo* jct = findNearest(mx, my, junctions_snap, kSnapRadius);
    float best_dist = kSnapRadius;

    if (jct) {
        float dx = jct->x - static_cast<float>(mx);
        float dy = jct->y - static_cast<float>(my);
        best_dist = std::sqrt(dx * dx + dy * dy);
        sel_type_ = SelType::JUNCTION;
        sel_id_   = jct->id;
        sel_x_    = jct->x;
        sel_y_    = jct->y;
    }

    // 4. 寻找最近 Edge，若比 Junction 更近则优先
    for (const auto& e : edges_snap) {
        float d = distToSegment(
            static_cast<float>(mx), static_cast<float>(my),
            e.x0, e.y0, e.x1, e.y1);
        if (d < best_dist) {
            best_dist = d;
            sel_type_ = SelType::EDGE;
            sel_id_   = e.id;
            sel_x_    = (e.x0 + e.x1) * 0.5f;
            sel_y_    = (e.y0 + e.y1) * 0.5f;
        }
    }

    // 5. 对最近项叠加红色高亮
    if (sel_type_ == SelType::JUNCTION) {
        // 红色大圆圈＋十字
        drawRing(preview_obj_, sel_x_, sel_y_, kPreviewZ, 0.28f, 1.0f, 0.2f, 0.2f);
        drawCross(preview_obj_, sel_x_, sel_y_, kPreviewZ, 0.12f, 1.0f, 0.2f, 0.2f);
    } else if (sel_type_ == SelType::EDGE) {
        // 红色边线强调
        for (const auto& e : edges_snap) {
            if (e.id == sel_id_) {
                drawSegment(preview_obj_,
                            e.x0, e.y0, e.x1, e.y1,
                            kPreviewZ + 0.01f, 1.0f, 0.2f, 0.2f);
                drawCross(preview_obj_, sel_x_, sel_y_, kPreviewZ, 0.12f, 1.0f, 0.2f, 0.2f);
                break;
            }
        }
    }
}

void RouteGraphDeleteTool::refreshPreview() {
    // 仅绘制所有元素（蓝圈/绿线），不做 snap 高亮
    // 在数据异步加载完成后立即调用，避免等待下一次鼠标移动才刷新
    std::vector<EndpointInfo> junctions_snap;
    std::vector<EdgeInfo>     edges_snap;
    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        junctions_snap = junctions_;
        edges_snap     = edges_;
    }

    preview_obj_->clear();
    for (const auto& jct : junctions_snap) {
        drawRing(preview_obj_, jct.x, jct.y, kPreviewZ, 0.18f, 0.3f, 0.6f, 1.0f);
    }
    for (const auto& e : edges_snap) {
        drawSegment(preview_obj_, e.x0, e.y0, e.x1, e.y1, kPreviewZ, 0.2f, 0.7f, 0.2f);
    }
    preview_node_->setVisible(!junctions_snap.empty() || !edges_snap.empty());
}

void RouteGraphDeleteTool::clearPreview() {
    if (preview_obj_)  preview_obj_->clear();
    if (preview_node_) preview_node_->setVisible(false);
}

void RouteGraphDeleteTool::doDelete() {
    const QString qid = QString::fromStdString(sel_id_);

    if (sel_type_ == SelType::JUNCTION) {
        auto cli = node_->create_client<nav_interfaces::srv::SetAssets>(
            service_prefix_ + "/set_assets");
        auto req = std::make_shared<nav_interfaces::srv::SetAssets::Request>(); req->junctions.resize(1);
        req->mode = nav_interfaces::srv::SetAssets::Request::MODE_REMOVE;
        req->junctions[0].id = sel_id_;
        sel_type_ = SelType::NONE;
        clearPreview();
        cli->async_send_request(req,
            [this, qid](
                rclcpp::Client<nav_interfaces::srv::SetAssets>::SharedFuture fut) {
                try {
                    auto resp = fut.get();
                    if (resp->success) {
                        reloadGraph();
                        setStatus(QString("节点 '%1' 已删除，继续点击或右键退出").arg(qid));
                    } else {
                        setStatus(QString("删除失败: %1").arg(
                            QString::fromStdString(resp->message)));
                    }
                } catch (const std::exception& ex) {
                    setStatus(QString("异常: %1").arg(ex.what()));
                }
            });
    } else if (sel_type_ == SelType::EDGE) {
        auto cli = node_->create_client<nav_interfaces::srv::SetAssets>(
            service_prefix_ + "/set_assets");
        auto req = std::make_shared<nav_interfaces::srv::SetAssets::Request>(); req->edges.resize(1);
        req->mode = nav_interfaces::srv::SetAssets::Request::MODE_REMOVE;
        req->edges[0].id = sel_id_;
        sel_type_ = SelType::NONE;
        clearPreview();
        cli->async_send_request(req,
            [this, qid](
                rclcpp::Client<nav_interfaces::srv::SetAssets>::SharedFuture fut) {
                try {
                    auto resp = fut.get();
                    if (resp->success) {
                        reloadGraph();
                        setStatus(QString("边 '%1' 已删除，继续点击或右键退出").arg(qid));
                    } else {
                        setStatus(QString("删除失败: %1").arg(
                            QString::fromStdString(resp->message)));
                    }
                } catch (const std::exception& ex) {
                    setStatus(QString("异常: %1").arg(ex.what()));
                }
            });
    }
}

float RouteGraphDeleteTool::distToSegment(float px, float py,
                                          float x0, float y0,
                                          float x1, float y1) const {
    const float dx = x1 - x0;
    const float dy = y1 - y0;
    const float len2 = dx * dx + dy * dy;
    if (len2 < 1e-6f) {
        // 退化为点
        float ex = px - x0, ey = py - y0;
        return std::sqrt(ex * ex + ey * ey);
    }
    float t = ((px - x0) * dx + (py - y0) * dy) / len2;
    t = std::max(0.0f, std::min(1.0f, t));
    float cx = x0 + t * dx;
    float cy = y0 + t * dy;
    float ex = px - cx, ey = py - cy;
    return std::sqrt(ex * ex + ey * ey);
}

void RouteGraphDeleteTool::load(const rviz_common::Config& config) { Tool::load(config); }
void RouteGraphDeleteTool::save(rviz_common::Config config) const  { Tool::save(config); }

}  // namespace map_asset_rviz_plugin

PLUGINLIB_EXPORT_CLASS(map_asset_rviz_plugin::RouteGraphDeleteTool, rviz_common::Tool)
