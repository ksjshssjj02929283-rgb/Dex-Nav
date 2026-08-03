// AddEdgeTool — 直线路网边工具（快捷键 L）
// 对标 line_tool 交互逻辑：
//   左键选起点（snap 到最近端点）→ selecting_=true，实时预览连线
//   右键完成：snap 当前鼠标到最近端点为终点 → 弹出配置框 → 提交
//   右键（未选起点时）→ 退出工具

#include "map_asset_rviz_plugin/add_edge_tool.hpp"
#include "map_asset_rviz_plugin/route_graph_tools_common.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>

#include <OgreManualObject.h>
#include <OgreSceneManager.h>

#include <nav_interfaces/srv/add_route_edge.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/interaction/view_picker_iface.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/viewport_mouse_event.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace map_asset_rviz_plugin {

// ---------------------------------------------------------------------------
// 边配置弹窗
// ---------------------------------------------------------------------------
namespace {

EdgeConfig showEdgeConfigDialog(const EndpointInfo& from, const EndpointInfo& to,
                                const std::string& edge_type_label)
{
    QDialog dlg;
    dlg.setWindowTitle(QString("路网边配置 — %1")
                           .arg(QString::fromStdString(edge_type_label)));
    auto* form = new QFormLayout(&dlg);

    form->addRow("类型:", new QLabel(QString::fromStdString(edge_type_label)));
    form->addRow("起点:", new QLabel(
        QString::fromStdString("[" + from.type + "] " + from.id)));
    form->addRow("终点:", new QLabel(
        QString::fromStdString("[" + to.type + "] " + to.id)));

    auto* id_edit    = new QLineEdit(
        QString::fromStdString(from.id + "_" + to.id));
    auto* bidir_cb   = new QCheckBox("单向通行");
    auto* speed_spin = new QDoubleSpinBox();
    speed_spin->setRange(0.0, 10.0); speed_spin->setValue(0.8); speed_spin->setSuffix(" m/s");
    auto* xtol_spin  = new QDoubleSpinBox();
    xtol_spin->setRange(0.0, 5.0);   xtol_spin->setValue(0.3); xtol_spin->setSuffix(" m");
    auto* ttol_spin  = new QDoubleSpinBox();
    ttol_spin->setRange(0.0, 6.28);  ttol_spin->setValue(0.5); ttol_spin->setSuffix(" rad");
    auto* width_spin = new QDoubleSpinBox();
    width_spin->setRange(0.0, 10.0); width_spin->setValue(0.0); width_spin->setSuffix(" m (0=不限)");
    auto* policy_box = new QComboBox();
    policy_box->addItems({"STOP", "DETOUR"});

    form->addRow("边 ID:",    id_edit);
    form->addRow("",          bidir_cb);
    form->addRow("最大速度:", speed_spin);
    form->addRow("XY 容差:",  xtol_spin);
    form->addRow("朝向容差:", ttol_spin);
    form->addRow("通道宽度:", width_spin);
    form->addRow("障碍策略:", policy_box);

    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    QObject::connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    QObject::connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    form->addRow(btns);

    EdgeConfig res;
    if (dlg.exec() == QDialog::Accepted) {
        res.accepted      = true;
        res.edge_id       = id_edit->text().toStdString();
        res.bidirectional = !bidir_cb->isChecked();
        res.max_speed     = static_cast<float>(speed_spin->value());
        res.xy_tolerance  = static_cast<float>(xtol_spin->value());
        res.th_tolerance  = static_cast<float>(ttol_spin->value());
        res.lane_width    = static_cast<float>(width_spin->value());
        res.obs_policy    = policy_box->currentText().toStdString();
    }
    return res;
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
// AddEdgeTool
// ---------------------------------------------------------------------------

AddEdgeTool::AddEdgeTool() {
    shortcut_key_ = 'l';
}

AddEdgeTool::~AddEdgeTool() {
    if (context_) {
        auto* scene = context_->getSceneManager();
        if (preview_node_) { scene->destroySceneNode(preview_node_); preview_node_ = nullptr; }
        if (preview_obj_)  { scene->destroyManualObject(preview_obj_); preview_obj_ = nullptr; }
    }
}

void AddEdgeTool::onInitialize() {
    node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
    auto* scene = context_->getSceneManager();

    const std::string name =
        "map_asset_edge_preview_" + std::to_string(reinterpret_cast<uintptr_t>(this));
    preview_obj_ = scene->createManualObject(name);
    preview_obj_->setDynamic(true);
    preview_node_ = scene->getRootSceneNode()->createChildSceneNode();
    preview_node_->attachObject(preview_obj_);
    preview_node_->setVisible(false);
}

void AddEdgeTool::activate() {
    if (!s_route_graph_editing) {
        setStatus("请先点击「开始修改路网 [R]」再操作");
        return;
    }
    selecting_  = false;
    loading_    = true;
    endpoints_.clear();
    setStatus("正在加载端点列表…");

    // 异步加载：nav 点和 junction 并行请求，回调到 Qt 主线程刷新预览
    loadEndpointsAsync(node_, service_prefix_,
        [this](std::vector<EndpointInfo> eps) {
            QMetaObject::invokeMethod(this,
                [this, eps = std::move(eps)]() mutable {
                    endpoints_ = std::move(eps);
                    loading_   = false;
                    updatePreview(last_mx_, last_my_);
                    if (context_) context_->queueRender();
                    setStatus(endpoints_.empty()
                        ? "当前没有可用端点（先添加 NavPoint 或 Junction）"
                        : QString("已加载 %1 个端点，左键选起点，右键退出")
                              .arg(static_cast<int>(endpoints_.size())));
                }, Qt::QueuedConnection);
        });
}

void AddEdgeTool::deactivate() {
    // 同 line_tool.deactivate：若还在选择中，丢弃（服务版不需要发布）
    selecting_ = false;
    clearPreview();
}

int AddEdgeTool::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
    if (!s_route_graph_editing) return Render;
    Ogre::Vector3 pos;
    const bool got3d =
        context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, pos);
    const double mx = got3d ? pos.x : 0.0;
    const double my = got3d ? pos.y : 0.0;

    // ----- 鼠标移动：实时预览（同 line_tool 的 mouse_pos 发布）-----
    if (event.type == QEvent::MouseMove && got3d) {
        last_mx_ = mx;
        last_my_ = my;
        updatePreview(mx, my);
        return Render;
    }

    // ----- 左键：选起点（第一次）或选终点并提交（第二次起，链式）-----
    if (event.leftDown() && got3d) {
        last_mx_ = mx;
        last_my_ = my;
        if (!selecting_) {
            // 第一次左键：snap 到起点（同 line_tool 第一次 leftDown）
            const EndpointInfo* ep = findNearest(mx, my, endpoints_);
            if (!ep) {
                setStatus("附近没有端点（需在 0.5m 范围内点击）");
                return Render;
            }
            from_ep_   = *ep;
            selecting_ = true;
            setStatus(QString("起点 [%1] %2 — 左键选终点，右键取消")
                          .arg(QString::fromStdString(ep->type))
                          .arg(QString::fromStdString(ep->id)));
            updatePreview(mx, my);
            return Render;
        }

        // 第二次左键（selecting_=true）：snap 终点 → 弹对话框 → 提交 → 链式继续
        // 对标 line_tool 的 leftDown → AddLine → 新点成为下一次的起点
        const EndpointInfo* ep =
            findNearest(mx, my, endpoints_, kSnapRadius, from_ep_.id);
        if (!ep) {
            setStatus("附近没有可选终点（需在 0.5m 范围内）");
            return Render;
        }

        // 弹出配置框
        clearPreview();
        auto cfg = showEdgeConfigDialog(from_ep_, *ep, "LINE 直线");
        if (!cfg.accepted) {
            // 取消对话框：保持起点不变，继续选终点
            setStatus(QString("取消，起点仍为 [%1] %2")
                          .arg(QString::fromStdString(from_ep_.type))
                          .arg(QString::fromStdString(from_ep_.id)));
            updatePreview(mx, my);
            return Render;
        }

        // 提交边，并将终点设为新起点（链式，对标 line_tool 的 P2 自动成为下一段的 P1）
        EndpointInfo to_ep = *ep;
        submitEdge(from_ep_, to_ep, cfg);
        from_ep_ = to_ep;
        setStatus(QString("边已提交，新起点 [%1] %2 — 左键继续连边，右键退出")
                      .arg(QString::fromStdString(from_ep_.type))
                      .arg(QString::fromStdString(from_ep_.id)));
        updatePreview(mx, my);
        return Render;
    }

    // ----- 右键：取消当前起点或退出工具（同 line_tool 的 deactivate 行为）-----
    if (event.rightDown()) {
        if (selecting_) {
            // 取消当前段，保持工具激活（再次右键退出）
            selecting_ = false;
            clearPreview();
            setStatus("已取消，再次右键退出工具");
            return Render;
        }
        // 未选中状态：退出工具
        clearPreview();
        return Render | Finished;
    }

    return Render;
}

void AddEdgeTool::updatePreview(double mx, double my) {
    if (!preview_obj_) return;
    preview_obj_->clear();
    preview_node_->setVisible(true);

    // 始终显示所有可用端点（蓝色小圈），方便用户识别点击位置
    for (const auto& ep : endpoints_) {
        drawRing(preview_obj_, ep.x, ep.y, kPreviewZ, 0.15f, 0.3f, 0.6f, 1.0f, 12);
    }

    if (!selecting_) {
        // 高亮最近可 snap 的端点（黄色大圈）
        const EndpointInfo* ep = findNearest(mx, my, endpoints_);
        if (ep) {
            drawRing(preview_obj_, ep->x, ep->y, kPreviewZ, 0.22f, 1.0f, 0.9f, 0.1f);
        }
        return;
    }

    // 已选起点（绿色大圈）
    drawRing(preview_obj_, from_ep_.x, from_ep_.y, kPreviewZ, 0.22f, 0.1f, 1.0f, 0.3f);

    // 预览连线：起点 → 鼠标（青色）
    drawSegment(preview_obj_,
                from_ep_.x, from_ep_.y,
                static_cast<float>(mx), static_cast<float>(my),
                kPreviewZ, 0.2f, 0.8f, 1.0f);

    // 若当前鼠标附近有可 snap 的终点，高亮它（黄色）并改用 snap 连线（绿色）
    const EndpointInfo* ep = findNearest(mx, my, endpoints_, kSnapRadius, from_ep_.id);
    if (ep) {
        drawRing(preview_obj_, ep->x, ep->y, kPreviewZ, 0.22f, 1.0f, 0.9f, 0.1f);
        drawSegment(preview_obj_,
                    from_ep_.x, from_ep_.y,
                    ep->x, ep->y,
                    kPreviewZ + 0.01f, 0.1f, 1.0f, 0.3f);
    }
}

void AddEdgeTool::clearPreview() {
    if (preview_obj_)  preview_obj_->clear();
    if (preview_node_) preview_node_->setVisible(false);
}

void AddEdgeTool::submitEdge(const EndpointInfo& from, const EndpointInfo& to,
                              const EdgeConfig& cfg) {
    auto client = node_->create_client<nav_interfaces::srv::AddRouteEdge>(
        service_prefix_ + "/add_route_edge");
    auto req                 = std::make_shared<nav_interfaces::srv::AddRouteEdge::Request>();
    req->edge.id             = cfg.edge_id;
    req->edge.from_ep.id     = from.id;
    req->edge.from_ep.type   = from.type;
    req->edge.to_ep.id       = to.id;
    req->edge.to_ep.type     = to.type;
    req->edge.edge_type      = "LINE";
    req->edge.bidirectional  = cfg.bidirectional;
    req->edge.max_speed      = cfg.max_speed;
    req->edge.xy_tolerance   = cfg.xy_tolerance;
    req->edge.th_tolerance   = cfg.th_tolerance;
    req->edge.lane_width     = cfg.lane_width;
    req->edge.obs_policy     = cfg.obs_policy;

    const QString eid = QString::fromStdString(cfg.edge_id);
    client->async_send_request(req,
        [this, eid](rclcpp::Client<nav_interfaces::srv::AddRouteEdge>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                setStatus(resp->success
                    ? QString("边 '%1' 添加成功").arg(eid)
                    : QString("添加失败: %1").arg(
                          QString::fromStdString(resp->message)));
            } catch (const std::exception& ex) {
                setStatus(QString("异常: %1").arg(ex.what()));
            }
        });
}

void AddEdgeTool::load(const rviz_common::Config& config) { Tool::load(config); }
void AddEdgeTool::save(rviz_common::Config config) const  { Tool::save(config); }

}  // namespace map_asset_rviz_plugin

PLUGINLIB_EXPORT_CLASS(map_asset_rviz_plugin::AddEdgeTool, rviz_common::Tool)
