// AddCurveEdgeTool — 贝塞尔曲线路网边工具（快捷键 B）
// 对标 curve_line_tool 交互逻辑：
//   左键选起点（snap）→ selecting_=true
//   后续左键添加自由控制点（不 snap，实时贝塞尔预览）
//   右键完成：snap 当前鼠标到最近端点为终点 → 弹出配置框 → 提交

#include "map_asset_rviz_plugin/add_curve_edge_tool.hpp"
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

#include <nav_interfaces/msg/route_edge.hpp>
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

EdgeConfig showCurveConfigDialog(const EndpointInfo& from, const EndpointInfo& to,
                                 int ctrl_count)
{
    QDialog dlg;
    dlg.setWindowTitle("路网边配置 — BEZIER 曲线");
    auto* form = new QFormLayout(&dlg);

    form->addRow("起点:",     new QLabel(
        QString::fromStdString("[" + from.type + "] " + from.id)));
    form->addRow("终点:",     new QLabel(
        QString::fromStdString("[" + to.type + "] " + to.id)));
    form->addRow("控制点数:", new QLabel(QString::number(ctrl_count) + " 个"));

    auto* id_edit    = new QLineEdit(
        QString::fromStdString(from.id + "_" + to.id + "_curve"));
    auto* bidir_cb   = new QCheckBox("单向通行");
    auto* speed_spin = new QDoubleSpinBox();
    speed_spin->setRange(0.0, 10.0); speed_spin->setValue(0.5); speed_spin->setSuffix(" m/s");
    auto* xtol_spin  = new QDoubleSpinBox();
    xtol_spin->setRange(0.0, 5.0);   xtol_spin->setValue(0.3); xtol_spin->setSuffix(" m");
    auto* ttol_spin  = new QDoubleSpinBox();
    ttol_spin->setRange(0.0, 6.28);  ttol_spin->setValue(0.8); ttol_spin->setSuffix(" rad");
    auto* width_spin = new QDoubleSpinBox();
    width_spin->setRange(0.0, 10.0); width_spin->setValue(0.0); width_spin->setSuffix(" m (0=不限)");
    auto* policy_box = new QComboBox();
    policy_box->addItems({"STOP", "DETOUR"});

    form->addRow("边 ID:",    id_edit);
    form->addRow("",          bidir_cb);
    auto opt_w = addOptionalEdgeFieldRows(form);
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
    res.edge_type = "BEZIER";
    if (dlg.exec() == QDialog::Accepted) {
        res.accepted      = true;
        res.edge_id       = id_edit->text().toStdString();
        res.bidirectional = !bidir_cb->isChecked();
        res.max_speed     = static_cast<float>(speed_spin->value());
        res.xy_tolerance  = static_cast<float>(xtol_spin->value());
        res.th_tolerance  = static_cast<float>(ttol_spin->value());
        res.lane_width    = static_cast<float>(width_spin->value());
        res.obs_policy    = policy_box->currentText().toStdString();
        readOptionalEdgeFields(opt_w, res);
    }
    return res;
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
// AddCurveEdgeTool
// ---------------------------------------------------------------------------

AddCurveEdgeTool::AddCurveEdgeTool() {
    shortcut_key_ = 'b';
}

AddCurveEdgeTool::~AddCurveEdgeTool() {
    if (context_) {
        auto* scene = context_->getSceneManager();
        if (preview_node_) { scene->destroySceneNode(preview_node_); preview_node_ = nullptr; }
        if (preview_obj_)  { scene->destroyManualObject(preview_obj_); preview_obj_ = nullptr; }
    }
}

void AddCurveEdgeTool::onInitialize() {
    node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
    auto* scene = context_->getSceneManager();

    const std::string name =
        "map_asset_curve_preview_" + std::to_string(reinterpret_cast<uintptr_t>(this));
    preview_obj_ = scene->createManualObject(name);
    preview_obj_->setDynamic(true);
    preview_node_ = scene->getRootSceneNode()->createChildSceneNode();
    preview_node_->attachObject(preview_obj_);
    preview_node_->setVisible(false);
}

void AddCurveEdgeTool::activate() {
    if (!s_route_graph_editing) {
        setStatus("请先点击「开始修改路网 [R]」再操作");
        return;
    }
    ctrl_pts_.clear();
    selecting_  = false;
    loading_    = true;
    endpoints_.clear();
    setStatus("正在加载端点列表…");

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
                        : QString("已加载 %1 个端点，左键选曲线起点，右键退出")
                              .arg(static_cast<int>(endpoints_.size())));
                }, Qt::QueuedConnection);
        });
    selecting_ = false;
    setStatus(endpoints_.empty()
        ? "当前没有可用端点（先添加 NavPoint 或 Junction）"
        : QString("左键选择曲线起点（共 %1 个端点），右键退出")
              .arg(static_cast<int>(endpoints_.size())));
}

void AddCurveEdgeTool::deactivate() {
    // 同 curve_line_tool.deactivate：若还在选择中，丢弃（服务版无需发布）
    selecting_ = false;
    ctrl_pts_.clear();
    clearPreview();
}

int AddCurveEdgeTool::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
    if (!s_route_graph_editing) return Render;
    Ogre::Vector3 pos;
    const bool got3d =
        context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, pos);
    const double mx = got3d ? pos.x : 0.0;
    const double my = got3d ? pos.y : 0.0;

    // ----- 鼠标移动：实时贝塞尔预览（同 curve_line_tool 的 mouse_pos 发布）-----
    if (event.type == QEvent::MouseMove && got3d) {
        last_mx_ = mx;
        last_my_ = my;
        updatePreview(mx, my);
        return Render;
    }

    // ----- 左键 -----
    if (event.leftDown() && got3d) {
        if (!selecting_) {
            // 第一次左键：snap 到最近端点作为起点
            const EndpointInfo* ep = findNearest(mx, my, endpoints_);
            if (!ep) {
                setStatus("附近没有端点（需在 0.5m 范围内点击）");
                return Render;
            }
            from_ep_   = *ep;
            selecting_ = true;
            ctrl_pts_.clear();
            setStatus(QString("起点 [%1] %2，左键添加控制点，右键完成")
                          .arg(QString::fromStdString(ep->type))
                          .arg(QString::fromStdString(ep->id)));
            return Render;
        }

        // 后续左键：添加自由控制点（不 snap）
        ctrl_pts_.push_back({static_cast<float>(mx), static_cast<float>(my)});
        setStatus(QString("已添加 %1 个控制点，继续添加或右键完成")
                      .arg(static_cast<int>(ctrl_pts_.size())));
        updatePreview(mx, my);
        return Render;
    }

    // ----- 右键（同 curve_line_tool 的 rightDown && selecting）-----
    if (event.rightDown()) {
        if (selecting_) {
            // snap 当前鼠标到最近端点 → 终点
            const EndpointInfo* ep =
                findNearest(mx, my, endpoints_, kSnapRadius, from_ep_.id);
            if (!ep) {
                setStatus("附近没有终点端点（需在 0.5m 范围内），再右键可退出");
                selecting_ = false;
                ctrl_pts_.clear();
                clearPreview();
                return Render;
            }

            // 将终点信息保存，供 submitEdge 使用
            to_ep_ = *ep;

            clearPreview();
            auto cfg = showCurveConfigDialog(from_ep_, to_ep_,
                                             static_cast<int>(ctrl_pts_.size()));
            selecting_ = false;
            if (!cfg.accepted) {
                ctrl_pts_.clear();
                setStatus("取消，可重新选择起点");
                return Render;
            }

            submitEdge(cfg);
            ctrl_pts_.clear();
            setStatus("曲线边已提交，左键继续或右键退出");
            return Render;
        }
        // 未在选择中：退出工具
        clearPreview();
        return Render | Finished;
    }

    return Render;
}

void AddCurveEdgeTool::updatePreview(double mx, double my) {
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

    // 起点（绿色）
    drawRing(preview_obj_, from_ep_.x, from_ep_.y, kPreviewZ, 0.22f, 0.1f, 1.0f, 0.3f);

    // 已有控制点（蓝色叉）
    for (const auto& cp : ctrl_pts_) {
        drawCross(preview_obj_, cp.first, cp.second, kPreviewZ, 0.08f, 0.3f, 0.5f, 1.0f);
    }

    // 当前鼠标（白色叉）
    drawCross(preview_obj_,
              static_cast<float>(mx), static_cast<float>(my),
              kPreviewZ, 0.08f, 1.0f, 1.0f, 1.0f);

    // 候选终点高亮（黄色）
    const EndpointInfo* ep = findNearest(mx, my, endpoints_, kSnapRadius, from_ep_.id);
    if (ep) {
        drawRing(preview_obj_, ep->x, ep->y, kPreviewZ, 0.22f, 1.0f, 0.9f, 0.1f);
    }

    // 构建贝塞尔控制点序列：起点 + 已有控制点 + 鼠标（作为预览的末尾控制点）
    std::vector<std::pair<float, float>> bezier_ctrl;
    bezier_ctrl.push_back({from_ep_.x, from_ep_.y});
    for (const auto& cp : ctrl_pts_) bezier_ctrl.push_back(cp);
    // 若鼠标 snap 到终点，用终点；否则用鼠标
    if (ep) {
        bezier_ctrl.push_back({ep->x, ep->y});
    } else {
        bezier_ctrl.push_back({static_cast<float>(mx), static_cast<float>(my)});
    }

    if (bezier_ctrl.size() >= 2) {
        drawBezierCurve(preview_obj_, bezier_ctrl, kPreviewZ, 0.7f, 0.3f, 1.0f);
    }
}

void AddCurveEdgeTool::clearPreview() {
    if (preview_obj_)  preview_obj_->clear();
    if (preview_node_) preview_node_->setVisible(false);
}

void AddCurveEdgeTool::submitEdge(const EdgeConfig& cfg) {
    auto client = node_->create_client<nav_interfaces::srv::AddRouteEdge>(
        service_prefix_ + "/add_route_edge");
    auto req                 = std::make_shared<nav_interfaces::srv::AddRouteEdge::Request>();
    req->edge.id             = cfg.edge_id;
    req->edge.from_ep.id     = from_ep_.id;
    req->edge.from_ep.type   = from_ep_.type;
    req->edge.to_ep.id       = to_ep_.id;
    req->edge.to_ep.type     = to_ep_.type;
    req->edge.edge_type      = "BEZIER";
    req->edge.bidirectional  = cfg.bidirectional;
    req->edge.max_speed      = cfg.max_speed;
    req->edge.xy_tolerance   = cfg.xy_tolerance;
    req->edge.th_tolerance   = cfg.th_tolerance;
    req->edge.lane_width     = cfg.lane_width;
    req->edge.obs_policy     = cfg.obs_policy;
    applyOptionalEdgeFields(req->edge, cfg);  // 含 max_speed（0=系统默认）/hold_direction/padding/backward

    // 控制点序列：起点 + 中间控制点 + 终点
    auto addPt = [&](float x, float y) {
        geometry_msgs::msg::Point p;
        p.x = x; p.y = y; p.z = 0.0;
        req->edge.control_points.push_back(p);
    };
    addPt(from_ep_.x, from_ep_.y);
    for (const auto& cp : ctrl_pts_) addPt(cp.first, cp.second);
    addPt(to_ep_.x, to_ep_.y);

    const QString eid = QString::fromStdString(cfg.edge_id);
    client->async_send_request(req,
        [this, eid](rclcpp::Client<nav_interfaces::srv::AddRouteEdge>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                setStatus(resp->success
                    ? QString("曲线边 '%1' 添加成功").arg(eid)
                    : QString("添加失败: %1").arg(
                          QString::fromStdString(resp->message)));
            } catch (const std::exception& ex) {
                setStatus(QString("异常: %1").arg(ex.what()));
            }
        });
}

void AddCurveEdgeTool::load(const rviz_common::Config& config) { Tool::load(config); }
void AddCurveEdgeTool::save(rviz_common::Config config) const  { Tool::save(config); }

}  // namespace map_asset_rviz_plugin

PLUGINLIB_EXPORT_CLASS(map_asset_rviz_plugin::AddCurveEdgeTool, rviz_common::Tool)
