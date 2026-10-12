#include "map_asset_rviz_plugin/nav_point_tab.hpp"
#include "map_asset_rviz_plugin/asset_target.hpp"
#include "map_asset_rviz_plugin/nav_point_draft_bridge.hpp"

#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QInputDialog>
#include <QMessageBox>
#include <QVBoxLayout>

namespace map_asset_rviz_plugin {

static constexpr struct { const char* label; uint32_t value; } kTypes[] = {
    {"STATION",  1},
    {"CHARGING", 2},
    {"PARKING",  3},
    {"DOORWAIT", 4},
    {"LIFT",     5},
};

NavPointTab::NavPointTab(QWidget* parent) : QWidget(parent) {
    // ── 列表 ──────────────────────────────────────────────────────
    list_ = new QListWidget();
    list_->setSelectionMode(QAbstractItemView::SingleSelection);

    // ── 输入区 ────────────────────────────────────────────────────
    name_edit_ = new QLineEdit();
    name_edit_->setPlaceholderText("点位名称");

    type_combo_ = new QComboBox();
    for (const auto& t : kTypes) {
        type_combo_->addItem(t.label, static_cast<unsigned int>(t.value));
    }

    x_spin_     = new QDoubleSpinBox();
    y_spin_     = new QDoubleSpinBox();
    theta_spin_ = new QDoubleSpinBox();
    for (auto* s : {x_spin_, y_spin_}) {
        s->setRange(-1000.0, 1000.0);
        s->setDecimals(3);
        s->setSingleStep(0.1);
    }
    theta_spin_->setRange(-3.1416, 3.1416);
    theta_spin_->setDecimals(4);
    theta_spin_->setSingleStep(0.1);

    auto* form = new QFormLayout();
    form->addRow("名称:", name_edit_);
    form->addRow("类型:", type_combo_);
    form->addRow("X (m):", x_spin_);
    form->addRow("Y (m):", y_spin_);
    form->addRow("θ (rad):", theta_spin_);

    auto* input_box = new QGroupBox("手动添加 / 工具选点");
    input_box->setLayout(form);

    // ── 按钮行 ────────────────────────────────────────────────────
    refresh_btn_ = new QPushButton("刷新列表");
    sample_btn_  = new QPushButton("采样点位");
    add_btn_     = new QPushButton("添加点位");
    edit_btn_    = new QPushButton("修改属性");
    remove_btn_  = new QPushButton("删除选中");
    nav_btn_     = new QPushButton("导航到选中");
    cancel_nav_btn_ = new QPushButton("取消导航");
    cancel_nav_btn_->setEnabled(false);

    auto* btn_row1 = new QHBoxLayout();
    btn_row1->addWidget(refresh_btn_);
    btn_row1->addWidget(sample_btn_);

    auto* btn_row2 = new QHBoxLayout();
    btn_row2->addWidget(add_btn_);
    btn_row2->addWidget(edit_btn_);
    btn_row2->addWidget(remove_btn_);

    auto* btn_row3 = new QHBoxLayout();
    btn_row3->addWidget(nav_btn_);
    btn_row3->addWidget(cancel_nav_btn_);

    // ── 状态 ──────────────────────────────────────────────────────
    status_label_ = new QLabel("就绪");
    status_label_->setWordWrap(true);

    // ── 总布局 ────────────────────────────────────────────────────
    auto* root = new QVBoxLayout(this);
    root->addWidget(list_);
    root->addWidget(input_box);
    root->addLayout(btn_row1);
    root->addLayout(btn_row2);
    root->addLayout(btn_row3);
    root->addWidget(status_label_);

    connect(refresh_btn_, &QPushButton::clicked, this, &NavPointTab::onRefresh);
    connect(sample_btn_,  &QPushButton::clicked, this, &NavPointTab::onSample);
    connect(add_btn_,     &QPushButton::clicked, this, &NavPointTab::onAdd);
    connect(edit_btn_,    &QPushButton::clicked, this, &NavPointTab::onEdit);
    connect(remove_btn_,  &QPushButton::clicked, this, &NavPointTab::onRemove);
    connect(nav_btn_,     &QPushButton::clicked, this, &NavPointTab::onNavTo);
    connect(cancel_nav_btn_, &QPushButton::clicked,
            this, &NavPointTab::onCancelNav);

    connect(this, &NavPointTab::refreshDone,        this, &NavPointTab::applyRefresh,         Qt::QueuedConnection);
    connect(this, &NavPointTab::operationFinished,  this, &NavPointTab::applyOperationResult, Qt::QueuedConnection);
    connect(this, &NavPointTab::poseFilled,         this, &NavPointTab::applyPoseFilled,      Qt::QueuedConnection);
    connect(this, &NavPointTab::navigationStatus,
            status_label_, &QLabel::setText, Qt::QueuedConnection);
    connect(this, &NavPointTab::navigationActiveChanged,
            this, &NavPointTab::applyNavigationActive, Qt::QueuedConnection);

    // 工具拖拽完成后预填草稿（跨线程安全）
    connect(NavPointDraftBridge::instance(), &NavPointDraftBridge::draftReady,
            this, &NavPointTab::prefillDraft, Qt::QueuedConnection);
}

void NavPointTab::setNode(rclcpp::Node::SharedPtr node) {
    node_ = node;
    rebuildClients();
}

void NavPointTab::setServicePrefix(const std::string& prefix) {
    prefix_ = prefix;
    if (node_) rebuildClients();
}

void NavPointTab::rebuildClients() {
    {
        std::lock_guard<std::mutex> lock(navigation_goal_mutex_);
        ++navigation_generation_;
        navigation_goal_handle_.reset();
        navigation_active_ = false;
    }
    applyNavigationActive(false);

    get_all_client_ = node_->create_client<nav_interfaces::srv::GetAssets>(prefix_ + "/get_assets");
    add_client_     = node_->create_client<nav_interfaces::srv::SetAssets>(prefix_ + "/set_assets");
    remove_client_  = node_->create_client<nav_interfaces::srv::SetAssets>(prefix_ + "/set_assets");
    sample_client_  = node_->create_client<nav_interfaces::srv::SampleNavPoint>(prefix_ + "/sample_point");
    update_client_  = node_->create_client<nav_interfaces::srv::SetAssets>(prefix_ + "/set_assets");
    nav_client_     = rclcpp_action::create_client<NavigateToPointAction>(
        node_, "/navigation/navigate_to_point");   // 唯一导航入口：navigation 门面（DEC-041）
}

// ──────────────────────────────────────────────────────────────────────
// 槽函数
// ──────────────────────────────────────────────────────────────────────

void NavPointTab::onRefresh() {
    if (!get_all_client_) { status_label_->setText("节点未初始化"); return; }
    status_label_->setText("刷新中...");
    refresh_btn_->setEnabled(false);

    auto req = std::make_shared<nav_interfaces::srv::GetAssets::Request>(); applyTarget(req);
    req->kinds = nav_interfaces::srv::GetAssets::Request::KIND_POINTS;
    get_all_client_->async_send_request(
        req,
        [this](rclcpp::Client<nav_interfaces::srv::GetAssets>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                QVariantList points;
                for (const auto& pt : resp->points) {
                    QVariantMap m;
                    m["name"]      = QString::fromStdString(pt.name);
                    m["type"]      = QString::fromStdString(pt.type);
                    m["x"]         = static_cast<double>(pt.x);
                    m["y"]         = static_cast<double>(pt.y);
                    m["theta"]     = static_cast<double>(pt.theta);
                    m["tolerance"] = static_cast<double>(pt.tolerance);
                    points.append(m);
                }
                Q_EMIT refreshDone(points);
            } catch (const std::exception& ex) {
                Q_EMIT operationFinished(false, QString("刷新失败: %1").arg(ex.what()));
            }
        });
}

void NavPointTab::onSample() {
    if (!sample_client_) { status_label_->setText("节点未初始化"); return; }
    // 采样的是机器人当前位姿，只对当前已加载地图有意义
    if (!AssetTarget::isCurrent()) {
        status_label_->setText(QString("编辑目标是非当前地图 %1，不能录制机器人位姿").arg(QString::fromStdString(AssetTarget::label())));
        return;
    }

    if (!sample_client_->service_is_ready()) {
        status_label_->setText("等待采样服务...");
    }

    // 空 name = 仅采样坐标，不保存；坐标填入 UI 后由用户填写名称再点「添加点位」
    auto req = std::make_shared<nav_interfaces::srv::SampleNavPoint::Request>();
    req->name       = "";
    req->point_type = 0;

    sample_btn_->setEnabled(false);
    status_label_->setText("采样中（11次，约550ms）...");

    sample_client_->async_send_request(
        req,
        [this](rclcpp::Client<nav_interfaces::srv::SampleNavPoint>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                if (resp->success) {
                    Q_EMIT poseFilled(resp->x, resp->y, resp->theta);
                } else {
                    Q_EMIT operationFinished(false, "采样失败（TF 不可用）");
                }
            } catch (const std::exception& ex) {
                Q_EMIT operationFinished(false, QString("采样异常: %1").arg(ex.what()));
            }
        });
}

void NavPointTab::onAdd() {
    if (!add_client_) { status_label_->setText("节点未初始化"); return; }
    const std::string name = name_edit_->text().trimmed().toStdString();
    if (name.empty()) { status_label_->setText("错误：请填写点位名称"); return; }

    auto req = std::make_shared<nav_interfaces::srv::SetAssets::Request>(); applyTarget(req); req->points.resize(1);
    req->mode = nav_interfaces::srv::SetAssets::Request::MODE_MERGE;
    req->points[0].name  = name;
    req->points[0].type  = type_combo_->currentText().toStdString();
    req->points[0].x     = static_cast<float>(x_spin_->value());
    req->points[0].y     = static_cast<float>(y_spin_->value());
    req->points[0].theta = static_cast<float>(theta_spin_->value());

    add_btn_->setEnabled(false);
    status_label_->setText("添加中...");

    add_client_->async_send_request(
        req,
        [this](rclcpp::Client<nav_interfaces::srv::SetAssets>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                Q_EMIT operationFinished(resp->success,
                    resp->success ? "添加成功" : QString("添加失败: %1").arg(
                        assetMessage(*resp)));
            } catch (const std::exception& ex) {
                Q_EMIT operationFinished(false, QString("添加异常: %1").arg(ex.what()));
            }
        });
}

void NavPointTab::onRemove() {
    if (!remove_client_) { status_label_->setText("节点未初始化"); return; }
    auto* sel = list_->currentItem();
    if (!sel) { status_label_->setText("请先选择要删除的点位"); return; }

    // 列表项格式: "[TYPE] name  (x, y, θ)"，提取 name
    QString text = sel->text();
    // 找第一个 "]" 之后、"(" 之前的 name
    int bracket = text.indexOf(']');
    int paren   = text.indexOf('(');
    QString name = (bracket >= 0 && paren > bracket)
        ? text.mid(bracket + 2, paren - bracket - 3).trimmed()
        : text.trimmed();

    if (QMessageBox::question(this, "确认删除",
            QString("确定删除点位 \"%1\"？").arg(name),
            QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes) {
        return;
    }

    auto req = std::make_shared<nav_interfaces::srv::SetAssets::Request>(); applyTarget(req); req->points.resize(1);
    req->mode = nav_interfaces::srv::SetAssets::Request::MODE_REMOVE;
    req->points[0].name = name.toStdString();

    remove_btn_->setEnabled(false);
    status_label_->setText("删除中...");

    remove_client_->async_send_request(
        req,
        [this](rclcpp::Client<nav_interfaces::srv::SetAssets>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                Q_EMIT operationFinished(resp->success,
                    resp->success ? "删除成功" : QString("删除失败: %1").arg(
                        assetMessage(*resp)));
            } catch (const std::exception& ex) {
                Q_EMIT operationFinished(false, QString("删除异常: %1").arg(ex.what()));
            }
        });
}

void NavPointTab::onEdit() {
    if (!update_client_) { status_label_->setText("节点未初始化"); return; }
    auto* sel = list_->currentItem();
    if (!sel) { status_label_->setText("请先选中要修改的点位"); return; }

    QVariantMap m = sel->data(Qt::UserRole).toMap();
    const QString original_name = m["name"].toString();
    if (original_name.isEmpty()) { status_label_->setText("选中项缺少原始数据，请先「刷新列表」"); return; }

    QDialog dlg(this);
    dlg.setWindowTitle(QString("修改点位属性 — %1").arg(original_name));
    dlg.setMinimumWidth(380);

    auto* name_label = new QLabel(original_name);
    name_label->setStyleSheet("font-weight: bold; color: #555;");

    auto* type_combo = new QComboBox();
    for (const auto& t : kTypes) {
        type_combo->addItem(t.label, static_cast<unsigned int>(t.value));
    }
    const int cur = type_combo->findText(m["type"].toString());
    if (cur >= 0) type_combo->setCurrentIndex(cur);

    auto make_xy_spin = []() {
        auto* s = new QDoubleSpinBox();
        s->setRange(-1000.0, 1000.0);
        s->setDecimals(3);
        s->setSingleStep(0.1);
        return s;
    };
    auto* x_spin = make_xy_spin();     x_spin->setValue(m["x"].toDouble());
    auto* y_spin = make_xy_spin();     y_spin->setValue(m["y"].toDouble());
    auto* theta_spin = new QDoubleSpinBox();
    theta_spin->setRange(-3.1416, 3.1416);
    theta_spin->setDecimals(4);
    theta_spin->setSingleStep(0.1);
    theta_spin->setValue(m["theta"].toDouble());

    auto* tol_spin = new QDoubleSpinBox();
    tol_spin->setRange(0.0, 5.0);
    tol_spin->setDecimals(3);
    tol_spin->setSingleStep(0.05);
    tol_spin->setSpecialValueText("默认");
    tol_spin->setValue(m.value("tolerance").toDouble());

    auto* form = new QFormLayout();
    form->addRow("名称 (只读):", name_label);
    form->addRow("类型:",        type_combo);
    form->addRow("X (m):",       x_spin);
    form->addRow("Y (m):",       y_spin);
    form->addRow("θ (rad):",     theta_spin);
    form->addRow("到达精度:",    tol_spin);

    auto* tip = new QLabel("<i>名称作为主键不可修改；如需重命名请先删除后重建。</i>");
    tip->setStyleSheet("color: gray;");
    tip->setWordWrap(true);

    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    auto* root = new QVBoxLayout(&dlg);
    root->addLayout(form);
    root->addWidget(tip);
    root->addWidget(btns);

    if (dlg.exec() != QDialog::Accepted) return;

    auto req = std::make_shared<nav_interfaces::srv::SetAssets::Request>(); applyTarget(req); req->points.resize(1);
    req->mode = nav_interfaces::srv::SetAssets::Request::MODE_UPDATE_ONLY;
    req->points[0].name      = original_name.toStdString();
    req->points[0].type      = type_combo->currentText().toStdString();
    req->points[0].x         = static_cast<float>(x_spin->value());
    req->points[0].y         = static_cast<float>(y_spin->value());
    req->points[0].theta     = static_cast<float>(theta_spin->value());
    req->points[0].tolerance = static_cast<float>(tol_spin->value());

    edit_btn_->setEnabled(false);
    status_label_->setText(QString("更新点位 %1 中...").arg(original_name));

    update_client_->async_send_request(
        req,
        [this, name = original_name](rclcpp::Client<nav_interfaces::srv::SetAssets>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                Q_EMIT operationFinished(resp->success,
                    resp->success ? QString("点位 %1 修改成功").arg(name)
                                  : QString("修改失败: %1").arg(assetMessage(*resp)));
            } catch (const std::exception& ex) {
                Q_EMIT operationFinished(false, QString("更新异常: %1").arg(ex.what()));
            }
        });
}

void NavPointTab::onNavTo() {
    if (!nav_client_) { status_label_->setText("节点未初始化"); return; }
    auto* sel = list_->currentItem();
    if (!sel) { status_label_->setText("请先选择目标点位"); return; }

    const QString name =
        sel->data(Qt::UserRole).toMap().value("name").toString().trimmed();
    if (name.isEmpty()) {
        status_label_->setText("选中项缺少点位名称，请先「刷新列表」");
        return;
    }

    if (!nav_client_->action_server_is_ready()) {
        status_label_->setText(
            QString("导航 Action 未就绪：/navigation/navigate_to_point（%1）")
                .arg(QString::fromStdString(prefix_)));
        return;
    }

    NavigateToPointAction::Goal goal;
    goal.point_name = name.toStdString();
    goal.use_pose = false;

    uint64_t generation;
    {
        std::lock_guard<std::mutex> lock(navigation_goal_mutex_);
        generation = ++navigation_generation_;
        navigation_goal_handle_.reset();
        navigation_active_ = true;
    }
    applyNavigationActive(true);
    cancel_nav_btn_->setEnabled(false);
    status_label_->setText(QString("正在发送导航目标：%1...").arg(name));

    auto options =
        rclcpp_action::Client<NavigateToPointAction>::SendGoalOptions();
    options.goal_response_callback =
        [this, name, generation](const GoalHandleNavigateToPoint::SharedPtr& goal_handle) {
            if (!goal_handle) {
                {
                    std::lock_guard<std::mutex> lock(navigation_goal_mutex_);
                    if (navigation_generation_ == generation) {
                        navigation_goal_handle_.reset();
                        navigation_active_ = false;
                    }
                }
                Q_EMIT navigationActiveChanged(false);
                Q_EMIT operationFinished(
                    false, QString("导航目标被拒绝：%1").arg(name));
                return;
            }
            {
                std::lock_guard<std::mutex> lock(navigation_goal_mutex_);
                if (navigation_generation_ != generation) {
                    return;
                }
                navigation_goal_handle_ = goal_handle;
                navigation_active_ = true;
            }
            Q_EMIT navigationActiveChanged(true);
            Q_EMIT navigationStatus(
                QString("导航目标已接受：%1").arg(name));
        };
    options.feedback_callback =
        [this, name](
            GoalHandleNavigateToPoint::SharedPtr,
            const std::shared_ptr<const NavigateToPointAction::Feedback> feedback) {
            if (!feedback) return;
            Q_EMIT navigationStatus(
                QString("导航到 %1：%2")
                    .arg(name)
                    .arg(QString::fromStdString(feedback->current_status)));
        };
    options.result_callback =
        [this, name, generation](const GoalHandleNavigateToPoint::WrappedResult& result) {
            {
                std::lock_guard<std::mutex> lock(navigation_goal_mutex_);
                if (navigation_generation_ != generation) {
                    return;
                }
                navigation_goal_handle_.reset();
                navigation_active_ = false;
            }
            Q_EMIT navigationActiveChanged(false);
            const QString message = result.result
                ? QString::fromStdString(result.result->message)
                : QString();
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED: {
                    const bool success = result.result && result.result->success;
                    Q_EMIT operationFinished(
                        success,
                        success
                            ? QString("已到达：%1").arg(name)
                            : QString("导航失败：%1").arg(message));
                    break;
                }
                case rclcpp_action::ResultCode::ABORTED:
                    Q_EMIT operationFinished(
                        false, QString("导航中止：%1").arg(message));
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    Q_EMIT operationFinished(false, "导航已取消");
                    break;
                default:
                    Q_EMIT operationFinished(false, "导航结果未知");
                    break;
            }
        };

    try {
        nav_client_->async_send_goal(goal, options);
    } catch (const std::exception& ex) {
        {
            std::lock_guard<std::mutex> lock(navigation_goal_mutex_);
            if (navigation_generation_ == generation) {
                navigation_goal_handle_.reset();
                navigation_active_ = false;
            }
        }
        Q_EMIT navigationActiveChanged(false);
        Q_EMIT operationFinished(
            false, QString("导航异常: %1").arg(ex.what()));
    }
}

void NavPointTab::onCancelNav() {
    if (!nav_client_) {
        status_label_->setText("节点未初始化");
        return;
    }

    GoalHandleNavigateToPoint::SharedPtr goal_handle;
    {
        std::lock_guard<std::mutex> lock(navigation_goal_mutex_);
        goal_handle = navigation_goal_handle_;
    }
    if (!goal_handle) {
        status_label_->setText("当前没有可取消的导航任务");
        applyNavigationActive(false);
        return;
    }

    cancel_nav_btn_->setEnabled(false);
    status_label_->setText("正在发送取消导航请求...");
    try {
        nav_client_->async_cancel_goal(goal_handle);
        status_label_->setText("取消请求已发送，等待导航端确认...");
    } catch (const std::exception& ex) {
        cancel_nav_btn_->setEnabled(true);
        status_label_->setText(QString("取消导航异常: %1").arg(ex.what()));
    }
}

void NavPointTab::applyRefresh(QVariantList points) {
    list_->clear();
    for (const auto& v : points) {
        QVariantMap m = v.toMap();
        const QString text = QString("[%1] %2  (%3, %4, %5 rad)")
            .arg(m["type"].toString())
            .arg(m["name"].toString())
            .arg(m["x"].toDouble(),     0, 'f', 3)
            .arg(m["y"].toDouble(),     0, 'f', 3)
            .arg(m["theta"].toDouble(), 0, 'f', 3);
        auto* item = new QListWidgetItem(text);
        item->setData(Qt::UserRole, v);   // 完整字段供 onEdit 取用
        list_->addItem(item);
    }
    refresh_btn_->setEnabled(true);
    status_label_->setText(QString("共 %1 个点位").arg(points.size()));
}

void NavPointTab::applyOperationResult(bool success, QString msg) {
    status_label_->setText(msg);
    refresh_btn_->setEnabled(true);
    sample_btn_->setEnabled(true);
    add_btn_->setEnabled(true);
    edit_btn_->setEnabled(true);
    remove_btn_->setEnabled(true);
    if (success) onRefresh();
}

void NavPointTab::applyNavigationActive(bool) {
    bool active = false;
    bool has_goal = false;
    {
        std::lock_guard<std::mutex> lock(navigation_goal_mutex_);
        active = navigation_active_;
        has_goal = navigation_goal_handle_ != nullptr;
    }
    nav_btn_->setEnabled(!active);
    cancel_nav_btn_->setEnabled(active && has_goal);
}

void NavPointTab::applyPoseFilled(double x, double y, double yaw_rad) {
    x_spin_->setValue(x);
    y_spin_->setValue(y);
    theta_spin_->setValue(yaw_rad);
    sample_btn_->setEnabled(true);
    status_label_->setText(
        QString("已采样：(%.3f, %.3f) θ=%.3f rad — 填写名称后点「添加点位」")
            .arg(x, 0, 'f', 3).arg(y, 0, 'f', 3).arg(yaw_rad, 0, 'f', 3));
}

void NavPointTab::prefillDraft(double x, double y, double yaw_rad) {
    x_spin_->setValue(x);
    y_spin_->setValue(y);
    theta_spin_->setValue(yaw_rad);
    status_label_->setText(
        QString("已选点：(%.3f, %.3f) θ=%.3f rad — 填写名称后点「添加点位」")
            .arg(x, 0, 'f', 3).arg(y, 0, 'f', 3).arg(yaw_rad, 0, 'f', 3));
}

}  // namespace map_asset_rviz_plugin
