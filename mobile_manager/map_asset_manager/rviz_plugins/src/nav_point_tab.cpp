#include "map_asset_rviz_plugin/nav_point_tab.hpp"

#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QInputDialog>
#include <QMessageBox>
#include <QVBoxLayout>

#include "map_asset_rviz_plugin/nav_point_draft_bridge.hpp"

namespace map_asset_rviz_plugin {

static constexpr struct {
    const char* label;
    uint32_t value;
} kTypes[] = {
    {"STATION", 1}, {"CHARGING", 2}, {"PARKING", 3}, {"DOORWAIT", 4}, {"LIFT", 5},
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

    x_spin_ = new QDoubleSpinBox();
    y_spin_ = new QDoubleSpinBox();
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
    sample_btn_ = new QPushButton("采样点位");
    add_btn_ = new QPushButton("添加点位");
    remove_btn_ = new QPushButton("删除选中");
    nav_btn_ = new QPushButton("导航到选中");

    auto* btn_row1 = new QHBoxLayout();
    btn_row1->addWidget(refresh_btn_);
    btn_row1->addWidget(sample_btn_);

    auto* btn_row2 = new QHBoxLayout();
    btn_row2->addWidget(add_btn_);
    btn_row2->addWidget(remove_btn_);
    btn_row2->addWidget(nav_btn_);

    // ── 状态 ──────────────────────────────────────────────────────
    status_label_ = new QLabel("就绪");
    status_label_->setWordWrap(true);

    // ── 总布局 ────────────────────────────────────────────────────
    auto* root = new QVBoxLayout(this);
    root->addWidget(list_);
    root->addWidget(input_box);
    root->addLayout(btn_row1);
    root->addLayout(btn_row2);
    root->addWidget(status_label_);

    connect(refresh_btn_, &QPushButton::clicked, this, &NavPointTab::onRefresh);
    connect(sample_btn_, &QPushButton::clicked, this, &NavPointTab::onSample);
    connect(add_btn_, &QPushButton::clicked, this, &NavPointTab::onAdd);
    connect(remove_btn_, &QPushButton::clicked, this, &NavPointTab::onRemove);
    connect(nav_btn_, &QPushButton::clicked, this, &NavPointTab::onNavTo);

    connect(this, &NavPointTab::refreshDone, this, &NavPointTab::applyRefresh, Qt::QueuedConnection);
    connect(this, &NavPointTab::operationFinished, this, &NavPointTab::applyOperationResult, Qt::QueuedConnection);
    connect(this, &NavPointTab::poseFilled, this, &NavPointTab::applyPoseFilled, Qt::QueuedConnection);

    // 工具拖拽完成后预填草稿（跨线程安全）
    connect(NavPointDraftBridge::instance(), &NavPointDraftBridge::draftReady, this, &NavPointTab::prefillDraft,
            Qt::QueuedConnection);
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
    get_all_client_ = node_->create_client<nav_interfaces::srv::GetAllNavPoints>(prefix_ + "/get_all_nav_points");
    add_client_ = node_->create_client<nav_interfaces::srv::AddNavPoint>(prefix_ + "/add_point");
    remove_client_ = node_->create_client<nav_interfaces::srv::RemoveNavPoint>(prefix_ + "/remove_point");
    sample_client_ = node_->create_client<nav_interfaces::srv::SampleNavPoint>(prefix_ + "/sample_point");
    nav_client_ = node_->create_client<nav_interfaces::srv::NavToPoint>(prefix_ + "/nav_to_point");
}

// ──────────────────────────────────────────────────────────────────────
// 槽函数
// ──────────────────────────────────────────────────────────────────────

void NavPointTab::onRefresh() {
    if (!get_all_client_) {
        status_label_->setText("节点未初始化");
        return;
    }
    status_label_->setText("刷新中...");
    refresh_btn_->setEnabled(false);

    auto req = std::make_shared<nav_interfaces::srv::GetAllNavPoints::Request>();
    get_all_client_->async_send_request(req,
                                        [this](rclcpp::Client<nav_interfaces::srv::GetAllNavPoints>::SharedFuture fut) {
                                            try {
                                                auto resp = fut.get();
                                                QStringList items;
                                                for (const auto& pt : resp->points) {
                                                    items << QString("[%1] %2  (%3, %4, %5 rad)")
                                                                 .arg(QString::fromStdString(pt.type))
                                                                 .arg(QString::fromStdString(pt.name))
                                                                 .arg(static_cast<double>(pt.x), 0, 'f', 3)
                                                                 .arg(static_cast<double>(pt.y), 0, 'f', 3)
                                                                 .arg(static_cast<double>(pt.theta), 0, 'f', 3);
                                                }
                                                Q_EMIT refreshDone(items);
                                            } catch (const std::exception& ex) {
                                                Q_EMIT operationFinished(false, QString("刷新失败: %1").arg(ex.what()));
                                            }
                                        });
}

void NavPointTab::onSample() {
    if (!sample_client_) {
        status_label_->setText("节点未初始化");
        return;
    }

    if (!sample_client_->service_is_ready()) {
        status_label_->setText("等待采样服务...");
    }

    // 空 name = 仅采样坐标，不保存；坐标填入 UI 后由用户填写名称再点「添加点位」
    auto req = std::make_shared<nav_interfaces::srv::SampleNavPoint::Request>();
    req->name = "";
    req->point_type = 0;

    sample_btn_->setEnabled(false);
    status_label_->setText("采样中（11次，约550ms）...");

    sample_client_->async_send_request(req,
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
    if (!add_client_) {
        status_label_->setText("节点未初始化");
        return;
    }
    const std::string name = name_edit_->text().trimmed().toStdString();
    if (name.empty()) {
        status_label_->setText("错误：请填写点位名称");
        return;
    }

    auto req = std::make_shared<nav_interfaces::srv::AddNavPoint::Request>();
    req->point.name = name;
    req->point.type = type_combo_->currentText().toStdString();
    req->point.x = static_cast<float>(x_spin_->value());
    req->point.y = static_cast<float>(y_spin_->value());
    req->point.theta = static_cast<float>(theta_spin_->value());

    add_btn_->setEnabled(false);
    status_label_->setText("添加中...");

    add_client_->async_send_request(req, [this](rclcpp::Client<nav_interfaces::srv::AddNavPoint>::SharedFuture fut) {
        try {
            auto resp = fut.get();
            Q_EMIT operationFinished(
                resp->success,
                resp->success ? "添加成功" : QString("添加失败: %1").arg(QString::fromStdString(resp->message)));
        } catch (const std::exception& ex) {
            Q_EMIT operationFinished(false, QString("添加异常: %1").arg(ex.what()));
        }
    });
}

void NavPointTab::onRemove() {
    if (!remove_client_) {
        status_label_->setText("节点未初始化");
        return;
    }
    auto* sel = list_->currentItem();
    if (!sel) {
        status_label_->setText("请先选择要删除的点位");
        return;
    }

    // 列表项格式: "[TYPE] name  (x, y, θ)"，提取 name
    QString text = sel->text();
    // 找第一个 "]" 之后、"(" 之前的 name
    int bracket = text.indexOf(']');
    int paren = text.indexOf('(');
    QString name =
        (bracket >= 0 && paren > bracket) ? text.mid(bracket + 2, paren - bracket - 3).trimmed() : text.trimmed();

    if (QMessageBox::question(this, "确认删除", QString("确定删除点位 \"%1\"？").arg(name),
                              QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes) {
        return;
    }

    auto req = std::make_shared<nav_interfaces::srv::RemoveNavPoint::Request>();
    req->name = name.toStdString();

    remove_btn_->setEnabled(false);
    status_label_->setText("删除中...");

    remove_client_->async_send_request(
        req, [this](rclcpp::Client<nav_interfaces::srv::RemoveNavPoint>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                Q_EMIT operationFinished(
                    resp->success,
                    resp->success ? "删除成功" : QString("删除失败: %1").arg(QString::fromStdString(resp->message)));
            } catch (const std::exception& ex) {
                Q_EMIT operationFinished(false, QString("删除异常: %1").arg(ex.what()));
            }
        });
}

void NavPointTab::onNavTo() {
    if (!nav_client_) {
        status_label_->setText("节点未初始化");
        return;
    }
    auto* sel = list_->currentItem();
    if (!sel) {
        status_label_->setText("请先选择目标点位");
        return;
    }

    QString text = sel->text();
    int bracket = text.indexOf(']');
    int paren = text.indexOf('(');
    QString name =
        (bracket >= 0 && paren > bracket) ? text.mid(bracket + 2, paren - bracket - 3).trimmed() : text.trimmed();

    auto req = std::make_shared<nav_interfaces::srv::NavToPoint::Request>();
    req->nav_point.name = name.toStdString();

    nav_btn_->setEnabled(false);
    status_label_->setText(QString("导航到 %1...").arg(name));

    nav_client_->async_send_request(req, [this](rclcpp::Client<nav_interfaces::srv::NavToPoint>::SharedFuture fut) {
        try {
            auto resp = fut.get();
            Q_EMIT operationFinished(resp->success, resp->success ? "导航指令已发送" : "导航指令发送失败");
        } catch (const std::exception& ex) {
            Q_EMIT operationFinished(false, QString("导航异常: %1").arg(ex.what()));
        }
    });
}

void NavPointTab::applyRefresh(QStringList items) {
    list_->clear();
    list_->addItems(items);
    refresh_btn_->setEnabled(true);
    status_label_->setText(QString("共 %1 个点位").arg(items.size()));
}

void NavPointTab::applyOperationResult(bool success, QString msg) {
    status_label_->setText(msg);
    refresh_btn_->setEnabled(true);
    sample_btn_->setEnabled(true);
    add_btn_->setEnabled(true);
    remove_btn_->setEnabled(true);
    nav_btn_->setEnabled(true);
    if (success) onRefresh();
}

void NavPointTab::applyPoseFilled(double x, double y, double yaw_rad) {
    x_spin_->setValue(x);
    y_spin_->setValue(y);
    theta_spin_->setValue(yaw_rad);
    sample_btn_->setEnabled(true);
    status_label_->setText(QString("已采样：(%.3f, %.3f) θ=%.3f rad — 填写名称后点「添加点位」")
                               .arg(x, 0, 'f', 3)
                               .arg(y, 0, 'f', 3)
                               .arg(yaw_rad, 0, 'f', 3));
}

void NavPointTab::prefillDraft(double x, double y, double yaw_rad) {
    x_spin_->setValue(x);
    y_spin_->setValue(y);
    theta_spin_->setValue(yaw_rad);
    status_label_->setText(QString("已选点：(%.3f, %.3f) θ=%.3f rad — 填写名称后点「添加点位」")
                               .arg(x, 0, 'f', 3)
                               .arg(y, 0, 'f', 3)
                               .arg(yaw_rad, 0, 'f', 3));
}

}  // namespace map_asset_rviz_plugin
