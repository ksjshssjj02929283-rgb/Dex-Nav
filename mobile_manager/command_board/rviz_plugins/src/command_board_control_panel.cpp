#include "command_board_rviz_plugins/command_board_control_panel.hpp"

#include <QGroupBox>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace command_board_rviz_plugins {

CommandBoardControlPanel::CommandBoardControlPanel(QWidget* parent)
    : rviz_common::Panel(parent) {
    setupUi();
}

void CommandBoardControlPanel::setupUi() {
    auto* root = new QVBoxLayout(this);

    // 前缀（默认 /command_board，可改 /nav2_command_board）+ 当前地图
    auto* topic_row = new QHBoxLayout();
    topic_row->addWidget(new QLabel("前缀:"));
    topic_edit_ = new QLineEdit("/command_board");
    topic_row->addWidget(topic_edit_);
    root->addLayout(topic_row);
    connect(topic_edit_, &QLineEdit::editingFinished, this,
            &CommandBoardControlPanel::onTopicChanged);

    map_label_ = new QLabel("当前地图: --");
    root->addWidget(map_label_);

    // 💾 保存导航地图 + 📦 备份地图与资产（同一行）
    {
        auto* roww = new QHBoxLayout();
        auto* save_btn = new QPushButton("💾 保存导航地图");
        auto* backup_btn = new QPushButton("📦 备份地图与资产");
        roww->addWidget(save_btn);
        roww->addWidget(backup_btn);
        root->addLayout(roww);
        connect(save_btn, &QPushButton::clicked, this, &CommandBoardControlPanel::onSaveMap);
        connect(backup_btn, &QPushButton::clicked, this, &CommandBoardControlPanel::onBackup);
    }

    // ↩ 回退地图（底图/资产版本 ts 输入）
    {
        auto* roww = new QHBoxLayout();
        auto* rollback_btn = new QPushButton("↩ 回退地图");
        rollback_mapver_edit_ = new QLineEdit();
        rollback_mapver_edit_->setPlaceholderText("底图ts(空=最新)");
        rollback_assetver_edit_ = new QLineEdit();
        rollback_assetver_edit_->setPlaceholderText("资产ts(空=最新)");
        roww->addWidget(rollback_btn);
        roww->addWidget(rollback_mapver_edit_);
        roww->addWidget(rollback_assetver_edit_);
        root->addLayout(roww);
        connect(rollback_btn, &QPushButton::clicked, this, &CommandBoardControlPanel::onRollback);
    }

    // 🎯 重置原点·当前位姿
    {
        auto* reset_pose_btn = new QPushButton("🎯 重置原点·当前位姿");
        root->addWidget(reset_pose_btn);
        connect(reset_pose_btn, &QPushButton::clicked, this,
                &CommandBoardControlPanel::onResetByPose);
    }

    // 🎯 重置原点·指定 x/y/yaw（作用于当前地图）
    {
        auto* roww = new QHBoxLayout();
        auto* reset_xyyaw_btn = new QPushButton("🎯 重置原点·指定");
        reset_x_edit_ = new QLineEdit();   reset_x_edit_->setPlaceholderText("x");
        reset_y_edit_ = new QLineEdit();   reset_y_edit_->setPlaceholderText("y");
        reset_yaw_edit_ = new QLineEdit(); reset_yaw_edit_->setPlaceholderText("yaw");
        roww->addWidget(reset_xyyaw_btn);
        roww->addWidget(reset_x_edit_);
        roww->addWidget(reset_y_edit_);
        roww->addWidget(reset_yaw_edit_);
        root->addLayout(roww);
        connect(reset_xyyaw_btn, &QPushButton::clicked, this,
                &CommandBoardControlPanel::onResetByXYYaw);
    }

    // 🌐 全局重定位（localization_node）
    {
        auto* global_btn = new QPushButton("🌐 全局重定位");
        root->addWidget(global_btn);
        connect(global_btn, &QPushButton::clicked, this, &CommandBoardControlPanel::onGlobalReloc);
    }

    op_result_label_ = new QLabel("操作结果: --");
    op_result_label_->setWordWrap(true);
    root->addWidget(op_result_label_);
    root->addStretch();
}

void CommandBoardControlPanel::onInitialize() {
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    makeClients();
    subscribeStatus();

    ui_timer_ = new QTimer(this);
    connect(ui_timer_, &QTimer::timeout, this, &CommandBoardControlPanel::refreshUi);
    ui_timer_->start(1000);
}

void CommandBoardControlPanel::makeClients() {
    if (!node_) return;
    const std::string prefix = topic_edit_->text().toStdString();
    save_map_cli_     = node_->create_client<nav_interfaces::srv::SaveMap>(prefix + "/save_map");
    backup_cli_       = node_->create_client<std_srvs::srv::Trigger>(prefix + "/backup_unified");
    rollback_cli_     = node_->create_client<nav_interfaces::srv::RollbackMap>(prefix + "/rollback_map");
    reset_pose_cli_   = node_->create_client<std_srvs::srv::Trigger>(prefix + "/reset_map_origin_by_robot_pose");
    reset_xyyaw_cli_  = node_->create_client<nav_interfaces::srv::ResetMapOriginByXYYaw>(prefix + "/reset_map_origin_by_xyyaw");
    // 全局重定位是 localization_node 的服务（command_board 不暴露）。
    global_reloc_cli_ = node_->create_client<std_srvs::srv::Trigger>("/localization_node/trigger_global_relocalization");
}

void CommandBoardControlPanel::subscribeStatus() {
    if (!node_) return;
    status_sub_ = node_->create_subscription<nav_interfaces::msg::NavSystemStatus>(
        topic_edit_->text().toStdString() + "/nav_system_status", rclcpp::QoS(1).reliable(),
        std::bind(&CommandBoardControlPanel::onStatus, this, std::placeholders::_1));
}

void CommandBoardControlPanel::onTopicChanged() {
    makeClients();
    subscribeStatus();
}

void CommandBoardControlPanel::onStatus(nav_interfaces::msg::NavSystemStatus::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(cache_mutex_);
    current_map_name_ = msg->current_map_name;
}

void CommandBoardControlPanel::setOpPending(const std::string& op) {
    std::lock_guard<std::mutex> lk(cache_mutex_);
    last_op_ = op + " … 调用中";
}

void CommandBoardControlPanel::storeOpResult(const std::string& op, bool ok, const std::string& msg) {
    std::lock_guard<std::mutex> lk(cache_mutex_);
    last_op_ = op + (ok ? "  ✓  " : "  ✗  ") + msg;
    last_op_ok_ = ok;
}

void CommandBoardControlPanel::refreshUi() {
    std::string mapname, op_str;
    bool op_ok = true;
    {
        std::lock_guard<std::mutex> lk(cache_mutex_);
        mapname = current_map_name_;
        op_str = last_op_;
        op_ok = last_op_ok_;
    }
    map_label_->setText(QString("当前地图: %1")
                            .arg(mapname.empty() ? "（未加载）" : QString::fromStdString(mapname)));
    op_result_label_->setText(QString("操作结果: %1").arg(QString::fromStdString(op_str)));
    op_result_label_->setStyleSheet(op_ok ? "color: green;" : "color: red;");
}

void CommandBoardControlPanel::onSaveMap() {
    if (!save_map_cli_ || !save_map_cli_->service_is_ready()) {
        storeOpResult("save_map", false, "服务未就绪（检查前缀/节点）");
        return;
    }
    auto req = std::make_shared<nav_interfaces::srv::SaveMap::Request>();
    req->map_name = "";  // 空 = 当前已加载地图
    setOpPending("save_map");
    save_map_cli_->async_send_request(
        req, [this](rclcpp::Client<nav_interfaces::srv::SaveMap>::SharedFuture fut) {
            auto r = fut.get();
            storeOpResult("save_map", r->success, r->success ? r->map_path : r->message);
        });
}

void CommandBoardControlPanel::onBackup() {
    if (!backup_cli_ || !backup_cli_->service_is_ready()) {
        storeOpResult("backup_unified", false, "服务未就绪");
        return;
    }
    setOpPending("backup_unified");
    backup_cli_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>(),
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture fut) {
            auto r = fut.get();
            storeOpResult("backup_unified", r->success, r->message);
        });
}

void CommandBoardControlPanel::onRollback() {
    if (!rollback_cli_ || !rollback_cli_->service_is_ready()) {
        storeOpResult("rollback_map", false, "服务未就绪");
        return;
    }
    const std::string mapver = rollback_mapver_edit_->text().toStdString();
    const std::string assetver = rollback_assetver_edit_->text().toStdString();
    const QString summary =
        QString("回退地图\n底图版本: %1\n资产版本: %2\n（空=各取最新备份）")
            .arg(mapver.empty() ? "最新" : QString::fromStdString(mapver))
            .arg(assetver.empty() ? "最新" : QString::fromStdString(assetver));
    if (QMessageBox::question(this, "确认回退地图", summary) != QMessageBox::Yes) return;
    auto req = std::make_shared<nav_interfaces::srv::RollbackMap::Request>();
    req->map_version_ts = mapver;
    req->assets_version_ts = assetver;
    setOpPending("rollback_map");
    rollback_cli_->async_send_request(
        req, [this](rclcpp::Client<nav_interfaces::srv::RollbackMap>::SharedFuture fut) {
            auto r = fut.get();
            storeOpResult("rollback_map", r->success, r->message);
        });
}

void CommandBoardControlPanel::onResetByPose() {
    if (!reset_pose_cli_ || !reset_pose_cli_->service_is_ready()) {
        storeOpResult("reset_origin_by_robot_pose", false, "服务未就绪");
        return;
    }
    if (QMessageBox::question(this, "确认重置原点",
                              "用当前机器人位姿重置当前地图原点？\n（停定位→工具→重载→启定位）")
        != QMessageBox::Yes)
        return;
    setOpPending("reset_origin_by_robot_pose");
    reset_pose_cli_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>(),
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture fut) {
            auto r = fut.get();
            storeOpResult("reset_origin_by_robot_pose", r->success, r->message);
        });
}

void CommandBoardControlPanel::onResetByXYYaw() {
    if (!reset_xyyaw_cli_ || !reset_xyyaw_cli_->service_is_ready()) {
        storeOpResult("reset_origin_by_xyyaw", false, "服务未就绪");
        return;
    }
    std::string mapname;
    {
        std::lock_guard<std::mutex> lk(cache_mutex_);
        mapname = current_map_name_;
    }
    if (mapname.empty()) {
        storeOpResult("reset_origin_by_xyyaw", false, "当前地图未知（等 nav_system_status）");
        return;
    }
    bool okx = false, oky = false, okyaw = false;
    const double x = reset_x_edit_->text().toDouble(&okx);
    const double y = reset_y_edit_->text().toDouble(&oky);
    const double yaw = reset_yaw_edit_->text().toDouble(&okyaw);
    if (!okx || !oky || !okyaw) {
        storeOpResult("reset_origin_by_xyyaw", false, "x/y/yaw 需为数字");
        return;
    }
    const QString summary = QString("当前地图 %1 原点设为\nx=%2  y=%3  yaw=%4 ？")
                                .arg(QString::fromStdString(mapname))
                                .arg(x).arg(y).arg(yaw);
    if (QMessageBox::question(this, "确认重置原点", summary) != QMessageBox::Yes) return;
    auto req = std::make_shared<nav_interfaces::srv::ResetMapOriginByXYYaw::Request>();
    req->map_name = mapname;  // 当前地图
    req->x = x;
    req->y = y;
    req->yaw = yaw;
    setOpPending("reset_origin_by_xyyaw");
    reset_xyyaw_cli_->async_send_request(
        req, [this](rclcpp::Client<nav_interfaces::srv::ResetMapOriginByXYYaw>::SharedFuture fut) {
            auto r = fut.get();
            storeOpResult("reset_origin_by_xyyaw", r->success, r->message);
        });
}

void CommandBoardControlPanel::onGlobalReloc() {
    if (!global_reloc_cli_ || !global_reloc_cli_->service_is_ready()) {
        storeOpResult("global_relocalization", false, "服务未就绪（/localization_node）");
        return;
    }
    if (QMessageBox::question(this, "确认全局重定位",
                              "触发全局重定位？\n（全图搜索，机器人当前定位会被重置）")
        != QMessageBox::Yes)
        return;
    setOpPending("global_relocalization");
    global_reloc_cli_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>(),
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture fut) {
            auto r = fut.get();
            storeOpResult("global_relocalization", r->success, r->message);
        });
}

void CommandBoardControlPanel::save(rviz_common::Config config) const {
    rviz_common::Panel::save(config);
    config.mapSetValue("topic_prefix", topic_edit_->text());
}

void CommandBoardControlPanel::load(const rviz_common::Config& config) {
    rviz_common::Panel::load(config);
    QString prefix;
    if (config.mapGetString("topic_prefix", &prefix)) {
        topic_edit_->setText(prefix);
    }
}

}  // namespace command_board_rviz_plugins

PLUGINLIB_EXPORT_CLASS(command_board_rviz_plugins::CommandBoardControlPanel,
                       rviz_common::Panel)
