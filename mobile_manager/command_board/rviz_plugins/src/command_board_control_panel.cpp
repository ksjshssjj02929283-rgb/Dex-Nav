#include "command_board_rviz_plugins/command_board_control_panel.hpp"

#include <QDateTime>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QPushButton>
#include <QTimer>
#include <QPointer>
#include <QVBoxLayout>

#include <iomanip>
#include <map>
#include <sstream>

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

    // 前缀：command_board（默认 /command_board，可改 /nav2_command_board）+ map_manager（取地图列表）
    auto* topic_row = new QHBoxLayout();
    topic_row->addWidget(new QLabel("前缀:"));
    topic_edit_ = new QLineEdit("/command_board");
    topic_row->addWidget(topic_edit_);
    topic_row->addWidget(new QLabel("map_manager:"));
    mm_prefix_edit_ = new QLineEdit("/map_manager");
    topic_row->addWidget(mm_prefix_edit_);
    root->addLayout(topic_row);
    connect(topic_edit_, &QLineEdit::editingFinished, this,
            &CommandBoardControlPanel::onTopicChanged);
    connect(mm_prefix_edit_, &QLineEdit::editingFinished, this,
            &CommandBoardControlPanel::onTopicChanged);

    map_label_ = new QLabel("当前地图: --");
    root->addWidget(map_label_);

    // ── 当前地图：依赖实时定位的操作 ─────────────────────────────────
    auto* cur_box = new QGroupBox("当前地图（依赖实时定位）");
    auto* cur_v = new QVBoxLayout(cur_box);
    {
        auto* roww = new QHBoxLayout();
        auto* save_btn = new QPushButton("💾 保存导航地图");
        auto* backup_btn = new QPushButton("📦 备份地图与资产");
        roww->addWidget(save_btn);
        roww->addWidget(backup_btn);
        cur_v->addLayout(roww);
        connect(save_btn, &QPushButton::clicked, this, &CommandBoardControlPanel::onSaveMap);
        connect(backup_btn, &QPushButton::clicked, this, &CommandBoardControlPanel::onBackup);
    }
    {
        auto* roww = new QHBoxLayout();
        auto* reset_pose_btn = new QPushButton("🎯 重置原点·当前位姿");
        auto* global_btn = new QPushButton("🌐 全局重定位");
        roww->addWidget(reset_pose_btn);
        roww->addWidget(global_btn);
        cur_v->addLayout(roww);
        connect(reset_pose_btn, &QPushButton::clicked, this,
                &CommandBoardControlPanel::onResetByPose);
        connect(global_btn, &QPushButton::clicked, this, &CommandBoardControlPanel::onGlobalReloc);
    }
    root->addWidget(cur_box);

    // ── 目标地图：任意一张地图都能做的操作 ───────────────────────────
    auto* tgt_box = new QGroupBox("目标地图（任意地图；是当前地图时会停启定位并重新加载）");
    auto* tgt_v = new QVBoxLayout(tgt_box);
    {
        auto* roww = new QHBoxLayout();
        target_combo_ = new QComboBox();
        auto* refresh_btn = new QPushButton("🔄 刷新地图列表");
        roww->addWidget(new QLabel("目标:"));
        roww->addWidget(target_combo_, 1);
        roww->addWidget(refresh_btn);
        tgt_v->addLayout(roww);
        connect(refresh_btn, &QPushButton::clicked, this, &CommandBoardControlPanel::onRefreshMaps);
        connect(target_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
                [this](int) { onTargetChanged(); });
    }
    // ↩ 回退：底图版本与资产版本都从目标地图的版本列表里选
    {
        auto* roww = new QHBoxLayout();
        auto* rollback_btn = new QPushButton("↩ 回退");
        mapver_combo_ = new QComboBox();
        mapver_combo_->setToolTip("底图版本（该图 slam/backups 下的版本，新的在前）");
        assetver_combo_ = new QComboBox();
        assetver_combo_->setToolTip("资产版本（该图 assets/backups 下的版本，新的在前；由 map_asset_manager 回滚）");
        roww->addWidget(rollback_btn);
        roww->addWidget(mapver_combo_, 1);
        roww->addWidget(assetver_combo_, 1);
        tgt_v->addLayout(roww);
        connect(rollback_btn, &QPushButton::clicked, this, &CommandBoardControlPanel::onRollback);
    }
    // 🎯 重置原点·指定 x/y/yaw
    {
        auto* roww = new QHBoxLayout();
        auto* reset_xyyaw_btn = new QPushButton("🎯 重置原点·指定");
        reset_x_edit_ = new QLineEdit();   reset_x_edit_->setPlaceholderText("x");
        reset_y_edit_ = new QLineEdit();   reset_y_edit_->setPlaceholderText("y");
        reset_yaw_edit_ = new QLineEdit(); reset_yaw_edit_->setPlaceholderText("yaw(rad)");
        roww->addWidget(reset_xyyaw_btn);
        roww->addWidget(reset_x_edit_);
        roww->addWidget(reset_y_edit_);
        roww->addWidget(reset_yaw_edit_);
        tgt_v->addLayout(roww);
        connect(reset_xyyaw_btn, &QPushButton::clicked, this,
                &CommandBoardControlPanel::onResetByXYYaw);
    }
    // 📐 对齐：先试算估角，再执行（底图与资产同角度旋转，失败联合回滚）
    {
        auto* roww = new QHBoxLayout();
        auto* dry_btn = new QPushButton("📐 对齐·试算");
        auto* apply_btn = new QPushButton("📐 执行对齐");
        roww->addWidget(dry_btn);
        roww->addWidget(apply_btn);
        tgt_v->addLayout(roww);
        connect(dry_btn, &QPushButton::clicked, this, &CommandBoardControlPanel::onAlignDryRun);
        connect(apply_btn, &QPushButton::clicked, this, &CommandBoardControlPanel::onAlignApply);
    }
    // 🔁 替换：用源图内容替换目标底图（坐标系沿用目标，资产不动）
    {
        auto* roww = new QHBoxLayout();
        source_combo_ = new QComboBox();
        source_combo_->setToolTip("提供新内容的地图（整场重走新建的图），不能与目标相同");
        keep_source_check_ = new QCheckBox("保留源图");
        accept_degraded_check_ = new QCheckBox("未收敛也执行");
        roww->addWidget(new QLabel("源图:"));
        roww->addWidget(source_combo_, 1);
        roww->addWidget(keep_source_check_);
        roww->addWidget(accept_degraded_check_);
        tgt_v->addLayout(roww);
        auto* row2 = new QHBoxLayout();
        manual_check_ = new QCheckBox("手动变换");
        manual_check_->setToolTip("不做自动匹配，直接用给定的 源图原点在目标坐标系中的 x / y / yaw");
        manual_x_edit_ = new QLineEdit();   manual_x_edit_->setPlaceholderText("x");
        manual_y_edit_ = new QLineEdit();   manual_y_edit_->setPlaceholderText("y");
        manual_yaw_edit_ = new QLineEdit(); manual_yaw_edit_->setPlaceholderText("yaw(rad)");
        row2->addWidget(manual_check_);
        row2->addWidget(manual_x_edit_);
        row2->addWidget(manual_y_edit_);
        row2->addWidget(manual_yaw_edit_);
        tgt_v->addLayout(row2);
        auto* row3 = new QHBoxLayout();
        auto* dry_btn = new QPushButton("🔁 替换·试算");
        auto* apply_btn = new QPushButton("🔁 执行替换");
        row3->addWidget(dry_btn);
        row3->addWidget(apply_btn);
        tgt_v->addLayout(row3);
        connect(dry_btn, &QPushButton::clicked, this, &CommandBoardControlPanel::onReplaceDryRun);
        connect(apply_btn, &QPushButton::clicked, this, &CommandBoardControlPanel::onReplaceApply);
    }
    root->addWidget(tgt_box);

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
    // map_manager 服务发现需要一点时间，稍后自动拉一次地图列表
    QTimer::singleShot(1500, this, &CommandBoardControlPanel::onRefreshMaps);
}

void CommandBoardControlPanel::makeClients() {
    if (!node_) return;
    const std::string prefix = topic_edit_->text().toStdString();
    save_map_cli_     = node_->create_client<nav_interfaces::srv::SaveMap>(prefix + "/save_map");
    backup_cli_       = node_->create_client<std_srvs::srv::Trigger>(prefix + "/backup_unified");
    rollback_cli_     = node_->create_client<nav_interfaces::srv::RollbackMap>(prefix + "/rollback_map");
    reset_pose_cli_   = node_->create_client<std_srvs::srv::Trigger>(prefix + "/reset_map_origin_by_robot_pose");
    reset_xyyaw_cli_  = node_->create_client<nav_interfaces::srv::ResetMapOriginByXYYaw>(prefix + "/reset_map_origin_by_xyyaw");
    // 全局重定位是 localization 的服务（command_board 不暴露）。
    global_reloc_cli_ = node_->create_client<std_srvs::srv::Trigger>("/localization/trigger_global_relocalization");
    align_cli_        = node_->create_client<nav_interfaces::srv::AlignMap>(prefix + "/align_map");
    replace_cli_      = node_->create_client<nav_interfaces::srv::ReplaceMap>(prefix + "/replace_map");
    get_maps_cli_     = node_->create_client<nav_interfaces::srv::GetMaps>(mm_prefix_edit_->text().toStdString() + "/get_maps");
    get_assets_cli_   = node_->create_client<nav_interfaces::srv::GetAssets>("/map_asset_manager/get_assets");
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
    const std::string target = targetMap();
    if (target.empty()) { storeOpResult("rollback_map", false, "先刷新地图列表并选择目标地图"); return; }
    const std::string mapver = mapver_combo_->currentData().toString().toStdString();
    const std::string assetver = assetver_combo_->currentData().toString().toStdString();
    std::string current;
    {
        std::lock_guard<std::mutex> lk(cache_mutex_);
        current = current_map_name_;
    }
    auto label = [](const std::string& v) {
        return v.empty() ? QString("最新") : (v == "none" ? QString("不回退") : QString::fromStdString(v));
    };
    const QString summary =
        QString("回退地图「%1」\n底图版本: %2\n资产版本: %3\n%4")
            .arg(QString::fromStdString(target)).arg(label(mapver)).arg(label(assetver))
            .arg(target == current ? "这是当前地图：回退后重新下发，需要重新定位。" : "这不是当前地图：只恢复文件。");
    if (QMessageBox::question(this, "确认回退地图", summary) != QMessageBox::Yes) return;
    auto req = std::make_shared<nav_interfaces::srv::RollbackMap::Request>();
    req->map_name = target;
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
    const std::string mapname = targetMap();
    if (mapname.empty()) {
        storeOpResult("reset_origin_by_xyyaw", false, "先刷新地图列表并选择目标地图");
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
    const QString summary = QString("地图 %1 原点设为\nx=%2  y=%3  yaw=%4 ？\n改动前自动存底图版本（重设原点前）。")
                                .arg(QString::fromStdString(mapname))
                                .arg(x).arg(y).arg(yaw);
    if (QMessageBox::question(this, "确认重置原点", summary) != QMessageBox::Yes) return;
    auto req = std::make_shared<nav_interfaces::srv::ResetMapOriginByXYYaw::Request>();
    req->map_name = mapname;  // 目标地图（任意一张）
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
        storeOpResult("global_relocalization", false, "服务未就绪（/localization）");
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

// ── 目标地图 ──────────────────────────────────────────────────────────

std::string CommandBoardControlPanel::targetMap() const {
    return target_combo_->currentData().toString().toStdString();
}

void CommandBoardControlPanel::onRefreshMaps() {
    if (!get_maps_cli_ || !get_maps_cli_->service_is_ready()) {
        storeOpResult("get_maps", false, "服务未就绪（检查 map_manager 前缀）");
        return;
    }
    QPointer<CommandBoardControlPanel> self(this);
    get_maps_cli_->async_send_request(
        std::make_shared<nav_interfaces::srv::GetMaps::Request>(),
        [self](rclcpp::Client<nav_interfaces::srv::GetMaps>::SharedFuture fut) {
            auto r = fut.get();
            if (!self) return;
            // 下拉框只能在 GUI 线程改
            QMetaObject::invokeMethod(self.data(), [self, r]() {
                if (!self) return;
                if (!r->success) {
                    self->storeOpResult("get_maps", false, r->message);
                    return;
                }
                self->applyMaps(r->maps, r->current_map_name);
            }, Qt::QueuedConnection);
        });
}

void CommandBoardControlPanel::applyMaps(const std::vector<nav_interfaces::msg::MapInfo>& maps,
                                         const std::string& current) {
    const QString keep = target_combo_->currentData().toString();
    maps_ = maps;
    target_combo_->blockSignals(true);
    target_combo_->clear();
    int keep_idx = -1, current_idx = -1;
    for (const auto& m : maps_) {
        const QString name = QString::fromStdString(m.name);
        target_combo_->addItem(m.name == current ? name + "（当前）" : name, name);
        if (name == keep) keep_idx = target_combo_->count() - 1;
        if (m.name == current) current_idx = target_combo_->count() - 1;
    }
    target_combo_->setCurrentIndex(keep_idx >= 0 ? keep_idx : current_idx);
    target_combo_->blockSignals(false);
    onTargetChanged();
    storeOpResult("get_maps", true, std::to_string(maps_.size()) + " 张地图");
}

void CommandBoardControlPanel::onTargetChanged() {
    static const std::map<std::string, QString> kSource = {
        {"localization_save", "定位保存"}, {"grid_edit", "底图编辑前"}, {"rollback", "回滚前"},
        {"align", "对齐前"}, {"replace", "替换前"}, {"reset_origin", "重设原点前"}};
    const std::string target = targetMap();
    mapver_combo_->clear();
    mapver_combo_->addItem("底图：最新版本", QString());
    mapver_combo_->addItem("底图：不回退", QString("none"));
    const QString keep_src = source_combo_->currentText();
    source_combo_->clear();
    for (const auto& m : maps_) {
        if (m.name != target) {
            source_combo_->addItem(QString::fromStdString(m.name));
            continue;
        }
        for (const auto& v : m.versions) {
            const auto it = kSource.find(v.source);
            const QString when = v.saved_at.sec > 0
                ? QDateTime::fromSecsSinceEpoch(v.saved_at.sec).toString("MM-dd HH:mm:ss") : QString("-");
            mapver_combo_->addItem(QString("%1  %2  %3%4")
                                       .arg(QString::fromStdString(v.version_ts)).arg(when)
                                       .arg(it != kSource.end() ? it->second : QString("未标记"))
                                       .arg(v.complete ? "" : "（不完整）"),
                                   QString::fromStdString(v.version_ts));
        }
    }
    const int idx = source_combo_->findText(keep_src);
    if (idx >= 0) source_combo_->setCurrentIndex(idx);
    refreshAssetVersions(target);
}

void CommandBoardControlPanel::refreshAssetVersions(const std::string& target) {
    assetver_combo_->clear();
    assetver_combo_->addItem("资产：最新版本", QString());
    assetver_combo_->addItem("资产：不回退", QString("none"));
    if (target.empty() || !get_assets_cli_ || !get_assets_cli_->service_is_ready()) return;
    auto req = std::make_shared<nav_interfaces::srv::GetAssets::Request>();
    req->map_name = target;
    req->kinds = nav_interfaces::srv::GetAssets::Request::KIND_POINTS;  // 只为取版本列表，数据量取最小的一类
    QPointer<CommandBoardControlPanel> self(this);
    get_assets_cli_->async_send_request(req, [self, target](rclcpp::Client<nav_interfaces::srv::GetAssets>::SharedFuture fut) {
        auto r = fut.get();
        if (!self) return;
        QMetaObject::invokeMethod(self.data(), [self, r, target]() {
            if (!self || self->targetMap() != target || !r->success) return;
            static const std::map<std::string, QString> kAssetSource = {
                {"save", "显式保存"}, {"transform", "坐标变换前"}, {"rollback", "回滚前"}};
            for (const auto& v : r->versions) {
                const auto it = kAssetSource.find(v.source);
                const QString when = v.saved_at.sec > 0
                    ? QDateTime::fromSecsSinceEpoch(v.saved_at.sec).toString("MM-dd HH:mm:ss") : QString("-");
                self->assetver_combo_->addItem(QString("%1  %2  %3")
                                                   .arg(QString::fromStdString(v.version_ts)).arg(when)
                                                   .arg(it != kAssetSource.end() ? it->second : QString("未标记")),
                                               QString::fromStdString(v.version_ts));
            }
        }, Qt::QueuedConnection);
    });
}

void CommandBoardControlPanel::onAlignDryRun() { callAlign(true); }

void CommandBoardControlPanel::onAlignApply() {
    const std::string target = targetMap();
    if (target.empty()) { storeOpResult("align_map", false, "先刷新地图列表并选择目标地图"); return; }
    std::string current;
    {
        std::lock_guard<std::mutex> lk(cache_mutex_);
        current = current_map_name_;
    }
    const QString summary =
        QString("对齐「%1」：按墙面方向估角，底图与资产同角度旋转，失败联合回滚。\n改动前自动存底图版本（对齐前）。\n%2")
            .arg(QString::fromStdString(target))
            .arg(target == current ? "这是当前地图：会停导航与定位，完成后重新加载并重定位。" : "这不是当前地图：只改文件。");
    if (QMessageBox::question(this, "确认对齐", summary) != QMessageBox::Yes) return;
    callAlign(false);
}

void CommandBoardControlPanel::callAlign(bool dry_run) {
    const std::string target = targetMap();
    if (target.empty()) { storeOpResult("align_map", false, "先刷新地图列表并选择目标地图"); return; }
    if (!align_cli_ || !align_cli_->service_is_ready()) { storeOpResult("align_map", false, "服务未就绪"); return; }
    auto req = std::make_shared<nav_interfaces::srv::AlignMap::Request>();
    req->map_name = target;
    req->dry_run = dry_run;
    const std::string op = dry_run ? "align_map(试算)" : "align_map";
    setOpPending(op);
    align_cli_->async_send_request(
        req, [this, op](rclcpp::Client<nav_interfaces::srv::AlignMap>::SharedFuture fut) {
            auto r = fut.get();
            std::ostringstream s;
            s << std::fixed << std::setprecision(2) << "估计对齐角 " << r->estimated_yaw_deg
              << "°，正交误差 " << r->orthogonality_error_deg << "°";
            if (r->applied) s << "，执行后残余 " << r->residual_yaw_deg << "°，资产点/区/节点 "
                              << r->transformed_points << "/" << r->transformed_areas << "/" << r->transformed_junctions;
            s << "；" << r->message;
            storeOpResult(op, r->success, s.str());
        });
}

void CommandBoardControlPanel::onReplaceDryRun() { callReplace(true); }

void CommandBoardControlPanel::onReplaceApply() {
    const std::string target = targetMap();
    const std::string source = source_combo_->currentText().toStdString();
    if (target.empty() || source.empty()) { storeOpResult("replace_map", false, "选择目标地图与源图"); return; }
    std::string current;
    {
        std::lock_guard<std::mutex> lk(cache_mutex_);
        current = current_map_name_;
    }
    const QString summary =
        QString("用「%1」替换「%2」的底图（坐标系沿用目标，资产不动）。\n改动前自动存底图版本（替换前）。\n%3%4")
            .arg(QString::fromStdString(source)).arg(QString::fromStdString(target))
            .arg(keep_source_check_->isChecked() ? "" : "完成后源图移入回收站。\n")
            .arg(target == current ? "目标是当前地图：会停导航与定位，完成后重新加载并重定位。" : "");
    if (QMessageBox::question(this, "确认替换", summary) != QMessageBox::Yes) return;
    callReplace(false);
}

void CommandBoardControlPanel::callReplace(bool dry_run) {
    const std::string target = targetMap();
    const std::string source = source_combo_->currentText().toStdString();
    if (target.empty() || source.empty()) { storeOpResult("replace_map", false, "选择目标地图与源图"); return; }
    if (!replace_cli_ || !replace_cli_->service_is_ready()) { storeOpResult("replace_map", false, "服务未就绪"); return; }
    auto req = std::make_shared<nav_interfaces::srv::ReplaceMap::Request>();
    req->target_map_name = target;
    req->source_map_name = source;
    req->dry_run = dry_run;
    req->keep_source = keep_source_check_->isChecked();
    req->accept_degraded = accept_degraded_check_->isChecked();
    if (manual_check_->isChecked()) {
        bool okx = false, oky = false, okyaw = false;
        req->use_manual_transform = true;
        req->manual_x = manual_x_edit_->text().toDouble(&okx);
        req->manual_y = manual_y_edit_->text().toDouble(&oky);
        req->manual_yaw = manual_yaw_edit_->text().toDouble(&okyaw);
        if (!okx || !oky || !okyaw) { storeOpResult("replace_map", false, "手动变换 x/y/yaw 需为数字"); return; }
    }
    const std::string op = dry_run ? "replace_map(试算)" : "replace_map";
    setOpPending(op);
    replace_cli_->async_send_request(
        req, [this, op](rclcpp::Client<nav_interfaces::srv::ReplaceMap>::SharedFuture fut) {
            auto r = fut.get();
            std::ostringstream s;
            s << std::fixed << std::setprecision(3) << "变换 x=" << r->x << " y=" << r->y
              << " yaw=" << std::setprecision(2) << r->yaw_deg << "°，内点 " << r->inliers << "，"
              << (r->converged ? "已收敛" : (r->degraded ? "未收敛（质量存疑）" : "未收敛"));
            if (r->coverage_warning) s << "，源图覆盖不足目标的 80%";
            if (!r->source_trashed_to.empty()) s << "，源图已移入回收站";
            s << "；" << r->message;
            storeOpResult(op, r->success, s.str());
        });
}

void CommandBoardControlPanel::save(rviz_common::Config config) const {
    rviz_common::Panel::save(config);
    config.mapSetValue("topic_prefix", topic_edit_->text());
    config.mapSetValue("map_manager_prefix", mm_prefix_edit_->text());
}

void CommandBoardControlPanel::load(const rviz_common::Config& config) {
    rviz_common::Panel::load(config);
    QString prefix;
    if (config.mapGetString("topic_prefix", &prefix)) {
        topic_edit_->setText(prefix);
    }
    if (config.mapGetString("map_manager_prefix", &prefix)) {
        mm_prefix_edit_->setText(prefix);
    }
}

}  // namespace command_board_rviz_plugins

PLUGINLIB_EXPORT_CLASS(command_board_rviz_plugins::CommandBoardControlPanel,
                       rviz_common::Panel)
