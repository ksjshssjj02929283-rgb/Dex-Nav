#include "command_board_rviz_plugins/command_board_status_panel.hpp"

#include <cmath>

#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QTimer>
#include <QVBoxLayout>

#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_interfaces/msg/localization_state.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace command_board_rviz_plugins {

namespace {

struct StatusInfo {
    QString text;
    QString color;
};

// LocalizationState.status → 文字 + 颜色（绿=成功 / 橙=过渡 / 灰=未初始化 / 红=失败）
StatusInfo locStatusInfo(int status) {
    using LS = nav_interfaces::msg::LocalizationState;
    switch (status) {
        case LS::LOC_SUCCESS:      return {"LOC_SUCCESS", "green"};
        case LS::LOC_RELOCALIZING: return {"RELOCALIZING", "orange"};
        case LS::LOC_INITIALIZING: return {"INITIALIZING", "orange"};
        case LS::LOC_INITIALIZED:  return {"INITIALIZED", "orange"};
        case LS::LOC_NOT_INIT:     return {"NOT_INIT", "gray"};
        case LS::LOC_INIT_FAILED:  return {"INIT_FAILED", "red"};
        case LS::LOC_FAILED:       return {"LOC_FAILED", "red"};
        default:                   return {QString("UNKNOWN(%1)").arg(status), "gray"};
    }
}

double quatToYaw(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

CommandBoardStatusPanel::CommandBoardStatusPanel(QWidget* parent)
    : rviz_common::Panel(parent) {
    setupUi();
}

void CommandBoardStatusPanel::setupUi() {
    auto* root = new QVBoxLayout(this);

    // 话题前缀（默认 /command_board，可改为 /nav2_command_board）
    auto* topic_row = new QHBoxLayout();
    topic_row->addWidget(new QLabel("话题前缀:"));
    topic_edit_ = new QLineEdit("/command_board");
    topic_row->addWidget(topic_edit_);
    root->addLayout(topic_row);
    connect(topic_edit_, &QLineEdit::editingFinished, this,
            &CommandBoardStatusPanel::onTopicChanged);

    // 🗺 当前地图
    {
        auto* box = new QGroupBox("🗺 当前地图");
        auto* g = new QHBoxLayout(box);
        map_label_ = new QLabel("--");
        g->addWidget(map_label_);
        root->addWidget(box);
    }

    // 🔋 电池
    {
        auto* box = new QGroupBox("🔋 电池");
        auto* g = new QGridLayout(box);
        bat_bar_ = new QProgressBar();
        bat_bar_->setRange(0, 100);
        bat_text_ = new QLabel("--");
        bat_vi_ = new QLabel("-- V   -- A");
        g->addWidget(bat_bar_, 0, 0);
        g->addWidget(bat_text_, 0, 1);
        g->addWidget(bat_vi_, 1, 0, 1, 2);
        root->addWidget(box);
    }

    // 🛡 安全
    {
        auto* box = new QGroupBox("🛡 安全");
        auto* g = new QGridLayout(box);
        estop_led_ = new QLabel("●");
        bumper_f_led_ = new QLabel("●");
        bumper_r_led_ = new QLabel("●");
        g->addWidget(new QLabel("急停"), 0, 0);
        g->addWidget(estop_led_, 0, 1);
        g->addWidget(new QLabel("前撞"), 0, 2);
        g->addWidget(bumper_f_led_, 0, 3);
        g->addWidget(new QLabel("后撞"), 0, 4);
        g->addWidget(bumper_r_led_, 0, 5);
        root->addWidget(box);
    }

    // 📍 定位
    {
        auto* box = new QGroupBox("📍 定位");
        auto* g = new QGridLayout(box);
        loc_status_ = new QLabel("--");
        loc_score_ = new QLabel("--");
        g->addWidget(new QLabel("状态"), 0, 0);
        g->addWidget(loc_status_, 0, 1);
        g->addWidget(new QLabel("重合率"), 1, 0);
        g->addWidget(loc_score_, 1, 1);
        root->addWidget(box);
    }

    // 📋 任务
    {
        auto* box = new QGroupBox("📋 任务");
        auto* g = new QHBoxLayout(box);
        task_ = new QLabel("--");
        g->addWidget(task_);
        root->addWidget(box);
    }

    // 🧭 位姿 / 速度
    {
        auto* box = new QGroupBox("🧭 位姿 / 速度");
        auto* g = new QVBoxLayout(box);
        pose_ = new QLabel("x --   y --   yaw --");
        vel_ = new QLabel("v --   ω --");
        g->addWidget(pose_);
        g->addWidget(vel_);
        root->addWidget(box);
    }

    // 💾 自动保存
    {
        auto* box = new QGroupBox("💾 自动保存");
        auto* g = new QHBoxLayout(box);
        saver_ = new QLabel("--");
        saver_->setWordWrap(true);
        g->addWidget(saver_);
        root->addWidget(box);
    }

    // 🚨 急停丢位恢复（紧凑：margins/spacing 减小，alarm 默认隐藏）
    {
        auto* box = new QGroupBox("🚨 急停丢位恢复");
        auto* g = new QHBoxLayout(box);
        g->setContentsMargins(8, 2, 8, 2);
        g->setSpacing(6);
        recover_ = new QLabel("NORMAL");
        alarm_ = new QLabel("");
        alarm_->setWordWrap(true);
        alarm_->setVisible(false);
        g->addWidget(recover_);
        g->addWidget(alarm_, 1);
        root->addWidget(box);
    }

    // ⏱ 数据新鲜度
    fresh_ = new QLabel("⏱ 等待数据…");
    root->addWidget(fresh_);
    root->addStretch();
}

void CommandBoardStatusPanel::onInitialize() {
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    last_recv_ = node_->now();
    subscribe();

    ui_timer_ = new QTimer(this);
    connect(ui_timer_, &QTimer::timeout, this, &CommandBoardStatusPanel::refreshUi);
    ui_timer_->start(1000);  // 1Hz
}

void CommandBoardStatusPanel::subscribe() {
    if (!node_) return;
    const std::string prefix = topic_edit_->text().toStdString();
    const std::string status_topic = prefix + "/nav_system_status";
    const std::string saver_topic = prefix + "/auto_map_saver_status";

    // NavSystemStatus：StatusMonitor 用 KeepLast(1).reliable() 发布。
    status_sub_ = node_->create_subscription<nav_interfaces::msg::NavSystemStatus>(
        status_topic, rclcpp::QoS(1).reliable(),
        std::bind(&CommandBoardStatusPanel::onStatus, this, std::placeholders::_1));

    // auto_map_saver_status：latched，订阅 transient_local 才能收到最近一帧。
    saver_sub_ = node_->create_subscription<std_msgs::msg::String>(
        saver_topic, rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&CommandBoardStatusPanel::onSaverStatus, this, std::placeholders::_1));
}

void CommandBoardStatusPanel::onTopicChanged() {
    subscribe();  // 赋值新订阅会析构旧订阅
}

void CommandBoardStatusPanel::onStatus(nav_interfaces::msg::NavSystemStatus::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(cache_mutex_);
    last_status_ = *msg;
    has_status_ = true;
    last_recv_ = node_->now();
}

void CommandBoardStatusPanel::onSaverStatus(std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(cache_mutex_);
    last_saver_ = msg->data;
    has_saver_ = true;
}

void CommandBoardStatusPanel::setLed(QLabel* led, const QString& color) {
    led->setText("●");
    led->setStyleSheet(QString("color: %1; font-size: 16px;").arg(color));
}

void CommandBoardStatusPanel::refreshUi() {
    nav_interfaces::msg::NavSystemStatus s;
    std::string saver;
    bool has_status = false, has_saver = false;
    double age = 999.0;
    {
        std::lock_guard<std::mutex> lk(cache_mutex_);
        s = last_status_;
        saver = last_saver_;
        has_status = has_status_;
        has_saver = has_saver_;
        if (node_) age = (node_->now() - last_recv_).seconds();
    }

    const bool online = has_status && age < 5.0;

    // 数据新鲜度
    if (!has_status) {
        fresh_->setText("⏱ 等待数据…   ●离线");
        fresh_->setStyleSheet("color: gray;");
        return;
    }
    fresh_->setText(QString("⏱ 上次更新 %1s 前   %2")
                        .arg(age, 0, 'f', 1)
                        .arg(online ? "●在线" : "●离线"));
    fresh_->setStyleSheet(online ? "color: green;" : "color: red;");

    // 🔋 电池
    bat_bar_->setValue(s.battery_percentage);
    bat_text_->setText(QString("%1 %").arg(s.battery_percentage) +
                       (s.battery_charging ? "   ⚡ 充电" : ""));
    bat_vi_->setText(QString("%1 V   %2 A")
                         .arg(s.battery_voltage, 0, 'f', 1)
                         .arg(s.battery_current, 0, 'f', 1));
    const QString bat_color =
        s.battery_percentage > 30 ? "green" : (s.battery_percentage >= 10 ? "orange" : "red");
    bat_bar_->setStyleSheet(
        QString("QProgressBar::chunk { background-color: %1; }").arg(bat_color));

    // 🛡 安全
    setLed(estop_led_, s.estop_active ? "red" : "green");
    setLed(bumper_f_led_, s.bumper_front_active ? "red" : "green");
    setLed(bumper_r_led_, s.bumper_rear_active ? "red" : "green");

    // 📍 定位
    const StatusInfo li = locStatusInfo(s.localization.status);
    loc_status_->setText(li.text);
    loc_status_->setStyleSheet(QString("color: %1; font-weight: bold;").arg(li.color));
    // score 由 cartographer 源头（GetLocalizationConfidence）校准到 [0,1]（重合率 0.8→1.0），
    // 此处直接显示；评价档 0.8/0.5 与 command_board 决策判据同源。
    const float score = s.localization.score;
    const QString conf = score > 0.8f ? "GOOD" : (score >= 0.5f ? "NORMAL" : "BAD");
    const QString conf_color = score > 0.8f ? "green" : (score >= 0.5f ? "orange" : "red");
    loc_score_->setText(QString("%1   置信度 %2").arg(score, 0, 'f', 2).arg(conf));
    loc_score_->setStyleSheet(QString("color: %1;").arg(conf_color));

    // 🗺 当前地图
    map_label_->setText(QString::fromStdString(s.current_map_name.empty() ? "（未加载）" : s.current_map_name));

    // 📋 任务
    task_->setText(QString::fromStdString(s.task_status.empty() ? "--" : s.task_status));

    // 🧭 位姿 / 速度
    const double yaw = quatToYaw(s.pose.orientation);
    pose_->setText(QString("x %1   y %2   yaw %3")
                       .arg(s.pose.position.x, 0, 'f', 2)
                       .arg(s.pose.position.y, 0, 'f', 2)
                       .arg(yaw, 0, 'f', 2));
    vel_->setText(QString("v %1 m/s   ω %2 rad/s")
                      .arg(s.velocity.linear.x, 0, 'f', 2)
                      .arg(s.velocity.angular.z, 0, 'f', 2));

    // 💾 自动保存
    saver_->setText(has_saver ? QString::fromStdString(saver) : "--");

    // 🚨 急停丢位恢复
    const std::string rc = s.recover_state.empty() ? "NORMAL" : s.recover_state;
    recover_->setText(QString::fromStdString(rc));
    const QString rc_color =
        rc == "ALARM" ? "red" : (rc == "NORMAL" ? "green" : "orange");
    recover_->setStyleSheet(QString("color: %1; font-weight: bold;").arg(rc_color));
    if (!s.safety_alarm.empty()) {
        alarm_->setText(QString("⚠️ %1").arg(QString::fromStdString(s.safety_alarm)));
        alarm_->setStyleSheet("color: red;");
        alarm_->setVisible(true);
    } else {
        alarm_->setText("");
        alarm_->setVisible(false);
    }
}

void CommandBoardStatusPanel::save(rviz_common::Config config) const {
    rviz_common::Panel::save(config);
    config.mapSetValue("topic_prefix", topic_edit_->text());
}

void CommandBoardStatusPanel::load(const rviz_common::Config& config) {
    rviz_common::Panel::load(config);
    QString prefix;
    if (config.mapGetString("topic_prefix", &prefix)) {
        topic_edit_->setText(prefix);
    }
}

}  // namespace command_board_rviz_plugins

PLUGINLIB_EXPORT_CLASS(command_board_rviz_plugins::CommandBoardStatusPanel,
                       rviz_common::Panel)
