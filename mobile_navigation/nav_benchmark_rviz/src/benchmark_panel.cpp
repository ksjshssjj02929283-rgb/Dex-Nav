#include "nav_benchmark_rviz/benchmark_panel.hpp"

#include <algorithm>

#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPointer>
#include <QPushButton>
#include <QRadioButton>
#include <QSpinBox>
#include <QTabWidget>
#include <QTableWidget>
#include <QTimer>
#include <QUuid>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace nav_benchmark_rviz {

namespace {

constexpr int kProtocol = 1;
constexpr int kMaxEvents = 200;
constexpr qint64 kStaleMs = 3000;

const char* kCasePresets[] = {"G-MAP", "G-LOC", "G-ROUTE", "G-DOCK", "G-MOT", "G-STOP", "G-CHG", "G-STB",
                              "G-SYS", "S-TRAY", "G-FREE", "G-AVOID"};
const char* kMarkPresets[] = {"放障碍", "移走障碍", "人员进入", "人员离开", "按下急停", "松开急停", "放上 tray", "取下 tray"};

QString levelColor(const QString& level) {
    if (level == "ok") return "#2e7d32";
    if (level == "warn") return "#ef6c00";
    return "#c62828";
}

QString levelText(const QString& level) {
    if (level == "ok") return "通过";
    if (level == "warn") return "注意";
    return "不通过";
}

QString str(const QJsonObject& o, const char* key) {
    const QJsonValue v = o.value(key);
    if (v.isString()) return v.toString();
    if (v.isBool()) return v.toBool() ? "是" : "否";
    if (v.isDouble()) return QString::number(v.toDouble());
    return QString();
}

QString fmtTime(const QString& t) { return t.size() >= 19 ? t.mid(11, 8) : t; }

}  // namespace

BenchmarkPanel::BenchmarkPanel(QWidget* parent) : rviz_common::Panel(parent) {
    buildUi();
    stale_timer_ = new QTimer(this);
    connect(stale_timer_, &QTimer::timeout, this, &BenchmarkPanel::onStaleCheck);
    stale_timer_->start(1000);
}

BenchmarkPanel::~BenchmarkPanel() = default;

QString BenchmarkPanel::preflightName(const QString& key) {
    static const QHash<QString, QString> names = {
        {"localization", "定位"}, {"clock", "机器人时钟"}, {"backend", "导航后端"}, {"system", "系统状态"},
        {"map", "当前地图"}, {"safety", "急停 / 防撞条"}, {"sensors", "必要传感器"}, {"battery", "电量"},
        {"nav_fault", "导航故障"}, {"obstacle", "停障区"}, {"assets", "点位 / 路线"}, {"nav_server", "导航入口"},
        {"disk", "磁盘余量"}, {"versions", "版本"}};
    return names.value(key, key);
}

void BenchmarkPanel::buildUi() {
    auto* root = new QVBoxLayout(this);

    // 顶栏：连接状态、测前检查、硬停（始终可见）
    auto* top = new QHBoxLayout();
    conn_label_ = new QLabel("后端：未连接");
    preflight_btn_ = new QPushButton("测前检查");
    hard_stop_btn_ = new QPushButton("硬停");
    hard_stop_btn_->setMinimumHeight(40);
    hard_stop_btn_->setStyleSheet("QPushButton{background:#c62828;color:white;font-weight:bold;font-size:16px;}");
    top->addWidget(conn_label_, 1);
    top->addWidget(preflight_btn_);
    top->addWidget(hard_stop_btn_);
    root->addLayout(top);
    hint_label_ = new QLabel();
    hint_label_->setWordWrap(true);
    root->addWidget(hint_label_);

    auto* tabs = new QTabWidget();
    root->addWidget(tabs, 1);

    // 状态
    auto* status_page = new QWidget();
    auto* sl = new QVBoxLayout(status_page);
    auto* tiles = new QGroupBox("状态看板");
    auto* tg = new QGridLayout(tiles);
    auto addTile = [&](int row, const QString& name, QLabel*& label) {
        tg->addWidget(new QLabel(name), row, 0);
        label = new QLabel("—");
        label->setWordWrap(true);
        label->setTextInteractionFlags(Qt::TextSelectableByMouse);
        tg->addWidget(label, row, 1);
    };
    addTile(0, "定位", loc_label_);
    addTile(1, "导航", nav_label_);
    addTile(2, "障碍", obs_label_);
    addTile(3, "急停 / 防撞条", safety_label_);
    addTile(4, "电量", power_label_);
    addTile(5, "后端 / 地图", backend_label_);
    addTile(6, "传感器", sensors_label_);
    tg->setColumnStretch(1, 1);
    sl->addWidget(tiles);
    auto* pf = new QGroupBox("测前检查结果");
    auto* pl = new QVBoxLayout(pf);
    preflight_table_ = new QTableWidget(0, 3);
    preflight_table_->setHorizontalHeaderLabels({"项", "结果", "详情"});
    preflight_table_->horizontalHeader()->setStretchLastSection(true);
    preflight_table_->verticalHeader()->setVisible(false);
    preflight_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    pl->addWidget(preflight_table_);
    sl->addWidget(pf, 1);
    tabs->addTab(status_page, "状态");

    // 会话与打点
    auto* sess_page = new QWidget();
    auto* ssl = new QVBoxLayout(sess_page);
    auto* sg = new QGridLayout();
    case_combo_ = new QComboBox();
    case_combo_->setEditable(true);
    for (const char* c : kCasePresets) case_combo_->addItem(c);
    label_edit_ = new QLineEdit();
    label_edit_->setPlaceholderText("工况说明，如：空载 0.5 m/s");
    sg->addWidget(new QLabel("用例"), 0, 0);
    sg->addWidget(case_combo_, 0, 1);
    sg->addWidget(new QLabel("标签"), 1, 0);
    sg->addWidget(label_edit_, 1, 1);
    ssl->addLayout(sg);
    session_btn_ = new QPushButton("开始会话（录包 + 环境快照）");
    ssl->addWidget(session_btn_);
    session_label_ = new QLabel("没有进行中的会话");
    session_label_->setWordWrap(true);
    session_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ssl->addWidget(session_label_);
    auto* mg = new QGroupBox("打点（时刻记进录包和事件表）");
    auto* ml = new QGridLayout(mg);
    int i = 0;
    for (const char* m : kMarkPresets) {
        auto* b = new QPushButton(m);
        connect(b, &QPushButton::clicked, this, [this, m]() { onMark(QString::fromUtf8(m)); });
        ml->addWidget(b, i / 2, i % 2);
        ++i;
    }
    mark_edit_ = new QLineEdit();
    mark_edit_->setPlaceholderText("自定义打点文字");
    auto* mark_btn = new QPushButton("打点");
    ml->addWidget(mark_edit_, (i + 1) / 2, 0);
    ml->addWidget(mark_btn, (i + 1) / 2, 1);
    connect(mark_btn, &QPushButton::clicked, this, [this]() { onMark(mark_edit_->text()); });
    connect(mark_edit_, &QLineEdit::returnPressed, this, [this]() { onMark(mark_edit_->text()); });
    ssl->addWidget(mg);
    ssl->addStretch(1);
    tabs->addTab(sess_page, "会话");

    // 导航任务
    auto* task_page = new QWidget();
    auto* tl = new QVBoxLayout(task_page);
    auto* kind_row = new QHBoxLayout();
    kind_route_ = new QRadioButton("路线");
    kind_point_ = new QRadioButton("点位");
    kind_route_->setChecked(true);
    kind_row->addWidget(kind_route_);
    kind_row->addWidget(kind_point_);
    kind_row->addStretch(1);
    tl->addLayout(kind_row);
    target_list_ = new QListWidget();
    target_list_->setSelectionMode(QAbstractItemView::ExtendedSelection);
    target_list_->setMaximumHeight(120);
    tl->addWidget(target_list_);
    auto* og = new QGridLayout();
    repeat_spin_ = new QSpinBox();
    repeat_spin_->setRange(1, 10000);
    gap_spin_ = new QDoubleSpinBox();
    gap_spin_->setRange(0.0, 600.0);
    gap_spin_->setValue(2.0);
    gap_spin_->setSuffix(" s");
    stop_on_fail_ = new QCheckBox("失败即停");
    clear_check_ = new QCheckBox("已清场，行驶路径上无人");
    og->addWidget(new QLabel("轮数"), 0, 0);
    og->addWidget(repeat_spin_, 0, 1);
    og->addWidget(new QLabel("间隔"), 0, 2);
    og->addWidget(gap_spin_, 0, 3);
    og->addWidget(stop_on_fail_, 1, 0, 1, 2);
    og->addWidget(clear_check_, 1, 2, 1, 2);
    tl->addLayout(og);
    auto* btn_row = new QHBoxLayout();
    run_btn_ = new QPushButton("开始");
    auto* pause_btn = new QPushButton("暂停");
    auto* resume_btn = new QPushButton("继续");
    auto* cancel_btn = new QPushButton("取消");
    btn_row->addWidget(run_btn_);
    btn_row->addWidget(pause_btn);
    btn_row->addWidget(resume_btn);
    btn_row->addWidget(cancel_btn);
    tl->addLayout(btn_row);
    auto* sp_row = new QHBoxLayout();
    speed_spin_ = new QDoubleSpinBox();
    speed_spin_->setRange(0.0, 100.0);
    speed_spin_->setDecimals(2);
    speed_spin_->setValue(0.3);
    speed_percent_ = new QCheckBox("按百分比");
    auto* speed_btn = new QPushButton("限速");
    auto* speed_clear = new QPushButton("解除限速");
    sp_row->addWidget(new QLabel("限速"));
    sp_row->addWidget(speed_spin_);
    sp_row->addWidget(speed_percent_);
    sp_row->addWidget(speed_btn);
    sp_row->addWidget(speed_clear);
    tl->addLayout(sp_row);
    task_label_ = new QLabel("空闲");
    task_label_->setWordWrap(true);
    tl->addWidget(task_label_);
    result_table_ = new QTableWidget(0, 4);
    result_table_->setHorizontalHeaderLabels({"目标", "结果", "耗时（诊断）", "错误码"});
    result_table_->horizontalHeader()->setStretchLastSection(true);
    result_table_->verticalHeader()->setVisible(false);
    result_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    tl->addWidget(result_table_, 1);
    tabs->addTab(task_page, "任务");

    // 事件
    event_list_ = new QListWidget();
    tabs->addTab(event_list_, "事件");

    connect(preflight_btn_, &QPushButton::clicked, this, &BenchmarkPanel::onPreflight);
    connect(hard_stop_btn_, &QPushButton::clicked, this, &BenchmarkPanel::onHardStop);
    connect(session_btn_, &QPushButton::clicked, this, &BenchmarkPanel::onSessionToggle);
    connect(run_btn_, &QPushButton::clicked, this, &BenchmarkPanel::onNavRun);
    connect(pause_btn, &QPushButton::clicked, this, [this]() { send("nav_pause"); });
    connect(resume_btn, &QPushButton::clicked, this, [this]() { send("nav_resume"); });
    connect(cancel_btn, &QPushButton::clicked, this, [this]() { send("nav_cancel"); });
    connect(speed_btn, &QPushButton::clicked, this, [this]() {
        send("speed_limit", QJsonObject{{"value", speed_spin_->value()}, {"percent", speed_percent_->isChecked()}});
    });
    connect(speed_clear, &QPushButton::clicked, this,
            [this]() { send("speed_limit", QJsonObject{{"value", 0.0}, {"percent", false}}); });
    connect(kind_route_, &QRadioButton::toggled, this, [this](bool) { points_sig_.clear(); routes_sig_.clear(); });
}

void BenchmarkPanel::onInitialize() {
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    cmd_pub_ = node_->create_publisher<std_msgs::msg::String>("/nbt/cmd", rclcpp::QoS(10));
    QPointer<BenchmarkPanel> self(this);
    status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/nbt/status", rclcpp::QoS(1).reliable().transient_local(), [self](std_msgs::msg::String::SharedPtr m) {
            const QByteArray data = QByteArray::fromStdString(m->data);
            QMetaObject::invokeMethod(self.data(), [self, data]() {
                if (!self) return;
                const QJsonDocument doc = QJsonDocument::fromJson(data);
                if (doc.isObject()) self->applyStatus(doc.object());
            }, Qt::QueuedConnection);
        });
    events_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/nbt/events", rclcpp::QoS(50), [self](std_msgs::msg::String::SharedPtr m) {
            const QByteArray data = QByteArray::fromStdString(m->data);
            QMetaObject::invokeMethod(self.data(), [self, data]() {
                if (!self) return;
                const QJsonDocument doc = QJsonDocument::fromJson(data);
                if (doc.isObject()) self->appendEvent(doc.object());
            }, Qt::QueuedConnection);
        });
    sink_ = [this](const QString& json) {
        std_msgs::msg::String msg;
        msg.data = json.toStdString();
        cmd_pub_->publish(msg);
    };
}

void BenchmarkPanel::load(const rviz_common::Config& config) {
    rviz_common::Panel::load(config);
    QString s;
    if (config.mapGetString("case", &s) && !s.isEmpty()) case_combo_->setEditText(s);
    if (config.mapGetString("label", &s)) label_edit_->setText(s);
    int n = 0;
    if (config.mapGetInt("repeat", &n) && n > 0) repeat_spin_->setValue(n);
    float g = 0.0F;
    if (config.mapGetFloat("gap", &g)) gap_spin_->setValue(g);
    if (config.mapGetString("kind", &s)) (s == "point" ? kind_point_ : kind_route_)->setChecked(true);
}

void BenchmarkPanel::save(rviz_common::Config config) const {
    rviz_common::Panel::save(config);
    config.mapSetValue("case", case_combo_->currentText());
    config.mapSetValue("label", label_edit_->text());
    config.mapSetValue("repeat", repeat_spin_->value());
    config.mapSetValue("gap", gap_spin_->value());
    config.mapSetValue("kind", kind_point_->isChecked() ? "point" : "route");
}

void BenchmarkPanel::send(const QString& op, const QJsonObject& args) {
    if (!sink_) {
        hint("面板还没有连上 ROS，命令没有发出", true);
        return;
    }
    QJsonObject cmd{{"version", kProtocol},
                    {"id", QUuid::createUuid().toString(QUuid::WithoutBraces).left(8)},
                    {"op", op},
                    {"args", args}};
    sink_(QString::fromUtf8(QJsonDocument(cmd).toJson(QJsonDocument::Compact)));
}

void BenchmarkPanel::hint(const QString& text, bool error) {
    hint_label_->setText(text);
    hint_label_->setStyleSheet(error ? "color:#c62828;" : "color:#2e7d32;");
}

void BenchmarkPanel::onPreflight() {
    send("preflight");
    hint("测前检查中…");
}

void BenchmarkPanel::onHardStop() {
    send("hard_stop");
    hint("已发硬停，并取消当前任务", true);
}

void BenchmarkPanel::onSessionToggle() {
    if (session_active_) {
        send("session_stop");
        hint("正在结束会话（等录包写完）…");
    } else {
        send("session_start", QJsonObject{{"case", case_combo_->currentText().trimmed()},
                                          {"label", label_edit_->text().trimmed()}});
        hint("正在开始会话…");
    }
}

void BenchmarkPanel::onMark(const QString& text) {
    const QString t = text.trimmed();
    if (t.isEmpty()) {
        hint("打点文字为空", true);
        return;
    }
    send("mark", QJsonObject{{"text", t}});
    mark_edit_->clear();
}

void BenchmarkPanel::onNavRun() {
    if (!clear_check_->isChecked()) {
        hint("先确认已清场：勾选「已清场，行驶路径上无人」后再开始", true);
        return;
    }
    QJsonArray targets;
    for (auto* item : target_list_->selectedItems()) targets.append(item->text());
    if (targets.isEmpty()) {
        hint("先在列表里选目标（可多选，按列表顺序轮流去）", true);
        return;
    }
    send("nav_run", QJsonObject{{"kind", kind_point_->isChecked() ? "point" : "route"},
                                {"targets", targets},
                                {"repeat", repeat_spin_->value()},
                                {"gap", gap_spin_->value()},
                                {"stop_on_fail", stop_on_fail_->isChecked()},
                                {"confirm_clear", true}});
    hint(QString("已下发 %1 个目标 × %2 轮").arg(targets.size()).arg(repeat_spin_->value()));
}

void BenchmarkPanel::onStaleCheck() {
    if (last_status_ms_ == 0) return;
    const qint64 age = QDateTime::currentMSecsSinceEpoch() - last_status_ms_;
    if (age > kStaleMs) {
        conn_label_->setText(QString("后端：%1 s 没有状态（nbt serve 是否在运行？）").arg(age / 1000));
        conn_label_->setStyleSheet("color:#c62828;");
    }
}

void BenchmarkPanel::applyStatus(const QJsonObject& st) {
    last_status_ms_ = QDateTime::currentMSecsSinceEpoch();
    if (st.value("version").toInt() != kProtocol) {
        conn_label_->setText(QString("后端协议版本 %1，面板是 %2，请升级到同一版本").arg(st.value("version").toInt()).arg(kProtocol));
        conn_label_->setStyleSheet("color:#c62828;");
        return;
    }
    conn_label_->setText(QString("后端在线 nbt %1 · %2%3")
                             .arg(st.value("serve_version").toString(), fmtTime(st.value("time").toString()),
                                  st.value("nav_ready").toBool() ? "" : " · 导航入口不在线"));
    conn_label_->setStyleSheet(st.value("nav_ready").toBool() ? "color:#2e7d32;" : "color:#ef6c00;");
    refreshTargets(st.value("assets").toObject());
    renderTiles(st.value("tiles").toObject());
    renderPreflight(st.value("preflight").toObject());
    renderSession(st.value("session"));
    renderTask(st.value("task").toObject());
    const QJsonObject ack = st.value("ack").toObject();
    if (!ack.isEmpty()) {
        const QString id = ack.value("id").toString();
        if (id != last_ack_id_) {
            last_ack_id_ = id;
            hint(QString("%1：%2").arg(ack.value("op").toString(), ack.value("message").toString()),
                 !ack.value("ok").toBool());
        }
    }
}

void BenchmarkPanel::refreshTargets(const QJsonObject& assets) {
    const bool route = kind_route_->isChecked();
    const QJsonArray names = assets.value(route ? "routes" : "points").toArray();
    QStringList list;
    for (const auto& v : names) list << v.toString();
    const QString sig = list.join('\n');
    QString& last = route ? routes_sig_ : points_sig_;
    if (sig == last) return;   // 列表没变就不重建，保留用户的选择
    last = sig;
    QStringList selected;
    for (auto* item : target_list_->selectedItems()) selected << item->text();
    target_list_->clear();
    target_list_->addItems(list);
    for (int i = 0; i < target_list_->count(); ++i) {
        if (selected.contains(target_list_->item(i)->text())) target_list_->item(i)->setSelected(true);
    }
    if (list.isEmpty()) {
        target_list_->addItem(route ? "（当前地图没有命名路线）" : "（当前地图没有点位）");
        target_list_->item(0)->setFlags(Qt::NoItemFlags);
    }
}

void BenchmarkPanel::renderPreflight(const QJsonObject& pf) {
    const QJsonArray items = pf.value("items").toArray();
    if (items.isEmpty()) return;
    preflight_table_->setRowCount(items.size());
    int fails = 0;
    for (int i = 0; i < items.size(); ++i) {
        const QJsonObject it = items[i].toObject();
        const QString level = it.value("level").toString();
        if (level == "fail") ++fails;
        auto* name = new QTableWidgetItem(preflightName(it.value("key").toString()));
        auto* res = new QTableWidgetItem(levelText(level));
        res->setForeground(QColor(levelColor(level)));
        auto* det = new QTableWidgetItem(it.value("detail").toString());
        preflight_table_->setItem(i, 0, name);
        preflight_table_->setItem(i, 1, res);
        preflight_table_->setItem(i, 2, det);
    }
    preflight_btn_->setText(fails ? QString("测前检查（%1 项不通过）").arg(fails) : "测前检查（通过）");
    preflight_btn_->setStyleSheet(fails ? "color:#c62828;" : "color:#2e7d32;");
}

void BenchmarkPanel::renderTiles(const QJsonObject& tiles) {
    const QJsonObject loc = tiles.value("loc").toObject();
    if (!loc.isEmpty())
        loc_label_->setText(QString("%1 · 匹配 %2 · 分数 %3 · 错误码 %4")
                                .arg(str(loc, "status"), str(loc, "match_level"), str(loc, "score"),
                                     str(loc, "error_code")));
    const QJsonObject nav = tiles.value("nav").toObject();
    if (!nav.isEmpty())
        nav_label_->setText(QString("%1 · %2 · 剩余 %3 m · 错误码 %4 · 限速 %5")
                                .arg(str(nav, "state"), str(nav, "target"), str(nav, "distance_remaining"),
                                     str(nav, "error_code"), str(nav, "speed_limit")));
    const QJsonObject obs = tiles.value("obs").toObject();
    if (!obs.isEmpty()) {
        const QString s = str(obs, "status");
        obs_label_->setText(QString("%1 %2").arg(s, str(obs, "zone")));
        obs_label_->setStyleSheet(s.startsWith("0") ? "" : "color:#ef6c00;");
    }
    const QJsonObject sys = tiles.value("sys").toObject();
    if (!sys.isEmpty()) {
        const bool alarm = sys.value("estop").toBool() || sys.value("bumper_front").toBool() ||
                           sys.value("bumper_rear").toBool();
        safety_label_->setText(QString("急停 %1 · 前防撞条 %2 · 后防撞条 %3 · 恢复状态 %4")
                                   .arg(str(sys, "estop"), str(sys, "bumper_front"), str(sys, "bumper_rear"),
                                        str(sys, "recover_state")));
        safety_label_->setStyleSheet(alarm ? "color:#c62828;font-weight:bold;" : "");
        power_label_->setText(str(sys, "battery") + "%" + (sys.value("charging").toBool() ? " · 充电中" : ""));
        QStringList sensors;
        for (const auto& v : sys.value("_sensors").toArray()) {
            const QJsonObject s = v.toObject();
            static const char* st[] = {"未知", "正常", "故障", "已抑制"};
            const int k = s.value("state").toInt();
            sensors << QString("%1 %2 %3Hz").arg(s.value("name").toString(), (k >= 0 && k < 4) ? st[k] : "?",
                                                QString::number(s.value("rate_hz").toDouble()));
        }
        sensors_label_->setText(sensors.isEmpty() ? "—" : sensors.join(" · "));
        const QString fault = str(sys, "nav_fault_code");
        backend_label_->setText(QString("%1 · 地图 %2%3")
                                    .arg(str(tiles.value("backend").toObject(), "backend"), str(sys, "map"),
                                         (fault.isEmpty() || fault == "0") ? "" : " · 故障 " + fault));
    }
}

void BenchmarkPanel::renderSession(const QJsonValue& session) {
    const bool active = session.isObject();
    if (active) {
        const QJsonObject s = session.toObject();
        session_label_->setText(QString("进行中：%1 %2 · %3 s · 打点 %4 · %5\n%6")
                                    .arg(str(s, "case"), str(s, "label"), str(s, "elapsed"), str(s, "markers"),
                                         s.value("recording").toBool() ? "录包中" : "录包未运行",
                                         s.value("dir").toString()));
    } else if (session_active_) {
        session_label_->setText("会话已结束，结果在机器人上的会话目录（见事件页）");
    }
    if (active != session_active_) {
        session_active_ = active;
        session_btn_->setText(active ? "结束会话" : "开始会话（录包 + 环境快照）");
        case_combo_->setEnabled(!active);
        label_edit_->setEnabled(!active);
    }
}

void BenchmarkPanel::renderTask(const QJsonObject& task) {
    const bool running = task.value("running").toBool();
    if (running) {
        task_label_->setText(QString("运行中 %1 / %2：%3 · %4 · 剩余 %5 m")
                                 .arg(task.value("index").toInt())
                                 .arg(task.value("total").toInt())
                                 .arg(task.value("current").toString(), task.value("state").toString(),
                                      task.value("distance").isNull() ? "—" : QString::number(task.value("distance").toDouble())));
    }
    const QJsonObject st = task.value("stats").toObject();
    if (!running && !st.isEmpty()) {
        task_label_->setText(QString("空闲 · 共 %1，成功 %2，失败 %3 · 成功耗时 最短 %4 / 平均 %5 / 最长 %6 s（诊断）")
                                 .arg(st.value("total").toInt())
                                 .arg(st.value("ok").toInt())
                                 .arg(st.value("fail").toInt())
                                 .arg(str(st, "min"), str(st, "avg"), str(st, "max")));
    }
    if (task_running_ && !running) clear_check_->setChecked(false);   // 每次开跑都要重新确认清场
    task_running_ = running;
    run_btn_->setEnabled(!running);
    const QJsonArray rows = task.value("rows").toArray();
    const int n = std::min<int>(rows.size(), 20);
    result_table_->setRowCount(n);
    for (int i = 0; i < n; ++i) {
        const QJsonObject r = rows[rows.size() - 1 - i].toObject();   // 最新在上
        const bool ok = r.value("success").toBool();
        auto* res = new QTableWidgetItem(ok ? "成功" : "失败");
        res->setForeground(QColor(ok ? "#2e7d32" : "#c62828"));
        result_table_->setItem(i, 0, new QTableWidgetItem(r.value("target").toString()));
        result_table_->setItem(i, 1, res);
        result_table_->setItem(i, 2, new QTableWidgetItem(r.value("elapsed_s").toString()));
        result_table_->setItem(i, 3, new QTableWidgetItem(ok ? "" : r.value("error_text").toString() + " " +
                                                                        r.value("message").toString()));
    }
}

void BenchmarkPanel::appendEvent(const QJsonObject& ev) {
    const QString type = ev.value("type").toString();
    const QString t = fmtTime(ev.value("t").toString());
    QString line;
    if (type == "state") {
        QStringList parts;
        const QJsonObject ch = ev.value("changes").toObject();
        for (auto it = ch.begin(); it != ch.end(); ++it) {
            const QJsonArray pair = it.value().toArray();
            parts << (pair.at(0).isNull() ? QString("%1=%2").arg(it.key(), pair.at(1).toString())
                                          : QString("%1: %2 → %3").arg(it.key(), pair.at(0).toString(),
                                                                       pair.at(1).toString()));
        }
        line = QString("%1 [%2] %3").arg(t, ev.value("group").toString(), parts.join("，"));
    } else if (type == "mark") {
        line = QString("%1 打点：%2").arg(t, ev.value("text").toString());
    } else if (type == "ack") {
        line = QString("%1 命令 %2：%3 %4").arg(t, ev.value("op").toString(), ev.value("ok").toBool() ? "完成" : "失败",
                                             ev.value("message").toString());
    } else if (type == "task") {
        line = QString("%1 任务 %2：%3 %4 s %5").arg(t, ev.value("target").toString(),
                                                ev.value("success").toBool() ? "成功" : "失败",
                                                ev.value("elapsed_s").toString(), ev.value("error_text").toString());
    } else if (type == "task_done") {
        const QJsonObject st = ev.value("stats").toObject();
        line = QString("%1 任务序列结束：共 %2，成功 %3").arg(t).arg(st.value("total").toInt()).arg(st.value("ok").toInt());
    } else {
        line = QString("%1 %2").arg(t, QString::fromUtf8(QJsonDocument(ev).toJson(QJsonDocument::Compact)));
    }
    event_list_->insertItem(0, line);
    while (event_list_->count() > kMaxEvents) delete event_list_->takeItem(event_list_->count() - 1);
}

}  // namespace nav_benchmark_rviz

PLUGINLIB_EXPORT_CLASS(nav_benchmark_rviz::BenchmarkPanel, rviz_common::Panel)
