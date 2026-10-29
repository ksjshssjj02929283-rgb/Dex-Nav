#pragma once

#include <QJsonObject>
#include <QString>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;
class QRadioButton;
class QSpinBox;
class QTableWidget;
class QTimer;

namespace nav_benchmark_rviz {

// 导航 Benchmark 测试面板。界面只管按钮和显示，测试逻辑在机器人上的 nbt serve 里；
// 两者只走话题：/nbt/cmd（发命令）、/nbt/status（锁存状态）、/nbt/events（事件流），载荷是 JSON 字符串。
class BenchmarkPanel : public rviz_common::Panel {
    Q_OBJECT

public:
    explicit BenchmarkPanel(QWidget* parent = nullptr);
    ~BenchmarkPanel() override;

    void onInitialize() override;
    void load(const rviz_common::Config& config) override;
    void save(rviz_common::Config config) const override;

    // 以下与 ROS 无关，便于单测直接驱动界面
    void applyStatus(const QJsonObject& status);
    void appendEvent(const QJsonObject& event);
    void setCommandSink(std::function<void(const QString&)> sink) { sink_ = std::move(sink); }
    static QString preflightName(const QString& key);

    // 单测用的控件访问
    QListWidget* targetList() const { return target_list_; }
    QTableWidget* preflightTable() const { return preflight_table_; }
    QCheckBox* clearCheck() const { return clear_check_; }
    QPushButton* runButton() const { return run_btn_; }
    QPushButton* hardStopButton() const { return hard_stop_btn_; }
    QLabel* hintLabel() const { return hint_label_; }
    QListWidget* eventList() const { return event_list_; }

private Q_SLOTS:
    void onPreflight();
    void onSessionToggle();
    void onMark(const QString& text);
    void onNavRun();
    void onHardStop();
    void onStaleCheck();

private:
    void buildUi();
    void send(const QString& op, const QJsonObject& args = QJsonObject());
    void hint(const QString& text, bool error = false);
    void refreshTargets(const QJsonObject& assets);
    void renderPreflight(const QJsonObject& preflight);
    void renderTiles(const QJsonObject& tiles);
    void renderSession(const QJsonValue& session);
    void renderTask(const QJsonObject& task);

    std::function<void(const QString&)> sink_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr events_sub_;

    qint64 last_status_ms_ = 0;
    bool task_running_ = false;
    bool session_active_ = false;
    QString points_sig_, routes_sig_;
    QString last_ack_id_;

    QLabel* conn_label_ = nullptr;
    QLabel* hint_label_ = nullptr;
    QPushButton* preflight_btn_ = nullptr;
    QPushButton* hard_stop_btn_ = nullptr;
    QTableWidget* preflight_table_ = nullptr;
    QLabel* loc_label_ = nullptr;
    QLabel* nav_label_ = nullptr;
    QLabel* obs_label_ = nullptr;
    QLabel* safety_label_ = nullptr;
    QLabel* power_label_ = nullptr;
    QLabel* backend_label_ = nullptr;
    QLabel* sensors_label_ = nullptr;
    QComboBox* case_combo_ = nullptr;
    QLineEdit* label_edit_ = nullptr;
    QPushButton* session_btn_ = nullptr;
    QLabel* session_label_ = nullptr;
    QLineEdit* mark_edit_ = nullptr;
    QRadioButton* kind_point_ = nullptr;
    QRadioButton* kind_route_ = nullptr;
    QListWidget* target_list_ = nullptr;
    QSpinBox* repeat_spin_ = nullptr;
    QDoubleSpinBox* gap_spin_ = nullptr;
    QCheckBox* stop_on_fail_ = nullptr;
    QCheckBox* clear_check_ = nullptr;
    QPushButton* run_btn_ = nullptr;
    QLabel* task_label_ = nullptr;
    QTableWidget* result_table_ = nullptr;
    QDoubleSpinBox* speed_spin_ = nullptr;
    QCheckBox* speed_percent_ = nullptr;
    QListWidget* event_list_ = nullptr;
    QTimer* stale_timer_ = nullptr;
};

}  // namespace nav_benchmark_rviz
