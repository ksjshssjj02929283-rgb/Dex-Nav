// command_board_rviz_plugins / CommandBoardStatusPanel
//
// RViz Panel：订阅 command_board 聚合发布的 NavSystemStatus + auto_map_saver_status，
// 分区卡片可视化电池/安全/定位/任务/位姿/自动保存/急停丢位恢复。纯只读。
//
// 线程模型：ROS 订阅回调（executor 线程）只写缓存；QTimer(1Hz, GUI 线程)读缓存刷
// widget。严禁在回调里直接操作 Qt widget（跨线程崩溃）。缓存用 mutex 保护。

#ifndef COMMAND_BOARD_RVIZ_PLUGINS__COMMAND_BOARD_STATUS_PANEL_HPP_
#define COMMAND_BOARD_RVIZ_PLUGINS__COMMAND_BOARD_STATUS_PANEL_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <QLabel>
#include <QLineEdit>
#include <QProgressBar>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_interfaces/msg/nav_system_status.hpp>

class QTimer;

namespace command_board_rviz_plugins {

class CommandBoardStatusPanel : public rviz_common::Panel {
    Q_OBJECT

public:
    explicit CommandBoardStatusPanel(QWidget* parent = nullptr);
    ~CommandBoardStatusPanel() override = default;

    void onInitialize() override;
    void save(rviz_common::Config config) const override;
    void load(const rviz_common::Config& config) override;

private Q_SLOTS:
    void refreshUi();        // QTimer(1Hz)：读缓存刷 widget + 数据新鲜度
    void onTopicChanged();   // 改话题前缀重订阅

private:
    void setupUi();
    void subscribe();
    void onStatus(nav_interfaces::msg::NavSystemStatus::SharedPtr msg);   // 回调：仅存缓存
    void onSaverStatus(std_msgs::msg::String::SharedPtr msg);            // 回调：仅存缓存
    static void setLed(QLabel* led, const QString& color);

    // UI widgets
    QLineEdit*    topic_edit_{nullptr};
    QLabel*       map_label_{nullptr};
    QProgressBar* bat_bar_{nullptr};
    QLabel*       bat_text_{nullptr};
    QLabel*       bat_vi_{nullptr};
    QLabel*       estop_led_{nullptr};
    QLabel*       bumper_f_led_{nullptr};
    QLabel*       bumper_r_led_{nullptr};
    QLabel*       loc_status_{nullptr};
    QLabel*       loc_score_{nullptr};
    QLabel*       task_{nullptr};
    QLabel*       pose_{nullptr};
    QLabel*       vel_{nullptr};
    QLabel*       saver_{nullptr};
    QLabel*       recover_{nullptr};
    QLabel*       alarm_{nullptr};
    QLabel*       fresh_{nullptr};
    QTimer*       ui_timer_{nullptr};

    // ROS
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<nav_interfaces::msg::NavSystemStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr saver_sub_;

    // Cache（mutex 保护；回调写、QTimer 读）
    mutable std::mutex cache_mutex_;
    nav_interfaces::msg::NavSystemStatus last_status_;
    std::string last_saver_;
    bool        has_status_{false};
    bool        has_saver_{false};
    rclcpp::Time last_recv_;
};

}  // namespace command_board_rviz_plugins

#endif  // COMMAND_BOARD_RVIZ_PLUGINS__COMMAND_BOARD_STATUS_PANEL_HPP_
