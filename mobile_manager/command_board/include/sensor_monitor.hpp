// command_board / SensorMonitor (CE: 导航必要传感器监测)
//
// 判活只看话题是否按时到达（接收时刻用单调时钟，不用消息时间戳——ptp4l 曾把系统
// 时钟往回跳 170~220 ms）；订阅与判定定时器跑在**独立回调组**，不与
// OnRecoverTimer / 重置原点服务共用的互斥回调组串行，避免地图操作期间失明。
//
// 判定结果：
//   - 写入 StatusMonitor（nav_fault_code / nav_fault_detail / sensors）
//   - 进入故障时取消导航（async_cancel_all_goals），故障持续每 cancel_repeat_sec 再取消一次
//   - 解除需连续无故障 recover_hold_sec，不自动恢复被取消的任务

#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "nav_interfaces/msg/nav_system_status.hpp"
#include "nav_interfaces/msg/sensor_state.hpp"

namespace command_board {

class SensorMonitor {
public:
    // 导航故障码：编排域状态码 E_CB_NAV_SENSOR_FAULT（模块 22 / 子模块 04 / 等级 2）。
    static constexpr int32_t kNavSensorFaultCode = nav_interfaces::msg::NavSystemStatus::E_CB_NAV_SENSOR_FAULT;

    struct SensorConfig {
        std::string name;
        std::string topic;
        std::string type;
        double timeout_sec = 1.0;
        bool required = true;
        bool suppress_on_estop = false;
    };

    struct Config {
        bool enable = true;
        double check_period_sec = 0.1;
        double startup_grace_sec = 15.0;
        double recover_hold_sec = 2.0;
        double cancel_repeat_sec = 1.0;
        double estop_release_grace_sec = 3.0;
        double stats_window_sec = 10.0;
        int32_t fault_code = kNavSensorFaultCode;
        std::vector<SensorConfig> sensors;
    };

    // 单调时钟，单位秒。
    using ClockFn = std::function<double()>;

    struct Hooks {
        std::function<void()> cancel_navigation;  // async_cancel_all_goals
        std::function<bool()> is_estop_active;    // 轻量查询，不走 TF
        std::function<void()> publish_now;        // 立即补发一次 NavSystemStatus
        std::function<void(int32_t, const std::string &, const std::vector<nav_interfaces::msg::SensorState> &)>
            set_nav_fault;
    };

    SensorMonitor(const Config &config, Hooks hooks, const ClockFn clock = DefaultClock);
    ~SensorMonitor() = default;

    SensorMonitor(const SensorMonitor &) = delete;
    SensorMonitor &operator=(const SensorMonitor &) = delete;

    // 创建订阅（generic subscription，不反序列化）与判定定时器；enable=false 时不做任何事。
    void AttachToNode(rclcpp::Node *node, const rclcpp::CallbackGroup::SharedPtr group);

    // 收到一帧（由订阅回调直接驱动，O(1) 下标直达）。
    void OnFrame(const std::size_t index);
    // 收到一帧（由测试或名称驱动）。name 为配置中的名称。
    void OnFrame(const std::string &name);

    // 判定 + 故障动作；由定时器驱动，测试里可手工调用。
    void Tick();

    // 当前快照（供断言 / 上报）。
    int32_t NavFaultCode() const { return nav_fault_code_; }
    const std::string &NavFaultDetail() const { return nav_fault_detail_; }
    bool NavFaultActive() const { return nav_fault_latched_; }
    bool Enabled() const { return config_.enable; }
    const std::vector<nav_interfaces::msg::SensorState> &SensorStates() const { return sensor_states_; }

    static double DefaultClock();

private:
    struct SensorRuntime {
        SensorConfig config;
        bool ever_received = false;
        double last_rx_steady = -1.0;
        uint8_t state = nav_interfaces::msg::SensorState::STATE_UNKNOWN;
        std::deque<double> rx_times;  // 统计窗口内的接收时刻
    };

    std::string ComposeFaultDetail() const;
    static std::string FormatAge(const double age_sec);

    Config config_;
    Hooks hooks_;
    ClockFn clock_;

    std::vector<SensorRuntime> sensors_;
    std::vector<nav_interfaces::msg::SensorState> sensor_states_;

    double start_time_ = -1.0;
    double estop_released_at_ = -1.0;
    bool estop_active_ = false;

    // 导航故障状态机
    bool nav_fault_latched_ = false;
    double clear_since_ = -1.0;
    double last_cancel_ = -1.0;
    int32_t nav_fault_code_ = 0;
    std::string nav_fault_detail_;
    double last_snapshot_sync_ = -1.0;  // 降低快照深拷贝与加锁频率

    // ROS 资源（AttachToNode 后有效）
    std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// 从节点参数读取 sensor_monitor 段（§5.5）；不通过则返回 false 并填 err。
bool LoadSensorMonitorConfig(rclcpp::Node *node, SensorMonitor::Config &config, std::string &err);

// 参数校验（缺 topic/type、timeout<=0、check_period > 最小超时/2、名称重复等）。
bool ValidateSensorMonitorConfig(const SensorMonitor::Config &config, std::string &err);

}  // namespace command_board
