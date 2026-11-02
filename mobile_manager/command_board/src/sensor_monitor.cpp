#include "sensor_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace command_board {

namespace {

constexpr double kEpsilon = 1e-9;

rclcpp::SubscriptionOptions MakeOptions(const rclcpp::CallbackGroup::SharedPtr group) {
    rclcpp::SubscriptionOptions options;
    options.callback_group = group;
    return options;
}

}  // namespace

double SensorMonitor::DefaultClock() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

SensorMonitor::SensorMonitor(const Config& config, Hooks hooks, const ClockFn clock)
    : config_(config), hooks_(std::move(hooks)), clock_(clock ? clock : ClockFn(DefaultClock)) {
    start_time_ = clock_();
    sensors_.reserve(config_.sensors.size());
    for (const auto& sensor : config_.sensors) {
        SensorRuntime runtime;
        runtime.config = sensor;
        sensors_.push_back(std::move(runtime));
    }
}

void SensorMonitor::AttachToNode(rclcpp::Node* node, const rclcpp::CallbackGroup::SharedPtr group) {
    if (!config_.enable) {
        RCLCPP_INFO(node->get_logger(), "SensorMonitor disabled by parameter");
        return;
    }

    // 传感器判活：generic subscription 不反序列化，只记接收时刻与计数；
    // 类型由参数给定，换机型不改代码；best-effort 可同时匹配 reliable / best-effort 发布者。
    // 使用索引直达（O(1)），避免高频回调遍历比对字符串。
    for (std::size_t i = 0; i < sensors_.size(); ++i) {
        const auto options = MakeOptions(group);
        subscriptions_.push_back(node->create_generic_subscription(
            sensors_[i].config.topic, sensors_[i].config.type, rclcpp::SensorDataQoS(),
            [this, i](std::shared_ptr<rclcpp::SerializedMessage>) { OnFrame(i); }, options));
    }

    const auto period = std::chrono::duration<double>(std::max(config_.check_period_sec, 0.01));
    timer_ = node->create_wall_timer(
        period, [this]() { Tick(); }, group);

    RCLCPP_INFO(node->get_logger(),
                "SensorMonitor enabled: %zu sensors, check_period=%.3fs grace=%.1fs hold=%.1fs", sensors_.size(),
                config_.check_period_sec, config_.startup_grace_sec, config_.recover_hold_sec);
}

void SensorMonitor::OnFrame(const std::size_t index) {
    if (index >= sensors_.size()) {
        return;
    }
    auto& sensor = sensors_[index];
    const double now = clock_();
    sensor.ever_received = true;
    sensor.last_rx_steady = now;
    sensor.rx_times.push_back(now);
    while (!sensor.rx_times.empty() && now - sensor.rx_times.front() > config_.stats_window_sec) {
        sensor.rx_times.pop_front();
    }
}

void SensorMonitor::OnFrame(const std::string& name) {
    for (std::size_t i = 0; i < sensors_.size(); ++i) {
        if (sensors_[i].config.name == name) {
            OnFrame(i);
            return;
        }
    }
}

std::string SensorMonitor::FormatAge(const double age_sec) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.1f", age_sec);
    return std::string(buf);
}

std::string SensorMonitor::ComposeFaultDetail() const {
    std::string detail;
    for (const auto& sensor : sensors_) {
        if (sensor.state != nav_interfaces::msg::SensorState::STATE_FAULT) {
            continue;
        }
        if (!detail.empty()) {
            detail += ", ";
        }
        detail += sensor.config.name;
        if (!sensor.ever_received) {
            detail += "(never received)";
        } else {
            detail += "(stale " + FormatAge(clock_() - sensor.last_rx_steady) + "s)";
        }
    }

    if (detail.empty()) {
        return "";
    }
    return "sensor fault: " + detail;
}

void SensorMonitor::Tick() {
    if (!config_.enable) {
        return;
    }

    const double now = clock_();

    // 急停状态与解除时刻（true→false 边沿）。
    const bool estop = hooks_.is_estop_active ? hooks_.is_estop_active() : false;
    if (estop_active_ && !estop) {
        estop_released_at_ = now;
    }
    estop_active_ = estop;

    // ── 每路状态判定 + 统计 ─────────────────────────────────────
    sensor_states_.clear();
    sensor_states_.reserve(sensors_.size());
    bool required_fault = false;
    bool states_changed = false;
    for (auto& sensor : sensors_) {
        nav_interfaces::msg::SensorState state;
        state.name = sensor.config.name;
        state.topic = sensor.config.topic;
        state.required = sensor.config.required;

        const bool in_release_grace = sensor.config.suppress_on_estop && estop_released_at_ >= 0.0 &&
                                      (now - estop_released_at_) < config_.estop_release_grace_sec;

        const uint8_t prev_state = sensor.state;
        if (estop && sensor.config.suppress_on_estop) {
            sensor.state = nav_interfaces::msg::SensorState::STATE_SUPPRESSED;
        } else if (!sensor.ever_received) {
            sensor.state = (now > start_time_ + config_.startup_grace_sec)
                                ? nav_interfaces::msg::SensorState::STATE_FAULT
                                : nav_interfaces::msg::SensorState::STATE_UNKNOWN;
        } else {
            const bool stale = (now - sensor.last_rx_steady) > sensor.config.timeout_sec;
            sensor.state = (stale && !in_release_grace) ? nav_interfaces::msg::SensorState::STATE_FAULT
                                                        : nav_interfaces::msg::SensorState::STATE_OK;
        }
        if (sensor.state != prev_state) {
            states_changed = true;
        }

        // 统计窗口内：rate_hz / max_gap_sec
        while (!sensor.rx_times.empty() && now - sensor.rx_times.front() > config_.stats_window_sec) {
            sensor.rx_times.pop_front();
        }
        if (sensor.rx_times.size() >= 2) {
            const double span = sensor.rx_times.back() - sensor.rx_times.front();
            state.rate_hz = span > kEpsilon ? static_cast<float>((sensor.rx_times.size() - 1) / span) : 0.0f;
        } else {
            state.rate_hz = 0.0f;
        }
        state.max_gap_sec = 0.0f;
        state.age_sec = sensor.ever_received ? static_cast<float>(now - sensor.last_rx_steady) : -1.0f;
        state.state = sensor.state;

        if (sensor.state == nav_interfaces::msg::SensorState::STATE_FAULT &&
            sensor.config.required) {
            required_fault = true;
        }
        sensor_states_.push_back(state);
    }

    // ── 导航故障判定与状态机处理 ──────────────────────────────
    // 任意标记为 required=true 的必要传感器处于 STATE_FAULT 状态
    const bool active = required_fault;

    if (active) {
        clear_since_ = -1.0;
        // ── 故障首次进入（上升沿）：闭锁故障并执行取消动作 ──
        if (!nav_fault_latched_) {
            nav_fault_latched_ = true;
            nav_fault_code_ = config_.fault_code;
            nav_fault_detail_ = ComposeFaultDetail();
            last_cancel_ = now;
            // 调用 hook 向 Nav2 发起 async_cancel_all_goals，主动中止导航任务
            if (hooks_.cancel_navigation) {
                hooks_.cancel_navigation();
                RCLCPP_WARN(rclcpp::get_logger("command_board.sensor_monitor"),
                            "Navigation cancelled due to sensor fault (code %d): %s",
                            nav_fault_code_, nav_fault_detail_.c_str());
            }
            RCLCPP_ERROR(rclcpp::get_logger("command_board.sensor_monitor"), "Navigation sensor fault (code %d): %s",
                         nav_fault_code_, nav_fault_detail_.c_str());
            // 同步写入 StatusMonitor 快照
            if (hooks_.set_nav_fault) {
                last_snapshot_sync_ = now;
                hooks_.set_nav_fault(nav_fault_code_, nav_fault_detail_, sensor_states_);
            }
            // 立即广播一次 NavSystemStatus，不等 1Hz 定时器
            if (hooks_.publish_now) {
                hooks_.publish_now();
            }
        } else {
            // ── 故障持续期间：周期性重发取消指令，防止新任务进入或取消丢包 ──
            if (now - last_cancel_ >= config_.cancel_repeat_sec) {
                last_cancel_ = now;
                if (hooks_.cancel_navigation) {
                    hooks_.cancel_navigation();
                }
            }
            // 若故障明细产生变化（例如更多传感器超时），及时更新状态并刷新发布
            const std::string detail = ComposeFaultDetail();
            if (detail != nav_fault_detail_) {
                nav_fault_detail_ = detail;
                if (hooks_.set_nav_fault) {
                    last_snapshot_sync_ = now;
                    hooks_.set_nav_fault(nav_fault_code_, nav_fault_detail_, sensor_states_);
                }
                if (hooks_.publish_now) {
                    hooks_.publish_now();
                }
            }
        }
    } else if (nav_fault_latched_) {
        // ── 故障解除滤波与恢复（下降沿防抖）：需持续稳定无故障 recover_hold_sec ──
        if (clear_since_ < 0.0) {
            clear_since_ = now; // 记录首次恢复正常时刻
        }
        if (now - clear_since_ >= config_.recover_hold_sec) {
            // 持续无故障时间达标，正式解除故障闭锁
            nav_fault_latched_ = false;
            nav_fault_code_ = 0;
            nav_fault_detail_.clear();
            clear_since_ = -1.0;
            RCLCPP_INFO(rclcpp::get_logger("command_board.sensor_monitor"), "Navigation sensor fault cleared");
            if (hooks_.set_nav_fault) {
                last_snapshot_sync_ = now;
                hooks_.set_nav_fault(0, "", sensor_states_);
            }
            if (hooks_.publish_now) {
                hooks_.publish_now();
            }
        }
    }

    // 快照写入：仅在传感器状态改变或到达同步周期（如 1.0s）时才同步，避免 10Hz 高频加锁深拷贝
    const bool periodic_sync = (last_snapshot_sync_ < 0.0) || (now - last_snapshot_sync_ >= 1.0);
    if ((states_changed || periodic_sync) && hooks_.set_nav_fault) {
        last_snapshot_sync_ = now;
        hooks_.set_nav_fault(nav_fault_code_, nav_fault_detail_, sensor_states_);
    }
}

bool ValidateSensorMonitorConfig(const SensorMonitor::Config& config, std::string& err) {
    auto fail = [&err](const std::string& msg) {
        err = msg;
        return false;
    };

    if (!(config.check_period_sec > 0.0) || !std::isfinite(config.check_period_sec)) {
        return fail("check_period_sec must be positive and finite");
    }
    if (!(config.recover_hold_sec >= 0.0) || !std::isfinite(config.recover_hold_sec)) {
        return fail("recover_hold_sec must be >= 0 and finite");
    }
    if (!(config.cancel_repeat_sec > 0.0) || !std::isfinite(config.cancel_repeat_sec)) {
        return fail("cancel_repeat_sec must be positive and finite");
    }
    if (!(config.startup_grace_sec >= 0.0) || !std::isfinite(config.startup_grace_sec)) {
        return fail("startup_grace_sec must be >= 0 and finite");
    }

    double min_timeout = std::numeric_limits<double>::max();
    for (const auto& sensor : config.sensors) {
        if (sensor.name.empty()) {
            return fail("sensor name must not be empty");
        }
        if (sensor.topic.empty()) {
            return fail("sensor '" + sensor.name + "': topic must not be empty");
        }
        if (sensor.type.empty()) {
            return fail("sensor '" + sensor.name + "': type must not be empty");
        }
        if (!(sensor.timeout_sec > 0.0) || !std::isfinite(sensor.timeout_sec)) {
            return fail("sensor '" + sensor.name + "': timeout_sec must be positive and finite");
        }
        min_timeout = std::min(min_timeout, sensor.timeout_sec);
    }

    for (std::size_t i = 0; i < config.sensors.size(); ++i) {
        for (std::size_t j = i + 1; j < config.sensors.size(); ++j) {
            if (config.sensors[i].name == config.sensors[j].name) {
                return fail("duplicate sensor name: " + config.sensors[i].name);
            }
        }
    }

    if (min_timeout < std::numeric_limits<double>::max() && config.check_period_sec > min_timeout / 2.0) {
        return fail("check_period_sec must be <= min(timeout_sec) / 2");
    }
    return true;
}

bool LoadSensorMonitorConfig(rclcpp::Node* node, SensorMonitor::Config& config, std::string& err) {
    const std::string prefix = "sensor_monitor.";

    config.enable = node->declare_parameter(prefix + "enable", config.enable);
    if (!config.enable) {
        RCLCPP_INFO(node->get_logger(), "sensor_monitor.enable=false — monitor not started");
        return true;
    }

    config.check_period_sec = node->declare_parameter(prefix + "check_period_sec", config.check_period_sec);
    config.startup_grace_sec = node->declare_parameter(prefix + "startup_grace_sec", config.startup_grace_sec);
    config.recover_hold_sec = node->declare_parameter(prefix + "recover_hold_sec", config.recover_hold_sec);
    config.cancel_repeat_sec = node->declare_parameter(prefix + "cancel_repeat_sec", config.cancel_repeat_sec);
    config.estop_release_grace_sec =
        node->declare_parameter(prefix + "estop_release_grace_sec", config.estop_release_grace_sec);
    config.stats_window_sec = node->declare_parameter(prefix + "stats_window_sec", config.stats_window_sec);

    const auto names =
        node->declare_parameter<std::vector<std::string>>(prefix + "sensors", std::vector<std::string>{});
    config.sensors.clear();
    for (const auto& name : names) {
        SensorMonitor::SensorConfig sensor;
        sensor.name = name;
        const std::string sensor_prefix = prefix + name + ".";
        sensor.topic = node->declare_parameter(sensor_prefix + "topic", std::string{});
        sensor.type = node->declare_parameter(sensor_prefix + "type", std::string{});
        sensor.timeout_sec = node->declare_parameter(sensor_prefix + "timeout_sec", 1.0);
        sensor.required = node->declare_parameter(sensor_prefix + "required", true);
        sensor.suppress_on_estop = node->declare_parameter(sensor_prefix + "suppress_on_estop", false);
        config.sensors.push_back(std::move(sensor));
    }

    if (config.sensors.empty()) {
        err = "sensor_monitor.sensors must not be empty when enabled";
        return false;
    }
    return ValidateSensorMonitorConfig(config, err);
}

}  // namespace command_board
