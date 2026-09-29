#include "auto_charging_bt/auto_charging_bt.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include "auto_charging_bt/reverse_docking_bt_controller.hpp"

// 充电节点实例由行为树黑板传递，各行为节点共用同一实例。

// ==================== AutoChargingNode 实现 ====================
AutoChargingNode::AutoChargingNode() : Node("AutoChargingNode") {
    this->declare_parameter<double>("charging.navigation_backend_wait_timeout_sec", 5.0);
    this->declare_parameter<double>("charging.map_state_wait_timeout_sec", 5.0);
    this->declare_parameter<double>("charging.qr_detection_timeout_sec", 5.0);
    this->declare_parameter<double>("charging.dock_timeout_sec", 60.0);

    // 末端压入参数
    this->declare_parameter<double>("backward.initial_speed", -0.01);
    this->declare_parameter<double>("backward.max_distance", 0.015);
    this->declare_parameter<double>("backward.max_duration", 2.0);
    this->declare_parameter<double>("backward.control_rate", 50.0);
    this->declare_parameter<double>("backward.max_dt", 0.1);

    // 退桩参数
    this->declare_parameter<double>("undocking.speed", -0.08);
    this->declare_parameter<double>("undocking.duration", 3.0);

    // 行为树入口状态确认参数
    this->declare_parameter<double>("entry_state.max_duration", 0.5);
    this->declare_parameter<double>("entry_state.sample_interval", 0.1);
    this->declare_parameter<int>("entry_state.required_stable_count", 3);
    this->declare_parameter<int>("check_charging.required_stable_count", 10);

    // 充电确认后的短距离前进，用于减小机器人继续顶住充电桩的机械压力
    this->declare_parameter<double>("pressure_relief.speed", 0.01);
    this->declare_parameter<double>("pressure_relief.duration", 0.5);
    this->declare_parameter<double>("pressure_relief.control_rate", 50.0);

    // 倒车对接参数
    this->declare_parameter<std::string>("dock.reverse.base_frame", "base_link");
    this->declare_parameter<std::string>("dock.reverse.target_base_frame", "reverse_docking_target");
    this->declare_parameter<std::string>("dock.reverse.static_target_frame", "reverse_docking_static_target");
    this->declare_parameter<std::string>("dock.reverse.target_topic", "apriltag_target_pose");
    this->declare_parameter<std::string>("dock.reverse.fixed_frame", "odom");
    this->declare_parameter<double>("dock.reverse.initial_sampling_duration", 1.0);
    this->declare_parameter<double>("dock.reverse.second_sampling_duration", 2.0);
    this->declare_parameter<double>("dock.reverse.final_sampling_duration", 1.0);
    this->declare_parameter<double>("dock.reverse.final_resampling_duration", 2.0);
    this->declare_parameter<double>("dock.reverse.max_sample_age", 1.5);
    this->declare_parameter<double>("dock.reverse.initial_sampling_max_distance", 0.0);
    this->declare_parameter<double>("dock.reverse.second_sampling_max_distance", 0.20);
    this->declare_parameter<double>("dock.reverse.final_sampling_max_distance", 0.08);
    this->declare_parameter<int>("dock.reverse.min_valid_sample_count", 5);
    this->declare_parameter<int>("dock.reverse.initial_min_valid_sample_count", 5);
    this->declare_parameter<int>("dock.reverse.second_min_valid_sample_count", 10);
    this->declare_parameter<int>("dock.reverse.final_min_valid_sample_count", 5);
    this->declare_parameter<int>("dock.reverse.final_resample_min_valid_sample_count", 10);
    this->declare_parameter<double>("dock.reverse.sample_trim_ratio", 0.20);
    this->declare_parameter<int>("dock.reverse.final_resample_max_attempts", 3);
    this->declare_parameter<double>("dock.reverse.final_resample_linear_margin", 0.020);
    this->declare_parameter<double>("dock.reverse.final_resample_lateral_margin", 0.015);
    this->declare_parameter<double>("dock.reverse.final_resample_yaw_margin", 0.035);
    this->declare_parameter<double>("dock.reverse.target_lock_duration", 2.0);
    this->declare_parameter<double>("dock.reverse.refinement_lock_duration", 1.0);
    this->declare_parameter<double>("dock.reverse.first_stage_linear_tolerance", 0.10);
    this->declare_parameter<double>("dock.reverse.first_stage_lateral_tolerance", 0.08);
    this->declare_parameter<double>("dock.reverse.first_stage_final_yaw_tolerance", 0.05);
    this->declare_parameter<double>("dock.reverse.second_stage_linear_tolerance", 0.040);
    this->declare_parameter<double>("dock.reverse.second_stage_lateral_tolerance", 0.020);
    this->declare_parameter<double>("dock.reverse.second_stage_final_yaw_tolerance", 0.020);
    this->declare_parameter<double>("dock.reverse.final_stage_linear_tolerance", 0.020);
    this->declare_parameter<double>("dock.reverse.final_stage_lateral_tolerance", 0.020);
    this->declare_parameter<double>("dock.reverse.final_stage_final_yaw_tolerance", 0.020);
    this->declare_parameter<double>("dock.reverse.second_stage_linear_scale", 0.7);
    this->declare_parameter<double>("dock.reverse.second_stage_angular_scale", 0.8);
    this->declare_parameter<double>("dock.reverse.second_stage_max_linear_speed", 0.10);
    this->declare_parameter<double>("dock.reverse.second_stage_max_angular_speed", 0.25);
    this->declare_parameter<double>("dock.reverse.final_stage_linear_scale", 0.5);
    this->declare_parameter<double>("dock.reverse.final_stage_angular_scale", 0.7);
    this->declare_parameter<double>("dock.reverse.final_stage_max_linear_speed", 0.08);
    this->declare_parameter<double>("dock.reverse.final_stage_max_angular_speed", 0.20);
    this->declare_parameter<double>("dock.reverse.control_rate", 20.0);
    this->declare_parameter<double>("dock.reverse.linear_gain", 0.35);
    this->declare_parameter<double>("dock.reverse.heading_gain", 1.20);
    this->declare_parameter<double>("dock.reverse.final_yaw_gain", 0.90);
    this->declare_parameter<double>("dock.reverse.final_yaw_i_gain", 0.0);
    this->declare_parameter<double>("dock.reverse.final_yaw_d_gain", 0.05);
    this->declare_parameter<double>("dock.reverse.final_yaw_i_limit", 0.30);
    this->declare_parameter<double>("dock.reverse.final_yaw_offset", 0.0);
    this->declare_parameter<double>("dock.reverse.linear_tolerance", 0.020);
    this->declare_parameter<double>("dock.reverse.lateral_tolerance", 0.020);
    this->declare_parameter<double>("dock.reverse.heading_tolerance", 0.06);
    this->declare_parameter<double>("dock.reverse.final_yaw_tolerance", 0.02);
    this->declare_parameter<double>("dock.reverse.approach_distance", 0.25);
    this->declare_parameter<double>("dock.reverse.large_heading_threshold", 0.35);
    this->declare_parameter<double>("dock.reverse.max_linear_speed", 0.18);
    this->declare_parameter<double>("dock.reverse.min_linear_speed", 0.015);
    this->declare_parameter<double>("dock.reverse.max_angular_speed", 0.45);
    this->declare_parameter<double>("dock.reverse.min_angular_speed", 0.015);
    this->declare_parameter<double>("dock.reverse.final_max_angular_speed", 0.20);
    this->declare_parameter<double>("dock.reverse.final_min_angular_speed", 0.0);
    this->declare_parameter<double>("dock.reverse.max_linear_accel", 0.20);
    this->declare_parameter<double>("dock.reverse.max_angular_accel", 0.80);

    // 导航客户端
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

    // 速度发布器
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("control/cmd_vel_nav", 10);

    // 诊断数据订阅器
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    sub_diagnostics_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics_agg", qos, std::bind(&AutoChargingNode::cbDiagnostics, this, std::placeholders::_1));

    // 导航后端是回充流程选择的唯一真值源，使用 latched QoS 获取 command_board 的当前值。
    navigation_backend_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/command_board/navigation_backend",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&AutoChargingNode::cbNavigationBackend, this, std::placeholders::_1));

    // AprilTag检测服务客户端
    start_detection_client_ = this->create_client<std_srvs::srv::Trigger>("start_apriltag_detection");
    stop_detection_client_ = this->create_client<std_srvs::srv::Trigger>("stop_apriltag_detection");
}

void AutoChargingNode::cbNavigationBackend(const std_msgs::msg::String::SharedPtr msg) {
    const bool changed = navigation_backend_name_ != msg->data;
    navigation_backend_name_ = msg->data;
    if (changed) {
        ++navigation_backend_revision_;
    }
}

AutoChargingNode::NavigationBackendSnapshot AutoChargingNode::waitForNavigationBackend(
    double timeout_sec) {
    if (!std::isfinite(timeout_sec) || timeout_sec <= 0.0) {
        throw std::runtime_error(
            "Navigation backend unavailable; charging.navigation_backend_wait_timeout_sec must be positive.");
    }

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(timeout_sec);

    // 节点仅在回充任务中主动 spin；开始新任务时先处理队列中的 latched/切换消息，
    // 避免沿用上一次任务缓存的旧后端。
    rclcpp::spin_some(this->shared_from_this());
    while (rclcpp::ok() && navigation_backend_name_.empty() &&
           std::chrono::steady_clock::now() < deadline) {
        rclcpp::sleep_for(std::chrono::milliseconds(50));
        rclcpp::spin_some(this->shared_from_this());
    }

    if (navigation_backend_name_.empty()) {
        std::ostringstream message;
        message << "Navigation backend unavailable; topic=/command_board/navigation_backend, "
                << "timeout=" << timeout_sec << " s.";
        throw std::runtime_error(message.str());
    }
    if (navigation_backend_name_ != kNavigationBackendNav2 &&
        navigation_backend_name_ != kNavigationBackendPathNavigation) {
        throw std::runtime_error(
            "Unsupported navigation backend; value='" + navigation_backend_name_ +
            "', expected nav2 or path_navigation.");
    }

    return {navigation_backend_name_, navigation_backend_revision_};
}

bool AutoChargingNode::navigationBackendChanged(
    const NavigationBackendSnapshot& expected) const {
    // revision 只记录状态值实际变化，因此既忽略同值重复发布，也能识别切换后又切回的情况。
    return navigation_backend_revision_ != expected.revision ||
           navigation_backend_name_ != expected.name;
}

void AutoChargingNode::setTaskInterruptionCheck(std::function<bool()> check) {
    task_interruption_check_ = std::move(check);
}

void AutoChargingNode::clearTaskInterruptionCheck() {
    task_interruption_check_ = {};
}

bool AutoChargingNode::taskInterruptionRequested() const {
    return task_interruption_check_ && task_interruption_check_();
}

void AutoChargingNode::cbDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
    bool charging_key_found = false;
    bool status_powerboard_name = false;
    // 遍历所有诊断状态
    for (const auto& status : msg->status) {
        // 找到电源板相关的诊断项
        if (status.name == "/Devices/System/PowerBoard/collector_node: PowerBoard") {
            status_powerboard_name = true;
            // 遍历键值对查找充电状态
            for (const auto& value : status.values) {
                if (value.key == "battery_charging") {
                    // 将字符串'true'/'false'转换为布尔值
                    is_charging_ = (value.value == "true");
                    charging_key_found = true;
                } else if (value.key == "battery_percentage") {
                    try {
                        battery_percentage_ = std::stod(value.value);
                    } catch (const std::exception& e) {
                        battery_percentage_ = -1.0;
                        RCLCPP_DEBUG_THROTTLE(
                            this->get_logger(), *this->get_clock(), 30000,
                            "Battery percentage parsing failed; error=%s.", e.what());
                    }
                }
            }
        }

        if (status_powerboard_name) {
            if (!charging_key_found) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 30000,
                    "Charging state unavailable; PowerBoard diagnostics missing "
                    "battery_charging.");
            }
            return;
        }
    }

    // 如果没有找到电源板诊断项
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
                         "PowerBoard diagnostics unavailable.");
}

AutoChargingNode::ChargerState AutoChargingNode::confirmEntryChargerState() {
    const double max_duration = std::max(0.1, this->get_parameter("entry_state.max_duration").as_double());
    const double sample_interval = std::max(0.02, this->get_parameter("entry_state.sample_interval").as_double());
    const int required_stable_count =
        std::max(1, static_cast<int>(this->get_parameter("entry_state.required_stable_count").as_int()));

    ChargerState last_state = ChargerState::UNKNOWN;
    int stable_count = 0;
    auto start_time = this->now();

    while (rclcpp::ok()) {
        rclcpp::spin_some(this->shared_from_this());

        const bool charging = isCharging();
        ChargerState current_state = charging ? ChargerState::CHARGING : ChargerState::DISCONNECTED;

        if (current_state == last_state) {
            ++stable_count;
        } else {
            last_state = current_state;
            stable_count = 1;
        }

        if (stable_count >= required_stable_count) {
            return current_state;
        }

        const double elapsed = (this->now() - start_time).seconds();
        if (elapsed >= max_duration) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::duration<double>(sample_interval));
    }

    RCLCPP_WARN(this->get_logger(),
                "Entry charging state unstable; timeout=%.2f s, continue docking.",
                max_duration);
    return ChargerState::DISCONNECTED;
}

bool AutoChargingNode::navToPose(const geometry_msgs::msg::PoseStamped& goal_pose) {
    RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 action server.");
    nav_client_->wait_for_action_server();

    nav2_msgs::action::NavigateToPose::Goal goal_msg;
    goal_msg.pose = goal_pose;

    RCLCPP_INFO(this->get_logger(), "Sending Nav2 goal.");
    auto send_goal_future = nav_client_->async_send_goal(goal_msg);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), send_goal_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Nav2 goal sending failed.");
        return false;
    }

    auto goal_handle = send_goal_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Nav2 goal rejected.");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Nav2 goal accepted; navigation started.");
    auto result_future = nav_client_->async_get_result(goal_handle);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Nav2 result unavailable.");
        return false;
    }

    auto result = result_future.get();

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Nav2 navigation succeeded.");
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Nav2 navigation failed; code=%d.",
                     static_cast<int>(result.code));
        return false;
    }
}

// ==================== 异步导航接口（供 NavigateToCharging StatefulActionNode 使用） ====================
// 这三个接口与上面阻塞版 navToPose 并存，目的是让行为树主循环不再被 spin_until_future_complete
// 卡住：onStart 里 sendNavGoal 发出 goal 后立即返回 RUNNING；onRunning 里 isNavDone 非阻塞轮询；
// onHalted 里 cancelNav 取消 Nav2 goal。这样回充取消请求能通过 is_canceling() 检查立即生效。

bool AutoChargingNode::sendNavGoal(const geometry_msgs::msg::PoseStamped& goal_pose) {
    // 复位上一次导航的残留状态
    nav_goal_sent_ = false;
    nav_cancel_state_ = NavCancelState::IDLE;
    nav_cancel_future_ = {};
    nav_goal_handle_ = nullptr;
    nav_result_status_ = kNavStatusRunning;
    nav_result_ready_ = false;
    const std::uint64_t goal_generation = ++nav_goal_generation_;
    nav_send_options_ = std::make_shared<NavClient::SendGoalOptions>();

    // goal_response 回调：记录 goal_handle，若被拒绝则直接置失败。
    nav_send_options_->goal_response_callback =
        [this, goal_generation](std::shared_ptr<NavGoalHandle> handle) {
            if (goal_generation != nav_goal_generation_) {
                return;
            }
            if (!handle) {
                RCLCPP_ERROR(this->get_logger(), "Nav2 goal rejected.");
                nav_result_status_ = kNavStatusFailed;
                nav_result_ready_ = true;
                if (nav_cancel_state_ == NavCancelState::WAITING_FOR_GOAL_RESPONSE) {
                    nav_cancel_state_ = NavCancelState::CONFIRMED;
                }
                return;
            }
            nav_goal_handle_ = handle;
            RCLCPP_INFO(this->get_logger(), "Nav2 goal accepted; navigation started.");
            if (nav_cancel_state_ == NavCancelState::WAITING_FOR_GOAL_RESPONSE) {
                RCLCPP_INFO(this->get_logger(), "Nav2 goal accepted after cancel; canceling it now.");
                requestNavCancellation(handle);
            }
        };

    // result 回调：记录最终状态字符串，置 nav_result_ready_。
    nav_send_options_->result_callback =
        [this, goal_generation](const NavGoalHandle::WrappedResult& result) {
            if (goal_generation != nav_goal_generation_) {
                return;
            }
            nav_result_ready_ = true;
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Nav2 navigation succeeded.");
                nav_result_status_ = kNavStatusSucceeded;
            } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
                RCLCPP_INFO(this->get_logger(), "Nav2 navigation canceled.");
                nav_result_status_ = kNavStatusCancelled;
            } else {
                RCLCPP_ERROR(this->get_logger(),
                             "Nav2 navigation failed; code=%d.",
                             static_cast<int>(result.code));
                nav_result_status_ = kNavStatusFailed;
            }
            nav_goal_handle_ = nullptr;
            if (nav_cancel_state_ == NavCancelState::WAITING_FOR_GOAL_RESPONSE ||
                nav_cancel_state_ == NavCancelState::WAITING_FOR_CANCEL_RESPONSE ||
                nav_cancel_state_ == NavCancelState::WAITING_FOR_TERMINAL_RESULT) {
                nav_cancel_state_ = NavCancelState::CONFIRMED;
            }
        };

    if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Nav2 action server unavailable; timeout=5.0 s.");
        return false;
    }

    nav2_msgs::action::NavigateToPose::Goal goal_msg;
    goal_msg.pose = goal_pose;
    nav_client_->async_send_goal(goal_msg, *nav_send_options_);
    nav_goal_sent_ = true;
    RCLCPP_INFO(this->get_logger(), "Nav2 goal sent.");
    return true;
}

std::string AutoChargingNode::isNavDone() {
    // 没发过 goal 直接当作失败，避免 onRunning 在 onStart 失败后被无意义调用。
    if (!nav_goal_sent_) {
        return kNavStatusFailed;
    }
    return nav_result_ready_ ? nav_result_status_ : kNavStatusRunning;
}

void AutoChargingNode::cancelNav() {
    if (!nav_goal_sent_) {
        return;
    }
    if (nav_cancel_state_ == NavCancelState::WAITING_FOR_GOAL_RESPONSE ||
        nav_cancel_state_ == NavCancelState::WAITING_FOR_CANCEL_RESPONSE ||
        nav_cancel_state_ == NavCancelState::WAITING_FOR_TERMINAL_RESULT ||
        nav_cancel_state_ == NavCancelState::CONFIRMED ||
        nav_cancel_state_ == NavCancelState::FAILED) {
        return;
    }
    if (nav_result_ready_ && !nav_goal_handle_) {
        nav_cancel_state_ = NavCancelState::CONFIRMED;
        return;
    }
    if (nav_goal_handle_) {
        RCLCPP_INFO(this->get_logger(), "Nav2 goal cancel requested; behavior halted.");
        requestNavCancellation(nav_goal_handle_);
    } else {
        // 句柄返回前不能定向取消；任务退出前会继续 spin，拿到本次句柄后再取消。
        nav_cancel_state_ = NavCancelState::WAITING_FOR_GOAL_RESPONSE;
        RCLCPP_INFO(this->get_logger(), "Nav2 goal handle pending; waiting to cancel this goal.");
    }
    nav_result_status_ = kNavStatusCancelled;
    nav_result_ready_ = true;
}

void AutoChargingNode::requestNavCancellation(const std::shared_ptr<NavGoalHandle>& goal_handle) {
    try {
        nav_cancel_future_ = nav_client_->async_cancel_goal(goal_handle);
        nav_cancel_state_ = NavCancelState::WAITING_FOR_CANCEL_RESPONSE;
    } catch (const std::exception& error) {
        nav_cancel_state_ = NavCancelState::FAILED;
        RCLCPP_ERROR(this->get_logger(), "Nav2 goal cancel request failed; error=%s.",
                     error.what());
    }
}

bool AutoChargingNode::waitForNavCancellation(double timeout_sec) {
    if (nav_cancel_state_ == NavCancelState::IDLE ||
        nav_cancel_state_ == NavCancelState::CONFIRMED) {
        return true;
    }
    if (nav_cancel_state_ == NavCancelState::FAILED ||
        !std::isfinite(timeout_sec) || timeout_sec <= 0.0) {
        return false;
    }

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(timeout_sec);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(this->shared_from_this());

        if (nav_cancel_state_ == NavCancelState::CONFIRMED) {
            return true;
        }
        if (nav_cancel_state_ == NavCancelState::FAILED) {
            return false;
        }
        if (nav_cancel_state_ == NavCancelState::WAITING_FOR_CANCEL_RESPONSE &&
            nav_cancel_future_.valid() &&
            nav_cancel_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            const auto response = nav_cancel_future_.get();
            nav_cancel_future_ = {};
            if (response && response->return_code == NavCancelResponse::ERROR_NONE) {
                // CancelGoal 成功只表示服务端接受了请求；继续等待 result_callback
                // 收到终态，避免目标仍处于 CANCELING 时提前结束回充任务。
                nav_cancel_state_ = NavCancelState::WAITING_FOR_TERMINAL_RESULT;
                RCLCPP_INFO(this->get_logger(),
                            "Nav2 goal cancel accepted; waiting for terminal result.");
                continue;
            }
            if (response &&
                (response->return_code == NavCancelResponse::ERROR_UNKNOWN_GOAL_ID ||
                 response->return_code == NavCancelResponse::ERROR_GOAL_TERMINATED)) {
                nav_cancel_state_ = NavCancelState::CONFIRMED;
                RCLCPP_INFO(this->get_logger(), "Nav2 goal is already terminal.");
                return true;
            }

            nav_cancel_state_ = NavCancelState::FAILED;
            const int return_code = response ? response->return_code : -1;
            RCLCPP_ERROR(this->get_logger(), "Nav2 goal cancel rejected; code=%d.", return_code);
            return false;
        }

        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }

    nav_cancel_state_ = NavCancelState::FAILED;
    RCLCPP_ERROR(this->get_logger(), "Nav2 goal cancel confirmation timed out; timeout=%.1f s.",
                 timeout_sec);
    return false;
}

void AutoChargingNode::publishTwist(double linear_x, double angular_z) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear_x;
    twist.angular.z = angular_z;
    cmd_pub_->publish(twist);
}

bool AutoChargingNode::backward() {
    // 从参数服务器获取配置参数
    const double initial_speed = this->get_parameter("backward.initial_speed").as_double();
    const double max_distance = this->get_parameter("backward.max_distance").as_double();
    const double max_backward_duration = this->get_parameter("backward.max_duration").as_double();
    const double control_rate = this->get_parameter("backward.control_rate").as_double();
    const double max_dt = this->get_parameter("backward.max_dt").as_double();

    RCLCPP_INFO(this->get_logger(),
                "Final reverse started; speed=%.3f m/s, max_distance=%.3f m, "
                "max_duration=%.1f s.",
                initial_speed, max_distance, max_backward_duration);

    double current_speed = initial_speed;
    double total_distance = 0.0;
    double total_time = 0.0;
    auto start_time = this->now();  // 后退开始时间
    auto last_time = start_time;
    bool last_charging = false;
    bool task_interrupted = false;
    const char* stop_reason = "ROS shutdown";

    rclcpp::Rate rate(control_rate);

    while (rclcpp::ok()) {
        // 阻塞运动期间主动派发监控回调，并在每次速度输出前检查任务是否应终止。
        rclcpp::spin_some(this->shared_from_this());
        if (taskInterruptionRequested()) {
            task_interrupted = true;
            stop_reason = "task interrupted";
            break;
        }

        auto current_time = this->now();
        double dt = (current_time - last_time).seconds();

        // 防止异常dt值
        if (dt > max_dt) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 3000,
                "Final reverse time step limited; actual=%.3f s, limit=%.3f s.", dt,
                max_dt);
            dt = max_dt;
        }

        last_time = current_time;
        total_time = (current_time - start_time).seconds();
        total_distance += std::abs(current_speed * dt);

        // 检查最大距离
        if (total_distance >= max_distance) {
            stop_reason = "maximum distance reached";
            break;
        }

        // 检查后退总时长
        if (total_time >= max_backward_duration) {
            stop_reason = "maximum duration reached";
            break;
        }

        // 获取充电状态
        bool is_charging = isCharging();
        last_charging = is_charging;

        if (is_charging) {
            stop_reason = "charging detected";
            break;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Final reverse running; charging=false, speed=%.3f m/s, "
                             "distance=%.3f/%.3f m, elapsed=%.2f/%.2f s.",
                             current_speed, total_distance, max_distance, total_time,
                             max_backward_duration);

        // 发布速度
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = current_speed;
        twist.angular.z = 0.0;
        cmd_pub_->publish(twist);

        rate.sleep();
    }

    cmd_pub_->publish(geometry_msgs::msg::Twist());
    if (last_charging) {
        RCLCPP_INFO(this->get_logger(),
                    "Final reverse stopped; charging detected, distance=%.3f m, elapsed=%.2f s.",
                    total_distance, total_time);
    } else {
        RCLCPP_WARN(this->get_logger(),
                    "Final reverse stopped; %s, distance=%.3f m, elapsed=%.2f s.",
                    stop_reason, total_distance, total_time);
    }
    return !task_interrupted && rclcpp::ok();
}

bool AutoChargingNode::relieveDockingPressure() {
    const double speed = this->get_parameter("pressure_relief.speed").as_double();
    const double duration = this->get_parameter("pressure_relief.duration").as_double();
    const double control_rate = this->get_parameter("pressure_relief.control_rate").as_double();

    if (duration <= 0.0 || control_rate <= 0.0) {
        RCLCPP_WARN(this->get_logger(),
                    "Pressure relief skipped; duration=%.3f s, control_rate=%.3f Hz.",
                    duration, control_rate);
        publishTwist(0.0, 0.0);
        return true;
    }

    RCLCPP_INFO(this->get_logger(),
                "Pressure relief started; speed=%.3f m/s, duration=%.3f s.", speed,
                duration);

    const auto start_time = this->now();
    rclcpp::Rate rate(control_rate);
    bool task_interrupted = false;
    while (rclcpp::ok() && (this->now() - start_time).seconds() < duration) {
        rclcpp::spin_some(this->shared_from_this());
        if (taskInterruptionRequested()) {
            task_interrupted = true;
            RCLCPP_WARN(this->get_logger(), "Pressure relief stopped; task interrupted.");
            break;
        }
        publishTwist(speed, 0.0);
        rate.sleep();
    }

    publishTwist(0.0, 0.0);
    if (task_interrupted) {
        return false;
    }
    if (!rclcpp::ok()) {
        RCLCPP_WARN(this->get_logger(), "Pressure relief stopped; ROS is shutting down.");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Pressure relief complete; robot stopped.");
    return true;
}

bool AutoChargingNode::undocking(double duration) {
    double speed;

    this->get_parameter("undocking.speed", speed);
    RCLCPP_INFO(this->get_logger(), "Undocking started; speed=%.2f m/s, duration=%.1f s.",
                speed, duration);

    auto start_time = this->now();
    rclcpp::Rate rate(50);
    bool task_interrupted = false;

    while (rclcpp::ok()) {
        rclcpp::spin_some(this->shared_from_this());
        if (taskInterruptionRequested()) {
            task_interrupted = true;
            RCLCPP_WARN(this->get_logger(), "Undocking stopped; task interrupted.");
            break;
        }

        auto elapsed = (this->now() - start_time).seconds();
        if (elapsed >= duration) break;

        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = speed;
        twist.angular.z = 0.0;
        cmd_pub_->publish(twist);

        rate.sleep();
    }

    cmd_pub_->publish(geometry_msgs::msg::Twist());
    if (task_interrupted) {
        return false;
    }
    if (!rclcpp::ok()) {
        RCLCPP_WARN(this->get_logger(), "Undocking stopped; ROS is shutting down.");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Undocking complete; robot stopped.");
    return true;
}

bool AutoChargingNode::startDetection() {
    if (apriltag_detection_running_) {
        return true;
    }

    if (!start_detection_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(),
                     "AprilTag detection start failed; type=DETECTION_START_FAILED, "
                     "service unavailable.");
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = start_detection_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(),
                     "AprilTag detection start failed; type=DETECTION_START_FAILED, "
                     "request did not complete.");
        return false;
    }

    auto response = future.get();
    if (response->success) {
        apriltag_detection_running_ = true;
        RCLCPP_INFO(this->get_logger(), "AprilTag detection started.");
        return true;
    } else {
        std::string message_lower = response->message;
        std::transform(message_lower.begin(), message_lower.end(), message_lower.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        if (message_lower.find("already") != std::string::npos ||
            response->message.find("已") != std::string::npos) {
            apriltag_detection_running_ = true;
            return true;
        }
        RCLCPP_ERROR(this->get_logger(),
                     "AprilTag detection start failed; type=DETECTION_START_FAILED, message=%s.",
                     response->message.c_str());
        return false;
    }
}

bool AutoChargingNode::stopDetection() {
    if (!apriltag_detection_running_) {
        return true;
    }

    if (!stop_detection_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(),
                     "AprilTag detection stop failed; service unavailable.");
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = stop_detection_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(),
                     "AprilTag detection stop failed; request did not complete.");
        return false;
    }

    auto response = future.get();
    if (response->success) {
        apriltag_detection_running_ = false;
        RCLCPP_INFO(this->get_logger(), "AprilTag detection stopped.");
        return true;
    } else {
        std::string message_lower = response->message;
        std::transform(message_lower.begin(), message_lower.end(), message_lower.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        if (message_lower.find("already") != std::string::npos ||
            message_lower.find("not running") != std::string::npos ||
            response->message.find("已") != std::string::npos ||
            response->message.find("未启动") != std::string::npos) {
            apriltag_detection_running_ = false;
            return true;
        }
        RCLCPP_ERROR(this->get_logger(), "AprilTag detection stop failed; message=%s.",
                     response->message.c_str());
        return false;
    }
}

// ==================== 地图回充点读取 ====================
ChargingPointAsset readChargingPointFromCurrentMap(
    const std::shared_ptr<AutoChargingNode>& charging_node, double wait_timeout_sec) {
    if (!charging_node) {
        throw std::runtime_error("Charging point loading failed; AutoChargingNode is null.");
    }
    if (!std::isfinite(wait_timeout_sec) || wait_timeout_sec <= 0.0) {
        throw std::runtime_error(
            "Charging point loading failed; charging.map_state_wait_timeout_sec must be positive.");
    }

    nav_interfaces::msg::MapState::SharedPtr current_map_state;
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    auto subscription = charging_node->create_subscription<nav_interfaces::msg::MapState>(
        "/map_manager/map_state", qos,
        [&current_map_state](const nav_interfaces::msg::MapState::SharedPtr msg) {
            current_map_state = msg;
        });
    (void)subscription;  // 保持订阅对象存活到等待结束。

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(wait_timeout_sec);
    while (rclcpp::ok() && !current_map_state && std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(charging_node);
        rclcpp::sleep_for(std::chrono::milliseconds(50));
    }

    if (!current_map_state) {
        std::ostringstream message;
        message << "Charging point loading failed; topic=/map_manager/map_state, timeout="
                << wait_timeout_sec << " s.";
        throw std::runtime_error(message.str());
    }
    if (current_map_state->epoch == 0 || current_map_state->map_folder.empty() ||
        current_map_state->map_name.empty()) {
        throw std::runtime_error("Charging point loading failed; map state is invalid.");
    }

    const std::filesystem::path assets_file =
        std::filesystem::path(current_map_state->map_folder) /
        current_map_state->map_name / "assets" / "map_assets.json";
    std::error_code error;
    if (!std::filesystem::is_regular_file(assets_file, error)) {
        throw std::runtime_error("Charging point loading failed; map asset file unavailable: " +
                                 assets_file.string());
    }

    std::ifstream file(assets_file);
    if (!file.is_open()) {
        throw std::runtime_error("Charging point loading failed; cannot open map asset file: " +
                                 assets_file.string());
    }

    nlohmann::json assets;
    try {
        file >> assets;
    } catch (const std::exception& error) {
        throw std::runtime_error("Charging point loading failed; invalid map asset file: " +
                                 assets_file.string() + "; " + error.what());
    }
    if (!assets.is_object() || !assets.contains("points") || !assets["points"].is_array()) {
        throw std::runtime_error("Charging point loading failed; points array missing: " +
                                 assets_file.string());
    }

    std::vector<std::pair<std::string, nlohmann::json>> charging_points;
    for (const auto& item : assets["points"]) {
        if (!item.is_object() || !item.contains("name") || !item["name"].is_string() ||
            !item.contains("type") || !item["type"].is_string()) {
            throw std::runtime_error(
                "Charging point loading failed; map point missing name or type.");
        }
        if (item["type"].get<std::string>() == "CHARGING") {
            const auto name = item["name"].get<std::string>();
            if (name.empty()) {
                throw std::runtime_error(
                    "Charging point loading failed; CHARGING point name is empty.");
            }
            charging_points.emplace_back(name, item);
        }
    }

    if (charging_points.empty()) {
        throw std::runtime_error(
            "Charging point loading failed; no type=CHARGING point in map_assets.json.");
    }
    if (charging_points.size() > 1) {
        std::ostringstream message;
        message << "Charging point loading failed; multiple CHARGING points: ";
        for (std::size_t i = 0; i < charging_points.size(); ++i) {
            if (i > 0) message << ", ";
            message << charging_points[i].first;
        }
        throw std::runtime_error(message.str());
    }

    const auto& point = charging_points.front().second;
    if (!point.contains("pose") || !point["pose"].is_object() ||
        !point["pose"].contains("x") || !point["pose"]["x"].is_number() ||
        !point["pose"].contains("y") || !point["pose"]["y"].is_number() ||
        !point["pose"].contains("yaw") || !point["pose"]["yaw"].is_number()) {
        throw std::runtime_error(
            "Charging point loading failed; invalid pose for CHARGING point: " +
            charging_points.front().first);
    }

    const double x = point["pose"]["x"].get<double>();
    const double y = point["pose"]["y"].get<double>();
    const double yaw = point["pose"]["yaw"].get<double>();
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
        throw std::runtime_error(
            "Charging point loading failed; non-finite pose for CHARGING point: " +
            charging_points.front().first);
    }

    ChargingPointAsset result;
    result.name = charging_points.front().first;
    result.assets_file = assets_file.lexically_normal().string();
    result.map_folder = current_map_state->map_folder;
    result.map_name = current_map_state->map_name;
    result.map_epoch = current_map_state->epoch;
    result.pose.header.frame_id = "map";
    result.pose.pose.position.x = x;
    result.pose.pose.position.y = y;
    result.pose.pose.orientation.z = std::sin(yaw * 0.5);
    result.pose.pose.orientation.w = std::cos(yaw * 0.5);
    return result;
}

// ==================== NavigateToCharging ====================
// 改造为 StatefulActionNode：onStart 异步发 Nav2 goal 后立即返回 RUNNING，
// onRunning 非阻塞轮询导航结果，onHalted 取消 Nav2 goal。
// 这样行为树主循环每 tick 只执行一次轻量轮询，is_canceling() 检查不再被
// spin_until_future_complete 阻塞，回充取消请求能立即生效。
NavigateToCharging::NavigateToCharging(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList NavigateToCharging::providedPorts() {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")};
}

BT::NodeStatus NavigateToCharging::onStart() {
    auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
    if (!goal) {
        throw BT::RuntimeError("Missing goal pose");
    }

    auto charging_node = config().blackboard->get<std::shared_ptr<AutoChargingNode>>("charging_node");
    if (!charging_node) {
        throw BT::RuntimeError("charging_node not set in blackboard");
    }
    charging_node_ = charging_node;

    if (charging_node_->taskInterruptionRequested()) {
        charging_node_->publishTwist(0.0, 0.0);
        RCLCPP_WARN(charging_node_->get_logger(),
                    "Navigate to charging paused; task interruption requested.");
        return BT::NodeStatus::RUNNING;
    }

    auto goal_pose = goal.value();
    goal_pose.header.stamp = charging_node_->now();
    if (!charging_node_->sendNavGoal(goal_pose)) {
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToCharging::onRunning() {
    const std::string status = charging_node_->isNavDone();
    if (status == AutoChargingNode::kNavStatusRunning) {
        return BT::NodeStatus::RUNNING;     // 仍在导航中
    }
    if (status == AutoChargingNode::kNavStatusSucceeded) {
        return BT::NodeStatus::SUCCESS;     // Nav2 返回 SUCCEEDED
    }
    return BT::NodeStatus::FAILURE;          // FAILED / CANCELLED / 其他
}

void NavigateToCharging::onHalted() {
    if (charging_node_) {
        charging_node_->cancelNav();
    }
}

// ==================== Dock ====================
Dock::Dock(const std::string& name, const BT::NodeConfiguration& config) : BT::StatefulActionNode(name, config) {}

Dock::~Dock() = default;

BT::PortsList Dock::providedPorts() {
    return {BT::InputPort<double>("timeout", 120.0, "超时时间(秒)"),
            BT::InputPort<double>("qr_detection_timeout", 5.0,
                                  "停车采样阶段连续无有效二维码目标超时时间(秒)"),
            BT::OutputPort<int>("result", "执行结果 0=成功 1=失败")};
}

BT::NodeStatus Dock::onStart() {
    auto charging_node = config().blackboard->get<std::shared_ptr<AutoChargingNode>>("charging_node");
    if (!charging_node) {
        throw BT::RuntimeError("charging_node not set in blackboard");
    }
    charging_node_ = charging_node;

    if (charging_node_->taskInterruptionRequested()) {
        charging_node_->publishTwist(0.0, 0.0);
        RCLCPP_WARN(charging_node_->get_logger(),
                    "Dock start paused; task interruption requested.");
        return BT::NodeStatus::RUNNING;
    }

    // 读取超时参数
    if (auto t = getInput<double>("timeout")) timeout_ = t.value();
    if (auto t = getInput<double>("qr_detection_timeout")) qr_detection_timeout_ = t.value();

    if (!std::isfinite(timeout_) || timeout_ <= 0.0) {
        RCLCPP_ERROR(charging_node_->get_logger(),
                     "Docking failed; type=INVALID_DOCK_TIMEOUT, timeout=%.3f s.", timeout_);
        return BT::NodeStatus::FAILURE;
    }
    if (!std::isfinite(qr_detection_timeout_) || qr_detection_timeout_ <= 0.0) {
        RCLCPP_ERROR(charging_node_->get_logger(),
                     "Docking failed; type=INVALID_QR_DETECTION_TIMEOUT, timeout=%.3f s.",
                     qr_detection_timeout_);
        return BT::NodeStatus::FAILURE;
    }
    if (qr_detection_timeout_ > timeout_) {
        RCLCPP_ERROR(charging_node_->get_logger(),
                     "Docking failed; type=INVALID_TIMEOUT_ORDER, detection_timeout=%.3f s, "
                     "dock_timeout=%.3f s.",
                     qr_detection_timeout_, timeout_);
        return BT::NodeStatus::FAILURE;
    }

    start_time_ = charging_node_->now();
    reverse_docking_ = std::make_unique<ReverseDockingBtController>(charging_node_);
    reverse_docking_->start();
    RCLCPP_INFO(charging_node_->get_logger(), "Docking started; timeout=%.1f s.", timeout_);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Dock::onRunning() {
    // 超时检测
    double elapsed = (charging_node_->now() - start_time_).seconds();

    // 二维码看门狗只覆盖停车采样；运动阶段依赖锁定到 odom 的目标继续控制。
    if (reverse_docking_) {
        const auto no_target_elapsed = reverse_docking_->samplingTargetMissingDuration();
        if (no_target_elapsed && no_target_elapsed.value() > qr_detection_timeout_) {
            RCLCPP_ERROR(charging_node_->get_logger(),
                         "Docking failed; type=QR_DETECTION_TIMEOUT, missing=%.1f s, "
                         "limit=%.1f s.",
                         no_target_elapsed.value(), qr_detection_timeout_);
            reverse_docking_->stop();
            charging_node_->publishTwist(0.0, 0.0);
            setOutput("result", 1);
            return BT::NodeStatus::FAILURE;
        }
    }

    if (elapsed > timeout_) {
        const std::string reason_code = reverse_docking_ ?
            reverse_docking_->failureReasonCode() : "CONTROLLER_NOT_CREATED";
        const std::string reason = reverse_docking_ ?
            reverse_docking_->failureReasonSummary() : "Reverse docking controller unavailable.";
        const std::string details = reverse_docking_ ?
            reverse_docking_->diagnosticSummary() : "controller=unavailable";
        RCLCPP_ERROR(charging_node_->get_logger(),
                     "Docking failed; type=%s, elapsed=%.1f/%.1f s, reason=%s, details=%s.",
                     reason_code.c_str(), elapsed, timeout_, reason.c_str(), details.c_str());
        if (reverse_docking_) reverse_docking_->stop();
        charging_node_->publishTwist(0.0, 0.0);
        setOutput("result", 1);
        return BT::NodeStatus::FAILURE;
    }

    if (reverse_docking_ && reverse_docking_->done()) {
        reverse_docking_->stop();
        charging_node_->publishTwist(0.0, 0.0);
        if (!charging_node_->backward()) {
            setOutput("result", 1);
            return BT::NodeStatus::RUNNING;
        }
        setOutput("result", 0);
        return BT::NodeStatus::SUCCESS;
    }

    // 继续运行
    return BT::NodeStatus::RUNNING;
}

void Dock::onHalted() {
    if (reverse_docking_) reverse_docking_->stop();
    if (charging_node_) charging_node_->publishTwist(0.0, 0.0);
}

// ==================== CheckCharging ====================
CheckCharging::CheckCharging(const std::string& name, const BT::NodeConfiguration& config)
    : StatefulActionNode(name, config) {
    timeout_ = getInput<double>("timeout").value();
}

BT::PortsList CheckCharging::providedPorts() { return {BT::InputPort<double>("timeout")}; }

BT::NodeStatus CheckCharging::onStart() {
    auto charging_node = config().blackboard->get<std::shared_ptr<AutoChargingNode>>("charging_node");
    if (!charging_node) {
        throw BT::RuntimeError("charging_node not set in blackboard");
    }
    charging_node_ = charging_node;  // 保存节点实例，供后续周期回调和停止回调使用。

    start_time_ = charging_node_->now();
    consecutive_charging_ = 0;
    const int required_stable_count =
        std::max(1, static_cast<int>(charging_node_->get_parameter("check_charging.required_stable_count").as_int()));
    rclcpp::spin_some(charging_node_);
    RCLCPP_INFO(charging_node_->get_logger(),
                "Charging check started; timeout=%.1f s, required=%d consecutive samples.",
                timeout_, required_stable_count);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CheckCharging::onRunning() {
    double elapsed = (charging_node_->now() - start_time_).seconds();

    // 处理诊断订阅队列，获取最新充电状态
    rclcpp::spin_some(charging_node_);

    const int required_stable_count =
        std::max(1, static_cast<int>(charging_node_->get_parameter("check_charging.required_stable_count").as_int()));
    const bool charging = charging_node_->isCharging();

    consecutive_charging_ = charging ? (consecutive_charging_ + 1) : 0;

    RCLCPP_INFO_THROTTLE(charging_node_->get_logger(), *charging_node_->get_clock(), 2000,
                         "Charging check running; charging=%s, stable=%d/%d, elapsed=%.1f s.",
                         charging ? "true" : "false",
                         consecutive_charging_, required_stable_count, elapsed);

    if (consecutive_charging_ >= required_stable_count) {
        RCLCPP_INFO(charging_node_->get_logger(),
                    "Charging confirmed; stable=%d/%d.", consecutive_charging_,
                    required_stable_count);
        consecutive_charging_ = 0;
        if (!charging_node_->relieveDockingPressure()) {
            return BT::NodeStatus::RUNNING;
        }
        return BT::NodeStatus::SUCCESS;
    }

    if (elapsed >= timeout_) {
        RCLCPP_ERROR(charging_node_->get_logger(),
                     "Charging check failed; type=CHARGING_NOT_CONFIRMED, timeout=%.1f s, "
                     "charging=%s, stable=%d/%d.",
                     timeout_, charging ? "true" : "false",
                     consecutive_charging_, required_stable_count);
        consecutive_charging_ = 0;
        return BT::NodeStatus::FAILURE;
    }

    // 短暂休眠后继续检测
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return BT::NodeStatus::RUNNING;
}

void CheckCharging::onHalted() {}

// ==================== CheckBatteryAboveThreshold ====================
CheckBatteryAboveThreshold::CheckBatteryAboveThreshold(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList CheckBatteryAboveThreshold::providedPorts() { return {BT::InputPort<double>("threshold")}; }

BT::NodeStatus CheckBatteryAboveThreshold::tick() {
    auto charging_node = config().blackboard->get<std::shared_ptr<AutoChargingNode>>("charging_node");
    if (!charging_node) {
        throw BT::RuntimeError("charging_node not set in blackboard");
    }

    auto threshold = getInput<double>("threshold");
    if (!threshold) {
        RCLCPP_ERROR(charging_node->get_logger(), "Battery check failed; threshold missing.");
        return BT::NodeStatus::FAILURE;
    }

    // 处理诊断订阅队列，获取最新电量
    rclcpp::spin_some(charging_node);

    double battery = charging_node->getBatteryPercentage();
    if (battery < 0.0) {
        RCLCPP_WARN(charging_node->get_logger(), "Battery check skipped; value unavailable.");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(charging_node->get_logger(), "Battery check; value=%.2f, threshold=%.2f.",
                battery, threshold.value());

    return battery >= threshold.value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ==================== IsCharging ====================
IsCharging::IsCharging(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList IsCharging::providedPorts() { return {}; }

BT::NodeStatus IsCharging::tick() {
    auto charging_node = config().blackboard->get<std::shared_ptr<AutoChargingNode>>("charging_node");
    if (!charging_node) {
        throw BT::RuntimeError("charging_node not set in blackboard");
    }

    const auto state = charging_node->confirmEntryChargerState();

    return state == AutoChargingNode::ChargerState::CHARGING ? BT::NodeStatus::SUCCESS
                                                             : BT::NodeStatus::FAILURE;
}

// ==================== IsChargingNavigationEnabled ====================
IsChargingNavigationEnabled::IsChargingNavigationEnabled(
    const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList IsChargingNavigationEnabled::providedPorts() {
    return {BT::InputPort<bool>("enabled")};
}

BT::NodeStatus IsChargingNavigationEnabled::tick() {
    const auto enabled = getInput<bool>("enabled");
    if (!enabled) {
        throw BT::RuntimeError("IsChargingNavigationEnabled missing enabled input");
    }
    return enabled.value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ==================== Undocking ====================
Undocking::Undocking(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode(name, config) {}

// 退桩节点没有额外启动状态，实际动作在运行回调中执行。
BT::PortsList Undocking::providedPorts() { return {BT::InputPort<double>("duration")}; }

BT::NodeStatus Undocking::tick() {
    auto charging_node = config().blackboard->get<std::shared_ptr<AutoChargingNode>>("charging_node");
    if (!charging_node) {
        throw BT::RuntimeError("charging_node not set in blackboard");
    }

    double duration = 2.0;
    if (auto d = getInput<double>("duration")) {
        duration = d.value();
    } else {
        RCLCPP_WARN(charging_node->get_logger(),
                    "Undocking duration missing; default=%.2f s.", duration);
    }

    return charging_node->undocking(duration) ? BT::NodeStatus::SUCCESS
                                               : BT::NodeStatus::FAILURE;
}

// ==================== CounterCheck ====================
CounterCheck::CounterCheck(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config) {
    max_retries_ = getInput<int>("max_retries").value();
}

BT::PortsList CounterCheck::providedPorts() { return {BT::InputPort<int>("max_retries")}; }

BT::NodeStatus CounterCheck::tick() {
    auto charging_node = config().blackboard->get<std::shared_ptr<AutoChargingNode>>("charging_node");
    if (!charging_node) {
        throw BT::RuntimeError("charging_node not set in blackboard");
    }

    auto blackboard = config().blackboard;
    int count = blackboard->get<int>("retry_count");

    if (count >= max_retries_) {
        RCLCPP_ERROR(charging_node->get_logger(), "Retry limit reached; limit=%d.", max_retries_);
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(charging_node->get_logger(), "Retry count; count=%d/%d.", count,
                max_retries_);
    return BT::NodeStatus::SUCCESS;
}

// ==================== IncrementCounter ====================
IncrementCounter::IncrementCounter(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config) {}

BT::NodeStatus IncrementCounter::tick() {
    auto charging_node = config().blackboard->get<std::shared_ptr<AutoChargingNode>>("charging_node");
    if (!charging_node) {
        throw BT::RuntimeError("charging_node not set in blackboard");
    }

    auto blackboard = config().blackboard;
    int count = blackboard->get<int>("retry_count");
    blackboard->set("retry_count", count + 1);
    RCLCPP_INFO(charging_node->get_logger(), "Retry count updated; count=%d->%d.", count,
                count + 1);
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList IncrementCounter::providedPorts() { return {}; }

// ==================== AttemptLogger ====================
AttemptLogger::AttemptLogger(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config) {}

BT::PortsList AttemptLogger::providedPorts() {
    return {BT::InputPort<std::string>("label"), BT::InputPort<std::string>("counter_key"),
            BT::InputPort<int>("total")};
}

BT::NodeStatus AttemptLogger::tick() {
    auto charging_node = config().blackboard->get<std::shared_ptr<AutoChargingNode>>("charging_node");
    if (!charging_node) {
        throw BT::RuntimeError("charging_node not set in blackboard");
    }

    const auto label = getInput<std::string>("label");
    const auto counter_key = getInput<std::string>("counter_key");
    const auto total = getInput<int>("total");
    if (!label || !counter_key || !total) {
        throw BT::RuntimeError("AttemptLogger missing input port");
    }

    int count = 0;
    try {
        count = config().blackboard->get<int>(counter_key.value());
    } catch (const std::exception&) {
        count = 0;
    }

    ++count;
    config().blackboard->set(counter_key.value(), count);
    const bool is_dock_attempt = label.value() == "回充对接";
    const char* attempt_tag = is_dock_attempt ? "[DockAttempt]" : "[NavigationAttempt]";
    const char* retry_tag = is_dock_attempt ? "[DockRetry]" : "[NavigationRetry]";
    if (count == 1) {
        RCLCPP_INFO(charging_node->get_logger(), "%s Started; attempt=%d/%d.",
                    attempt_tag, count, total.value());
    } else {
        RCLCPP_WARN(charging_node->get_logger(),
                    "%s Retrying; attempt=%d/%d, retry=%d.",
                    retry_tag, count, total.value(), count - 1);
    }
    return BT::NodeStatus::SUCCESS;
}

// ==================== StartDetection ====================
StartDetection::StartDetection(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config) {}

BT::PortsList StartDetection::providedPorts() { return {}; }

BT::NodeStatus StartDetection::tick() {
    auto charging_node = config().blackboard->get<std::shared_ptr<AutoChargingNode>>("charging_node");
    if (!charging_node) {
        throw BT::RuntimeError("charging_node not set in blackboard");
    }
    const bool ok = charging_node->startDetection();
    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ==================== StopDetection ====================
StopDetection::StopDetection(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config) {}

BT::PortsList StopDetection::providedPorts() { return {}; }

BT::NodeStatus StopDetection::tick() {
    auto charging_node = config().blackboard->get<std::shared_ptr<AutoChargingNode>>("charging_node");
    if (!charging_node) {
        throw BT::RuntimeError("charging_node not set in blackboard");
    }
    bool ok = charging_node->stopDetection();
    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
