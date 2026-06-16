#include "auto_charging_bt/auto_charging_bt.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <future>
#include <nlohmann/json.hpp>
#include <thread>

#include "auto_charging_bt/reverse_docking_bt_controller.hpp"

// Global charging_node removed. It's now passed via Blackboard.

// ==================== AutoChargingNode 实现 ====================
AutoChargingNode::AutoChargingNode() : Node("AutoChargingNode") {
    // backward 参数
    this->declare_parameter<double>("backward.initial_speed", -0.01);
    this->declare_parameter<double>("backward.connected_speed", -0.005);
    this->declare_parameter<double>("backward.max_distance", 0.015);
    this->declare_parameter<double>("backward.stable_duration", 1.0);
    this->declare_parameter<double>("backward.max_duration", 2.0);
    this->declare_parameter<double>("backward.control_rate", 50.0);
    this->declare_parameter<double>("backward.max_dt", 0.1);

    // undocking 参数
    this->declare_parameter<double>("undocking.speed", -0.08);
    this->declare_parameter<double>("undocking.duration", 3.0);

    // Dock 反向对接参数
    this->declare_parameter<std::string>("dock.reverse.base_frame", "base_link");
    this->declare_parameter<std::string>("dock.reverse.target_base_frame", "reverse_docking_target");
    this->declare_parameter<std::string>("dock.reverse.static_target_frame", "reverse_docking_static_target");
    this->declare_parameter<std::string>("dock.reverse.target_topic", "apriltag_target_pose");
    this->declare_parameter<std::string>("dock.reverse.fixed_frame", "odom");
    this->declare_parameter<double>("dock.reverse.target_lock_duration", 2.0);
    this->declare_parameter<double>("dock.reverse.refinement_lock_duration", 1.0);
    this->declare_parameter<double>("dock.reverse.second_stage_linear_scale", 0.5);
    this->declare_parameter<double>("dock.reverse.second_stage_angular_scale", 0.5);
    this->declare_parameter<double>("dock.reverse.second_stage_max_linear_speed", 0.08);
    this->declare_parameter<double>("dock.reverse.second_stage_max_angular_speed", 0.20);
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
    this->declare_parameter<double>("dock.reverse.final_max_angular_speed", 0.15);
    this->declare_parameter<double>("dock.reverse.final_min_angular_speed", 0.0);
    this->declare_parameter<double>("dock.reverse.max_linear_accel", 0.20);
    this->declare_parameter<double>("dock.reverse.max_angular_accel", 0.60);

    // 导航客户端
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

    // 速度发布器
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("control/cmd_vel_nav", 10);

    // 诊断数据订阅器
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    sub_diagnostics_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics_agg", qos, std::bind(&AutoChargingNode::cbDiagnostics, this, std::placeholders::_1));

    // AprilTag检测服务客户端
    start_detection_client_ = this->create_client<std_srvs::srv::Trigger>("start_apriltag_detection");
    stop_detection_client_ = this->create_client<std_srvs::srv::Trigger>("stop_apriltag_detection");
}

void AutoChargingNode::cbDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
    bool charging_key_found = false;
    bool percentage_key_found = false;
    bool charger_connect_key_found = false;
    bool status_powerboardsafety_name = false;
    bool status_powerboard_name = false;
    // 遍历所有诊断状态
    for (const auto& status : msg->status) {
        if (status.name == "/Devices/System/PowerBoard/collector_node: PowerBoard Safety") {
            status_powerboardsafety_name = true;
            for (const auto& value : status.values) {
                if (value.key == "charger_connect_state") {
                    std::string val_lower = value.value;
                    std::transform(val_lower.begin(), val_lower.end(), val_lower.begin(),
                                   [](unsigned char c) { return std::tolower(c); });
                    is_connected_to_charger_ = (val_lower == "true");
                    charger_connect_key_found = true;
                }
            }
        }
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
                        percentage_key_found = true;
                    } catch (const std::exception& e) {
                        battery_percentage_ = -1.0;
                        RCLCPP_WARN(this->get_logger(), "解析电量失败: %s", e.what());
                    }
                }
            }
        }

        if (status_powerboard_name && status_powerboardsafety_name) {
            if (!charging_key_found) {
                RCLCPP_WARN(this->get_logger(), "未找到battery_charging键");
            }
            if (!percentage_key_found) {
                RCLCPP_WARN(this->get_logger(), "未找到battery_percentage键");
            }
            if (!charger_connect_key_found) {
                RCLCPP_WARN(this->get_logger(), "未找到charger_connect_state键");
            }
            RCLCPP_INFO(this->get_logger(), "充电状态: %s, 当前电量: %.2f, 连接到充电器: %s",
                        is_charging_ ? "true" : "false", battery_percentage_,
                        is_connected_to_charger_ ? "true" : "false");
            return;
        }
    }

    // 如果没有找到电源板诊断项
    RCLCPP_WARN(this->get_logger(), "未找到PowerBoard诊断信息");
}

bool AutoChargingNode::navToPose(const geometry_msgs::msg::PoseStamped& goal_pose) {
    RCLCPP_INFO(this->get_logger(), "等待 nav2 服务器...");
    while (rclcpp::ok() && !cancel_requested_.load()) {
        if (nav_client_->wait_for_action_server(std::chrono::milliseconds(200))) {
            break;
        }
    }

    if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "ROS 已关闭，等待导航服务器失败");
        return false;
    }

    if (cancel_requested_.load()) {
        RCLCPP_WARN(this->get_logger(), "导航前收到回充取消请求");
        return false;
    }

    nav2_msgs::action::NavigateToPose::Goal goal_msg;
    goal_msg.pose = goal_pose;

    RCLCPP_INFO(this->get_logger(), "发送导航目标...");
    auto send_goal_future = nav_client_->async_send_goal(goal_msg);
    bool cancel_requested_while_sending = false;
    auto cancel_request_time = std::chrono::steady_clock::time_point{};

    while (rclcpp::ok()) {
        if (cancel_requested_.load()) {
            if (!cancel_requested_while_sending) {
                RCLCPP_WARN(this->get_logger(), "发送导航目标期间收到回充取消请求");
                cancel_requested_while_sending = true;
                cancel_request_time = std::chrono::steady_clock::now();
                cancelActiveNavigation();
            }

            if (std::chrono::steady_clock::now() - cancel_request_time > std::chrono::seconds(2)) {
                RCLCPP_WARN(this->get_logger(), "等待导航目标响应超时，已发送取消请求");
                return false;
            }
        }

        if (send_goal_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            break;
        }

        rclcpp::spin_some(this->shared_from_this());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "ROS 已关闭，发送导航目标失败");
        return false;
    }

    auto goal_handle = send_goal_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "导航目标被拒绝！");
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(active_nav_goal_mutex_);
        active_nav_goal_handle_ = goal_handle;
    }

    if (cancel_requested_.load()) {
        RCLCPP_WARN(this->get_logger(), "导航目标已接受但回充已取消，立即取消内部导航目标");
        cancelActiveNavigation();
        {
            std::lock_guard<std::mutex> lock(active_nav_goal_mutex_);
            active_nav_goal_handle_.reset();
        }
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "导航目标已接受，导航中...");
    auto result_future = nav_client_->async_get_result(goal_handle);

    while (rclcpp::ok()) {
        if (cancel_requested_.load()) {
            RCLCPP_WARN(this->get_logger(), "回充取消请求已收到，取消内部导航目标");
            cancelActiveNavigation();
            {
                std::lock_guard<std::mutex> lock(active_nav_goal_mutex_);
                active_nav_goal_handle_.reset();
            }
            return false;
        }

        if (result_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            break;
        }

        rclcpp::spin_some(this->shared_from_this());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "ROS 已关闭，获取导航结果失败");
        {
            std::lock_guard<std::mutex> lock(active_nav_goal_mutex_);
            active_nav_goal_handle_.reset();
        }
        return false;
    }

    auto result = result_future.get();
    {
        std::lock_guard<std::mutex> lock(active_nav_goal_mutex_);
        active_nav_goal_handle_.reset();
    }

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "导航完成！");
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "导航失败，状态码: %d", static_cast<int>(result.code));
        return false;
    }
}

void AutoChargingNode::requestCancel() {
    cancel_requested_.store(true);
    cancelActiveNavigation();
    publishTwist(0.0, 0.0);
}

void AutoChargingNode::clearCancel() { cancel_requested_.store(false); }

bool AutoChargingNode::isCancelRequested() const { return cancel_requested_.load(); }

void AutoChargingNode::cancelActiveNavigation() {
    NavigateToPoseGoalHandle::SharedPtr goal_handle;
    {
        std::lock_guard<std::mutex> lock(active_nav_goal_mutex_);
        goal_handle = active_nav_goal_handle_;
    }

    try {
        if (goal_handle) {
            nav_client_->async_cancel_goal(goal_handle);
        } else {
            nav_client_->async_cancel_all_goals();
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "取消内部导航目标失败: %s", e.what());
    }
}

void AutoChargingNode::publishTwist(double linear_x, double angular_z) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear_x;
    twist.angular.z = angular_z;
    cmd_pub_->publish(twist);
}

void AutoChargingNode::backward() {
    // 从参数服务器获取配置参数
    const double initial_speed = this->get_parameter("backward.initial_speed").as_double();
    const double connected_speed = this->get_parameter("backward.connected_speed").as_double();
    const double max_distance = this->get_parameter("backward.max_distance").as_double();
    const double stable_duration = this->get_parameter("backward.stable_duration").as_double();
    const double max_backward_duration = this->get_parameter("backward.max_duration").as_double();
    const double control_rate = this->get_parameter("backward.control_rate").as_double();
    const double max_dt = this->get_parameter("backward.max_dt").as_double();

    RCLCPP_INFO(this->get_logger(), "开始后退 (初始速度=%.3f m/s, 最大距离=%.2f m, 最长时间=%.1f s)", initial_speed,
                max_distance, max_backward_duration);

    // 状态变量：INITIAL=未曾连接, CONNECTED=已曾连接(速度切换后)
    enum class State { INITIAL, CONNECTED };
    State state = State::INITIAL;

    double current_speed = initial_speed;
    double total_distance = 0.0;
    auto start_time = this->now();  // 后退开始时间
    auto last_time = start_time;
    auto current_connection_start = rclcpp::Time(0, 0, RCL_ROS_TIME);
    bool prev_connected = false;

    rclcpp::Rate rate(control_rate);
    int log_counter = 0;  // 降低日志频率

    while (rclcpp::ok() && !cancel_requested_.load()) {
        auto current_time = this->now();
        double dt = (current_time - last_time).seconds();

        // 防止异常dt值
        if (dt > max_dt) {
            RCLCPP_WARN(this->get_logger(), "异常时间步长 %.3f秒，限制为 %.3f秒", dt, max_dt);
            dt = max_dt;
        }

        last_time = current_time;
        total_distance += std::abs(current_speed * dt);

        // 检查最大距离
        if (total_distance >= max_distance) {
            RCLCPP_INFO(this->get_logger(), "达到最大后退距离 %.3f m", total_distance);
            break;
        }

        // 检查后退总时长
        double total_time = (current_time - start_time).seconds();
        if (total_time >= max_backward_duration) {
            RCLCPP_INFO(this->get_logger(), "⏰ 后退时间达到 %.1f秒，停止", total_time);
            break;
        }

        // 获取连接状态
        rclcpp::spin_some(this->shared_from_this());
        bool is_connected = isConnectedToCharger();

        // 状态机处理
        switch (state) {
            case State::INITIAL:
                if (is_connected) {
                    RCLCPP_INFO(this->get_logger(), "🔌 第一次连接，切换到慢速 %.3f m/s", connected_speed);
                    current_speed = connected_speed;
                    current_connection_start = current_time;
                    prev_connected = true;
                    state = State::CONNECTED;
                }
                break;

            case State::CONNECTED:
                // 检测连接上沿：重新连接时重置计时
                if (is_connected && !prev_connected) {
                    RCLCPP_INFO(this->get_logger(), "🔌 重新连接");
                    current_connection_start = current_time;
                } else if (!is_connected && prev_connected) {
                    RCLCPP_WARN(this->get_logger(), "⚠️ 连接断开");
                }

                if (is_connected) {
                    double connected_time = (current_time - current_connection_start).seconds();

                    // 每0.2秒打印一次状态
                    if (log_counter++ % 10 == 0) {
                        RCLCPP_INFO(this->get_logger(), "连接中 %.2f/%.1f秒", connected_time, stable_duration);
                    }

                    if (connected_time >= stable_duration) {
                        RCLCPP_INFO(this->get_logger(), "✅ 持续连接 %.1f秒，完成", connected_time);
                        goto exit_loop;
                    }
                }
                break;
        }

        prev_connected = is_connected;

        // 发布速度
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = current_speed;
        twist.angular.z = 0.0;
        cmd_pub_->publish(twist);

        rate.sleep();
    }

exit_loop:
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    RCLCPP_INFO(this->get_logger(), "后退完成 (总距离: %.3f m)", total_distance);
}

void AutoChargingNode::undocking(double duration) {
    double speed;

    this->get_parameter("undocking.speed", speed);
    RCLCPP_INFO(this->get_logger(), "Undocking 前进 %.1f秒 (速度=%.2f m/s)...", duration, speed);

    auto start_time = this->now();
    rclcpp::Rate rate(50);

    while (rclcpp::ok() && !cancel_requested_.load()) {
        auto elapsed = (this->now() - start_time).seconds();
        if (elapsed >= duration) break;

        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = speed;
        twist.angular.z = 0.0;
        cmd_pub_->publish(twist);

        rate.sleep();
    }

    cmd_pub_->publish(geometry_msgs::msg::Twist());
    RCLCPP_INFO(this->get_logger(), "Undocking 完成，已停止");
}

bool AutoChargingNode::startDetection() {
    RCLCPP_INFO(this->get_logger(), "等待 AprilTag 检测服务...");
    if (!start_detection_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "AprilTag 检测服务不可用！");
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = start_detection_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "调用启动检测服务失败！");
        return false;
    }

    auto response = future.get();
    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "✅ AprilTag 检测已启动: %s", response->message.c_str());
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "❌ 启动检测失败: %s", response->message.c_str());
        return false;
    }
}

bool AutoChargingNode::stopDetection() {
    RCLCPP_INFO(this->get_logger(), "等待 AprilTag 检测服务...");
    if (!stop_detection_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "AprilTag 检测服务不可用！");
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = stop_detection_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "调用停止检测服务失败！");
        return false;
    }

    auto response = future.get();
    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "✅ AprilTag 检测已停止: %s", response->message.c_str());
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "❌ 停止检测失败: %s", response->message.c_str());
        return false;
    }
}

// ==================== 工具函数 ====================
geometry_msgs::msg::PoseStamped readChargingPose(const std::string& json_path) {
    geometry_msgs::msg::PoseStamped pose;

    std::ifstream f(json_path);
    if (!f.is_open()) {
        throw std::runtime_error("无法打开充电点配置文件: " + json_path);
    }

    nlohmann::json data = nlohmann::json::parse(f);

    pose.header.frame_id = data.value("frame_id", "map");
    pose.pose.position.x = data["position"]["x"];
    pose.pose.position.y = data["position"]["y"];
    pose.pose.position.z = data["position"]["z"];
    pose.pose.orientation.x = data["orientation"]["x"];
    pose.pose.orientation.y = data["orientation"]["y"];
    pose.pose.orientation.z = data["orientation"]["z"];
    pose.pose.orientation.w = data["orientation"]["w"];

    return pose;
}

// ==================== NavigateToCharging ====================
NavigateToCharging::NavigateToCharging(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config) {}

BT::PortsList NavigateToCharging::providedPorts() { return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")}; }

BT::NodeStatus NavigateToCharging::tick() {
    auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
    if (!goal) {
        throw BT::RuntimeError("Missing goal pose");
    }

    auto charging_node = config().blackboard->get<std::shared_ptr<AutoChargingNode>>("charging_node");
    if (!charging_node) {
        throw BT::RuntimeError("charging_node not set in blackboard");
    }

    bool ok = charging_node->navToPose(goal.value());
    if (charging_node->isCancelRequested()) {
        return BT::NodeStatus::FAILURE;
    }
    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ==================== Dock ====================
Dock::Dock(const std::string& name, const BT::NodeConfiguration& config) : BT::StatefulActionNode(name, config) {}

Dock::~Dock() = default;

BT::PortsList Dock::providedPorts() {
    return {BT::InputPort<double>("timeout", 120.0, "超时时间(秒)"),
            BT::OutputPort<int>("result", "执行结果 0=成功 1=失败")};
}

BT::NodeStatus Dock::onStart() {
    auto charging_node = config().blackboard->get<std::shared_ptr<AutoChargingNode>>("charging_node");
    if (!charging_node) {
        throw BT::RuntimeError("charging_node not set in blackboard");
    }
    charging_node_ = charging_node;

    // 读取超时参数
    if (auto t = getInput<double>("timeout")) timeout_ = t.value();

    start_time_ = charging_node_->now();
    reverse_docking_ = std::make_unique<ReverseDockingBtController>(charging_node_);
    reverse_docking_->start();
    RCLCPP_INFO(charging_node_->get_logger(), "Dock started with reverse controller, timeout=%.1f", timeout_);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Dock::onRunning() {
    if (charging_node_->isCancelRequested()) {
        if (reverse_docking_) reverse_docking_->stop();
        charging_node_->publishTwist(0.0, 0.0);
        return BT::NodeStatus::FAILURE;
    }

    // 超时检测
    double elapsed = (charging_node_->now() - start_time_).seconds();
    if (elapsed > timeout_) {
        RCLCPP_ERROR(charging_node_->get_logger(), "Dock 超时 %.1f 秒", elapsed);
        if (reverse_docking_) reverse_docking_->stop();
        charging_node_->publishTwist(0.0, 0.0);
        setOutput("result", 1);
        return BT::NodeStatus::FAILURE;
    }

    if (reverse_docking_ && reverse_docking_->done()) {
        reverse_docking_->stop();
        charging_node_->publishTwist(0.0, 0.0);
        charging_node_->backward();
        if (charging_node_->isCancelRequested()) {
            return BT::NodeStatus::FAILURE;
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
    charging_node_ = charging_node;  // Store for use in other methods

    start_time_ = charging_node_->now();
    RCLCPP_INFO(charging_node_->get_logger(), "开始检测充电状态，超时: %.1f秒", timeout_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CheckCharging::onRunning() {
    if (charging_node_->isCancelRequested()) {
        RCLCPP_WARN(charging_node_->get_logger(), "充电检测收到取消请求");
        return BT::NodeStatus::FAILURE;
    }

    double elapsed = (charging_node_->now() - start_time_).seconds();

    // 处理诊断订阅队列，获取最新连接状态
    rclcpp::spin_some(charging_node_);

    static int consecutive_charging = 0;
    // bool connected = charging_node_->isConnectedToCharger();
    bool charging = charging_node_->isCharging();

    consecutive_charging = charging ? (consecutive_charging + 1) : 0;

    RCLCPP_INFO(charging_node_->get_logger(), "⚡ 充电状态检测: %s, 连续次数: %d/10, 已等待: %.1f秒",
                charging ? "true" : "false", consecutive_charging, elapsed);

    if (consecutive_charging >= 10) {
        RCLCPP_INFO(charging_node_->get_logger(), "✅ 检测到充电状态稳定(连续10次)！");
        consecutive_charging = 0;
        return BT::NodeStatus::SUCCESS;
    }

    if (elapsed >= timeout_) {
        RCLCPP_WARN(charging_node_->get_logger(), "⏰ 充电检测超时 (%.1f秒)", timeout_);
        consecutive_charging = 0;
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
        RCLCPP_ERROR(charging_node->get_logger(), "缺少阈值输入 threshold");
        return BT::NodeStatus::FAILURE;
    }

    // 处理诊断订阅队列，获取最新电量
    rclcpp::spin_some(charging_node);

    double battery = charging_node->getBatteryPercentage();
    if (battery < 0.0) {
        RCLCPP_WARN(charging_node->get_logger(), "尚未获取到有效电量信息");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(charging_node->get_logger(), "电量检测: 当前 %.2f 对比阈值 %.2f", battery, threshold.value());

    return battery >= threshold.value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ==================== Undocking ====================
Undocking::Undocking(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode(name, config) {}

// ✅ 添加空实现
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
        RCLCPP_WARN(charging_node->get_logger(), "未提供Undocking持续时间，使用默认值 %.2f 秒", duration);
    }

    charging_node->undocking(duration);
    if (charging_node->isCancelRequested()) {
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
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
        RCLCPP_ERROR(charging_node->get_logger(), "❌ 已达到最大重试次数 %d", max_retries_);
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(charging_node->get_logger(), "📊 当前重试次数: %d/%d", count, max_retries_);
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
    RCLCPP_INFO(charging_node->get_logger(), "➕ 重试计数器增加: %d -> %d", count, count + 1);
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList IncrementCounter::providedPorts() { return {}; }

// ==================== StartDetection ====================
StartDetection::StartDetection(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config) {}

BT::PortsList StartDetection::providedPorts() { return {}; }

BT::NodeStatus StartDetection::tick() {
    auto charging_node = config().blackboard->get<std::shared_ptr<AutoChargingNode>>("charging_node");
    if (!charging_node) {
        throw BT::RuntimeError("charging_node not set in blackboard");
    }
    if (charging_node->isCancelRequested()) {
        return BT::NodeStatus::FAILURE;
    }
    bool ok = charging_node->startDetection();
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
