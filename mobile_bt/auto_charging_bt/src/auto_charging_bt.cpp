#include "auto_charging_bt/auto_charging_bt.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>
#include <thread>

#include "auto_charging_bt/reverse_docking_bt_controller.hpp"

// 充电节点实例由行为树黑板传递，各行为节点共用同一实例。

// 异步导航状态字符串，与 navigation_client.py 的 Action 状态约定保持一致。
// 集中定义避免魔法字符串散落各处，便于统一修改和排查。
namespace {
constexpr const char* kNavStatusRunning   = "RUNNING";    // 进行中
constexpr const char* kNavStatusSucceeded = "SUCCEEDED";  // 成功
constexpr const char* kNavStatusFailed    = "FAILED";     // 失败
constexpr const char* kNavStatusCancelled = "CANCELLED";  // 被取消
}  // namespace

// ==================== AutoChargingNode 实现 ====================
AutoChargingNode::AutoChargingNode() : Node("AutoChargingNode") {
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

    // AprilTag检测服务客户端
    start_detection_client_ = this->create_client<std_srvs::srv::Trigger>("start_apriltag_detection");
    stop_detection_client_ = this->create_client<std_srvs::srv::Trigger>("stop_apriltag_detection");
}

void AutoChargingNode::cbDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
    bool charging_key_found = false;
    bool percentage_key_found = false;
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
                        percentage_key_found = true;
                    } catch (const std::exception& e) {
                        battery_percentage_ = -1.0;
                        RCLCPP_WARN(this->get_logger(), "解析电量失败: %s", e.what());
                    }
                }
            }
        }

        if (status_powerboard_name) {
            if (!charging_key_found) {
                RCLCPP_WARN(this->get_logger(), "未找到battery_charging键");
            }
            if (!percentage_key_found) {
                RCLCPP_WARN(this->get_logger(), "未找到battery_percentage键");
            }
            RCLCPP_INFO(this->get_logger(), "充电状态: %s, 当前电量: %.2f",
                        is_charging_ ? "true" : "false", battery_percentage_);
            return;
        }
    }

    // 如果没有找到电源板诊断项
    RCLCPP_WARN(this->get_logger(), "未找到PowerBoard诊断信息");
}

AutoChargingNode::ChargerState AutoChargingNode::confirmEntryChargerState() {
    const double max_duration = std::max(0.1, this->get_parameter("entry_state.max_duration").as_double());
    const double sample_interval = std::max(0.02, this->get_parameter("entry_state.sample_interval").as_double());
    const int required_stable_count =
        std::max(1, static_cast<int>(this->get_parameter("entry_state.required_stable_count").as_int()));

    RCLCPP_INFO(this->get_logger(), "入口状态确认开始: 最长 %.2f 秒, 间隔 %.2f 秒, 连续 %d 次一致",
                max_duration, sample_interval, required_stable_count);

    ChargerState last_state = ChargerState::UNKNOWN;
    int stable_count = 0;
    int sample_count = 0;
    auto start_time = this->now();

    while (rclcpp::ok()) {
        rclcpp::spin_some(this->shared_from_this());

        const bool charging = isCharging();
        ChargerState current_state = charging ? ChargerState::CHARGING : ChargerState::DISCONNECTED;

        ++sample_count;
        if (current_state == last_state) {
            ++stable_count;
        } else {
            last_state = current_state;
            stable_count = 1;
        }

        RCLCPP_INFO(this->get_logger(),
                    "入口状态确认: 第 %d 次, 充电=%s, 当前状态=%s, 连续=%d/%d",
                    sample_count, charging ? "true" : "false",
                    chargerStateName(current_state), stable_count, required_stable_count);

        if (stable_count >= required_stable_count) {
            RCLCPP_INFO(this->get_logger(), "入口状态确认完成: %s", chargerStateName(current_state));
            return current_state;
        }

        const double elapsed = (this->now() - start_time).seconds();
        if (elapsed >= max_duration) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::duration<double>(sample_interval));
    }

    RCLCPP_WARN(this->get_logger(), "入口状态 %.2f 秒内未稳定，按未充电处理，进入正常回充流程",
                max_duration);
    return ChargerState::DISCONNECTED;
}

const char* AutoChargingNode::chargerStateName(ChargerState state) const {
    switch (state) {
        case ChargerState::CHARGING:
            return "正在充电";
        case ChargerState::DISCONNECTED:
            return "未充电";
        case ChargerState::UNKNOWN:
        default:
            return "未知";
    }
}

bool AutoChargingNode::navToPose(const geometry_msgs::msg::PoseStamped& goal_pose) {
    RCLCPP_INFO(this->get_logger(), "等待 nav2 服务器...");
    nav_client_->wait_for_action_server();

    nav2_msgs::action::NavigateToPose::Goal goal_msg;
    goal_msg.pose = goal_pose;

    RCLCPP_INFO(this->get_logger(), "发送导航目标...");
    auto send_goal_future = nav_client_->async_send_goal(goal_msg);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), send_goal_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "发送导航目标失败！");
        return false;
    }

    auto goal_handle = send_goal_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "导航目标被拒绝！");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "导航目标已接受，导航中...");
    auto result_future = nav_client_->async_get_result(goal_handle);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "获取导航结果失败！");
        return false;
    }

    auto result = result_future.get();

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "导航完成！");
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "导航失败，状态码: %d", static_cast<int>(result.code));
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
    nav_goal_handle_ = nullptr;
    nav_result_status_ = kNavStatusRunning;
    nav_result_ready_ = false;
    nav_send_options_ = std::make_shared<
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions>();

    // goal_response 回调：记录 goal_handle，若被拒绝则直接置失败。
    nav_send_options_->goal_response_callback =
        [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> handle) {
            if (!handle) {
                RCLCPP_ERROR(this->get_logger(), "[NavigateToCharging] Nav2 拒绝了导航目标");
                nav_result_status_ = kNavStatusFailed;
                nav_result_ready_ = true;
                return;
            }
            nav_goal_handle_ = handle;
            RCLCPP_INFO(this->get_logger(), "[NavigateToCharging] Nav2 已接受导航目标，导航中...");
        };

    // result 回调：记录最终状态字符串，置 nav_result_ready_。
    nav_send_options_->result_callback =
        [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
            nav_result_ready_ = true;
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "[NavigateToCharging] 导航完成");
                nav_result_status_ = kNavStatusSucceeded;
            } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
                RCLCPP_WARN(this->get_logger(), "[NavigateToCharging] 导航被取消");
                nav_result_status_ = kNavStatusCancelled;
            } else {
                RCLCPP_ERROR(this->get_logger(),
                             "[NavigateToCharging] 导航失败，状态码: %d",
                             static_cast<int>(result.code));
                nav_result_status_ = kNavStatusFailed;
            }
            nav_goal_handle_ = nullptr;
        };

    if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "[NavigateToCharging] 等待 Nav2 服务超时，未发送 goal");
        return false;
    }

    nav2_msgs::action::NavigateToPose::Goal goal_msg;
    goal_msg.pose = goal_pose;
    nav_client_->async_send_goal(goal_msg, *nav_send_options_);
    nav_goal_sent_ = true;
    RCLCPP_INFO(this->get_logger(), "[NavigateToCharging] 已异步发送 Nav2 goal");
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
    if (nav_goal_handle_) {
        RCLCPP_INFO(this->get_logger(), "[NavigateToCharging] onHalted，取消 Nav2 goal");
        nav_client_->async_cancel_goal(nav_goal_handle_);
        nav_goal_handle_ = nullptr;
    } else {
        // goal 已发出但 goal_response 回调还没回来，拿不到 goal_handle。
        // 用 async_cancel_all_goals 兜底：取消该 client 上所有待处理 goal，
        // 确保正在执行的 Nav2 导航被停止，避免机器人继续向回充点移动。
        RCLCPP_WARN(this->get_logger(),
                    "[NavigateToCharging] onHalted 时 goal_handle 尚未就绪，"
                    "改用 async_cancel_all_goals 取消所有 Nav2 goal");
        nav_client_->async_cancel_all_goals();
    }
    nav_result_status_ = kNavStatusCancelled;
    nav_result_ready_ = true;
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
    const double max_distance = this->get_parameter("backward.max_distance").as_double();
    const double max_backward_duration = this->get_parameter("backward.max_duration").as_double();
    const double control_rate = this->get_parameter("backward.control_rate").as_double();
    const double max_dt = this->get_parameter("backward.max_dt").as_double();

    RCLCPP_INFO(this->get_logger(),
                "末端后退开始: speed=%.3f m/s, max_distance=%.3f m, max_duration=%.1f s, "
                "停止条件=检测到正在充电",
                initial_speed, max_distance, max_backward_duration);

    double current_speed = initial_speed;
    double total_distance = 0.0;
    double total_time = 0.0;
    auto start_time = this->now();  // 后退开始时间
    auto last_time = start_time;
    bool last_charging = false;
    const char* stop_reason = "ROS停止或循环退出";

    rclcpp::Rate rate(control_rate);

    while (rclcpp::ok()) {
        auto current_time = this->now();
        double dt = (current_time - last_time).seconds();

        // 防止异常dt值
        if (dt > max_dt) {
            RCLCPP_WARN(this->get_logger(), "异常时间步长 %.3f秒，限制为 %.3f秒", dt, max_dt);
            dt = max_dt;
        }

        last_time = current_time;
        total_time = (current_time - start_time).seconds();
        total_distance += std::abs(current_speed * dt);

        // 检查最大距离
        if (total_distance >= max_distance) {
            stop_reason = "达到最大后退距离";
            RCLCPP_WARN(this->get_logger(), "末端后退停止: 达到最大后退距离 %.3f/%.3f m", total_distance,
                        max_distance);
            break;
        }

        // 检查后退总时长
        if (total_time >= max_backward_duration) {
            stop_reason = "达到最大后退时间";
            RCLCPP_WARN(this->get_logger(), "末端后退停止: 达到最大后退时间 %.2f/%.2f s", total_time,
                        max_backward_duration);
            break;
        }

        // 获取充电状态
        rclcpp::spin_some(this->shared_from_this());
        bool is_charging = isCharging();
        last_charging = is_charging;

        if (is_charging) {
            stop_reason = "检测到正在充电";
            RCLCPP_INFO(this->get_logger(),
                        "末端后退停止: charging=%s, distance=%.3f m, elapsed=%.2f s",
                        is_charging ? "true" : "false", total_distance, total_time);
            break;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "末端后退中: charging=false, speed=%.3f m/s, "
                             "distance=%.3f/%.3f m, elapsed=%.2f/%.2f s",
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
    RCLCPP_INFO(this->get_logger(),
                "末端后退完成: reason=%s, final_charging=%s, distance=%.3f m, elapsed=%.2f s",
                stop_reason, last_charging ? "true" : "false", total_distance, total_time);
}

void AutoChargingNode::relieveDockingPressure() {
    const double speed = this->get_parameter("pressure_relief.speed").as_double();
    const double duration = this->get_parameter("pressure_relief.duration").as_double();
    const double control_rate = this->get_parameter("pressure_relief.control_rate").as_double();

    if (duration <= 0.0 || control_rate <= 0.0) {
        RCLCPP_WARN(this->get_logger(),
                    "充电桩压力释放参数无效，跳过前进动作: duration=%.3f s, control_rate=%.3f Hz",
                    duration, control_rate);
        publishTwist(0.0, 0.0);
        return;
    }

    RCLCPP_INFO(this->get_logger(),
                "充电状态已确认，开始短时前进减小顶桩压力: speed=%.3f m/s, duration=%.3f s",
                speed, duration);

    const auto start_time = this->now();
    rclcpp::Rate rate(control_rate);
    while (rclcpp::ok() && (this->now() - start_time).seconds() < duration) {
        publishTwist(speed, 0.0);
        rate.sleep();
    }

    publishTwist(0.0, 0.0);
    RCLCPP_INFO(this->get_logger(), "短时前进完成，已发布零速度");
}

void AutoChargingNode::undocking(double duration) {
    double speed;

    this->get_parameter("undocking.speed", speed);
    RCLCPP_INFO(this->get_logger(), "Undocking 前进 %.1f秒 (速度=%.2f m/s)...", duration, speed);

    auto start_time = this->now();
    rclcpp::Rate rate(50);

    while (rclcpp::ok()) {
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
    if (apriltag_detection_running_) {
        RCLCPP_INFO(this->get_logger(), "AprilTag 检测已处于启动状态，重复启动按成功处理");
        return true;
    }

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
        apriltag_detection_running_ = true;
        RCLCPP_INFO(this->get_logger(), "✅ AprilTag 检测已启动: %s", response->message.c_str());
        return true;
    } else {
        std::string message_lower = response->message;
        std::transform(message_lower.begin(), message_lower.end(), message_lower.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        if (message_lower.find("already") != std::string::npos ||
            response->message.find("已") != std::string::npos) {
            apriltag_detection_running_ = true;
            RCLCPP_WARN(this->get_logger(), "AprilTag 检测服务返回已启动状态，按成功处理: %s",
                        response->message.c_str());
            return true;
        }
        RCLCPP_ERROR(this->get_logger(), "❌ 启动检测失败: %s", response->message.c_str());
        return false;
    }
}

bool AutoChargingNode::stopDetection() {
    if (!apriltag_detection_running_) {
        RCLCPP_INFO(this->get_logger(), "AprilTag 检测已处于停止状态，重复停止按成功处理");
        return true;
    }

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
        apriltag_detection_running_ = false;
        RCLCPP_INFO(this->get_logger(), "✅ AprilTag 检测已停止: %s", response->message.c_str());
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
            RCLCPP_WARN(this->get_logger(), "AprilTag 检测服务返回已停止状态，按成功处理: %s",
                        response->message.c_str());
            return true;
        }
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

    if (!charging_node_->sendNavGoal(goal.value())) {
        RCLCPP_ERROR(charging_node_->get_logger(),
                     "[NavigateToCharging] 发送 Nav2 goal 失败，返回 FAILURE");
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToCharging::onRunning() {
    const std::string status = charging_node_->isNavDone();
    if (status == kNavStatusRunning) {
        return BT::NodeStatus::RUNNING;     // 仍在导航中
    }
    if (status == kNavStatusSucceeded) {
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
    RCLCPP_INFO(charging_node_->get_logger(),
                "[DockAttempt] 开始二维码视觉对接，本次超时上限 %.1f 秒", timeout_);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Dock::onRunning() {
    // 超时检测
    double elapsed = (charging_node_->now() - start_time_).seconds();
    if (elapsed > timeout_) {
        const std::string reason_code = reverse_docking_ ?
            reverse_docking_->failureReasonCode() : "CONTROLLER_NOT_CREATED";
        const std::string reason = reverse_docking_ ?
            reverse_docking_->failureReasonSummary() : "倒车对接控制器未创建";
        const std::string details = reverse_docking_ ?
            reverse_docking_->diagnosticSummary() : "倒车对接控制器未创建";
        RCLCPP_ERROR(charging_node_->get_logger(),
                     "[DockFailure] type=%s，二维码检测或视觉对接超时，"
                     "实际等待 %.1f 秒（上限 %.1f 秒）；原因=%s；"
                     "处置=停止检测、不退桩，由外层行为树决定是否重试；诊断=%s",
                     reason_code.c_str(), elapsed, timeout_, reason.c_str(), details.c_str());
        if (reverse_docking_) reverse_docking_->stop();
        charging_node_->publishTwist(0.0, 0.0);
        setOutput("result", 1);
        return BT::NodeStatus::FAILURE;
    }

    if (reverse_docking_ && reverse_docking_->done()) {
        reverse_docking_->stop();
        charging_node_->publishTwist(0.0, 0.0);
        RCLCPP_INFO(charging_node_->get_logger(),
                    "[DockSuccess] 二维码视觉对接完成，开始执行末端后退压入");
        charging_node_->backward();
        RCLCPP_INFO(charging_node_->get_logger(),
                    "[DockSuccess] 末端后退压入完成，Dock 返回成功，下一步检查充电状态");
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
                "[ChargingCheck] 开始确认充电状态：超时上限=%.1f 秒，连续确认次数=%d，"
                "初始充电=%s",
                timeout_, required_stable_count,
                charging_node_->isCharging() ? "true" : "false");
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

    RCLCPP_INFO_THROTTLE(charging_node_->get_logger(), *charging_node_->get_clock(), 1000,
                         "[ChargingCheck] 充电=%s，连续确认=%d/%d，已等待=%.1f 秒",
                         charging ? "true" : "false",
                         consecutive_charging_, required_stable_count, elapsed);

    if (consecutive_charging_ >= required_stable_count) {
        RCLCPP_INFO(charging_node_->get_logger(),
                    "[DockSuccess] type=CHARGING_CONFIRMED，充电状态连续稳定，回充成功");
        consecutive_charging_ = 0;
        charging_node_->relieveDockingPressure();
        return BT::NodeStatus::SUCCESS;
    }

    if (elapsed >= timeout_) {
        RCLCPP_ERROR(charging_node_->get_logger(),
                     "[DockFailure] type=CHARGING_NOT_CONFIRMED，视觉对接和末端压入已完成，"
                     "但 %.1f 秒内未确认稳定充电；最后状态：充电=%s，连续确认=%d/%d；"
                     "处置=返回失败，由行为树停止检测、退桩并决定是否重试",
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
    RCLCPP_INFO(charging_node->get_logger(), "行为树入口充电状态检查结果: %s",
                charging_node->chargerStateName(state));

    return state == AutoChargingNode::ChargerState::CHARGING ? BT::NodeStatus::SUCCESS
                                                             : BT::NodeStatus::FAILURE;
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
        RCLCPP_WARN(charging_node->get_logger(), "未提供Undocking持续时间，使用默认值 %.2f 秒", duration);
    }

    charging_node->undocking(duration);
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
        RCLCPP_INFO(charging_node->get_logger(), "%s %s开始第 %d/%d 次尝试",
                    attempt_tag, label.value().c_str(), count, total.value());
    } else {
        RCLCPP_WARN(charging_node->get_logger(),
                    "%s %s上一次尝试失败，开始第 %d/%d 次尝试（第 %d 次重试）",
                    retry_tag, label.value().c_str(), count, total.value(), count - 1);
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
    bool ok = charging_node->startDetection();
    if (!ok) {
        RCLCPP_ERROR(charging_node->get_logger(),
                     "[DockFailure] type=DETECTION_START_FAILED，启动二维码检测失败；"
                     "本次回充对接尝试结束，由外层行为树决定是否重试");
    }
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
