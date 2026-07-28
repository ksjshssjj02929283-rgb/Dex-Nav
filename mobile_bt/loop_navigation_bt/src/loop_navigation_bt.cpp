#include "loop_navigation_bt/loop_navigation_bt.hpp"
#include "loop_navigation_bt/logging.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <sstream>

#include "nlohmann/json.hpp"

namespace loop_navigation_bt {

// 将角度归一化到 [-PI, PI]，避免跨越正负 PI 时误差突变。
double normalize_angle(double a) {
    while (a > M_PI) {
        a -= 2.0 * M_PI;
    }
    while (a < -M_PI) {
        a += 2.0 * M_PI;
    }
    return a;
}

double circularMean(const std::vector<double>& angles) {
    if (angles.empty()) {
        return 0.0;
    }

    double sin_sum = 0.0;
    double cos_sum = 0.0;
    for (const double angle : angles) {
        sin_sum += std::sin(angle);
        cos_sum += std::cos(angle);
    }
    if (std::hypot(sin_sum, cos_sum) < 1e-9) {
        return angles.back();
    }
    return std::atan2(sin_sum, cos_sum);
}

float parseBatteryPercentage(const diagnostic_msgs::msg::DiagnosticArray& msg) {
    for (const auto& status : msg.status) {
        if (status.name != "/Devices/System/PowerBoard/collector_node: PowerBoard") {
            continue;
        }
        for (const auto& kv : status.values) {
            if (kv.key != "battery_percentage") {
                continue;
            }
            try {
                const float parsed = static_cast<float>(std::stod(kv.value));
                if (std::isfinite(parsed) && parsed >= 0.0f && parsed <= 100.0f) {
                    return parsed;
                }
            } catch (...) {
                return -1.0f;
            }
        }
    }
    return -1.0f;
}

// 读取导航点 JSON，并同时获得点位名称、停留时间和总循环轮数。
std::vector<geometry_msgs::msg::PoseStamped> parseNavPoints(const std::string& json_path,
                                                            rclcpp::Logger logger,
                                                            std::vector<std::string>& goal_ids,
                                                            int& total_rounds,
                                                            std::vector<double>& goal_wait_times_sec,
                                                            double default_wait_time_sec) {
    std::vector<geometry_msgs::msg::PoseStamped> points;
    std::ifstream file(json_path);
    if (!file.is_open()) {
        LOOP_RCLCPP_ERROR(logger, "Failed to open navigation points file: %s", json_path.c_str());
        return points;
    }
    nlohmann::json j;
    file >> j;
    std::string frame = j.value("frame_id", std::string("map"));
    total_rounds = j.value("rounds", 0);
    goal_ids.clear();
    goal_wait_times_sec.clear();
    for (const auto& item : j["points"]) {
        const std::string goal_id = item.value("id", "goal_" + std::to_string(points.size()));
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame;
        pose.header.stamp = rclcpp::Clock().now();
        pose.pose.position.x = item["position"].value("x", 0.0);
        pose.pose.position.y = item["position"].value("y", 0.0);
        pose.pose.position.z = item["position"].value("z", 0.0);
        pose.pose.orientation.x = item["orientation"].value("x", 0.0);
        pose.pose.orientation.y = item["orientation"].value("y", 0.0);
        pose.pose.orientation.z = item["orientation"].value("z", 0.0);
        pose.pose.orientation.w = item["orientation"].value("w", 1.0);
        double wait_time_sec = default_wait_time_sec;
        const auto wait_time = item.find("wait_time_sec");
        if (wait_time != item.end()) {
            if (wait_time->is_number()) {
                const double configured_wait_time_sec = wait_time->get<double>();
                if (std::isfinite(configured_wait_time_sec) && configured_wait_time_sec >= 0.0) {
                    wait_time_sec = configured_wait_time_sec;
                } else {
                    LOOP_RCLCPP_WARN(logger,
                                "点位 %s 的 wait_time_sec 必须是非负有限数值，改用 YAML 默认值 %.2f 秒。",
                                goal_id.c_str(), default_wait_time_sec);
                }
            } else {
                LOOP_RCLCPP_WARN(logger,
                            "点位 %s 的 wait_time_sec 不是数值，改用 YAML 默认值 %.2f 秒。",
                            goal_id.c_str(), default_wait_time_sec);
            }
        }
        goal_ids.push_back(goal_id);
        goal_wait_times_sec.push_back(wait_time_sec);
        points.push_back(pose);
    }
    LOOP_RCLCPP_INFO(logger, "Successfully parsed %zu navigation points, rounds=%d.", points.size(), total_rounds);
    return points;
}

std::vector<geometry_msgs::msg::PoseStamped> parseNavPoints(const std::string& json_path, rclcpp::Logger logger) {
    std::vector<std::string> unused_goal_ids;
    std::vector<double> unused_goal_wait_times_sec;
    int unused_total_rounds = 0;
    return parseNavPoints(json_path, logger, unused_goal_ids, unused_total_rounds,
                          unused_goal_wait_times_sec, 0.0);
}

//--- Implementation for LoopNavigateFourPoints ---
LoopNavigateFourPoints::LoopNavigateFourPoints(const std::string& name,
                                               const BT::NodeConfiguration& config,
                                               rclcpp::Node::SharedPtr node_ptr)
    : BtActionNode(name, config, node_ptr) {
    action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");

    // TF 用于在导航结束后读取机器人实际位姿并计算到点误差。
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    if (!node_->has_parameter("navigation.wait_time_sec")) {
        node_->declare_parameter("navigation.wait_time_sec", wait_time_sec_);
    }
    if (!node_->has_parameter("navigation.action_server_wait_timeout_sec")) {
        node_->declare_parameter("navigation.action_server_wait_timeout_sec", action_server_wait_timeout_sec_);
    }
    if (!node_->has_parameter("navigation.odom_topic")) {
        node_->declare_parameter("navigation.odom_topic", odom_topic_);
    }
    if (!node_->has_parameter("navigation.odom_max_increment_m")) {
        node_->declare_parameter("navigation.odom_max_increment_m", odom_max_increment_m_);
    }
    if (!node_->has_parameter("goal_error_stats.desired_position_tolerance")) {
        node_->declare_parameter("goal_error_stats.desired_position_tolerance", desired_position_tolerance_);
    }
    if (!node_->has_parameter("goal_error_stats.accept_position_tolerance")) {
        node_->declare_parameter("goal_error_stats.accept_position_tolerance", accept_position_tolerance_);
    }
    if (!node_->has_parameter("goal_error_stats.sample_count")) {
        node_->declare_parameter("goal_error_stats.sample_count", static_cast<int>(pose_sample_count_));
    }
    if (!node_->has_parameter("goal_error_stats.sample_interval_sec")) {
        node_->declare_parameter("goal_error_stats.sample_interval_sec", pose_sample_interval_sec_);
    }
    if (!node_->has_parameter("goal_error_stats.use_goal_checker_error")) {
        node_->declare_parameter("goal_error_stats.use_goal_checker_error", use_goal_checker_error_);
    }
    if (!node_->has_parameter("goal_error_stats.stats_log_interval")) {
        node_->declare_parameter("goal_error_stats.stats_log_interval", static_cast<int>(stats_log_interval_));
    }
    node_->get_parameter("navigation.wait_time_sec", wait_time_sec_);
    node_->get_parameter("navigation.action_server_wait_timeout_sec", action_server_wait_timeout_sec_);
    node_->get_parameter("navigation.odom_topic", odom_topic_);
    node_->get_parameter("navigation.odom_max_increment_m", odom_max_increment_m_);
    node_->get_parameter("goal_error_stats.desired_position_tolerance", desired_position_tolerance_);
    node_->get_parameter("goal_error_stats.accept_position_tolerance", accept_position_tolerance_);
    int pose_sample_count = static_cast<int>(pose_sample_count_);
    int stats_log_interval = static_cast<int>(stats_log_interval_);
    node_->get_parameter("goal_error_stats.sample_count", pose_sample_count);
    node_->get_parameter("goal_error_stats.sample_interval_sec", pose_sample_interval_sec_);
    node_->get_parameter("goal_error_stats.use_goal_checker_error", use_goal_checker_error_);
    node_->get_parameter("goal_error_stats.stats_log_interval", stats_log_interval);
    if (!std::isfinite(wait_time_sec_) || wait_time_sec_ < 0.0) {
        LOOP_RCLCPP_WARN(node_->get_logger(), "navigation.wait_time_sec 必须是非负有限数值，重置为 2.0 秒。");
        wait_time_sec_ = 2.0;
    }
    if (action_server_wait_timeout_sec_ <= 0.0) {
        LOOP_RCLCPP_WARN(node_->get_logger(),
                    "navigation.action_server_wait_timeout_sec 必须大于 0，重置为 2.0 秒。");
        action_server_wait_timeout_sec_ = 2.0;
    }
    if (odom_topic_.empty()) {
        LOOP_RCLCPP_WARN(node_->get_logger(), "navigation.odom_topic 不能为空，重置为 /feedback/odom。");
        odom_topic_ = "/feedback/odom";
    }
    if (odom_max_increment_m_ <= 0.0) {
        LOOP_RCLCPP_WARN(node_->get_logger(), "navigation.odom_max_increment_m 必须大于 0，重置为 0.5 米。");
        odom_max_increment_m_ = 0.5;
    }
    if (pose_sample_count > 0) {
        pose_sample_count_ = static_cast<std::size_t>(pose_sample_count);
    } else {
        LOOP_RCLCPP_WARN(node_->get_logger(), "goal_error_stats.sample_count 必须为正数，保持默认值 %zu。",
                    pose_sample_count_);
    }
    if (pose_sample_interval_sec_ < 0.0) {
        LOOP_RCLCPP_WARN(node_->get_logger(), "goal_error_stats.sample_interval_sec 不能为负数，重置为 0.1。");
        pose_sample_interval_sec_ = 0.1;
    }
    if (stats_log_interval > 0) {
        stats_log_interval_ = static_cast<std::size_t>(stats_log_interval);
    } else {
        LOOP_RCLCPP_WARN(node_->get_logger(), "goal_error_stats.stats_log_interval 必须为正数，保持默认值 %zu。",
                    stats_log_interval_);
    }
    LOOP_RCLCPP_INFO(node_->get_logger(),
                "循环导航参数：点位默认等待 %.2f s，Action server 超时 %.2f s。",
                wait_time_sec_, action_server_wait_timeout_sec_);
    LOOP_RCLCPP_INFO(node_->get_logger(), "导航里程统计：订阅 %s，单次里程计增量上限 %.3f m。",
                odom_topic_.c_str(), odom_max_increment_m_);
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LoopNavigateFourPoints::odometryCallback, this, std::placeholders::_1));
    LOOP_RCLCPP_INFO(node_->get_logger(), "goal_checker 最终误差读取开关: %s。%s",
                use_goal_checker_error_ ? "true" : "false",
                use_goal_checker_error_ ? "优先读取 /navigation_goal_error，读取不到则回退 TF 采样。"
                                        : "禁用 /navigation_goal_error，直接使用 TF 窗口采样。");
    if (use_goal_checker_error_) {
        goal_error_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            "/navigation_goal_error", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
            [this](const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(latest_goal_error_mutex_);
                latest_goal_error_ = *msg;
                has_latest_goal_error_ = true;
            });
    }

    // 每次启动覆盖旧日志，避免不同批次的统计结果混在一起。
    log_file_.open("nav_error_log.txt", std::ios::out | std::ios::trunc);
    if (log_file_.is_open()) {
        log_file_ << "# 结果 耗时_s 点序号 点ID 目标x 目标y 目标yaw 当前x 当前y 当前yaw "
                     "纵向误差 横向误差 误差yaw 距离误差 本次里程_m 累计里程_m 里程有效 "
                     "期望阈值内 接受阈值内"
                  << std::endl;
        log_file_ << "# 误差定义: 纵向/横向误差均按目标点坐标系投影，和 goal_checker 保持一致" << std::endl;
        log_file_ << "# 采样方式: 导航结束后按点位 wait_time_sec 等待；未配置时默认等待 "
                  << wait_time_sec_ << " s，再连续采样 " << pose_sample_count_ << " 次，每次间隔 "
                  << pose_sample_interval_sec_ << " s，对误差窗口求平均" << std::endl;
        log_file_ << "# 统计阈值: 期望阈值=" << desired_position_tolerance_
                  << " m, 接受阈值=" << accept_position_tolerance_ << " m" << std::endl;
        log_file_ << "# 里程定义: 仅在 Nav2 目标执行期间，对 " << odom_topic_
                  << " 相邻位置增量进行累加；不包含自动回充和退桩距离" << std::endl;
    }

    // ROS 关闭时补写最终统计，避免 Ctrl+C 后丢失汇总结果。
    rclcpp::on_shutdown([this]() {
        if (!stats_logged_ && log_file_.is_open()) {
            LOOP_RCLCPP_INFO(node_->get_logger(), "ROS shutdown detected, calculating navigation statistics...");
            calculateAndLogStats();
            log_file_.close();
            stats_logged_ = true;
        }
    });
}

LoopNavigateFourPoints::~LoopNavigateFourPoints() {
    if (log_file_.is_open()) {
        calculateAndLogStats();
        log_file_.close();
    }
}

BT::PortsList LoopNavigateFourPoints::providedPorts() {
    return {BT::InputPort<std::string>("points", "Path to the JSON file with navigation points")};
}

BT::NodeStatus LoopNavigateFourPoints::onStart() {
    if (navigation_complete_) {
        try {
            config().blackboard->set("nav_loop_complete", true);
        } catch (...) {
        }
        return BT::NodeStatus::SUCCESS;
    }

    // If navigation is paused (e.g. during undocking/charging), don't dispatch a new goal
    try {
        bool paused = config().blackboard->get<bool>("nav_paused");
        if (paused) {
            LOOP_RCLCPP_DEBUG(node_->get_logger(), "Navigation paused -> skip goal dispatch");
            return BT::NodeStatus::SUCCESS;
        }
    } catch (...) {
    }

    // Check if we should stop due to low battery (for no-charging mode)
    try {
        bool should_stop = config().blackboard->get<bool>("should_stop_navigation");
        if (should_stop) {
            if (!stop_flag_logged_) {
                LOOP_RCLCPP_INFO(node_->get_logger(), "检测到停止标志，暂停发送新的导航目标。");
                stop_flag_logged_ = true;
            }
            return BT::NodeStatus::SUCCESS;
        }
        stop_flag_logged_ = false;
    } catch (...) {
    }

    // Load points once
    if (goals_.empty()) {
        auto json_path = getInput<std::string>("points");
        if (!json_path) {
            LOOP_RCLCPP_ERROR(node_->get_logger(), "'points' port is missing.");
            return BT::NodeStatus::FAILURE;
        }
        goals_ = parseNavPoints(json_path.value(), node_->get_logger(), goal_ids_, total_rounds_,
                                goal_wait_times_sec_, wait_time_sec_);
        if (goals_.empty()) {
            return BT::NodeStatus::FAILURE;
        }
        per_goal_stats_.resize(goals_.size());
        for (std::size_t i = 0; i < per_goal_stats_.size(); ++i) {
            per_goal_stats_[i].id = goal_ids_[i];
        }
    }

    if (!action_client_->wait_for_action_server(std::chrono::duration<double>(action_server_wait_timeout_sec_))) {
        LOOP_RCLCPP_ERROR(node_->get_logger(), "Action server /navigate_to_pose not available.");
        return BT::NodeStatus::FAILURE;
    }

    // Reset state for this goal
    result_ready_ = false;
    result_code_ = rclcpp_action::ResultCode::UNKNOWN;
    goal_handle_.reset();

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goals_[current_goal_index_];
    goal_ros_start_time_ = node_->now();
    goal_msg.pose.header.stamp = goal_ros_start_time_;
    const auto goal_wall_start_time = std::chrono::system_clock::now();
    goal_start_time_ = std::chrono::steady_clock::now();
    const std::string& current_goal_id = goal_ids_[current_goal_index_];
    const std::string goal_start_time_text = formatUtc8Time(goal_wall_start_time);
    LOOP_RCLCPP_INFO_AT(node_->get_logger(), goal_start_time_text, "开始导航目标点 #%d（%s）。",
                        current_goal_index_ + 1, current_goal_id.c_str());
    LOOP_RCLCPP_INFO_AT(node_->get_logger(), goal_start_time_text, "目标坐标：x=%.3f m，y=%.3f m。",
                        goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);
    startDistanceTracking();
    {
        std::lock_guard<std::mutex> lock(latest_goal_error_mutex_);
        has_latest_goal_error_ = false;
    }

    // 本次目标的代次会被回调捕获，用于过滤低电量取消后延迟到达的旧结果。
    const std::uint64_t goal_generation = ++goal_generation_;
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this, goal_generation](GoalHandleNavigateToPose::SharedPtr gh) {
        if (goal_generation != goal_generation_.load()) {
            return;
        }
        if (!gh) {
            LOOP_RCLCPP_ERROR(node_->get_logger(), "Goal rejected by server");
        } else {
            goal_handle_ = gh;
        }
    };
    send_goal_options.result_callback =
        [this, goal_generation](const GoalHandleNavigateToPose::WrappedResult& result) {
        if (goal_generation != goal_generation_.load()) {
            return;
        }
        result_code_ = result.code;
        result_ready_ = true;
    };

    future_goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LoopNavigateFourPoints::onRunning() {
    // If paused mid-execution, cancel current goal and yield SUCCESS
    try {
        bool paused = config().blackboard->get<bool>("nav_paused");
        if (paused) {
            if (goal_handle_) {
                LOOP_RCLCPP_INFO(node_->get_logger(), "Navigation paused: cancel current goal");
                action_client_->async_cancel_goal(goal_handle_);
            }
            const auto distance_snapshot = stopDistanceTracking();
            if (distance_snapshot.active) {
                LOOP_RCLCPP_WARN(node_->get_logger(),
                            "导航暂停：本次已移动 %.3f m，累计移动 %.3f m。",
                            distance_snapshot.goal_distance_m, distance_snapshot.cumulative_distance_m);
            }
            goal_handle_.reset();
            result_ready_ = false;
            return BT::NodeStatus::SUCCESS;
        }
    } catch (...) {
    }

    // Wait until goal is accepted
    if (!goal_handle_) {
        if (future_goal_handle_.valid() &&
            future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            auto tmp = future_goal_handle_.get();
            if (!tmp) {
                const auto distance_snapshot = stopDistanceTracking();
                LOOP_RCLCPP_WARN(node_->get_logger(),
                            "导航目标被服务器拒绝，本次已累计移动 %.3f m，累计导航距离 %.3f m。",
                            distance_snapshot.goal_distance_m, distance_snapshot.cumulative_distance_m);
                LOOP_RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server.");
                return BT::NodeStatus::FAILURE;
            }
            // goal_handle_ will have been set in goal_response_callback
        }
        return BT::NodeStatus::RUNNING;
    }

    // Wait for result
    if (!result_ready_) {
        return BT::NodeStatus::RUNNING;
    }

    // Handle result
    if (result_code_ == rclcpp_action::ResultCode::SUCCEEDED) {
        const auto arrival_wall_time = std::chrono::system_clock::now();
        succeeded_nav_++;
        const auto distance_snapshot = stopDistanceTracking();

        // 到点后按当前点位配置停留；点位未配置时已在解析阶段回退到 YAML 默认值。
        const std::string& current_goal_id = goal_ids_[current_goal_index_];
        const double current_wait_time_sec = goal_wait_times_sec_[current_goal_index_];
        LOOP_RCLCPP_INFO(node_->get_logger(),
                    "已到达目标点 #%d（%s）：本次移动 %.3f m，累计移动 %.3f m；"
                    "等待 %.2f 秒后采集到点误差并继续下一个目标。",
                    current_goal_index_ + 1, current_goal_id.c_str(), distance_snapshot.goal_distance_m,
                    distance_snapshot.cumulative_distance_m, current_wait_time_sec);
        if (!distance_snapshot.valid) {
            LOOP_RCLCPP_WARN(node_->get_logger(), "本次导航未收到有效里程计，显示的本次移动距离为 0。");
        }
        spinNodeForDuration(current_wait_time_sec, "成功到点后的稳定等待");

        // Log error
        const auto& goal_pose_msg = goals_[current_goal_index_].pose;
        tf2::Quaternion q_goal;
        tf2::fromMsg(goal_pose_msg.orientation, q_goal);
        double roll, pitch, yaw_goal;
        tf2::Matrix3x3(q_goal).getRPY(roll, pitch, yaw_goal);
        Pose goal_pose = {goal_pose_msg.position.x, goal_pose_msg.position.y, yaw_goal};

        const double elapsed_sec =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - goal_start_time_).count();
        const auto error_sample = getNavigationErrorSample(goal_pose);
        const std::string arrival_time_text = formatUtc8Time(arrival_wall_time);
        LOOP_RCLCPP_INFO_AT(node_->get_logger(), arrival_time_text, "到达目标点 #%d（%s）。",
                            current_goal_index_ + 1, current_goal_id.c_str());
        LOOP_RCLCPP_INFO_AT(node_->get_logger(), arrival_time_text,
                            "目标坐标：x=%.3f m，y=%.3f m，yaw=%.4f rad。",
                            goal_pose.x, goal_pose.y, goal_pose.yaw);
        if (error_sample.valid) {
            LOOP_RCLCPP_INFO_AT(node_->get_logger(), arrival_time_text,
                                "实际坐标：x=%.3f m，y=%.3f m，yaw=%.4f rad。",
                                error_sample.current.x, error_sample.current.y, error_sample.current.yaw);
            LOOP_RCLCPP_INFO_AT(node_->get_logger(), arrival_time_text,
                                "到点误差：纵向=%.4f m，横向=%.4f m，距离=%.4f m，角度=%.4f rad。",
                                error_sample.lon_error, error_sample.lat_error,
                                error_sample.dist_error, error_sample.yaw_error);
        } else {
            LOOP_RCLCPP_WARN_AT(node_->get_logger(), arrival_time_text,
                                "未获取到有效最终位姿，无法计算有效误差。");
        }
        const bool completes_round = ((current_goal_index_ + 1) % goals_.size()) == 0;
        if (completes_round) {
            // 先更新轮数，避免本次误差统计写出旧轮数
            completed_rounds_++;
        }
        logNavigationError(goal_pose, error_sample, distance_snapshot, elapsed_sec, true, current_goal_index_);

        current_goal_index_ = (current_goal_index_ + 1) % goals_.size();
        if (current_goal_index_ == 0) {
            if (total_rounds_ > 0) {
                LOOP_RCLCPP_INFO(
                    node_->get_logger(), "Completed navigation round %d/%d.", completed_rounds_, total_rounds_);
            } else {
                LOOP_RCLCPP_INFO(node_->get_logger(), "Completed navigation round %d.", completed_rounds_);
            }
            if (total_rounds_ > 0 && completed_rounds_ >= total_rounds_) {
                navigation_complete_ = true;
                try {
                    config().blackboard->set("nav_loop_complete", true);
                } catch (...) {
                }
            }
            if (last_stats_logged_sample_count_ != dist_errors_.size()) {
                calculateAndLogStats(true);
            }
        }
        goal_handle_.reset();
        result_ready_ = false;
        return BT::NodeStatus::SUCCESS;
    } else if (result_code_ == rclcpp_action::ResultCode::ABORTED) {
        // If navigation to a point fails, continue with the next point
        LOOP_RCLCPP_WARN(node_->get_logger(), "Goal #%d aborted (failed). Continue to next goal.", current_goal_index_ + 1);
        failed_nav_++;
        const auto distance_snapshot = stopDistanceTracking();
        LOOP_RCLCPP_WARN(node_->get_logger(),
                    "目标 #%d 导航失败：本次移动 %.3f m，累计移动 %.3f m。",
                    current_goal_index_ + 1, distance_snapshot.goal_distance_m,
                    distance_snapshot.cumulative_distance_m);
        if (!distance_snapshot.valid) {
            LOOP_RCLCPP_WARN(node_->get_logger(), "本次导航未收到有效里程计，显示的本次移动距离为 0。");
        }

        // 失败后使用与成功相同的当前点位停留时间，再统计机器人最终停止位置。
        const double current_wait_time_sec = goal_wait_times_sec_[current_goal_index_];
        spinNodeForDuration(current_wait_time_sec, "导航失败后的稳定等待");

        const auto& goal_pose_msg = goals_[current_goal_index_].pose;
        tf2::Quaternion q_goal;
        tf2::fromMsg(goal_pose_msg.orientation, q_goal);
        double roll, pitch, yaw_goal;
        tf2::Matrix3x3(q_goal).getRPY(roll, pitch, yaw_goal);
        Pose goal_pose = {goal_pose_msg.position.x, goal_pose_msg.position.y, yaw_goal};

        const double elapsed_sec =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - goal_start_time_).count();
        const auto error_sample = getNavigationErrorSample(goal_pose);
        const bool completes_round = ((current_goal_index_ + 1) % goals_.size()) == 0;
        if (completes_round) {
            // 先更新轮数，避免本次误差统计写出旧轮数
            completed_rounds_++;
        }
        logNavigationError(goal_pose, error_sample, distance_snapshot, elapsed_sec, false, current_goal_index_);

        current_goal_index_ = (current_goal_index_ + 1) % goals_.size();
        if (current_goal_index_ == 0) {
            if (total_rounds_ > 0) {
                LOOP_RCLCPP_INFO(
                    node_->get_logger(), "Completed navigation round %d/%d.", completed_rounds_, total_rounds_);
            } else {
                LOOP_RCLCPP_INFO(node_->get_logger(), "Completed navigation round %d.", completed_rounds_);
            }
            if (total_rounds_ > 0 && completed_rounds_ >= total_rounds_) {
                navigation_complete_ = true;
                try {
                    config().blackboard->set("nav_loop_complete", true);
                } catch (...) {
                }
            }
            if (last_stats_logged_sample_count_ != dist_errors_.size()) {
                calculateAndLogStats(true);
            }
        }
        goal_handle_.reset();
        result_ready_ = false;
        return BT::NodeStatus::SUCCESS;
    } else if (result_code_ == rclcpp_action::ResultCode::CANCELED) {
        // If navigation was canceled, propagate failure to BT
        const auto distance_snapshot = stopDistanceTracking();
        LOOP_RCLCPP_WARN(node_->get_logger(),
                    "目标 #%d 被取消：本次已移动 %.3f m，累计移动 %.3f m。",
                    current_goal_index_ + 1, distance_snapshot.goal_distance_m,
                    distance_snapshot.cumulative_distance_m);
        LOOP_RCLCPP_WARN(node_->get_logger(), "Goal #%d canceled. Return FAILURE.", current_goal_index_ + 1);
        goal_handle_.reset();
        result_ready_ = false;
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void LoopNavigateFourPoints::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    const double current_x = msg->pose.pose.position.x;
    const double current_y = msg->pose.pose.position.y;
    if (!std::isfinite(current_x) || !std::isfinite(current_y)) {
        LOOP_RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "收到非有限值里程计位置，本次消息不参与距离统计。");
        return;
    }

    std::lock_guard<std::mutex> lock(distance_mutex_);
    if (!distance_tracking_active_) {
        return;
    }

    received_odom_for_goal_ = true;
    if (!has_last_odom_pose_) {
        last_odom_x_ = current_x;
        last_odom_y_ = current_y;
        has_last_odom_pose_ = true;
        return;
    }

    const double increment = std::hypot(current_x - last_odom_x_, current_y - last_odom_y_);
    last_odom_x_ = current_x;
    last_odom_y_ = current_y;
    if (increment > odom_max_increment_m_) {
        LOOP_RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "里程计单次位置跳变 %.3f m，超过阈值 %.3f m，本次增量已忽略。",
                             increment, odom_max_increment_m_);
        return;
    }

    current_goal_distance_m_ += increment;
    cumulative_navigation_distance_m_ += increment;
}

void LoopNavigateFourPoints::startDistanceTracking() {
    std::lock_guard<std::mutex> lock(distance_mutex_);
    current_goal_distance_m_ = 0.0;
    distance_tracking_active_ = true;
    has_last_odom_pose_ = false;
    received_odom_for_goal_ = false;
}

LoopNavigateFourPoints::DistanceSnapshot LoopNavigateFourPoints::stopDistanceTracking() {
    std::lock_guard<std::mutex> lock(distance_mutex_);
    DistanceSnapshot snapshot;
    snapshot.goal_distance_m = current_goal_distance_m_;
    snapshot.cumulative_distance_m = cumulative_navigation_distance_m_;
    snapshot.active = distance_tracking_active_;
    snapshot.valid = received_odom_for_goal_;
    distance_tracking_active_ = false;
    has_last_odom_pose_ = false;
    return snapshot;
}

void LoopNavigateFourPoints::spinNodeForDuration(double duration_sec, const char* reason) {
    const auto wall_start_time = std::chrono::system_clock::now();
    const auto end_time = std::chrono::steady_clock::now() + std::chrono::duration<double>(duration_sec);
    if (reason != nullptr) {
        LOOP_RCLCPP_INFO_AT(node_->get_logger(), formatUtc8Time(wall_start_time), "%s开始计时：%.2f 秒。",
                            reason, duration_sec);
    }
    while (std::chrono::steady_clock::now() < end_time && rclcpp::ok()) {
        rclcpp::spin_some(node_);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}

LoopNavigateFourPoints::NavigationErrorSample LoopNavigateFourPoints::getNavigationErrorSample(const Pose& goal) {
    NavigationErrorSample sample;
    rclcpp::spin_some(node_);
    if (use_goal_checker_error_ && getLatestGoalCheckerErrorSample(goal, sample)) {
        LOOP_RCLCPP_INFO(node_->get_logger(),
                    "使用 goal_checker 发布的最终误差写入导航统计：lon=%.4f, lat=%.4f, yaw=%.4f, dist=%.4f",
                    sample.lon_error, sample.lat_error, sample.yaw_error, sample.dist_error);
        return sample;
    }

    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> yaws;
    std::vector<double> lon_errors;
    std::vector<double> lat_errors;
    std::vector<double> yaw_errors;
    std::vector<double> dist_errors;
    xs.reserve(pose_sample_count_);
    ys.reserve(pose_sample_count_);
    yaws.reserve(pose_sample_count_);
    lon_errors.reserve(pose_sample_count_);
    lat_errors.reserve(pose_sample_count_);
    yaw_errors.reserve(pose_sample_count_);
    dist_errors.reserve(pose_sample_count_);

    const double cos_yaw = std::cos(goal.yaw);
    const double sin_yaw = std::sin(goal.yaw);

    for (std::size_t i = 0; i < pose_sample_count_; ++i) {
        try {
            rclcpp::spin_some(node_);
            const auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

            const auto& tr = transform.transform.translation;
            const auto& rot = transform.transform.rotation;

            tf2::Quaternion q;
            tf2::fromMsg(rot, q);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            const double dx = tr.x - goal.x;
            const double dy = tr.y - goal.y;
            xs.push_back(tr.x);
            ys.push_back(tr.y);
            yaws.push_back(yaw);
            lon_errors.push_back(cos_yaw * dx + sin_yaw * dy);
            lat_errors.push_back(-sin_yaw * dx + cos_yaw * dy);
            yaw_errors.push_back(normalize_angle(yaw - goal.yaw));
            dist_errors.push_back(std::hypot(dx, dy));
        } catch (const tf2::TransformException& ex) {
            LOOP_RCLCPP_WARN(node_->get_logger(), "误差统计获取机器人 TF 失败: %s，继续重试。", ex.what());
            spinNodeForDuration(0.5);
            continue;
        }

        if (i + 1 < pose_sample_count_) {
            spinNodeForDuration(pose_sample_interval_sec_);
        }
    }

    if (xs.empty()) {
        return sample;
    }

    sample.current.x = std::accumulate(xs.begin(), xs.end(), 0.0) / xs.size();
    sample.current.y = std::accumulate(ys.begin(), ys.end(), 0.0) / ys.size();
    sample.current.yaw = circularMean(yaws);
    sample.lon_error = std::accumulate(lon_errors.begin(), lon_errors.end(), 0.0) / lon_errors.size();
    sample.lat_error = std::accumulate(lat_errors.begin(), lat_errors.end(), 0.0) / lat_errors.size();
    sample.yaw_error = circularMean(yaw_errors);
    sample.dist_error = std::accumulate(dist_errors.begin(), dist_errors.end(), 0.0) / dist_errors.size();
    sample.within_desired_tolerance =
        isWithinPositionTolerance(sample.lon_error, sample.lat_error, desired_position_tolerance_);
    sample.within_accept_tolerance =
        isWithinPositionTolerance(sample.lon_error, sample.lat_error, accept_position_tolerance_);
    sample.valid = true;

    std::ostringstream xs_stream;
    std::ostringstream ys_stream;
    std::ostringstream yaws_stream;
    std::ostringstream lon_stream;
    std::ostringstream lat_stream;
    std::ostringstream yaw_err_stream;
    for (std::size_t i = 0; i < xs.size(); ++i) {
        if (i > 0) {
            xs_stream << ", ";
            ys_stream << ", ";
            yaws_stream << ", ";
            lon_stream << ", ";
            lat_stream << ", ";
            yaw_err_stream << ", ";
        }
        xs_stream << std::fixed << std::setprecision(4) << xs[i];
        ys_stream << std::fixed << std::setprecision(4) << ys[i];
        yaws_stream << std::fixed << std::setprecision(4) << yaws[i];
        lon_stream << std::fixed << std::setprecision(4) << lon_errors[i];
        lat_stream << std::fixed << std::setprecision(4) << lat_errors[i];
        yaw_err_stream << std::fixed << std::setprecision(4) << yaw_errors[i];
    }

    LOOP_RCLCPP_INFO(node_->get_logger(),
                "误差统计TF采样: x=[%s], y=[%s], yaw=[%s]; TF平均值: x=%.4f, y=%.4f, yaw=%.4f",
                xs_stream.str().c_str(), ys_stream.str().c_str(), yaws_stream.str().c_str(), sample.current.x,
                sample.current.y, sample.current.yaw);
    LOOP_RCLCPP_INFO(node_->get_logger(),
                "误差统计窗口: lon=[%s], lat=[%s], yaw=[%s]; 平均误差: lon=%.4f, lat=%.4f, yaw=%.4f, dist=%.4f",
                lon_stream.str().c_str(), lat_stream.str().c_str(), yaw_err_stream.str().c_str(), sample.lon_error,
                sample.lat_error, sample.yaw_error, sample.dist_error);
    return sample;
}

bool LoopNavigateFourPoints::getLatestGoalCheckerErrorSample(const Pose& goal, NavigationErrorSample& sample) {
    std::lock_guard<std::mutex> lock(latest_goal_error_mutex_);
    if (!has_latest_goal_error_) {
        return false;
    }

    const auto& msg = latest_goal_error_;
    if (msg.status.empty()) {
        return false;
    }

    const auto& status = msg.status.front();
    if (status.name != "navigation_goal_error") {
        LOOP_RCLCPP_WARN(node_->get_logger(), "收到的 /navigation_goal_error 诊断消息 name 不匹配，改用 TF 窗口统计。");
        return false;
    }

    auto get_value = [&status](const std::string& key, std::string& value) {
        for (const auto& item : status.values) {
            if (item.key == key) {
                value = item.value;
                return true;
            }
        }
        return false;
    };
    auto get_double = [&get_value](const std::string& key, double& value) {
        std::string text;
        if (!get_value(key, text)) {
            return false;
        }
        try {
            value = std::stod(text);
        } catch (const std::exception&) {
            return false;
        }
        return true;
    };
    auto get_bool = [&get_value](const std::string& key, bool& value) {
        std::string text;
        if (!get_value(key, text)) {
            return false;
        }
        value = text == "true" || text == "1";
        return true;
    };

    const rclcpp::Time msg_time(msg.header.stamp);
    if (msg_time < goal_ros_start_time_) {
        LOOP_RCLCPP_WARN(node_->get_logger(), "goal_checker 误差消息属于旧目标，改用 TF 窗口统计。");
        return false;
    }

    if ((node_->now() - msg_time).seconds() > 10.0) {
        LOOP_RCLCPP_WARN(node_->get_logger(), "goal_checker 误差消息超过 10s，改用 TF 窗口统计。");
        return false;
    }

    double msg_goal_x = 0.0;
    double msg_goal_y = 0.0;
    double msg_goal_yaw = 0.0;
    if (!get_double("goal_x", msg_goal_x) ||
        !get_double("goal_y", msg_goal_y) ||
        !get_double("goal_yaw", msg_goal_yaw)) {
        LOOP_RCLCPP_WARN(node_->get_logger(), "goal_checker 误差消息缺少目标位姿字段，改用 TF 窗口统计。");
        return false;
    }

    const double goal_dx = msg_goal_x - goal.x;
    const double goal_dy = msg_goal_y - goal.y;
    const double goal_dyaw = normalize_angle(msg_goal_yaw - goal.yaw);
    if (std::hypot(goal_dx, goal_dy) > accept_position_tolerance_ || std::abs(goal_dyaw) > 0.05) {
        LOOP_RCLCPP_WARN(node_->get_logger(),
                    "goal_checker 误差消息的 path 终点与原始目标点略有差异，仍采用 controller 最终误差："
                    "goal_delta=%.4f m, yaw_delta=%.4f rad",
                    std::hypot(goal_dx, goal_dy), goal_dyaw);
    }

    std::string result_type = status.message;
    std::string result_type_value;
    if (get_value("result_type", result_type_value)) {
        result_type = result_type_value;
    }

    if (!get_double("current_x", sample.current.x) ||
        !get_double("current_y", sample.current.y) ||
        !get_double("current_yaw", sample.current.yaw) ||
        !get_double("longitudinal_error", sample.lon_error) ||
        !get_double("lateral_error", sample.lat_error) ||
        !get_double("yaw_error", sample.yaw_error) ||
        !get_double("distance_error", sample.dist_error) ||
        !get_bool("within_accept_tolerance", sample.within_accept_tolerance)) {
        LOOP_RCLCPP_WARN(node_->get_logger(), "goal_checker 误差消息字段不完整，改用 TF 窗口统计。");
        return false;
    }

    // 1cm统计属于测试指标，不再依赖已删除的高精度GoalChecker模式。
    sample.within_desired_tolerance =
        isWithinPositionTolerance(sample.lon_error, sample.lat_error, desired_position_tolerance_);
    sample.from_goal_checker = true;
    sample.valid = true;
    LOOP_RCLCPP_INFO(node_->get_logger(),
                "采用 goal_checker 最终误差消息：result_type=%s, lon=%.4f, lat=%.4f, yaw=%.4f, dist=%.4f",
                result_type.c_str(), sample.lon_error, sample.lat_error, sample.yaw_error, sample.dist_error);
    return true;
}

void LoopNavigateFourPoints::logNavigationError(const Pose& goal,
                                                const NavigationErrorSample& error_sample,
                                                const DistanceSnapshot& distance_snapshot,
                                                double elapsed_sec,
                                                bool succeeded,
                                                std::size_t goal_index) {
    if (!node_) {
        return;
    }

    const Pose& current = error_sample.current;
    const double err_lon = error_sample.lon_error;
    const double err_lat = error_sample.lat_error;
    const double err_yaw = error_sample.yaw_error;
    const double err_dist = error_sample.dist_error;
    if (!error_sample.valid) {
        LOOP_RCLCPP_WARN(node_->get_logger(), "误差统计未获取到有效 TF 样本，本次写入零误差样本。");
    }
    const bool within_1cm = error_sample.from_goal_checker ?
        error_sample.within_desired_tolerance :
        isWithinPositionTolerance(err_lon, err_lat, desired_position_tolerance_);
    const bool within_2cm = error_sample.from_goal_checker ?
        error_sample.within_accept_tolerance :
        isWithinPositionTolerance(err_lon, err_lat, accept_position_tolerance_);

    lon_errors_.push_back(err_lon);
    lat_errors_.push_back(err_lat);
    yaw_errors_.push_back(std::abs(err_yaw));
    dist_errors_.push_back(err_dist);
    if (within_1cm) {
        within_1cm_count_++;
    }
    if (within_2cm) {
        within_2cm_count_++;
    }

    if (succeeded) {
        success_lon_errors_.push_back(err_lon);
        success_lat_errors_.push_back(err_lat);
        success_yaw_errors_.push_back(std::abs(err_yaw));
        success_dist_errors_.push_back(err_dist);
        if (within_1cm) {
            success_within_1cm_count_++;
        }
        if (within_2cm) {
            success_within_2cm_count_++;
        }
    } else {
        failed_lon_errors_.push_back(err_lon);
        failed_lat_errors_.push_back(err_lat);
        failed_yaw_errors_.push_back(std::abs(err_yaw));
        failed_dist_errors_.push_back(err_dist);
        if (within_1cm) {
            failed_within_1cm_count_++;
        }
        if (within_2cm) {
            failed_within_2cm_count_++;
        }
    }

    if (goal_index < per_goal_stats_.size()) {
        auto& goal_stats = per_goal_stats_[goal_index];
        goal_stats.elapsed_times.push_back(elapsed_sec);
        if (succeeded) {
            goal_stats.succeeded++;
            goal_stats.success_elapsed_times.push_back(elapsed_sec);
            goal_stats.success_lon_errors.push_back(err_lon);
            goal_stats.success_lat_errors.push_back(err_lat);
            goal_stats.success_yaw_errors.push_back(std::abs(err_yaw));
            goal_stats.success_dist_errors.push_back(err_dist);
            if (within_1cm) {
                goal_stats.success_within_1cm++;
            }
            if (within_2cm) {
                goal_stats.success_within_2cm++;
            }
        } else {
            goal_stats.failed++;
            goal_stats.failed_elapsed_times.push_back(elapsed_sec);
            goal_stats.failed_lon_errors.push_back(err_lon);
            goal_stats.failed_lat_errors.push_back(err_lat);
            goal_stats.failed_yaw_errors.push_back(std::abs(err_yaw));
            goal_stats.failed_dist_errors.push_back(err_dist);
            if (within_1cm) {
                goal_stats.failed_within_1cm++;
            }
            if (within_2cm) {
                goal_stats.failed_within_2cm++;
            }
        }
    }

    if (log_file_.is_open()) {
        const std::string goal_id =
            goal_index < goal_ids_.size() ? goal_ids_[goal_index] : "goal_" + std::to_string(goal_index);
        log_file_ << (succeeded ? "成功" : "失败") << " " << std::fixed << std::setprecision(4) << elapsed_sec
                  << " " << goal_index << " " << goal_id << " " << goal.x << " " << goal.y << " " << goal.yaw << " "
                  << current.x << " " << current.y << " " << current.yaw << " " << err_lon << " " << err_lat << " "
                  << err_yaw << " " << err_dist << " " << distance_snapshot.goal_distance_m << " "
                  << distance_snapshot.cumulative_distance_m << " " << (distance_snapshot.valid ? 1 : 0) << " "
                  << (within_1cm ? 1 : 0) << " " << (within_2cm ? 1 : 0)
                  << std::endl;
    }

    if (dist_errors_.size() % stats_log_interval_ == 0) {
        calculateAndLogStats();
    }
}

bool LoopNavigateFourPoints::isWithinPositionTolerance(double lon_error, double lat_error, double tolerance) const {
    return std::abs(lon_error) <= tolerance && std::abs(lat_error) <= tolerance;
}

LoopNavigateFourPoints::MetricStats LoopNavigateFourPoints::calculateMetricStats(const std::vector<double>& data) const {
    MetricStats stats;
    if (data.empty()) {
        return stats;
    }

    const double sum = std::accumulate(
        data.begin(), data.end(), 0.0, [](double acc, double value) { return acc + std::abs(value); });
    stats.mean = sum / data.size();

    double sum_sq_diff = 0.0;
    for (double value : data) {
        const double abs_value = std::abs(value);
        sum_sq_diff += (abs_value - stats.mean) * (abs_value - stats.mean);
    }
    stats.std_dev = std::sqrt(sum_sq_diff / data.size());

    if (stats.std_dev == 0.0) {
        stats.sigma3_probability = 100.0;
        return stats;
    }

    int in_3sigma = 0;
    for (double value : data) {
        if (std::abs(std::abs(value) - stats.mean) <= 3.0 * stats.std_dev) {
            ++in_3sigma;
        }
    }
    stats.sigma3_probability = static_cast<double>(in_3sigma) * 100.0 / data.size();
    return stats;
}

void LoopNavigateFourPoints::calculateAndLogStats(bool force) {
    if (dist_errors_.empty()) {
        return;
    }

    if (!force && dist_errors_.size() == last_stats_logged_sample_count_) {
        return;
    }

    const int total_nav = succeeded_nav_ + failed_nav_;
    const double success_rate = total_nav > 0 ? static_cast<double>(succeeded_nav_) * 100.0 / total_nav : 0.0;
    auto percentage = [](int count, std::size_t total) {
        return total > 0 ? static_cast<double>(count) * 100.0 / static_cast<double>(total) : 0.0;
    };

    auto write_metric_block = [this](const std::string& label,
                                     const std::vector<double>& lon_errors,
                                     const std::vector<double>& lat_errors,
                                     const std::vector<double>& dist_errors,
                                     const std::vector<double>& yaw_errors,
                                     int within_1cm_count,
                                     int within_2cm_count,
                                     const auto& percentage_fn) {
        log_file_ << "- " << label << "\n";
        log_file_ << "  样本数: " << dist_errors.size() << "\n";
        if (dist_errors.empty()) {
            log_file_ << "  误差: 无样本\n";
            return;
        }

        const MetricStats lon_stats = calculateMetricStats(lon_errors);
        const MetricStats lat_stats = calculateMetricStats(lat_errors);
        const MetricStats dist_stats = calculateMetricStats(dist_errors);
        const MetricStats yaw_stats = calculateMetricStats(yaw_errors);

        log_file_ << std::fixed << std::setprecision(4);
        log_file_ << "  纵向误差: MAE " << lon_stats.mean << " m, 标准差 " << lon_stats.std_dev << " m\n";
        log_file_ << "  横向误差: MAE " << lat_stats.mean << " m, 标准差 " << lat_stats.std_dev << " m\n";
        log_file_ << "  距离误差: MAE " << dist_stats.mean << " m, 标准差 " << dist_stats.std_dev << " m\n";
        log_file_ << std::fixed << std::setprecision(2);
        log_file_ << "  角度误差: MAE " << yaw_stats.mean * 180.0 / M_PI << " deg, 标准差 "
                  << yaw_stats.std_dev * 180.0 / M_PI << " deg\n";
        log_file_ << "  位置误差满足期望阈值(" << desired_position_tolerance_ << " m): " << within_1cm_count
                  << " 次, 占比 " << percentage_fn(within_1cm_count, dist_errors.size()) << "%\n";
        log_file_ << "  位置误差满足接受阈值(" << accept_position_tolerance_ << " m): " << within_2cm_count
                  << " 次, 占比 " << percentage_fn(within_2cm_count, dist_errors.size()) << "%\n";
    };

    auto calculate_average_elapsed = [](const std::vector<double>& elapsed_times) {
        if (elapsed_times.empty()) {
            return 0.0;
        }
        return std::accumulate(elapsed_times.begin(), elapsed_times.end(), 0.0) / elapsed_times.size();
    };

    auto write_average_elapsed = [&calculate_average_elapsed](const std::string& label,
                                                              const std::vector<double>& elapsed_times,
                                                              std::ostream& os) {
        os << label;
        if (elapsed_times.empty()) {
            os << "无样本";
            return;
        }
        os << std::fixed << std::setprecision(4) << calculate_average_elapsed(elapsed_times) << " s";
    };

    if (log_file_.is_open()) {
        log_file_ << "\n\n================ 导航统计汇总（截至当前轮） ================\n";
        log_file_ << "轮数: " << completed_rounds_ << "/";
        if (total_rounds_ > 0) {
            log_file_ << total_rounds_;
        } else {
            log_file_ << "无限";
        }
        log_file_ << " | 样本数: " << dist_errors_.size() << " | 成功: " << succeeded_nav_
                  << " | 失败: " << failed_nav_ << " | 成功率: " << std::fixed << std::setprecision(2)
                  << success_rate << "%\n";
        {
            std::lock_guard<std::mutex> lock(distance_mutex_);
            log_file_ << std::fixed << std::setprecision(3)
                      << "累计导航移动距离: " << cumulative_navigation_distance_m_ << " m\n\n";
        }

        log_file_ << "[总体误差]\n";
        write_metric_block("全部样本", lon_errors_, lat_errors_, dist_errors_, yaw_errors_, within_1cm_count_,
                           within_2cm_count_, percentage);
        write_metric_block("成功样本", success_lon_errors_, success_lat_errors_, success_dist_errors_,
                           success_yaw_errors_, success_within_1cm_count_, success_within_2cm_count_, percentage);
        write_metric_block("失败样本", failed_lon_errors_, failed_lat_errors_, failed_dist_errors_, failed_yaw_errors_,
                           failed_within_1cm_count_, failed_within_2cm_count_, percentage);

        log_file_ << "\n[各点成功率与平均耗时]\n";
        for (std::size_t i = 0; i < per_goal_stats_.size(); ++i) {
            const auto& goal_stats = per_goal_stats_[i];
            const int goal_total = goal_stats.succeeded + goal_stats.failed;
            const double goal_success_rate =
                goal_total > 0 ? static_cast<double>(goal_stats.succeeded) * 100.0 / goal_total : 0.0;
            const std::string goal_id = goal_stats.id.empty() ? "goal_" + std::to_string(i) : goal_stats.id;

            log_file_ << "- 点 " << i << " (" << goal_id << "): 总数 " << goal_total << ", 成功 "
                      << goal_stats.succeeded << ", 失败 " << goal_stats.failed << ", 成功率 " << std::fixed
                      << std::setprecision(2) << goal_success_rate << "%, ";
            log_file_ << "期望阈值内 " << (goal_stats.success_within_1cm + goal_stats.failed_within_1cm)
                      << " 次, 接受阈值内 " << (goal_stats.success_within_2cm + goal_stats.failed_within_2cm)
                      << " 次, ";
            write_average_elapsed("总平均耗时 ", goal_stats.elapsed_times, log_file_);
            log_file_ << ", ";
            write_average_elapsed("成功平均耗时 ", goal_stats.success_elapsed_times, log_file_);
            log_file_ << ", ";
            write_average_elapsed("失败平均耗时 ", goal_stats.failed_elapsed_times, log_file_);
            log_file_ << "\n";
        }

        log_file_ << "\n[各点误差]\n";
        for (std::size_t i = 0; i < per_goal_stats_.size(); ++i) {
            const auto& goal_stats = per_goal_stats_[i];
            const std::string goal_id = goal_stats.id.empty() ? "goal_" + std::to_string(i) : goal_stats.id;
            log_file_ << "点 " << i << " (" << goal_id << ")\n";
            write_metric_block("成功误差", goal_stats.success_lon_errors, goal_stats.success_lat_errors,
                               goal_stats.success_dist_errors, goal_stats.success_yaw_errors,
                               goal_stats.success_within_1cm, goal_stats.success_within_2cm, percentage);
            write_metric_block("失败误差", goal_stats.failed_lon_errors, goal_stats.failed_lat_errors,
                               goal_stats.failed_dist_errors, goal_stats.failed_yaw_errors,
                               goal_stats.failed_within_1cm, goal_stats.failed_within_2cm, percentage);
        }

        log_file_ << "============================================================\n";
        log_file_.flush();
    }

    const MetricStats dist_stats = calculateMetricStats(dist_errors_);
    const MetricStats yaw_stats = calculateMetricStats(yaw_errors_);
    LOOP_RCLCPP_INFO(node_->get_logger(),
                "Navigation stats (%zu samples): dist mean=%.4f m, dist 3sigma=%.2f%%, yaw mean=%.2f deg, "
                "yaw 3sigma=%.2f%%",
                dist_errors_.size(), dist_stats.mean, dist_stats.sigma3_probability, yaw_stats.mean * 180.0 / M_PI,
                yaw_stats.sigma3_probability);

    last_stats_logged_sample_count_ = dist_errors_.size();
}

void LoopNavigateFourPoints::onHalted() {
    // 先让当前代次失效，防止取消结果晚到后被下一次导航误用。
    ++goal_generation_;
    const auto distance_snapshot = stopDistanceTracking();
    if (distance_snapshot.active) {
        LOOP_RCLCPP_WARN(node_->get_logger(),
                    "当前导航被中断：本次已移动 %.3f m，累计移动 %.3f m。",
                    distance_snapshot.goal_distance_m, distance_snapshot.cumulative_distance_m);
    }
    if (goal_handle_) {
        LOOP_RCLCPP_WARN(node_->get_logger(), "LoopNavigateFourPoints halted, canceling goal.");
        action_client_->async_cancel_goal(goal_handle_);
        goal_handle_.reset();
    }
    future_goal_handle_ = {};
    result_ready_ = false;
    result_code_ = rclcpp_action::ResultCode::UNKNOWN;
}

//--- Implementation for CheckBatteryAboveThreshold ---
CheckBatteryAboveThreshold::CheckBatteryAboveThreshold(const std::string& name,
                                                       const BT::NodeConfiguration& config,
                                                       rclcpp::Node::SharedPtr node_ptr)
    : BT::ConditionNode(name, config), node_(node_ptr) {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    battery_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics_agg", qos, std::bind(&CheckBatteryAboveThreshold::battery_callback, this, std::placeholders::_1));
}

BT::PortsList CheckBatteryAboveThreshold::providedPorts() {
    return {
        BT::InputPort<double>("threshold", "Minimum battery percentage (0.0 to 100.0)"),
        BT::InputPort<bool>("enabled", true, "Whether battery monitoring is enabled"),
    };
}

void CheckBatteryAboveThreshold::battery_callback(
    const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
    latest_battery_percentage_ = parseBatteryPercentage(*msg);
    if (latest_battery_percentage_ >= 0.0f) {
        LOOP_RCLCPP_INFO(node_->get_logger(), "当前电量：%.3f%%", latest_battery_percentage_);
    }
}

BT::NodeStatus CheckBatteryAboveThreshold::tick() {
    auto enabled = getInput<bool>("enabled");
    if (!enabled) {
        LOOP_RCLCPP_ERROR(node_->get_logger(), "电量检查的 enabled 端口缺失或无效。");
        return BT::NodeStatus::FAILURE;
    }

    if (!enabled.value()) {
        LOOP_RCLCPP_INFO_ONCE(node_->get_logger(), "电量监控未启用，跳过电量检查。");
        return BT::NodeStatus::SUCCESS;
    }

    auto threshold = getInput<double>("threshold");
    if (!threshold) {
        LOOP_RCLCPP_ERROR(node_->get_logger(), "电量检查的 threshold 端口缺失。");
        return BT::NodeStatus::FAILURE;
    }

    // 先处理诊断回调，保证本次判断尽量使用最新电量。
    rclcpp::spin_some(node_);

    if (latest_battery_percentage_ < 0) {
        LOOP_RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "尚未收到有效电量数据。");
        return BT::NodeStatus::FAILURE;  // Keep retrying
    }

    LOOP_RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "电量检查：当前 %.2f%%，要求 >= %.2f%%。", latest_battery_percentage_,
                         threshold.value());

    bool battery_ok = latest_battery_percentage_ >= threshold.value();

    // 电量不足时阻止循环导航节点继续发送新目标。
    if (!battery_ok) {
        try {
            config().blackboard->set("should_stop_navigation", true);
            // LOOP_RCLCPP_WARN(node_->get_logger(), "Battery low! Setting stop flag.");
        } catch (...) {
        }
    }

    return battery_ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

//--- Implementation for WaitForBatteryAboveThreshold ---
WaitForBatteryAboveThreshold::WaitForBatteryAboveThreshold(const std::string& name,
                                                           const BT::NodeConfiguration& config,
                                                           rclcpp::Node::SharedPtr node_ptr)
    : BtActionNode(name, config, node_ptr) {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    battery_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics_agg", qos,
        std::bind(&WaitForBatteryAboveThreshold::battery_callback, this, std::placeholders::_1));
}

BT::PortsList WaitForBatteryAboveThreshold::providedPorts() {
    return {BT::InputPort<double>("threshold", "Battery percentage to wait for")};
}

void WaitForBatteryAboveThreshold::battery_callback(
    const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
    latest_battery_percentage_ = parseBatteryPercentage(*msg);
    if (latest_battery_percentage_ >= 0.0f) {
        battery_updated_ = true;
    }
}

BT::NodeStatus WaitForBatteryAboveThreshold::onStart() {
    return onRunning();
}

BT::NodeStatus WaitForBatteryAboveThreshold::onRunning() {
    auto threshold = getInput<double>("threshold");
    if (!threshold || threshold.value() < 0.0 || threshold.value() > 100.0) {
        LOOP_RCLCPP_ERROR(node_->get_logger(), "WaitForBatteryAboveThreshold 的 threshold 缺失或超出 0~100。");
        return BT::NodeStatus::FAILURE;
    }

    rclcpp::spin_some(node_);
    if (latest_battery_percentage_ < 0.0f) {
        LOOP_RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "等待底盘发布有效电量，暂不启动导航或回充。");
        return BT::NodeStatus::RUNNING;
    }

    if (latest_battery_percentage_ < threshold.value()) {
        if (battery_updated_) {
            LOOP_RCLCPP_INFO(node_->get_logger(), "充电等待中：当前 %.2f%%，退桩要求 >= %.2f%%。",
                             latest_battery_percentage_, threshold.value());
            battery_updated_ = false;
        }
        return BT::NodeStatus::RUNNING;
    }

    if (battery_updated_) {
        if (threshold.value() == 0.0) {
            LOOP_RCLCPP_INFO(node_->get_logger(), "已收到有效电量：当前 %.2f%%。", latest_battery_percentage_);
        } else {
            LOOP_RCLCPP_INFO(node_->get_logger(), "充电完成：当前 %.2f%%，已达到退桩阈值 %.2f%%。",
                             latest_battery_percentage_, threshold.value());
        }
        battery_updated_ = false;
    }
    return BT::NodeStatus::SUCCESS;
}

//--- Implementation for CancelNavigation ---
CancelNavigation::CancelNavigation(const std::string& name,
                                   const BT::NodeConfiguration& config,
                                   rclcpp::Node::SharedPtr node_ptr)
    : BtActionNode(name, config, node_ptr) {
    action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");
}

BT::NodeStatus CancelNavigation::onStart() {
    LOOP_RCLCPP_INFO(node_->get_logger(), "正在取消所有导航目标。");
    action_client_->async_cancel_all_goals();
    return BT::NodeStatus::SUCCESS;
}

//--- Implementation for Undocking ---
Undocking::Undocking(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node_ptr)
    : BtActionNode(name, config, node_ptr) {
    vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("control/cmd_vel", rclcpp::SystemDefaultsQoS());
}

BT::PortsList Undocking::providedPorts() {
    return {
        BT::InputPort<double>("duration", 3.0, "Undocking duration in seconds"),
        BT::InputPort<double>("linear_speed", 0.15, "Undocking linear speed in m/s"),
    };
}

void Undocking::publish_cmd(double linear_x) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_x;
    cmd.angular.z = 0.0;
    vel_pub_->publish(cmd);
}

BT::NodeStatus Undocking::onStart() {
    // 退桩期间保持导航暂停，避免 Nav2 与直接速度控制同时输出。
    try {
        config().blackboard->set("nav_paused", true);
    } catch (...) {
        LOOP_RCLCPP_WARN(node_->get_logger(), "Undocking: failed to set 'nav_paused' on blackboard");
    }

    // 退桩时间和速度均由 YAML 经 blackboard 传入。
    if (auto dur = getInput<double>("duration")) {
        duration_sec_ = dur.value();
    } else {
        LOOP_RCLCPP_WARN(node_->get_logger(), "Undocking: no 'duration' provided, using default %.2f s", duration_sec_);
    }
    if (auto speed = getInput<double>("linear_speed")) {
        linear_speed_ = speed.value();
    } else {
        LOOP_RCLCPP_WARN(node_->get_logger(), "Undocking: no 'linear_speed' provided, using default %.2f m/s",
                    linear_speed_);
    }

    if (duration_sec_ <= 0.0 || linear_speed_ <= 0.0) {
        LOOP_RCLCPP_WARN(node_->get_logger(),
                    "Undocking: duration (%.2f) 或 linear_speed (%.2f) 非正数，立即停止。",
                    duration_sec_, linear_speed_);
        publish_cmd(0.0);
        // Clear pause flag immediately
        try {
            config().blackboard->set("nav_paused", false);
            config().blackboard->set("charging_active", false);
            config().blackboard->set("should_stop_navigation", false);
        } catch (...) {
        }
        return BT::NodeStatus::SUCCESS;
    }

    const auto wall_start_time = std::chrono::system_clock::now();
    start_time_ = std::chrono::steady_clock::now();
    LOOP_RCLCPP_INFO_AT(node_->get_logger(), formatUtc8Time(wall_start_time),
                        "退桩开始计时：持续 %.2f 秒，速度 %.2f m/s。", duration_sec_, linear_speed_);
    publish_cmd(linear_speed_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Undocking::onRunning() {
    auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
    if (elapsed >= duration_sec_) {
        publish_cmd(0.0);
        // Clear flags so navigation can resume and charging mode ends
        try {
            config().blackboard->set("nav_paused", false);
            config().blackboard->set("charging_active", false);
            config().blackboard->set("should_stop_navigation", false);
        } catch (...) {
        }
        LOOP_RCLCPP_INFO(node_->get_logger(), "Undocking finished (ran %.2f / %.2f s)", elapsed, duration_sec_);
        return BT::NodeStatus::SUCCESS;
    }

    // keep moving forward
    publish_cmd(linear_speed_);
    return BT::NodeStatus::RUNNING;
}

void Undocking::onHalted() {
    LOOP_RCLCPP_WARN(node_->get_logger(), "Undocking halted. Stopping robot.");
    publish_cmd(0.0);
}

//--- Implementation for NavigateToFirstPointAndStop ---
NavigateToFirstPointAndStop::NavigateToFirstPointAndStop(const std::string& name,
                                                         const BT::NodeConfiguration& config,
                                                         rclcpp::Node::SharedPtr node_ptr)
    : BtActionNode(name, config, node_ptr) {
    action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");
    if (!node_->has_parameter("navigation.action_server_wait_timeout_sec")) {
        node_->declare_parameter("navigation.action_server_wait_timeout_sec", action_server_wait_timeout_sec_);
    }
    node_->get_parameter("navigation.action_server_wait_timeout_sec", action_server_wait_timeout_sec_);
    if (action_server_wait_timeout_sec_ <= 0.0) {
        action_server_wait_timeout_sec_ = 2.0;
    }
}

BT::PortsList NavigateToFirstPointAndStop::providedPorts() {
    return {BT::InputPort<std::string>("points", "Path to the JSON file with navigation points")};
}

BT::NodeStatus NavigateToFirstPointAndStop::onStart() {
    if (goals_.empty()) {
        auto json_path = getInput<std::string>("points");
        if (!json_path) {
            LOOP_RCLCPP_ERROR(node_->get_logger(), "[LowBattery] 'points' port is missing.");
            return BT::NodeStatus::FAILURE;
        }
        goals_ = parseNavPoints(json_path.value(), node_->get_logger());
        if (goals_.empty()) {
            return BT::NodeStatus::FAILURE;
        }
    }

    if (!action_client_->wait_for_action_server(std::chrono::duration<double>(action_server_wait_timeout_sec_))) {
        LOOP_RCLCPP_ERROR(node_->get_logger(), "[LowBattery] Action server /navigate_to_pose not available.");
        return BT::NodeStatus::FAILURE;
    }

    result_ready_ = false;
    result_code_ = rclcpp_action::ResultCode::UNKNOWN;
    goal_handle_.reset();

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goals_[0];  // 低电量停止点固定使用 JSON 中的第一个点。
    goal_msg.pose.header.stamp = node_->now();

    LOOP_RCLCPP_WARN(node_->get_logger(), "[LowBattery] Battery low! Navigating to first point (%.2f, %.2f) then stopping.",
                goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](GoalHandleNavigateToPose::SharedPtr gh) {
        if (!gh) {
            LOOP_RCLCPP_ERROR(node_->get_logger(), "[LowBattery] Goal rejected by server.");
        } else {
            goal_handle_ = gh;
        }
    };
    send_goal_options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult& result) {
        result_code_ = result.code;
        result_ready_ = true;
    };

    future_goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToFirstPointAndStop::onRunning() {
    // Wait until goal is accepted
    if (!goal_handle_) {
        if (future_goal_handle_.valid() &&
            future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            auto tmp = future_goal_handle_.get();
            if (!tmp) {
                LOOP_RCLCPP_ERROR(node_->get_logger(), "[LowBattery] Goal was rejected by server.");
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::RUNNING;
    }

    if (!result_ready_) {
        return BT::NodeStatus::RUNNING;
    }

    if (result_code_ == rclcpp_action::ResultCode::SUCCEEDED) {
        LOOP_RCLCPP_INFO(node_->get_logger(), "[LowBattery] Reached first point. Signaling program to stop.");
    } else if (result_code_ == rclcpp_action::ResultCode::ABORTED) {
        LOOP_RCLCPP_WARN(node_->get_logger(), "[LowBattery] Navigation to first point aborted. Stopping anyway.");
    } else {
        LOOP_RCLCPP_WARN(node_->get_logger(), "[LowBattery] Navigation canceled or unknown result. Stopping anyway.");
    }

    // 无论返回首点成功还是失败，都通知主循环结束，避免低电量继续派发目标。
    try {
        config().blackboard->set("low_battery_shutdown", true);
    } catch (...) {
        LOOP_RCLCPP_ERROR(node_->get_logger(), "[LowBattery] Failed to set 'low_battery_shutdown' on blackboard.");
    }
    return BT::NodeStatus::SUCCESS;
}

void NavigateToFirstPointAndStop::onHalted() {
    if (goal_handle_) {
        LOOP_RCLCPP_WARN(node_->get_logger(), "[LowBattery] NavigateToFirstPointAndStop halted, canceling goal.");
        action_client_->async_cancel_goal(goal_handle_);
        goal_handle_.reset();
    }
}

}  // namespace loop_navigation_bt
