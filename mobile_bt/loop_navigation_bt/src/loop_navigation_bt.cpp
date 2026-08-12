#include "loop_navigation_bt/loop_navigation_bt.hpp"
#include "loop_navigation_bt/logging.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <unordered_map>

#include "nav_interfaces/msg/map_state.hpp"
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

// 对样本排序后从两端剔除相同数量的极值，再对保留样本求平均。
double trimmedMean(std::vector<double> values, std::size_t trim_count_per_side) {
    if (values.empty()) {
        return 0.0;
    }

    std::sort(values.begin(), values.end());
    const std::size_t safe_trim_count =
        std::min(trim_count_per_side, (values.size() - 1) / 2);
    double sum = 0.0;
    for (std::size_t i = safe_trim_count; i < values.size() - safe_trim_count; ++i) {
        sum += values[i];
    }
    return sum / static_cast<double>(values.size() - 2 * safe_trim_count);
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

MapAssetsContext waitForCurrentMapAssets(const rclcpp::Node::SharedPtr& node, double timeout_sec) {
    nav_interfaces::msg::MapState::SharedPtr current_map_state;
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    auto subscription = node->create_subscription<nav_interfaces::msg::MapState>(
        "/map_manager/map_state", qos,
        [&current_map_state](const nav_interfaces::msg::MapState::SharedPtr msg) { current_map_state = msg; });

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
    while (rclcpp::ok() && !current_map_state && std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::milliseconds(50));
    }

    if (!current_map_state) {
        LOOP_RCLCPP_ERROR(node->get_logger(),
                          "等待 /map_manager/map_state 超时（%.1f 秒），无法确定当前地图。", timeout_sec);
        return {};
    }
    if (current_map_state->epoch == 0 || current_map_state->map_folder.empty() ||
        current_map_state->map_name.empty()) {
        LOOP_RCLCPP_ERROR(node->get_logger(),
                          "当前地图状态无效：epoch=%lu，map_folder='%s'，map_name='%s'。",
                          static_cast<unsigned long>(current_map_state->epoch),
                          current_map_state->map_folder.c_str(), current_map_state->map_name.c_str());
        return {};
    }

    const std::filesystem::path assets_file = std::filesystem::path(current_map_state->map_folder) /
                                              current_map_state->map_name / "assets" / "map_assets.json";
    std::error_code error;
    if (!std::filesystem::is_regular_file(assets_file, error)) {
        LOOP_RCLCPP_ERROR(node->get_logger(), "当前地图的资产文件不存在或不可读取：%s。",
                          assets_file.string().c_str());
        return {};
    }

    LOOP_RCLCPP_INFO(node->get_logger(), "当前地图：%s（epoch=%lu），导航点坐标读取自：%s。",
                     current_map_state->map_name.c_str(),
                     static_cast<unsigned long>(current_map_state->epoch), assets_file.string().c_str());
    return {assets_file.lexically_normal().string(), current_map_state->map_name, current_map_state->epoch};
}

void writeStartupFailureLog(const std::string& reason) {
    std::ofstream log_file("nav_error_log.txt", std::ios::out | std::ios::trunc);
    if (!log_file.is_open()) {
        return;
    }
    log_file << "循环导航测试记录\n";
    log_file << "开始时间：" << currentUtc8Time() << "\n\n";
    log_file << "测试状态：启动失败\n";
    log_file << "原因：" << reason << "\n";
}

namespace {

struct AssetPointPose {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

enum class TableAlignment {
    LEFT,
    CENTER,
    RIGHT
};

using TableRow = std::vector<std::string>;

// 日志使用 UTF-8；终端中中文通常占两列，不能直接用 std::setw() 的字节长度对齐。
std::size_t terminalDisplayWidth(const std::string& text) {
    std::size_t width = 0;
    for (std::size_t i = 0; i < text.size();) {
        const auto byte = static_cast<unsigned char>(text[i]);
        if (byte < 0x80) {
            ++width;
            ++i;
            continue;
        }

        std::size_t code_point_bytes = 1;
        if ((byte & 0xE0) == 0xC0) {
            code_point_bytes = 2;
        } else if ((byte & 0xF0) == 0xE0) {
            code_point_bytes = 3;
        } else if ((byte & 0xF8) == 0xF0) {
            code_point_bytes = 4;
        }
        i += std::min(code_point_bytes, text.size() - i);
        width += 2;
    }
    return width;
}

std::string alignTableCell(const std::string& text, std::size_t width, TableAlignment alignment) {
    const std::size_t text_width = terminalDisplayWidth(text);
    if (text_width >= width) {
        return text;
    }

    const std::size_t padding = width - text_width;
    std::size_t left_padding = 0;
    std::size_t right_padding = 0;
    switch (alignment) {
        case TableAlignment::RIGHT:
            left_padding = padding;
            break;
        case TableAlignment::CENTER:
            left_padding = padding / 2;
            right_padding = padding - left_padding;
            break;
        case TableAlignment::LEFT:
        default:
            right_padding = padding;
            break;
    }
    return std::string(left_padding, ' ') + text + std::string(right_padding, ' ');
}

std::vector<std::size_t> calculateTableWidths(const TableRow& header,
                                              const std::vector<TableRow>& rows = {}) {
    std::vector<std::size_t> widths;
    widths.reserve(header.size());
    for (const auto& cell : header) {
        widths.push_back(terminalDisplayWidth(cell));
    }
    for (const auto& row : rows) {
        const std::size_t count = std::min(row.size(), widths.size());
        for (std::size_t i = 0; i < count; ++i) {
            widths[i] = std::max(widths[i], terminalDisplayWidth(row[i]));
        }
    }
    return widths;
}

void writeTableRow(std::ostream& output,
                   const TableRow& row,
                   const std::vector<std::size_t>& widths,
                   const std::vector<TableAlignment>& alignments) {
    const std::size_t count = std::min(row.size(), widths.size());
    for (std::size_t i = 0; i < count; ++i) {
        if (i > 0) {
            output << " | ";
        }
        const TableAlignment alignment =
            i < alignments.size() ? alignments[i] : TableAlignment::LEFT;
        output << alignTableCell(row[i], widths[i], alignment);
    }
    output << "\n";
}

void writeTableSeparator(std::ostream& output, const std::vector<std::size_t>& widths) {
    const std::size_t separator_width =
        std::accumulate(widths.begin(), widths.end(), std::size_t{0}) +
        (widths.empty() ? 0 : (widths.size() - 1) * 3);
    output << std::string(separator_width, '-') << "\n";
}

void writeAlignedTable(std::ostream& output,
                       const TableRow& header,
                       const std::vector<TableRow>& rows,
                       const std::vector<TableAlignment>& row_alignments) {
    const auto widths = calculateTableWidths(header, rows);
    writeTableRow(output, header, widths,
                  std::vector<TableAlignment>(header.size(), TableAlignment::CENTER));
    writeTableSeparator(output, widths);
    for (const auto& row : rows) {
        writeTableRow(output, row, widths, row_alignments);
    }
}

std::string formatFixed(double value, int precision) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
}

const TableRow& navigationTableHeader() {
    static const TableRow header = {
        "轮次", "点位", "结果", "导航耗时(秒)", "横向误差(厘米)",
        "纵向误差(厘米)", "角度误差(度)", "本次行驶(米)", "累计行驶(米)"};
    return header;
}

std::vector<std::size_t> navigationTableWidths(int total_rounds,
                                               int completed_rounds,
                                               const std::vector<std::string>& goal_ids) {
    auto widths = calculateTableWidths(navigationTableHeader());
    const int maximum_round = std::max({1, completed_rounds, total_rounds});
    const std::string widest_round =
        total_rounds > 0 ? std::to_string(maximum_round) + "/" + std::to_string(total_rounds)
                         : std::to_string(maximum_round);
    widths[0] = std::max({widths[0], terminalDisplayWidth(widest_round), std::size_t{9}});
    widths[1] = std::max(widths[1], std::size_t{12});
    for (const auto& goal_id : goal_ids) {
        widths[1] = std::max(widths[1], terminalDisplayWidth(goal_id));
    }
    return widths;
}

bool readJsonFile(const std::string& path, const char* label, rclcpp::Logger logger, nlohmann::json& json) {
    std::ifstream file(path);
    if (!file.is_open()) {
        LOOP_RCLCPP_ERROR(logger, "无法打开%s：%s。", label, path.c_str());
        return false;
    }
    try {
        file >> json;
    } catch (const nlohmann::json::exception& error) {
        LOOP_RCLCPP_ERROR(logger, "%s不是有效 JSON：%s（%s）。", label, path.c_str(), error.what());
        return false;
    }
    return true;
}

}  // namespace

// 导航计划只决定轮数、点位顺序和停留时间；所有目标坐标均按名称从地图资产解析。
std::vector<geometry_msgs::msg::PoseStamped> parseNavPoints(const std::string& plan_path,
                                                            const std::string& assets_path,
                                                            rclcpp::Logger logger,
                                                            std::vector<std::string>& goal_ids,
                                                            int& total_rounds,
                                                            std::vector<double>& goal_wait_times_sec) {
    std::vector<geometry_msgs::msg::PoseStamped> points;
    nlohmann::json plan;
    nlohmann::json assets;
    if (!readJsonFile(plan_path, "导航计划文件", logger, plan) ||
        !readJsonFile(assets_path, "地图资产文件", logger, assets)) {
        return points;
    }

    if (!plan.is_object() || !plan.contains("rounds") || !plan["rounds"].is_number_integer() ||
        !plan.contains("points") || !plan["points"].is_array() || plan["points"].empty()) {
        LOOP_RCLCPP_ERROR(logger, "导航计划必须包含非负整数 rounds 和非空 points 数组：%s。",
                          plan_path.c_str());
        return points;
    }
    const auto rounds = plan["rounds"].get<std::int64_t>();
    if (rounds < 0 || rounds > std::numeric_limits<int>::max()) {
        LOOP_RCLCPP_ERROR(logger, "导航计划 rounds 超出有效范围：%ld。", static_cast<long>(rounds));
        return points;
    }
    total_rounds = static_cast<int>(rounds);

    if (!assets.is_object() || !assets.contains("points") || !assets["points"].is_array()) {
        LOOP_RCLCPP_ERROR(logger, "地图资产缺少 points 数组：%s。", assets_path.c_str());
        return points;
    }

    std::unordered_map<std::string, AssetPointPose> asset_points;
    for (const auto& item : assets["points"]) {
        if (!item.is_object() || !item.contains("name") || !item["name"].is_string() ||
            !item.contains("pose") || !item["pose"].is_object()) {
            LOOP_RCLCPP_ERROR(logger, "地图资产中存在缺少 name 或 pose 的点位：%s。", assets_path.c_str());
            return points;
        }
        const std::string name = item["name"].get<std::string>();
        const auto& pose = item["pose"];
        if (name.empty() || !pose.contains("x") || !pose["x"].is_number() || !pose.contains("y") ||
            !pose["y"].is_number() || !pose.contains("yaw") || !pose["yaw"].is_number()) {
            LOOP_RCLCPP_ERROR(logger, "地图资产点位 '%s' 的 x、y 或 yaw 无效。", name.c_str());
            return points;
        }
        const AssetPointPose asset_pose{pose["x"].get<double>(), pose["y"].get<double>(),
                                        pose["yaw"].get<double>()};
        if (!std::isfinite(asset_pose.x) || !std::isfinite(asset_pose.y) || !std::isfinite(asset_pose.yaw)) {
            LOOP_RCLCPP_ERROR(logger, "地图资产点位 '%s' 的坐标必须是有限数值。", name.c_str());
            return points;
        }
        if (!asset_points.emplace(name, asset_pose).second) {
            LOOP_RCLCPP_ERROR(logger, "地图资产中存在重名点位：'%s'。", name.c_str());
            return points;
        }
    }

    goal_ids.clear();
    goal_wait_times_sec.clear();
    for (const auto& item : plan["points"]) {
        if (!item.is_object() || !item.contains("id") || !item["id"].is_string()) {
            LOOP_RCLCPP_ERROR(logger, "导航计划的每个点都必须设置字符串 id。");
            return {};
        }
        if (item.contains("position") || item.contains("orientation") || item.contains("pose")) {
            LOOP_RCLCPP_ERROR(logger,
                              "导航计划点位 '%s' 仍包含坐标字段；坐标只能保存在 map_assets.json。",
                              item["id"].get<std::string>().c_str());
            return {};
        }
        const std::string goal_id = item["id"].get<std::string>();
        const auto asset_point = asset_points.find(goal_id);
        if (goal_id.empty() || asset_point == asset_points.end()) {
            LOOP_RCLCPP_ERROR(logger, "导航计划点位 '%s' 在当前地图资产中不存在：%s。", goal_id.c_str(),
                              assets_path.c_str());
            return {};
        }
        if (!item.contains("wait_time_sec") || !item["wait_time_sec"].is_number()) {
            LOOP_RCLCPP_ERROR(logger, "导航计划点位 '%s' 必须设置 wait_time_sec。", goal_id.c_str());
            return {};
        }
        const double wait_time_sec = item["wait_time_sec"].get<double>();
        if (!std::isfinite(wait_time_sec) || wait_time_sec < 0.0) {
            LOOP_RCLCPP_ERROR(logger, "导航计划点位 '%s' 的 wait_time_sec 必须是非负有限数值。",
                              goal_id.c_str());
            return {};
        }

        const auto& asset_pose = asset_point->second;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = asset_pose.x;
        pose.pose.position.y = asset_pose.y;
        pose.pose.orientation.z = std::sin(asset_pose.yaw * 0.5);
        pose.pose.orientation.w = std::cos(asset_pose.yaw * 0.5);
        goal_ids.push_back(goal_id);
        goal_wait_times_sec.push_back(wait_time_sec);
        points.push_back(pose);
    }
    LOOP_RCLCPP_INFO(logger, "导航计划加载成功：%zu 个点位，轮数=%d；坐标全部来自当前地图资产。",
                     points.size(), total_rounds);
    return points;
}

//--- Implementation for LoopNavigateFourPoints ---
LoopNavigateFourPoints::LoopNavigateFourPoints(const std::string& name,
                                               const BT::NodeConfiguration& config,
                                               rclcpp::Node::SharedPtr node_ptr)
    : BtActionNode(name, config, node_ptr) {
    action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");

    try {
        expected_map_epoch_ = config.blackboard->get<std::uint64_t>("map_epoch");
        expected_map_assets_file_ =
            std::filesystem::path(config.blackboard->get<std::string>("map_assets_file"))
                .lexically_normal()
                .string();
    } catch (const std::exception& error) {
        map_changed_ = true;
        setTestEndState(TestEndState::FAILED, "程序内部缺少当前地图信息");
        LOOP_RCLCPP_ERROR(node_->get_logger(), "读取行为树地图上下文失败：%s。", error.what());
    }

    map_state_sub_ = node_->create_subscription<nav_interfaces::msg::MapState>(
        "/map_manager/map_state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        [this](const nav_interfaces::msg::MapState::ConstSharedPtr msg) {
            handleMapState(msg->epoch, msg->map_folder, msg->map_name);
        });

    // TF 用于在导航结束后读取机器人实际位姿并计算到点误差。
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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
        node_->declare_parameter("goal_error_stats.desired_position_tolerance", high_precision_tolerance_m_);
    }
    if (!node_->has_parameter("goal_error_stats.accept_position_tolerance")) {
        node_->declare_parameter("goal_error_stats.accept_position_tolerance", acceptance_tolerance_m_);
    }
    if (!node_->has_parameter("goal_error_stats.sample_count")) {
        node_->declare_parameter("goal_error_stats.sample_count", static_cast<int>(pose_sample_count_));
    }
    if (!node_->has_parameter("goal_error_stats.trim_count_per_side")) {
        node_->declare_parameter("goal_error_stats.trim_count_per_side",
                                 static_cast<int>(pose_sample_trim_count_));
    }
    if (!node_->has_parameter("goal_error_stats.sample_interval_sec")) {
        node_->declare_parameter("goal_error_stats.sample_interval_sec", pose_sample_interval_sec_);
    }
    if (!node_->has_parameter("goal_error_stats.use_goal_checker_error")) {
        node_->declare_parameter("goal_error_stats.use_goal_checker_error", use_goal_checker_error_);
    }
    if (!node_->has_parameter("goal_error_stats.summary_interval_rounds")) {
        node_->declare_parameter("goal_error_stats.summary_interval_rounds", stats_interval_rounds_);
    }
    node_->get_parameter("navigation.action_server_wait_timeout_sec", action_server_wait_timeout_sec_);
    node_->get_parameter("navigation.odom_topic", odom_topic_);
    node_->get_parameter("navigation.odom_max_increment_m", odom_max_increment_m_);
    node_->get_parameter("goal_error_stats.desired_position_tolerance", high_precision_tolerance_m_);
    node_->get_parameter("goal_error_stats.accept_position_tolerance", acceptance_tolerance_m_);
    int pose_sample_count = static_cast<int>(pose_sample_count_);
    int pose_sample_trim_count = static_cast<int>(pose_sample_trim_count_);
    node_->get_parameter("goal_error_stats.sample_count", pose_sample_count);
    node_->get_parameter("goal_error_stats.trim_count_per_side", pose_sample_trim_count);
    node_->get_parameter("goal_error_stats.sample_interval_sec", pose_sample_interval_sec_);
    node_->get_parameter("goal_error_stats.use_goal_checker_error", use_goal_checker_error_);
    node_->get_parameter("goal_error_stats.summary_interval_rounds", stats_interval_rounds_);
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
    if (pose_sample_trim_count >= 0 &&
        static_cast<std::size_t>(pose_sample_trim_count) * 2 < pose_sample_count_) {
        pose_sample_trim_count_ = static_cast<std::size_t>(pose_sample_trim_count);
    } else {
        pose_sample_trim_count_ = std::min<std::size_t>(2, (pose_sample_count_ - 1) / 2);
        LOOP_RCLCPP_WARN(node_->get_logger(),
                    "goal_error_stats.trim_count_per_side 非法，重置为每端 %zu 个。",
                    pose_sample_trim_count_);
    }
    if (pose_sample_interval_sec_ < 0.0) {
        LOOP_RCLCPP_WARN(node_->get_logger(), "goal_error_stats.sample_interval_sec 不能为负数，重置为 0.1。");
        pose_sample_interval_sec_ = 0.1;
    }
    if (stats_interval_rounds_ <= 0) {
        LOOP_RCLCPP_WARN(node_->get_logger(),
                    "goal_error_stats.summary_interval_rounds 必须为正整数，重置为 5 轮。");
        stats_interval_rounds_ = 5;
    }
    if (!std::isfinite(high_precision_tolerance_m_) || high_precision_tolerance_m_ <= 0.0) {
        LOOP_RCLCPP_WARN(node_->get_logger(),
                    "goal_error_stats.desired_position_tolerance 必须是正数，重置为 0.01 米。");
        high_precision_tolerance_m_ = 0.01;
    }
    if (!std::isfinite(acceptance_tolerance_m_) || acceptance_tolerance_m_ < high_precision_tolerance_m_) {
        LOOP_RCLCPP_WARN(node_->get_logger(),
                    "goal_error_stats.accept_position_tolerance 不能小于高精度阈值，重置为 %.3f 米。",
                    std::max(0.02, high_precision_tolerance_m_));
        acceptance_tolerance_m_ = std::max(0.02, high_precision_tolerance_m_);
    }
    LOOP_RCLCPP_INFO(node_->get_logger(), "循环导航参数：Action server 超时 %.2f s。",
                     action_server_wait_timeout_sec_);
    LOOP_RCLCPP_INFO(node_->get_logger(), "导航里程统计：订阅 %s，单次里程计增量上限 %.3f m。",
                odom_topic_.c_str(), odom_max_increment_m_);
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LoopNavigateFourPoints::odometryCallback, this, std::placeholders::_1));
    LOOP_RCLCPP_INFO(node_->get_logger(), "goal_checker 最终误差读取开关: %s。%s",
                use_goal_checker_error_ ? "true" : "false",
                use_goal_checker_error_ ? "优先读取 /navigation_goal_error，读取不到则回退 TF 采样。"
                                        : "禁用 /navigation_goal_error，直接使用 TF 窗口采样。");
    LOOP_RCLCPP_INFO(node_->get_logger(),
                "误差累计统计：每完成 %d 个完整轮次，输出一次截至当前轮次的累计结果。",
                stats_interval_rounds_);
    LOOP_RCLCPP_INFO(node_->get_logger(),
                "精度判定：横向和纵向误差均不超过 %.2f cm 为高精度，均不超过 %.2f cm 为接收精度。",
                high_precision_tolerance_m_ * 100.0, acceptance_tolerance_m_ * 100.0);
    LOOP_RCLCPP_INFO(node_->get_logger(),
                "TF 回退误差采样：尝试 %zu 次，每端去除 %zu 个，完整采样时保留 %zu 个求平均。",
                pose_sample_count_, pose_sample_trim_count_,
                pose_sample_count_ - 2 * pose_sample_trim_count_);
    if (use_goal_checker_error_) {
        goal_error_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            "/navigation_goal_error", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
            [this](const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(latest_goal_error_mutex_);
                latest_goal_error_ = *msg;
                has_latest_goal_error_ = true;
            });
    }

    // 每次启动覆盖旧日志；先写测试标题，后续即使配置错误也不会留下空文件。
    log_file_.open("nav_error_log.txt", std::ios::out | std::ios::trunc);
    if (log_file_.is_open()) {
        log_file_ << "循环导航测试记录\n";
        log_file_ << "开始时间：" << currentUtc8Time() << "\n";
        log_file_.flush();
    } else {
        LOOP_RCLCPP_ERROR(node_->get_logger(), "无法创建 nav_error_log.txt，请检查进程工作目录写权限。");
    }
}

LoopNavigateFourPoints::~LoopNavigateFourPoints() {
    if (log_file_.is_open()) {
        if (test_end_state_ == TestEndState::RUNNING) {
            std::string failure_reason;
            std::string stop_reason;
            try {
                failure_reason = config().blackboard->get<std::string>("test_failure_reason");
            } catch (...) {
            }
            try {
                stop_reason = config().blackboard->get<std::string>("test_stop_reason");
            } catch (...) {
            }

            if (!failure_reason.empty()) {
                setTestEndState(TestEndState::FAILED, failure_reason);
            } else if (!stop_reason.empty()) {
                setTestEndState(TestEndState::STOPPED, stop_reason);
            } else {
                setTestEndState(TestEndState::STOPPED, "程序在完成计划前结束");
            }
        }
        calculateAndLogStats();
        log_file_.close();
    }
}

void LoopNavigateFourPoints::handleMapState(std::uint64_t epoch,
                                            const std::string& map_folder,
                                            const std::string& map_name) {
    // 测试已经结束后不再改变最终状态，避免退出阶段收到晚到的 MapState 后误报。
    if (test_end_state_ != TestEndState::RUNNING) {
        return;
    }

    const std::string current_assets_file =
        (std::filesystem::path(map_folder) / map_name / "assets" / "map_assets.json")
            .lexically_normal()
            .string();
    if (epoch == expected_map_epoch_ && current_assets_file == expected_map_assets_file_) {
        return;
    }
    if (map_changed_.exchange(true)) {
        return;
    }

    std::ostringstream reason;
    reason << "检测到地图已切换：启动时 epoch=" << expected_map_epoch_ << "，当前 epoch=" << epoch
           << "；为防止使用旧地图坐标，测试已停止";
    setTestEndState(TestEndState::STOPPED, reason.str());
    try {
        config().blackboard->set("map_changed", true);
        config().blackboard->set("test_stop_reason", reason.str());
    } catch (...) {
    }
    LOOP_RCLCPP_ERROR(node_->get_logger(), "%s。", reason.str().c_str());
}

void LoopNavigateFourPoints::setTestEndState(TestEndState state, const std::string& reason) {
    if (test_end_state_ != TestEndState::RUNNING) {
        return;
    }
    test_end_state_ = state;
    test_end_reason_ = reason;
}

const char* LoopNavigateFourPoints::testEndStateText() const {
    switch (test_end_state_) {
        case TestEndState::COMPLETED:
            return "正常完成";
        case TestEndState::STOPPED:
            return "提前停止";
        case TestEndState::FAILED:
            return "测试失败";
        case TestEndState::RUNNING:
        default:
            return "运行中";
    }
}

BT::PortsList LoopNavigateFourPoints::providedPorts() {
    return {BT::InputPort<std::string>("points", "只包含轮数、点位顺序和停留时间的导航计划"),
            BT::InputPort<std::string>("assets", "当前地图的 map_assets.json")};
}

void LoopNavigateFourPoints::writeNavigationTableHeader() {
    if (!log_file_.is_open()) {
        return;
    }

    const auto widths = navigationTableWidths(total_rounds_, completed_rounds_, goal_ids_);
    writeTableRow(log_file_, navigationTableHeader(), widths,
                  std::vector<TableAlignment>(navigationTableHeader().size(), TableAlignment::CENTER));
    writeTableSeparator(log_file_, widths);
}

void LoopNavigateFourPoints::writeTestLogHeader() {
    if (!log_file_.is_open() || log_header_written_) {
        return;
    }

    log_file_ << "\n";
    log_file_ << "计划轮数：" << (total_rounds_ > 0 ? std::to_string(total_rounds_) : "持续运行") << "\n";
    log_file_ << "点位顺序：";
    for (std::size_t i = 0; i < goal_ids_.size(); ++i) {
        if (i > 0) {
            log_file_ << " -> ";
        }
        log_file_ << goal_ids_[i] << "（停留 " << std::fixed << std::setprecision(1)
                  << goal_wait_times_sec_[i] << " 秒）";
    }
    log_file_ << "\n";
    log_file_ << "累计统计：每 " << stats_interval_rounds_
              << " 个完整轮次输出一次截至当前轮次的结果\n";
    log_file_ << "精度标准：高精度 " << std::fixed << std::setprecision(2)
              << high_precision_tolerance_m_ * 100.0 << " 厘米，接收精度 "
              << acceptance_tolerance_m_ * 100.0 << " 厘米（横向和纵向均需满足）\n\n";
    writeNavigationTableHeader();
    log_file_.flush();
    log_header_written_ = true;
}

BT::NodeStatus LoopNavigateFourPoints::onStart() {
    if (map_changed_) {
        LOOP_RCLCPP_ERROR(node_->get_logger(), "当前地图与启动地图不一致，停止循环导航。");
        return BT::NodeStatus::FAILURE;
    }

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
        const auto plan_path = getInput<std::string>("points");
        const auto assets_path = getInput<std::string>("assets");
        if (!plan_path || !assets_path) {
            LOOP_RCLCPP_ERROR(node_->get_logger(), "行为树缺少 points 或 assets 输入端口。");
            setTestEndState(TestEndState::FAILED, "程序内部缺少导航计划或地图资产路径");
            return BT::NodeStatus::FAILURE;
        }
        goals_ = parseNavPoints(plan_path.value(), assets_path.value(), node_->get_logger(), goal_ids_,
                                total_rounds_, goal_wait_times_sec_);
        if (goals_.empty()) {
            setTestEndState(TestEndState::FAILED, "导航计划或当前地图点位配置错误，请联系技术人员");
            return BT::NodeStatus::FAILURE;
        }
        cumulative_per_goal_stats_.resize(goals_.size());
        writeTestLogHeader();
    }

    if (!action_client_->wait_for_action_server(std::chrono::duration<double>(action_server_wait_timeout_sec_))) {
        LOOP_RCLCPP_ERROR(node_->get_logger(), "Action server /navigate_to_pose not available.");
        setTestEndState(TestEndState::FAILED, "导航服务未启动，测试无法开始");
        return BT::NodeStatus::FAILURE;
    }

    // Action server 等待期间地图可能发生变化，发送目标前再处理一次 MapState。
    rclcpp::spin_some(node_);
    if (map_changed_) {
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
    if (map_changed_) {
        if (goal_handle_) {
            action_client_->async_cancel_goal(goal_handle_);
        }
        stopDistanceTracking();
        goal_handle_.reset();
        result_ready_ = false;
        return BT::NodeStatus::FAILURE;
    }

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
                failed_nav_++;
                const double elapsed_sec =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - goal_start_time_).count();
                logNavigationError(NavigationErrorSample{}, distance_snapshot, elapsed_sec, false,
                                   current_goal_index_);
                setTestEndState(TestEndState::FAILED, "导航目标被导航服务拒绝");
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
        const double navigation_elapsed_sec =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - goal_start_time_).count();
        succeeded_nav_++;
        const auto distance_snapshot = stopDistanceTracking();

        // 到点后按当前点位配置的时间停留，再采集稳定后的最终误差。
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
        if (map_changed_) {
            logNavigationError(NavigationErrorSample{}, distance_snapshot, navigation_elapsed_sec, true,
                               current_goal_index_);
            goal_handle_.reset();
            result_ready_ = false;
            return BT::NodeStatus::FAILURE;
        }

        // Log error
        const auto& goal_pose_msg = goals_[current_goal_index_].pose;
        tf2::Quaternion q_goal;
        tf2::fromMsg(goal_pose_msg.orientation, q_goal);
        double roll, pitch, yaw_goal;
        tf2::Matrix3x3(q_goal).getRPY(roll, pitch, yaw_goal);
        Pose goal_pose = {goal_pose_msg.position.x, goal_pose_msg.position.y, yaw_goal};

        const auto error_sample = getNavigationErrorSample(goal_pose);
        if (map_changed_) {
            logNavigationError(NavigationErrorSample{}, distance_snapshot, navigation_elapsed_sec, true,
                               current_goal_index_);
            goal_handle_.reset();
            result_ready_ = false;
            return BT::NodeStatus::FAILURE;
        }
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
                                "到点误差：纵向=%.4f m，横向=%.4f m，角度=%.4f rad。",
                                error_sample.lon_error, error_sample.lat_error, error_sample.yaw_error);
        } else {
            LOOP_RCLCPP_WARN_AT(node_->get_logger(), arrival_time_text,
                                "未获取到有效最终位姿，无法计算有效误差。");
        }
        const bool completes_round = ((current_goal_index_ + 1) % goals_.size()) == 0;
        if (completes_round) {
            // 先更新轮数，避免本次误差统计写出旧轮数
            completed_rounds_++;
        }
        logNavigationError(error_sample, distance_snapshot, navigation_elapsed_sec, true, current_goal_index_);

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
                setTestEndState(TestEndState::COMPLETED, "已完成计划轮数");
                try {
                    config().blackboard->set("nav_loop_complete", true);
                } catch (...) {
                }
            }
            writePeriodicSummaryIfNeeded();
            if (navigation_complete_) {
                calculateAndLogStats();
            }
        }
        goal_handle_.reset();
        result_ready_ = false;
        return BT::NodeStatus::SUCCESS;
    } else if (result_code_ == rclcpp_action::ResultCode::ABORTED) {
        // If navigation to a point fails, continue with the next point
        LOOP_RCLCPP_WARN(node_->get_logger(), "Goal #%d aborted (failed). Continue to next goal.", current_goal_index_ + 1);
        const double navigation_elapsed_sec =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - goal_start_time_).count();
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
        if (map_changed_) {
            logNavigationError(NavigationErrorSample{}, distance_snapshot, navigation_elapsed_sec, false,
                               current_goal_index_);
            goal_handle_.reset();
            result_ready_ = false;
            return BT::NodeStatus::FAILURE;
        }

        const auto& goal_pose_msg = goals_[current_goal_index_].pose;
        tf2::Quaternion q_goal;
        tf2::fromMsg(goal_pose_msg.orientation, q_goal);
        double roll, pitch, yaw_goal;
        tf2::Matrix3x3(q_goal).getRPY(roll, pitch, yaw_goal);
        Pose goal_pose = {goal_pose_msg.position.x, goal_pose_msg.position.y, yaw_goal};

        const auto error_sample = getNavigationErrorSample(goal_pose);
        if (map_changed_) {
            logNavigationError(NavigationErrorSample{}, distance_snapshot, navigation_elapsed_sec, false,
                               current_goal_index_);
            goal_handle_.reset();
            result_ready_ = false;
            return BT::NodeStatus::FAILURE;
        }
        const bool completes_round = ((current_goal_index_ + 1) % goals_.size()) == 0;
        if (completes_round) {
            // 先更新轮数，避免本次误差统计写出旧轮数
            completed_rounds_++;
        }
        logNavigationError(error_sample, distance_snapshot, navigation_elapsed_sec, false, current_goal_index_);

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
                setTestEndState(TestEndState::COMPLETED, "已完成计划轮数");
                try {
                    config().blackboard->set("nav_loop_complete", true);
                } catch (...) {
                }
            }
            writePeriodicSummaryIfNeeded();
            if (navigation_complete_) {
                calculateAndLogStats();
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
        failed_nav_++;
        const double elapsed_sec =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - goal_start_time_).count();
        logNavigationError(NavigationErrorSample{}, distance_snapshot, elapsed_sec, false, current_goal_index_);
        setTestEndState(TestEndState::FAILED, "导航目标被意外取消");
        goal_handle_.reset();
        result_ready_ = false;
        return BT::NodeStatus::FAILURE;
    }

    const auto distance_snapshot = stopDistanceTracking();
    failed_nav_++;
    const double elapsed_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - goal_start_time_).count();
    logNavigationError(NavigationErrorSample{}, distance_snapshot, elapsed_sec, false, current_goal_index_);
    setTestEndState(TestEndState::FAILED, "导航返回了未知结果");
    LOOP_RCLCPP_ERROR(node_->get_logger(), "导航返回未知结果，测试停止。");
    goal_handle_.reset();
    result_ready_ = false;
    return BT::NodeStatus::FAILURE;
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
    while (std::chrono::steady_clock::now() < end_time && rclcpp::ok() && !map_changed_) {
        rclcpp::spin_some(node_);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}

LoopNavigateFourPoints::NavigationErrorSample LoopNavigateFourPoints::getNavigationErrorSample(const Pose& goal) {
    NavigationErrorSample sample;
    if (map_changed_) {
        return sample;
    }
    rclcpp::spin_some(node_);
    if (map_changed_) {
        return sample;
    }
    if (use_goal_checker_error_ && getLatestGoalCheckerErrorSample(goal, sample)) {
        LOOP_RCLCPP_INFO(node_->get_logger(),
                    "使用 goal_checker 发布的最终误差写入导航统计：lon=%.4f, lat=%.4f, yaw=%.4f",
                    sample.lon_error, sample.lat_error, sample.yaw_error);
        return sample;
    }

    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> yaws;
    std::vector<double> lon_errors;
    std::vector<double> lat_errors;
    std::vector<double> yaw_errors;
    xs.reserve(pose_sample_count_);
    ys.reserve(pose_sample_count_);
    yaws.reserve(pose_sample_count_);
    lon_errors.reserve(pose_sample_count_);
    lat_errors.reserve(pose_sample_count_);
    yaw_errors.reserve(pose_sample_count_);

    const double cos_yaw = std::cos(goal.yaw);
    const double sin_yaw = std::sin(goal.yaw);

    for (std::size_t i = 0; i < pose_sample_count_ && !map_changed_; ++i) {
        try {
            rclcpp::spin_some(node_);
            if (map_changed_) {
                return {};
            }
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
        } catch (const tf2::TransformException& ex) {
            LOOP_RCLCPP_WARN(node_->get_logger(), "误差统计获取机器人 TF 失败: %s，继续重试。", ex.what());
            spinNodeForDuration(0.5);
            continue;
        }

        if (i + 1 < pose_sample_count_) {
            spinNodeForDuration(pose_sample_interval_sec_);
        }
    }

    if (map_changed_ || xs.empty()) {
        return sample;
    }

    const std::size_t effective_trim_count =
        std::min(pose_sample_trim_count_, (xs.size() - 1) / 2);
    const std::size_t retained_sample_count = xs.size() - 2 * effective_trim_count;
    if (xs.size() < pose_sample_count_) {
        LOOP_RCLCPP_WARN(node_->get_logger(),
                    "TF 误差采样仅成功 %zu/%zu 次，本次每端去除 %zu 个后使用 %zu 个样本。",
                    xs.size(), pose_sample_count_, effective_trim_count, retained_sample_count);
    }
    if (effective_trim_count < pose_sample_trim_count_) {
        LOOP_RCLCPP_WARN(node_->get_logger(),
                    "有效样本不足以按配置每端去除 %zu 个，本次自动改为每端去除 %zu 个。",
                    pose_sample_trim_count_, effective_trim_count);
    }

    sample.current.x = trimmedMean(xs, effective_trim_count);
    sample.current.y = trimmedMean(ys, effective_trim_count);
    sample.lon_error = trimmedMean(lon_errors, effective_trim_count);
    sample.lat_error = trimmedMean(lat_errors, effective_trim_count);
    sample.yaw_error = trimmedMean(yaw_errors, effective_trim_count);
    sample.current.yaw = normalize_angle(goal.yaw + sample.yaw_error);
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
                "误差统计TF采样: x=[%s], y=[%s], yaw=[%s]; 每端去除 %zu 个后保留 %zu 个，"
                "平均位姿: x=%.4f, y=%.4f, yaw=%.4f",
                xs_stream.str().c_str(), ys_stream.str().c_str(), yaws_stream.str().c_str(),
                effective_trim_count, retained_sample_count, sample.current.x, sample.current.y,
                sample.current.yaw);
    LOOP_RCLCPP_INFO(node_->get_logger(),
                "误差统计窗口: lon=[%s], lat=[%s], yaw=[%s]; 去极值平均误差: "
                "lon=%.4f, lat=%.4f, yaw=%.4f",
                lon_stream.str().c_str(), lat_stream.str().c_str(), yaw_err_stream.str().c_str(), sample.lon_error,
                sample.lat_error, sample.yaw_error);
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
    if (std::hypot(goal_dx, goal_dy) > acceptance_tolerance_m_ || std::abs(goal_dyaw) > 0.05) {
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
        !get_double("yaw_error", sample.yaw_error)) {
        LOOP_RCLCPP_WARN(node_->get_logger(), "goal_checker 误差消息字段不完整，改用 TF 窗口统计。");
        return false;
    }

    sample.valid = true;
    LOOP_RCLCPP_INFO(node_->get_logger(),
                "采用 goal_checker 最终误差消息：result_type=%s, lon=%.4f, lat=%.4f, yaw=%.4f",
                result_type.c_str(), sample.lon_error, sample.lat_error, sample.yaw_error);
    return true;
}

void LoopNavigateFourPoints::logNavigationError(const NavigationErrorSample& error_sample,
                                                const DistanceSnapshot& distance_snapshot,
                                                double elapsed_sec,
                                                bool succeeded,
                                                std::size_t goal_index) {
    if (!node_) {
        return;
    }

    const double err_lon = error_sample.lon_error;
    const double err_lat = error_sample.lat_error;
    const double err_yaw = error_sample.yaw_error;
    if (!error_sample.valid) {
        LOOP_RCLCPP_WARN(node_->get_logger(), "未获取到有效到点误差，本次导航仍记录结果，但不纳入误差统计。");
    }
    if (!distance_snapshot.valid) {
        ++missing_distance_count_;
    }
    const bool within_high_precision = error_sample.valid &&
        isWithinAxisTolerance(err_lon, err_lat, high_precision_tolerance_m_);
    const bool within_acceptance = error_sample.valid &&
        isWithinAxisTolerance(err_lon, err_lat, acceptance_tolerance_m_);

    auto record_cumulative_stats = [&](CumulativeSummaryStats& stats) {
        auto& result_stats = succeeded ? stats.success : stats.failure;
        ++result_stats.count;
        result_stats.elapsed_times.push_back(elapsed_sec);
        if (!error_sample.valid) {
            return;
        }
        result_stats.lon_errors.push_back(err_lon);
        result_stats.lat_errors.push_back(err_lat);
        result_stats.yaw_errors.push_back(std::abs(err_yaw));
        if (within_high_precision) {
            ++result_stats.high_precision_count;
        }
        if (within_acceptance) {
            ++result_stats.acceptance_count;
        }
    };
    record_cumulative_stats(cumulative_overall_stats_);
    if (goal_index < cumulative_per_goal_stats_.size()) {
        record_cumulative_stats(cumulative_per_goal_stats_[goal_index]);
    }

    if (log_file_.is_open()) {
        const std::string goal_id =
            goal_index < goal_ids_.size() ? goal_ids_[goal_index] : "goal_" + std::to_string(goal_index);
        const int round_number =
            completed_rounds_ + ((goal_index + 1 < goals_.size()) ? 1 : 0);
        std::string round_text = std::to_string(round_number);
        if (total_rounds_ > 0) {
            round_text += "/" + std::to_string(total_rounds_);
        }

        std::ostringstream lateral_error;
        std::ostringstream longitudinal_error;
        std::ostringstream yaw_error;
        if (error_sample.valid) {
            lateral_error << std::fixed << std::setprecision(2) << std::abs(err_lat) * 100.0;
            longitudinal_error << std::fixed << std::setprecision(2) << std::abs(err_lon) * 100.0;
            yaw_error << std::fixed << std::setprecision(2) << std::abs(err_yaw) * 180.0 / M_PI;
        } else {
            lateral_error << "未获取";
            longitudinal_error << "未获取";
            yaw_error << "未获取";
        }

        std::ostringstream goal_distance;
        std::ostringstream cumulative_distance;
        if (distance_snapshot.valid) {
            goal_distance << std::fixed << std::setprecision(2) << distance_snapshot.goal_distance_m;
            cumulative_distance << std::fixed << std::setprecision(2)
                                << distance_snapshot.cumulative_distance_m;
        } else {
            goal_distance << "未获取";
            cumulative_distance << "未获取";
        }

        const TableRow row = {
            round_text,
            goal_id,
            succeeded ? "成功" : "失败",
            formatFixed(elapsed_sec, 2),
            lateral_error.str(),
            longitudinal_error.str(),
            yaw_error.str(),
            goal_distance.str(),
            cumulative_distance.str()};
        const std::vector<TableAlignment> alignments = {
            TableAlignment::RIGHT, TableAlignment::LEFT,  TableAlignment::CENTER,
            TableAlignment::RIGHT, TableAlignment::RIGHT, TableAlignment::RIGHT,
            TableAlignment::RIGHT, TableAlignment::RIGHT, TableAlignment::RIGHT};
        writeTableRow(log_file_, row,
                      navigationTableWidths(total_rounds_, completed_rounds_, goal_ids_), alignments);
        log_file_.flush();
    }
}

bool LoopNavigateFourPoints::isWithinAxisTolerance(double lon_error, double lat_error, double tolerance) const {
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

void LoopNavigateFourPoints::writeSeparatedSummaryTables() {
    if (!log_file_.is_open()) {
        return;
    }

    auto format_tolerance_cm = [](double tolerance_m) {
        std::ostringstream stream;
        stream << std::defaultfloat << std::setprecision(6) << tolerance_m * 100.0;
        return stream.str();
    };
    auto format_ratio = [](std::size_t count, std::size_t total) {
        if (total == 0) {
            return std::string("未获取");
        }
        std::ostringstream stream;
        stream << count << "/" << total << "（" << std::fixed << std::setprecision(1)
               << static_cast<double>(count) * 100.0 / static_cast<double>(total) << "%）";
        return stream.str();
    };
    auto make_summary_row = [&](const std::string& label, const CumulativeSummaryStats& stats,
                                bool write_success) {
        const auto& result_stats = write_success ? stats.success : stats.failure;
        const std::size_t total = static_cast<std::size_t>(stats.success.count + stats.failure.count);
        const std::size_t result_count = static_cast<std::size_t>(result_stats.count);
        const std::size_t valid_count = result_stats.lon_errors.size();

        TableRow row = {label, format_ratio(result_count, total), "未获取", "未获取",
                        "未获取", "未获取", "未获取", "未获取"};
        if (!result_stats.elapsed_times.empty()) {
            const double average_elapsed =
                std::accumulate(result_stats.elapsed_times.begin(), result_stats.elapsed_times.end(), 0.0) /
                static_cast<double>(result_stats.elapsed_times.size());
            row[2] = formatFixed(average_elapsed, 2);
        }

        if (valid_count > 0) {
            row[3] = formatFixed(calculateMetricStats(result_stats.lat_errors).mean * 100.0, 2);
            row[4] = formatFixed(calculateMetricStats(result_stats.lon_errors).mean * 100.0, 2);
            row[5] = formatFixed(
                calculateMetricStats(result_stats.yaw_errors).mean * 180.0 / M_PI, 2);
            row[6] = format_ratio(
                static_cast<std::size_t>(result_stats.high_precision_count), valid_count);
            row[7] = format_ratio(
                static_cast<std::size_t>(result_stats.acceptance_count), valid_count);
        }
        return row;
    };
    auto write_summary_table = [&](const std::string& title, bool write_success) {
        log_file_ << "\n" << title << "\n";
        const TableRow header = {
            "对象",
            write_success ? "成功次数/总次数（成功率）" : "失败次数/总次数（失败占比）",
            "平均耗时(秒)",
            "平均横向误差(厘米)",
            "平均纵向误差(厘米)",
            "平均角度误差(度)",
            "高精度（" + format_tolerance_cm(high_precision_tolerance_m_) + "cm）占比",
            "接收精度（" + format_tolerance_cm(acceptance_tolerance_m_) + "cm）占比"};
        std::vector<TableRow> rows;
        rows.push_back(make_summary_row("总体", cumulative_overall_stats_, write_success));
        for (std::size_t i = 0; i < cumulative_per_goal_stats_.size() && i < goal_ids_.size(); ++i) {
            rows.push_back(make_summary_row(goal_ids_[i], cumulative_per_goal_stats_[i], write_success));
        }
        const std::vector<TableAlignment> alignments = {
            TableAlignment::LEFT,  TableAlignment::RIGHT, TableAlignment::RIGHT, TableAlignment::RIGHT,
            TableAlignment::RIGHT, TableAlignment::RIGHT, TableAlignment::RIGHT, TableAlignment::RIGHT};
        writeAlignedTable(log_file_, header, rows, alignments);
    };

    write_summary_table("成功汇总", true);
    write_summary_table("失败汇总", false);
}

void LoopNavigateFourPoints::writePeriodicSummaryIfNeeded() {
    // 正常完成时由测试结束汇总输出分类表，避免同一份统计连续打印两次。
    if (!log_file_.is_open() || navigation_complete_ || completed_rounds_ <= 0 ||
        completed_rounds_ % stats_interval_rounds_ != 0) {
        return;
    }

    log_file_ << "\n第 1-" << completed_rounds_ << " 轮累计汇总\n";
    writeSeparatedSummaryTables();

    const bool has_next_round = total_rounds_ == 0 || completed_rounds_ < total_rounds_;
    if (has_next_round) {
        log_file_ << "\n";
        writeNavigationTableHeader();
    }
    log_file_.flush();
}

void LoopNavigateFourPoints::calculateAndLogStats() {
    const std::size_t total_nav = static_cast<std::size_t>(succeeded_nav_ + failed_nav_);
    if (summary_written_ || !log_file_.is_open()) {
        return;
    }

    const double success_rate =
        total_nav > 0 ? static_cast<double>(succeeded_nav_) * 100.0 / static_cast<double>(total_nav) : 0.0;

    double cumulative_distance_m = 0.0;
    {
        std::lock_guard<std::mutex> lock(distance_mutex_);
        cumulative_distance_m = cumulative_navigation_distance_m_;
    }

    log_file_ << "\n================ 测试汇总 ================\n";
    log_file_ << "测试状态：" << testEndStateText() << "\n";
    if (!test_end_reason_.empty()) {
        log_file_ << "原因：" << test_end_reason_ << "\n";
    }
    log_file_ << "完成轮数：" << completed_rounds_;
    if (total_rounds_ > 0) {
        log_file_ << "/" << total_rounds_;
    }
    log_file_ << "\n";
    log_file_ << "导航次数：" << total_nav << "（成功 " << succeeded_nav_ << "，失败 " << failed_nav_
              << "，成功率 " << std::fixed << std::setprecision(1) << success_rate << "%）\n";
    log_file_ << "累计导航距离：" << std::fixed << std::setprecision(2) << cumulative_distance_m << " 米";
    if (missing_distance_count_ > 0) {
        log_file_ << "（" << missing_distance_count_ << " 次未获取里程计，本数值不完整）";
    }
    log_file_ << "\n";
    writeSeparatedSummaryTables();

    if (test_end_state_ == TestEndState::COMPLETED && failed_nav_ == 0) {
        log_file_ << "\n测试结果：计划已完成，全部导航成功\n";
    } else if (test_end_state_ == TestEndState::COMPLETED) {
        log_file_ << "\n测试结果：计划已完成，但存在导航失败\n";
    } else {
        log_file_ << "\n测试结果：计划未正常完成，请查看测试状态和原因\n";
    }
    log_file_ << "============================================\n";
    log_file_.flush();
    summary_written_ = true;

    LOOP_RCLCPP_INFO(node_->get_logger(),
                     "导航测试汇总已写入 nav_error_log.txt：状态=%s，共 %zu 次，成功 %d，失败 %d。",
                     testEndStateText(), total_nav, succeeded_nav_, failed_nav_);
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
    return {BT::InputPort<std::string>("points", "只包含轮数、点位顺序和停留时间的导航计划"),
            BT::InputPort<std::string>("assets", "当前地图的 map_assets.json")};
}

BT::NodeStatus NavigateToFirstPointAndStop::onStart() {
    if (goals_.empty()) {
        const auto plan_path = getInput<std::string>("points");
        const auto assets_path = getInput<std::string>("assets");
        if (!plan_path || !assets_path) {
            LOOP_RCLCPP_ERROR(node_->get_logger(), "[LowBattery] 行为树缺少 points 或 assets 输入端口。");
            return BT::NodeStatus::FAILURE;
        }
        std::vector<std::string> unused_goal_ids;
        std::vector<double> unused_goal_wait_times_sec;
        int unused_total_rounds = 0;
        goals_ = parseNavPoints(plan_path.value(), assets_path.value(), node_->get_logger(), unused_goal_ids,
                                unused_total_rounds, unused_goal_wait_times_sec);
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
    goal_msg.pose = goals_[0];  // 低电量停止点固定使用导航计划中的第一个点。
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
