#include "nav2_nav_monitor/nav_monitor_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <vector>

#include <nlohmann/json.hpp>

namespace nav2_nav_monitor
{

namespace
{
constexpr uint8_t kStatusAccepted = 1;
constexpr uint8_t kStatusExecuting = 2;
constexpr uint8_t kStatusSucceeded = 4;
constexpr uint8_t kStatusCanceled = 5;
constexpr uint8_t kStatusAborted = 6;

std::string statusToString(uint8_t status)
{
  switch (status) {
    case kStatusSucceeded: return "succeeded";
    case kStatusCanceled: return "canceled";
    case kStatusAborted: return "aborted";
    default: return "unknown";
  }
}

std::array<uint8_t, 16> toArray(const std::array<uint8_t, 16> & uuid)
{
  return uuid;
}
}  // namespace

NavMonitorNode::NavMonitorNode()
: rclcpp::Node("nav2_nav_monitor")
{
  declare_parameter("enable", false);
  declare_parameter("output_dir", std::string("~/.dexe/navigation/nav_monitor"));
  declare_parameter("max_retained_files", 100);
  declare_parameter("passing_path_sample_period", 0.5);
  declare_parameter("goal_pose_echo_topic", std::string("/nav_monitor/goal_pose_echo"));
  declare_parameter(
    "goal_behavior_tree_echo_topic", std::string("/nav_monitor/goal_behavior_tree_echo"));
  declare_parameter(
    "global_path_echo_topic", std::string("/nav_monitor/current_global_path"));
  declare_parameter("nav_status_topic", std::string("/navigate_to_pose/_action/status"));
  declare_parameter("nav_feedback_topic", std::string("/navigate_to_pose/_action/feedback"));
  // 双速度源：控制器原始输出（velocity_smoother 之前）+ 平滑后（velocity_smoother 之后）
  declare_parameter("cmd_vel_topic", std::string("cmd_vel_nav"));
  declare_parameter("smoothed_vel_topic", std::string("control/cmd_vel_nav"));
  // 规划期望速度（controller_server 回显）与 odom 实测
  declare_parameter("desired_vel_topic", std::string("/nav_monitor/desired_velocity"));
  declare_parameter(
    "controller_target_topic", std::string("/nav_monitor/controller_target"));
  declare_parameter("odom_topic", std::string("odom"));
  declare_parameter(
    "controller_phase_topic", std::string("/nav_monitor/controller_phase"));

  get_parameter("enable", enable_);
  get_parameter("output_dir", output_dir_);
  get_parameter("max_retained_files", max_retained_files_);
  get_parameter("passing_path_sample_period", passing_path_sample_period_sec_);

  if (!enable_) {
    RCLCPP_INFO(get_logger(), "nav2_nav_monitor disabled (enable=false), 不订阅任何话题。");
    return;
  }
  if (output_dir_.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "nav2_nav_monitor enable=true 但 output_dir 为空，无法落盘，实际不生效。");
    return;
  }
  if (output_dir_.front() == '~') {
    const char * home = std::getenv("HOME");
    if (home != nullptr) {
      output_dir_ = std::string(home) + output_dir_.substr(1);
    }
  }
  std::error_code ec;
  std::filesystem::create_directories(output_dir_, ec);
  if (ec) {
    RCLCPP_ERROR(
      get_logger(), "创建 output_dir 失败：%s (%s)", output_dir_.c_str(), ec.message().c_str());
    return;
  }

  // 把解析后的 output_dir 写入固定 marker 文件——分析脚本离线运行（节点已停）
  // 时据此找到自定义目录，见 scripts/nav_task_common.py 的解析顺序
  const char * home = std::getenv("HOME");
  if (home != nullptr) {
    const std::string marker_dir = std::string(home) + "/.dexe/navigation";
    std::filesystem::create_directories(marker_dir, ec);
    std::ofstream marker(marker_dir + "/.nav_monitor_last_output_dir");
    if (marker.is_open()) {
      marker << output_dir_;
    }
  }

  std::string goal_pose_echo_topic, goal_bt_echo_topic, global_path_echo_topic;
  std::string nav_status_topic, nav_feedback_topic;
  get_parameter("goal_pose_echo_topic", goal_pose_echo_topic);
  get_parameter("goal_behavior_tree_echo_topic", goal_bt_echo_topic);
  get_parameter("global_path_echo_topic", global_path_echo_topic);
  get_parameter("nav_status_topic", nav_status_topic);
  get_parameter("nav_feedback_topic", nav_feedback_topic);
  get_parameter("cmd_vel_topic", cmd_vel_topic_);
  get_parameter("smoothed_vel_topic", smoothed_vel_topic_);
  get_parameter("desired_vel_topic", desired_vel_topic_);
  get_parameter("controller_target_topic", controller_target_topic_);
  get_parameter("odom_topic", odom_topic_);

  goal_pose_echo_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_pose_echo_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&NavMonitorNode::goalPoseEchoCallback, this, std::placeholders::_1));
  goal_bt_echo_sub_ = create_subscription<std_msgs::msg::String>(
    goal_bt_echo_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&NavMonitorNode::goalBtEchoCallback, this, std::placeholders::_1));
  global_path_echo_sub_ = create_subscription<nav_msgs::msg::Path>(
    global_path_echo_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&NavMonitorNode::globalPathEchoCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&NavMonitorNode::cmdVelCallback, this, std::placeholders::_1));
  smoothed_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    smoothed_vel_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&NavMonitorNode::smoothedVelCallback, this, std::placeholders::_1));
  desired_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    desired_vel_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&NavMonitorNode::desiredVelCallback, this, std::placeholders::_1));
  controller_target_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
    controller_target_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&NavMonitorNode::controllerTargetCallback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&NavMonitorNode::odomCallback, this, std::placeholders::_1));
  std::string controller_phase_topic;
  get_parameter("controller_phase_topic", controller_phase_topic);
  phase_echo_sub_ = create_subscription<std_msgs::msg::String>(
    controller_phase_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&NavMonitorNode::phaseEchoCallback, this, std::placeholders::_1));
  feedback_sub_ = create_subscription<nav2_msgs::action::NavigateToPose_FeedbackMessage>(
    nav_feedback_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&NavMonitorNode::feedbackCallback, this, std::placeholders::_1));
  status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
    nav_status_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
    std::bind(&NavMonitorNode::statusCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(), "nav2_nav_monitor 已启用，output_dir=%s，max_retained_files=%d",
    output_dir_.c_str(), max_retained_files_);
}

void NavMonitorNode::goalPoseEchoCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (current_task_.has_value()) {
    RCLCPP_WARN(
      get_logger(),
      "收到新的 goal_pose_echo，但上一次导航任务尚未收到终态（可能是抢占）——"
      "用当前已知信息强制收尾上一条记录，再开始新任务。");
    finalizeAndWrite("preempted_unknown");
  }
  openNewTask(*msg);
}

void NavMonitorNode::openNewTask(const geometry_msgs::msg::PoseStamped & goal_echo)
{
  NavTaskRecord task;
  task.start_time = rclcpp::Time(goal_echo.header.stamp);
  if (task.start_time.nanoseconds() == 0) {
    task.start_time = this->now();  // 回显方未填 stamp 时兜底，保证相对时间戳有效
  }
  task.start_wall_time = std::time(nullptr);
  task.goal_pose = goal_echo;
  task.behavior_tree = pending_bt_name_;  // 大概率还是上一条的值，goal_bt_echo 稍后到达会覆盖
  task.last_passing_sample_time = task.start_time;
  current_task_ = std::move(task);
}

void NavMonitorNode::goalBtEchoCallback(const std_msgs::msg::String::SharedPtr msg)
{
  pending_bt_name_ = msg->data;
  if (current_task_.has_value()) {
    current_task_->behavior_tree = msg->data;
  }
}

void NavMonitorNode::globalPathEchoCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (!current_task_.has_value()) {
    return;
  }
  const double length = pathLength(*msg);
  if (current_task_->planned_paths.empty()) {
    current_task_->planned_path_length_first = length;
  }
  std::vector<std::array<double, 3>> poses;
  poses.reserve(msg->poses.size());
  for (const auto & p : msg->poses) {
    poses.push_back(
      {p.pose.position.x, p.pose.position.y, yawFromQuaternion(p.pose.orientation)});
  }
  current_task_->planned_paths.push_back(std::move(poses));
  current_task_->replanning_count++;
}

void NavMonitorNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!current_task_.has_value()) {
    return;
  }
  const double t = (this->now() - current_task_->start_time).seconds();
  current_task_->cmd_vel_samples.push_back({t, msg->linear.x, msg->angular.z});
}

void NavMonitorNode::smoothedVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!current_task_.has_value()) {
    return;
  }
  const double t = (this->now() - current_task_->start_time).seconds();
  current_task_->smoothed_vel_samples.push_back({t, msg->linear.x, msg->angular.z});
}

void NavMonitorNode::desiredVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!current_task_.has_value()) {
    return;
  }
  const double t = (this->now() - current_task_->start_time).seconds();
  current_task_->desired_vel_samples.push_back({t, msg->linear.x, msg->angular.z});
}

void NavMonitorNode::controllerTargetCallback(
  const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  if (!current_task_.has_value()) {
    return;
  }
  const double t = (this->now() - current_task_->start_time).seconds();
  current_task_->controller_target_samples.push_back({t, msg->vector.x, msg->vector.y});
}

void NavMonitorNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!current_task_.has_value()) {
    return;
  }
  const double t = (this->now() - current_task_->start_time).seconds();
  current_task_->odom_vel_samples.push_back(
    {t, msg->twist.twist.linear.x, msg->twist.twist.angular.z});
}

void NavMonitorNode::phaseEchoCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (!current_task_.has_value()) {
    return;
  }
  const double t = (this->now() - current_task_->start_time).seconds();
  current_task_->phase_transitions.emplace_back(t, msg->data);
}

void NavMonitorNode::feedbackCallback(
  const nav2_msgs::action::NavigateToPose_FeedbackMessage::SharedPtr msg)
{
  if (!current_task_.has_value()) {
    return;
  }
  auto & task = *current_task_;
  const auto & fb = msg->feedback;

  // 主认领：feedback 外层自带正在执行 goal 的 id，首帧即认领
  if (!task.goal_id_claimed) {
    task.tracked_goal_id = msg->goal_id.uuid;
    task.goal_id_claimed = true;
  }

  if (!task.start_pose_set) {
    task.start_pose = fb.current_pose;
    task.start_pose_set = true;
  }
  task.last_known_pose = fb.current_pose;
  task.last_known_pose_set = true;
  task.number_of_recoveries = fb.number_of_recoveries;
  task.distance_remaining_final = fb.distance_remaining;
  task.navigation_time_final = rclcpp::Duration(fb.navigation_time).seconds();

  const rclcpp::Time now = fb.current_pose.header.stamp.sec == 0 ? this->now() :
    rclcpp::Time(fb.current_pose.header.stamp);
  if ((now - task.last_passing_sample_time).seconds() >= passing_path_sample_period_sec_ ||
    task.passing_path.empty())
  {
    const double t = (this->now() - task.start_time).seconds();
    const double x = fb.current_pose.pose.position.x;
    const double y = fb.current_pose.pose.position.y;
    const double yaw = yawFromQuaternion(fb.current_pose.pose.orientation);
    if (!task.passing_path.empty()) {
      const auto & last = task.passing_path.back();
      task.passing_length += std::hypot(x - last[1], y - last[2]);
    }
    task.passing_path.push_back({t, x, y, yaw});
    task.last_passing_sample_time = now;
  }
}

void NavMonitorNode::statusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
{
  if (!current_task_.has_value()) {
    return;
  }
  auto & task = *current_task_;

  if (!task.goal_id_claimed) {
    // 兜底认领（主认领在 feedbackCallback）：任务开启后，第一条恰好只有一个非终态 goal_id 的状态数组，
    // 认领为本任务的 goal_id（/navigate_to_pose 同一时刻只有一个活跃
    // goal，这个假设在本项目当前用法下成立，不额外改 bt_navigator 透传 goal_id）
    int live_count = 0;
    std::array<uint8_t, 16> candidate{};
    for (const auto & s : msg->status_list) {
      if (s.status == kStatusAccepted || s.status == kStatusExecuting) {
        live_count++;
        candidate = toArray(s.goal_info.goal_id.uuid);
      }
    }
    if (live_count == 1) {
      task.tracked_goal_id = candidate;
      task.goal_id_claimed = true;
    }
  }

  for (const auto & s : msg->status_list) {
    const auto uuid = toArray(s.goal_info.goal_id.uuid);
    const auto prev_it = last_known_status_.find(uuid);
    const bool changed = prev_it == last_known_status_.end() || prev_it->second != s.status;
    last_known_status_[uuid] = s.status;

    if (!changed || !task.goal_id_claimed || uuid != task.tracked_goal_id) {
      continue;
    }
    if (s.status == kStatusSucceeded || s.status == kStatusCanceled ||
      s.status == kStatusAborted)
    {
      finalizeAndWrite(statusToString(s.status));
      return;
    }
  }
}

void NavMonitorNode::finalizeAndWrite(const std::string & result)
{
  if (!current_task_.has_value()) {
    return;
  }
  NavTaskRecord task = std::move(*current_task_);
  current_task_.reset();

  const rclcpp::Time end_time = this->now();
  const geometry_msgs::msg::PoseStamped & reached =
    task.last_known_pose_set ? task.last_known_pose :
    (task.start_pose_set ? task.start_pose : task.goal_pose);

  const double err_x = reached.pose.position.x - task.goal_pose.pose.position.x;
  const double err_y = reached.pose.position.y - task.goal_pose.pose.position.y;
  const double err_dist = std::hypot(err_x, err_y);
  double err_yaw = yawFromQuaternion(reached.pose.orientation) -
    yawFromQuaternion(task.goal_pose.pose.orientation);
  while (err_yaw > M_PI) {err_yaw -= 2.0 * M_PI;}
  while (err_yaw < -M_PI) {err_yaw += 2.0 * M_PI;}
  // 纵向/横向分解（goal 朝向系）：与 goal checker 的 lon/lat 容差同一语义
  const double goal_yaw = yawFromQuaternion(task.goal_pose.pose.orientation);
  const double err_lon = err_x * std::cos(goal_yaw) + err_y * std::sin(goal_yaw);
  const double err_lat = -err_x * std::sin(goal_yaw) + err_y * std::cos(goal_yaw);

  // 每个速度源的统计 + 原始采样（t,vx,wz），统计量按绝对值计
  auto vel_block = [](const std::string & topic,
      const std::vector<std::array<double, 3>> & samples) {
      double sum_v = 0.0, max_v = 0.0, sum_w = 0.0, max_w = 0.0;
      for (const auto & s : samples) {
        sum_v += std::fabs(s[1]);
        max_v = std::max(max_v, std::fabs(s[1]));
        sum_w += std::fabs(s[2]);
        max_w = std::max(max_w, std::fabs(s[2]));
      }
      const double n = samples.empty() ? 1.0 : static_cast<double>(samples.size());
      nlohmann::json b;
      b["topic"] = topic;
      b["avg_linear_speed"] = sum_v / n;
      b["max_linear_speed"] = max_v;
      b["avg_angular_speed"] = sum_w / n;
      b["max_angular_speed"] = max_w;
      b["samples"] = samples;
      return b;
    };

  nlohmann::json j;
  j["start_time"] = task.start_time.seconds();
  j["end_time"] = end_time.seconds();
  j["start_pose"] = {
    {"x", task.start_pose.pose.position.x}, {"y", task.start_pose.pose.position.y},
    {"yaw", yawFromQuaternion(task.start_pose.pose.orientation)}};
  j["goal_pose"] = {
    {"x", task.goal_pose.pose.position.x}, {"y", task.goal_pose.pose.position.y},
    {"yaw", yawFromQuaternion(task.goal_pose.pose.orientation)}};
  j["behavior_tree"] = task.behavior_tree;
  j["result"] = result;
  j["number_of_recoveries"] = task.number_of_recoveries;
  j["distance_remaining_final"] = task.distance_remaining_final;
  j["navigation_time_final"] = task.navigation_time_final;
  j["goal_reached_pose"] = {
    {"x", reached.pose.position.x}, {"y", reached.pose.position.y},
    {"yaw", yawFromQuaternion(reached.pose.orientation)}};
  j["goal_reached_error"] = {
    {"x", err_x}, {"y", err_y}, {"yaw", err_yaw}, {"distance", err_dist},
    {"longitudinal", err_lon}, {"lateral", err_lat}};
  j["planned_path_length_first"] = task.planned_path_length_first;
  j["replanning_count"] = task.replanning_count;
  j["passing_length"] = task.passing_length;
  j["desired_vel"] = vel_block(desired_vel_topic_, task.desired_vel_samples);
  {
    double sum_v = 0.0, max_v = 0.0, sum_a = 0.0, max_a = 0.0;
    for (const auto & sample : task.controller_target_samples) {
      sum_v += std::fabs(sample[1]);
      max_v = std::max(max_v, std::fabs(sample[1]));
      sum_a += std::fabs(sample[2]);
      max_a = std::max(max_a, std::fabs(sample[2]));
    }
    const double count = task.controller_target_samples.empty() ?
      1.0 : static_cast<double>(task.controller_target_samples.size());
    j["controller_target"] = {
      {"topic", controller_target_topic_},
      {"avg_linear_speed", sum_v / count},
      {"max_linear_speed", max_v},
      {"avg_abs_acceleration", sum_a / count},
      {"max_abs_acceleration", max_a},
      {"samples", task.controller_target_samples}};
  }
  j["cmd_vel"] = vel_block(cmd_vel_topic_, task.cmd_vel_samples);
  j["smoothed_vel"] = vel_block(smoothed_vel_topic_, task.smoothed_vel_samples);
  j["odom_vel"] = vel_block(odom_topic_, task.odom_vel_samples);
  {
    nlohmann::json trans = nlohmann::json::array();
    for (const auto & [t, name] : task.phase_transitions) {
      trans.push_back({t, name});
    }
    j["phase_transitions"] = trans;  // 每项 [t, 阶段名]，t 为相对起始秒
  }
  j["passing_path"] = task.passing_path;  // 每项 [t,x,y,yaw]，t 为相对起始秒
  j["planned_paths"] = task.planned_paths;  // 每条路径每项 [x,y,yaw]

  char ts[32];
  std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&task.start_wall_time));
  const std::string filename =
    output_dir_ + "/nav_task_" + std::string(ts) + "_" + result + ".json";
  std::ofstream ofs(filename);
  if (ofs.is_open()) {
    ofs << j.dump(2);
    ofs.close();
    RCLCPP_INFO(get_logger(), "导航任务记录已保存：%s", filename.c_str());
  } else {
    RCLCPP_ERROR(get_logger(), "无法写入导航任务记录：%s", filename.c_str());
  }

  pruneOldFiles();
}

void NavMonitorNode::pruneOldFiles()
{
  namespace fs = std::filesystem;
  std::error_code ec;
  if (!fs::exists(output_dir_, ec)) {
    return;
  }
  std::vector<fs::path> files;
  for (const auto & entry : fs::directory_iterator(output_dir_, ec)) {
    if (entry.path().extension() == ".json") {
      files.push_back(entry.path());
    }
  }
  if (static_cast<int>(files.size()) <= max_retained_files_) {
    return;
  }
  std::sort(
    files.begin(), files.end(),
    [](const fs::path & a, const fs::path & b) {
      return fs::last_write_time(a) < fs::last_write_time(b);
    });
  const size_t remove_count = files.size() - static_cast<size_t>(max_retained_files_);
  for (size_t i = 0; i < remove_count; ++i) {
    fs::remove(files[i], ec);
  }
}

double NavMonitorNode::pathLength(const nav_msgs::msg::Path & path)
{
  double length = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    length += std::hypot(
      path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x,
      path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y);
  }
  return length;
}

double NavMonitorNode::yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace nav2_nav_monitor
