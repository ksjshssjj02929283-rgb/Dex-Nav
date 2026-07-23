// 导航任务级监控：以 /navigate_to_pose 的 goal 生命周期为唯一任务边界
//（controller_server 内部调用会因 BT 恢复重试在同一次导航内重入，
// 不能作任务边界）；开始信号靠 bt_navigator 的
// goal 回显话题（Humble 被动订阅拿不到别的 client 发的 goal 内容），结束/过程
// 信号复用标准 action 内省话题（status/feedback），零改动。

#ifndef NAV2_NAV_MONITOR__NAV_MONITOR_NODE_HPP_
#define NAV2_NAV_MONITOR__NAV_MONITOR_NODE_HPP_

#include <array>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"

namespace nav2_nav_monitor
{

// 一次导航任务的记录状态，从 goal_pose_echo 到达开始累积，到终态落盘后清空。
struct NavTaskRecord
{
  rclcpp::Time start_time;
  std::time_t start_wall_time = 0;

  geometry_msgs::msg::PoseStamped start_pose;
  bool start_pose_set = false;

  geometry_msgs::msg::PoseStamped goal_pose;
  std::string behavior_tree;

  // goal_id 关联：主认领源是 feedback 外层消息自带的 goal_id（导航期间持续
  // 发布，无时序依赖）；status 数组认领只作兜底——status 只在状态转移时发布，
  // 且 EXECUTING 转移发生在 goal 回显之前（任务尚未开启会被丢弃），实机上
  // 常常等不到"任务开启后
  // 首个非终态 goal_id"启发式认领——/navigate_to_pose 同一时刻只有一个活跃
  // goal，这个假设在本项目当前用法下成立）
  bool goal_id_claimed = false;
  std::array<uint8_t, 16> tracked_goal_id{};

  geometry_msgs::msg::PoseStamped last_known_pose;
  bool last_known_pose_set = false;

  int32_t number_of_recoveries = 0;
  double distance_remaining_final = 0.0;
  double navigation_time_final = 0.0;

  size_t replanning_count = 0;
  double planned_path_length_first = 0.0;
  std::vector<std::vector<std::array<double, 3>>> planned_paths;  // 每条 x,y,yaw

  std::vector<std::array<double, 4>> passing_path;  // 降频采样 t(相对起始),x,y,yaw
  double passing_length = 0.0;
  rclcpp::Time last_passing_sample_time;

  // 四速度源：规划期望 + 控制器原始输出 + velocity_smoother 平滑后 + odom 实测，
  // 各自带相对时间戳采样，供 V-T / V-S 曲线离线绘制
  std::vector<std::array<double, 3>> desired_vel_samples;   // t,v,0
  std::vector<std::array<double, 3>> controller_target_samples;  // t,v,a
  std::vector<std::array<double, 3>> cmd_vel_samples;       // t,vx,wz
  std::vector<std::array<double, 3>> smoothed_vel_samples;  // t,vx,wz
  std::vector<std::array<double, 3>> odom_vel_samples;      // t,vx,wz

  // 四阶段切换序列（controller_server setControllerPhase 回显）：t + 阶段名，
  // 供批量分析计算 prepare/align/track/approach 各段时长
  std::vector<std::pair<double, std::string>> phase_transitions;
};

class NavMonitorNode : public rclcpp::Node
{
public:
  NavMonitorNode();

private:
  void goalPoseEchoCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void goalBtEchoCallback(const std_msgs::msg::String::SharedPtr msg);
  void globalPathEchoCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void smoothedVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void desiredVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void controllerTargetCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void phaseEchoCallback(const std_msgs::msg::String::SharedPtr msg);
  void feedbackCallback(
    const nav2_msgs::action::NavigateToPose_FeedbackMessage::SharedPtr msg);
  void statusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg);

  void openNewTask(const geometry_msgs::msg::PoseStamped & goal_echo);
  void finalizeAndWrite(const std::string & result);
  void pruneOldFiles();

  static double pathLength(const nav_msgs::msg::Path & path);
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q);

  bool enable_{false};
  std::string output_dir_;
  int max_retained_files_{100};
  double passing_path_sample_period_sec_{0.5};
  std::string cmd_vel_topic_;
  std::string smoothed_vel_topic_;
  std::string desired_vel_topic_;
  std::string controller_target_topic_;
  std::string odom_topic_;

  std::optional<NavTaskRecord> current_task_;

  // 每个 goal_id 上一次观测到的 status，用来判定"是否是新的终态转移"，
  // 避免 GoalStatusArray 全量重发导致重复落盘。
  std::map<std::array<uint8_t, 16>, uint8_t> last_known_status_;

  std::string pending_bt_name_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_echo_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_bt_echo_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_echo_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr smoothed_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr desired_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr controller_target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr phase_echo_sub_;
  rclcpp::Subscription<nav2_msgs::action::NavigateToPose_FeedbackMessage>::SharedPtr
    feedback_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
};

}  // namespace nav2_nav_monitor

#endif  // NAV2_NAV_MONITOR__NAV_MONITOR_NODE_HPP_
