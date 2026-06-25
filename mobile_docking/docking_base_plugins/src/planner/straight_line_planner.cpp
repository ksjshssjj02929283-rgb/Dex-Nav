#include "docking_base_plugins/planner/straight_line_planner.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace docking_base_plugins
{

namespace
{
// 梯形速度剖面：弧长 s∈[0,length] 处速度幅值（>=0）。够长→梯形(到 v_max)，过短→对称三角形。
// 与 scripts/validate_velocity_profile.py 同一算法（已数值验证 accel≤a_max、末端速度=0）。
double trapezoidalSpeed(double s, double length, double v_max, double a_max)
{
  if (length <= 1e-9 || v_max <= 0.0 || a_max <= 0.0) {return 0.0;}
  s = std::clamp(s, 0.0, length);
  const double d_acc = v_max * v_max / (2.0 * a_max);   // 0→v_max 加速距离（=减速距离）
  if (2.0 * d_acc <= length) {                          // 梯形
    if (s < d_acc) {return std::sqrt(2.0 * a_max * s);}
    if (s <= length - d_acc) {return v_max;}
    return std::sqrt(std::max(0.0, 2.0 * a_max * (length - s)));
  }
  const double s_peak = length / 2.0;                   // 三角形（峰值 < v_max）
  if (s < s_peak) {return std::sqrt(2.0 * a_max * s);}
  return std::sqrt(std::max(0.0, 2.0 * a_max * (length - s)));
}
}  // namespace

void StraightLinePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  using nav2_util::declare_parameter_if_not_declared;
  declare_parameter_if_not_declared(node_, name_ + ".path_resolution", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node_, name_ + ".max_accel", rclcpp::ParameterValue(0.2));
  node_->get_parameter(name_ + ".path_resolution", path_resolution_);
  node_->get_parameter(name_ + ".max_accel", max_accel_);
}

nav_interfaces::msg::RobotTrajectory StraightLinePlanner::createTrajectory(
  const nav_msgs::msg::Path & coarse_path,
  const geometry_msgs::msg::PoseStamped & /*robot_pose*/,
  double vel_limit, bool reverse)
{
  nav_interfaces::msg::RobotTrajectory traj;
  traj.header = coarse_path.header;

  // 单点/无效：原样回传几何点（速度 0），由 controller setSpeedLimit 兜底
  if (coarse_path.poses.size() < 2 || vel_limit <= 0.0) {
    for (const auto & p : coarse_path.poses) {
      nav_interfaces::msg::TrajectoryPoint tp;
      tp.pose = p;
      tp.decision_label = reverse ? 1 : 0;
      traj.points.push_back(tp);
    }
    return traj;
  }

  // 1) 等距重采样（位置线性插值 + 朝向 slerp）+ 累计弧长
  const double res = std::max(1e-3, path_resolution_);
  std::vector<geometry_msgs::msg::PoseStamped> dense;
  std::vector<double> arc;
  double total = 0.0;
  for (size_t i = 0; i + 1 < coarse_path.poses.size(); ++i) {
    const auto & a = coarse_path.poses[i];
    const auto & b = coarse_path.poses[i + 1];
    const double dx = b.pose.position.x - a.pose.position.x;
    const double dy = b.pose.position.y - a.pose.position.y;
    const double seg = std::hypot(dx, dy);
    const int n = std::max(1, static_cast<int>(std::ceil(seg / res)));
    tf2::Quaternion qa, qb;
    tf2::fromMsg(a.pose.orientation, qa);
    tf2::fromMsg(b.pose.orientation, qb);
    for (int k = 0; k < n; ++k) {  // [a, b) 避免与下一段起点重复
      const double t = static_cast<double>(k) / n;
      geometry_msgs::msg::PoseStamped p;
      p.header = coarse_path.header;
      p.pose.position.x = a.pose.position.x + dx * t;
      p.pose.position.y = a.pose.position.y + dy * t;
      p.pose.position.z = a.pose.position.z;
      tf2::Quaternion q = qa.slerp(qb, t);
      q.normalize();
      p.pose.orientation = tf2::toMsg(q);
      if (!dense.empty()) {
        total += std::hypot(
          p.pose.position.x - dense.back().pose.position.x,
          p.pose.position.y - dense.back().pose.position.y);
      }
      dense.push_back(p);
      arc.push_back(total);
    }
  }
  {  // 收尾保留终点
    geometry_msgs::msg::PoseStamped p = coarse_path.poses.back();
    p.header = coarse_path.header;
    if (!dense.empty()) {
      total += std::hypot(
        p.pose.position.x - dense.back().pose.position.x,
        p.pose.position.y - dense.back().pose.position.y);
    }
    dense.push_back(p);
    arc.push_back(total);
  }

  // 2) 逐点纵向速度（梯形剖面）+ 段语义
  const double length = total;
  traj.points.reserve(dense.size());
  for (size_t i = 0; i < dense.size(); ++i) {
    nav_interfaces::msg::TrajectoryPoint tp;
    tp.pose = dense[i];
    const double v = trapezoidalSpeed(arc[i], length, vel_limit, max_accel_);
    tp.longitudinal_velocity = reverse ? -v : v;
    tp.path_kappa = 0.0;                    // 直线
    tp.decision_label = reverse ? 1 : 0;    // 1=REVERSE / 0=NORMAL
    traj.points.push_back(tp);
  }
  return traj;
}

}  // namespace docking_base_plugins

PLUGINLIB_EXPORT_CLASS(docking_base_plugins::StraightLinePlanner, docking_base_core::DockingPlanner)
