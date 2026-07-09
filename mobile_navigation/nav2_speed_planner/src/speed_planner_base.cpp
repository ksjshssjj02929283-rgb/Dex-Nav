// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 速度规划器抽象基类实现：Phase A 约束包络（障碍 → 巡航 → 曲率 → 末端蠕行，
// 逐点 min 叠加，安全置零绝对优先）+ Phase B 纯虚 hook。
// base 不含任何具体速度剖面算法（恒加速度剖面在 Trapezoidal，jerk 限制剖面在 SCurve）。
#include "nav2_speed_planner/speed_planner_base.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"

namespace nav2_speed_planner
{

// ============================ 生命周期 ============================

void SpeedPlannerBase::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = parent.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_speed", rclcpp::ParameterValue(0.6));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_speed", rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".accel", rclcpp::ParameterValue(0.35));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".decel", rclcpp::ParameterValue(-0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".curve_compute_dist", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".regulated_linear_scaling_min_radius", rclcpp::ParameterValue(1.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_curvature_speed", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".constant_velocity_dist_threshold", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".curve_lead_in_distance", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".curve_lead_out_distance", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".enable_constant_velocity_while_turning", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".end_speed", rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".stop_distance", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".collision_stop_distance", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".obstacle_cost_threshold", rclcpp::ParameterValue(254));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".obstacle_start_exempt_distance", rclcpp::ParameterValue(0.25));

  node->get_parameter(plugin_name_ + ".max_speed", max_speed_);
  node->get_parameter(plugin_name_ + ".min_speed", min_speed_);
  node->get_parameter(plugin_name_ + ".accel", accel_);
  node->get_parameter(plugin_name_ + ".decel", decel_);
  node->get_parameter(plugin_name_ + ".curve_compute_dist", curve_compute_dist_);
  node->get_parameter(
    plugin_name_ + ".regulated_linear_scaling_min_radius", regulated_linear_scaling_min_radius_);
  node->get_parameter(plugin_name_ + ".min_curvature_speed", min_curvature_speed_);
  node->get_parameter(
    plugin_name_ + ".constant_velocity_dist_threshold", constant_velocity_dist_threshold_);
  node->get_parameter(plugin_name_ + ".curve_lead_in_distance", curve_lead_in_distance_);
  node->get_parameter(plugin_name_ + ".curve_lead_out_distance", curve_lead_out_distance_);
  node->get_parameter(
    plugin_name_ + ".enable_constant_velocity_while_turning",
    enable_constant_velocity_while_turning_);
  node->get_parameter(plugin_name_ + ".end_speed", end_speed_);
  end_speed_ = std::max(end_speed_, 0.0);
  node->get_parameter(plugin_name_ + ".stop_distance", stop_distance_);
  node->get_parameter(plugin_name_ + ".collision_stop_distance", collision_stop_distance_);
  node->get_parameter(plugin_name_ + ".obstacle_cost_threshold", obstacle_cost_threshold_);
  obstacle_cost_threshold_ = std::clamp(obstacle_cost_threshold_, 1, 254);
  node->get_parameter(
    plugin_name_ + ".obstacle_start_exempt_distance", obstacle_start_exempt_distance_);
  obstacle_start_exempt_distance_ = std::max(obstacle_start_exempt_distance_, 0.0);

  // 参数非法只告警不 abort（避免拖垮整个 nav 栈）。
  if (max_speed_ <= min_speed_) {
    RCLCPP_WARN(
      logger_, "[%s] max_speed (%.3f) should be > min_speed (%.3f).",
      plugin_name_.c_str(), max_speed_, min_speed_);
  }
  if (accel_ <= 0.0) {
    RCLCPP_WARN(logger_, "[%s] accel (%.3f) should be > 0.", plugin_name_.c_str(), accel_);
  }
  if (decel_ >= 0.0) {
    RCLCPP_WARN(logger_, "[%s] decel (%.3f) should be < 0.", plugin_name_.c_str(), decel_);
  }

  RCLCPP_INFO(logger_, "Configured SpeedPlanner plugin: %s", plugin_name_.c_str());
}

void SpeedPlannerBase::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up SpeedPlanner plugin: %s", plugin_name_.c_str());
  tf_.reset();
  costmap_ros_.reset();
}

void SpeedPlannerBase::activate()
{
  RCLCPP_INFO(logger_, "Activating SpeedPlanner plugin: %s", plugin_name_.c_str());
}

void SpeedPlannerBase::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating SpeedPlanner plugin: %s", plugin_name_.c_str());
}

void SpeedPlannerBase::setPlan(const nav_msgs::msg::Path & path)
{
  current_plan_ = path;
}

void SpeedPlannerBase::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

// ============================ 模板方法入口 ============================

nav_interfaces::msg::RobotTrajectory SpeedPlannerBase::computeSpeedPlan(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Twist & current_velocity)
{
  nav_interfaces::msg::RobotTrajectory traj;
  traj.header = path.header;
  path_frame_ = path.header.frame_id;

  // 构建内部工作结构。
  std::vector<TrajPointWork> work;
  work.reserve(path.poses.size());
  for (const auto & ps : path.poses) {
    TrajPointWork p;
    p.pose = ps;
    // nav_msgs/Path 无段语义字段：透传置默认（known_gap：未来由带语义 path 提供）。
    p.speed_hint = 0.0f;
    p.region_type = "";
    p.segment_id = "";
    // 相邻重复点去重（<0.1mm）：重复点破坏曲率三点拟合并使点距统计失真；
    // 姿态取后到者（保住终点朝向）。
    if (!work.empty()) {
      const auto & prev = work.back().pose.pose.position;
      const double dx = ps.pose.position.x - prev.x;
      const double dy = ps.pose.position.y - prev.y;
      if (dx * dx + dy * dy < 1e-8) {
        work.back().pose = p.pose;
        continue;
      }
    }
    work.push_back(std::move(p));
  }

  if (work.size() < 2) {
    // 点数不足：原样返回零速轨迹。
    for (const auto & p : work) {
      nav_interfaces::msg::TrajectoryPoint tp;
      tp.pose = p.pose;
      tp.longitudinal_velocity = 0.0;
      tp.path_kappa = 0.0;
      tp.speed_hint = p.speed_hint;
      tp.region_type = p.region_type;
      tp.segment_id = p.segment_id;
      traj.points.push_back(std::move(tp));
    }
    return traj;
  }

  // 初始限速取严：max_speed / path speed_hint / 外部限速。
  double external_limit = std::numeric_limits<double>::max();
  if (speed_limit_ > 0.0) {
    external_limit = speed_limit_is_percentage_ ?
      (max_speed_ * speed_limit_ / 100.0) : speed_limit_;
  }
  for (auto & p : work) {
    double init_limit = max_speed_;
    if (p.speed_hint > 0.0f) {
      init_limit = std::min(init_limit, static_cast<double>(p.speed_hint));
    }
    init_limit = std::min(init_limit, external_limit);
    p.speed_limit = init_limit;
  }

  curr_path_length_ = computePathLength(work);
  points_interval_ = curr_path_length_ / static_cast<double>(work.size() - 1);

  const double curr_vel = current_velocity.linear.x;

  // ====== Phase A 约束包络（逐点 min 叠加，安全置零绝对优先）======
  const bool has_obstacle = applyObstacleConstraint(work);   // A1 障碍 + 缓冲带置零
  const double cruise = computeCruiseLimit(work);            // A2 理想巡航求解（纯查询）
  setPlanSpeedLimit(work, cruise);                           //    巡航回填
  applyCurvatureConstraint(work, cruise);                    // A3 曲率约束
  applyEndSpeedConstraint(work);                             // A4 末端蠕行区

  // ====== Phase B hook：派生类生成运动学可行剖面 ======
  assignVelocityProfile(work, curr_vel, cruise);

  // 可观测：巡航上界 vs 实际峰值（"能否到最大限速"直接看日志）。
  double peak = 0.0;
  for (const auto & p : work) {
    peak = std::max(peak, p.speed_limit);
  }
  RCLCPP_DEBUG(
    logger_,
    "[%s] speed profile: cruise_limit=%.3f peak=%.3f reached=%s obstacle=%s points=%zu",
    plugin_name_.c_str(), cruise, peak,
    peak + 1e-6 >= cruise ? "true" : "false",
    has_obstacle ? "true" : "false", work.size());

  // 回填 RobotTrajectory。
  traj.points.reserve(work.size());
  for (const auto & p : work) {
    nav_interfaces::msg::TrajectoryPoint tp;
    tp.pose = p.pose;
    tp.longitudinal_velocity = p.speed_limit;
    tp.path_kappa = p.curvature;
    tp.acceleration = p.accel;
    tp.relative_time = rclcpp::Duration::from_seconds(p.rel_time);
    tp.speed_hint = p.speed_hint;
    tp.region_type = p.region_type;
    tp.segment_id = p.segment_id;
    traj.points.push_back(std::move(tp));
  }

  return traj;
}

// ============================ Phase A 约束包络 ============================

bool SpeedPlannerBase::applyObstacleConstraint(std::vector<TrajPointWork> & path)
{
  // A1 障碍约束（安全置零绝对优先）：首个障碍点及其后全部置零，
  // 障碍前 collision_stop_distance_ 缓冲带置零（index 0 为机器人当前位置，不动）。
  if (path.empty()) {
    return false;
  }

  // 路径与 costmap 坐标系对齐：FollowPath 路径通常在 map 系，而宿主的 local
  // costmap 在 odom 系——直接用路径坐标查询会相差 map→odom 修正量，把自由区
  // 路径按偏移量"平移"进墙带造成误停障。查询前统一变换到 costmap 系
  //（2D 刚体变换，每周期取一次 TF，逐点仅乘加）。
  double tx = 0.0, ty = 0.0, cos_t = 1.0, sin_t = 0.0;
  bool need_transform = false;
  if (costmap_ros_ && tf_) {
    const std::string costmap_frame = costmap_ros_->getGlobalFrameID();
    if (!path_frame_.empty() && !costmap_frame.empty() && path_frame_ != costmap_frame) {
      geometry_msgs::msg::TransformStamped to_costmap;
      try {
        to_costmap = tf_->lookupTransform(costmap_frame, path_frame_, tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        // 变换不可用（如启动期 map 尚未发布）：本周期跳过障碍约束，
        // 好过用错误坐标误停障；下一周期重试（receding-horizon）。
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 1000,
          "[%s] cannot transform path (%s) into costmap frame (%s), "
          "skipping obstacle constraint this cycle: %s",
          plugin_name_.c_str(), path_frame_.c_str(), costmap_frame.c_str(), ex.what());
        return false;
      }
      const double yaw = tf2::getYaw(to_costmap.transform.rotation);
      cos_t = std::cos(yaw);
      sin_t = std::sin(yaw);
      tx = to_costmap.transform.translation.x;
      ty = to_costmap.transform.translation.y;
      need_transform = true;
    }
  }

  const std::vector<double> arc_s = computeCumulativeArcLengths(path);
  // 全路径扫描并用 region_type 标记障碍语义（透传到 RobotTrajectory，供消费方与
  // 剖面日志评判使用；M11 真语义接入后此处改为叠加标记而非独占）：
  //   obstacle        — path 点真正落在障碍格内（触发停障）
  //   obstacle_exempt — 起点豁免区内的障碍命中（仅标记不置零：贴墙停放防起步死锁）
  //   obstacle_buffer — 首个障碍前的缓冲带（置零）
  size_t first_obstacle_index = path.size();
  for (size_t i = 0; i < path.size(); ++i) {
    double wx = path[i].pose.pose.position.x;
    double wy = path[i].pose.pose.position.y;
    if (need_transform) {
      const double px = wx;
      const double py = wy;
      wx = cos_t * px - sin_t * py + tx;
      wy = sin_t * px + cos_t * py + ty;
    }
    if (!isObstacleAt(wx, wy)) {
      continue;
    }
    if (arc_s[i] <= obstacle_start_exempt_distance_) {
      path[i].region_type = "obstacle_exempt";
      continue;
    }
    path[i].region_type = "obstacle";
    if (first_obstacle_index == path.size()) {
      first_obstacle_index = i;
    }
  }
  if (first_obstacle_index == path.size()) {
    return false;
  }
  for (size_t i = first_obstacle_index; i < path.size(); ++i) {
    path[i].speed_limit = 0.0;
  }
  const double buffer_start = arc_s[first_obstacle_index] - collision_stop_distance_;
  for (size_t j = first_obstacle_index; j-- > 1;) {
    if (arc_s[j] <= buffer_start) {
      break;
    }
    path[j].speed_limit = 0.0;
    if (path[j].region_type.empty()) {
      path[j].region_type = "obstacle_buffer";
    }
  }
  return true;
}

double SpeedPlannerBase::computeCruiseLimit(const std::vector<TrajPointWork> & path) const
{
  // A2 理想巡航上界（纯查询）：非零速点限速的最小值；安全零速点不参与求解。
  double speed_limit = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.size(); i++) {
    if (path[i].speed_limit < 1e-3) {
      continue;
    }
    if (speed_limit > path[i].speed_limit) {
      speed_limit = path[i].speed_limit;
    }
  }
  if (!(speed_limit < std::numeric_limits<double>::max())) {
    // 全部点被安全置零（起点即阻塞）。
    return 0.0;
  }
  return std::clamp(speed_limit, min_speed_, max_speed_);
}

void SpeedPlannerBase::setPlanSpeedLimit(
  std::vector<TrajPointWork> & path, const double speed_limit)
{
  // 双边钳制到 [min_speed_, max_speed_]；零速点（安全置零）跳过，不回填巡航速度。
  const double valid_speed_limit = std::clamp(speed_limit, min_speed_, max_speed_);

  for (size_t i = 0; i < path.size(); i++) {
    if (path[i].speed_limit < 1e-3) {
      continue;
    }
    path[i].speed_limit = valid_speed_limit;
  }
}

void SpeedPlannerBase::applyCurvatureConstraint(
  std::vector<TrajPointWork> & path, const double ex_speed_limit)
{
  if (path.size() < 3) {
    return;
  }

  size_t idx_dist = static_cast<size_t>(std::round(curve_compute_dist_ / points_interval_));
  const std::vector<double> curvature_vec = calcTrajectoryCurvatureFrom3Points(path, idx_dist);

  std::vector<TrajPointWork> & output = path;

  // 对路径上每个点：写入曲率，并按小半径处限速。
  std::vector<int> filtered_points;
  for (size_t i = 0; i < output.size(); ++i) {
    output.at(i).curvature = curvature_vec.at(i);
    const double radius = std::fabs(1.0 / curvature_vec.at(i));
    const double & min_rad = regulated_linear_scaling_min_radius_;
    if (radius > min_rad) {
      continue;
    }
    double v_curvature_max = ex_speed_limit * (1.0 - (std::fabs(radius - min_rad) / min_rad));

    // 如果该点速度已经小于阈值，则不再要求其大于阈值。
    v_curvature_max = std::max(v_curvature_max, min_curvature_speed_);
    if (output.at(i).speed_limit < v_curvature_max) {
      continue;
    }

    // 将速度限幅在 v_curvature_max 之下。
    if (output.at(i).speed_limit > v_curvature_max) {
      output.at(i).speed_limit = v_curvature_max;
      filtered_points.push_back(static_cast<int>(i));
      if (output.at(i).region_type.empty()) {
        output.at(i).region_type = "curve";  // 语义标注：曲率限速段（锚点可视化/日志）
      }
    }
  }

  // 转弯保持匀速：把相近的限速点聚成区间，区间内统一取最小速度。
  const double dist_threshold = constant_velocity_dist_threshold_;
  std::vector<std::tuple<size_t, size_t, double>> latacc_filtered_ranges;
  size_t start_index = 0;
  size_t end_index = 0;
  bool is_updated = false;
  double min_latacc_velocity = 0.0;
  for (size_t i = 0; i < filtered_points.size(); ++i) {
    const size_t index = static_cast<size_t>(filtered_points.at(i));

    if (is_updated == false) {
      start_index = index;
      end_index = index;
      min_latacc_velocity = output.at(index).speed_limit;
      is_updated = true;
      continue;
    }
    // 将 start_index 后 dist_threshold 内的点归入同一区间。
    double seg_dist = std::sqrt(
      std::pow(
        output.at(end_index).pose.pose.position.x - output.at(index).pose.pose.position.x, 2) +
      std::pow(
        output.at(end_index).pose.pose.position.y - output.at(index).pose.pose.position.y, 2));
    if (seg_dist < dist_threshold) {
      end_index = index;
      min_latacc_velocity = std::min(
        static_cast<double>(output.at(index).speed_limit), min_latacc_velocity);
    } else {
      latacc_filtered_ranges.emplace_back(start_index, end_index, min_latacc_velocity);
      start_index = index;
      end_index = index;
      min_latacc_velocity = output.at(index).speed_limit;
    }
  }
  if (is_updated) {
    latacc_filtered_ranges.emplace_back(start_index, end_index, min_latacc_velocity);
  }

  for (size_t i = 0; i < output.size(); ++i) {
    for (const auto & lat_acc_filtered_range : latacc_filtered_ranges) {
      const size_t s_idx = std::get<0>(lat_acc_filtered_range);
      const size_t e_idx = std::get<1>(lat_acc_filtered_range);
      const double min_v = std::get<2>(lat_acc_filtered_range);

      if (s_idx <= i && i <= e_idx && enable_constant_velocity_while_turning_) {
        // 只降不升：区间内安全置零点/更严约束不得被抬升到 min_v。
        output.at(i).speed_limit = std::min(output.at(i).speed_limit, min_v);
        break;
      }
    }
  }

  // O-6 曲率约束前后延伸（独立于 enable_constant_velocity_while_turning_）：
  // 弯道本身的进入坡道已由 Phase B 的 backward pass 生成，但那是"物理上刚好
  // 够减速"的最晚时刻——零安全边际。这里让弯前 curve_lead_in_distance_、
  // 弯后 curve_lead_out_distance_ 弧长内也直接钳到该弯的限速，提前量成为
  // Phase B 减速坡道的显式起点，给跟踪滞后（R22）留边际，也让转弯前的
  // 减速更从容。沿用与主循环相同的 region_type="curve" 标注，RViz 锚点
  // （curve_start/curve_end）与日志自动反映延伸后的真实范围，无需改动。
  if ((curve_lead_in_distance_ > 1e-6 || curve_lead_out_distance_ > 1e-6) &&
    !latacc_filtered_ranges.empty())
  {
    const std::vector<double> arc_s = computeCumulativeArcLengths(path);
    for (const auto & range : latacc_filtered_ranges) {
      const size_t s_idx = std::get<0>(range);
      const size_t e_idx = std::get<1>(range);
      const double min_v = std::get<2>(range);

      if (curve_lead_in_distance_ > 1e-6) {
        const double lead_in_start_s = arc_s[s_idx] - curve_lead_in_distance_;
        size_t i = s_idx;
        while (i-- > 1) {  // index 0 为状态戳，不参与（同 O-2 backward pass 约定）
          if (arc_s[i] < lead_in_start_s) {
            break;
          }
          output.at(i).speed_limit = std::min(output.at(i).speed_limit, min_v);
          if (output.at(i).region_type.empty()) {
            output.at(i).region_type = "curve";
          }
        }
      }
      if (curve_lead_out_distance_ > 1e-6) {
        const double lead_out_end_s = arc_s[e_idx] + curve_lead_out_distance_;
        for (size_t i = e_idx + 1; i < output.size(); ++i) {
          if (arc_s[i] > lead_out_end_s) {
            break;
          }
          output.at(i).speed_limit = std::min(output.at(i).speed_limit, min_v);
          if (output.at(i).region_type.empty()) {
            output.at(i).region_type = "curve";
          }
        }
      }
    }
  }
}

void SpeedPlannerBase::applyEndSpeedConstraint(std::vector<TrajPointWork> & path)
{
  // A4 末端蠕行区：距终点 stop_distance_ 内包络压到 end_speed_——低速到点保证精度。
  // min 保留障碍零速；end_speed_ < 1e-3（硬停配置）时无蠕行区（终点收敛交给 Phase B）。
  const double v_creep = std::max(end_speed_, 0.0);
  if (path.size() < 2 || stop_distance_ <= 0.0 || v_creep < 1e-3) {
    return;
  }
  const std::vector<double> arc_s = computeCumulativeArcLengths(path);
  for (size_t i = path.size(); i-- > 0;) {
    if (arc_s.back() - arc_s[i] > stop_distance_) {
      break;
    }
    path[i].speed_limit = std::min(path[i].speed_limit, v_creep);
    if (path[i].region_type.empty()) {
      path[i].region_type = "stop_zone";  // 语义标注：末端蠕行区（锚点可视化/日志）
    }
  }
}

// ============================ 共享工具 ============================

double SpeedPlannerBase::computePathLength(const std::vector<TrajPointWork> & path)
{
  double length = 0.0;
  for (int i = 1; i < static_cast<int>(path.size()); ++i) {
    double dx = path.at(i).pose.pose.position.x - path.at(i - 1).pose.pose.position.x;
    double dy = path.at(i).pose.pose.position.y - path.at(i - 1).pose.pose.position.y;
    length += std::sqrt(dx * dx + dy * dy);
  }
  return length;
}

std::vector<double> SpeedPlannerBase::computeCumulativeArcLengths(
  const std::vector<TrajPointWork> & path)
{
  std::vector<double> arc_s(path.size(), 0.0);
  for (size_t i = 1; i < path.size(); ++i) {
    const double dx = path[i].pose.pose.position.x - path[i - 1].pose.pose.position.x;
    const double dy = path[i].pose.pose.position.y - path[i - 1].pose.pose.position.y;
    arc_s[i] = arc_s[i - 1] + std::hypot(dx, dy);
  }
  return arc_s;
}

std::vector<double> SpeedPlannerBase::calcTrajectoryCurvatureFrom3Points(
  const std::vector<TrajPointWork> & path, size_t idx_dist)
{
  if (path.size() < 3) {
    return std::vector<double>(path.size(), 0.0);
  }

  // idx 不足时收缩 idx_dist。
  const auto max_idx_dist = static_cast<size_t>(std::floor((path.size() - 1) / 2.0));
  idx_dist = std::max(static_cast<size_t>(1), std::min(idx_dist, max_idx_dist));

  if (idx_dist < 1) {
    throw std::logic_error("idx_dist less than 1 is not expected");
  }

  // 三点圆拟合求曲率。
  std::vector<double> k_arr(path.size(), 0.0);
  for (size_t i = 1; i + 1 < path.size(); i++) {
    double curvature = 0.0;
    const auto p0 = path.at(i - std::min(idx_dist, i)).pose.pose;
    const auto p1 = path.at(i).pose.pose;
    const auto p2 = path.at(i + std::min(idx_dist, path.size() - 1 - i)).pose.pose;
    try {
      curvature = calcCurvature(p0, p1, p2);
    } catch (std::exception const & e) {
      if (clock_) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "%s", e.what());
      }
      if (i > 1) {
        curvature = k_arr.at(i - 1);  // 沿用前一点曲率
      } else {
        curvature = 0.0;
      }
    }
    k_arr.at(i) = curvature;
  }
  // 首尾点复制相邻点曲率。
  k_arr.at(0) = k_arr.at(1);
  k_arr.back() = k_arr.at(path.size() - 2);

  return k_arr;
}

double SpeedPlannerBase::calcCurvature(
  const geometry_msgs::msg::Pose & p1,
  const geometry_msgs::msg::Pose & p2,
  const geometry_msgs::msg::Pose & p3)
{
  // Menger 曲率：https://en.wikipedia.org/wiki/Menger_curvature
  auto dist = [](const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b) {
      double dx = a.position.x - b.position.x;
      double dy = a.position.y - b.position.y;
      return std::sqrt(dx * dx + dy * dy);
    };
  const double denominator = dist(p1, p2) * dist(p2, p3) * dist(p3, p1);
  if (std::fabs(denominator) < 1e-10) {
    throw std::runtime_error("points are too close for curvature calculation.");
  }
  return 2.0 *
         ((p2.position.x - p1.position.x) * (p3.position.y - p1.position.y) -
         (p2.position.y - p1.position.y) * (p3.position.x - p1.position.x)) / denominator;
}

bool SpeedPlannerBase::isObstacleAt(double x, double y) const
{
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_ ? costmap_ros_->getCostmap() : nullptr;
  if (costmap == nullptr) {
    return false;
  }
  unsigned int mx, my;
  if (!costmap->worldToMap(x, y, mx, my)) {
    return false;
  }
  const unsigned char cost = costmap->getCost(mx, my);
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return false;  // 未知区域不当障碍（阈值 254 时 255 会被 >= 误伤）
  }
  return cost >= static_cast<unsigned char>(obstacle_cost_threshold_);
}

}  // namespace nav2_speed_planner
