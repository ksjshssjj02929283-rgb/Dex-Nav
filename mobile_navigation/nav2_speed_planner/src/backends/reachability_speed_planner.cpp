// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 纯可达性滤波后端：后向减速 → 前向加速，两遍均带 jerk 斜率限制。
#include "nav2_speed_planner/backends/reachability_speed_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "nav2_speed_planner/core/profile_derivatives.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_speed_planner
{

namespace
{
constexpr double kArcEpsilon = 1e-9;
constexpr double kZeroSpeed = 1e-3;

/// 本段允许的加速度。四项取严，再由物理下限兜底：
///   ① a ≤ a_limit                        —— 加速度上限
///   ② a ≤ a_prev + j·Δt                  —— jerk 斜率，加速度不能跳着涨
///   ③ a ≤ a_cap + √(2j·(v_cap − v₁))     —— **提前收手**
///   ④ a ≤ (v_cap² − v²)/(2Δs)            —— 本段不得越过上界
///
/// ③ 对齐的是目标曲线的斜率 a_cap，不是 0：前后两条坡道相交时目标是一条
/// a=−0.25 的制动曲线，收到 0 再撞上去，交点处一步跳 0.25（实测 −6.2）。
/// 但 a_cap 只在**下降**时参与放宽——上升时收到 0 永远安全，曲率井出口包络
/// 陡升会把 a_cap 钳到 a_limit 使收手项全开，出井瞬间一步崩塌（实测 −9.5）。
///
/// ③ 的余量取段末 v₁ 而非段首 v：积分在一段内按恒加速度走，而收手公式假设
/// 加速度同期已开始下降；用段首余量算，一段就把余量吃超（实测巡航入口 a 从
/// 0.037 一步掉到 0.004）。v₁ 依赖 a，故并入下面的不动点迭代一起解。
///
/// ④ 让 `min(v_to, v_cap)` 那道钳位成为空操作。钳位一旦真的生效，写回值与
/// 积分值不一致，反算出的加速度就是个没走过的数，下一段的 ② 便建立在错误
/// 基准上——jerk 限制看着在做，实际是从一个跳变后的值起算。
///
/// ⚠ **试过把 ③ 换成「收手距离内的前瞻窗口」，三个变体全部净变差，已回退。**
///   动机是：把 a 按 j 收到 0 需走 0.77 m，而 ③ 只看下一个点（2 cm）。但窗口
///   里那条 `(V²−v²)/(2D)` 在 `V < v` 时会**强制制动**，与后向遍历抢活，且
///   恒加速度界比后向那条 jerk 限制的坡道更狠，剖面被扎到坡道以下再往回爬
///   （straight 0.85 → 3.28、hardstop 0.95 → 2.97）；把窗口下限钳 0 又等于
///   丢掉 ④ 的越界保护，全线崩到 6~11。根因是前向读的是**后向已改写过的**
///   包络，任何前瞻都在重算后向做过的事——这正是「独立滤波 + merge」结构
///   要解决的问题，不是加一项能补的。详见 decisions.md。
///
/// Δt、v₁ 与 a 互为因变量，迭代求不动点；映射单调下降且收缩，收敛呈振荡。
/// 此前固定迭代 3 次且无收敛判据，首点算出 0.1939 而真值 0.2154——
/// **结果由迭代次数决定，而不是由物理决定**。此处加判据，直到不再变化为止。
double allowedAccel(
  double v, double a_prev, double ds, double v_cap, double a_cap,
  double a_limit, double a_floor, double max_jerk, double dt_slope)
{
  const double landing = (v_cap * v_cap - v * v) / (2.0 * ds);
  double a = a_limit;
  for (int k = 0; k < 32; ++k) {
    const double sq = v * v + 2.0 * a * ds;
    const double v1 = sq > 0.0 ? std::sqrt(sq) : 0.0;
    const double v_sum = v + v1;
    // ② 的分母必须是**正时间序更早那一段**的用时——前向遍历时它是上一段
    //    （由调用方传入），后向遍历时正是本段（传 0，此处现算）。取错会让同一
    //    对加速度「倒着走合法、正着走违规」：zone 边界插点使 ds 从 20 mm 变
    //    5 mm，Δt 从 0.258 掉到 0.104，前向只允许 a 回升 0.052（实测 jerk 1.45）。
    const double dt = dt_slope > 0.0 ?
      dt_slope : (v_sum > kZeroSpeed ? 2.0 * ds / v_sum : 1.0);
    const double taper =
      std::min(a_cap, 0.0) + std::sqrt(2.0 * max_jerk * std::max(0.0, v_cap - v1));
    double next = std::min(std::min(a_limit, a_prev + max_jerk * dt), std::min(taper, landing));
    // ⚠ 这里**没有**对称的 jerk 下界（`a ≥ a_prev − j·Δt`），是刻意的：加过一版，
    //   离线全面回归（straight 0.85 → 6.49、curve_wells 1.50 → 6.87）——下界把
    //   首点撑在高位，包络随后只能用更急的修正拽回来，防住的抖动反而更大。
    next = std::max(next, a_floor);
    const bool settled = std::fabs(next - a) < 1e-12;
    a = next;
    if (settled) {
      break;
    }
  }
  return a;
}

/// 段 [i, i+1] 的通过时间。
///
/// 两端速度都为零时该段**永远走不完**，真实用时是无穷。此前返回常数 `1.0`，
/// 于是 `rel_time` 在零速带里每点 +1 秒：障碍场景（障碍占后 2 m）实测 113 个
/// 零速点把到点时间刷到 **118.08 s**（同族 SCurve 6.95 s）。该字段是发布给
/// 消费侧的 `TrajectoryPoint::relative_time`，这个数直接是错的；它还会污染
/// jerk 度量——`Δt = 1.0` 把零速带附近的 jerk 除小，等于**掩盖**问题。
///
/// 返回 0 让时间在停止处**冻结**：其后所有点共用同一个 `rel_time`，
/// 语义是"本计划走不到这里"。这既保持单调不减，也让按时间前瞻的消费侧
/// 落在停止点上——正是机器人该瞄准的位置。返回无穷会击穿下游算术。
double segmentDuration(double v_a, double v_b, double ds)
{
  const double v_avg = 0.5 * (std::max(v_a, 0.0) + std::max(v_b, 0.0));
  if (v_avg < kZeroSpeed) {
    return 0.0;
  }
  return ds / v_avg;
}

}  // namespace

void ReachabilitySpeedPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  SpeedPlannerBase::configure(parent, name, tf, costmap_ros);
  auto node = parent.lock();
  if (!node) {
    return;
  }
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_jerk", rclcpp::ParameterValue(0.5));
  node->get_parameter(plugin_name_ + ".max_jerk", max_jerk_);
  max_jerk_ = std::max(max_jerk_, 1e-3);
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".control_dt", rclcpp::ParameterValue(0.1));
  node->get_parameter(plugin_name_ + ".control_dt", control_dt_);
  control_dt_ = std::max(control_dt_, 1e-3);
}

void ReachabilitySpeedPlanner::generateProfile(
  std::vector<TrajPointWork> & work, double curr_vel, double curr_accel,
  double cruise_limit, bool /*terminal_brake_latched*/)
{
  if (work.size() < 2) {
    return;
  }
  if (cruise_limit < kZeroSpeed) {
    // 起点即被安全置零：保持零速剖面。
    return;
  }

  // 硬停配置（无末端蠕行区）：把终点锚成零，后向遍历自动生成收敛坡道。
  if (end_speed_ < kZeroSpeed) {
    lowerBound(work.back(), 0.0, ConstraintSource::kEndBoundary);
  }

  const std::vector<double> arc_s = computeCumulativeArcLengths(work);

  // 顺序：后向必须在前向之前。前向把 v[i+1] 压到可达值时结果仍不低于 v[i]，
  // 不会破坏已生成的减速坡道；反序则加速可行性无法保证。
  // 两条坡道相交处即为峰值——短路径的三角形剖面由此自动涌现，无需特判。
  filterBackward(work, arc_s);
  filterForward(work, arc_s, std::max(curr_vel, 0.0), curr_accel);
  enforceDecelFeasibility(work, arc_s, curr_accel);
  // 滤波输出**即剖面**，不再经跟踪器。Ruckig 速度模式跟踪一个位置相关目标
  // 本质是一阶滞后跟随器——它只知道"此刻目标多少"、不知道"必须在哪达到"，
  // 目标随弧长下降时永远在追。短路径上滞后吃掉全部余量（1 m 场景末点 0.41
  // vs end_speed 0.05）；目标若再随当前速度变化，还会构成自激反馈。
  fillProfileDerivatives(work, arc_s);

  // ── 不可行性上报 ────────────────────────────────────────────────
  // 本后端的输出**始终可执行**（每步取严/取松，无求解分支），故不可行性
  // 不表现为"算不出来"，而表现为"算出来的达不到要求"。两种形态都必须上报，
  // 否则消费侧看到的是一条外观正常的轨迹。

  // 形态二：终端距离物理上不够减到 end_speed。
  // 剖面已在满减速下尽力，这是**真实的物理不可行**，不是缺陷——
  // 与"求解器没算出来"分开表达，前者消费侧须响应，后者是开发者的事。
  if (end_speed_ >= kZeroSpeed &&
    work.back().velocity_upper_bound > end_speed_ + 1e-2)
  {
    last_planning_diagnostics_.terminal_infeasible = true;
    last_planning_diagnostics_.terminal_infeasible_kind =
      static_cast<int>(FeasibilityResult::Kind::kPhysical);
    last_planning_diagnostics_.solver_target_v = end_speed_;
    last_planning_diagnostics_.solver_v0 = work.back().velocity_upper_bound;
    last_planning_diagnostics_.solver_distance = arc_s.back();
  }

  // 形态一：初速高于起点包络。前向遍历已让曲线以物理极限收敛，
  // 此处只把「初值在可行集之外」告知消费侧。
  const double v_env_start = work.front().velocity_upper_bound;
  if (curr_vel > v_env_start + 1e-3) {
    last_planning_diagnostics_.terminal_infeasible = true;
    last_planning_diagnostics_.terminal_infeasible_kind =
      static_cast<int>(FeasibilityResult::Kind::kPhysical);
    last_planning_diagnostics_.solver_v0 = curr_vel;
    last_planning_diagnostics_.solver_vmax = v_env_start;
  }
}

void ReachabilitySpeedPlanner::integrateReachable(
  std::vector<TrajPointWork> & work, const std::vector<double> & arc_s,
  bool forward, double a_limit, double v_seed, double a_seed,
  ConstraintSource source) const
{
  const size_t n = work.size();
  auto idx = [n, forward](size_t p) {return forward ? p : n - 1 - p;};
  // 加速度下限：反向遍历里「减速」对应正的 a_limit，「加速」才是负向。
  const double a_floor = -(forward ? std::fabs(decel_) : accel_);

  // 目标曲线在遍历方向上的加速度快照。必须先整条算好——积分过程会就地改写
  // 包络，边走边算会读到自己刚写下的值，收手项就对齐到了自己身上。
  //
  // **只有前向遍历取这个量**。收手项要对齐的目标斜率，前提是目标曲线本身
  // 运动学可行；前向读到的是后向遍历滤过的包络（可行），而后向读到的是带
  // 阶跃的原始包络（曲率井壁、障碍置零），其"加速度"是任意大的数。
  // 后向若也取，井壁的 −8.75 被钳成 a_floor 后喂进收手项，taper 恒为负——
  // 明明有富余却一路把速度压到 0，剖面在井后停死再重启（实测 v=0.00000）。
  std::vector<double> env_accel(n, 0.0);
  for (size_t p = 0; forward && p + 1 < n; ++p) {
    const size_t i = idx(p), j = idx(p + 1);
    const double d = std::fabs(arc_s[j] - arc_s[i]);
    if (d <= kArcEpsilon) {
      continue;
    }
    const double vi = std::max(work[i].velocity_upper_bound, 0.0);
    const double vj = std::max(work[j].velocity_upper_bound, 0.0);
    env_accel[p] = std::clamp((vj * vj - vi * vi) / (2.0 * d), a_floor, a_limit);
  }

  double v = std::max(v_seed, 0.0);
  double a = std::clamp(a_seed, a_floor, a_limit);
  double dt_prev = 0.0;   // 上一段用时；0 = 尚无（首段回落到用本段）

  for (size_t step = 0; step + 1 < n; ++step) {
    const size_t from = idx(step);
    const size_t to = idx(step + 1);
    const double ds = std::fabs(arc_s[to] - arc_s[from]);
    if (ds <= kArcEpsilon) {
      continue;
    }
    const double v_cap = std::max(work[to].velocity_upper_bound, 0.0);
    // jerk 斜率的分母：
    //   · 前向首段 → **控制周期**。首点的加速度是"此刻执行的那一个"，每个控制
    //     周期都被下一版计划替换，seed → 首点这一次变化只有 control_dt 去完成，
    //     与该点所在路径段要走多久无关。静止起步时 Δs/v → ∞ 会让约束形同虚设，
    //     实机实测首点 a 一个周期内从 0.309 跳到 0.500（jerk 1.90，限值 0.5），
    //     且这是整条剖面上唯一超限处。
    //   · 前向内部段 → 上一段的通过时长（正时间序更早那一段）。
    //   · 后向 → 传 0，由 allowedAccel 现算本段时长（反向遍历时本段即更早那段）。
    const double dt_slope = forward ? (step == 0 ? control_dt_ : dt_prev) : 0.0;
    const double a_allow = allowedAccel(
      v, a, ds, v_cap, env_accel[step + 1], a_limit, a_floor, max_jerk_, dt_slope);

    const double sq = v * v + 2.0 * a_allow * ds;
    const double v_free = sq > 0.0 ? std::sqrt(sq) : 0.0;   // 未受包络压制的积分结果
    lowerBound(work[to], std::min(v_free, v_cap), source);
    const double v_real = std::max(work[to].velocity_upper_bound, 0.0);

    // 状态推进分两种情形：
    //
    // · **自走**（积分结果没被包络压住）：状态就是刚走出来的那段加速度。
    //
    // · **骑在包络上**（积分想更快、被包络压住）：下一段的 jerk 基准必须取
    //   **包络自身的斜率**，而不是把包络那一步反算成加速度。包络在井壁 / 障碍
    //   处是阶跃，反算值可达 −7.9；即便钳进物理箱成 −0.5，它也是个从未执行过
    //   的数，一旦当作 `a_prev + j·Δt` 的基准，就会把其后若干步的加速度上限
    //   全按成负值，`sqrt` 内转负 → 剖面归零。实测 startwell 井后 s=0.44
    //   出现孤立的 v=0.00000（两侧都是 0.10004）：不是不光滑，是**停死再重启**。
    // 骑在包络上时，下一段的 jerk 基准取 0（把包络局部视为平），而不是把包络
    // 那一步阶跃反算成加速度——后者在井壁处可达 −7.9，会把其后若干步全按住，
    // 剖面在井后停死再重启（实测 startwell s=0.44 处孤立 v=0.00000）。
    a = (v_real < v_free - 1e-9) ?
      env_accel[step + 1] :
      std::clamp((v_real * v_real - v * v) / (2.0 * ds), a_floor, a_limit);
    dt_prev = segmentDuration(v, v_real, ds);
    v = v_real;
  }
}

void ReachabilitySpeedPlanner::filterBackward(
  std::vector<TrajPointWork> & work, const std::vector<double> & arc_s) const
{
  const double d_max = std::fabs(decel_);
  if (d_max < 1e-6) {
    return;
  }
  // 从终点反向积分：种子速度 = 终点包络值，种子加速度 = 0（到达时制动已释放）。
  // 时间反演后这就是「以 jerk 限制的减速抵达每个下游约束」，且天然可组合——
  // 状态一路带着走，任意远的约束都会传播回来。
  integrateReachable(
    work, arc_s, false /* forward */, d_max,
    std::max(work.back().velocity_upper_bound, 0.0), 0.0,
    ConstraintSource::kKinematicBrake);
}

void ReachabilitySpeedPlanner::filterForward(
  std::vector<TrajPointWork> & work, const std::vector<double> & arc_s,
  double v_seed, double a_seed) const
{
  if (accel_ < 1e-6) {
    return;
  }
  // 初值只作为**种子**，不写进包络——包络是约束，初值是状态。
  const bool blocked_start = work.front().velocity_upper_bound < kZeroSpeed;
  // 剖面首点 = **实测速度**：机器人此刻就是这个速度，计划说别的既不可执行、
  // 也让消费侧无从判断该跟哪个。安全置零（阻塞）例外，不得被抬升。
  // ⚠ 不写这一点，首点会留着包络值，随后被向下可达遍当作"减不下来的下限"
  //   向后传播——剖面变成先减速再加速，起步段 jerk 发散。
  if (!blocked_start) {
    work.front().velocity_upper_bound = std::max(v_seed, 0.0);
  }
  // 加速度种子取实测/继承值：计划每周期重算，若每次都从零起坡，
  // 控制器采样到的永远是坡道起点，机器人升不上速度。
  integrateReachable(
    work, arc_s, true /* forward */, accel_,
    blocked_start ? 0.0 : std::max(v_seed, 0.0),
    blocked_start ? 0.0 : a_seed,
    ConstraintSource::kKinematicAccel);
}

void ReachabilitySpeedPlanner::enforceDecelFeasibility(
  std::vector<TrajPointWork> & work, const std::vector<double> & arc_s,
  double /*a_seed*/) const
{
  // 向下可达：包络可能要求一个物理上做不到的减速（初速高于起点包络、
  // 限速骤降、障碍突现）。此处把剖面**抬高**到「物理上也只能减到这么慢」，
  // 使输出始终可执行；代价是这些点会暂时高于包络——那是如实反映不可行，
  // 而不是假装能做到。正常情况下 reachable_down 低于当前值，本遍为空操作。
  //
  // ⚠ 本遍**不含 jerk 限制**，是经过实测后的刻意选择。
  //
  // 试过两版：(a) 从剖面反算 a 再往负向爬——把环闭错了，终端坡道本已在 −d_max
  // 却被判"尚未建立"从而**抬高**，实测 `straight` 末速 0.4732（应 0.05）、
  // `hardstop` 0.4880（应 0）、`fast_near_obs` 开进障碍 v=0.3265；
  // (b) 从初始状态**开环**积分、只作用于不可行前缀——语义正确，但把制动的
  // jerk 建立算进去后物理上就真的刹不住了：`fast_near_obs` 仍开进障碍
  // （0.6→0 按恒定 −0.4 需 0.45 m < 障碍距 0.5 m 恰好够，加上 a 从 0 建立到
  // −0.4 的 0.8 s 就不够了），`latched_0.60m` 末速 0.0500 → 0.1632。
  //
  // (b) 更**诚实**——恒定 −d_max 宣称了机器人没有的制动能力。但它改变的是
  // 安全相关行为（障碍带内是否零速、末速是否达标），代价是 jerk 只从 32 项
  // 降到 30 项。这个取舍应由人显式裁决，不该由一次重构顺手改掉：
  // 若采纳 (b)，障碍安全裕度必须从别处补（更早检测 / 更大缓冲带）。
  // 详见 decisions.md 与 R47。
  const double d_max = std::fabs(decel_);
  if (d_max < 1e-6) {
    return;
  }
  for (size_t i = 0; i + 1 < work.size(); ++i) {
    const double ds = arc_s[i + 1] - arc_s[i];
    if (ds <= kArcEpsilon) {
      continue;
    }
    const double v_here = std::max(work[i].velocity_upper_bound, 0.0);
    const double sq = v_here * v_here - 2.0 * d_max * ds;
    const double reachable_down = sq > 0.0 ? std::sqrt(sq) : 0.0;
    if (work[i + 1].velocity_upper_bound < reachable_down) {
      work[i + 1].velocity_upper_bound = reachable_down;
    }
  }
}


void ReachabilitySpeedPlanner::fillProfileDerivatives(
  std::vector<TrajPointWork> & work, const std::vector<double> & arc_s) const
{
  double t = 0.0;
  for (size_t i = 0; i + 1 < work.size(); ++i) {
    const double ds = arc_s[i + 1] - arc_s[i];
    const double v0 = std::max(work[i].velocity_upper_bound, 0.0);
    const double v1 = std::max(work[i + 1].velocity_upper_bound, 0.0);
    work[i].rel_time = t;
    if (ds > 1e-6) {
      work[i].accel = (v1 * v1 - v0 * v0) / (2.0 * ds);
      t += segmentDuration(v0, v1, ds);
    } else {
      work[i].accel = 0.0;
    }
  }
  work.back().rel_time = t;
  work.back().accel = work[work.size() - 2].accel;
  fillJerkFromAccel(work);
}

}  // namespace nav2_speed_planner

PLUGINLIB_EXPORT_CLASS(
  nav2_speed_planner::ReachabilitySpeedPlanner, nav2_core::SpeedPlanner)
