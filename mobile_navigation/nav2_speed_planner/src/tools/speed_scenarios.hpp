// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 离线速度剖面验证——**场景定义与场景表**（纯数据，不含执行与校验逻辑）。
//
// 分类维度有两个，互相正交：
//   Category  这个场景在测什么（距离族 / 弯道 / 障碍 / 交叉 / 退化输入 / 失败策略…）
//   Expect    这个场景**应该**得到什么（成功 / 必须空计划 / 全程零速 / 已知降级）
//
// Expect 是必需的一维：早期版本把"非空轨迹"当成无条件断言，结果
//   ① 步 1 引入的 FAIL_CLOSED 语义（不可评估时必须返回空计划）无法被表达，
//      于是一个离线场景都没覆盖；
//   ② "应该失败却静默成功"的已知缺陷只能靠一个 bool 标记，说不清期望是什么。

#ifndef TOOLS__SPEED_SCENARIOS_HPP_
#define TOOLS__SPEED_SCENARIOS_HPP_

#include <cstdio>
#include <string>
#include <vector>

#include "nav2_speed_planner/speed_planner_base.hpp"

namespace speed_offline
{

// ── 共享参数（与历史 CSV 一致，勿改）──
constexpr double kMaxSpeed = 1.0;
constexpr double kMinSpeed = 0.03;
constexpr double kAccel = 0.6;
constexpr double kDecel = -0.6;
constexpr double kEndSpeed = 0.03;
constexpr double kStopDistance = 0.2;
constexpr double kPointStep = 0.02;
constexpr double kMaxJerk = 0.5;
constexpr double kControlDt = 0.05;

constexpr double kCurveStart = 2.0;
constexpr double kCurveLen = 0.5;
constexpr double kCurveRadius = 0.5;

enum class Backend { Trapezoidal, SCurve, Reachability };

/// 一整套运动学参数。默认值是历史离线常量（保回归基线逐字节）；
/// `production()` 逐字对齐 `w1_nav2_params_diff.yaml` 的 SCurveSpeedPlanner 段。
///
/// 为什么必须有这一层：离线常量与生产**每一项都不同**（max_speed 1.0 vs 0.6、
/// decel -0.6 vs -0.4、stop_distance 0.2 vs 0.075…）。用离线常量做的对拍曾
/// 6 次测不出锁存锚点修复的差异——因为 decel 0.6 让 continuity_braking_distance
/// 只有 ~0.31 m，始终小于 (剩余 − 裕度)，min() 在新旧两版取同一个锚点。
/// **验一套机器人上不存在的配置，等于没验。**
struct Params
{
  double max_speed{kMaxSpeed};
  double min_speed{kMinSpeed};
  double accel{kAccel};
  double decel{kDecel};
  double end_speed{kEndSpeed};
  double stop_distance{kStopDistance};
  double collision_stop_distance{0.2};
  double max_jerk{kMaxJerk};
  double control_dt{kControlDt};
  double min_curvature_radius{0.0};   // 0 = 不覆写（沿用实现默认）
  double min_curvature_speed{0.0};
  /// A3 的物理基准 [m/s²]。0.3 与 min_radius 1.2 / max_speed 0.6 自洽
  /// （0.6²/1.2 = 0.30），限速在激活门处连续。
  double max_lateral_accel{0.3};

  static Params production()
  {
    Params p;
    p.max_speed = 0.6;
    p.min_speed = 0.05;
    p.accel = 0.5;
    p.decel = -0.4;
    p.end_speed = 0.05;
    p.stop_distance = 0.075;
    p.collision_stop_distance = 0.2;
    p.max_jerk = 0.5;
    p.control_dt = 0.05;
    // ⚠ **此处与部署 yaml 有一处已知分歧，是刻意的。**
    //
    // 部署中曲率限速被临时关闭（`regulated_linear_scaling_min_radius: 0.00001`，
    // 使判据恒不触发），以绕开「公式以巡航速度为基准」导致的整路径误判。
    // 但离线的曲率族场景（起点谷 / 曲率井 / 弯道平台）**存在的目的就是暴露
    // 该公式的缺陷**——若此处同步为 0.00001，这些场景会全部退化为空跑，
    // 反而失去验证能力。
    //
    // 故此处保持 1.2 = **曲率公式改为 v = √(a_lat_max/|κ|) 之后的目标配置**。
    // 曲率恢复上线时，本注释与部署 yaml 中的对应注释应一并删除。
    p.min_curvature_radius = 1.2;
    p.min_curvature_speed = 0.1;
    p.max_lateral_accel = 0.3;
    return p;
  }
};

enum class Category
{
  Distance,     // 直线距离族
  Moving,       // 运动中起步
  Curve,        // 弯道
  Obstacle,     // 障碍
  Terminal,     // 终端 / 硬停
  Crossing,     // 多约束交叉
  Degenerate,   // 退化输入（空路径 / 单点 / 重合点 / 超速进入…）
  FailPolicy,   // 约束不可评估时的失败策略（FAIL_CLOSED / FAIL_OPEN）
  Malformed,    // 非法输入（NaN / Inf / 非法四元数 / 瞬移 / 倒行）
  Latched,      // 末端制动已锁存（裕度是否贯穿整个制动过程）
  Production,   // 生产参数（逐字对齐部署 yaml）下重跑关键场景
  Zone,         // 逐点限速区
};

enum class Expect
{
  Success,     // 非空，且全部不变量成立
  EmptyPlan,   // **必须**返回空轨迹（fail-closed，或输入本就无法规划）
  ZeroSpeed,   // 非空，但全程线速度为零（点数不足 / 纯原地转）
  Degraded,    // 非空，但已知违反部分不变量——失败记为 XFAIL，须写明 reason
};

// 路径形态。Generated 走 makePath；其余是刻意构造的退化输入。
enum class PathKind
{
  Generated,
  Empty,           // 0 点
  SinglePoint,     // 1 点
  TwoPoints,       // 2 点
  AllCoincident,   // N 点全部重合（纯原地）
  WithDuplicates,  // 正常路径中插入重复点
  NaNPose,         // 中段插入 NaN 坐标
  InfPose,         // 中段插入 Inf 坐标
  ZeroQuaternion,  // 四元数全零（非法姿态）
  Backward,        // 路径朝 -x 延伸
  Teleport,        // 相邻两点间瞬移 10m
  OffRouteStart,   // 首点（机器人状态戳）横向偏离路网——跟踪误差的离线形态
  NonUniform,      // 点距不均（实机全局路径形态：2.56~70.5mm，27 倍跨度）
};

// 限速区：路径 x∈[from,to] 段的逐点限速。经 pointSpeedHint 接缝供给，
// 这是 M11 区域限速在今天唯一能被离线验证的形态（生产端 P0-2 才补上）。
struct Zone
{
  double from;
  double to;
  double speed;
};

struct Arc
{
  double start;    // 起始弧长 [m]
  double len;      // 弧段长度 [m]
  double radius;   // 曲率半径 [m]（正=左转）
};

struct Band
{
  double from;     // 起始 x 坐标 [m]
  double to;       // 结束 x 坐标 [m]
};

struct Scenario
{
  std::string tag;
  Backend backend{Backend::Trapezoidal};
  Category category{Category::Distance};
  Expect expect{Expect::Success};

  double length{1.0};
  double v0{0.0};
  std::vector<Arc> arcs{};
  std::vector<Band> obstacles{};   // 单位：路径 x 坐标
  std::vector<Zone> zones{};       // 逐点限速区（单位：路径 x 坐标）
  double end_speed{kEndSpeed};     // 覆写 params.end_speed（仅硬停场景用）
  bool override_end_speed{false};
  Params params{};                 // 默认离线常量；生产场景显式设 production()
  double external_limit{0.0};      // 0 = 不设

  // 末端制动锁存态。离线此前从不置位 → latch 分支零覆盖，
  // 而制动裕度的存亡恰恰全发生在该分支里。
  bool terminal_brake_latched{false};
  double terminal_execution_delay{0.0};   // 0 = 不启用裕度
  double terminal_safety_margin{0.0};

  PathKind path_kind{PathKind::Generated};
  double lateral_offset{0.0};      // OffRouteStart：首点横向偏移量 [m]
  double dense_spacing{0.0};       // NonUniform：密集点簇的点距 [m]
  bool obstacle_available{true};   // false → 触发不可评估分支
  nav2_speed_planner::ConstraintFailurePolicy unavailable_policy{
    nav2_speed_planner::ConstraintFailurePolicy::FAIL_CLOSED};

  // 形状无关的**下界**断言：路径足够长且无持续约束时，计划必须爬到该速度。
  // 此前整套校验只有上界（≤ 限值）和不变量，全都被"处处 0.05 也合法"绕过——
  // C1 逐点限速被压平、R48 全程蠕行都是这样瞒过 800+ 项检查的。
  double expect_peak_speed{0.0};     // 0 = 不检查
  std::string peak_speed_xfail{};    // 非空 = 已知缺陷，记 XFAIL 并附此理由

  std::string degraded_reason{};   // Expect::Degraded 必填
  std::string note{};
};

inline const char * categoryName(Category c)
{
  switch (c) {
    case Category::Distance: return "距离族";
    case Category::Moving: return "运动中起步";
    case Category::Curve: return "弯道";
    case Category::Obstacle: return "障碍";
    case Category::Terminal: return "终端";
    case Category::Crossing: return "交叉约束";
    case Category::Degenerate: return "退化输入";
    case Category::FailPolicy: return "失败策略";
    case Category::Malformed: return "非法输入";
    case Category::Latched: return "锁存制动";
    case Category::Production: return "生产参数";
    case Category::Zone: return "限速区";
  }
  return "?";
}

// ───────────────────────── 场景表 ─────────────────────────
//
// ── 场景命名规则 ────────────────────────────────────────────────────
//
//   <场景描述>[_<参数>]__<后端>
//
// 后端后缀（放末尾，使同一场景的不同后端在文件列表里相邻，便于对拍）：
//   __trapz        TrapezoidalSpeedPlanner
//   __scurve       SCurveSpeedPlanner（离线常量参数）
//   __scurve_prod  SCurveSpeedPlanner（生产参数，逐字对齐部署 yaml）
//   __reach        ReachabilitySpeedPlanner
//
// 场景描述用完整词，不用单字母。此前用 T/S/X/D/E/F/L/N/O/P/R/W/Z 十三种
// 单字母前缀，且 `S` 同时表示「SCurve 后端」与「startwell 起点谷」——
// 两个维度挤在同一位、含义还冲突，读 tag 无法判断测的是什么。
//
// CSV 文件名为 <tag>_<路径长度>.csv，故 tag 变更会改变文件名；
// 回归基线需同步重新快照（内容不变可由改名前的哈希逐一校验）。

inline std::vector<Scenario> buildScenarios()
{
  using P = nav2_speed_planner::ConstraintFailurePolicy;
  std::vector<Scenario> v;
  const std::vector<Arc> curve2{{kCurveStart, kCurveLen, kCurveRadius}};

  auto add = [&v](Scenario s) {v.push_back(std::move(s));};

  // ── 距离族 ──
  for (double len : {0.3, 0.5, 1.0, 2.5, 5.0}) {
    Scenario t; t.tag = "straight__trapz"; t.category = Category::Distance; t.length = len;
    add(t);
    Scenario s = t; s.tag = "straight__scurve"; s.backend = Backend::SCurve;
    add(s);
  }

  // ── 运动中起步 ──
  for (double v0 : {0.3, 0.6, 1.0}) {
    char nt[40];
    Scenario t; t.category = Category::Moving; t.length = 2.5; t.v0 = v0;
    std::snprintf(nt, sizeof(nt), "straight_moving_v%.1f__trapz", v0); t.tag = nt;
    add(t);
    Scenario s = t; s.backend = Backend::SCurve;
    std::snprintf(nt, sizeof(nt), "straight_moving_v%.1f__scurve", v0); s.tag = nt;
    add(s);
  }

  // ── 弯道 ──
  {
    Scenario t; t.tag = "curve__trapz"; t.category = Category::Curve;
    t.length = 5.0; t.arcs = curve2;
    add(t);
    Scenario s = t; s.tag = "curve__scurve"; s.backend = Backend::SCurve;
    add(s);
  }

  // ── 障碍 ──
  {
    Scenario t; t.tag = "obs_at_start__trapz"; t.category = Category::Obstacle;
    t.length = 1.0; t.obstacles = {{0.0, 0.10}};
    t.note = "起点在障碍内切区（豁免起步）";
    add(t);
  }
  {
    Scenario t; t.tag = "obs_mid__trapz"; t.category = Category::Obstacle;
    t.length = 1.0; t.obstacles = {{0.7, 1.0}};
    add(t);
  }
  {
    Scenario t; t.tag = "obs_far__trapz"; t.category = Category::Obstacle;
    t.length = 5.0; t.obstacles = {{3.0, 5.0}};
    add(t);
    Scenario s = t; s.tag = "obs_far__scurve"; s.backend = Backend::SCurve;
    add(s);
  }

  // ── 终端 ──
  {
    Scenario t; t.tag = "hardstop__trapz"; t.category = Category::Terminal;
    t.length = 1.0; t.end_speed = 0.0; t.override_end_speed = true;
    add(t);
    Scenario s = t; s.tag = "hardstop__scurve"; s.backend = Backend::SCurve;
    add(s);
  }

  // ══════════ 交叉约束 ══════════
  {
    Scenario t; t.tag = "obs_after_curve__trapz"; t.category = Category::Crossing;
    t.length = 5.0; t.arcs = curve2; t.obstacles = {{3.0, 5.0}};
    t.note = "障碍紧随弯道之后（带按 x，弯后 x<弧长）";
    add(t);
    Scenario s = t; s.tag = "obs_after_curve__scurve"; s.backend = Backend::SCurve;
    add(s);
  }
  {
    Scenario t; t.tag = "obs_near_goal__trapz"; t.category = Category::Crossing;
    t.length = 2.0; t.obstacles = {{1.8, 2.0}};
    t.note = "障碍与终端区重叠，A1 零速应压过 A4 end_speed";
    add(t);
  }
  {
    Scenario t; t.tag = "curve_near_goal__trapz"; t.category = Category::Crossing;
    t.length = 2.5; t.arcs = {{2.0, 0.4, kCurveRadius}};
    t.note = "弯道落在终端区";
    add(t);
  }
  // 制动物理不可达：两个后端都静默输出违反约束的计划——已知缺陷，标 Degraded。
  {
    Scenario t; t.tag = "fast_near_obs__trapz"; t.category = Category::Crossing;
    t.expect = Expect::Degraded; t.length = 5.0; t.v0 = 1.0;
    t.obstacles = {{0.5, 5.0}};
    t.degraded_reason = "制动不可达时输出 |a|≫限值 的阶跃计划，无不可行标志";
    t.note = "v0=1.0 距障碍 0.5m（制动距离需 0.83m）";
    add(t);
    Scenario s = t; s.tag = "fast_near_obs__scurve"; s.backend = Backend::SCurve;
    s.degraded_reason = "jerk 限制下根本刹不住，直接驶入障碍带且无不可行标志";
    add(s);
  }
  {
    Scenario t; t.tag = "fast_near_curve__trapz"; t.category = Category::Crossing;
    t.expect = Expect::Degraded; t.length = 5.0; t.v0 = 1.0;
    t.arcs = {{0.3, 0.5, kCurveRadius}};
    t.degraded_reason = "同上，曲率版本";
    t.note = "v0=1.0 距弯道 0.3m";
    add(t);
  }
  for (double lim : {0.3, 0.6, 1.0}) {
    char nt[48];
    std::snprintf(nt, sizeof(nt), "X_T_limit%.1f_curve", lim);
    Scenario t; t.tag = nt; t.category = Category::Crossing;
    t.length = 5.0; t.arcs = curve2; t.external_limit = lim;
    t.note = "外部限速扫描，用于暴露曲率公式对巡航的依赖";
    add(t);
  }
  {
    Scenario t; t.tag = "two_obstacles__trapz"; t.category = Category::Crossing;
    t.length = 5.0; t.obstacles = {{1.5, 1.7}, {3.5, 5.0}};
    add(t);
  }
  {
    Scenario t; t.tag = "dense_all__trapz"; t.category = Category::Crossing;
    t.length = 1.0; t.arcs = {{0.3, 0.2, 0.4}}; t.obstacles = {{0.8, 1.0}};
    t.end_speed = 0.0; t.override_end_speed = true; t.note = "1m 内弯道+障碍+硬停";
    add(t);
  }
  {
    Scenario t; t.tag = "obs_hardstop__trapz"; t.category = Category::Crossing;
    t.length = 2.0; t.obstacles = {{1.5, 2.0}}; t.end_speed = 0.0; t.override_end_speed = true;
    add(t);
  }

  // ══════════ 退化输入 ══════════
  // 这些不测"规划得好不好"，只测"不崩、且给出可解释的退化结果"。
  {
    Scenario t; t.tag = "degenerate_empty_path__trapz"; t.category = Category::Degenerate;
    t.expect = Expect::EmptyPlan; t.path_kind = PathKind::Empty;
    t.note = "0 点路径";
    add(t);
  }
  {
    Scenario t; t.tag = "degenerate_single_point__trapz"; t.category = Category::Degenerate;
    t.expect = Expect::ZeroSpeed; t.path_kind = PathKind::SinglePoint;
    t.note = "1 点路径（点数不足，应保 pose 序列并禁线速度）";
    add(t);
  }
  {
    // 2 点足够规划：路径仅 0.02m，整段落在 stop_distance 内 → 爬行速度。
    // （最初写成 Expect::ZeroSpeed 是我判断错，不是规划器的问题。）
    Scenario t; t.tag = "degenerate_two_points__trapz"; t.category = Category::Degenerate;
    t.path_kind = PathKind::TwoPoints;
    t.note = "2 点、间距一个采样步（整段在 stop_distance 内）";
    add(t);
  }
  {
    Scenario t; t.tag = "degenerate_all_coincident__trapz"; t.category = Category::Degenerate;
    t.expect = Expect::ZeroSpeed; t.path_kind = PathKind::AllCoincident;
    t.note = "N 点全部重合（纯原地转向形态）";
    add(t);
  }
  {
    Scenario t; t.tag = "degenerate_with_duplicates__trapz"; t.category = Category::Degenerate;
    t.path_kind = PathKind::WithDuplicates; t.length = 1.0;
    t.note = "正常路径中插入重复点（去重逻辑）";
    add(t);
  }
  {
    Scenario t; t.tag = "degenerate_v0_over_max__trapz"; t.category = Category::Degenerate;
    t.expect = Expect::Degraded; t.length = 2.5; t.v0 = 1.5;
    t.degraded_reason =
      "初始状态超出包络时，计划要求瞬间掉回（|a|≫限值），"
      "而非以 decel_ 平滑收敛——与 X_*_fast_near_obs 同一缺陷";
    t.note = "以超过 max_speed 的速度进入（上游超调）";
    add(t);
  }
  {
    Scenario t; t.tag = "degenerate_all_obstacle__trapz"; t.category = Category::Degenerate;
    t.length = 2.0; t.obstacles = {{0.0, 2.0}};
    t.note = "整条路径都在障碍里（仅起点豁免）";
    add(t);
  }

  // 上面那条的**真实触发形态**：机器人以 1.0 巡航时进入 0.3 的限速区。
  // 不需要障碍突然出现，区域限速（M11）一变就会命中同一条不可行路径。
  {
    Scenario t; t.tag = "limit_drop_while_moving__trapz";
    t.category = Category::Crossing; t.expect = Expect::Degraded;
    t.length = 5.0; t.v0 = 1.0; t.external_limit = 0.3;
    t.degraded_reason = "运动中限速下调，同样要求瞬间掉速而非按 decel_ 收敛";
    t.note = "v0=1.0 进入限速 0.3 区（区域限速的常规形态）";
    add(t);
    Scenario s = t; s.tag = "limit_drop_while_moving__scurve";
    s.backend = Backend::SCurve;
    add(s);
  }

  // ══════════ 限速区（经 pointSpeedHint 接缝）══════════
  // 真正的区域限速形态：正常 → 慢区 → 正常。测的是**进区前是否提前减速**、
  // **出区后是否重新加速**——这两件事单一全局 external_limit 表达不了。
  {
    Scenario t; t.tag = "speed_zone__trapz"; t.category = Category::Zone;
    t.length = 5.0; t.zones = {{1.5, 3.0, 0.3}};
    t.note = "1.5~3.0m 限速 0.3，两侧正常（**实测全路径被压到 0.3**）";
    add(t);
    Scenario s = t; s.tag = "speed_zone__scurve"; s.backend = Backend::SCurve;
    add(s);
  }
  {
    Scenario t; t.tag = "zone_entry_fast__trapz"; t.category = Category::Zone;
    t.expect = Expect::Degraded;
    t.degraded_reason = "进区来不及减速时要求瞬间掉速（同 bug-2571）";
    t.length = 5.0; t.v0 = 1.0; t.zones = {{0.5, 3.0, 0.3}};
    t.note = "v0=1.0，慢区 0.5m 处就开始（进区来不及减速）";
    add(t);
  }
  {
    Scenario t; t.tag = "zone_with_obstacle__trapz"; t.category = Category::Zone;
    t.length = 5.0; t.zones = {{1.5, 3.0, 0.3}}; t.obstacles = {{3.5, 5.0}};
    t.note = "限速区 + 区外障碍（逐点限速与 A1 叠加）";
    add(t);
  }
  {
    Scenario t; t.tag = "two_zones__trapz"; t.category = Category::Zone;
    t.length = 5.0; t.zones = {{1.0, 1.8, 0.5}, {3.0, 4.0, 0.2}};
    t.note = "两段不同限速区";
    add(t);
  }

  // ══════════ 非法输入 ══════════
  // 期望先留 Success（"不崩且给出可解释结果"），跑完按实际表现分诊——
  // 事先假定某个具体行为正确，等于把当前实现当成规范。
  {
    Scenario t; t.tag = "malformed_nan_pose__trapz"; t.category = Category::Malformed;
    t.expect = Expect::Degraded;
    t.degraded_reason =
      "单个 NaN 位姿 → 整条计划塌成 0.03 爬行、s 列全 NaN，且无任何告警";
    t.length = 2.0; t.path_kind = PathKind::NaNPose;
    t.note = "中段坐标为 NaN";
    add(t);
  }
  {
    Scenario t; t.tag = "malformed_inf_pose__trapz"; t.category = Category::Malformed;
    t.expect = Expect::Degraded;
    t.degraded_reason = "同 NaN：Inf 位姿被静默吞掉";
    t.length = 2.0; t.path_kind = PathKind::InfPose;
    t.note = "中段坐标为 Inf";
    add(t);
  }
  {
    Scenario t; t.tag = "malformed_zero_quaternion__trapz"; t.category = Category::Malformed;
    t.length = 2.0; t.path_kind = PathKind::ZeroQuaternion;
    t.note = "四元数全零（非法姿态）";
    add(t);
  }
  {
    Scenario t; t.tag = "malformed_backward_path__trapz"; t.category = Category::Malformed;
    t.length = 2.0; t.path_kind = PathKind::Backward;
    t.note = "路径朝 -x 延伸（倒行 / 上游方向错）";
    add(t);
  }
  {
    Scenario t; t.tag = "malformed_teleport__trapz"; t.category = Category::Malformed;
    t.length = 2.0; t.path_kind = PathKind::Teleport;
    t.note = "相邻两点瞬移 10m（丢帧 / 拼接错）";
    add(t);
  }
  {
    Scenario t; t.tag = "malformed_negative_v0__trapz"; t.category = Category::Malformed;
    t.expect = Expect::Degraded;
    t.degraded_reason =
      "v0 的符号被忽略、只用幅值：计划首点 -0.5 直接跳到 +0.52，"
      "完全没建模倒车穿越零点所需的 0.21m / 0.83s";
    t.length = 2.5; t.v0 = -0.5;
    t.note = "进入速度为负（倒车中收到前进计划）";
    add(t);
  }
  {
    Scenario t; t.tag = "malformed_end_speed_over_max__trapz"; t.category = Category::Malformed;
    t.length = 2.0; t.end_speed = 2.0; t.override_end_speed = true;
    t.note = "end_speed(2.0) > max_speed(1.0)（配置自相矛盾）";
    add(t);
  }

  // ══════════ 锁存制动（裕度必须贯穿整个制动过程）══════════
  // 历史实现锁存后取 min(剩余距离, 可达制动距离)，等于丢弃裕度 → 零裕度制动。
  // 锚点只在**窄带**内体现差异：可达制动距离介于 (剩余−裕度) 与 剩余 之间。
  // 距离太短 → 怎么刹都刹不住，两版同为尽力而为；太长 → min 取可达制动距离，
  // 两版同值。故扫多个长度覆盖该带。
  for (double len : {0.9, 1.0, 1.1, 1.3}) {
    for (int latched = 0; latched < 2; ++latched) {
      char nt[64];
      std::snprintf(
        nt, sizeof(nt), "L_S_%s", latched ? "latched" : "unlatched");
      Scenario t;
      t.tag = nt;
      t.category = Category::Latched;
      t.backend = Backend::SCurve;
      t.length = len;
      t.v0 = 0.6;
      t.terminal_brake_latched = (latched != 0);
      t.terminal_execution_delay = 0.35;   // 与生产一致
      t.terminal_safety_margin = 0.05;
      t.note = latched ? "已锁存：裕度须贯穿制动全程" : "未锁存对照";
      add(t);
    }
  }

  // ══════════ 生产参数（逐字对齐部署 yaml 的 SCurveSpeedPlanner 段）══════════
  // max_speed 0.6 / decel -0.4 / end_speed 0.05 / stop_distance 0.075 /
  // min_radius 1.2 / min_curvature_speed 0.1 / jerk 0.5 / delay 0.35 + margin 0.05
  {
    const Params prod = Params::production();
    const std::vector<Arc> curve_prod{{2.0, 0.5, kCurveRadius}};

    for (double len : {1.0, 2.5, 5.0}) {
      Scenario t; t.tag = "straight__scurve_prod"; t.category = Category::Production;
      t.backend = Backend::SCurve; t.params = prod; t.length = len;
      add(t);
    }
    {
      Scenario t; t.tag = "curve__scurve_prod"; t.category = Category::Production;
      t.backend = Backend::SCurve; t.params = prod; t.length = 5.0;
      t.arcs = curve_prod; t.note = "生产曲率参数（min_radius 1.2）";
      add(t);
    }
    {
      Scenario t; t.tag = "obs_far__scurve_prod"; t.category = Category::Production;
      t.backend = Backend::SCurve; t.params = prod; t.length = 5.0;
      t.obstacles = {{3.0, 5.0}};
      add(t);
    }
    {
      Scenario t; t.tag = "fast_near_obs__scurve_prod"; t.category = Category::Production;
      t.expect = Expect::Degraded; t.backend = Backend::SCurve; t.params = prod;
      t.length = 5.0; t.v0 = 0.6; t.obstacles = {{0.5, 5.0}};
      t.degraded_reason = "生产参数下的制动不可达（离线参数版为 X_S_fast_near_obs）";
      add(t);
    }
    {
      Scenario t; t.tag = "limit_drop__scurve_prod"; t.category = Category::Production;
      t.expect = Expect::Degraded; t.backend = Backend::SCurve; t.params = prod;
      t.length = 5.0; t.v0 = 0.6; t.external_limit = 0.2;
      t.degraded_reason = "生产参数下运动中限速下调";
      add(t);
    }
    // ⭐ 锁存锚点的差异窄带：生产 decel -0.4 使可达制动距离 ~0.5-0.6 m，
    //    裕度 0.6×0.35+0.05 = 0.26 m → 窄带约在剩余距离 0.55~0.85 m。
    for (double len : {0.6, 0.7, 0.8, 1.0}) {
      for (int latched = 0; latched < 2; ++latched) {
        Scenario t;
        t.tag = latched ? "latched__scurve_prod" : "unlatched__scurve_prod";
        t.category = Category::Production;
        t.backend = Backend::SCurve; t.params = prod;
        t.length = len; t.v0 = 0.6;
        t.terminal_brake_latched = (latched != 0);
        t.terminal_execution_delay = 0.35;
        t.terminal_safety_margin = 0.05;
        // 生产参数下 0.6 m/s 的制动距离实测 0.665 m（理论 0.676 m，
        // 由 jerk 0.5 + decel 0.4 必然导出）。短于它的路径**物理上停不住**，
        // 末点速度必然高于 end_speed——这不是缺陷，是场景在要求做不到的事。
        // 仍保留这两个长度：它们是 C2「不可行却无标志」的直接证据。
        if (len < 0.72) {
          t.expect = Expect::Degraded;
          t.degraded_reason =
            "路径短于生产参数下的制动距离 0.665 m，物理不可停；"
            "计划照常输出且不置不可行标志（C2）";
        }
        t.note = latched ? "已锁存：裕度须贯穿制动全程" : "未锁存对照";
        add(t);
      }
    }
  }

  // ══════════ 曲率井（R48 实机形态）══════════
  // 实机根因：曲率约束在生产参数下把点压到 min_curvature_speed=0.1，
  // lead_in/out 把井加宽但井壁仍是阶跃；相邻谷底之间（0.1 → 0.4148，
  // 间距 0.3375 m）在 jerk 限制下需要 0.41 m，Ruckig 返回 -110 →
  // 掉进制动兜底 → 输出恒 end_speed → 全程蠕行 53 s 走 2.4 m。
  // 用多个短弧段（r < min_curvature_radius 1.2）间隔短直线复现该形态。
  {
    const Params prod = Params::production();
    for (double gap : {0.30, 0.45, 0.60}) {
      char nt[56];
      std::snprintf(nt, sizeof(nt), "curve_wells_gap%.2f__scurve", gap);
      Scenario t; t.tag = nt; t.category = Category::Production;
      t.backend = Backend::SCurve; t.params = prod; t.length = 3.0;
      // 三个 r=0.6 的短弧，间隔 gap 的直线 → 三个曲率井
      t.arcs = {
        {0.5, 0.25, 0.6},
        {0.5 + 0.25 + gap, 0.25, 0.6},
        {0.5 + 0.5 + 2 * gap, 0.25, 0.6}};
      t.note = "曲率井间距 " + std::string(nt).substr(19) + "，井壁阶跃是否可达";
      add(t);
    }
  }

  // ══════════ 偏离路网起点（R48 真根因的离线形态）══════════
  // 机器人与路网之间永远有跟踪误差：work[0]（机器人实际位姿）到 work[1]
  // （路网投影）之间是一个横向折角。若把 work[0] 纳入三点圆拟合，会算出很小的
  // 半径 → 曲率约束在起点持续触发 → 速度压到 min_curvature_speed（生产 0.1）
  // → 包络阶跃 → Ruckig 目标互不可达 → 兜底 → 输出恒 end_speed → 全程蠕行。
  // 实机日志里反复出现的 `solver v0=0.1000` 就是这个 0.1。
  {
    const Params prod = Params::production();
    for (double lat : {0.05, 0.10, 0.15}) {
      char nt[56];
      std::snprintf(nt, sizeof(nt), "offroute_lat%.2f__scurve", lat);
      Scenario t; t.tag = nt; t.category = Category::Production;
      t.backend = Backend::SCurve; t.params = prod; t.length = 3.0;
      t.path_kind = PathKind::OffRouteStart;
      t.lateral_offset = lat;
      // ⚠ 必须带初速：实机是**运动中**持续偏离（跟踪误差），静止起步时
      //   起点低速本来就自然，掩盖了折角造成的曲率坑。
      t.v0 = 0.4;
      t.note = "运动中偏离路网，曲率拟合是否被折角污染";
      add(t);
    }
  }

  // ══════════ 起点即曲率谷（R48 实机日志直接反推的形态）══════════
  // 2026-08-08 实机日志（003/8/nav.log）：任务**起点静止**（v0=0.000）、
  // 5.763 m / 110 点的路径，Ruckig 在爬升段直接失败：
  //   solver: code=-110 v0=0.1000 a0=0.0000 vmax=0.5824 target_v=0.5713 dist=0.6512
  // `v0=0.1000` 精确等于生产 min_curvature_speed=0.1 → 包络在**起点**就被
  // 曲率约束压到 floor；`vmax=0.5824`/`target_v=0.5713` 反推半径 1.165/1.143 m
  // （略小于 min_radius 1.2 的平缓段）。于是第一批降序目标之一是
  // 「从 0.1 爬到 0.5713，0.65 m 内，vmax 仅 0.5824」——目标只比上限低 1.9%，
  // 几乎没有超调再回落的余量，求解器解不出。
  //
  // 关键差别（此前零覆盖的原因）：既有 W_S_curve_wells / N_S_nonuniform 的井
  // **都在中段**，失败时段级兜底还能保住一段像样的前缀；井在起点时前缀最高
  // 只到 0.1，保住了也是蠕行。用户在实机把 min_radius 降到 1e-5（等价于整个
  // 关闭曲率约束）后问题消失，是对该因果链的消融验证。
  //
  // 半径→曲率限速：v = max_speed × (r / min_radius) = 0.6 × r / 1.2，
  // 再被 min_curvature_speed=0.1 兜底。r=0.2 → 0.1（触底）/ 0.3 → 0.15 / 0.5 → 0.25。
  //
  // ⚠ 第一版只放「起点深井 + 其后纯直线」**没能复现**：井确实形成
  //   （kappa 4.998、速度压到 0.10004、region=curve 23 点），但井后有 4.5 m
  //   直线余量，爬升轻松可解，全部到达 0.6。差别正是 `dist=0.6512`——
  //   实机井后 **0.65 m 就是下一个降序目标**，且 `vmax` 只比目标高 1.9%。
  //   故本组必须是「深井 + 紧随其后的平缓弯」两段几何。
  {
    const Params prod = Params::production();
    // 平缓弯半径由实机反推：0.6 × r / 1.2 = 0.5713 → r = 1.1426。
    constexpr double kGentleR = 1.1426;
    // gap = 深井结束到平缓弯起点的直线长度。降序目标间距 ≈ 0.1 + gap，
    // 扫过实机的 0.6512 两侧。
    for (double gap : {0.30, 0.55, 0.80}) {
      char nt[56];
      std::snprintf(nt, sizeof(nt), "startwell_gap%.2f__scurve", gap);
      Scenario t; t.tag = nt; t.category = Category::Production;
      t.backend = Backend::SCurve; t.params = prod; t.length = 5.0;
      t.arcs = {
        {0.0, 0.3, 0.20},                 // 起点深井：0.6×0.2/1.2 = 0.1 → 触 floor
        {0.3 + gap, 2.0, kGentleR}};      // 平缓弯：限速 ≈ 0.5713，压低 vmax
      t.v0 = 0.0;                         // 与实机一致：静止起步
      // 5 m 路径、井外无持续约束 → 本应爬到平缓弯限速 0.5713 附近。
      t.expect_peak_speed = 0.5;
      if (gap > 0.4) {
        t.peak_speed_xfail =
          "R48：起点谷 + 0.65 m 外的爬升目标 → Ruckig -110 → 兜底刹车，"
          "而失败在首段附近使段级兜底无前缀可保，整条计划塌成 0.205";
      }
      t.note = "起点深井 + 紧随平缓弯（实机形态），爬升目标是否可解";
      add(t);
    }
    // ⭐ 初值超包络**恰在 index 0**——步 4a 的复现场景。
    //
    // 起点 r=0.2 深井把包络压到 min_curvature_speed=0.1，而机器人正以 v0 > 0.1
    // 行驶。SCurve 的 `v_out[0] = clamp(v_start, 0, v_env[0])` 会把首点直接钳到
    // 0.1 —— **要求机器人瞬间从 v0 掉到 0.1**，物理不可能且不置任何不可行标志。
    //
    // 梯形后端有 `stampStartVelocity`（`work[0].velocity_upper_bound = curr_vel`）把这个
    // 现象**挡住了**——它是 C2 在 index 0 的挡箭牌。故 T/S 两版互为对照：
    // 同一场景 T 版首点应为 v0，S 版首点应被钳到 0.1。
    for (double v0 : {0.30, 0.50}) {
      for (int scurve = 0; scurve < 2; ++scurve) {
        char nt[64];
        std::snprintf(
          nt, sizeof(nt), "startwell_moving_v%.2f__%s", v0, scurve ? "scurve" : "trapz");
        Scenario t; t.tag = nt; t.category = Category::Production;
        t.backend = scurve ? Backend::SCurve : Backend::Trapezoidal;
        t.params = prod; t.length = 5.0;
        t.arcs = {{0.0, 0.3, 0.20}};   // 起点深井：包络 → 0.1
        t.v0 = v0;
        if (!scurve) {
          // 梯形后端在此暴露 C2：起点状态戳把 index 0 抬到 v0，而后向 pass
          // 从 i=1 起不写 index 0 —— 0↔1 之间形成阶跃（v0=0.5 实测 a=-6.0）。
          // SCurve 无此现象（index 0 曲率为 0，包络不被压低）。生产用 SCurve。
          t.expect = Expect::Degraded;
          t.degraded_reason =
            "梯形后端起点状态戳与后向 pass 的 index-0 特判共同造成首点阶跃（C2）；"
            "生产后端为 SCurve，不走此路径";
        }
        t.note = scurve ?
          "初值超包络@index0（S：无阶跃，对照组）" :
          "初值超包络@index0（T：状态戳造成阶跃）";
        add(t);
      }
    }

    // 对照 A：只有起点深井，井后纯直线 → 爬升余量充足（第一版形态，已知通过）
    {
      Scenario t; t.tag = "startwell_only__scurve"; t.category = Category::Production;
      t.backend = Backend::SCurve; t.params = prod; t.length = 5.0;
      t.arcs = {{0.0, 0.3, 0.20}}; t.v0 = 0.0;
      t.expect_peak_speed = 0.5;   // 必须过：证明深井本身不致病
      t.note = "对照：起点深井但井后无约束，爬升余量充足";
      add(t);
    }
    // 对照 B：同样的「深井 + 平缓弯」整体后移到中段 → 失败时前缀非空。
    // 若起点组失败而本组正常，则「谷的位置」而非「谷的深度」是判别性因素。
    {
      Scenario t; t.tag = "midwell_gap0.55__scurve"; t.category = Category::Production;
      t.backend = Backend::SCurve; t.params = prod; t.length = 5.0;
      t.arcs = {{2.0, 0.3, 0.20}, {2.0 + 0.3 + 0.55, 2.0, kGentleR}};
      t.v0 = 0.0;
      // 必须过：这是段级兜底（2026-08-08）生效的直接证据——同样的几何、
      // 同样的求解器失败，仅因失败点之前有前缀可保，全程速度就正常。
      t.expect_peak_speed = 0.5;
      t.note = "对照：同形态谷在中段，兜底有前缀可保";
      add(t);
    }
  }

  // ══════════ 点距不均（实机全局路径形态）══════════
  // 实测 2026-08-07 两次任务共 177 个间距样本：min 2.56 / p50 41.9 / max 70.5 mm，
  // **27 倍跨度**。而 ④ 旧曲率实现用 points_interval_ = 总长/(点数−1)（平均点距）
  // 换算 idx_dist，隐含等距假设：在 2.56 mm 的密集处，idx_dist=2 取的三点只跨
  // 约 5 mm → 三点圆拟合数值退化 → 虚假曲率 → 包络阶跃 → R48 全程蠕行。
  // 本场景在弯道中段插入密集点簇，复现该形态。
  {
    const Params prod = Params::production();
    for (double dense : {0.0026, 0.010, 0.020}) {
      char nt[56];
      std::snprintf(nt, sizeof(nt), "nonuniform_%.0fmm__scurve", dense * 1000.0);
      Scenario t; t.tag = nt; t.category = Category::Production;
      t.backend = Backend::SCurve; t.params = prod; t.length = 5.0;
      t.arcs = {{kCurveStart, kCurveLen, kCurveRadius}};
      t.path_kind = PathKind::NonUniform;
      t.dense_spacing = dense;
      t.note = "弯道中段插入密集点簇，曲率是否因间距不均退化";
      add(t);
    }
  }

  // ══════════ 纯可达性滤波后端对照组 ══════════
  //
  // 与同名 SCurve 场景一一对应，用于直接对比两种后端的形状与失败率。
  // 关注三处：起点谷能否爬出、弯道平台是否平、(v,a) 是否自洽。
  {
    const Params prod = Params::production();
    constexpr double kGentleR = 1.1426;
    for (double len : {1.0, 2.5, 5.0}) {
      Scenario t; t.tag = "straight__reach"; t.category = Category::Production;
      t.backend = Backend::Reachability; t.params = prod; t.length = len;
      add(t);
    }
    {
      Scenario t; t.tag = "curve__reach"; t.category = Category::Production;
      t.backend = Backend::Reachability; t.params = prod; t.length = 5.0;
      t.arcs = {{2.0, 0.5, kCurveRadius}};
      t.note = "对照 P_S_curve：弯道平台是否仍鼓包";
      add(t);
    }
    for (double gap : {0.30, 0.55, 0.80}) {
      char nt[56];
      std::snprintf(nt, sizeof(nt), "startwell_gap%.2f__reach", gap);
      Scenario t; t.tag = nt; t.category = Category::Production;
      t.backend = Backend::Reachability; t.params = prod; t.length = 5.0;
      t.arcs = {{0.0, 0.3, 0.20}, {0.3 + gap, 2.0, kGentleR}};
      t.v0 = 0.0;
      t.expect_peak_speed = 0.5;   // 对照组必须爬得出来，不设 XFAIL
      t.note = "对照 S_S_startwell：滤波是否不再塌陷";
      add(t);
    }
    for (double gap : {0.30, 0.45, 0.60}) {
      char nt[56];
      std::snprintf(nt, sizeof(nt), "curve_wells_gap%.2f__reach", gap);
      Scenario t; t.tag = nt; t.category = Category::Production;
      t.backend = Backend::Reachability; t.params = prod; t.length = 3.0;
      t.arcs = {{0.5, 0.25, 0.6}, {0.5 + 0.25 + gap, 0.25, 0.6},
        {0.5 + 0.5 + 2 * gap, 0.25, 0.6}};
      t.note = "对照 W_S_curve_wells";
      add(t);
    }
    for (double v0 : {0.30, 0.50}) {
      char nt[56];
      std::snprintf(nt, sizeof(nt), "startwell_moving_v%.2f__reach", v0);
      Scenario t; t.tag = nt; t.category = Category::Production;
      t.backend = Backend::Reachability; t.params = prod; t.length = 5.0;
      t.arcs = {{0.0, 0.3, 0.20}}; t.v0 = v0;
      t.note = "对照：初值超包络@index0 是否仍产生阶跃";
      add(t);
    }

    // ── 以下为「能否顶替 SCurve」而补的族 ──────────────────────────────
    //
    // 此前 `__reach` 只覆盖 10 个场景族而 `__scurve` 有 38 个：障碍、外部限速、
    // 限速区、硬停、锁存制动、点距不均、离线路、运动中起步全部**零覆盖**。
    // 缺的恰恰是包络含**阶跃**的那几族——而阶跃正是本后端连续踩坑的地方
    // （剖面中间零速洞、a_cap 被井壁污染）。没跑过就无从判断是否等价。
    //
    // 一律用**生产参数**：部署跑的就是这套，且既有 `__scurve_prod` 同族可直接
    // 对拍。偏移参数的 `__scurve` 版本（max_speed 1.0 等）不作对照——
    // 验一套机器人上不存在的配置等于没验。
    //
    // 期望一律先设 `Success`：跑完按实际表现分诊。事先假定某个具体行为正确，
    // 等于把当前实现当成规范。

    // 障碍族（A1 置零 + 缓冲带 → 包络阶跃）
    {
      Scenario t; t.tag = "obs_far__reach"; t.category = Category::Production;
      t.backend = Backend::Reachability; t.params = prod; t.length = 5.0;
      t.obstacles = {{3.0, 5.0}};
      t.note = "对照 obs_far__scurve_prod：远处障碍前是否平顺停住";
      add(t);
    }
    {
      Scenario t; t.tag = "obs_after_curve__reach"; t.category = Category::Production;
      t.backend = Backend::Reachability; t.params = prod; t.length = 5.0;
      t.arcs = {{kCurveStart, kCurveLen, kCurveRadius}}; t.obstacles = {{3.0, 5.0}};
      t.note = "弯道与障碍叠加：两处包络下压能否复合";
      add(t);
    }
    {
      Scenario t; t.tag = "fast_near_obs__reach"; t.category = Category::Production;
      t.backend = Backend::Reachability; t.params = prod;
      t.length = 5.0; t.v0 = 0.6; t.obstacles = {{0.5, 5.0}};
      t.note = "全速逼近近距障碍（生产制动距离 0.665 m > 0.5 m，物理不可停）";
      add(t);
    }

    // 外部限速与限速区（pointSpeedHint 接缝）
    {
      Scenario t; t.tag = "limit_drop__reach"; t.category = Category::Production;
      t.backend = Backend::Reachability; t.params = prod;
      t.length = 5.0; t.v0 = 0.6; t.external_limit = 0.2;
      t.note = "对照 limit_drop__scurve_prod：运动中限速下调";
      add(t);
    }
    {
      Scenario t; t.tag = "limit_drop_while_moving__reach"; t.category = Category::Production;
      t.backend = Backend::Reachability; t.params = prod;
      t.length = 5.0; t.v0 = 0.6; t.external_limit = 0.3;
      t.note = "区域限速的常规形态（v0 取生产上限 0.6）";
      add(t);
    }
    {
      Scenario t; t.tag = "speed_zone__reach"; t.category = Category::Production;
      t.backend = Backend::Reachability; t.params = prod;
      t.length = 5.0; t.zones = {{1.5, 3.0, 0.3}};
      t.note = "正常→慢区→正常：进区前是否提前减速、出区后是否重新加速";
      add(t);
    }

    // 终端模式：硬停（end_speed=0，无蠕行区）
    {
      Scenario t; t.tag = "hardstop__reach"; t.category = Category::Production;
      t.backend = Backend::Reachability; t.params = prod;
      t.length = 1.0; t.end_speed = 0.0; t.override_end_speed = true;
      t.note = "硬停：终点锚零，后向遍历自动生成收敛坡道";
      add(t);
    }

    // 锁存制动：本后端**完全忽略** `terminal_brake_latched` 入参，
    // 故 latched / unlatched 两版应逐字节相同。这不是冗余用例——
    // 它把「锁存机制在本后端下是空转的」这件事钉成可执行的事实。
    for (double len : {0.6, 0.7, 0.8, 1.0}) {
      for (int latched = 0; latched < 2; ++latched) {
        Scenario t;
        t.tag = latched ? "latched__reach" : "unlatched__reach";
        t.category = Category::Production;
        t.backend = Backend::Reachability; t.params = prod;
        t.length = len; t.v0 = 0.6;
        t.terminal_brake_latched = (latched != 0);
        t.terminal_execution_delay = 0.35;
        t.terminal_safety_margin = 0.05;
        t.note = latched ? "已锁存（本后端忽略该入参）" : "未锁存对照";
        add(t);
      }
    }

    // 点距不均（实机全局路径形态，27 倍跨度）
    for (double dense : {0.0026, 0.010, 0.020}) {
      char nt[56];
      std::snprintf(nt, sizeof(nt), "nonuniform_%.0fmm__reach", dense * 1000.0);
      Scenario t; t.tag = nt; t.category = Category::Production;
      t.backend = Backend::Reachability; t.params = prod; t.length = 5.0;
      t.arcs = {{kCurveStart, kCurveLen, kCurveRadius}};
      t.path_kind = PathKind::NonUniform; t.dense_spacing = dense;
      t.note = "弯道中段密集点簇：jerk 斜率分母取段时长在此最敏感";
      add(t);
    }

    // 运动中偏离路网（跟踪误差造成的起点折角）
    for (double lat : {0.05, 0.10, 0.15}) {
      char nt[56];
      std::snprintf(nt, sizeof(nt), "offroute_lat%.2f__reach", lat);
      Scenario t; t.tag = nt; t.category = Category::Production;
      t.backend = Backend::Reachability; t.params = prod; t.length = 3.0;
      t.path_kind = PathKind::OffRouteStart; t.lateral_offset = lat; t.v0 = 0.4;
      t.note = "运动中偏离路网，曲率拟合是否被折角污染";
      add(t);
    }

    // 运动中起步（无约束直路）。v0=1.0 超过生产 max_speed 0.6，
    // 是「初值在可行集之外」的直路形态，与起点井那组机制不同。
    for (double v0 : {0.3, 0.6, 1.0}) {
      char nt[56];
      std::snprintf(nt, sizeof(nt), "straight_moving_v%.1f__reach", v0);
      Scenario t; t.tag = nt; t.category = Category::Production;
      t.backend = Backend::Reachability; t.params = prod; t.length = 5.0;
      t.v0 = v0;
      t.note = v0 > prod.max_speed ? "初值超 max_speed（生产 0.6）" : "运动中起步";
      add(t);
    }

    // 起点井的两个对照（谷的位置 vs 谷的深度）
    {
      Scenario t; t.tag = "startwell_only__reach"; t.category = Category::Production;
      t.backend = Backend::Reachability; t.params = prod; t.length = 5.0;
      t.arcs = {{0.0, 0.3, 0.20}}; t.v0 = 0.0;
      t.expect_peak_speed = 0.5;
      t.note = "对照：起点深井但井后无约束";
      add(t);
    }
    {
      Scenario t; t.tag = "midwell_gap0.55__reach"; t.category = Category::Production;
      t.backend = Backend::Reachability; t.params = prod; t.length = 5.0;
      t.arcs = {{2.0, 0.3, 0.20}, {2.0 + 0.3 + 0.55, 2.0, kGentleR}}; t.v0 = 0.0;
      t.expect_peak_speed = 0.5;
      t.note = "对照：同形态谷在中段";
      add(t);
    }

    // 鲁棒性：非法/退化输入。SCurve 是现役后端、这些输入它都见过，
    // 顶替前本后端必须证明同样不崩、不产生 NaN 剖面。
    {
      Scenario t; t.tag = "malformed_nan_pose__reach"; t.category = Category::Malformed;
      t.backend = Backend::Reachability; t.params = prod;
      t.length = 2.0; t.path_kind = PathKind::NaNPose;
      t.note = "中段坐标为 NaN";
      add(t);
    }
    {
      Scenario t; t.tag = "malformed_inf_pose__reach"; t.category = Category::Malformed;
      t.backend = Backend::Reachability; t.params = prod;
      t.length = 2.0; t.path_kind = PathKind::InfPose;
      t.note = "中段坐标为 Inf";
      add(t);
    }
    {
      Scenario t; t.tag = "malformed_teleport__reach"; t.category = Category::Malformed;
      t.backend = Backend::Reachability; t.params = prod;
      t.length = 2.0; t.path_kind = PathKind::Teleport;
      t.note = "相邻两点瞬移 10m（丢帧 / 拼接错）";
      add(t);
    }
    {
      Scenario t; t.tag = "degenerate_all_obstacle__reach"; t.category = Category::Degenerate;
      t.backend = Backend::Reachability; t.params = prod;
      t.length = 2.0; t.obstacles = {{0.0, 2.0}};
      t.note = "整条路径都在障碍里（仅起点豁免）";
      add(t);
    }
  }

  // ══════════ 失败策略（步 1 引入的语义，此前零覆盖）══════════
  {
    Scenario t; t.tag = "failpolicy_unavailable_fail_closed__trapz";
    t.category = Category::FailPolicy; t.expect = Expect::EmptyPlan;
    t.length = 2.5; t.obstacle_available = false;
    t.unavailable_policy = P::FAIL_CLOSED;
    t.note = "障碍不可评估 + FAIL_CLOSED → 必须返回空计划";
    add(t);
  }
  {
    Scenario t; t.tag = "failpolicy_unavailable_fail_open__trapz";
    t.category = Category::FailPolicy; t.expect = Expect::Success;
    t.length = 2.5; t.obstacle_available = false;
    t.unavailable_policy = P::FAIL_OPEN;
    t.note = "同上但 FAIL_OPEN → 必须照常出计划（对照组）";
    add(t);
  }

  return v;
}

}  // namespace speed_offline

#endif  // TOOLS__SPEED_SCENARIOS_HPP_
