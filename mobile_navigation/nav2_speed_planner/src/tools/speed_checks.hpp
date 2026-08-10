// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 离线速度剖面验证——**校验逻辑**。
//
// 断言纪律：只查"与剖面形状无关"的性质。三约束叠加后的剖面该长什么样，
// 事先写不出预期；硬写等于把当前行为当成正确答案钉死，反而有害。
// 形状相关的判断留给人看详图（plot_speed_profiles.py 按目录 glob 自动出图）。

#ifndef TOOLS__SPEED_CHECKS_HPP_
#define TOOLS__SPEED_CHECKS_HPP_

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#include "speed_harness.hpp"
#include "speed_scenarios.hpp"

namespace speed_offline
{

struct Check
{
  std::string scenario;
  Category category{Category::Distance};
  std::string name;
  bool passed{false};
  bool expected_fail{false};
  std::string detail;
};

inline std::vector<Check> & checks()
{
  static std::vector<Check> v;
  return v;
}

inline void record(
  const Scenario & sc, const std::string & name, bool ok,
  const std::string & detail = "")
{
  checks().push_back(
    {sc.tag, sc.category, name, ok, sc.expect == Expect::Degraded, detail});
}

// 曲率段最低速：跨场景检查"弯道限速不应随巡航漂移"
inline std::vector<std::pair<std::string, double>> & curveMins()
{
  static std::vector<std::pair<std::string, double>> v;
  return v;
}

inline void checkExpectation(
  const Scenario & sc, const nav_interfaces::msg::RobotTrajectory & traj)
{
  char buf[160];
  std::snprintf(buf, sizeof(buf), "实得 %zu 点", traj.points.size());

  switch (sc.expect) {
    case Expect::EmptyPlan:
      record(sc, "必须返回空计划", traj.points.empty(), buf);
      return;
    case Expect::ZeroSpeed: {
        record(sc, "非空轨迹", !traj.points.empty(), buf);
        double vmax = 0.0;
        for (const auto & p : traj.points) {
          vmax = std::max(vmax, std::fabs(static_cast<double>(p.longitudinal_velocity)));
        }
        std::snprintf(buf, sizeof(buf), "max|v|=%.5f", vmax);
        record(sc, "全程零线速度", vmax <= 1e-9, buf);
        return;
      }
    default:
      record(sc, "非空轨迹", !traj.points.empty(), buf);
  }
}

inline void checkInvariants(
  const Scenario & sc, const nav_interfaces::msg::RobotTrajectory & traj,
  const std::vector<double> & arc)
{
  if (traj.points.empty()) {return;}
  char buf[256];

  // ⚠ NaN 必须显式查：NaN 的一切比较都是 false，所有 `x <= 限值` 形式的
  //   断言都会被它静默绕过（实测 E_nan_pose 全部不变量"通过"，实际整条
  //   计划已塌成 0.03 爬行且 s 列全是 nan）。
  size_t bad = 0;
  for (size_t i = 0; i < traj.points.size(); ++i) {
    const auto & p = traj.points[i];
    const double vals[] = {
      p.pose.pose.position.x, p.pose.pose.position.y,
      static_cast<double>(p.longitudinal_velocity),
      static_cast<double>(p.acceleration), arc[i]};
    for (double x : vals) {
      if (std::isnan(x) || std::isinf(x)) {++bad; break;}
    }
  }
  std::snprintf(buf, sizeof(buf), "含 NaN/Inf 的点 %zu / %zu", bad, traj.points.size());
  record(sc, "输出无 NaN/Inf", bad == 0, buf);

  double vmin = 1e18, vmax = -1e18;
  for (const auto & p : traj.points) {
    vmin = std::min(vmin, static_cast<double>(p.longitudinal_velocity));
    vmax = std::max(vmax, static_cast<double>(p.longitudinal_velocity));
  }
  std::snprintf(buf, sizeof(buf), "v∈[%.4f, %.4f]", vmin, vmax);
  record(sc, "v ≥ 0", vmin >= -1e-6, buf);
  const double ceiling = sc.external_limit > 0.0 ?
    std::min(sc.params.max_speed, sc.external_limit) : sc.params.max_speed;
  record(sc, "v ≤ 上界", vmax <= ceiling + 1e-4, buf);

  bool s_mono = true, t_mono = true;
  double prev_t = -1e18;
  for (size_t i = 0; i < traj.points.size(); ++i) {
    if (i > 0 && arc[i] < arc[i - 1] - 1e-9) {s_mono = false;}
    const double t = traj.points[i].relative_time.sec +
      traj.points[i].relative_time.nanosec * 1e-9;
    if (t < prev_t - 1e-9) {t_mono = false;}
    prev_t = t;
  }
  record(sc, "s 单调不减", s_mono);
  record(sc, "rel_time 单调不减", t_mono);

  double amax = 0.0;
  for (const auto & p : traj.points) {
    amax = std::max(amax, std::fabs(static_cast<double>(p.acceleration)));
  }
  const double alim = std::max(sc.params.accel, std::fabs(sc.params.decel));
  std::snprintf(buf, sizeof(buf), "max|a|=%.4f 限值=%.2f", amax, alim);
  record(sc, "|a| ≤ 限值", amax <= alim + 1e-3, buf);

  // jerk 断言只对**声明做 jerk 限制**的后端成立。T 型是恒加速度剖面，
  // 段界本就是无穷 jerk，对它断言等于要求它变成另一个后端。
  //
  // ⚠ 本条此前只覆盖 SCurve，且 `p.jerk` 从未被任何后端填写（实测全 252 行
  //   恒 0）——`0 ≤ 0.5` 恒真，整套离线校验从未真正检查过 jerk。祖传绿。
  //   现由 `fillJerkFromAccel()` 统一回填，并把 Reachability 一并纳入。
  if (sc.backend == Backend::SCurve || sc.backend == Backend::Reachability) {
    double jmax = 0.0, jmax_s = -1.0;
    for (size_t i = 0; i < traj.points.size(); ++i) {
      const double j = std::fabs(static_cast<double>(traj.points[i].jerk));
      if (j > jmax) {jmax = j; jmax_s = arc[i];}
    }
    std::snprintf(
      buf, sizeof(buf), "max|jerk|=%.4f @s=%.2f 限值=%.2f",
      jmax, jmax_s, sc.params.max_jerk);
    record(sc, "|jerk| ≤ 限值", jmax <= sc.params.max_jerk + 1e-3, buf);
  }

  if (!sc.obstacles.empty()) {
    double worst = 0.0, worst_s = -1.0;
    for (size_t i = 0; i < traj.points.size(); ++i) {
      if (arc[i] < 0.15) {continue;}   // 起步豁免窗
      // ⚠ 带判定必须用 x，与 Harness::checkObstacleAt 同坐标。用弧长会在
      //   弯道场景误判：转过 1 rad 后 x < s，弧长 3.0 处 x 才 ~2.7。
      const double bx = traj.points[i].pose.pose.position.x;
      for (const auto & b : sc.obstacles) {
        if (bx >= b.from && bx <= b.to) {
          const double v = traj.points[i].longitudinal_velocity;
          if (v > worst) {worst = v; worst_s = arc[i];}
        }
      }
    }
    std::snprintf(buf, sizeof(buf), "带内最大 v=%.4f @ s=%.2f", worst, worst_s);
    record(sc, "障碍带内零速", worst <= 1e-3, buf);
  }

  const double vlast = traj.points.back().longitudinal_velocity;
  const double es = sc.override_end_speed ? sc.end_speed : sc.params.end_speed;
  std::snprintf(buf, sizeof(buf), "v_last=%.4f end_speed=%.3f", vlast, es);
  record(sc, "末点 ≤ end_speed", vlast <= es + 1e-3, buf);
}

// 单调性：同一条路径上，移除一个**可移除**约束后逐点速度不得更低。
// 曲率不可移除（烘焙在几何里）；end_speed 也不参与——它不是可加减的标量，
// 而是参与终端模式选择（>0 启用 stop_distance 爬行区，=0 走平滑减速到零），
// 两者剖面中段互有高低，单调性前提不成立。
inline void checkMonotonicity(const Scenario & sc, const nav_msgs::msg::Path & path)
{
  if (sc.expect != Expect::Success && sc.expect != Expect::Degraded) {return;}
  const auto full = runScenario(sc, path);
  if (full.points.empty()) {return;}

  auto compare = [&](Scenario relaxed, const std::string & what) {
      const auto loose = runScenario(relaxed, path);
      if (loose.points.size() != full.points.size()) {
        record(sc, "单调性（移除" + what + "）", false, "点数不一致，无法逐点比较");
        return;
      }
      double worst = 0.0; size_t worst_i = 0;
      for (size_t i = 0; i < full.points.size(); ++i) {
        const double d = full.points[i].longitudinal_velocity -
          loose.points[i].longitudinal_velocity;
        if (d > worst) {worst = d; worst_i = i;}
      }
      char buf[160];
      std::snprintf(buf, sizeof(buf), "最大超出 %.5f m/s @ idx %zu", worst, worst_i);
      record(sc, "单调性（移除" + what + "）", worst <= 1e-4, buf);
    };

  if (!sc.obstacles.empty()) {
    Scenario r = sc; r.obstacles.clear();
    compare(r, "障碍");
  }
  if (sc.external_limit > 0.0) {
    Scenario r = sc; r.external_limit = 0.0;
    compare(r, "外部限速");
  }
}

inline void collectCurveMin(
  const Scenario & sc, const nav_interfaces::msg::RobotTrajectory & traj,
  const std::vector<double> & arc)
{
  if (sc.tag.rfind("X_T_limit", 0) != 0) {return;}
  double m = 1e18;
  for (size_t i = 0; i < traj.points.size(); ++i) {
    if (arc[i] >= kCurveStart && arc[i] <= kCurveStart + kCurveLen) {
      m = std::min(m, static_cast<double>(traj.points[i].longitudinal_velocity));
    }
  }
  if (m < 1e17) {curveMins().emplace_back(sc.tag, m);}
}

// 横向加速度限制 v=√(a_y/|κ|) 对给定半径是常数，与外部限速无关。
// 当前公式 cruise×radius/min_radius 依赖巡航 → 本检查今天必然失败。
/// 峰值速度下界：计划是否**爬得上去**。
///
/// 整套校验此前只有上界与不变量，没有任何一条要求"该快的时候要快"。
/// 结果是 R48 全程蠕行（0.205 / 5 m 直路）通过了全部 841 项——处处低速
/// 物理上完全合法。下界断言是这类"结果合法但语义错误"缺陷唯一的抓手。
inline void checkPeakSpeed(
  const Scenario & sc, const nav_interfaces::msg::RobotTrajectory & traj)
{
  if (sc.expect_peak_speed <= 0.0) {return;}
  double peak = 0.0;
  for (const auto & p : traj.points) {
    peak = std::max(peak, static_cast<double>(p.longitudinal_velocity));
  }
  char buf[192];
  std::snprintf(
    buf, sizeof(buf), "峰值 v=%.5f 要求 ≥%.3f", peak, sc.expect_peak_speed);
  Scenario pseudo = sc;
  if (!sc.peak_speed_xfail.empty()) {
    pseudo.expect = Expect::Degraded;
    pseudo.degraded_reason = sc.peak_speed_xfail;
  }
  record(pseudo, "峰值速度 ≥ 下界", peak >= sc.expect_peak_speed, buf);
}

inline void checkCurvatureInvariance()
{
  if (curveMins().size() < 2) {return;}

  // 判据从「跨场景相等」改为**预测值比对**。
  //
  // 旧判据要求弯道段最低速与外部限速完全无关，但那只在曲率是唯一约束时成立：
  // 外部限速 0.3 比曲率限速 √(0.3×0.5)=0.387 更严时，弯道最低速取 0.3 是
  // **正确行为**，不是漂移。旧公式 `v = cruise × r / min_r` 掩盖了这一点——
  // 它本身就依赖巡航，所以两种效应混在一起，只能整体记 XFAIL。
  //
  // 公式改为 `v = √(a_lat_max/|κ|)` 后二者可分离：逐场景预测
  //   v_expect = min(max_speed, external_limit, √(a_lat_max × r))
  // 并核对实际横向加速度不超过 a_lat_max。
  // 参与本检查的 X_T_limit 场景用的是**离线默认参数**（max_speed 1.0），不是生产参数。
  const Params prod{};
  const double v_curve = std::sqrt(prod.max_lateral_accel * kCurveRadius);
  char buf[256];
  bool all_ok = true;
  double worst_lat = 0.0;
  for (const auto & kv : curveMins()) {
    // tag 形如 X_T_limit<lim>_curve
    double lim = 0.0;
    const size_t pos = kv.first.find("limit");
    if (pos != std::string::npos) {lim = std::atof(kv.first.c_str() + pos + 5);}
    const double expect = std::min(
      lim > 0.0 ? lim : prod.max_speed, std::min(prod.max_speed, v_curve));
    if (std::fabs(kv.second - expect) > 5e-3) {all_ok = false;}
    worst_lat = std::max(worst_lat, kv.second * kv.second / kCurveRadius);
  }
  std::snprintf(
    buf, sizeof(buf),
    "逐场景预测 min(max_speed, external_limit, √(a_lat×r)=%.4f) 命中；"
    "实测最大横向加速度 %.3f m/s²（限值 %.3f）",
    v_curve, worst_lat, prod.max_lateral_accel);
  Scenario pseudo;
  pseudo.tag = "曲率约束（跨场景）";
  pseudo.category = Category::Curve;
  record(
    pseudo, "弯道限速 = 物理式预测值",
    all_ok && worst_lat <= prod.max_lateral_accel + 1e-3, buf);
}

}  // namespace speed_offline

#endif  // TOOLS__SPEED_CHECKS_HPP_
