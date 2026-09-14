// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 离线速度剖面生成器 + 校验器（不依赖 ROS 运行时：障碍由子类注入）。
// 本文件只做**编排与报告**，三块内容各自独立：
//   speed_scenarios.hpp  场景定义与场景表（纯数据）
//   speed_harness.hpp    路径构造 + 规划器注入 + 跑一个场景
//   speed_checks.hpp     期望校验 + 不变量 + 单调性 + 跨场景检查
//
// 用法：ros2 run nav2_speed_planner speed_profile_gen [输出目录]
//   缺省输出 $HOME/.dexe/navigation/speed_planner/csv/
//   退出码：0 = 全部通过（XFAIL 不阻塞）；1 = 有 FAIL
//
// 每场景导一份 CSV（s,v,accel,jerk,rel_time,region），
// plot_speed_profiles.py 按目录 glob 自动出四联详图——加场景无需改绘图代码。

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <sys/stat.h>
#include <utility>
#include <vector>

#include "speed_checks.hpp"
#include "speed_harness.hpp"
#include "speed_scenarios.hpp"

namespace
{

std::string g_out_dir;

void mkdirP(const std::string & path)
{
  std::string acc;
  for (size_t i = 0; i < path.size(); ++i) {
    acc += path[i];
    if (path[i] == '/' || i + 1 == path.size()) {
      ::mkdir(acc.c_str(), 0755);
    }
  }
}

/// CSV 按后端分子目录存放。tag 里虽已带 `__trapz`/`__scurve`/`__reach` 后缀，
/// 但一个目录下平铺三百多个文件时肉眼仍难区分是哪个规划器的产物；
/// 物理分开后可以直接整目录对比 / 整目录删。
const char * backendDir(speed_offline::Backend b)
{
  switch (b) {
    case speed_offline::Backend::Trapezoidal: return "trapezoidal";
    case speed_offline::Backend::SCurve: return "scurve";
    case speed_offline::Backend::Reachability: return "reachability";
  }
  return "other";
}

std::vector<double> dumpCsv(
  const speed_offline::Scenario & sc,
  const nav_interfaces::msg::RobotTrajectory & traj)
{
  const std::string dir = g_out_dir + "/" + backendDir(sc.backend);
  mkdirP(dir);
  char fn[512];
  std::snprintf(
    fn, sizeof(fn), "%s/%s_%.2fm.csv", dir.c_str(), sc.tag.c_str(), sc.length);
  FILE * f = std::fopen(fn, "w");
  if (f == nullptr) {
    std::fprintf(stderr, "cannot open %s\n", fn);
    std::exit(1);
  }
  std::fprintf(f, "s,v,accel,jerk,rel_time,region,kappa\n");
  std::vector<double> arc;
  double s = 0.0;
  for (size_t i = 0; i < traj.points.size(); ++i) {
    if (i > 0) {
      const auto & a = traj.points[i - 1].pose.pose.position;
      const auto & b = traj.points[i].pose.pose.position;
      s += std::hypot(b.x - a.x, b.y - a.y);
    }
    arc.push_back(s);
    const auto & p = traj.points[i];
    // kappa 是步 0 的关键观测量：短跨度三点拟合是否算出虚假大曲率，
    // 此前完全不可见——所有关于"不等距/短跨度致病"的判断都只能靠推理。
    std::fprintf(
      f, "%.4f,%.5f,%.5f,%.5f,%.4f,%s,%.6f\n", s, p.longitudinal_velocity,
      p.acceleration, p.jerk,
      p.relative_time.sec + p.relative_time.nanosec * 1e-9,
      p.region_type.c_str(), p.path_kappa);
  }
  std::fclose(f);
  return arc;
}

}  // namespace

int main(int argc, char ** argv)
{
  using speed_offline::Category;
  using speed_offline::Expect;

  if (argc > 1) {
    g_out_dir = argv[1];
  } else {
    const char * home = std::getenv("HOME");
    g_out_dir = std::string(home ? home : ".") + "/.dexe/navigation/speed_planner/csv";
  }
  mkdirP(g_out_dir);
  std::printf(
    "速度剖面 CSV 输出目录：%s/{trapezoidal,scurve,reachability}/\n\n",
    g_out_dir.c_str());

  const auto scenarios = speed_offline::buildScenarios();
  std::printf("== 1/2 生成剖面（%zu 个场景）==\n", scenarios.size());

  bool first = true;
  Category last = Category::Distance;
  for (const auto & sc : scenarios) {
    if (first || sc.category != last) {
      first = false;
      last = sc.category;
      std::printf("\n  [%s]\n", speed_offline::categoryName(sc.category));
    }
    const auto path =
      speed_offline::makePath(
        sc.length, sc.arcs, sc.path_kind, sc.lateral_offset, sc.dense_spacing);
    const auto traj = speed_offline::runScenario(sc, path);
    const auto arc = dumpCsv(sc, traj);
    std::printf(
      "    %-28s %5zu 点  %s\n", sc.tag.c_str(), traj.points.size(),
      sc.note.c_str());

    // 不可行时打印归因——回答"卡在求解器还是卡在我们自己的判据"
    {
      const auto & d = speed_offline::lastDiagnostics();
      if (d.terminal_infeasible) {
        static const char * kReason[] = {
          "-", "稳定末段生成失败[物理]", "兜底后仍不可达[物理]",
          "duration budget 退化[数值]", "求解器报错[数值]",
          "求解器输出非有限[数值]", "迭代预算耗尽[数值]"};
        static const char * kKind[] = {"可行", "物理不可行", "求解失败"};
        const int r = d.terminal_infeasible_reason;
        std::printf(
          "        ↳ %s / %s / %s\n",
          kKind[d.terminal_infeasible_kind >= 0 && d.terminal_infeasible_kind < 3 ?
          d.terminal_infeasible_kind : 0],
          (r >= 0 && r < 7) ? kReason[r] : "?",
          d.terminal_result_provenance ? "走了制动兜底" : "正常求解");
        std::printf(
          "          进兜底首因：%s\n",
          (d.terminal_entry_reason >= 0 && d.terminal_entry_reason < 7) ?
          kReason[d.terminal_entry_reason] : "?");
      }
    }
    speed_offline::checkExpectation(sc, traj);
    speed_offline::checkPeakSpeed(sc, traj);
    if (sc.expect == Expect::Success || sc.expect == Expect::Degraded) {
      speed_offline::checkInvariants(sc, traj, arc);
      speed_offline::checkMonotonicity(sc, path);
      speed_offline::collectCurveMin(sc, traj, arc);
    }
  }
  speed_offline::checkCurvatureInvariance();

  // ── 校验报告 ──
  std::printf("\n== 2/2 校验 ==\n");
  size_t fails = 0, xfails = 0, passed = 0;
  std::map<std::string, std::pair<size_t, size_t>> per_cat;

  for (const auto & c : speed_offline::checks()) {
    auto & e = per_cat[speed_offline::categoryName(c.category)];
    ++e.second;
    if (c.passed) {++e.first; ++passed;}
  }

  std::string cur;
  for (const auto & c : speed_offline::checks()) {
    if (c.passed || c.expected_fail) {continue;}
    if (c.scenario != cur) {
      cur = c.scenario;
      std::printf("\n  ✗ %s\n", cur.c_str());
    }
    std::printf("      %-30s %s\n", c.name.c_str(), c.detail.c_str());
    ++fails;
  }
  cur.clear();
  for (const auto & c : speed_offline::checks()) {
    if (c.passed || !c.expected_fail) {continue;}
    if (c.scenario != cur) {
      cur = c.scenario;
      std::printf("\n  ⚠ XFAIL %s（已知缺陷，不阻塞）\n", cur.c_str());
    }
    std::printf("      %-30s %s\n", c.name.c_str(), c.detail.c_str());
    ++xfails;
  }

  std::printf("\n  按类别：\n");
  for (const auto & kv : per_cat) {
    std::printf(
      "    %-14s %3zu / %3zu 通过\n", kv.first.c_str(), kv.second.first,
      kv.second.second);
  }
  std::printf(
    "\n  合计 %zu 项：通过 %zu / 失败 %zu / 预期失败 %zu\n",
    speed_offline::checks().size(), passed, fails, xfails);
  std::printf(fails == 0 ? "\n完成：全部通过。\n" : "\n完成：存在失败项。\n");
  return fails == 0 ? 0 : 1;
}
