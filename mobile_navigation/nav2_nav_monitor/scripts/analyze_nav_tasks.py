#!/usr/bin/env python3
# Copyright 2026 DEXE Robot Engineer
# SPDX-License-Identifier: Apache-2.0
"""导航任务批量分析：一条指令扫描 nav_monitor 目录下全部 nav_task_*.json，
生成两份汇总（按任务 / 按导航点）+ 每任务图表，保存到带时间戳的子目录。

数据来源：nav2_nav_monitor output_dir（默认 ~/.dexe/navigation/nav_monitor）

用法：
  ros2 run nav2_nav_monitor analyze_nav_tasks.py [目录] [--no-per-task]
  （目录缺省为 ~/.dexe/navigation/nav_monitor）

输出（<目录>/analysis_<时间戳>/）：
  summary_tasks.png   按任务横铺 + 总平均——
    表 0：总平均统计（任务数/成功率/三维平均误差/平均总时长/各阶段平均时间/速度）
    图 1：每任务三维到达误差（纵向/横向带符号条 + |yaw| 散点，全部标值）
    图 2：每任务执行时间（四阶段堆叠 + 顶部总时长标值 + succeeded 均值线）
    图 3：每任务平均/最大速度（标值）
  summary_tasks.csv   每任务一行（含纵/横/角误差列）+ 末尾"均值(succeeded)"行
  summary_points.png  按导航点聚类合并——
    图 1：每点位三维误差（|纵|/|横| mean+max 分组条 + |yaw| mean，全部标值）
    图 2：每点位平均速度（标值）
    图 3：每点位各阶段平均时间 + 总时长（分组条，全部标值）
    图 4：每点位任务次数与成功率（标值）
  summary_points.csv  每点位一行的聚合指标
  tasks/              每任务的路径图 + 速度图（复用 plot_nav_task_path/speed.py）

误差三维语义（goal 朝向系，与 goal checker lon/lat 容差一致）：
  纵向 = 沿目标朝向的前后偏差（正=过冲）；横向 = 垂直方向偏差；角度 = yaw 差。
（精度/阶段时间/速度等均值只统计 succeeded 任务——中止/取消任务的
"误差"是中途停止位置，不是到达精度。）
"""
import argparse
import csv
import json
import math
import os
import subprocess
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
matplotlib.rcParams["font.sans-serif"] = [
    "Noto Sans CJK SC", "WenQuanYi Micro Hei", "AR PL UMing CN", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

from nav_task_common import resolve_data_dir

PHASES = ["prepare", "align", "track", "approach"]
PHASE_COLORS = {
    "prepare": "#ffd54f", "align": "#64b5f6", "track": "#81c784",
    "approach": "#f48fb1", "其他": "#e0e0e0",
}
RESULT_COLORS = {"succeeded": "#2e7d32", "aborted": "#c62828",
                 "canceled": "#f9a825", "preempted_unknown": "#9e9e9e"}
GOAL_CLUSTER_TOL = 0.3  # m，目标点聚类容差


def mean(vals):
    return sum(vals) / len(vals) if vals else 0.0


def label_bars(ax, bars, fmt="{:.3f}", fontsize=7, signed=False):
    """条形图逐条标值。"""
    for b in bars:
        h = b.get_height() + b.get_y() if not signed else b.get_height()
        va = "bottom" if h >= 0 else "top"
        ax.annotate(fmt.format(b.get_height()),
                    (b.get_x() + b.get_width() / 2, h),
                    ha="center", va=va, fontsize=fontsize)


def phase_durations(transitions, total):
    """[[t,phase],...] + 总时长 → {phase: 秒}；无切换记录时全归"其他"。"""
    durs = {p: 0.0 for p in PHASES}
    durs["其他"] = 0.0
    if not transitions:
        durs["其他"] = total
        return durs
    durs["其他"] = max(transitions[0][0], 0.0)  # 首次切换前
    for i, (t, name) in enumerate(transitions):
        end = transitions[i + 1][0] if i + 1 < len(transitions) else total
        durs[name if name in durs else "其他"] += max(end - t, 0.0)
    return durs


def load_tasks(d):
    tasks = []
    for fn in sorted(os.listdir(d)):
        if not (fn.startswith("nav_task_") and fn.endswith(".json")):
            continue
        try:
            with open(os.path.join(d, fn)) as f:
                data = json.load(f)
            data["_file"] = fn
            tasks.append(data)
        except (json.JSONDecodeError, OSError) as e:
            print(f"跳过无法解析的 {fn}: {e}", file=sys.stderr)
    return tasks


def cluster_goals(tasks):
    """目标点聚类（贪心，容差 GOAL_CLUSTER_TOL）→ (簇中心列表, 每任务簇编号)。"""
    centers, labels = [], []
    for t in tasks:
        g = t.get("goal_pose", {})
        gx, gy = g.get("x", 0.0), g.get("y", 0.0)
        for i, (cx, cy) in enumerate(centers):
            if math.hypot(gx - cx, gy - cy) < GOAL_CLUSTER_TOL:
                labels.append(i)
                break
        else:
            centers.append((gx, gy))
            labels.append(len(centers) - 1)
    return centers, labels


def lon_lat_err(err, goal_yaw):
    """goal 系纵/横误差：JSON 已有字段直接用，旧数据由世界系 x/y 旋转得到。"""
    if "longitudinal" in err and "lateral" in err:
        return err["longitudinal"], err["lateral"]
    ex, ey = err.get("x", 0.0), err.get("y", 0.0)
    lon = ex * math.cos(goal_yaw) + ey * math.sin(goal_yaw)
    lat = -ex * math.sin(goal_yaw) + ey * math.cos(goal_yaw)
    return lon, lat


def build_rows(tasks):
    rows = []
    for t in tasks:
        total = t.get("end_time", 0) - t.get("start_time", 0)
        durs = phase_durations(t.get("phase_transitions", []), total)
        err = t.get("goal_reached_error", {})
        goal = t.get("goal_pose", {})
        lon, lat = lon_lat_err(err, goal.get("yaw", 0.0))
        sm = t.get("smoothed_vel", {})
        rows.append({
            "file": t["_file"],
            "result": t.get("result", "?"),
            "goal_x": round(goal.get("x", 0.0), 3),
            "goal_y": round(goal.get("y", 0.0), 3),
            "duration_s": round(total, 2),
            "err_lon_m": round(lon, 4),
            "err_lat_m": round(lat, 4),
            "err_yaw_deg": round(math.degrees(err.get("yaw", 0.0)), 2),
            "err_dist_m": round(err.get("distance", 0.0), 4),
            "avg_speed_mps": round(sm.get("avg_linear_speed", 0.0), 3),
            "max_speed_mps": round(sm.get("max_linear_speed", 0.0), 3),
            "passing_length_m": round(t.get("passing_length", 0.0), 3),
            "replanning_count": t.get("replanning_count", 0),
            "recoveries": t.get("number_of_recoveries", 0),
            **{f"{p}_s": round(durs[p], 2) for p in PHASES},
            "other_s": round(durs["其他"], 2),
        })
    return rows


def overall_stats(rows):
    """总平均（succeeded 任务）。"""
    ok = [r for r in rows if r["result"] == "succeeded"]
    s = {
        "任务数": f"{len(rows)}（成功 {len(ok)}，成功率 {len(ok)/len(rows)*100:.0f}%）",
        "平均纵向误差 |lon|": f"{mean([abs(r['err_lon_m']) for r in ok]):.4f} m"
                            f"（max {max([abs(r['err_lon_m']) for r in ok], default=0):.4f}）",
        "平均横向误差 |lat|": f"{mean([abs(r['err_lat_m']) for r in ok]):.4f} m"
                            f"（max {max([abs(r['err_lat_m']) for r in ok], default=0):.4f}）",
        "平均角度误差 |yaw|": f"{mean([abs(r['err_yaw_deg']) for r in ok]):.2f} °"
                            f"（max {max([abs(r['err_yaw_deg']) for r in ok], default=0):.2f}）",
        "平均总时长": f"{mean([r['duration_s'] for r in ok]):.2f} s",
        **{f"平均 {p} 段时间": f"{mean([r[f'{p}_s'] for r in ok]):.2f} s"
           for p in PHASES},
        "平均速度(每任务avg的均值)": f"{mean([r['avg_speed_mps'] for r in ok]):.3f} m/s",
        "平均实走里程": f"{mean([r['passing_length_m'] for r in ok]):.2f} m",
        "总重规划/recovery 次数":
            f"{sum(r['replanning_count'] for r in rows)} / "
            f"{sum(r['recoveries'] for r in rows)}",
    }
    return s, ok


def average_row(ok_rows):
    """summary_tasks.csv 末尾的均值行（succeeded；误差列取绝对值均值）。"""
    row = {"file": "均值(succeeded)", "result": f"n={len(ok_rows)}",
           "goal_x": "", "goal_y": ""}
    for k in ["duration_s", "avg_speed_mps", "max_speed_mps",
              "passing_length_m", "replanning_count", "recoveries"] + \
            [f"{p}_s" for p in PHASES] + ["other_s"]:
        row[k] = round(mean([r[k] for r in ok_rows]), 4)
    for k in ["err_lon_m", "err_lat_m", "err_yaw_deg", "err_dist_m"]:
        row[k] = round(mean([abs(r[k]) for r in ok_rows]), 4)
    return row


def plot_summary_tasks(rows, stats, out_dir):
    ok = [r for r in rows if r["result"] == "succeeded"]
    idx = list(range(len(rows)))
    short = [r["file"].replace("nav_task_", "").replace(".json", "") for r in rows]
    fig = plt.figure(figsize=(max(12, len(rows) * 1.1), 20))
    gs = fig.add_gridspec(4, 1, height_ratios=[1.2, 1.3, 1.3, 1.3], hspace=0.55)
    ax_tab = fig.add_subplot(gs[0])
    ax_err = fig.add_subplot(gs[1])
    ax_time = fig.add_subplot(gs[2])
    ax_speed = fig.add_subplot(gs[3])

    # 表 0：总平均统计
    ax_tab.axis("off")
    cells = [[k, v] for k, v in stats.items()]
    tab = ax_tab.table(
        cellText=cells, colLabels=["指标（succeeded 口径）", "值"],
        cellLoc="left", colWidths=[0.42, 0.58], loc="center")
    tab.auto_set_font_size(False)
    tab.set_fontsize(10)
    tab.scale(1.0, 1.3)
    ax_tab.set_title("表 0：总平均统计", pad=18)

    # 图 1：每任务三维到达误差（纵/横带符号条 + |yaw| 散点）
    b1 = ax_err.bar([i - 0.2 for i in idx], [r["err_lon_m"] for r in rows], 0.38,
                    color="#1565c0", label="纵向误差 lon（正=过冲）")
    b2 = ax_err.bar([i + 0.2 for i in idx], [r["err_lat_m"] for r in rows], 0.38,
                    color="#e65100", label="横向误差 lat")
    label_bars(ax_err, b1, "{:+.3f}", signed=True)
    label_bars(ax_err, b2, "{:+.3f}", signed=True)
    ax_err.axhline(0, color="k", lw=0.8)
    ax2 = ax_err.twinx()
    ax2.plot(idx, [abs(r["err_yaw_deg"]) for r in rows], "k^", ms=6,
             label="|yaw 误差| (°)")
    for i, r in zip(idx, rows):
        ax2.annotate(f"{abs(r['err_yaw_deg']):.1f}°",
                     (i, abs(r["err_yaw_deg"])), ha="center", va="bottom",
                     fontsize=7)
    ax2.set_ylabel("|yaw 误差| (°)")
    ax_err.set_title(
        "图 1：每任务三维到达误差（纵向/横向为 goal 系带符号值；"
        "失败任务的横轴标签标红，误差为中途停止位置仅供参考）")
    ax_err.set_ylabel("误差 (m)")
    h1, l1 = ax_err.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax_err.legend(h1 + h2, l1 + l2, loc="best", fontsize=9)

    # 图 2：每任务执行时间（四阶段堆叠 + 总时长标值）
    bottom = [0.0] * len(rows)
    for p in PHASES + ["其他"]:
        key = f"{p}_s" if p in PHASES else "other_s"
        vals = [r[key] for r in rows]
        ax_time.bar(idx, vals, bottom=bottom, color=PHASE_COLORS[p], label=p)
        # 段内标值（段高足够才标，避免小段文字重叠）
        for i, v in enumerate(vals):
            if v >= 1.2:
                ax_time.annotate(f"{v:.1f}", (i, bottom[i] + v / 2),
                                 ha="center", va="center", fontsize=7)
        bottom = [b + v for b, v in zip(bottom, vals)]
    for i, tot in enumerate(bottom):
        ax_time.annotate(f"{tot:.1f}s", (i, tot), ha="center", va="bottom",
                         fontsize=8, fontweight="bold")
    if ok:
        m = mean([r["duration_s"] for r in ok])
        ax_time.axhline(m, color="#1565c0", ls="--", lw=1.2,
                        label=f"succeeded 总时长均值 {m:.1f} s")
    ax_time.set_title("图 2：每任务执行时间（四阶段堆叠，段内/顶部标值；"
                      "\"其他\"=无阶段回显覆盖的时间）")
    ax_time.set_ylabel("时间 (s)")
    ax_time.legend(loc="best", fontsize=9)

    # 图 3：每任务平均/最大速度
    b1 = ax_speed.bar([i - 0.2 for i in idx], [r["avg_speed_mps"] for r in rows],
                      0.4, color="#1565c0", label="avg（平滑后）")
    b2 = ax_speed.bar([i + 0.2 for i in idx], [r["max_speed_mps"] for r in rows],
                      0.4, color="#e65100", label="max（平滑后）")
    label_bars(ax_speed, b1)
    label_bars(ax_speed, b2)
    if ok:
        m = mean([r["avg_speed_mps"] for r in ok])
        ax_speed.axhline(m, color="k", ls="--", lw=1.2,
                         label=f"succeeded avg 均值 {m:.3f} m/s")
    ax_speed.set_title("图 3：每任务平均/最大线速度")
    ax_speed.set_ylabel("v (m/s)")
    ax_speed.legend(loc="best", fontsize=9)

    for ax in (ax_err, ax_time, ax_speed):
        ax.set_xticks(idx)
        ax.set_xticklabels(short, rotation=45, ha="right", fontsize=7)
        # 失败任务横轴标签标红
        for lbl, r in zip(ax.get_xticklabels(), rows):
            if r["result"] != "succeeded":
                lbl.set_color(RESULT_COLORS.get(r["result"], "#c62828"))
        ax.grid(True, axis="y", alpha=0.3)

    fig.savefig(os.path.join(out_dir, "summary_tasks.png"), dpi=130,
                bbox_inches="tight")
    plt.close(fig)


def build_point_rows(rows, centers, labels):
    """每点位聚合指标（精度/速度/阶段时间用 succeeded；次数含全部）。"""
    prows = []
    for i, (cx, cy) in enumerate(centers):
        all_i = [rows[j] for j in range(len(rows)) if labels[j] == i]
        ok = [r for r in all_i if r["result"] == "succeeded"]
        prows.append({
            "point": f"P{i+1}",
            "goal_x": round(cx, 3), "goal_y": round(cy, 3),
            "n_total": len(all_i), "n_success": len(ok),
            "success_rate": round(len(ok) / len(all_i), 3) if all_i else 0.0,
            "err_lon_mean_m": round(mean([abs(r["err_lon_m"]) for r in ok]), 4),
            "err_lon_max_m": round(max([abs(r["err_lon_m"]) for r in ok], default=0), 4),
            "err_lat_mean_m": round(mean([abs(r["err_lat_m"]) for r in ok]), 4),
            "err_lat_max_m": round(max([abs(r["err_lat_m"]) for r in ok], default=0), 4),
            "err_yaw_mean_deg": round(mean([abs(r["err_yaw_deg"]) for r in ok]), 2),
            "err_yaw_max_deg": round(max([abs(r["err_yaw_deg"]) for r in ok], default=0), 2),
            "avg_speed_mean_mps": round(mean([r["avg_speed_mps"] for r in ok]), 3),
            "duration_mean_s": round(mean([r["duration_s"] for r in ok]), 2),
            **{f"{p}_mean_s": round(mean([r[f"{p}_s"] for r in ok]), 2)
               for p in PHASES},
        })
    return prows


def plot_summary_points(prows, out_dir):
    xs = list(range(len(prows)))
    xlabels = [f"{p['point']}\n({p['goal_x']:.2f},{p['goal_y']:.2f})" for p in prows]
    fig, axes = plt.subplots(4, 1, figsize=(max(11, len(prows) * 2.2), 18))
    ax_err, ax_speed, ax_phase, ax_cnt = axes

    # 图 1：每点位三维误差（|lon|/|lat| mean+max 分组 + |yaw| mean）
    w = 0.19
    groups = [
        ("|lon| mean", "err_lon_mean_m", "#1565c0"),
        ("|lon| max", "err_lon_max_m", "#90caf9"),
        ("|lat| mean", "err_lat_mean_m", "#e65100"),
        ("|lat| max", "err_lat_max_m", "#ffcc80"),
    ]
    for k, (lab, key, color) in enumerate(groups):
        bars = ax_err.bar([x - 0.38 + w * (k + 0.5) for x in xs],
                          [p[key] for p in prows], w, color=color, label=lab)
        label_bars(ax_err, bars)
    ax2 = ax_err.twinx()
    ax2.plot(xs, [p["err_yaw_mean_deg"] for p in prows], "k^", ms=7,
             label="|yaw| mean (°)")
    for x, p in zip(xs, prows):
        ax2.annotate(f"{p['err_yaw_mean_deg']:.2f}°",
                     (x, p["err_yaw_mean_deg"]), ha="center", va="bottom",
                     fontsize=8)
    ax2.set_ylabel("|yaw 误差| (°)")
    ax_err.set_title(
        f"图 1：每点位三维到达误差（{GOAL_CLUSTER_TOL}m 聚类；仅统计 succeeded；"
        "纵向 lon=沿目标朝向，横向 lat=垂直方向）")
    ax_err.set_ylabel("误差 (m)")
    h1, l1 = ax_err.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax_err.legend(h1 + h2, l1 + l2, loc="best", fontsize=9, ncol=2)

    # 图 2：每点位平均速度
    bars = ax_speed.bar(xs, [p["avg_speed_mean_mps"] for p in prows],
                        0.5, color="#1565c0")
    label_bars(ax_speed, bars, fontsize=9)
    ax_speed.set_title("图 2：每点位平均速度（到达该点任务的 avg_speed 均值）")
    ax_speed.set_ylabel("v (m/s)")

    # 图 3：每点位各阶段平均时间 + 总时长
    cols = PHASES + ["总时长"]
    w = 0.8 / len(cols)
    for k, p in enumerate(cols):
        key = f"{p}_mean_s" if p in PHASES else "duration_mean_s"
        color = PHASE_COLORS.get(p, "#616161")
        bars = ax_phase.bar([x - 0.4 + w * (k + 0.5) for x in xs],
                            [pr[key] for pr in prows], w, color=color, label=p)
        label_bars(ax_phase, bars, "{:.2f}", fontsize=8)
    ax_phase.set_title("图 3：每点位各阶段平均时间 + 总时长")
    ax_phase.set_ylabel("时间 (s)")
    ax_phase.legend(loc="best", fontsize=9)

    # 图 4：每点位任务次数与成功率
    ax_cnt.bar(xs, [p["n_success"] for p in prows], 0.5,
               color="#2e7d32", label="成功")
    ax_cnt.bar(xs, [p["n_total"] - p["n_success"] for p in prows], 0.5,
               bottom=[p["n_success"] for p in prows],
               color="#c62828", label="失败/取消")
    for x, p in zip(xs, prows):
        ax_cnt.text(x, p["n_total"],
                    f"{p['n_success']}/{p['n_total']}  "
                    f"{p['success_rate']*100:.0f}%",
                    ha="center", va="bottom", fontsize=9)
    ax_cnt.set_title("图 4：每点位任务次数与成功率（成功数/总数 成功率）")
    ax_cnt.set_ylabel("次数")
    ax_cnt.legend(loc="best", fontsize=9)

    for ax in axes:
        ax.set_xticks(xs)
        ax.set_xticklabels(xlabels, fontsize=8)
        ax.grid(True, axis="y", alpha=0.3)

    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "summary_points.png"), dpi=130)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument(
        "dir", nargs="?", default=None,
        help="nav_task_*.json 所在目录（缺省=nav2_nav_monitor 配置的 output_dir）")
    ap.add_argument(
        "--no-per-task", action="store_true", help="跳过每任务的路径/速度图，只出汇总")
    args = ap.parse_args()

    d = resolve_data_dir(args.dir)
    if not os.path.isdir(d):
        sys.exit(f"数据目录不存在：{d}（还没有导航记录？或用位置参数指定目录）")
    tasks = load_tasks(d)
    if not tasks:
        sys.exit(f"目录中没有 nav_task_*.json：{d}")

    out_dir = os.path.join(d, "analysis_" + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)
    print(f"分析 {len(tasks)} 个导航任务 → {out_dir}")

    rows = build_rows(tasks)
    stats, ok_rows = overall_stats(rows)

    # ---- 第一部分：按任务横铺 + 总平均 ----
    with open(os.path.join(out_dir, "summary_tasks.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
        if ok_rows:
            w.writerow(average_row(ok_rows))
    plot_summary_tasks(rows, stats, out_dir)

    # ---- 第二部分：按导航点聚类合并 ----
    centers, labels = cluster_goals(tasks)
    prows = build_point_rows(rows, centers, labels)
    with open(os.path.join(out_dir, "summary_points.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(prows[0].keys()))
        w.writeheader()
        w.writerows(prows)
    plot_summary_points(prows, out_dir)
    print(f"已保存：summary_tasks.png/csv + summary_points.png/csv（{len(prows)} 个点位）")

    # ---- 每任务路径/速度图（复用兄弟脚本）----
    if not args.no_per_task:
        tasks_dir = os.path.join(out_dir, "tasks")
        os.makedirs(tasks_dir, exist_ok=True)
        here = os.path.dirname(os.path.abspath(__file__))
        for t in tasks:
            stem = os.path.splitext(t["_file"])[0]
            src = os.path.join(d, t["_file"])
            for script, suffix in (
                    ("plot_nav_task_path.py", "_path.png"),
                    ("plot_nav_task_speed.py", "_speed.png")):
                subprocess.run(
                    [sys.executable, os.path.join(here, script), src,
                     "--out", os.path.join(tasks_dir, stem + suffix)],
                    check=True, capture_output=True)
        print(f"已保存：{tasks_dir}/ 下 {len(tasks) * 2} 张每任务图")


if __name__ == "__main__":
    main()
