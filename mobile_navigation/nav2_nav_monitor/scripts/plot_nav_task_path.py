#!/usr/bin/env python3
# Copyright 2026 DEXE Robot Engineer
# SPDX-License-Identifier: Apache-2.0
"""导航任务路径回溯图：nav2_nav_monitor 落盘的 nav_task_*.json 的
规划路径（含每次重规划）与实际移动轨迹地图视图对照。

数据来源：nav2_nav_monitor output_dir（默认 ~/.dexe/navigation/nav_monitor）
下的 nav_task_<时间戳>_<结果>.json

用法：
  python3 plot_nav_task_path.py <json 或目录> [--out xxx.png]
  （传目录时自动取最新一个 nav_task_*.json）

图 1：地图视图——规划路径 ×N（首条深色、重规划渐淡）+ 实际轨迹（按时间
      渐变色）+ 起点/目标/到达点标注 + 到达误差注记
      → 看走了哪条路、重规划改道了几次、终点差了多少
图 2：横向偏差-时间——实际轨迹各采样点到首条规划路径的最近距离
      → 看跟踪贴合度随时间的变化、哪一段偏得最远
"""
import argparse
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
matplotlib.rcParams["font.sans-serif"] = [
    "Noto Sans CJK SC", "WenQuanYi Micro Hei", "AR PL UMing CN", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

from nav_task_common import resolve_data_dir


def resolve_json(path):
    if not os.path.exists(path):
        sys.exit(f"路径不存在：{path}（还没有导航记录？或用位置参数指定目录/文件）")
    if os.path.isdir(path):
        cands = sorted(
            f for f in os.listdir(path)
            if f.startswith("nav_task_") and f.endswith(".json"))
        if not cands:
            sys.exit(f"目录中没有 nav_task_*.json：{path}")
        return os.path.join(path, cands[-1])
    return path


PHASE_COLORS = {
    "prepare": "#ffd54f", "align": "#64b5f6", "track": "#81c784",
    "approach": "#f48fb1",
}


def pose_at_time(t, passing):
    """时间→实走轨迹位置线性插值（passing 每项 [t,x,y,yaw]）。"""
    if not passing:
        return None
    if t <= passing[0][0]:
        return passing[0][1], passing[0][2]
    for i in range(1, len(passing)):
        if t <= passing[i][0]:
            a, b = passing[i - 1], passing[i]
            r = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
            return a[1] + r * (b[1] - a[1]), a[2] + r * (b[2] - a[2])
    return passing[-1][1], passing[-1][2]


def nearest_dist(px, py, path_xy):
    """点到折线各顶点的最近距离（顶点级，够用；不做线段投影）。"""
    return min(math.hypot(px - x, py - y) for x, y in path_xy)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument(
        "json", nargs="?", default=None,
        help="nav_task_*.json 文件或其所在目录（缺省=nav2_nav_monitor 配置的 "
             "output_dir，取其中最新一个）")
    ap.add_argument("--out", default=None, help="输出 png（默认与 json 同目录同名）")
    args = ap.parse_args()

    jf = resolve_json(resolve_data_dir(args.json))
    with open(jf) as f:
        d = json.load(f)

    passing = d.get("passing_path", [])   # [t,x,y,yaw]
    planned = d.get("planned_paths", [])  # 每条 [[x,y,yaw],...]
    start = d.get("start_pose", {})
    goal = d.get("goal_pose", {})
    reached = d.get("goal_reached_pose", {})
    err = d.get("goal_reached_error", {})

    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(10, 12), gridspec_kw={"height_ratios": [3, 1]})

    # ---- 图 1：地图视图 ----
    for i, path in enumerate(planned):
        if not path:
            continue
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        alpha = 0.9 if i == 0 else max(0.25, 0.7 - 0.1 * i)
        label = "规划路径(首条)" if i == 0 else ("重规划" if i == 1 else None)
        ax1.plot(xs, ys, "--", color="#1565c0", alpha=alpha, lw=1.5, label=label)

    if len(passing) >= 2:
        ts = [p[0] for p in passing]
        pts = [(p[1], p[2]) for p in passing]
        segs = [[pts[i], pts[i + 1]] for i in range(len(pts) - 1)]
        lc = LineCollection(segs, cmap="viridis", linewidths=2.5)
        lc.set_array(matplotlib.colors.Normalize()(ts))
        ax1.add_collection(lc)
        cbar = fig.colorbar(lc, ax=ax1, fraction=0.03, pad=0.02)
        cbar.set_label("时间进度（归一化）")
        ax1.plot([], [], color="#2e7d32", lw=2.5, label="实际轨迹（色=时间）")

    if start:
        ax1.plot(start["x"], start["y"], "o", color="#2e7d32", ms=10, label="起点")
    if goal:
        ax1.plot(goal["x"], goal["y"], "*", color="#c62828", ms=16, label="目标")
        yaw = goal.get("yaw", 0.0)
        ax1.annotate(
            "", xy=(goal["x"] + 0.3 * math.cos(yaw), goal["y"] + 0.3 * math.sin(yaw)),
            xytext=(goal["x"], goal["y"]),
            arrowprops=dict(arrowstyle="->", color="#c62828"))
    if reached:
        ax1.plot(reached["x"], reached["y"], "x", color="#e65100", ms=12,
                 mew=3, label="到达点")

    # 阶段切换点标注：切换时刻在实走轨迹上的位置
    transitions = d.get("phase_transitions", [])
    marked = set()
    for t_tr, name in transitions:
        pos = pose_at_time(t_tr, passing)
        if pos is None:
            continue
        c = PHASE_COLORS.get(name, "#616161")
        ax1.plot(pos[0], pos[1], "o", color=c, ms=9,
                 mec="k", mew=0.8, zorder=5,
                 label=None if name in marked else f"进入 {name}")
        ax1.annotate(f"{name}@{t_tr:.1f}s", pos, textcoords="offset points",
                     xytext=(6, 6), fontsize=7, color=c)
        marked.add(name)

    result = d.get("result", "?")
    ax1.set_title(
        f"导航任务路径回溯  result={result}  重规划={d.get('replanning_count', 0)} 次  "
        f"recovery={d.get('number_of_recoveries', 0)} 次\n"
        f"到达误差: dist={err.get('distance', 0):.3f}m  "
        f"(x={err.get('x', 0):.3f}, y={err.get('y', 0):.3f}, "
        f"yaw={math.degrees(err.get('yaw', 0)):.1f}°)   "
        f"规划长度={d.get('planned_path_length_first', 0):.2f}m  "
        f"实走={d.get('passing_length', 0):.2f}m")
    ax1.set_xlabel("x (m)")
    ax1.set_ylabel("y (m)")
    ax1.set_aspect("equal", adjustable="datalim")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="best", fontsize=9)

    # ---- 图 2：横向偏差-时间 ----
    if planned and planned[0] and len(passing) >= 2:
        ref = [(p[0], p[1]) for p in planned[0]]
        ts = [p[0] for p in passing]
        devs = [nearest_dist(p[1], p[2], ref) for p in passing]
        ax2.plot(ts, devs, color="#ad1457", lw=1.5)
        # 阶段底色
        for i, (t_tr, name) in enumerate(transitions):
            t_end = transitions[i + 1][0] if i + 1 < len(transitions) else ts[-1]
            ax2.axvspan(t_tr, t_end, color=PHASE_COLORS.get(name, "#e0e0e0"),
                        alpha=0.18, lw=0)
        ax2.set_title("实际轨迹相对首条规划路径的最近距离（顶点级近似；底色=阶段）")
        ax2.set_xlabel("t (s，相对任务开始)")
        ax2.set_ylabel("偏差 (m)")
        ax2.grid(True, alpha=0.3)
    else:
        ax2.text(0.5, 0.5, "无规划路径或轨迹数据，跳过偏差图",
                 ha="center", va="center", transform=ax2.transAxes)
        ax2.set_axis_off()

    fig.tight_layout()
    out = args.out or os.path.splitext(jf)[0] + "_path.png"
    fig.savefig(out, dpi=130)
    print(f"已保存：{out}")


if __name__ == "__main__":
    main()
