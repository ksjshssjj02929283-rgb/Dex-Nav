#!/usr/bin/env python3
# Copyright 2026 DEXE Robot Engineer
# SPDX-License-Identifier: Apache-2.0
"""导航任务速度回溯图：nav2_nav_monitor 落盘的 nav_task_*.json 的
规划状态、控制器轨迹目标、控制器输出、平滑后与 odom 实测的 V-T / V-S
曲线对照，并单独显示控制器目标加速度。

数据来源：nav2_nav_monitor output_dir（默认 ~/.dexe/navigation/nav_monitor）
下的 nav_task_<时间戳>_<结果>.json（samples 每项 [t,vx,wz]，t 相对任务开始）

用法：
  python3 plot_nav_task_speed.py <json 或目录> [--out xxx.png]
  （传目录时自动取最新一个 nav_task_*.json）

图 1：线速度 V-T——期望/控制器输出/平滑后/odom 实测四线叠画
      → 实测贴期望的程度=跟踪质量；平滑器削峰与滞后一眼可见
图 2：角速度 W-T——控制器输出/平滑后/odom 实测（期望无角速度定义不画）
图 3：线速度 V-S——横轴换为沿实走轨迹的累计里程（由 passing_path 的
      [t,x,y] 累计距离按时间插值映射）
      → 看"哪一段路快哪一段路慢"，对照弯道/障碍位置定位减速原因
图 4：角速度 W-S——同上
"""
import argparse
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
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


def cumulative_s(passing):
    """passing_path [t,x,y,yaw] → (ts, ss) 累计里程曲线。"""
    ts, ss = [], []
    s = 0.0
    for i, p in enumerate(passing):
        if i > 0:
            q = passing[i - 1]
            s += math.hypot(p[1] - q[1], p[2] - q[2])
        ts.append(p[0])
        ss.append(s)
    return ts, ss


def t_to_s(t, ts, ss):
    """时间 → 里程线性插值（越界钳位到端点）。"""
    if not ts:
        return None
    if t <= ts[0]:
        return ss[0]
    if t >= ts[-1]:
        return ss[-1]
    for i in range(1, len(ts)):
        if t <= ts[i]:
            r = (t - ts[i - 1]) / (ts[i] - ts[i - 1]) if ts[i] > ts[i - 1] else 0.0
            return ss[i - 1] + r * (ss[i] - ss[i - 1])
    return ss[-1]


# (json键, 图例名, 颜色, 线宽, 线型, 是否画角速度面板)
PHASE_COLORS = {
    "prepare": "#ffd54f", "align": "#64b5f6", "track": "#81c784",
    "approach": "#f48fb1",
}


def phase_spans(transitions, total):
    """[[t,阶段],...] → [(t0,t1,阶段)]，相邻同名合并。"""
    spans = []
    for i, (t0, name) in enumerate(transitions):
        t1 = transitions[i + 1][0] if i + 1 < len(transitions) else total
        if spans and spans[-1][2] == name:
            spans[-1] = (spans[-1][0], t1, name)
        else:
            spans.append((t0, t1, name))
    return spans


SOURCES = [
    ("desired_vel", "规划状态(最近点)", "#212121", 1.8, "--", False),
    ("cmd_vel", "控制器输出", "#1565c0", 1.0, "-", True),
    ("smoothed_vel", "平滑后", "#e65100", 1.4, "-", True),
    ("odom_vel", "odom实测", "#2e7d32", 1.4, "-", True),
]


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

    ts_path, ss_path = cumulative_s(d.get("passing_path", []))
    have_s = len(ts_path) >= 2

    fig, axes = plt.subplots(5, 1, figsize=(12, 17))
    ax_vt, ax_wt, ax_at, ax_vs, ax_ws = axes

    # 阶段底色：时间轴面板直接铺，里程轴面板经 t→s 映射铺
    total_t = d.get("end_time", 0) - d.get("start_time", 0)
    spans = phase_spans(d.get("phase_transitions", []), total_t)
    seen_phases = []
    for t0, t1, name in spans:
        c = PHASE_COLORS.get(name, "#e0e0e0")
        for ax in (ax_vt, ax_wt, ax_at):
            ax.axvspan(t0, t1, color=c, alpha=0.18, lw=0)
        if have_s:
            s0, s1 = t_to_s(t0, ts_path, ss_path), t_to_s(t1, ts_path, ss_path)
            for ax in (ax_vs, ax_ws):
                ax.axvspan(s0, s1, color=c, alpha=0.18, lw=0)
        if name not in seen_phases:
            seen_phases.append(name)
    phase_handles = [Patch(color=PHASE_COLORS.get(n, "#e0e0e0"), alpha=0.4, label=n)
                     for n in seen_phases]

    for key, label, color, lw, ls, with_wz in SOURCES:
        block = d.get(key, {})
        samples = block.get("samples", [])
        if not samples:
            continue
        t = [s[0] for s in samples]
        vx = [s[1] for s in samples]
        wz = [s[2] for s in samples]
        stat = (f"{label}（avg={block.get('avg_linear_speed', 0):.3f} "
                f"max={block.get('max_linear_speed', 0):.3f}）")
        ax_vt.plot(t, vx, color=color, lw=lw, ls=ls, label=stat)
        if with_wz:
            ax_wt.plot(t, wz, color=color, lw=lw, ls=ls, label=label)
        if have_s:
            s_axis = [t_to_s(x, ts_path, ss_path) for x in t]
            ax_vs.plot(s_axis, vx, color=color, lw=lw, ls=ls, label=label)
            if with_wz:
                ax_ws.plot(s_axis, wz, color=color, lw=lw, ls=ls, label=label)

    target = d.get("controller_target", {})
    target_samples = target.get("samples", [])
    if target_samples:
        target_t = [sample[0] for sample in target_samples]
        target_v = [sample[1] for sample in target_samples]
        target_a = [sample[2] for sample in target_samples]
        ax_vt.plot(target_t, target_v, color="#7b1fa2", lw=1.6,
                   label="控制器轨迹目标")
        ax_at.plot(target_t, target_a, color="#7b1fa2", lw=1.4,
                   label="控制器目标加速度")
        ax_at.axhline(0.0, color="gray", lw=0.8)
        if have_s:
            target_s = [t_to_s(x, ts_path, ss_path) for x in target_t]
            ax_vs.plot(target_s, target_v, color="#7b1fa2", lw=1.6,
                       label="控制器轨迹目标")
    else:
        ax_at.text(0.5, 0.5, "旧记录无 controller_target 数据",
                   ha="center", va="center", transform=ax_at.transAxes)

    result = d.get("result", "?")
    total_t = d.get("end_time", 0) - d.get("start_time", 0)
    ax_vt.set_title(
        f"导航任务速度回溯  result={result}  总时长={total_t:.1f}s  "
        f"实走={d.get('passing_length', 0):.2f}m\n图 1：线速度 V-T")
    ax_vt.set_ylabel("vx (m/s)")
    ax_wt.set_title("图 2：角速度 W-T")
    ax_wt.set_ylabel("wz (rad/s)")
    ax_at.set_title("图 3：控制器轨迹目标加速度 A-T")
    ax_at.set_ylabel("a (m/s²)")
    for ax in (ax_vt, ax_wt, ax_at):
        ax.set_xlabel("t (s，相对任务开始)")

    if have_s:
        ax_vs.set_title("图 4：线速度 V-S（横轴=沿实走轨迹累计里程）")
        ax_vs.set_ylabel("vx (m/s)")
        ax_ws.set_title("图 5：角速度 W-S")
        ax_ws.set_ylabel("wz (rad/s)")
        for ax in (ax_vs, ax_ws):
            ax.set_xlabel("s (m)")
    else:
        for ax in (ax_vs, ax_ws):
            ax.text(0.5, 0.5, "passing_path 采样不足，跳过 V-S 图",
                    ha="center", va="center", transform=ax.transAxes)
            ax.set_axis_off()

    for ax in axes:
        if ax.has_data():
            ax.grid(True, alpha=0.3)
            h, l = ax.get_legend_handles_labels()
            ax.legend(handles=h + phase_handles, loc="best", fontsize=8, ncol=2)

    fig.tight_layout()
    out = args.out or os.path.splitext(jf)[0] + "_speed.png"
    fig.savefig(out, dpi=130)
    print(f"已保存：{out}")


if __name__ == "__main__":
    main()
