#!/usr/bin/env python3
# Copyright 2026 DEXE Robot Engineer
# SPDX-License-Identifier: Apache-2.0
"""速度跟踪调参证据图：controller_server 每控制周期记录的
期望速度（最近轨迹点规划速度）/ 下发指令 / odom 实测三速对照。

数据来源：speed_plan_log_dir 目录下的 speed_tracking_*.csv
（列：t,stage,desired_v,cmd_v,actual_v,cmd_wz,actual_wz）

用法：
  python3 plot_speed_tracking.py <csv> [--accel 0.6] [--decel 0.6] [--jerk 0.5]
                                 [--t-from S] [--t-to S] [--out xxx.png]

图 1：v-t 三速对照 + 控制阶段底色（align/track/approach 切换一眼可见）
      → 调 PID：看 actual 对 desired 的跟踪滞后/超调/稳态误差
图 2：跟踪误差（actual-desired / cmd-desired）
图 3：实测加速度（中心差分 + 滑窗平滑）+ accel/decel 限值线
      → 调加速度：实测是否贴着限值走、有没有超限尖峰
图 4：实测 jerk（加速度差分）+ 限值线 → 调 jerk（S 型）
图 5：wz 指令 vs 实测 → 旋转环 PID
"""
import argparse
import csv

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
matplotlib.rcParams["font.sans-serif"] = [
    "Noto Sans CJK SC", "WenQuanYi Micro Hei", "AR PL UMing CN", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

STAGE_COLORS = {
    "prepare": "#ffd54f", "align": "#64b5f6", "track": "#81c784",
    "approach": "#f48fb1", "unset": "#e0e0e0",
}
# stage 列兼容：新格式 "阶段:控制器"（直接命中）；旧格式为控制器名，按别名映射
STAGE_ALIASES = {"mppi": "track", "pid": "approach", "rotate": "align"}


def smooth(vals, win):
    if win <= 1:
        return list(vals)
    out = []
    n = len(vals)
    for i in range(n):
        lo = max(0, i - win // 2)
        hi = min(n, i + win // 2 + 1)
        out.append(sum(vals[lo:hi]) / (hi - lo))
    return out


def finite_diff(t, v):
    """中心差分（端点用单侧）。"""
    n = len(v)
    d = [0.0] * n
    for i in range(n):
        lo = max(0, i - 1)
        hi = min(n - 1, i + 1)
        dt = t[hi] - t[lo]
        d[i] = (v[hi] - v[lo]) / dt if dt > 1e-6 else 0.0
    return d


def stage_key(stage):
    s = stage.lower()
    for key in STAGE_COLORS:
        if key in s:
            return key
    for alias, key in STAGE_ALIASES.items():
        if alias in s:
            return key
    return "unset"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv_file")
    ap.add_argument("--accel", type=float, default=None, help="加速度限值参考线 (m/s²)")
    ap.add_argument("--decel", type=float, default=None, help="减速度限值参考线（正值）")
    ap.add_argument("--jerk", type=float, default=None, help="jerk 限值参考线 (m/s³)")
    ap.add_argument("--smooth-win", type=int, default=5, help="加速度平滑窗（采样点数）")
    ap.add_argument("--t-from", type=float, default=None)
    ap.add_argument("--t-to", type=float, default=None)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    rows = []
    with open(args.csv_file) as f:
        for row in csv.DictReader(l for l in f if not l.startswith("#")):
            rows.append(row)
    if not rows:
        raise SystemExit("empty csv")

    t0 = float(rows[0]["t"])
    t, stage, dv, cv, av, cwz, awz = [], [], [], [], [], [], []
    for r in rows:
        ti = float(r["t"]) - t0
        if args.t_from is not None and ti < args.t_from:
            continue
        if args.t_to is not None and ti > args.t_to:
            continue
        t.append(ti)
        stage.append(r["stage"])
        dv.append(float(r["desired_v"]))
        cv.append(float(r["cmd_v"]))
        av.append(float(r["actual_v"]))
        cwz.append(float(r["cmd_wz"]))
        awz.append(float(r["actual_wz"]))
    if len(t) < 3:
        raise SystemExit("too few samples in range")

    acc = smooth(finite_diff(t, av), args.smooth_win)
    jerk = smooth(finite_diff(t, acc), args.smooth_win)

    fig, axes = plt.subplots(5, 1, figsize=(14, 15), sharex=True)

    # 阶段底色（各面板共用）
    def shade(ax):
        seg_start = 0
        seen = set()
        for i in range(1, len(t) + 1):
            if i == len(t) or stage_key(stage[i]) != stage_key(stage[seg_start]):
                key = stage_key(stage[seg_start])
                ax.axvspan(t[seg_start], t[min(i, len(t) - 1)], alpha=0.30,
                           color=STAGE_COLORS[key],
                           label=key if key not in seen else None)
                seen.add(key)
                seg_start = i

    ax = axes[0]
    shade(ax)
    ax.plot(t, dv, "-", color="#0033cc", linewidth=1.8, label="期望 desired_v（规划）")
    ax.plot(t, cv, "-", color="#ff8c00", linewidth=0.9, alpha=0.85, label="指令 cmd_v")
    ax.plot(t, av, "-", color="#cc0000", linewidth=1.6, label="实测 actual_v（odom）")
    ax.set_ylabel("v (m/s)")
    ax.set_title("三速对照（底色=控制阶段）：actual 贴 desired 看 PID，desired 形状看规划")
    ax.legend(fontsize=8, ncol=3)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    shade(ax)
    ax.plot(t, [a - d for a, d in zip(av, dv)], "-", color="#cc0000",
            linewidth=1.0, label="actual - desired（跟踪误差）")
    ax.plot(t, [c - d for c, d in zip(cv, dv)], "-", color="#7b1fa2",
            linewidth=0.9, alpha=0.7, label="cmd - desired（控制器输出偏差）")
    ax.axhline(0, color="gray", linewidth=0.8)
    ax.set_ylabel("Δv (m/s)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[2]
    shade(ax)
    ax.plot(t, acc, "-", color="#00695c", linewidth=1.4,
            label=f"实测加速度（平滑窗 {args.smooth_win}）")
    if args.accel:
        ax.axhline(args.accel, color="#1f77b4", linestyle="--", linewidth=0.9,
                   label=f"accel 限值 {args.accel}")
    if args.decel:
        ax.axhline(-abs(args.decel), color="#d62728", linestyle="--", linewidth=0.9,
                   label=f"decel 限值 -{abs(args.decel)}")
    ax.set_ylabel("a (m/s²)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[3]
    shade(ax)
    ax.plot(t, jerk, "-", color="#4e342e", linewidth=1.2, label="实测 jerk")
    if args.jerk:
        ax.axhline(args.jerk, color="#1f77b4", linestyle="--", linewidth=0.9)
        ax.axhline(-args.jerk, color="#d62728", linestyle="--", linewidth=0.9,
                   label=f"jerk 限值 ±{args.jerk}")
    ax.set_ylabel("j (m/s³)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[4]
    shade(ax)
    ax.plot(t, cwz, "-", color="#ff8c00", linewidth=0.9, alpha=0.85, label="指令 wz")
    ax.plot(t, awz, "-", color="#cc0000", linewidth=1.5, label="实测 wz")
    ax.set_ylabel("wz (rad/s)")
    ax.set_xlabel("t (s)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    fig.suptitle(f"speed tracking: {args.csv_file}（{len(t)} 周期）", fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    out = args.out or args.csv_file.rsplit(".", 1)[0] + ".png"
    fig.savefig(out, dpi=110)
    print(f"saved: {out}")


if __name__ == "__main__":
    main()
