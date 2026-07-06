#!/usr/bin/env python3
# Copyright 2026 DEXE Robot Engineer
# SPDX-License-Identifier: Apache-2.0
"""绘制 T/S 速度规划剖面曲线。

用法：
  1. 先跑测试导出 CSV：./build/nav2_speed_planner/test_speed_profiles
     （CSV 输出到 /tmp/speed_plan_test/）
  2. python3 plot_speed_profiles.py [--dir /tmp/speed_plan_test] [--out speed_profiles.png]

第 1 行：T vs S 各距离的 v-s 曲线对照（注意：恒加速度在弧长域是 √ 曲线，
        v(s)=√(2as)，斜率 dv/ds=a/v 随速度增大变缓——"T 型"直线只出现在时间域）。
第 2 行：时间域 v-t 验证图（T 型应为直线三角/梯形）+ 障碍场景。
"""
import argparse
import csv
import glob
import os
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
matplotlib.rcParams["font.sans-serif"] = [
    "Noto Sans CJK SC", "WenQuanYi Micro Hei", "AR PL UMing CN", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

STOP_DISTANCE = 0.2
END_SPEED = 0.03


def load_csv(path):
    s, v = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            s.append(float(row["s"]))
            v.append(float(row["v"]))
    return s, v


def integrate_time(s, v):
    """由 v-s 数值积分时间轴：dt = ds / v_avg（时间域验证用）。"""
    t = [0.0]
    for i in range(1, len(s)):
        ds = s[i] - s[i - 1]
        v_avg = max((v[i] + v[i - 1]) / 2.0, 1e-3)
        t.append(t[-1] + ds / v_avg)
    return t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default="/tmp/speed_plan_test")
    ap.add_argument("--out", default="/tmp/speed_plan_test/speed_profiles.png")
    args = ap.parse_args()

    dist_files = {}      # {length: {planner: path}}
    obstacle_files = {}  # {tag: path}
    curve_files = {}     # {planner: path}（2m 处 0.5m 弧段场景，T/S 同面板对照）
    for p in sorted(glob.glob(os.path.join(args.dir, "*.csv"))):
        name = os.path.basename(p)
        m = re.match(r"([TS])_(\d+\.\d+)m\.csv$", name)
        if m:
            dist_files.setdefault(float(m.group(2)), {})[m.group(1)] = p
            continue
        m = re.match(r"([TS])_curve2_(\d+\.\d+)m\.csv$", name)
        if m:
            curve_files[m.group(1)] = p
            continue
        m = re.match(r"([TS]_(?:start_obstacle|mid_obstacle|obs3))_(\d+\.\d+)m\.csv$", name)
        if m:
            obstacle_files[m.group(1)] = p
    if not dist_files:
        raise SystemExit(f"no CSV found in {args.dir} — run test_speed_profiles first")

    lengths = sorted(dist_files)
    n_cols = max(len(lengths), 5)
    fig, axes = plt.subplots(2, n_cols, figsize=(3.9 * n_cols, 7.4), squeeze=False)

    # ---- 第 1 行：各距离 v-s（弧长域）----
    for col, length in enumerate(lengths):
        ax = axes[0][col]
        for planner, style, color in (("T", "-o", "#1f77b4"), ("S", "-s", "#d62728")):
            if planner in dist_files[length]:
                s, v = load_csv(dist_files[length][planner])
                ax.plot(s, v, style, color=color, markersize=2.0, linewidth=1.1,
                        label=f"{planner} 型")
        ax.axvspan(max(length - STOP_DISTANCE, 0), length, alpha=0.12, color="orange",
                   label="蠕行区")
        ax.axhline(END_SPEED, color="gray", linewidth=0.8, linestyle="--")
        ax.set_title(f"{length:.2f} m（v-s 弧长域）", fontsize=10)
        ax.set_xlabel("s (m)")
        ax.set_ylabel("v (m/s)")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)
    for col in range(len(lengths), n_cols):
        axes[0][col].axis("off")

    # ---- 第 2 行：v-t 时间域验证 + 障碍场景 ----
    row2 = []
    # 面板 1：v-t 时间域（取最长的常规距离）——恒加速度应为直线
    vt_len = lengths[-1]
    ax = axes[1][0]
    for planner, style, color in (("T", "-o", "#1f77b4"), ("S", "-s", "#d62728")):
        if planner in dist_files[vt_len]:
            s, v = load_csv(dist_files[vt_len][planner])
            t = integrate_time(s, v)
            ax.plot(t, v, style, color=color, markersize=2.0, linewidth=1.1,
                    label=f"{planner} 型")
    ax.set_title(f"{vt_len:.2f} m（v-t 时间域）：T 型为直线三角/梯形", fontsize=10)
    ax.set_xlabel("t (s)")
    ax.set_ylabel("v (m/s)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7)
    row2.append(0)

    panels = [
        ("T_start_obstacle", "T：起点在障碍内切区（豁免起步）", [(0.0, 0.10, "障碍(豁免)")]),
        ("T_mid_obstacle", "T：中途障碍 x≥0.70", [(0.5, 0.7, "缓冲带"), (0.7, 1.0, "障碍区")]),
        ("T_obs3", "T：5m 路径、3m 处障碍", [(2.8, 3.0, "缓冲带"), (3.0, 5.0, "障碍区")]),
        ("S_obs3", "S：5m 路径、3m 处障碍", [(2.8, 3.0, "缓冲带"), (3.0, 5.0, "障碍区")]),
    ]
    col = 1
    for tag, title, spans in panels:
        if tag not in obstacle_files or col >= n_cols:
            continue
        ax = axes[1][col]
        s, v = load_csv(obstacle_files[tag])
        color = "#2ca02c" if tag.startswith("T") else "#d62728"
        ax.plot(s, v, "-o", color=color, markersize=2.0, linewidth=1.1)
        for x0, x1, lbl in spans:
            ax.axvspan(x0, min(x1, max(s)), alpha=0.15 if "缓冲" in lbl or "豁免" in lbl else 0.25,
                       color="red", label=lbl)
        ax.set_title(title, fontsize=10)
        ax.set_xlabel("s (m)")
        ax.set_ylabel("v (m/s)")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)
        col += 1

    # 弯道面板：5m 路径、2m 处 0.5m 弧段（r=0.5），期望弯内 v = 1.0·0.5/1.2 ≈ 0.417
    if curve_files and col < n_cols:
        ax = axes[1][col]
        for planner, color in (("T", "#1f77b4"), ("S", "#d62728")):
            if planner in curve_files:
                s, v = load_csv(curve_files[planner])
                ax.plot(s, v, "-o", color=color, markersize=2.0, linewidth=1.1,
                        label=f"{planner} 型")
        ax.axvspan(2.0, 2.5, alpha=0.2, color="purple", label="弧段 r=0.5")
        ax.axhline(1.0 * 0.5 / 1.2, color="purple", linewidth=0.9, linestyle="--",
                   label="曲率限速 0.417")
        ax.set_title("弯道：5m 路径、2m 处 0.5m 弧段", fontsize=10)
        ax.set_xlabel("s (m)")
        ax.set_ylabel("v (m/s)")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)
        col += 1
    for c in range(col, n_cols):
        axes[1][c].axis("off")

    fig.suptitle("nav2_speed_planner 剖面测试（max_speed=1.0, accel=0.6, decel=-0.6, "
                 "end_speed=0.03, stop_distance=0.2）", fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(args.out, dpi=105)
    print(f"saved: {args.out}")


if __name__ == "__main__":
    main()
