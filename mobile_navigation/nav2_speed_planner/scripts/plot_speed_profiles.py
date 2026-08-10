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


def load_csv_full(path):
    """全列读取（新 CSV 含 accel/jerk/rel_time/region，旧 CSV 自动缺省）。"""
    cols = {"s": [], "v": [], "accel": [], "jerk": [], "t": [], "region": []}
    with open(path) as f:
        for row in csv.DictReader(f):
            cols["s"].append(float(row["s"]))
            cols["v"].append(float(row["v"]))
            cols["accel"].append(float(row.get("accel", 0) or 0))
            cols["jerk"].append(float(row.get("jerk", 0) or 0))
            cols["t"].append(float(row.get("rel_time", 0) or 0))
            cols["region"].append(row.get("region", "") or "")
    return cols


REGION_COLORS = {
    "obstacle": "#ef9a9a", "obstacle_buffer": "#ffcc80",
    "obstacle_exempt": "#c5e1a5", "curve": "#b39ddb", "stop_zone": "#90caf9",
}


def render_detail(csv_path, out_dir, accel=0.6, decel=-0.6):
    """渲染单场景四联详图。空 CSV（期望失败的场景）直接跳过，返回 None。"""
    """单场景四联详图：v-s(区域底色) / a-s(限值线) / jerk-s / v-t。"""
    d = load_csv_full(csv_path)
    name = os.path.splitext(os.path.basename(csv_path))[0]
    if not d["s"]:
        return None   # 期望返回空计划的场景（D_empty_path / F_*_fail_closed）
    fig, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=False)
    ax_v, ax_a, ax_j, ax_t = axes

    # region 底色（v-s 面板）
    seen = {}
    i = 0
    n = len(d["s"])
    while i < n:
        r = d["region"][i]
        j = i
        while j + 1 < n and d["region"][j + 1] == r:
            j += 1
        if r in REGION_COLORS:
            ax_v.axvspan(d["s"][i], d["s"][j], color=REGION_COLORS[r],
                         alpha=0.35, lw=0)
            seen[r] = True
        i = j + 1
    ax_v.plot(d["s"], d["v"], "-o", ms=2, lw=1.2, color="#1565c0")
    ax_v.set_title(f"{name}  图1: v-s（底色=region 标记）")
    ax_v.set_ylabel("v (m/s)")
    from matplotlib.patches import Patch
    if seen:
        ax_v.legend(handles=[Patch(color=REGION_COLORS[r], alpha=0.5, label=r)
                             for r in seen], fontsize=8)

    ax_a.plot(d["s"], d["accel"], "-o", ms=2, lw=1.0, color="#e65100")
    ax_a.axhline(accel, color="k", ls="--", lw=0.8)
    ax_a.axhline(decel, color="k", ls="--", lw=0.8)
    ax_a.set_title("图2: accel-s（虚线=accel/decel 配置限值）")
    ax_a.set_ylabel("a (m/s²)")

    # 隐含 jerk：da/dt 数值差分（不依赖 jerk 字段——S 型未回填该字段，
    # 且弧长域 da/ds=jerk/v 会随速度改变斜率，时间域差分才是合规判据）
    ts = d["t"] if any(x > 0 for x in d["t"]) else integrate_time(d["s"], d["v"])
    js, jv = [], []
    for i in range(1, len(ts)):
        dt = ts[i] - ts[i - 1]
        if dt > 1e-6:
            js.append((d["s"][i] + d["s"][i - 1]) / 2)
            jv.append((d["accel"][i] - d["accel"][i - 1]) / dt)
    ax_j.plot(js, jv, "-o", ms=2, lw=1.0, color="#6a1b9a")
    ax_j.set_title("图3: 隐含 jerk-s（da/dt 数值差分；T 型为脉冲、S 型应 ≤ max_jerk）")
    ax_j.set_ylabel("jerk (m/s³)")

    ax_t.plot(ts, d["v"], "-o", ms=2, lw=1.2, color="#2e7d32", label="v")
    ax_t2 = ax_t.twinx()
    ax_t2.plot(ts, d["accel"], "-s", ms=2, lw=1.0, color="#e65100", label="a")
    ax_t2.set_ylabel("a (m/s²)")
    ax_t.set_title("图4: v-t 与 a-t（时间域：恒 jerk 的 a 应为对称线性斜坡）")
    ax_t.set_xlabel("t (s)")
    ax_t.set_ylabel("v (m/s)")
    h1, l1 = ax_t.get_legend_handles_labels()
    h2, l2 = ax_t2.get_legend_handles_labels()
    ax_t.legend(h1 + h2, l1 + l2, fontsize=8)

    for ax in axes[:3]:
        ax.set_xlabel("s (m)")
    for ax in axes:
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    out = os.path.join(out_dir, name + ".png")
    fig.savefig(out, dpi=120)
    plt.close(fig)
    return out


def find_csvs(root, pattern="*.csv"):
    """递归收集 CSV。

    剖面按后端分子目录存放（csv/{trapezoidal,scurve,reachability}/），
    平铺三百多个文件时肉眼分不清是哪个规划器的产物。此处递归 glob，
    同时兼容历史的平铺布局。
    """
    return sorted(glob.glob(os.path.join(root, "**", pattern), recursive=True))


def main():
    ap = argparse.ArgumentParser()
    default_root = os.path.expanduser("~/.dexe/navigation/speed_planner")
    ap.add_argument("--dir", default=os.path.join(default_root, "csv"))
    ap.add_argument("--out", default=os.path.join(default_root, "speed_profiles.png"))
    ap.add_argument("--no-detail", action="store_true", help="跳过每场景四联详图")
    args = ap.parse_args()
    args.dir = os.path.expanduser(args.dir)
    args.out = os.path.expanduser(args.out)

    dist_files = {}      # {length: {planner: path}}
    obstacle_files = {}  # {tag: path}
    curve_files = {}     # {planner: path}（2m 处 0.5m 弧段场景，T/S 同面板对照）
    for p in find_csvs(args.dir):
        name = os.path.basename(p)
        # 场景 tag 形如 <场景描述>__<后端>_<长度>m.csv，后端 ∈ trapz/scurve/reach
        # （scurve_prod 是生产参数版，不进总览对照面板，避免与离线参数版混画）
        m = re.match(r"straight__(trapz|scurve|reach)_(\d+\.\d+)m\.csv$", name)
        if m:
            dist_files.setdefault(float(m.group(2)), {})[m.group(1)] = p
            continue
        m = re.match(r"curve__(trapz|scurve|reach)_(\d+\.\d+)m\.csv$", name)
        if m:
            curve_files[m.group(1)] = p
            continue
        m = re.match(r"(obs_at_start|obs_mid|obs_far)__(trapz|scurve|reach)_(\d+\.\d+)m\.csv$", name)
        if m:
            obstacle_files[f"{m.group(1)}__{m.group(2)}"] = p
    if not dist_files:
        raise SystemExit(
            f"{args.dir} 里没有匹配的场景 CSV。\n"
            f"总览面板按 straight__<后端>_<长度>m.csv 命名匹配；"
            f"若刚改过场景 tag，需同步更新本脚本的匹配式。\n"
            f"先跑：ros2 run nav2_speed_planner speed_profile_gen {args.dir}")

    lengths = sorted(dist_files)
    n_cols = max(len(lengths), 5)
    fig, axes = plt.subplots(3, n_cols, figsize=(3.9 * n_cols, 11.1), squeeze=False)

    # ---- 第 1 行：各距离 v-s（弧长域）----
    for col, length in enumerate(lengths):
        ax = axes[0][col]
        for planner, style, color in (
            ("trapz", "-o", "#1f77b4"), ("scurve", "-s", "#d62728"),
            ("reach", "-^", "#2ca02c")):
            if planner in dist_files[length]:
                s, v = load_csv(dist_files[length][planner])
                ax.plot(s, v, style, color=color, markersize=2.0, linewidth=1.1,
                        label=planner)
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
    for planner, style, color in (
            ("trapz", "-o", "#1f77b4"), ("scurve", "-s", "#d62728"),
            ("reach", "-^", "#2ca02c")):
        if planner in dist_files[vt_len]:
            s, v = load_csv(dist_files[vt_len][planner])
            t = integrate_time(s, v)
            ax.plot(t, v, style, color=color, markersize=2.0, linewidth=1.1,
                    label=planner)
    ax.set_title(f"{vt_len:.2f} m（v-t 时间域）：trapz 为直线三角/梯形", fontsize=10)
    ax.set_xlabel("t (s)")
    ax.set_ylabel("v (m/s)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7)
    row2.append(0)

    panels = [
        ("obs_at_start__trapz", "trapz：起点在障碍内切区（豁免起步）",
         [(0.0, 0.10, "障碍(豁免)")]),
        ("obs_mid__trapz", "trapz：中途障碍 x≥0.70",
         [(0.5, 0.7, "缓冲带"), (0.7, 1.0, "障碍区")]),
        ("obs_far__trapz", "trapz：5m 路径、3m 处障碍",
         [(2.8, 3.0, "缓冲带"), (3.0, 5.0, "障碍区")]),
        ("obs_far__scurve", "scurve：5m 路径、3m 处障碍",
         [(2.8, 3.0, "缓冲带"), (3.0, 5.0, "障碍区")]),
    ]
    col = 1
    for tag, title, spans in panels:
        if tag not in obstacle_files or col >= n_cols:
            continue
        ax = axes[1][col]
        s, v = load_csv(obstacle_files[tag])
        color = "#2ca02c" if tag.endswith("__trapz") else "#d62728"
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
        for planner, color in (
                ("trapz", "#1f77b4"), ("scurve", "#d62728"), ("reach", "#2ca02c")):
            if planner in curve_files:
                s, v = load_csv(curve_files[planner])
                ax.plot(s, v, "-o", color=color, markersize=2.0, linewidth=1.1,
                        label=planner)
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
    # ---- 第 3 行：交叉约束场景（X_*）----
    cross = [
        ("X_T_obs_after_curve", "T：障碍紧随弯道（曲率×障碍）",
         [(2.0, 2.5, "弯道"), (3.0, 5.0, "障碍区")]),
        ("X_T_fast_near_obs", "T：v0=1.0 距障碍 0.5m（制动不可达）",
         [(0.5, 5.0, "障碍区")]),
        ("X_S_fast_near_obs", "S：v0=1.0 距障碍 0.5m（jerk 限制刹不住）",
         [(0.5, 5.0, "障碍区")]),
        ("X_T_two_obstacles", "T：双障碍段", [(1.5, 1.7, "障碍1"), (3.5, 5.0, "障碍2")]),
        ("X_T_dense_all", "T：1m 内弯道+障碍+硬停",
         [(0.3, 0.5, "弯道"), (0.8, 1.0, "障碍区")]),
    ]
    used3 = 0
    for tag, title, spans in cross:
        if used3 >= n_cols:
            break
        hits = find_csvs(args.dir, tag + "_*.csv")
        if not hits:
            continue
        ax = axes[2][used3]
        s, v = load_csv(sorted(hits)[0])
        ax.plot(s, v, "-o", color="#2ca02c", markersize=2.0, linewidth=1.1)
        for lo, hi, lab in spans:
            ax.axvspan(lo, hi, alpha=0.13,
                       color="orange" if "障碍" in lab else "#9467bd", label=lab)
        ax.set_title(title, fontsize=9)
        ax.set_xlabel("s (m)")
        ax.set_ylabel("v (m/s)")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)
        used3 += 1
    for c in range(used3, n_cols):
        axes[2][c].axis("off")

    fig.savefig(args.out, dpi=105)
    print(f"saved: {args.out}")

    if not args.no_detail:
        detail_root = os.path.join(os.path.dirname(args.out), "detail")
        csvs = find_csvs(args.dir)
        outs, per_dir = [], {}
        for c in csvs:
            # 详图落在与 CSV 相同的后端子目录下，两边目录结构一一对应
            sub = os.path.relpath(os.path.dirname(c), args.dir)
            out_dir = detail_root if sub == "." else os.path.join(detail_root, sub)
            os.makedirs(out_dir, exist_ok=True)
            o = render_detail(c, out_dir)
            if o:
                outs.append(o)
                per_dir[sub] = per_dir.get(sub, 0) + 1
        skipped = len(csvs) - len(outs)
        print(f"每场景详图 {len(outs)} 张 → {detail_root}/"
              + (f"（跳过 {skipped} 个空计划场景）" if skipped else ""))
        for sub in sorted(per_dir):
            print(f"    {sub}/  {per_dir[sub]} 张")


if __name__ == "__main__":
    main()
