#!/usr/bin/env python3
"""Analyse move_record.csv produced by random_navigate_through_graph_points.py.

Usage:
    python3 plot_move_record.py                      # 读当前目录 move_record.csv，存 PNG
    python3 plot_move_record.py --show               # 同时弹窗
    python3 plot_move_record.py --csv /path/rec.csv --out /path/scatter.png

散点图：目标点 xy 红色，每次到点的平均 map 坐标 xy——
从另一个点驶来为绿色，目标点与上一次相同（原地移动）为蓝色三角。
x/y 轴等分辨率（aspect equal + 等跨度），否则毫米级偏差会被横纵不同缩放拉变形。

依赖 pandas / matplotlib，建议在开发机上跑（车上通常没装）。
"""

import argparse
import math
import os
import sys

import pandas as pd


def load(csv_path: str) -> pd.DataFrame:
    df = pd.read_csv(csv_path)

    # 到点位姿缺失（TF 断了 / goal 被拒）的行无法参与精度统计
    df["has_pose"] = df["mean_x"].notna() & df["mean_y"].notna()
    df["has_target"] = df["target_x"].notna() & df["target_y"].notna()

    # 终点名作为分组键：来回两个方向各自聚到自己的目标点上
    df["target_name"] = df["path"].astype(str).str.split("->").str[-1]

    # 原地移动判定：本次目标点 == 上一次目标点（随机序列抽到同一条时，
    # 车已经停在终点上，这一轮几乎没有实际行程）。按 CSV 行序（= 时间序）比对。
    # 首行没有上一次，按正常移动处理。
    df["in_place"] = df["target_name"].eq(df["target_name"].shift(1))
    df["move_type"] = df["in_place"].map({True: "in_place", False: "transit"})

    # err_* / dist 采集时已算好；这里对缺失的补算一次，便于手工拼接的旧数据也能用
    need = df["has_pose"] & df["has_target"] & df["dist"].isna()
    if need.any():
        df.loc[need, "err_x"] = df.loc[need, "mean_x"] - df.loc[need, "target_x"]
        df.loc[need, "err_y"] = df.loc[need, "mean_y"] - df.loc[need, "target_y"]
        df.loc[need, "dist"] = (df.loc[need, "err_x"] ** 2
                                + df.loc[need, "err_y"] ** 2) ** 0.5
    return df


def print_summary(df: pd.DataFrame) -> None:
    total = len(df)
    ok = int(df["success"].astype(str).str.lower().isin(["true", "1"]).sum())
    print(f"\n总计 {total} 次移动，success={ok}，失败/被拒 {total - ok}")

    valid = df[df["has_pose"] & df["has_target"]]
    if valid.empty:
        print("没有可用于精度统计的样本（缺 target 或缺到点位姿）")
        return

    aggs = dict(
        n=("dist", "size"),
        dist_mean_mm=("dist", lambda s: s.mean() * 1000.0),
        dist_p95_mm=("dist", lambda s: s.quantile(0.95) * 1000.0),
        dist_max_mm=("dist", "max"),
        err_x_mean_mm=("err_x", lambda s: s.mean() * 1000.0),
        err_y_mean_mm=("err_y", lambda s: s.mean() * 1000.0),
        err_x_std_mm=("err_x", lambda s: s.std() * 1000.0),
        err_y_std_mm=("err_y", lambda s: s.std() * 1000.0),
        err_yaw_mean_deg=("err_yaw", lambda s: math.degrees(s.mean())),
        err_yaw_std_deg=("err_yaw", lambda s: math.degrees(s.std())),
        settle_std_x_mm=("std_x", lambda s: s.mean() * 1000.0),
        settle_std_y_mm=("std_y", lambda s: s.mean() * 1000.0),
        duration_mean_s=("duration_s", "mean"),
    )
    # 极差列是后加的，旧 CSV 没有；缺列时静默跳过而不是整段统计报错
    if {"range_x", "range_y", "range_yaw"} <= set(valid.columns):
        aggs.update(
            settle_rng_x_mm=("range_x", lambda s: s.mean() * 1000.0),
            settle_rng_y_mm=("range_y", lambda s: s.mean() * 1000.0),
            settle_rng_yaw_deg=("range_yaw", lambda s: math.degrees(s.mean())),
        )

    summary = valid.groupby(["target_name", "move_type"]).agg(**aggs)
    summary["dist_max_mm"] *= 1000.0

    print("\n按目标点 × 移动类型分组（transit=从另一点驶来，in_place=目标点与上次相同的原地移动；"
          "长度 mm / 角度 deg）：")
    with pd.option_context("display.width", 220, "display.max_columns", 50):
        print(summary.round(2).to_string())

    print("\n说明：err_* 带符号——均值远大于标准差说明是系统性偏置（每次停同一侧），"
          "\n      标准差大而均值近 0 说明是随机散布；settle_std_* 是静止 1s 内的定位抖动，"
          "\n      它是精度的本底噪声，err 的标准差小于它就没有再优化的意义了；"
          "\n      settle_rng_* 是同一窗口的极差 max-min，远大于 std 的几倍说明抖动不是高斯噪声"
          "\n      而是偶发跳变（定位重定位/匹配跳帧）。")


def plot(df: pd.DataFrame, out_path: str, show: bool, min_span: float) -> None:
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    valid = df[df["has_pose"] & df["has_target"]]
    if valid.empty:
        print("无可绘制数据")
        return

    names = sorted(valid["target_name"].unique())
    fig, axes = plt.subplots(1, len(names) + 1,
                             figsize=(5.2 * (len(names) + 1), 5.4))
    if len(names) + 1 == 1:
        axes = [axes]

    # ── 总览：两个目标点 + 所有到点位置，真实相对关系 ──────────────
    ax = axes[0]
    targets = valid.groupby("target_name")[["target_x", "target_y"]].first()
    _scatter_arrivals(ax, valid, size=18)
    ax.scatter(targets["target_x"], targets["target_y"], c="red", marker="X",
               s=140, label="target", zorder=4)
    for name, row in targets.iterrows():
        ax.annotate(name, (row["target_x"], row["target_y"]),
                    textcoords="offset points", xytext=(8, 8), color="red")
    ax.set_title(f"overview (n={len(valid)})")
    _finish_axis(ax)
    ax.legend(loc="best", fontsize=8)

    # ── 每个目标点一张放大图：等跨度 + equal aspect ────────────────
    for ax, name in zip(axes[1:], names):
        g = valid[valid["target_name"] == name]
        tx = float(g["target_x"].iloc[0])
        ty = float(g["target_y"].iloc[0])

        _scatter_arrivals(ax, g, size=26)
        ax.scatter([tx], [ty], c="red", marker="X", s=160,
                   label="target", zorder=4)

        # 95% 分位圆：直观给出"绝大多数停车落在多大半径内"
        if len(g) >= 3:
            r95 = float(g["dist"].quantile(0.95))
            ax.add_patch(plt.Circle((tx, ty), r95, fill=False, color="gray",
                                    linestyle="--", linewidth=1.0, zorder=2))
            ax.annotate(f"p95 = {r95 * 1000:.0f} mm", (tx, ty + r95),
                        textcoords="offset points", xytext=(4, 4),
                        color="gray", fontsize=8)

        # 等跨度：以目标点为中心，x/y 用同一个半跨度，配合 aspect=equal
        # 保证 1mm 在横纵方向占同样像素，偏差形状不失真
        span = max(
            (g["mean_x"] - tx).abs().max(),
            (g["mean_y"] - ty).abs().max(),
            min_span,
        ) * 1.25
        ax.set_xlim(tx - span, tx + span)
        ax.set_ylim(ty - span, ty + span)

        ax.set_title(f"{name}  n={len(g)}  "
                     f"mean={g['dist'].mean() * 1000:.0f}mm  "
                     f"max={g['dist'].max() * 1000:.0f}mm")
        _finish_axis(ax, adjustable="box")
        ax.legend(loc="best", fontsize=8)

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"\n图已保存: {out_path}")
    if show:
        plt.show()


def _scatter_arrivals(ax, g, size: int) -> None:
    """到点均值散点：transit 绿色，in_place（与上次同一目标点的原地移动）蓝色。

    两类分开画而不是统一绿色：原地移动没有行程，考的是 goal_checker + 定位重复性，
    真实行驶后的到点误差才反映路径跟踪精度，混在一起会把两种误差源平均掉。
    """
    transit = g[~g["in_place"]]
    in_place = g[g["in_place"]]
    if not transit.empty:
        ax.scatter(transit["mean_x"], transit["mean_y"], c="green", s=size,
                   label=f"transit (n={len(transit)})", zorder=3)
    if not in_place.empty:
        ax.scatter(in_place["mean_x"], in_place["mean_y"], c="blue", s=size,
                   marker="^", label=f"in-place (n={len(in_place)})", zorder=3)


def _finish_axis(ax, adjustable: str = "datalim") -> None:
    # box: 保持已设的等跨度 xlim/ylim，改绘图框形状去满足 equal（放大图用）
    # datalim: 没手工设范围时让 matplotlib 自己扩范围（总览图用）
    ax.set_aspect("equal", adjustable=adjustable)
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--csv", default="move_record.csv",
                        help="输入 CSV (default: ./move_record.csv)")
    parser.add_argument("--out", default=None,
                        help="输出 PNG (default: 与 CSV 同目录的 move_record_scatter.png)")
    parser.add_argument("--show", action="store_true", help="除保存外再弹窗显示")
    parser.add_argument("--min-span", type=float, default=0.02,
                        help="放大图的最小半跨度 m，避免样本过于集中时坐标轴无限放大 (default: 0.02)")
    parser.add_argument("--no-plot", action="store_true", help="只打印统计，不画图")
    args = parser.parse_args(argv)

    csv_path = os.path.abspath(args.csv)
    if not os.path.exists(csv_path):
        print(f"找不到 {csv_path}", file=sys.stderr)
        return 1

    df = load(csv_path)
    print_summary(df)

    if not args.no_plot:
        out = args.out or os.path.join(os.path.dirname(csv_path),
                                       "move_record_scatter.png")
        plot(df, os.path.abspath(out), args.show, args.min_span)
    return 0


if __name__ == "__main__":
    sys.exit(main())
