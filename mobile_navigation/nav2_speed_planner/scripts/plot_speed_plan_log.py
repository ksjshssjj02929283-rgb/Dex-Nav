#!/usr/bin/env python3
# Copyright 2026 DEXE Robot Engineer
# SPDX-License-Identifier: Apache-2.0
"""分析 controller_server 运行期剖面日志（speed_plan_log_dir 产出的行式 CSV：
id,attr,v0,v1,... 同一 id 多行共用，每行一个属性的全路径序列）。

用法：
  python3 plot_speed_plan_log.py <csv 文件> [--seq-from N] [--seq-to M] [--stride K] [--out xxx.png]

图 1：v-s 剖面按轮次（seq）着色叠加——观察剖面逐轮演变（末端先降后增类问题一眼可见）；
      障碍标记点叠加显示（× = obstacle，▲ = buffer，○ = exempt）。
图 2：末端放大（最后 0.6m）。
图 3：曲率 |kappa| 随 s（最后一轮）。
"""
import argparse
import collections

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
matplotlib.rcParams["font.sans-serif"] = [
    "Noto Sans CJK SC", "WenQuanYi Micro Hei", "AR PL UMing CN", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv_file")
    ap.add_argument("--seq-from", type=int, default=None)
    ap.add_argument("--seq-to", type=int, default=None)
    ap.add_argument("--stride", type=int, default=1, help="每 K 轮取一轮（轮次太多时抽样）")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    # 行式格式：id,attr,v0,v1,...（attr ∈ stamp/x/y/s/v/a/jerk/kappa/obstacle）
    raw = collections.OrderedDict()  # id -> {attr: [values]}
    with open(args.csv_file) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            seq = int(parts[0])
            if args.seq_from is not None and seq < args.seq_from:
                continue
            if args.seq_to is not None and seq > args.seq_to:
                continue
            raw.setdefault(seq, {})[parts[1]] = parts[2:]
    rounds = collections.OrderedDict()
    for seq, attrs in raw.items():
        if "s" not in attrs or "v" not in attrs:
            continue
        rounds[seq] = {
            "s": [float(x) for x in attrs["s"]],
            "v": [float(x) for x in attrs["v"]],
            "k": [float(x) for x in attrs.get("kappa", ["0"] * len(attrs["s"]))],
            "obs": [int(x) for x in attrs.get("obstacle", ["0"] * len(attrs["s"]))],
        }
    if not rounds:
        raise SystemExit("no rows matched")

    seqs = list(rounds)[::max(args.stride, 1)]
    cmap = plt.cm.viridis
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.6))

    obs_style = {1: ("x", "red", "obstacle"), 2: ("^", "orange", "buffer"),
                 3: ("o", "gray", "exempt")}
    for panel, (ax, s_min_from_end) in enumerate(zip(axes[:2], [None, 0.6])):
        seen_obs_labels = set()
        for idx, seq in enumerate(seqs):
            r = rounds[seq]
            color = cmap(idx / max(len(seqs) - 1, 1))
            s_end = r["s"][-1]
            if s_min_from_end is not None:
                pts = [(s, v, o) for s, v, o in zip(r["s"], r["v"], r["obs"])
                       if s_end - s <= s_min_from_end]
                if not pts:
                    continue
                s_arr, v_arr, o_arr = zip(*pts)
            else:
                s_arr, v_arr, o_arr = r["s"], r["v"], r["obs"]
            ax.plot(s_arr, v_arr, "-", color=color, linewidth=0.9, alpha=0.8,
                    label=f"seq {seq}" if len(seqs) <= 12 else None)
            for s, v, o in zip(s_arr, v_arr, o_arr):
                if o in obs_style:
                    m, c, lbl = obs_style[o]
                    ax.plot(s, v, m, color=c, markersize=5,
                            label=lbl if lbl not in seen_obs_labels else None)
                    seen_obs_labels.add(lbl)
        ax.set_xlabel("s (m)")
        ax.set_ylabel("v (m/s)")
        ax.set_title("v-s 剖面按轮次演变" if panel == 0 else "末端放大（最后 0.6m）")
        ax.grid(True, alpha=0.3)
        if len(seqs) <= 12 or seen_obs_labels:
            ax.legend(fontsize=7)
        if panel == 0 and len(seqs) > 1:
            sm = plt.cm.ScalarMappable(
                cmap=cmap, norm=plt.Normalize(seqs[0], seqs[-1]))
            fig.colorbar(sm, ax=ax, label="seq（轮次）")

    last = rounds[seqs[-1]]
    axes[2].plot(last["s"], [abs(k) for k in last["k"]], "-o", markersize=2,
                 color="#9467bd")
    axes[2].set_xlabel("s (m)")
    axes[2].set_ylabel("|kappa| (1/m)")
    axes[2].set_title(f"曲率（seq {seqs[-1]}）")
    axes[2].grid(True, alpha=0.3)

    out = args.out or args.csv_file.rsplit(".", 1)[0] + ".png"
    fig.suptitle(f"speed plan log: {args.csv_file}（{len(rounds)} 轮）", fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(out, dpi=110)
    print(f"saved: {out}")


if __name__ == "__main__":
    main()
