#!/usr/bin/env python3
# Copyright 2026 DEXE Robot Engineer
# SPDX-License-Identifier: Apache-2.0
"""速度规划离线出图一键入口：跑剖面生成器 + 渲染全部图表。

用法：
  ros2 run nav2_speed_planner gen_speed_plan_charts.py [--out-root 目录]
  缺省输出 ~/.dexe/navigation/speed_planner/：
    csv/                 各场景剖面 CSV（speed_profile_gen 生成）
    speed_profiles.png   总览（距离族 v-s / v-t 时间域 / 障碍场景）
    detail/*.png         每场景四联详图（v-s+region / a-s+限值 / jerk / v-t）

场景覆盖：直线 0.3~5m 静止起步、运动中起步 v0=0.3/0.6/1.0、
弯道（2m 处 0.5m 弧段 r=0.5）、起点障碍豁免、中途障碍、5m@3m 障碍、
硬停（end_speed=0）——T/S 两 planner。
"""
import argparse
import os
import subprocess
import sys


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument(
        "--out-root", default="~/.dexe/navigation/speed_planner",
        help="输出根目录（默认 ~/.dexe/navigation/speed_planner）")
    args = ap.parse_args()
    root = os.path.expanduser(args.out_root)
    csv_dir = os.path.join(root, "csv")
    os.makedirs(csv_dir, exist_ok=True)

    here = os.path.dirname(os.path.abspath(__file__))
    gen = os.path.join(here, "speed_profile_gen")
    if not os.path.exists(gen):
        sys.exit(f"找不到剖面生成器：{gen}（请先 colcon build nav2_speed_planner）")

    print("== 1/2 生成剖面 CSV ==")
    subprocess.run([gen, csv_dir], check=True)

    print("== 2/2 渲染图表 ==")
    subprocess.run(
        [sys.executable, os.path.join(here, "plot_speed_profiles.py"),
         "--dir", csv_dir,
         "--out", os.path.join(root, "speed_profiles.png")],
        check=True)
    print(f"全部图表已输出到：{root}")


if __name__ == "__main__":
    main()
