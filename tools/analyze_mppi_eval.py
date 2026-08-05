#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MPPI Controller Evaluation Tool: Parses [MPPI_EVAL] and [MPPI_GOAL_EVAL] logs 
to compare Control Precision, Motion Behavior, and Goal Arrival Precision (CPU vs CUDA mode).
"""

import sys
import re
import argparse
import math
from typing import List, Dict

def parse_eval_logs(log_filepath: str):
    eval_pattern = re.compile(
        r'\[MPPI_EVAL\]\s+time=([\d\.]+),\s+cuda=(\d+),\s+heading_err=([-\d\.]+),\s+xte=([-\d\.]+),\s+vx=([-\d\.]+),\s+vy=([-\d\.]+),\s+wz=([-\d\.]+),\s+ax=([-\d\.]+),\s+wz_dot=([-\d\.]+)(?:,\s+goal_dist=([-\d\.]+),\s+goal_yaw_err=([-\d\.]+))?'
    )

    goal_eval_pattern = re.compile(
        r'\[MPPI_GOAL_EVAL\]\s+time=([\d\.]+),\s+cuda=(\d+),\s+goal_dist=([-\d\.]+),\s+goal_yaw_err=([-\d\.]+),\s+goal_yaw_deg=([-\d\.]+),\s+vx=([-\d\.]+),\s+wz=([-\d\.]+)'
    )

    records = {0: [], 1: []}  # 0: CPU, 1: CUDA
    goal_records = {0: [], 1: []}

    with open(log_filepath, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            m = eval_pattern.search(line)
            if m:
                groups = m.groups()
                t, cuda, heading_err, xte, vx, vy, wz, ax, wz_dot = map(float, groups[:9])
                goal_dist = float(groups[9]) if groups[9] is not None else 0.0
                goal_yaw_err = float(groups[10]) if groups[10] is not None else 0.0

                cuda_flag = int(cuda)
                records[cuda_flag].append({
                    'time': t,
                    'heading_err': heading_err,
                    'heading_deg': math.degrees(heading_err),
                    'xte': xte,
                    'vx': vx,
                    'vy': vy,
                    'wz': wz,
                    'ax': ax,
                    'wz_dot': wz_dot,
                    'goal_dist': goal_dist,
                    'goal_yaw_err': goal_yaw_err,
                    'goal_yaw_deg': math.degrees(goal_yaw_err)
                })

            mg = goal_eval_pattern.search(line)
            if mg:
                t, cuda, goal_dist, goal_yaw_err, goal_yaw_deg, vx, wz = map(float, mg.groups())
                cuda_flag = int(cuda)
                goal_records[cuda_flag].append({
                    'time': t,
                    'goal_dist': goal_dist,
                    'goal_yaw_err': goal_yaw_err,
                    'goal_yaw_deg': goal_yaw_deg,
                    'vx': vx,
                    'wz': wz
                })

    return records, goal_records

def calculate_metrics(data: List[Dict], goal_data: List[Dict]):
    if not data:
        return None

    n = len(data)
    heading_errs_deg = [abs(d['heading_deg']) for d in data]
    xte_list = [d['xte'] for d in data]
    vx_list = [d['vx'] for d in data]
    wz_list = [abs(d['wz']) for d in data]
    ax_list = [abs(d['ax']) for d in data]
    wz_dot_list = [abs(d['wz_dot']) for d in data]

    rmse_xte = math.sqrt(sum(x ** 2 for x in xte_list) / n)
    mae_heading = sum(heading_errs_deg) / n
    max_heading = max(heading_errs_deg)
    
    avg_vx = sum(vx_list) / n
    avg_wz = sum(wz_list) / n
    rms_ax = math.sqrt(sum(a ** 2 for a in ax_list) / n)
    rms_wz_dot = math.sqrt(sum(w ** 2 for w in wz_dot_list) / n)

    # 统计“大航向偏差下缓慢前进对齐”事件 (>25 deg 且 vx > 0.1 且 |wz| < 0.5)
    slow_align_events = sum(
        1 for d in data if abs(d['heading_deg']) > 25.0 and d['vx'] > 0.1 and abs(d['wz']) < 0.5
    )

    # 到位精度计算：终点停止/近终点（离终点 < 0.3m）时的位置偏差与姿态残差
    near_goal_data = goal_data if goal_data else [d for d in data if d.get('goal_dist', 1.0) < 0.30]
    
    final_pos_err = None
    final_yaw_err_deg = None
    if near_goal_data:
        # 取最靠近终点/最后到位的记录
        final_sample = min(near_goal_data, key=lambda x: (x['goal_dist'], abs(x['vx'])))
        final_pos_err = final_sample['goal_dist']
        final_yaw_err_deg = abs(final_sample.get('goal_yaw_deg', math.degrees(final_sample['goal_yaw_err'])))

    return {
        'count': n,
        'rmse_xte': rmse_xte,
        'mae_heading_deg': mae_heading,
        'max_heading_deg': max_heading,
        'avg_vx': avg_vx,
        'avg_wz': avg_wz,
        'rms_ax': rms_ax,
        'rms_wz_dot': rms_wz_dot,
        'slow_align_events': slow_align_events,
        'final_pos_err': final_pos_err,
        'final_yaw_err_deg': final_yaw_err_deg
    }

def print_comparison_report(cpu_metrics, cuda_metrics):
    print("===================================================================================")
    print("      MPPI Control Precision, Motion & Goal Precision Comparison Report            ")
    print("===================================================================================")
    print(f"{'Metric / Feature':<38} | {'CPU Mode (cuda=0)':<18} | {'CUDA Mode (cuda=1)':<18}")
    print("-" * 83)

    if not cpu_metrics and not cuda_metrics:
        print("No [MPPI_EVAL] log records found in log file!")
        return

    def fmt(m, key, suffix=""):
        if not m or key not in m or m[key] is None:
            return "N/A"
        val = m[key]
        if isinstance(val, float):
            return f"{val:.4f}{suffix}"
        return f"{val}{suffix}"

    print(f"{'Sample Frames Count':<38} | {fmt(cpu_metrics, 'count'):<18} | {fmt(cuda_metrics, 'count'):<18}")
    print(f"{'Cross-Track Error RMSE (m)':<38} | {fmt(cpu_metrics, 'rmse_xte', ' m'):<18} | {fmt(cuda_metrics, 'rmse_xte', ' m'):<18}")
    print(f"{'Heading Error MAE (deg)':<38} | {fmt(cpu_metrics, 'mae_heading_deg', ' deg'):<18} | {fmt(cuda_metrics, 'mae_heading_deg', ' deg'):<18}")
    print(f"{'Max Heading Error (deg)':<38} | {fmt(cpu_metrics, 'max_heading_deg', ' deg'):<18} | {fmt(cuda_metrics, 'max_heading_deg', ' deg'):<18}")
    print(f"{'Average Linear Velocity (m/s)':<38} | {fmt(cpu_metrics, 'avg_vx', ' m/s'):<18} | {fmt(cuda_metrics, 'avg_vx', ' m/s'):<18}")
    print(f"{'Average Angular Speed (rad/s)':<38} | {fmt(cpu_metrics, 'avg_wz', ' rad/s'):<18} | {fmt(cuda_metrics, 'avg_wz', ' rad/s'):<18}")
    print(f"{'Linear Accel RMS (m/s²)':<38} | {fmt(cpu_metrics, 'rms_ax', ' m/s²'):<18} | {fmt(cuda_metrics, 'rms_ax', ' m/s²'):<18}")
    print(f"{'Angular Accel RMS (rad/s²)':<38} | {fmt(cpu_metrics, 'rms_wz_dot', ' rad/s²'):<18} | {fmt(cuda_metrics, 'rms_wz_dot', ' rad/s²'):<18}")
    print(f"{'Slow Heading Alignment Events':<38} | {fmt(cpu_metrics, 'slow_align_events'):<18} | {fmt(cuda_metrics, 'slow_align_events'):<18}")
    print("-" * 83)
    print("📍 [Goal Arrival Precision / 到位精度检测]")
    print(f"{'Goal Position Final Error (m)':<38} | {fmt(cpu_metrics, 'final_pos_err', ' m'):<18} | {fmt(cuda_metrics, 'final_pos_err', ' m'):<18}")
    print(f"{'Goal Yaw Final Error (deg)':<38} | {fmt(cpu_metrics, 'final_yaw_err_deg', ' deg'):<18} | {fmt(cuda_metrics, 'final_yaw_err_deg', ' deg'):<18}")
    print("===================================================================================")

def main():
    parser = argparse.ArgumentParser(description="Analyze MPPI Evaluation Logs.")
    parser.add_argument("log_file", help="Path to ROS log file containing [MPPI_EVAL]")
    args = parser.parse_args()

    records, goal_records = parse_eval_logs(args.log_file)
    cpu_metrics = calculate_metrics(records[0], goal_records[0])
    cuda_metrics = calculate_metrics(records[1], goal_records[1])

    print_comparison_report(cpu_metrics, cuda_metrics)

if __name__ == "__main__":
    main()
