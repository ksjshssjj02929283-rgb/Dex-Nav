#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MPPI Controller Performance, Precision & Resource Synchronized Analyzer
Fully synchronized:
1. Optimization Performance (Optimizer, Controller cycle, Step1-3 Rollout & Critic scoring)
2. Individual Critic Breakdown ranking
3. Control Precision & Motion Behavior (Heading Error, XTE, Smoother ax/wz_dot, Reverse Cmd count)
4. Goal Arrival Precision (Final position error & final yaw error)
5. System CPU% & Memory RSS utilization (Fully Dual-Mode GPU vs CPU Comparison)
6. GPU Baseline Auto-Backup & Live Follow Dual-Mode Comparison
"""

import sys
import os
import re
import json
import math
import time
import signal
import argparse
import threading
from collections import defaultdict
from typing import List, Dict

try:
    import psutil
    HAS_PSUTIL = True
except ImportError:
    HAS_PSUTIL = False

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

try:
    import matplotlib.pyplot as plt
    plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
    plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'SimHei', 'Arial']
    plt.rcParams['axes.unicode_minus'] = False
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

HISTORY_FILE = "mppi_benchmark_history.json"
GPU_BASELINE_FILE = "mppi_gpu_baseline.json"
data_lock = threading.Lock()
running = True

# 系统资源数据结构
resource_data = {
    "cpu_percent": [],
    "mem_rss_mb": []
}

def find_controller_server_pid():
    if HAS_PSUTIL:
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                p_name = proc.info.get('name') or ''
                cmdline = proc.info.get('cmdline') or []
                cmd_str = ' '.join(cmdline) if isinstance(cmdline, list) else str(cmdline)
                if 'controller_server' in p_name or 'controller_serv' in p_name or 'controller_server' in cmd_str:
                    return proc.info['pid']
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue
    
    # 兜底：使用系统的 ps aux 指令查找 PID
    try:
        import subprocess
        output = subprocess.check_output(['ps', 'aux'], text=True)
        for line in output.splitlines():
            if 'controller_server' in line and 'grep' not in line:
                parts = line.split()
                if len(parts) > 1 and parts[1].isdigit():
                    return int(parts[1])
    except Exception:
        pass
    return None

def monitor_resources(interval=0.5):
    global running, resource_data
    target_pid = None

    while running:
        if target_pid is None:
            target_pid = find_controller_server_pid()
            if target_pid:
                pass
            else:
                time.sleep(0.5)
                continue

        try:
            if HAS_PSUTIL:
                proc = psutil.Process(target_pid)
                proc.cpu_percent(interval=None)  # 预热
                
                while running:
                    time.sleep(interval)
                    cpu_p = proc.cpu_percent(interval=None)
                    mem_rss = proc.memory_info().rss / (1024.0 * 1024.0)

                    with data_lock:
                        resource_data["cpu_percent"].append(cpu_p)
                        resource_data["mem_rss_mb"].append(mem_rss)
            else:
                import subprocess
                while running:
                    time.sleep(interval)
                    out = subprocess.check_output(['ps', '-p', str(target_pid), '-o', '%cpu,rss'], text=True)
                    lines = out.strip().splitlines()
                    if len(lines) > 1:
                        parts = lines[1].split()
                        cpu_p = float(parts[0])
                        mem_rss = float(parts[1]) / 1024.0
                        with data_lock:
                            resource_data["cpu_percent"].append(cpu_p)
                            resource_data["mem_rss_mb"].append(mem_rss)

        except Exception:
            target_pid = None

def parse_mppi_log(log_filepath: str):
    if not os.path.exists(log_filepath):
        print(f"Error: Log file '{log_filepath}' does not exist.")
        sys.exit(1)

    current_mode = "GPU (CUDA)"  # 默认模式
    modes_data = {
        "GPU (CUDA)": {
            "opt": [], "cycle": [], "step1": [], "step2": [], "step3": [],
            "critics": defaultdict(list),
            "eval_records": [], "goal_records": [], "resource_stats": None
        },
        "CPU": {
            "opt": [], "cycle": [], "step1": [], "step2": [], "step3": [],
            "critics": defaultdict(list),
            "eval_records": [], "goal_records": [], "resource_stats": None
        }
    }

    # 正则表达式集合
    re_cuda_cfg = re.compile(r"MPPI Optimizer use_cuda configuration set to:\s*(ENABLED|DISABLED)")
    re_cycle_opt = re.compile(r"cycle=([\d\.]+)ms\(opt=([\d\.]+)\)")
    re_step_breakdown = re.compile(
        r"Step1_Rollout=([\d\.]+)ms\s*\|\s*Step2_CriticScoring=([\d\.]+)ms\s*\|\s*Step3_SequenceUpdate=([\d\.]+)ms\s*\|\s*TotalOpt=([\d\.]+)ms"
    )
    re_critic_prefix = re.compile(r"Per-critic timings:\s*(.*)")
    
    # 精度与控制表现正则
    re_eval_pattern = re.compile(
        r'\[MPPI_EVAL\]\s+time=([\d\.]+),\s+cuda=(\d+),\s+heading_err=([-\d\.]+),\s+xte=([-\d\.]+),\s+vx=([-\d\.]+),\s+vy=([-\d\.]+),\s+wz=([-\d\.]+),\s+ax=([-\d\.]+),\s+wz_dot=([-\d\.]+)(?:,\s+goal_dist=([-\d\.]+),\s+goal_yaw_err=([-\d\.]+))?'
    )
    re_goal_eval_pattern = re.compile(
        r'\[MPPI_GOAL_EVAL\]\s+time=([\d\.]+),\s+cuda=(\d+),\s+goal_dist=([-\d\.]+),\s+goal_yaw_err=([-\d\.]+),\s+goal_yaw_deg=([-\d\.]+),\s+vx=([-\d\.]+),\s+wz=([-\d\.]+)'
    )
    re_goal_reached_pattern = re.compile(
        r'goal reached error:\s*lon_err:\s*([-\d\.]+),\s*lat_err:\s*([-\d\.]+),\s*yaw_err:\s*([-\d\.]+)'
    )
    re_overshoot_pattern = re.compile(
        r'\[MPPI_GOAL_OVERSHOOT_DIAG\]\s+Mode:\s*([^\s\|]+)\s*\|\s+DistToGoal:\s*([\d\.]+)m\s*\|\s+MaxTrajLength:\s*([\d\.]+)m\s*\|\s+Overshoot:\s*\+([\d\.]+)m\s*\|\s+Cmd_vx:\s*([\d\.]+)m/s'
    )

    with open(log_filepath, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            # 模式切换检测
            cfg_match = re_cuda_cfg.search(line)
            if cfg_match:
                status = cfg_match.group(1)
                current_mode = "GPU (CUDA)" if status == "ENABLED" else "CPU"
                continue

            # 检测周期及 optimization 耗时
            cycle_match = re_cycle_opt.search(line)
            if cycle_match:
                cycle_ms = float(cycle_match.group(1))
                opt_ms = float(cycle_match.group(2))
                modes_data[current_mode]["cycle"].append(cycle_ms)
                modes_data[current_mode]["opt"].append(opt_ms)

            # 检测 Step 三阶段耗时
            step_match = re_step_breakdown.search(line)
            if step_match:
                step1 = float(step_match.group(1))
                step2 = float(step_match.group(2))
                step3 = float(step_match.group(3))
                opt_tot = float(step_match.group(4))
                modes_data[current_mode]["step1"].append(step1)
                modes_data[current_mode]["step2"].append(step2)
                modes_data[current_mode]["step3"].append(step3)
                if not cycle_match:
                    modes_data[current_mode]["opt"].append(opt_tot)

            # 解析 Per-critic 细分耗时
            critic_match = re_critic_prefix.search(line)
            if critic_match:
                raw_critics = critic_match.group(1).split("|")
                for item in raw_critics:
                    if ":" in item:
                        name, val_str = item.split(":")
                        name = name.strip()
                        val_ms = float(val_str.replace("ms", "").strip())
                        modes_data[current_mode]["critics"][name].append(val_ms)

            # 解析控制精度 [MPPI_EVAL]
            eval_match = re_eval_pattern.search(line)
            if eval_match:
                groups = eval_match.groups()
                t, cuda, heading_err, xte, vx, vy, wz, ax, wz_dot = map(float, groups[:9])
                goal_dist = float(groups[9]) if groups[9] is not None else 0.0
                goal_yaw_err = float(groups[10]) if groups[10] is not None else 0.0
                mode_key = "GPU (CUDA)" if int(cuda) == 1 else "CPU"
                modes_data[mode_key]["eval_records"].append({
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

            # 解析到位精度 [MPPI_GOAL_EVAL]
            goal_match = re_goal_eval_pattern.search(line)
            if goal_match:
                t, cuda, goal_dist, goal_yaw_err, goal_yaw_deg, vx, wz = map(float, goal_match.groups())
                mode_key = "GPU (CUDA)" if int(cuda) == 1 else "CPU"
                modes_data[mode_key]["goal_records"].append({
                    'time': t,
                    'goal_dist': goal_dist,
                    'goal_yaw_err': goal_yaw_err,
                    'goal_yaw_deg': goal_yaw_deg,
                    'vx': vx,
                    'wz': wz
                })

            # 解析 HighPrecisionGoalChecker 最终停车结果 [goal reached error]
            reached_match = re_goal_reached_pattern.search(line)
            if reached_match:
                lon_err, lat_err, yaw_err = map(float, reached_match.groups())
                dist = math.hypot(lon_err, lat_err)
                modes_data[current_mode]["goal_records"].append({
                    'time': 0.0,
                    'goal_dist': dist,
                    'goal_yaw_err': yaw_err,
                    'goal_yaw_deg': math.degrees(yaw_err),
                    'vx': 0.0,
                    'wz': 0.0
                })

    # 将运行期间抓取到的系统资源填充进活跃模式中
    with data_lock:
        if resource_data["cpu_percent"]:
            cpu_st = calculate_stats(resource_data["cpu_percent"])
            mem_st = calculate_stats(resource_data["mem_rss_mb"])
            modes_data[current_mode]["resource_stats"] = {
                "cpu": cpu_st,
                "mem": mem_st
            }

    return modes_data

def save_gpu_baseline(gpu_data: Dict, baseline_file=GPU_BASELINE_FILE):
    """单独将有效的 GPU 数据备份持久化为独立的 Baseline 基准文件"""
    if not gpu_data or not gpu_data.get("opt"):
        return
    serializable = {
        "opt": gpu_data["opt"],
        "cycle": gpu_data["cycle"],
        "step1": gpu_data["step1"],
        "step2": gpu_data["step2"],
        "step3": gpu_data["step3"],
        "critics": {k: list(v) for k, v in gpu_data["critics"].items()},
        "eval_records": gpu_data["eval_records"],
        "goal_records": gpu_data["goal_records"],
        "resource_stats": gpu_data.get("resource_stats")
    }
    try:
        with open(baseline_file, "w", encoding="utf-8") as f:
            json.dump(serializable, f, indent=2, ensure_ascii=False)
        print(f"🎯 [GPU Baseline Auto-Saved] 已自动将 GPU 测试基准独立备份至: {os.path.abspath(baseline_file)}")
    except Exception as e:
        print(f"⚠️ [GPU Baseline Save Error]: {e}")

def load_gpu_baseline(baseline_file=GPU_BASELINE_FILE):
    """读取独立的 GPU 基准文件"""
    if not os.path.exists(baseline_file):
        return None
    try:
        with open(baseline_file, "r", encoding="utf-8") as f:
            h_data = json.load(f)
            return {
                "opt": h_data.get("opt", []),
                "cycle": h_data.get("cycle", []),
                "step1": h_data.get("step1", []),
                "step2": h_data.get("step2", []),
                "step3": h_data.get("step3", []),
                "critics": defaultdict(list, {k: list(v) for k, v in h_data.get("critics", {}).items()}),
                "eval_records": h_data.get("eval_records", []),
                "goal_records": h_data.get("goal_records", []),
                "resource_stats": h_data.get("resource_stats")
            }
    except Exception as e:
        print(f"⚠️ [GPU Baseline Read Error]: {e}")
        return None

def load_and_merge_history(current_modes_data, history_file=HISTORY_FILE, gpu_baseline_file=GPU_BASELINE_FILE):
    """从本地 JSON 文件读取历史测试结果，并与当前解析的数据合并"""
    merged_data = {
        "GPU (CUDA)": {
            "opt": list(current_modes_data["GPU (CUDA)"]["opt"]),
            "cycle": list(current_modes_data["GPU (CUDA)"]["cycle"]),
            "step1": list(current_modes_data["GPU (CUDA)"]["step1"]),
            "step2": list(current_modes_data["GPU (CUDA)"]["step2"]),
            "step3": list(current_modes_data["GPU (CUDA)"]["step3"]),
            "critics": defaultdict(list, {k: list(v) for k, v in current_modes_data["GPU (CUDA)"]["critics"].items()}),
            "eval_records": list(current_modes_data["GPU (CUDA)"]["eval_records"]),
            "goal_records": list(current_modes_data["GPU (CUDA)"]["goal_records"]),
            "resource_stats": current_modes_data["GPU (CUDA)"]["resource_stats"]
        },
        "CPU": {
            "opt": list(current_modes_data["CPU"]["opt"]),
            "cycle": list(current_modes_data["CPU"]["cycle"]),
            "step1": list(current_modes_data["CPU"]["step1"]),
            "step2": list(current_modes_data["CPU"]["step2"]),
            "step3": list(current_modes_data["CPU"]["step3"]),
            "critics": defaultdict(list, {k: list(v) for k, v in current_modes_data["CPU"]["critics"].items()}),
            "eval_records": list(current_modes_data["CPU"]["eval_records"]),
            "goal_records": list(current_modes_data["CPU"]["goal_records"]),
            "resource_stats": current_modes_data["CPU"]["resource_stats"]
        }
    }

    # 1. 若当前解析到了新的 GPU 数据，主动备份成独立 GPU 基准文件
    if current_modes_data["GPU (CUDA)"]["opt"]:
        save_gpu_baseline(current_modes_data["GPU (CUDA)"], gpu_baseline_file)

    # 2. 如果当前解析的数据里没有 GPU 数据，尝试从专用的 GPU Baseline 文件补全！
    if not merged_data["GPU (CUDA)"]["opt"]:
        gpu_base = load_gpu_baseline(gpu_baseline_file)
        if gpu_base and gpu_base["opt"]:
            merged_data["GPU (CUDA)"] = gpu_base

    # 3. 读取通用 History 补全缺失部分
    if os.path.exists(history_file):
        try:
            with open(history_file, "r", encoding="utf-8") as f:
                hist_raw = json.load(f)
                for mode in ["GPU (CUDA)", "CPU"]:
                    if mode in hist_raw:
                        h_mode = hist_raw[mode]
                        if not merged_data[mode]["opt"] and h_mode.get("opt"):
                            merged_data[mode]["opt"] = h_mode["opt"]
                            merged_data[mode]["cycle"] = h_mode.get("cycle", [])
                            merged_data[mode]["step1"] = h_mode.get("step1", [])
                            merged_data[mode]["step2"] = h_mode.get("step2", [])
                            merged_data[mode]["step3"] = h_mode.get("step3", [])
                            merged_data[mode]["eval_records"] = h_mode.get("eval_records", [])
                            merged_data[mode]["goal_records"] = h_mode.get("goal_records", [])
                            merged_data[mode]["resource_stats"] = h_mode.get("resource_stats")
                            for c_name, c_vals in h_mode.get("critics", {}).items():
                                merged_data[mode]["critics"][c_name] = c_vals
        except Exception as e:
            pass

    # 将合并后的有效数据反向保存持久化回 JSON
    save_history(merged_data, history_file)
    return merged_data

def save_history(data, history_file=HISTORY_FILE):
    """将数据持久化写回 JSON 文件"""
    serializable = {}
    for mode in ["GPU (CUDA)", "CPU"]:
        m_data = data[mode]
        serializable[mode] = {
            "opt": m_data["opt"],
            "cycle": m_data["cycle"],
            "step1": m_data["step1"],
            "step2": m_data["step2"],
            "step3": m_data["step3"],
            "critics": {k: list(v) for k, v in m_data["critics"].items()},
            "eval_records": m_data["eval_records"],
            "goal_records": m_data["goal_records"],
            "resource_stats": m_data["resource_stats"]
        }
    try:
        with open(history_file, "w", encoding="utf-8") as f:
            json.dump(serializable, f, indent=2, ensure_ascii=False)
    except Exception as e:
        pass

def calculate_stats(data_list: List[float]) -> Dict[str, float]:
    if not data_list:
        return {"avg": 0.0, "std": 0.0, "min": 0.0, "max": 0.0, "p50": 0.0, "p95": 0.0, "count": 0}
    if HAS_NUMPY:
        arr = np.array(data_list)
        return {
            "avg": float(np.mean(arr)),
            "std": float(np.std(arr)),
            "min": float(np.min(arr)),
            "max": float(np.max(arr)),
            "p50": float(np.median(arr)),
            "p95": float(np.percentile(arr, 95)),
            "count": len(arr)
        }
    else:
        n = len(data_list)
        s_data = sorted(data_list)
        avg = sum(s_data) / n
        std = math.sqrt(sum((x - avg) ** 2 for x in s_data) / n)
        return {
            "avg": avg, "std": std, "min": s_data[0], "max": s_data[-1],
            "p50": s_data[n // 2], "p95": s_data[int(n * 0.95)], "count": n
        }

def calculate_precision_metrics(eval_data: List[Dict], goal_data: List[Dict]):
    if not eval_data:
        return None

    n = len(eval_data)
    heading_errs_deg = [abs(d['heading_deg']) for d in eval_data]
    xte_list = [d['xte'] for d in eval_data]
    vx_list = [d['vx'] for d in eval_data]
    wz_list = [abs(d['wz']) for d in eval_data]
    ax_list = [abs(d['ax']) for d in eval_data]
    wz_dot_list = [abs(d['wz_dot']) for d in eval_data]

    rmse_xte = math.sqrt(sum(x ** 2 for x in xte_list) / n)
    mae_heading = sum(heading_errs_deg) / n
    max_heading = max(heading_errs_deg)
    
    avg_vx = sum(vx_list) / n
    avg_wz = sum(wz_list) / n
    rms_ax = math.sqrt(sum(a ** 2 for a in ax_list) / n)
    rms_wz_dot = math.sqrt(sum(w ** 2 for w in wz_dot_list) / n)

    slow_align_events = sum(
        1 for d in eval_data if abs(d['heading_deg']) > 25.0 and d['vx'] > 0.1 and abs(d['wz']) < 0.5
    )
    reverse_cmd_events = sum(
        1 for d in eval_data if d['vx'] < -0.01
    )

    near_goal_data = goal_data if goal_data else [d for d in eval_data if d.get('goal_dist', 1.0) < 0.30]
    final_pos_err = None
    final_yaw_err_deg = None
    if near_goal_data:
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
        'reverse_cmd_events': reverse_cmd_events,
        'final_pos_err': final_pos_err,
        'final_yaw_err_deg': final_yaw_err_deg
    }

def print_report(modes_data, monitor_resource_flag=False):
    gpu_stats_opt = calculate_stats(modes_data["GPU (CUDA)"]["opt"])
    cpu_stats_opt = calculate_stats(modes_data["CPU"]["opt"])
    gpu_stats_cycle = calculate_stats(modes_data["GPU (CUDA)"]["cycle"])
    cpu_stats_cycle = calculate_stats(modes_data["CPU"]["cycle"])

    print(f"\n===================================================================================")
    print(f"       MPPI Controller Dual-Mode (GPU vs CPU) Synchronized Comparison ")
    print(f"===================================================================================")

    print(f"\n 🚀 [1/5] 核心优化求解耗时对比 (Optimizer `optimize` Timing)")
    print(f" ---------------------------------------------------------------------------------------------------------")
    print(f"  模式        | 样本数 | 平均(ms) | 标准差  | P50(ms) | P95(ms) | 最小(ms) | 最大(ms)")
    print(f" ---------------------------------------------------------------------------------------------------------")
    print(f"  GPU (CUDA)  | {gpu_stats_opt['count']:6d} | {gpu_stats_opt['avg']:8.3f} | {gpu_stats_opt['std']:7.3f} | {gpu_stats_opt['p50']:7.3f} | {gpu_stats_opt['p95']:7.3f} | {gpu_stats_opt['min']:8.3f} | {gpu_stats_opt['max']:8.3f}")
    print(f"  CPU 纯串行   | {cpu_stats_opt['count']:6d} | {cpu_stats_opt['avg']:8.3f} | {cpu_stats_opt['std']:7.3f} | {cpu_stats_opt['p50']:7.3f} | {cpu_stats_opt['p95']:7.3f} | {cpu_stats_opt['min']:8.3f} | {cpu_stats_opt['max']:8.3f}")
    print(f" ---------------------------------------------------------------------------------------------------------")

    if cpu_stats_opt['avg'] > 0 and gpu_stats_opt['avg'] > 0:
        speedup = cpu_stats_opt['avg'] / gpu_stats_opt['avg']
        print(f"  >>> 🚀 Optimizer 核心优化 GPU 实测加速倍率: {speedup:.2f} x <<<\n")

    print(f" 📊 [2/5] MPPI 控制循环总耗时 (Controller `cycle` Timing)")
    print(f" ---------------------------------------------------------------------------------------------------------")
    print(f"  模式        | 样本数 | 平均(ms) | 标准差  | P50(ms) | P95(ms) | 最小(ms) | 最大(ms)")
    print(f" ---------------------------------------------------------------------------------------------------------")
    print(f"  GPU (CUDA)  | {gpu_stats_cycle['count']:6d} | {gpu_stats_cycle['avg']:8.3f} | {gpu_stats_cycle['std']:7.3f} | {gpu_stats_cycle['p50']:7.3f} | {gpu_stats_cycle['p95']:7.3f} | {gpu_stats_cycle['min']:8.3f} | {gpu_stats_cycle['max']:8.3f}")
    print(f"  CPU 纯串行   | {cpu_stats_cycle['count']:6d} | {cpu_stats_cycle['avg']:8.3f} | {cpu_stats_cycle['std']:7.3f} | {cpu_stats_cycle['p50']:7.3f} | {cpu_stats_cycle['p95']:7.3f} | {cpu_stats_cycle['min']:8.3f} | {cpu_stats_cycle['max']:8.3f}")
    print(f" ---------------------------------------------------------------------------------------------------------\n")

    # Step Breakdown 打印
    for mode in ["GPU (CUDA)", "CPU"]:
        s1 = calculate_stats(modes_data[mode]["step1"])
        s2 = calculate_stats(modes_data[mode]["step2"])
        s3 = calculate_stats(modes_data[mode]["step3"])
        if s1['count'] > 0:
            print(f" 🔍 [{mode}] 底层三阶段耗时拆解 (STEP BREAKDOWN):")
            print(f"   • Step1_Rollout (轨迹预测积分)     : 平均 {s1['avg']:.3f} ms (P95: {s1['p95']:.3f} ms)")
            print(f"   • Step2_CriticScoring (碰撞/角度打分): 平均 {s2['avg']:.3f} ms (P95: {s2['p95']:.3f} ms)")
            print(f"   • Step3_SequenceUpdate (控制序列更新): 平均 {s3['avg']:.3f} ms (P95: {s3['p95']:.3f} ms)")
            print(f" ---------------------------------------------------------------------------------------------------------")

    # Per-Critic 打分排名打印
    print(f"\n 🎯 [3/5] 各 Critic 详细耗时排名 (CRITIC BREAKDOWN - 按平均耗时降序)")
    for mode in ["GPU (CUDA)", "CPU"]:
        critics_dict = modes_data[mode]["critics"]
        if not critics_dict:
            continue
        print(f"\n  --- Mode: {mode} ---")
        print(f"  Critic 插件名称           | 样本数 | 平均(ms) | 标准差  | P50(ms) | P95(ms) | 最小(ms) | 最大(ms)")
        print(f"  -------------------------------------------------------------------------------------------------------")
        sorted_critics = []
        for c_name, c_vals in critics_dict.items():
            st = calculate_stats(c_vals)
            sorted_critics.append((c_name, st))
        sorted_critics.sort(key=lambda x: x[1]["avg"], reverse=True)

        for c_name, st in sorted_critics:
            print(f"  {c_name:<25s} | {st['count']:6d} | {st['avg']:8.3f} | {st['std']:7.3f} | {st['p50']:7.3f} | {st['p95']:7.3f} | {st['min']:8.3f} | {st['max']:8.3f}")

    # 控制精度与到位表现打印
    print(f"\n 📐 [4/5] 控制精度、运动行为及到位精度评估 (PRECISION & GOAL METRICS)")
    print(f" ---------------------------------------------------------------------------------------------------------")
    print(f"  评估指标 / 特性                 | CPU 模式 (cuda=0)     | GPU 模式 (cuda=1)")
    print(f" ---------------------------------------------------------------------------------------------------------")
    cpu_pm = calculate_precision_metrics(modes_data["CPU"]["eval_records"], modes_data["CPU"]["goal_records"])
    gpu_pm = calculate_precision_metrics(modes_data["GPU (CUDA)"]["eval_records"], modes_data["GPU (CUDA)"]["goal_records"])

    def fmt_val(m, key, suffix=""):
        if not m or key not in m or m[key] is None:
            return "N/A"
        v = m[key]
        if isinstance(v, float):
            return f"{v:.4f}{suffix}"
        return f"{v}{suffix}"

    print(f"  采样帧数 (Eval Samples)         | {fmt_val(cpu_pm, 'count'):<20} | {fmt_val(gpu_pm, 'count'):<20}")
    print(f"  离轨交叉误差 RMSE (Cross-Track) | {fmt_val(cpu_pm, 'rmse_xte', ' m'):<20} | {fmt_val(gpu_pm, 'rmse_xte', ' m'):<20}")
    print(f"  平均航向误差 MAE (Heading Deg)  | {fmt_val(cpu_pm, 'mae_heading_deg', ' deg'):<20} | {fmt_val(gpu_pm, 'mae_heading_deg', ' deg'):<20}")
    print(f"  最大航向误差 (Max Heading Deg)  | {fmt_val(cpu_pm, 'max_heading_deg', ' deg'):<20} | {fmt_val(gpu_pm, 'max_heading_deg', ' deg'):<20}")
    print(f"  线加速度平滑度 (Linear Accel RMS)| {fmt_val(cpu_pm, 'rms_ax', ' m/s²'):<20} | {fmt_val(gpu_pm, 'rms_ax', ' m/s²'):<20}")
    print(f"  角加速度平滑度 (Angular Acc RMS)| {fmt_val(cpu_pm, 'rms_wz_dot', ' rad/s²'):<20} | {fmt_val(gpu_pm, 'rms_wz_dot', ' rad/s²'):<20}")
    print(f"  缓慢对齐事件数 (Slow Align)    | {fmt_val(cpu_pm, 'slow_align_events'):<20} | {fmt_val(gpu_pm, 'slow_align_events'):<20}")
    print(f"  倒车指令触发数 (Reverse Cmd)   | {fmt_val(cpu_pm, 'reverse_cmd_events'):<20} | {fmt_val(gpu_pm, 'reverse_cmd_events'):<20}")
    print(f"  -------------------------------------------------------------------------------------------------------")
    print(f"  📍 [到位精度] 终点位置残差 (Pos Err) | {fmt_val(cpu_pm, 'final_pos_err', ' m'):<20} | {fmt_val(gpu_pm, 'final_pos_err', ' m'):<20}")
    print(f"  📍 [到位精度] 终点姿态残差 (Yaw Err) | {fmt_val(cpu_pm, 'final_yaw_err_deg', ' deg'):<20} | {fmt_val(gpu_pm, 'final_yaw_err_deg', ' deg'):<20}")
    print(f" ---------------------------------------------------------------------------------------------------------\n")

    # 5. 系统资源消耗评估与双模式横向对比
    print(f" 💻 [5/5] /controller_server 节点系统资源消耗对比 (psutil / ps 实时采集)")
    print(f" ---------------------------------------------------------------------------------------------------------")
    print(f"  系统资源评估指标                | CPU 模式 (cuda=0)     | GPU 模式 (cuda=1)")
    print(f" ---------------------------------------------------------------------------------------------------------")

    cpu_res = modes_data["CPU"].get("resource_stats")
    gpu_res = modes_data["GPU (CUDA)"].get("resource_stats")

    # 若当前运行抓取到了资源，合并到当前模式下显示
    with data_lock:
        if resource_data["cpu_percent"]:
            cur_cpu_st = calculate_stats(resource_data["cpu_percent"])
            cur_mem_st = calculate_stats(resource_data["mem_rss_mb"])
            cur_res = {"cpu": cur_cpu_st, "mem": cur_mem_st}
            # 根据最新数据刷新
            if cpu_stats_opt['count'] > 0:
                cpu_res = cur_res
            elif gpu_stats_opt['count'] > 0:
                gpu_res = cur_res

    def fmt_res(res_obj, field, stat_key='avg', suffix=""):
        if not res_obj or field not in res_obj or not res_obj[field]:
            return "N/A"
        st = res_obj[field]
        val = st.get(stat_key, 0.0)
        if isinstance(val, float):
            return f"{val:.2f}{suffix}"
        return f"{val}{suffix}"

    print(f"  采样数据帧数 (Resource Samples) | {fmt_res(cpu_res, 'cpu', 'count', ' 帧'):<20} | {fmt_res(gpu_res, 'cpu', 'count', ' 帧'):<20}")
    print(f"  CPU 平均使用率 % (Top 对齐)    | {fmt_res(cpu_res, 'cpu', 'avg', '%'):<20} | {fmt_res(gpu_res, 'cpu', 'avg', '%'):<20}")
    print(f"  CPU 峰值使用率 P95 %            | {fmt_res(cpu_res, 'cpu', 'p95', '%'):<20} | {fmt_res(gpu_res, 'cpu', 'p95', '%'):<20}")
    print(f"  内存平均物理占用 (RSS MB)       | {fmt_res(cpu_res, 'mem', 'avg', ' MB'):<20} | {fmt_res(gpu_res, 'mem', 'avg', ' MB'):<20}")
    print(f"  内存峰值物理占用 P95 (RSS MB)   | {fmt_res(cpu_res, 'mem', 'p95', ' MB'):<20} | {fmt_res(gpu_res, 'mem', 'p95', ' MB'):<20}")
    print(f" ---------------------------------------------------------------------------------------------------------\n")

def signal_handler(sig, frame):
    global running
    print("\n🛑 捕获终止信号 (Ctrl+C)，退出实时监控。")
    running = False
    sys.exit(0)

def main():
    parser = argparse.ArgumentParser(description="Analyze MPPI Performance, Precision and System Resource Logs.")
    parser.add_argument("log_file", nargs="?", default="log.log", help="Path to MPPI log file (default: log.log)")
    parser.add_argument("--monitor-resource", action="store_true", help="Enable background CPU/RAM psutil monitoring thread")
    parser.add_argument("--follow", action="store_true", help="Keep running and continuously monitor log and process resources until Ctrl+C")
    parser.add_argument("--reset-history", action="store_true", help="Clear historical benchmark json database")
    parser.add_argument("--gpu-baseline", type=str, default=GPU_BASELINE_FILE, help="Path to GPU baseline json file (default: mppi_gpu_baseline.json)")
    args = parser.parse_args()

    if args.reset_history and os.path.exists(HISTORY_FILE):
        os.remove(HISTORY_FILE)
        print(f"🧹 已重置通用历史数据库 ({HISTORY_FILE})。保留 GPU Baseline 基准。")

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    if args.monitor_resource or args.follow:
        mon_thread = threading.Thread(target=monitor_resources, args=(0.5,), daemon=True)
        mon_thread.start()
        time.sleep(0.5)

    current_data = parse_mppi_log(args.log_file)
    merged_data = load_and_merge_history(current_data, HISTORY_FILE, args.gpu_baseline)
    print_report(merged_data, monitor_resource_flag=(args.monitor_resource or args.follow))

    if args.follow:
        print("🔄 [Follow Mode] 正在保持在线实时监控... (按 Ctrl+C 可停止监控)")
        try:
            while running:
                time.sleep(2.0)
                os.system('clear')
                current_data = parse_mppi_log(args.log_file)
                merged_data = load_and_merge_history(current_data, HISTORY_FILE, args.gpu_baseline)
                print_report(merged_data, monitor_resource_flag=True)
        except KeyboardInterrupt:
            print("\n🛑 实时监控结束。")

if __name__ == "__main__":
    main()
