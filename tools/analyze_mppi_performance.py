#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MPPI Controller: CPU vs CUDA Performance Comparison & Log Analysis Tool
"""

import sys
import re
import argparse
from typing import List, Dict

def parse_mppi_logs(log_filepath: str):
    cpu_timings = []
    cuda_timings = []

    mppi_timing_pattern = re.compile(
        r'MPPI timing \(ms\): transform=([\d\.]+)\s+wait_costmap_lock=([\d\.]+)\s+optimize=([\d\.]+)\s+visualize=([\d\.]+)\s+total=([\d\.]+)'
    )
    optimizer_pattern = re.compile(
        r'Optimizer iteration \d+ timing \(ms\): generate=([\d\.]+)\s+critics=([\d\.]+)\s+update=([\d\.]+)\s+total=([\d\.]+)'
    )
    cuda_step1_pattern = re.compile(r'\[MPPI CUDA step-1\].*in ([\d\.]+) ms')
    cuda_step2_pattern = re.compile(r'\[MPPI CUDA step-2\].*in ([\d\.]+) ms')

    cuda_step1_times = []
    cuda_step2_times = []

    with open(log_filepath, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            # Check for timing warnings or log throttles
            m_timing = mppi_timing_pattern.search(line)
            if m_timing:
                tf, lock, opt, vis, tot = map(float, m_timing.groups())
                if '[MPPI CUDA' in line or 'CUDA step' in line:
                    cuda_timings.append((tf, lock, opt, vis, tot))
                else:
                    cpu_timings.append((tf, lock, opt, vis, tot))

            m_s1 = cuda_step1_pattern.search(line)
            if m_s1:
                cuda_step1_times.append(float(m_s1.group(1)))

            m_s2 = cuda_step2_pattern.search(line)
            if m_s2:
                cuda_step2_times.append(float(m_s2.group(1)))

    return cpu_timings, cuda_timings, cuda_step1_times, cuda_step2_times

def print_stats(name: str, values: List[float]):
    if not values:
        print(f"  {name}: N/A (No records found)")
        return
    values.sort()
    n = len(values)
    avg = sum(values) / n
    min_v = values[0]
    max_v = values[-1]
    p50 = values[int(n * 0.5)]
    p95 = values[int(n * 0.95)] if n > 20 else max_v

    print(f"  {name:25s} | Avg: {avg:7.2f} ms | P50: {p50:7.2f} ms | P95: {p95:7.2f} ms | Max: {max_v:7.2f} ms")

def analyze(log_file: str):
    print("=" * 80)
    print(f"📊 MPPI Controller Log Analysis Report: {log_file}")
    print("=" * 80)

    cpu_timings, cuda_timings, cuda_step1, cuda_step2 = parse_mppi_logs(log_file)

    print(f"\n[1] Overall Execution Summary:")
    print(f"  Total CPU Mode Timeout Logs (>=100ms)  : {len(cpu_timings)}")
    print(f"  Total CUDA Mode Log Records          : {len(cuda_timings)}")
    print(f"  CUDA Step-1 (Trajectory Rollout) Logs: {len(cuda_step1)}")
    print(f"  CUDA Step-2 (Obstacle Scoring) Logs  : {len(cuda_step2)}")

    if cuda_step1:
        print(f"\n[2] CUDA Kernel Step Micro-benchmarks:")
        print_stats("Step-1 Rollout (GPU)", cuda_step1)
        if cuda_step2:
            print_stats("Step-2 Scoring (GPU)", cuda_step2)

    if cpu_timings:
        print(f"\n[3] CPU Mode Performance Breakdown:")
        tot_list = [t[4] for t in cpu_timings]
        opt_list = [t[2] for t in cpu_timings]
        lock_list = [t[1] for t in cpu_timings]
        tf_list = [t[0] for t in cpu_timings]
        print_stats("Total Cycle (CPU)", tot_list)
        print_stats("Optimize Execution", opt_list)
        print_stats("Wait Costmap Lock", lock_list)
        print_stats("Transform & Prune", tf_list)

    if cuda_timings:
        print(f"\n[4] CUDA Mode Performance Breakdown:")
        tot_list = [t[4] for t in cuda_timings]
        opt_list = [t[2] for t in cuda_timings]
        lock_list = [t[1] for t in cuda_timings]
        print_stats("Total Cycle (CUDA)", tot_list)
        print_stats("Optimize Execution", opt_list)
        print_stats("Wait Costmap Lock", lock_list)

    if cpu_timings and cuda_timings:
        cpu_avg_opt = sum(t[2] for t in cpu_timings) / len(cpu_timings)
        cuda_avg_opt = sum(t[2] for t in cuda_timings) / len(cuda_timings)
        speedup = cpu_avg_opt / cuda_avg_opt if cuda_avg_opt > 0 else 1.0
        print(f"\n[5] Speedup Comparison:")
        print(f"  🚀 Optimization Speedup (CPU vs CUDA): {speedup:.2f}x Faster!")

    print("=" * 80)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_mppi_performance.py <path_to_log_file>")
        sys.exit(1)
    analyze(sys.argv[1])
