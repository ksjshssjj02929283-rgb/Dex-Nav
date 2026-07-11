#!/usr/bin/env python3
# Copyright 2026 DEXE Robot Engineer
# SPDX-License-Identifier: Apache-2.0
"""nav_monitor 测试数据生成器：按 nav2_nav_monitor 落盘的同款 JSON schema
生成带真实感变化的导航任务记录，供批量分析脚本演示/回归测试。

模拟场景：机器人在 4 个站点（A/B/C/D）间连续跑任务——站点间路径为
带弯道的贝塞尔曲线，梯形速度剖面+弯道减速，控制器输出带尖峰/平滑后干净，
到达误差 1~5cm 级随机分布，穿插中止/取消任务。

用法：
  ros2 run nav2_nav_monitor gen_test_nav_tasks.py [目录] [--count 10] [--seed 7]
  （目录缺省 ~/.dexe/navigation/nav_monitor_test，避免污染真实记录目录）

生成后可直接：
  ros2 run nav2_nav_monitor analyze_nav_tasks.py <同一目录>
"""
import argparse
import json
import math
import os
import random
import time

STATIONS = {
    "A": (2.0, 1.0, 0.0),
    "B": (8.0, 1.5, math.pi / 2),
    "C": (7.5, 6.0, math.pi),
    "D": (1.5, 5.5, -math.pi / 2),
}
BT_NAME = "navigate_to_pose_w_replanning_and_recovery.xml"


def bezier(p0, p1, ctrl, u):
    """二次贝塞尔 + 切向 yaw。"""
    x = (1 - u) ** 2 * p0[0] + 2 * (1 - u) * u * ctrl[0] + u ** 2 * p1[0]
    y = (1 - u) ** 2 * p0[1] + 2 * (1 - u) * u * ctrl[1] + u ** 2 * p1[1]
    dx = 2 * (1 - u) * (ctrl[0] - p0[0]) + 2 * u * (p1[0] - ctrl[0])
    dy = 2 * (1 - u) * (ctrl[1] - p0[1]) + 2 * u * (p1[1] - ctrl[1])
    return x, y, math.atan2(dy, dx)


def route_of(a, b, rng):
    """站点 a→b 的路径函数（带随机侧偏控制点）。"""
    p0, p1 = STATIONS[a][:2], STATIONS[b][:2]
    mx, my = (p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    n = math.hypot(dx, dy) or 1.0
    off = rng.uniform(0.8, 1.8) * rng.choice([-1, 1])
    ctrl = (mx - dy / n * off, my + dx / n * off)
    return lambda u: bezier(p0, p1, ctrl, u)


def curvature_zone(u):
    """弯道减速带（贝塞尔中段曲率最大）。"""
    return 0.55 if 0.35 < u < 0.65 else 1.0


def speed_profile(u, vmax):
    v = vmax
    if u < 0.12:
        v *= u / 0.12
    elif u > 0.88:
        v *= (1 - u) / 0.12
    return max(v * curvature_zone(u), 0.02)


def gen_task(a, b, result, t_wall, rng):
    route = route_of(a, b, rng)
    goal = STATIONS[b]
    vmax = rng.uniform(0.55, 0.85)

    # 路径长度（数值积分）
    pts = [route(u / 200) for u in range(201)]
    plan_len = sum(
        math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        for i in range(1, len(pts)))

    # 结束进度：succeeded=1.0，aborted/canceled 中途停
    end_u = 1.0 if result == "succeeded" else rng.uniform(0.35, 0.8)
    # 用 v(u) 反推总时长：dt = ds/v
    total_t, seg = 0.0, 200
    for i in range(1, int(seg * end_u) + 1):
        ds = math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        total_t += ds / speed_profile((i - 0.5) / seg, vmax)

    align_t = rng.uniform(0.4, 2.2)
    approach_t = rng.uniform(1.0, 2.8) if result == "succeeded" else 0.0
    track_t = total_t
    duration = align_t + track_t + approach_t

    def u_at(t_rel):
        """track 段内时间→路径进度（按剖面推进）。"""
        if t_rel <= align_t:
            return 0.0
        acc, tt = 0.0, t_rel - align_t
        for i in range(1, int(seg * end_u) + 1):
            ds = math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
            acc += ds / speed_profile((i - 0.5) / seg, vmax)
            if acc >= tt:
                return i / seg
        return end_u

    # 到达点：succeeded 带 goal 系纵/横误差（纵向略偏正=停车过冲倾向，横向更小）
    if result == "succeeded":
        e_lon = rng.gauss(0.012, 0.02)
        e_lat = rng.gauss(0.0, 0.009)
        gyaw = goal[2]
        ex = e_lon * math.cos(gyaw) - e_lat * math.sin(gyaw)
        ey = e_lon * math.sin(gyaw) + e_lat * math.cos(gyaw)
        eyaw = rng.gauss(0, math.radians(1.2))
        reached = (goal[0] + ex, goal[1] + ey, goal[2] + eyaw)
    else:
        rx, ry, ryaw = route(end_u)
        reached = (rx, ry, ryaw)
        ex, ey = reached[0] - goal[0], reached[1] - goal[1]
        eyaw = reached[2] - goal[2]

    # 采样流（弯道转向方向任务级固定，真实机器人一条弯不会来回换向）
    turn_dir = rng.choice([-1.0, 1.0])
    passing, desired, cmd, smooth, odom = [], [], [], [], []
    odom_lag = rng.uniform(0.10, 0.22)  # odom 相对指令的响应滞后(s)
    t, dt_v, dt_p = 0.0, 0.05, 0.5
    next_p = 0.0
    while t <= duration:
        u = min(u_at(t), end_u)
        x, y, yaw = route(u)
        in_track = align_t < t <= align_t + track_t
        v = speed_profile(u, vmax) if in_track else (
            0.12 if t > align_t + track_t else 0.0)
        wz = (0.45 * turn_dir if 0.35 < u < 0.65 and in_track
              else (0.3 if t <= align_t else 0.0))
        spike_v = v * (1.0 + (0.18 if rng.random() < 0.12 else 0.0))
        desired.append([round(t, 3),
                        round(speed_profile(min(u_at(t), end_u), vmax), 4), 0.0])
        cmd.append([round(t, 3), round(spike_v + rng.gauss(0, 0.008), 4),
                    round(wz * (1.0 + (0.2 if rng.random() < 0.1 else 0.0)), 4)])
        smooth.append([round(t, 3), round(v, 4), round(wz, 4)])
        # odom：平滑指令的滞后跟随 + 实测噪声
        u_lag = min(u_at(max(t - odom_lag, 0.0)), end_u)
        in_track_lag = align_t < (t - odom_lag) <= align_t + track_t
        v_lag = speed_profile(u_lag, vmax) if in_track_lag else (
            0.12 if (t - odom_lag) > align_t + track_t else 0.0)
        wz_lag = (0.45 * turn_dir if 0.35 < u_lag < 0.65 and in_track_lag
                  else (0.3 if (t - odom_lag) <= align_t and t > odom_lag else 0.0))
        odom.append([round(t, 3), round(v_lag + rng.gauss(0, 0.012), 4),
                     round(wz_lag + rng.gauss(0, 0.015), 4)])
        if t >= next_p:
            passing.append([round(t, 3), round(x, 4), round(y, 4), round(yaw, 4)])
            next_p += dt_p
        t += dt_v
    passing.append([round(duration, 3), round(reached[0], 4),
                    round(reached[1], 4), round(reached[2], 4)])

    pass_len = sum(
        math.hypot(passing[i][1] - passing[i - 1][1], passing[i][2] - passing[i - 1][2])
        for i in range(1, len(passing)))

    # 规划路径（首条 + 随机重规划截段）
    replans = rng.choice([0, 0, 1, 1, 2, 3])
    planned = []
    for k in range(replans + 1):
        u0 = 0.0 if k == 0 else rng.uniform(0.1, 0.6) * end_u
        planned.append([
            [round(v, 4) for v in route(u0 + (1 - u0) * i / 40)] for i in range(41)])

    phases = [[0.0, "align"], [round(align_t, 3), "track"]]
    if approach_t > 0:
        phases.append([round(align_t + track_t, 3), "approach"])

    def stats(samples):
        av = [abs(s[1]) for s in samples]
        aw = [abs(s[2]) for s in samples]
        return {
            "avg_linear_speed": round(sum(av) / len(av), 4),
            "max_linear_speed": round(max(av), 4),
            "avg_angular_speed": round(sum(aw) / len(aw), 4),
            "max_angular_speed": round(max(aw), 4),
        }

    start = STATIONS[a]
    recoveries = rng.choice([0, 0, 0, 1, 1, 2]) if result != "succeeded" else \
        rng.choice([0, 0, 0, 0, 1])
    return {
        "start_time": t_wall,
        "end_time": t_wall + duration,
        "start_pose": {"x": start[0], "y": start[1], "yaw": start[2]},
        "goal_pose": {"x": goal[0], "y": goal[1], "yaw": goal[2]},
        "behavior_tree": BT_NAME,
        "result": result,
        "number_of_recoveries": recoveries,
        "distance_remaining_final": round(
            0.02 if result == "succeeded" else plan_len * (1 - end_u), 3),
        "navigation_time_final": round(duration, 2),
        "goal_reached_pose": {
            "x": round(reached[0], 4), "y": round(reached[1], 4),
            "yaw": round(reached[2], 4)},
        "goal_reached_error": {
            "x": round(ex, 4), "y": round(ey, 4), "yaw": round(eyaw, 4),
            "distance": round(math.hypot(ex, ey), 4)},
        "planned_path_length_first": round(plan_len, 3),
        "replanning_count": replans + 1,
        "passing_length": round(pass_len, 3),
        "desired_vel": {"topic": "/nav_monitor/desired_velocity",
                        **stats(desired), "samples": desired},
        "cmd_vel": {"topic": "cmd_vel_nav", **stats(cmd), "samples": cmd},
        "smoothed_vel": {"topic": "control/cmd_vel_nav", **stats(smooth),
                         "samples": smooth},
        "odom_vel": {"topic": "feedback/odom", **stats(odom), "samples": odom},
        "phase_transitions": phases,
        "passing_path": passing,
        "planned_paths": planned,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument(
        "dir", nargs="?", default="~/.dexe/navigation/nav_monitor_test",
        help="输出目录（默认 ~/.dexe/navigation/nav_monitor_test，独立于真实记录）")
    ap.add_argument("--count", type=int, default=10)
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()

    rng = random.Random(args.seed)
    d = os.path.expanduser(args.dir)
    os.makedirs(d, exist_ok=True)

    names = list(STATIONS)
    cur = "A"
    # 结果序列：约 8 成功 + 1 aborted + 1 canceled（按 count 比例）
    results = ["succeeded"] * args.count
    if args.count >= 5:
        results[args.count // 3] = "aborted"
    if args.count >= 8:
        results[2 * args.count // 3] = "canceled"

    t_wall = time.time() - args.count * 120  # 每 2 分钟一次任务往前排
    for i in range(args.count):
        nxt = rng.choice([s for s in names if s != cur])
        task = gen_task(cur, nxt, results[i], t_wall, rng)
        ts = time.strftime("%Y%m%d_%H%M%S", time.localtime(t_wall))
        fn = os.path.join(d, f"nav_task_{ts}_{results[i]}.json")
        with open(fn, "w") as f:
            json.dump(task, f, indent=2, sort_keys=True)
        print(f"{os.path.basename(fn)}  {cur}→{nxt}  "
              f"时长={task['navigation_time_final']}s  "
              f"误差={task['goal_reached_error']['distance']}m")
        if results[i] == "succeeded":
            cur = nxt  # 到达才切换当前站点
        t_wall += 120 + rng.uniform(-20, 20)

    print(f"\n共 {args.count} 条 → {d}")
    print(f"查看效果：ros2 run nav2_nav_monitor analyze_nav_tasks.py {args.dir}")


if __name__ == "__main__":
    main()
