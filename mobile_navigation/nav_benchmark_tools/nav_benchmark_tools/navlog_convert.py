#!/usr/bin/env python3
"""nbt navlog convert：把路网任务（path_navigation）的录包转成 nav2_nav_monitor 同格式的任务文件 nav_task_*.json。

nav_monitor 只记录 /navigate_to_pose 的任务；调度规划器的 feedback 没有位姿，接不上。这里离线补齐：
  任务边界   /navigate_through_graph_points/_action/status（受理 → 终态）
  目标位姿   tasks.csv 的终点名 → targets.json（地图资产）；缺失时取最后一条 /tracking_path 末点 + target_yaw
  实走路径   /tf 复合出的 map → 底盘
  速度       跟踪器 /cmd_vel_nav_path、平滑器 /control/cmd_vel_nav、底盘指令 /control/cmd_vel、里程计 /feedback/odom
  到点误差   停稳后的误差（goal_reached_error），另记上报成功时刻的误差（report_reached_error）
生成的文件可直接喂给 nav2_nav_monitor 的 analyze_nav_tasks.py / plot_nav_task_path.py / plot_nav_task_speed.py。

  nbt navlog convert <录包目录> -o <输出目录> [--tasks tasks.csv] [--targets targets.json]
  nbt navlog convert <会话目录>               # 会话目录 = nbt pathnav run 的输出，自动找 rosbag/ tasks.csv targets.json
"""
import argparse
import json
import math
import os
import sys
import time

from nav_benchmark_tools import navlog as nl

PASSING_PERIOD = 0.1
PLANNED_STEP = 0.05


def _vel_block(topic, samples):
    n = len(samples) or 1
    return {"topic": topic,
            "avg_linear_speed": sum(abs(s[1]) for s in samples) / n,
            "max_linear_speed": max((abs(s[1]) for s in samples), default=0.0),
            "avg_angular_speed": sum(abs(s[2]) for s in samples) / n,
            "max_angular_speed": max((abs(s[2]) for s in samples), default=0.0),
            "samples": [[round(s[0], 4), round(s[1], 5), round(s[2], 5)] for s in samples]}


def _downsample(poses, step):
    out, acc = [], None
    for p in poses:
        if acc is None or math.hypot(p[0] - acc[0], p[1] - acc[1]) >= step:
            out.append(p)
            acc = p
    if poses and out[-1] is not poses[-1]:
        out.append(poses[-1])
    return out


def _pose_dict(p):
    return {"x": p[0], "y": p[1], "yaw": p[2]} if p else {"x": 0.0, "y": 0.0, "yaw": 0.0}


def build_tasks(d, rows=None, targets=None, limits=None, opts=None):
    """返回任务列表（绝对时间），每个元素含 nav_monitor 字段所需的全部原料。"""
    o = dict(nl.DEFAULTS, **(opts or {}))
    goals = nl.goal_windows(d["status"])
    matched = nl.match_rows(goals, rows or [])
    targets = targets or {}
    tasks = []
    for k, g in enumerate(goals):
        row = matched.get(g["uuid"], {})
        t0 = g["t_accept"]
        t_rep = g["t_report"] if g["t_report"] is not None else d["t_last"]
        next_t0 = goals[k + 1]["t_accept"] if k + 1 < len(goals) else None
        t_settle, settled = (None, None)
        if g["result"] == "succeeded":
            t_settle, settled = nl.settle_and_pose(d, t_rep, o)
            if t_settle is not None and next_t0 is not None and t_settle > next_t0:
                t_settle, settled = None, None          # 下一任务已经开始，说明没等停稳
        t_end = t_settle + o["avg_window"] if t_settle is not None else t_rep
        tracking = [tp for tp in d["tracking"] if t0 - 0.5 <= tp["t"] <= t_rep]
        points = [p for p in (row.get("points") or "").split(">") if p]
        end_name = points[-1] if points else ""
        goal, goal_src = None, ""
        if end_name and end_name in targets:
            tg = targets[end_name]
            goal, goal_src = (tg["x"], tg["y"], tg["theta"]), "assets"
        elif tracking and tracking[-1]["poses"]:
            last = tracking[-1]
            goal, goal_src = (last["poses"][-1][0], last["poses"][-1][1], last["target_yaw"]), "tracking_path"
        start = nl.interp_pose(d["pose"], t0)
        reported = nl.interp_pose(d["pose"], t_rep)
        tasks.append({
            "uuid": g["uuid"], "result": g["result"] or "unknown", "t_accept": t0, "t_report": t_rep,
            "t_settle": t_settle, "t_end": t_end, "row": row, "points": points, "end_name": end_name,
            "goal": goal, "goal_source": goal_src, "start": start, "reported": reported, "settled": settled,
            "tracking": tracking, "limits": limits or {}})
    return tasks


def task_json(task, d):
    """一个任务 → nav_monitor schema_version 2 兼容的 dict（时间字段相对受理时刻）。"""
    t0, t_end = task["t_accept"], task["t_end"]

    def rel(series):
        return [(s[0] - t0,) + tuple(s[1:]) for s in nl.window(series, t0, t_end)]

    path = nl.window(d["pose"], t0, t_end)
    passing, last = [], None
    for s in path:
        if last is None or s[0] - last >= PASSING_PERIOD:
            passing.append([round(s[0] - t0, 3), s[1], s[2], s[3]])
            last = s[0]
    goal = task["goal"]
    err = nl.target_error(*task["settled"], *goal) if task["settled"] and goal else {}
    rep_err = nl.target_error(*task["reported"], *goal) if task["reported"] and goal else {}
    follower = rel(d["levels"]["follower"])
    times = [s[0] for s in follower]
    acc = nl.local_slope(times, [s[1] for s in follower], 0.3) if len(times) >= 3 else []
    target = [[round(t, 4), round(v, 5), round(a, 5)] for (t, v, _), a in zip(follower, acc) if a is not None]
    phases, last_name = [], None
    events = sorted([(tb, f"planner:{st}") for tb, u, st, _, _ in d["feedback"] if u == task["uuid"]] +
                    [(tb, f"follower:{nl.NAV_STATUS.get(st, st)}") for tb, st, _ in d["nav_status"]])
    for tb, name in events:
        if t0 <= tb <= t_end and name != last_name:
            phases.append([round(tb - t0, 3), name])
            last_name = name
    planned = [[[round(x, 4), round(y, 4), round(w, 4)] for x, y, w in _downsample(tp["poses"], PLANNED_STEP)]
               for tp in task["tracking"]]
    lengths = [nl.path_length(tp["poses"]) for tp in task["tracking"]]
    row = task["row"]
    lim = task["limits"]
    obstacle, prev = [], None
    for t, st in nl.window(d["obstacle"], t0, t_end):
        if st != prev:
            obstacle.append([round(t - t0, 3), st])
            prev = st
    return {
        "schema_version": 2, "producer": "nbt navlog convert", "task_source": "schedule_planner",
        "start_time": t0, "end_time": task["t_report"], "result": task["result"],
        "start_pose": _pose_dict(task["start"]), "goal_pose": _pose_dict(goal), "goal_source": task["goal_source"],
        "goal_reached_pose": _pose_dict(task["settled"]), "goal_reached_error": err,
        "goal_reached_error_basis": "settled" if task["settled"] else "unavailable",
        "report_reached_pose": _pose_dict(task["reported"]), "report_reached_error": rep_err,
        "settle": {"report_rel": round(task["t_report"] - t0, 3),
                   "settle_rel": round(task["t_settle"] - t0, 3) if task["t_settle"] else None,
                   "delay": round(task["t_settle"] - task["t_report"], 3) if task["t_settle"] else None,
                   "status": "ok" if task["t_settle"] else ("timeout" if task["result"] == "succeeded" else "n/a")},
        "number_of_recoveries": 0, "replanning_count": 0, "distance_remaining_final": 0.0,
        "navigation_time_final": round(task["t_report"] - t0, 3),
        "planned_path_length_first": round(sum(lengths), 4) if lengths else 0.0,
        "passing_length": round(nl.path_length([(s[1], s[2]) for s in path]), 4),
        "motion_limits": {"max_acceleration": lim.get("accel"), "max_deceleration": lim.get("decel"),
                          "max_abs_jerk": lim.get("jerk"), "tolerance_ratio": 0.1,
                          "source": lim.get("source", "not_provided")},
        "desired_vel": _vel_block("/cmd_vel_nav_path", follower),
        "controller_target": {"topic": "/cmd_vel_nav_path", "acceleration_source": "estimated_from_velocity_0.3s",
                              "samples": target},
        "cmd_vel": _vel_block("/control/cmd_vel", rel(d["levels"]["mux"])),
        "smoothed_vel": _vel_block("/control/cmd_vel_nav", rel(d["levels"]["smoother"])),
        "odom_vel": _vel_block(nl.ODOM, rel(d["levels"]["odom"])),
        "phase_transitions": phases, "passing_path": passing, "planned_paths": planned,
        "planned_path_ids": [None] * len(planned), "obstacle_transitions": obstacle,
        "schedule_planner": {
            "run_id": row.get("run_id", ""), "batch": row.get("batch", ""), "lap": row.get("lap", ""),
            "task_id": row.get("task", ""), "points": task["points"], "goal_id": task["uuid"],
            "tracking_groups": [{"t_rel": round(tp["t"] - t0, 3), "length": round(L, 4), "mode": tp["mode"],
                                 "backward": tp["backward"], "max_speed": tp["max_speed"],
                                 "target_yaw": tp["target_yaw"]} for tp, L in zip(task["tracking"], lengths)],
            "marking": row.get("mark", "")},
    }


def file_name(task):
    ts = time.strftime("%Y%m%d_%H%M%S", time.localtime(task["t_accept"]))
    rid = task["row"].get("run_id") or task["uuid"][:8]
    return f"nav_task_{ts}_{rid}_{task['result']}.json"


def resolve_inputs(path, tasks_csv="", targets=""):
    """会话目录 → (录包, tasks.csv, targets.json)；录包目录原样返回。"""
    if os.path.exists(os.path.join(path, "metadata.yaml")):
        return path, tasks_csv, targets
    bag = os.path.join(path, "rosbag")
    if not os.path.exists(os.path.join(bag, "metadata.yaml")):
        raise SystemExit(f"{path} 既不是录包目录，也不是含 rosbag/ 的会话目录")
    tasks_csv = tasks_csv or os.path.join(path, "tasks.csv")
    if not targets:
        for cand in (os.path.join(path, "targets.json"), os.path.join(path, "..", "..", "targets.json")):
            if os.path.exists(cand):
                targets = cand
                break
    return bag, tasks_csv, targets


def load_limits(session_dir):
    """从会话的 run.json 读跟踪器加速度 / 减速度 / jerk（nbt pathnav run 开始时读的参数）。"""
    p = os.path.join(session_dir, "run.json")
    if not os.path.exists(p):
        return {}
    with open(p, encoding="utf-8") as f:
        prm = json.load(f).get("params", {})
    f = prm.get("/path_follower_node", {})
    return {k: f.get(n) for k, n in (("accel", "max_linear_acceleration"), ("decel", "max_linear_deceleration"),
                                     ("jerk", "max_linear_jerk"))} | {"source": "run.json /path_follower_node"}


def add_args(ap):
    ap.add_argument("--tasks", default="", help="nbt pathnav run 写的 tasks.csv（按 goal_id 对任务号与点序列）")
    ap.add_argument("--targets", default="", help="targets.json（nbt pathnav targets 读的地图资产点位）")
    ap.add_argument("--base-frame", default="", help="底盘坐标系，默认取 odom 下发布最频繁的子坐标系")
    ap.add_argument("--still-v", type=float, default=nl.DEFAULTS["still_v"], help="停稳线速度阈值 m/s")
    ap.add_argument("--still-w", type=float, default=nl.DEFAULTS["still_w"], help="停稳角速度阈值 rad/s")
    ap.add_argument("--still-hold", type=float, default=nl.DEFAULTS["still_hold"], help="停稳保持时间 s")
    ap.add_argument("--avg-window", type=float, default=nl.DEFAULTS["avg_window"], help="停稳后位姿平均窗 s")


def opts_of(a):
    return {"still_v": a.still_v, "still_w": a.still_w, "still_hold": a.still_hold, "avg_window": a.avg_window,
            "base_frame": a.base_frame}


def main(argv=None):
    ap = argparse.ArgumentParser(prog="nbt navlog convert", description="路网任务录包 → nav_monitor 格式任务文件")
    ap.add_argument("path", help="录包目录，或 nbt pathnav run 的会话目录")
    ap.add_argument("-o", "--out", default="", help="输出目录，默认 <会话目录>/nav_tasks 或 <录包>_nav_tasks")
    add_args(ap)
    a = ap.parse_args(argv)
    bag, tasks_csv, targets = resolve_inputs(os.path.abspath(a.path), a.tasks, a.targets)
    session = os.path.dirname(bag) if bag != os.path.abspath(a.path) else ""
    out = a.out or (os.path.join(session, "nav_tasks") if session else bag.rstrip("/") + "_nav_tasks")
    d = nl.read_bag(bag, opts_of(a))
    for s in d["skipped"]:
        print(f"跳过 {s}", file=sys.stderr)
    if not d["status"]:
        print(f"✗ 录包里没有 {nl.STATUS_TOPIC}（录包时要加 --include-hidden-topics）")
        return 2
    if not d["pose"]:
        print(f"✗ 录包里 /tf 复合不出 map → 底盘（底盘坐标系：{d['base_frame'] or '未找到'}）")
        return 2
    tasks = build_tasks(d, nl.read_tasks_csv(tasks_csv), nl.load_targets(targets),
                        load_limits(session) if session else {}, opts_of(a))
    os.makedirs(out, exist_ok=True)
    for t in tasks:
        with open(os.path.join(out, file_name(t)), "w", encoding="utf-8") as f:
            json.dump(task_json(t, d), f, ensure_ascii=False, indent=1)
    ok = sum(1 for t in tasks if t["result"] == "succeeded")
    nosettle = sum(1 for t in tasks if t["result"] == "succeeded" and t["t_settle"] is None)
    print(f"✓ {len(tasks)} 个任务（成功 {ok}，其中未判到停稳 {nosettle}）→ {out}")
    print(f"  底盘坐标系 {d['base_frame']}；目标位姿来源："
          f"地图资产 {sum(1 for t in tasks if t['goal_source'] == 'assets')}，"
          f"/tracking_path {sum(1 for t in tasks if t['goal_source'] == 'tracking_path')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
