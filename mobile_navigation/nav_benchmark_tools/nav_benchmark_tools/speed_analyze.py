#!/usr/bin/env python3
"""nbt speed analyze：能力摸底速度测试的离线分析（推荐速度、直线最大速度两份测试方案的指标）。

输入：批次目录（nbt pathnav run 的输出，含 sessions/*/rosbag、tasks.csv、targets.json、run.json）或单个会话目录。
输出（默认 <批次目录>/analysis/）：
  summary.md          关键点位汇总、每次停靠明细（记录表 1）、每任务速度（记录表 2）、关键路段、口径说明
  points.csv          每个关键点位：样本数、划线合格数、导航精度 mean / P95 / max、重复导航精度（标准差、极差）
  stops.csv           每次停在关键点位一行（停稳后导航反馈 + 人工录入的划线结果）
  tasks_metrics.csv   每任务：段时间、路线长度、平均 / 最大速度、峰值加减速度与 jerk、四级速度峰值与限制环节
  segment.csv         关键路段（计划里的 segment）每次通过：最高速度、平均速度、在目标 ±5% 内的持续时间
  joints.csv          满载姿态：各关节相对会话开始姿态的最大偏差（录了 /feedback/joint 时）
  legs.csv            各路段实测速度：每个路段的最高速度、在目标 ±5% 内的持续时间，看哪些路段真能跑到目标速度
  speed_curves.html   速度曲线叠加：每个任务每一圈一条线画在同一张图里（横轴路程 / 时间，四级速度可切），关键路段单独一张
  nav_tasks/          nav2_nav_monitor 同格式任务文件，可再用它的 analyze_nav_tasks.py 出图

  nbt speed analyze ~/nbt_eval/capability/E075
  nbt speed analyze ~/nbt_eval/capability/E075 --target-speed 0.75 --nav-monitor   # 顺带跑 nav_monitor 的出图脚本
口径：导航反馈 = 停稳（线速度 < 0.01 m/s 且角速度 < 0.02 rad/s 持续 0.5 s）后 1 s 内 map → 底盘的平均位姿，
按目标点朝向分解成纵向 / 横向 / 朝向；加速度由里程计速度局部线性拟合（窗 --accel-window），jerk 再拟合一次
（窗 --jerk-window），同时给底盘 IMU 纵向加速度（0.2 s 滑动平均、减去任务开始前静止段均值）。受干扰的停靠单列，不进统计。
"""
import argparse
import glob
import json
import math
import os
import re
import subprocess
import sys

import yaml

from nav_benchmark_tools import common
from nav_benchmark_tools import navlog as nl
from nav_benchmark_tools import navlog_convert as nc
from nav_benchmark_tools import speed_curves as sc

STAGE_TEXT = {k: name for k, _, name in nl.LEVELS}


def pct(vals, q):
    """线性插值分位数；空返回 None。"""
    if not vals:
        return None
    s = sorted(vals)
    k = (len(s) - 1) * q
    lo, hi = math.floor(k), math.ceil(k)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def std(vals):
    if len(vals) < 2:
        return None
    m = sum(vals) / len(vals)
    return math.sqrt(sum((v - m) ** 2 for v in vals) / (len(vals) - 1))


def fmt(v, nd=1, scale=1.0):
    return "—" if v is None else f"{v * scale:.{nd}f}"


def dynamics(series, accel_win, jerk_win):
    """[(t, v, w)] → (峰值加速度, 峰值减速度(正数), |jerk| 最大)，按速度大小 |v| 求导（倒车也适用）。"""
    if len(series) < 5:
        return None, None, None
    ts = [s[0] for s in series]
    sp = [abs(s[1]) for s in series]
    acc = nl.local_slope(ts, sp, accel_win)
    pa = [(t, a) for t, a in zip(ts, acc) if a is not None]
    if len(pa) < 3:
        return None, None, None
    jerk = nl.local_slope([p[0] for p in pa], [p[1] for p in pa], jerk_win)
    return (max(0.0, max(p[1] for p in pa)), max(0.0, -min(p[1] for p in pa)),
            max((abs(j) for j in jerk if j is not None), default=None))


def imu_peaks(imu, t0, t1):
    """IMU 纵向加速度：0.2 s 滑动平均，减去 t0 前 0.5 s 静止均值；返回 (正峰, 负峰的绝对值)。"""
    pre = [a for t, a in nl.window(imu, t0 - 0.5, t0)]
    seg = nl.window(imu, t0, t1)
    if not seg:
        return None, None
    bias = sum(pre) / len(pre) if pre else 0.0
    out, lo, acc = [], 0, 0.0
    for i, (t, a) in enumerate(seg):
        acc += a
        while seg[lo][0] < t - 0.2:
            acc -= seg[lo][1]
            lo += 1
        out.append(acc / (i - lo + 1) - bias)
    return max(0.0, max(out)), max(0.0, -min(out))


def level_peaks(d, t0, t1):
    return {k: max((abs(s[1]) for s in nl.window(d["levels"][k], t0, t1)), default=None) for k, _, _ in nl.LEVELS}


def limit_stage(peaks, target, ratio=0.95):
    """四级速度依次看，第一个峰值 < 目标 × ratio 的环节；都达到返回「无」；缺数据的级跳过。"""
    if not target:
        return ""
    for k, _, name in nl.LEVELS:
        p = peaks.get(k)
        if p is None:
            continue
        if p < target * ratio:
            return name
    return "无（达到目标）"


def band_time(series, target, tol=0.05):
    """|v| 在 target ±tol 内：(最长连续时长, 累计时长)。"""
    lo, hi = target * (1 - tol), target * (1 + tol)
    best = total = 0.0
    run_start, prev = None, None
    for t, v, _ in series:
        inside = lo <= abs(v) <= hi
        if prev is not None and inside and run_start is not None:
            total += t - prev
        if inside and run_start is None:
            run_start = t
        if not inside and run_start is not None:
            best = max(best, prev - run_start)
            run_start = None
        prev = t
    if run_start is not None and prev is not None:
        best = max(best, prev - run_start)
    return best, total


def segment_window(pose, t0, t1, a, b, lat_tol=0.5, end_tol=0.2):
    """任务窗内沿 a→b 的通过时段：先到 s ≤ end_tol（靠近 a），再到 s ≥ L − end_tol（靠近 b），横向偏差 ≤ lat_tol。
    返回 (进入时刻, 离开时刻, 路段长 L, 期间沿路段前进的距离)；平均速度用前进距离 / 用时，不用 L。"""
    ax, ay, bx, by = a["x"], a["y"], b["x"], b["y"]
    L = math.hypot(bx - ax, by - ay)
    if L < 1e-6:
        return None
    ux, uy = (bx - ax) / L, (by - ay) / L
    t_in = s_in = None
    for t, x, y, _ in nl.window(pose, t0, t1):
        s = (x - ax) * ux + (y - ay) * uy
        lat = -(x - ax) * uy + (y - ay) * ux
        if abs(lat) > lat_tol or s < -end_tol or s > L + end_tol:
            continue
        if t_in is None and s <= end_tol:
            t_in, s_in = t, s
        elif t_in is not None and s >= L - end_tol:
            return t_in, t, L, s - s_in
    return None


def segment_dir(points, seg):
    """任务点序列里先后出现 seg 两点（可以隔着途经点，如合并的共线路段）时返回行驶方向 (起, 终)，正反向都认；
    没有返回 None。"""
    a, b = seg
    for x, y in ((a, b), (b, a)):
        if x in points and y in points[points.index(x) + 1:]:
            return x, y
    return None


def joint_ref(joints, exclude):
    """会话开头 1 s 各关节中位数作参考姿态；名字匹配 exclude 的（轮子、舵）不算。"""
    if not joints:
        return {}
    t0 = joints[0][0]
    vals = {}
    for t, names, pos in joints:
        if t > t0 + 1.0:
            break
        for n, p in zip(names, pos):
            if not re.search(exclude, n, re.I):
                vals.setdefault(n, []).append(p)
    return {n: sorted(v)[len(v) // 2] for n, v in vals.items()}


def joint_dev(joints, ref, t0, t1):
    dev = {}
    for _, names, pos in nl.window(joints, t0, t1):
        for n, p in zip(names, pos):
            if n in ref:
                dev[n] = max(dev.get(n, 0.0), abs(p - ref[n]))
    return dev


def sessions_of(path):
    path = os.path.abspath(os.path.expanduser(path))
    if os.path.exists(os.path.join(path, "tasks.csv")):
        return os.path.dirname(os.path.dirname(path)), [path]
    ss = sorted(p for p in glob.glob(os.path.join(path, "sessions", "*")) if os.path.isdir(p))
    if not ss:
        raise SystemExit(f"{path} 下没有 sessions/*（不是 nbt pathnav run 的批次目录？）")
    return path, ss


def find_nav_monitor_script(name):
    try:
        out = subprocess.run(["ros2", "pkg", "prefix", "nav2_nav_monitor"], capture_output=True, text=True,
                             timeout=10)
    except (subprocess.SubprocessError, OSError):
        return None
    p = os.path.join(out.stdout.strip(), "lib", "nav2_nav_monitor", name)
    return p if out.returncode == 0 and os.path.exists(p) else None


def analyze_session(sdir, a, plan_override=None):
    """返回 (停靠行, 任务行, 关键路段行, 关节行, 说明, 目标速度, 路段速度行, 曲线数据)。"""
    rows = nl.read_tasks_csv(os.path.join(sdir, "tasks.csv"))
    plan = plan_override or {}
    if not plan and os.path.exists(os.path.join(sdir, "plan.yaml")):
        with open(os.path.join(sdir, "plan.yaml"), encoding="utf-8") as f:
            plan = yaml.safe_load(f) or {}
    key_points = [str(p) for p in plan.get("key_points") or []]
    seg = [str(p) for p in plan.get("segment") or []]
    target = a.target_speed if a.target_speed is not None else plan.get("target_speed")
    targets = nl.load_targets(os.path.join(sdir, "targets.json"))
    notes = []
    bag = os.path.join(sdir, "rosbag")
    stops, trows, srows, jrows, lrows = [], [], [], [], []
    curves = {"tasks": {}, "segment": []}
    if not os.path.exists(os.path.join(bag, "metadata.yaml")):
        notes.append(f"{os.path.basename(sdir)}：没有录包，导航反馈取现场读数，速度类指标空缺")
        for r in rows:
            end = r["points"].split(">")[-1]
            if end in key_points and r["result"] == "succeeded":
                stops.append({"run_id": r["run_id"], "batch": r["batch"], "lap": r["lap"], "point": end,
                              "mark": r["mark"], "note": r["note"],
                              "lon_mm": float(r["err_lon_mm"]) if r["err_lon_mm"] else None,
                              "lat_mm": float(r["err_lat_mm"]) if r["err_lat_mm"] else None,
                              "yaw_deg": float(r["err_yaw_deg"]) if r["err_yaw_deg"] else None,
                              "settle_delay": r["report_to_settle_s"], "source": "现场读数"})
        return stops, trows, srows, jrows, notes, target, lrows, curves
    d = nl.read_bag(bag, nc.opts_of(a))
    notes += [f"{os.path.basename(sdir)}：跳过 {s}" for s in d["skipped"]]
    tasks = nc.build_tasks(d, rows, targets, nc.load_limits(sdir), nc.opts_of(a))
    out_json = os.path.join(a.out_dir, "nav_tasks")
    os.makedirs(out_json, exist_ok=True)
    ref = joint_ref(d["joint"], a.joint_exclude)
    for t in tasks:
        with open(os.path.join(out_json, nc.file_name(t)), "w", encoding="utf-8") as f:
            json.dump(nc.task_json(t, d), f, ensure_ascii=False, indent=1)
        r = t["row"]
        t0 = t["t_accept"]
        t1 = t["t_settle"] or t["t_report"]
        err = nl.target_error(*t["settled"], *t["goal"]) if t["settled"] and t["goal"] else None
        if t["end_name"] in key_points and t["result"] == "succeeded":
            stops.append({"run_id": r.get("run_id", t["uuid"][:8]), "batch": r.get("batch", ""),
                          "lap": r.get("lap", ""), "point": t["end_name"], "mark": r.get("mark", ""),
                          "note": r.get("note", ""),
                          "lon_mm": err["longitudinal"] * 1000 if err else None,
                          "lat_mm": err["lateral"] * 1000 if err else None,
                          "yaw_deg": math.degrees(err["yaw"]) if err else None,
                          "settle_delay": f"{t['t_settle'] - t['t_report']:.2f}" if t["t_settle"] else "未停稳",
                          "source": "录包"})
        odom = nl.window(d["odom"], t0, t1)
        acc, dec, jerk = dynamics(odom, a.accel_window, a.jerk_window)
        ia, idc = imu_peaks(d["imu"], t0, t1)
        peaks = level_peaks(d, t0, t1)
        route = float(r["route_len_m"]) if r.get("route_len_m") else (
            sum(nl.path_length(tp["poses"]) for tp in t["tracking"]) or None)
        dur = (t["t_settle"] - t0) if t["t_settle"] else None
        obs, prev = 0, 0                     # 停障 / 减速触发次数：状态由 FREE 变为非 FREE 记一次
        for _, st in nl.window(d["obstacle"], t0, t1):
            obs += 1 if st != 0 and prev == 0 else 0
            prev = st
        trows.append({"run_id": r.get("run_id", t["uuid"][:8]), "batch": r.get("batch", ""), "lap": r.get("lap", ""),
                      "task": r.get("task", ""), "points": " → ".join(t["points"]), "result": t["result"],
                      "t_accept": common.now_text(t0), "report_s": t["t_report"] - t0, "settle_s": dur,
                      "route_m": route, "passing_m": nl.path_length([(p[1], p[2]) for p in
                                                                    nl.window(d["pose"], t0, t1)]),
                      "avg_v": route / dur if route and dur else None,
                      "max_v": max((abs(o[1]) for o in odom), default=None), "acc": acc, "dec": dec, "jerk": jerk,
                      "imu_acc": ia, "imu_dec": idc, "peaks": peaks, "stage": limit_stage(peaks, target),
                      "obstacle_samples": obs, "note": r.get("note", "")})
        meta = {"run_id": r.get("run_id", t["uuid"][:8]), "lap": r.get("lap", ""), "batch": r.get("batch", ""),
                "mark": r.get("mark", ""), "result": t["result"]}
        if t["points"]:
            ct = curves["tasks"].setdefault(r.get("task") or "?", {"points": t["points"], "runs": [], "pts": None})
            run = dict(meta, **sc.curve_series(d, t0, t1))
            ct["runs"].append(run)
            ts = sc.pass_times(d["pose"], t0, t1, t["points"], targets) if len(t["points"]) > 2 else None
            if ts and ct["pts"] is None:
                ct["pts"] = [{"name": n, "s": run["s"][min(len(run["s"]) - 1, int(round((tp - t0) * 10)))]}
                             for n, tp in zip(t["points"][1:-1], ts[1:-1])]
            if t["result"] == "succeeded":
                for lg in sc.leg_rows(d, t["points"], t0, t1, targets, target, band_time):
                    lrows.append(dict(lg, run_id=meta["run_id"], lap=meta["lap"]))
        sd = segment_dir(t["points"], seg) if len(seg) == 2 and seg[0] in targets and seg[1] in targets else None
        if sd:
            w = segment_window(d["pose"], t0, t1, targets[sd[0]], targets[sd[1]])
            if w is None:
                srows.append({"run_id": r.get("run_id", ""), "lap": r.get("lap", ""), "found": False,
                              "dir": f"{sd[0]}→{sd[1]}"})
            else:
                ti, to, L, adv = w
                so = nl.window(d["odom"], ti, to)
                best, total = band_time(so, target) if target else (None, None)
                sp = level_peaks(d, ti, to)
                curves["segment"].append(dict(meta, dir=f"{sd[0]}→{sd[1]}",
                                              **sc.segment_series(d, ti, to, targets[sd[0]], targets[sd[1]])))
                srows.append({"run_id": r.get("run_id", ""), "lap": r.get("lap", ""), "found": True,
                              "dir": f"{sd[0]}→{sd[1]}",
                              "length": L, "dt": to - ti, "avg_v": adv / (to - ti) if to > ti else None,
                              "max_v": max((abs(o[1]) for o in so), default=None), "steady": best,
                              "in_band": total, "peaks": sp, "stage": limit_stage(sp, target)})
        if ref:
            dev = joint_dev(d["joint"], ref, t0, t1)
            if dev:
                n, v = max(dev.items(), key=lambda kv: kv[1])
                jrows.append({"run_id": r.get("run_id", ""), "task": r.get("task", ""), "joint": n,
                              "max_dev_deg": math.degrees(v), "all": {k: math.degrees(x) for k, x in dev.items()}})
    if not targets:
        notes.append(f"{os.path.basename(sdir)}：没有 targets.json，目标位姿取 /tracking_path 末点")
    return stops, trows, srows, jrows, notes, target, lrows, curves


def point_stats(stops, key_points):
    out = []
    for kp in key_points:
        rs = [s for s in stops if s["point"] == kp]
        valid = [s for s in rs if s["mark"] != "受干扰" and s["lon_mm"] is not None]
        row = {"point": kp, "n": len(rs), "pass": sum(s["mark"] == "合格" for s in rs),
               "fail": sum(s["mark"] == "不合格" for s in rs), "disturbed": sum(s["mark"] == "受干扰" for s in rs),
               "unmarked": sum(s["mark"] == "" for s in rs), "n_stat": len(valid)}
        for k in ("lon_mm", "lat_mm", "yaw_deg"):
            vals = [s[k] for s in valid]
            ab = [abs(v) for v in vals]
            row[k] = {"mean": sum(vals) / len(vals) if vals else None, "p95": pct(ab, 0.95),
                      "max": max(ab) if ab else None, "std": std(vals),
                      "range": (max(vals) - min(vals)) if vals else None}
        out.append(row)
    return out


def write_outputs(a, bdir, key_points, stops, trows, srows, jrows, notes, target, lrows=(), curves=None, seg=None):
    out = a.out_dir
    ps = point_stats(stops, key_points)
    common.write_csv(os.path.join(out, "points.csv"),
                     ["point", "n", "pass", "fail", "disturbed", "unmarked", "n_stat"] +
                     [f"{k}_{s}" for k in ("lon_mm", "lat_mm", "yaw_deg")
                      for s in ("mean", "p95", "max", "std", "range")],
                     [[p["point"], p["n"], p["pass"], p["fail"], p["disturbed"], p["unmarked"], p["n_stat"]] +
                      [fmt(p[k][s], 2) for k in ("lon_mm", "lat_mm", "yaw_deg")
                       for s in ("mean", "p95", "max", "std", "range")] for p in ps])
    sh = ["run_id", "batch", "lap", "point", "mark", "lon_mm", "lat_mm", "yaw_deg", "settle_delay", "note", "source"]
    common.write_csv(os.path.join(out, "stops.csv"), sh,
                     [[fmt(s[h], 2) if h in ("lon_mm", "lat_mm") else fmt(s[h], 3) if h == "yaw_deg" else s[h]
                       for h in sh] for s in stops])
    lv = [k for k, _, _ in nl.LEVELS]
    common.write_csv(os.path.join(out, "tasks_metrics.csv"),
                     ["run_id", "batch", "lap", "task", "points", "result", "t_accept", "report_s", "settle_s",
                      "route_m", "passing_m", "avg_v", "max_v", "acc", "dec", "jerk", "imu_acc", "imu_dec"] +
                     [f"peak_{k}" for k in lv] + ["limit_stage", "obstacle_events", "note"],
                     [[t["run_id"], t["batch"], t["lap"], t["task"], t["points"], t["result"], t["t_accept"],
                       fmt(t["report_s"], 2), fmt(t["settle_s"], 2), fmt(t["route_m"], 3), fmt(t["passing_m"], 3),
                       fmt(t["avg_v"], 3), fmt(t["max_v"], 3), fmt(t["acc"], 3), fmt(t["dec"], 3), fmt(t["jerk"], 3),
                       fmt(t["imu_acc"], 3), fmt(t["imu_dec"], 3)] + [fmt(t["peaks"][k], 3) for k in lv] +
                      [t["stage"], t["obstacle_samples"], t["note"]] for t in trows])

    def seg_cells(s):
        if not s["found"]:
            return [""] * (6 + len(lv) + 1)
        return ([fmt(s["length"], 3), fmt(s["dt"], 2), fmt(s["avg_v"], 3), fmt(s["max_v"], 3), fmt(s["steady"], 2),
                 fmt(s["in_band"], 2)] + [fmt(s["peaks"][k], 3) for k in lv] + [s["stage"]])

    if srows:
        common.write_csv(os.path.join(out, "segment.csv"),
                         ["run_id", "lap", "direction", "found", "length_m", "dt_s", "avg_v", "max_v", "steady_s",
                          "in_band_s"] +
                         [f"peak_{k}" for k in lv] + ["limit_stage"],
                         [[s["run_id"], s["lap"], s["dir"], s["found"]] + seg_cells(s) for s in srows])
    if jrows:
        common.write_csv(os.path.join(out, "joints.csv"), ["run_id", "task", "joint", "max_dev_deg", "all_deg"],
                         [[j["run_id"], j["task"], j["joint"], f"{j['max_dev_deg']:.2f}",
                           json.dumps({k: round(v, 2) for k, v in j["all"].items()}, ensure_ascii=False)]
                          for j in jrows])

    L = [f"# 能力摸底速度测试分析：{os.path.basename(bdir)}", "",
         f"- 目标速度：{target if target else '未给'} m/s；停稳判据：线速度 < {a.still_v} m/s 且角速度 < {a.still_w} rad/s "
         f"持续 {a.still_hold} s，导航反馈取停稳后 {a.avg_window} s 平均",
         f"- 加速度窗 {a.accel_window} s、jerk 窗 {a.jerk_window} s（里程计）；IMU 0.2 s 滑动平均；受干扰的停靠不进统计",
         f"- 任务 {len(trows)} 个（成功 {sum(t['result'] == 'succeeded' for t in trows)}），关键点位停靠 {len(stops)} 次"]
    L += [f"- ⚠ {n}" for n in notes]
    L += ["", "## 关键点位汇总", "", common.md_table(
        ["点位", "停靠", "划线 合格 / 不合格 / 受干扰 / 未录", "纵 mean / P95 / max (mm)", "横 mean / P95 / max (mm)",
         "朝向 mean / P95 / max (°)", "重复：纵 σ / 横 σ (mm)、朝向 σ (°)", "极差 纵 / 横 (mm)"],
        [[p["point"], p["n"], f"{p['pass']} / {p['fail']} / {p['disturbed']} / {p['unmarked']}",
          " / ".join(fmt(p["lon_mm"][s]) for s in ("mean", "p95", "max")),
          " / ".join(fmt(p["lat_mm"][s]) for s in ("mean", "p95", "max")),
          " / ".join(fmt(p["yaw_deg"][s], 2) for s in ("mean", "p95", "max")),
          f"{fmt(p['lon_mm']['std'])} / {fmt(p['lat_mm']['std'])}、{fmt(p['yaw_deg']['std'], 2)}",
          f"{fmt(p['lon_mm']['range'])} / {fmt(p['lat_mm']['range'])}"] for p in ps])]
    L += ["", "P95 按统计样本数（不含受干扰）线性插值；样本少于 20 时 P95 接近最大值，以最大值为准。"]
    L += ["", "## 表 1　关键点位（每次停靠）", "", common.md_table(
        ["运行号", "批次", "圈", "点位", "划线合格", "导航反馈 Δ纵 / Δ横 (mm)", "导航反馈 Δθ (°)", "上报成功 → 停稳 (s)",
         "备注"],
        [[s["run_id"], s["batch"], s["lap"], s["point"], s["mark"] or "（未录）",
          f"{fmt(s['lon_mm'])} / {fmt(s['lat_mm'])}", fmt(s["yaw_deg"], 2), s["settle_delay"], s["note"]]
         for s in stops])]
    L += ["", "## 表 2　任务速度（每个任务）", "", common.md_table(
        ["运行号", "批次", "任务", "受理时间 / 点序列", "段时间 (s)", "路线长 (m)", "平均 / 最大速度 (m/s)",
         "峰值加 / 减速度 (m/s²)", "jerk (m/s³)", "限制环节", "备注"],
        [[t["run_id"], t["batch"], t["task"], f"{t['t_accept']} / {t['points']}", fmt(t["settle_s"], 2),
          fmt(t["route_m"], 2), f"{fmt(t['avg_v'], 3)} / {fmt(t['max_v'], 3)}",
          f"{fmt(t['acc'], 2)} / {fmt(t['dec'], 2)}", fmt(t["jerk"], 2), t["stage"],
          "；".join(x for x in (t["result"] if t["result"] != "succeeded" else "",
                                f"停障 / 减速触发 {t['obstacle_samples']} 次" if t["obstacle_samples"] else "",
                                t["note"]) if x)] for t in trows])]

    def seg_md(s):
        if not s["found"]:
            return ["没识别到通过时段"] + [""] * 5
        return [fmt(s["length"], 2), fmt(s["dt"], 2), f"{fmt(s['avg_v'], 3)} / {fmt(s['max_v'], 3)}",
                f"{fmt(s['steady'], 2)} / {fmt(s['in_band'], 2)}", " / ".join(fmt(s["peaks"][k], 3) for k in lv),
                s["stage"]]

    if srows:
        L += ["", "## 关键路段", "", common.md_table(
            ["运行号", "圈", "方向", "长度 (m)", "通过用时 (s)", "平均 / 最高速度 (m/s)", "目标 ±5% 最长连续 / 累计 (s)",
             "四级峰值 跟踪器 / 平滑器 / 底盘指令 / 里程计", "限制环节"],
            [[s["run_id"], s["lap"], s["dir"]] + seg_md(s) for s in srows])]
    if jrows:
        L += ["", "## 满载姿态（关节相对会话开始的最大偏差）", "", common.md_table(
            ["运行号", "任务", "偏差最大的关节", "最大偏差 (°)"],
            [[j["run_id"], j["task"], j["joint"], f"{j['max_dev_deg']:.2f}"] for j in jrows])]
    legs = sc.aggregate_legs(list(lrows), target)
    if legs:
        common.write_csv(os.path.join(out, "legs.csv"),
                         ["from", "to", "n", "max_v_mean", "max_v_min", "steady_mean_s", "steady_min_s", "reach_laps",
                          "hold2s_laps"],
                         [[g["from"], g["to"], g["n"], fmt(g["max_v_mean"], 3), fmt(g["max_v_min"], 3),
                           fmt(g["steady_mean"], 2), fmt(g["steady_min"], 2), g["reach"], g["hold"]] for g in legs])
        L += ["", "## 各路段实测速度（哪些路段能跑到目标速度）", "", common.md_table(
            ["路段", "样本", "最高速度 均值 / 最小 (m/s)", "目标 ±5% 最长连续 均值 / 最小 (s)", "跑到目标的次数",
             "保持 ≥ 2 s 的次数"],
            [[f"{g['from']} → {g['to']}", g["n"], f"{fmt(g['max_v_mean'], 3)} / {fmt(g['max_v_min'], 3)}",
              f"{fmt(g['steady_mean'], 2)} / {fmt(g['steady_min'], 2)}", f"{g['reach']} / {g['n']}",
              f"{g['hold']} / {g['n']}"] for g in legs])]
        L += ["", "按车依次经过点序列的时刻切路段；「跑到目标」= 最高速度 ≥ 目标 × 95%。关键路段应选这里稳态时间长、"
              "每次都能保持 ≥ 2 s 的路段。"]
    curve_file = ""
    if curves and (curves["tasks"] or curves["segment"]):
        data = {"batch": os.path.basename(bdir), "target": target, "generated": common.now_text(),
                "tasks": [{"id": k, "points": v["points"], "runs": v["runs"], "pts": v["pts"]}
                          for k, v in sorted(curves["tasks"].items(), key=lambda kv: kv[0])],
                "segments": []}
        for r in curves["segment"]:          # 关键路段按行驶方向分图
            a_, b_ = r["dir"].split("→")
            grp = next((g for g in data["segments"] if (g["from"], g["to"]) == (a_, b_)), None)
            if grp is None:
                grp = {"from": a_, "to": b_, "runs": []}
                data["segments"].append(grp)
            grp["runs"].append(r)
        curve_file = os.path.join(out, "speed_curves.html")
        with open(curve_file, "w", encoding="utf-8") as f:
            f.write(sc.render_html(data))
        if getattr(a, "png", False):
            files = sc.write_pngs(data, out)
            print("  ⚠ 没装 matplotlib，跳过 PNG" if files is None else f"  PNG：{len(files)} 张 → {out}/curves_*.png")
    L += ["", "## 文件", "", "points.csv、stops.csv、tasks_metrics.csv、segment.csv、legs.csv、joints.csv；"
          "speed_curves.html 每个任务各圈速度曲线叠加（浏览器打开）；"
          "nav_tasks/ 可用 nav2_nav_monitor 的 analyze_nav_tasks.py 再出路径图和速度图。"]
    with open(os.path.join(out, "summary.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")
    return ps


def main(argv=None):
    ap = argparse.ArgumentParser(prog="nbt speed analyze", description="能力摸底速度测试离线分析")
    ap.add_argument("path", help="批次目录或会话目录")
    ap.add_argument("-o", "--out", default="", help="输出目录，默认 <批次目录>/analysis")
    ap.add_argument("--plan", default="", help="任务计划 YAML，默认用会话里存的 plan.yaml")
    ap.add_argument("--target-speed", type=float, default=None, help="目标速度，默认取计划 target_speed")
    ap.add_argument("--accel-window", type=float, default=0.3)
    ap.add_argument("--jerk-window", type=float, default=0.3)
    ap.add_argument("--joint-exclude", default=r"wheel|steer|drive|caster", help="不算姿态偏差的关节名（正则）")
    ap.add_argument("--nav-monitor", action="store_true", help="顺带跑 nav2_nav_monitor 的 analyze_nav_tasks.py")
    ap.add_argument("--png", action="store_true", help="另出每个任务的速度曲线 PNG（需要 matplotlib）")
    nc.add_args(ap)
    a = ap.parse_args(argv)
    bdir, sessions = sessions_of(a.path)
    a.out_dir = os.path.abspath(os.path.expanduser(a.out or os.path.join(bdir, "analysis")))
    os.makedirs(a.out_dir, exist_ok=True)
    plan = None
    if a.plan:
        with open(a.plan, encoding="utf-8") as f:
            plan = yaml.safe_load(f) or {}
    stops, trows, srows, jrows, notes, key_points, target, lrows, seg = [], [], [], [], [], [], None, [], None
    curves = {"tasks": {}, "segment": []}
    for s in sessions:
        print(f"分析 {s}")
        st, tr, sr, jr, nt, tg, lr, cv = analyze_session(s, a, plan)
        stops += st
        trows += tr
        srows += sr
        jrows += jr
        notes += nt
        lrows += lr
        for k, v in cv["tasks"].items():
            c = curves["tasks"].setdefault(k, {"points": v["points"], "runs": [], "pts": v["pts"]})
            c["runs"] += v["runs"]
            c["pts"] = c["pts"] or v["pts"]
        curves["segment"] += cv["segment"]
        target = target or tg
        pl = plan
        if pl is None and os.path.exists(os.path.join(s, "plan.yaml")):
            with open(os.path.join(s, "plan.yaml"), encoding="utf-8") as f:
                pl = yaml.safe_load(f) or {}
        for k in (pl or {}).get("key_points") or []:
            if str(k) not in key_points:
                key_points.append(str(k))
        if seg is None and (pl or {}).get("segment"):
            seg = [str(x) for x in pl["segment"]]
    ps = write_outputs(a, bdir, key_points, stops, trows, srows, jrows, notes, target, lrows, curves, seg)
    print(f"\n✓ {len(trows)} 个任务，{len(stops)} 次关键点位停靠 → {a.out_dir}/summary.md")
    if os.path.exists(os.path.join(a.out_dir, "speed_curves.html")):
        print(f"  速度曲线叠加：{a.out_dir}/speed_curves.html（浏览器打开）")
    for p in ps:
        print(f"  {p['point']}：停靠 {p['n']}，划线合格 {p['pass']}，不合格 {p['fail']}，受干扰 {p['disturbed']}；"
              f"导航反馈 max 纵 {fmt(p['lon_mm']['max'])} / 横 {fmt(p['lat_mm']['max'])} mm、"
              f"朝向 {fmt(p['yaw_deg']['max'], 2)}°")
    if a.nav_monitor:
        script = find_nav_monitor_script("analyze_nav_tasks.py")
        if script is None:
            print("  ⚠ 找不到 nav2_nav_monitor 的 analyze_nav_tasks.py（没 source 含它的工作区），跳过")
        else:
            subprocess.run([sys.executable, script, os.path.join(a.out_dir, "nav_tasks")], check=False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
