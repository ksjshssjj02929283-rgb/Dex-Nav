#!/usr/bin/env python3
"""制动距离离线分析（G-MOT-04、通用约定 4.3）：读 ros2 录包，自动找出每一次制动，匹配触发来源，算制动距离。

适用于人工触发、事后才能找触发时刻的停车：急停按钮、防撞条、path_navigation 的软件 STOP、取消任务；
脚本触发的硬停 / 平滑器停车（nbt brake run）的录包也能用它复核。

怎么找制动：在 /feedback/odom 的线速度曲线上，先稳定在 ≥ --min-speed 至少 --steady-time 秒，随后下降并停稳
（|v| < --still-v 持续 --still-hold 秒），算一次制动。减速起点 = 速度比稳定值下降超过 --resp-dv 之前的最后一个采样。
已用合成录包验证：硬停 / 指令归零 / 软件 STOP / 急停四种触发都能识别，距离与仿真真值一致。

触发来源（在「减速起点前 --max-latency 秒 ～ 减速起点」窗口里找，按下面顺序取第一个找到的）：
  hard_stop   /cmd_vel_hard_stop（std_msgs/Empty）收到消息的时刻              精确
  obstacle    /obstacle_status 的 status 第一次变为 2（STOP）                   精确
  estop       /diagnostics_agg（或 /diagnostics）里 PowerBoard Safety 段          约 1 Hz，粗
              estop_state 由 false 变 true；防撞条同理（bumper）
  nav_state   /navigation/state 的 state 变为 CANCELLED / FAILED / BLOCKED       精确
  cmd_zero    /cmd_vel_nav（控制器输出、平滑器之前）的线速度指令掉到稳定值的 10% 以下   精确（指令时刻）
「粗」来源（急停、防撞条）的触发时刻误差可达 1 s，此时响应延迟不报，制动距离以减速起点算，并在结果里注明。

位移来源：定位 TF（录包里的 /tf，map → odom → 底盘，判定值；不用纯里程计，避免打滑影响）。底盘坐标系默认取 odom
下面发布最频繁的子坐标系，也可用 --base-frame 指定。另外给定位话题 /localization/loc_pose、里程计、雷达到墙距离
（--scan-topic）三个交叉核对值；定位话题频率低于 10 Hz 时只作参考。没录 /tf 时依次退到定位话题（≥ 10 Hz）、里程计，
并在备注里写明。判定值与雷达相差大时说明打滑或定位跟不上，以雷达为准并写备注。

用法：
  source /opt/ros/humble/setup.bash && source <dexe_interfaces 安装目录>/setup.bash   # 解析 nav_interfaces 消息用
  nbt brake analyze <录包目录> -o ~/brake_eval/bag_R2-1 --label "急停 0.7 m/s 空载"
  nbt brake analyze <录包目录> --trigger estop --scan-topic /laser_scan/front
输出：<out>/events.csv（每次制动一行）、<out>/summary.md（含可贴进 G-MOT-04 记录表的行）。
没有 source dexe_interfaces 时，/obstacle_status、/navigation/state 会被跳过，其余照常分析。
"""
import argparse
import bisect
import csv
import math
import os
import sys

import yaml

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    sys.exit("需要 ROS 2 环境：先 source /opt/ros/humble/setup.bash")

PRECISE = {"hard_stop", "obstacle", "nav_state", "cmd_zero", "given"}
NAV_END_STATES = {4, 6, 7}   # BLOCKED / FAILED / CANCELLED（NavigationState）
SAFETY_NAME = "PowerBoard Safety"


def stamp_or(msg_stamp, t_bag):
    s = msg_stamp.sec + msg_stamp.nanosec * 1e-9
    return s if s > 0 else t_bag


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def travel_dir(series, t, interp_fn, lookback=0.5, min_move=0.02):
    """series [(t, x, y, yaw)] 在 t 之前 lookback 秒内的位移方向；位移不足 min_move 米时返回 t 前一个采样的朝向。"""
    pa, pb = interp_fn(series, t - lookback), interp_fn(series, t)
    if pa is not None and pb is not None and math.hypot(pb[0] - pa[0], pb[1] - pa[1]) >= min_move:
        return math.atan2(pb[1] - pa[1], pb[0] - pa[0])
    k = max(0, bisect.bisect_left([x[0] for x in series], t) - 1)
    return series[k][3]


def interp_xy(series, t):
    if len(series) < 2:
        return None
    ts = [s[0] for s in series]
    i = bisect.bisect_left(ts, t)
    if i <= 0 or i >= len(series):
        return None   # 不外推：取值时刻必须落在数据范围内
    (t0, x0, y0), (t1, x1, y1) = series[i - 1][:3], series[i][:3]
    k = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
    return x0 + k * (x1 - x0), y0 + k * (y1 - y0)


def interp_val(series, t):
    ts = [s[0] for s in series]
    i = bisect.bisect_left(ts, t)
    if i <= 0 or i >= len(series):
        return None
    (t0, v0), (t1, v1) = series[i - 1], series[i]
    return v0 + (v1 - v0) * ((t - t0) / (t1 - t0) if t1 > t0 else 0.0)


def storage_id_of(bag):
    meta = os.path.join(bag, "metadata.yaml")
    if os.path.exists(meta):
        with open(meta) as f:
            m = yaml.safe_load(f)
        return m.get("rosbag2_bagfile_information", {}).get("storage_identifier", "sqlite3")
    return "sqlite3"


def read_bag(a):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id=storage_id_of(a.bag)),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = {a.odom_topic, a.pose_topic, a.hard_stop_topic, a.obstacle_topic, a.nav_state_topic, a.cmd_topic,
            a.tf_topic, "/diagnostics_agg", "/diagnostics"}
    if a.scan_topic:
        want.add(a.scan_topic)
    cls, skipped = {}, []
    for t in want:
        if t not in types:
            continue
        try:
            cls[t] = get_message(types[t])
        except (AttributeError, ModuleNotFoundError, ValueError):
            skipped.append(f"{t}（{types[t]}，消息定义没加载，先 source dexe_interfaces）")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(cls)))
    d = {"odom": [], "pose": [], "scan": [], "hard_stop": [], "obstacle": [], "estop": [], "nav_state": [],
         "cmd": [], "tf_map_odom": [], "tf_loc": []}
    tf_odom_child = {}   # odom 下各子坐标系：[(t, x, y, yaw)]
    last_obs, last_safety, last_nav = None, {}, None
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        tb = t_ns * 1e-9
        m = deserialize_message(raw, cls[topic])
        if topic == a.odom_topic:
            p = m.pose.pose
            d["odom"].append((stamp_or(m.header.stamp, tb), m.twist.twist.linear.x, p.position.x, p.position.y,
                              yaw_of(p.orientation)))
        elif topic == a.pose_topic:
            p = m.pose.pose if hasattr(m.pose, "pose") else m.pose
            d["pose"].append((stamp_or(m.header.stamp, tb), p.position.x, p.position.y, yaw_of(p.orientation)))
        elif a.scan_topic and topic == a.scan_topic:
            half = math.radians(a.scan_half_deg)
            vals = [r for i, r in enumerate(m.ranges)
                    if abs(math.atan2(math.sin(m.angle_min + i * m.angle_increment),
                                      math.cos(m.angle_min + i * m.angle_increment))) <= half
                    and m.range_min < r < m.range_max]
            if vals:
                vals.sort()
                d["scan"].append((stamp_or(m.header.stamp, tb), vals[len(vals) // 2]))
        elif topic == a.hard_stop_topic:
            d["hard_stop"].append(tb)
        elif topic == a.obstacle_topic:
            st = int(getattr(m, "status", 0))
            if st == 2 and last_obs != 2:
                d["obstacle"].append(stamp_or(m.header.stamp, tb))
            last_obs = st
        elif topic in ("/diagnostics_agg", "/diagnostics"):
            for s in m.status:
                if SAFETY_NAME not in s.name:
                    continue
                for kv in s.values:
                    if kv.key not in ("estop_state", "bumper_strip_front_state", "bumper_strip_rear_state"):
                        continue
                    on = kv.value.strip().lower() in ("1", "true", "yes", "on")
                    key = (topic, kv.key)
                    if on and not last_safety.get(key, False):
                        d["estop"].append((stamp_or(m.header.stamp, tb), kv.key))
                    last_safety[key] = on
        elif topic == a.nav_state_topic:
            st = int(getattr(m, "state", -1))
            if st in NAV_END_STATES and last_nav not in NAV_END_STATES:
                hdr = getattr(m, "header", None)
                d["nav_state"].append(stamp_or(hdr.stamp, tb) if hdr else tb)
            last_nav = st
        elif topic == a.cmd_topic:
            d["cmd"].append((tb, m.linear.x))
        elif topic == a.tf_topic:
            for tr in m.transforms:
                parent, child = tr.header.frame_id.lstrip("/"), tr.child_frame_id.lstrip("/")
                q, p = tr.transform.rotation, tr.transform.translation
                item = (stamp_or(tr.header.stamp, tb), p.x, p.y, yaw_of(q))
                if parent == a.map_frame and child == a.odom_frame:
                    d["tf_map_odom"].append(item)
                elif parent == a.odom_frame:
                    tf_odom_child.setdefault(child, []).append(item)
    for k in d:
        d[k].sort()
    base = a.base_frame or (max(tf_odom_child, key=lambda c: len(tf_odom_child[c])) if tf_odom_child else "")
    a.base_frame_used = base
    ob = sorted(tf_odom_child.get(base, []))
    d["tf_loc"] = compose_map_base(d["tf_map_odom"], ob)
    return d, skipped


def compose_map_base(map_odom, odom_base):
    """map→odom（插值）与 odom→底盘（原样）复合成 map 系底盘位置 [(t, x, y)]；map→odom 覆盖不到的时刻丢弃。"""
    if len(map_odom) < 2 or not odom_base:
        return []
    ts = [m[0] for m in map_odom]
    out = []
    for t, ox, oy, ow in odom_base:
        i = bisect.bisect_left(ts, t)
        if i <= 0 or i >= len(map_odom):
            continue
        (t0, x0, y0, w0), (t1, x1, y1, w1) = map_odom[i - 1], map_odom[i]
        k = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        dw = math.atan2(math.sin(w1 - w0), math.cos(w1 - w0))
        mx, my, mw = x0 + k * (x1 - x0), y0 + k * (y1 - y0), w0 + k * dw
        out.append((t, mx + math.cos(mw) * ox - math.sin(mw) * oy, my + math.sin(mw) * ox + math.cos(mw) * oy,
                    mw + ow))
    return out


def find_events(a, od):
    """返回 [(i_onset, i_stop, v_steady)]。"""
    ev, i, n = [], 0, len(od)
    while i < n:
        if abs(od[i][1]) < a.min_speed:
            i += 1
            continue
        j = i   # 稳定段：速度变化不超过 steady_tol
        while j + 1 < n and abs(od[j + 1][1] - od[i][1]) <= a.steady_tol:
            j += 1
        if od[j][0] - od[i][0] < a.steady_time:
            i = j + 1
            continue
        seg = [abs(o[1]) for o in od[i:j + 1]]
        v_st = sorted(seg)[len(seg) // 2]
        k = j + 1
        while k < n and abs(od[k][1]) > v_st - a.resp_dv:
            k += 1
        if k >= n:
            break
        onset = max(i, k - 1)   # 减速起点 = 速度掉出阈值前的最后一个采样
        s = onset   # 静止：|v| < still_v 并保持 still_hold
        stop = None
        while s < n:
            if abs(od[s][1]) < a.still_v:
                e = s
                while e + 1 < n and abs(od[e + 1][1]) < a.still_v:
                    e += 1
                if od[e][0] - od[s][0] >= a.still_hold:
                    stop = s
                    break
                s = e + 1
            elif abs(od[s][1]) > v_st + a.steady_tol:   # 没停又加速了，不算制动
                break
            else:
                s += 1
        if stop is not None and od[stop][0] - od[onset][0] <= a.max_brake_time:
            ev.append((onset, stop, v_st))
            i = stop + 1
        else:
            i = k + 1
    return ev


def pick_trigger(a, d, od, onset_t, v_st):
    lo, hi = onset_t - a.max_latency, onset_t + 0.05
    cands = []
    for t in d["hard_stop"]:
        if lo <= t <= hi:
            cands.append(("hard_stop", t))
    for t in d["obstacle"]:
        if lo <= t <= hi:
            cands.append(("obstacle", t))
    for t, key in d["estop"]:
        if onset_t - a.max_latency - 1.0 <= t <= onset_t + 1.0:   # 1 Hz 来源，窗口放宽 1 s
            cands.append(("estop" if key == "estop_state" else "bumper", t))
    for t in d["nav_state"]:
        if lo <= t <= hi:
            cands.append(("nav_state", t))
    prev = None
    for t, v in d["cmd"]:
        if lo <= t <= hi and abs(v) < 0.1 * v_st and (prev is None or abs(prev) >= 0.1 * v_st):
            cands.append(("cmd_zero", t))
            break
        prev = v
    order = ["hard_stop", "obstacle", "estop", "bumper", "nav_state", "cmd_zero"]
    if a.trigger != "auto":
        cands = [c for c in cands if c[0] == a.trigger or (a.trigger == "estop" and c[0] == "bumper")]
    for name in order:
        hit = [c for c in cands if c[0] == name]
        if hit:
            # 精确来源取窗口里最早的一条（nbt brake run 会在保持期内重发硬停）；约 1 Hz 的粗来源取离减速起点最近的
            return min(hit, key=lambda c: c[1]) if name in PRECISE else min(hit, key=lambda c: abs(c[1] - onset_t))
    return None


def analyse(a, d):
    od = d["odom"]
    if len(od) < 20:
        sys.exit(f"{a.odom_topic} 数据太少（{len(od)} 条）")
    res = []
    for n_ev, (io, ist, v_st) in enumerate(find_events(a, od), 1):
        t_on, t_stop = od[io][0], od[ist][0]
        trig = pick_trigger(a, d, od, t_on, v_st)
        src, t_trig = (trig if trig else ("无", None))
        precise = src in PRECISE
        t0 = t_trig if (t_trig is not None and precise) else t_on   # 粗来源或没找到：从减速起点算
        def along(series):
            """沿制动前行驶方向的位移 mm。方向取同一序列起算前 0.5 s 的位移方向（各来源在自己的坐标系里算，
            map 与 odom 朝向不同也不受影响）；那段位移太短时退到起算前一个采样的朝向。"""
            p0, p1 = interp_xy(series, t0), interp_xy(series, t_stop)
            if p0 is None or p1 is None:
                return None
            yaw = travel_dir(series, t0, interp_xy)
            return ((p1[0] - p0[0]) * math.cos(yaw) + (p1[1] - p0[1]) * math.sin(yaw)) * 1000.0

        d_odom = along([(o[0], o[2], o[3], o[4]) for o in od])
        d_tf = along(d["tf_loc"]) if len(d["tf_loc"]) > 3 else None
        d_pose = along(d["pose"]) if len(d["pose"]) > 3 else None
        d_scan = None
        if len(d["scan"]) > 3:
            s0 = interp_val(d["scan"], t0)
            rest = [v for t, v in d["scan"] if t_stop + 0.2 <= t <= t_stop + 1.0]
            if s0 is not None and rest:
                rest.sort()
                d_scan = (s0 - rest[len(rest) // 2]) * 1000.0
        seg = [o for o in od if t_on - 0.1 <= o[0] <= t_stop + 0.1]
        decs = []
        for k, o in enumerate(seg):
            j = next((m for m in range(k + 1, len(seg)) if seg[m][0] - o[0] >= a.decel_window), None)
            if j is None:
                break
            decs.append((abs(o[1]) - abs(seg[j][1])) / (seg[j][0] - o[0]))
        brake_s = t_stop - t_on
        pose_hz = None
        if len(d["pose"]) > 1:
            span = d["pose"][-1][0] - d["pose"][0][0]
            pose_hz = (len(d["pose"]) - 1) / span if span > 0 else None
        if d_tf is not None:
            d_judge, judge_src = d_tf, "定位 TF"
        elif d_pose is not None and pose_hz is not None and pose_hz >= 10:
            d_judge, judge_src = d_pose, "定位话题"
        else:
            d_judge, judge_src = d_odom, "里程计"
        note = []
        if judge_src != "定位 TF":
            note.append(f"包里没有可用的 {a.map_frame}→{a.odom_frame} TF，判定值改用{judge_src}")
        if src == "无":
            note.append("窗口内没找到触发来源，距离从减速起点算")
        elif not precise:
            note.append(f"{src} 来源约 1 Hz，触发时刻不准，距离从减速起点算、不报响应延迟")
        if d_judge is not None and d_scan is not None and abs(d_judge - d_scan) > a.slip_mm:
            note.append(f"{judge_src}与雷达相差 {abs(d_judge - d_scan):.0f} mm，可能打滑，以雷达为准")
        if d_pose is not None and pose_hz is not None and pose_hz < 10:
            note.append(f"定位话题只有 {pose_hz:.1f} Hz，该列只作参考")
        res.append({
            "event": n_ev, "trigger": src, "t_trigger": t_trig, "t_onset": t_on, "t_stop": t_stop,
            "v_trig": interp_val([(o[0], abs(o[1])) for o in od], t0) or v_st,
            "resp_ms": (t_on - t_trig) * 1000.0 if (t_trig is not None and precise) else None,
            "stop_s": t_stop - t0, "brake_s": brake_s,
            "dist_mm": d_judge, "dist_src": judge_src, "dist_tf_mm": d_tf,
            "dist_pose_mm": d_pose, "dist_odom_mm": d_odom, "dist_scan_mm": d_scan,
            "dist_from": "触发时刻" if t0 == t_trig else "减速起点",
            "peak_decel": max(decs) if decs else None,
            "mean_decel": (v_st / brake_s) if brake_s > 0 else None,
            "note": "；".join(note)})
    return res


def f(x, n=0):
    return "" if x is None else f"{x:.{n}f}"


def write(a, res, skipped):
    os.makedirs(a.out, exist_ok=True)
    t_base = a.t_base
    cols = ["event", "trigger", "t_trigger", "t_onset", "t_stop", "v_trig", "resp_ms", "stop_s", "brake_s",
            "dist_from", "dist_mm", "dist_src", "dist_tf_mm", "dist_pose_mm", "dist_odom_mm", "dist_scan_mm",
            "peak_decel", "mean_decel", "note"]
    with open(os.path.join(a.out, "events.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        for r in res:
            w.writerow({k: (f"{r[k]:.4f}" if isinstance(r[k], float) else r[k]) for k in cols})
    chan = {"hard_stop": "④ 硬停", "obstacle": "② 软件 STOP", "estop": "① 急停按钮", "bumper": "防撞条",
            "nav_state": "③ 取消 / 失败", "cmd_zero": "指令归零", "无": "未识别"}
    L = [f"# 制动距离分析：{a.label or os.path.basename(os.path.abspath(a.bag))}", "",
         f"- 录包：`{os.path.abspath(a.bag)}`；识别到 {len(res)} 次制动",
         f"- 判定值取定位 TF（`{a.map_frame}` → `{a.odom_frame}` → `{a.base_frame_used or '未找到'}`）；"
         f"定位话题 `{a.pose_topic}`、里程计、雷达到墙为交叉核对"]
    if skipped:
        L.append("- 跳过的话题：" + "；".join(skipped))
    L += ["", "| # | 触发来源 | 触发前速度 (m/s) | 响应延迟 (ms) | 距离起算 | 制动距离·判定 (mm) | 判定来源 | 定位话题 (mm) | "
          "里程计 (mm) | 雷达到墙 (mm) | 触发到停稳 (s) | 峰值减速度 (m/s²) | 平均减速度 (m/s²) | 备注 |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in res:
        L.append(f"| {r['event']} | {chan.get(r['trigger'], r['trigger'])} | {f(r['v_trig'], 3)} | {f(r['resp_ms'])} | "
                 f"{r['dist_from']} | {f(r['dist_mm'])} | {r['dist_src']} | {f(r['dist_pose_mm'])} | "
                 f"{f(r['dist_odom_mm'])} | {f(r['dist_scan_mm'])} | "
                 f"{f(r['stop_s'], 3)} | {f(r['peak_decel'], 2)} | {f(r['mean_decel'], 2)} | {r['note']} |")
    L += ["", "G-MOT-04 记录表行（时间、后端、速度档、负载按实际填）：", ""]
    for r in res:
        dist = r["dist_mm"]
        L.append(f"| {r['event']} | | | {chan.get(r['trigger'], r['trigger'])} | | {f(r['v_trig'], 3)} | | "
                 f"{'录包 +' + f(r['t_trigger'] - t_base, 3) + ' s' if r['t_trigger'] is not None else '—'} | "
                 f"录包 +{f(r['t_stop'] - t_base, 3)} s | {f(r['resp_ms'])} | {f(dist)} | {f(r['peak_decel'], 2)} | | "
                 f"{os.path.basename(os.path.abspath(a.bag))} | 距离从{r['dist_from']}算、判定取{r['dist_src']} {r['note']} |")
    with open(os.path.join(a.out, "summary.md"), "w") as fh:
        fh.write("\n".join(L) + "\n")
    print("\n".join(L))


def main(argv=None):
    ap = argparse.ArgumentParser(prog="nbt brake analyze", description="制动距离离线分析（G-MOT-04）")
    ap.add_argument("bag", help="ros2 bag 目录")
    ap.add_argument("-o", "--out", default="", help="输出目录，默认 <录包目录>_brake")
    ap.add_argument("--label", default="")
    ap.add_argument("--trigger", default="auto",
                    choices=["auto", "hard_stop", "obstacle", "estop", "nav_state", "cmd_zero"],
                    help="只用某一种触发来源（默认按顺序自动找）")
    ap.add_argument("--odom-topic", default="/feedback/odom")
    ap.add_argument("--pose-topic", default="/localization/loc_pose", help="定位话题，交叉核对用")
    ap.add_argument("--tf-topic", default="/tf")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--odom-frame", default="odom")
    ap.add_argument("--base-frame", default="", help="底盘坐标系，默认取 odom 下发布最频繁的子坐标系")
    ap.add_argument("--hard-stop-topic", default="/cmd_vel_hard_stop")
    ap.add_argument("--obstacle-topic", default="/obstacle_status")
    ap.add_argument("--nav-state-topic", default="/navigation/state")
    ap.add_argument("--cmd-topic", default="/cmd_vel_nav", help="控制器输出（平滑器之前）的速度指令")
    ap.add_argument("--scan-topic", default="", help="可选：正前方雷达，用到墙距离做交叉核对")
    ap.add_argument("--scan-half-deg", type=float, default=2.0)
    ap.add_argument("--min-speed", type=float, default=0.1, help="算作一次制动的最低稳定速度 m/s")
    ap.add_argument("--steady-tol", type=float, default=0.03)
    ap.add_argument("--steady-time", type=float, default=0.5)
    ap.add_argument("--resp-dv", type=float, default=0.02)
    ap.add_argument("--still-v", type=float, default=0.01)
    ap.add_argument("--still-hold", type=float, default=0.5)
    ap.add_argument("--max-latency", type=float, default=1.5, help="触发到减速起点的最长间隔 s")
    ap.add_argument("--max-brake-time", type=float, default=5.0)
    ap.add_argument("--decel-window", type=float, default=0.05)
    ap.add_argument("--slip-mm", type=float, default=20.0, help="定位与雷达相差超过多少 mm 标打滑")
    a = ap.parse_args(argv)
    a.out = a.out or (os.path.abspath(a.bag).rstrip("/") + "_brake")
    d, skipped = read_bag(a)
    a.t_base = d["odom"][0][0] if d["odom"] else 0.0
    res = analyse(a, d)
    if not res:
        print("没有识别到制动（检查速度是否达到 --min-speed、是否停稳）")
    write(a, res, skipped)


if __name__ == "__main__":
    main()
