"""路网任务（path_navigation）离线数据层：读录包、切任务、算停稳与到点误差，供 navlog convert / speed analyze 共用。

切任务：/navigate_through_graph_points/_action/status 里每个 goal 的「受理 → 终态」是一个任务；
有 nbt pathnav run 写的 tasks.csv 时按 goal_id 对上任务号、点序列和人工录入的划线结果。
停稳：任务终态后，里程计线速度 < still_v 且角速度 < still_w 持续 still_hold 秒（V0.5.1 到点即上报、不等停稳）。
停稳位姿：停稳后 avg_window 秒内 map → 底盘的平均值，按目标点朝向分解为纵向 / 横向 / 朝向误差。
"""
import bisect
import csv
import math
import os

import yaml

ACTION = "/navigate_through_graph_points"
STATUS_TOPIC = ACTION + "/_action/status"
FEEDBACK_TOPIC = ACTION + "/_action/feedback"
ODOM = "/feedback/odom"
LEVELS = [("follower", "/cmd_vel_nav_path", "跟踪器"), ("smoother", "/control/cmd_vel_nav", "平滑器"),
          ("mux", "/control/cmd_vel", "底盘指令"), ("odom", ODOM, "里程计")]
NAV_STATUS = {0: "IDLE", 1: "NAVIGATING", 2: "COLLISION", 3: "FINISHED", 4: "FAILED"}
GOAL_STATUS = {1: "accepted", 2: "executing", 3: "canceling", 4: "succeeded", 5: "canceled", 6: "aborted"}
TERMINAL = (4, 5, 6)

DEFAULTS = {"still_v": 0.01, "still_w": 0.02, "still_hold": 0.5, "avg_window": 1.0, "max_settle": 10.0,
            "map_frame": "map", "odom_frame": "odom", "base_frame": ""}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_s(st):
    return st.sec + st.nanosec * 1e-9


def stamp_or(st, t_bag):
    s = stamp_s(st)
    return s if s > 0 else t_bag


def uuid_hex(u):
    return bytes(bytearray(u)).hex()


def signed_speed(vx, vy):
    """全向车取合速度，符号跟随 vx（倒车为负）。"""
    v = math.hypot(vx, vy)
    return -v if vx < 0 else v


def target_error(px, py, pyaw, gx, gy, gyaw):
    """位姿相对目标的误差：世界系 dx/dy + 目标朝向系纵向 / 横向 + 朝向差（rad）。"""
    dx, dy = px - gx, py - gy
    lon = dx * math.cos(gyaw) + dy * math.sin(gyaw)
    lat = -dx * math.sin(gyaw) + dy * math.cos(gyaw)
    return {"x": dx, "y": dy, "yaw": wrap(pyaw - gyaw), "distance": math.hypot(dx, dy),
            "longitudinal": lon, "lateral": lat}


def mean_pose(samples):
    """[(t, x, y, yaw)] → (x, y, yaw) 平均（朝向用圆均值）；空返回 None。"""
    if not samples:
        return None
    n = len(samples)
    x = sum(s[1] for s in samples) / n
    y = sum(s[2] for s in samples) / n
    yaw = math.atan2(sum(math.sin(s[3]) for s in samples), sum(math.cos(s[3]) for s in samples))
    return x, y, yaw


def find_settle(odom, t_from, still_v, still_w, hold, max_wait):
    """odom [(t, v, w)]：t_from 之后第一次「|v| < still_v 且 |w| < still_w」连续保持 hold 秒，返回保持满的时刻；
    max_wait 秒内没有返回 None。"""
    ts = [o[0] for o in odom]
    i = bisect.bisect_left(ts, t_from)
    start = None
    while i < len(odom) and odom[i][0] <= t_from + max_wait + hold:
        t, v, w = odom[i]
        if abs(v) < still_v and abs(w) < still_w:
            if start is None:
                start = t
            if t - start >= hold:
                return t
        else:
            start = None
        i += 1
    return None


def window(series, t0, t1):
    """series 按 t 升序，取 [t0, t1] 内的元素。"""
    ts = [s[0] for s in series]
    return series[bisect.bisect_left(ts, t0):bisect.bisect_right(ts, t1)]


def interp_pose(series, t):
    """[(t, x, y, yaw)] 线性插值；超出范围返回 None。"""
    ts = [s[0] for s in series]
    i = bisect.bisect_left(ts, t)
    if i <= 0 or i >= len(series):
        return None
    (t0, x0, y0, w0), (t1, x1, y1, w1) = series[i - 1], series[i]
    k = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
    return x0 + k * (x1 - x0), y0 + k * (y1 - y0), w0 + k * wrap(w1 - w0)


def path_length(pts):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def local_slope(times, vals, win):
    """每个采样点在 ±win/2 窗内做最小二乘直线拟合，返回斜率（导数）；窗内不足 3 点返回 None。"""
    out, n, lo, hi = [], len(times), 0, 0
    half = win / 2.0
    for i in range(n):
        while times[lo] < times[i] - half - 1e-9:      # 1e-9：浮点边界两侧对称取点
            lo += 1
        while hi + 1 < n and times[hi + 1] <= times[i] + half + 1e-9:
            hi += 1
        k = hi - lo + 1
        if k < 3:
            out.append(None)
            continue
        tm = sum(times[lo:hi + 1]) / k
        vm = sum(vals[lo:hi + 1]) / k
        num = sum((times[j] - tm) * (vals[j] - vm) for j in range(lo, hi + 1))
        den = sum((times[j] - tm) ** 2 for j in range(lo, hi + 1))
        out.append(num / den if den > 0 else None)
    return out


def compose_map_base(map_odom, odom_base):
    """map→odom（插值）与 odom→底盘复合成 map 系底盘位姿 [(t, x, y, yaw)]。"""
    if not odom_base:
        return []
    if not map_odom:
        return []
    if len(map_odom) == 1:
        _, x0, y0, w0 = map_odom[0]
        return [(t, x0 + math.cos(w0) * ox - math.sin(w0) * oy, y0 + math.sin(w0) * ox + math.cos(w0) * oy,
                 wrap(w0 + ow)) for t, ox, oy, ow in odom_base]
    ts = [m[0] for m in map_odom]
    out = []
    for t, ox, oy, ow in odom_base:
        i = bisect.bisect_left(ts, t)
        if i <= 0:
            i = 1 if t >= ts[0] - 0.5 else None    # map→odom 发布慢于底盘 TF：首段 0.5 s 内用首值
        elif i >= len(map_odom):
            i = len(map_odom) - 1 if t <= ts[-1] + 0.5 else None
        if i is None:
            continue
        (t0, x0, y0, w0), (t1, x1, y1, w1) = map_odom[i - 1], map_odom[i]
        k = min(1.0, max(0.0, (t - t0) / (t1 - t0))) if t1 > t0 else 0.0
        mx, my, mw = x0 + k * (x1 - x0), y0 + k * (y1 - y0), w0 + k * wrap(w1 - w0)
        out.append((t, mx + math.cos(mw) * ox - math.sin(mw) * oy, my + math.sin(mw) * ox + math.cos(mw) * oy,
                    wrap(mw + ow)))
    return out


def storage_id_of(bag):
    meta = os.path.join(bag, "metadata.yaml")
    if os.path.exists(meta):
        with open(meta) as f:
            m = yaml.safe_load(f) or {}
        return m.get("rosbag2_bagfile_information", {}).get("storage_identifier", "sqlite3")
    return "sqlite3"


def read_bag(bag, opts=None):
    """流式读包，只抽取分析需要的字段，返回 dict。没 source dexe_interfaces 时导航自定义消息会跳过并列在 skipped。"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    o = dict(DEFAULTS, **(opts or {}))
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage_id_of(bag)),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = {STATUS_TOPIC, FEEDBACK_TOPIC, ODOM, "/tf", "/navigation_status", "/tracking_path", "/obstacle_status",
            "/imu/chassis", "/feedback/joint"} | {lv[1] for lv in LEVELS}
    cls, skipped = {}, []
    for t in want:
        if t not in types:
            continue
        try:
            cls[t] = get_message(types[t])
        except (AttributeError, ModuleNotFoundError, ValueError):
            skipped.append(f"{t}（{types[t]}：消息定义没加载，先 source dexe_interfaces）")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(cls)))
    d = {"status": [], "feedback": [], "odom": [], "nav_status": [], "tracking": [], "obstacle": [], "imu": [],
         "joint": [], "levels": {k: [] for k, _, _ in LEVELS}, "tf_map_odom": [], "skipped": skipped,
         "topics": sorted(types), "t_first": None, "t_last": None}
    odom_children = {}
    level_of = {topic: key for key, topic, _ in LEVELS if key != "odom"}
    mf, of = o["map_frame"], o["odom_frame"]
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        tb = t_ns * 1e-9
        d["t_first"] = tb if d["t_first"] is None else d["t_first"]
        d["t_last"] = tb
        m = deserialize_message(raw, cls[topic])
        if topic == ODOM:
            tw = m.twist.twist
            v = signed_speed(tw.linear.x, tw.linear.y)
            d["odom"].append((stamp_or(m.header.stamp, tb), v, tw.angular.z))
            d["levels"]["odom"].append((stamp_or(m.header.stamp, tb), v, tw.angular.z))
        elif topic in level_of:
            d["levels"][level_of[topic]].append((tb, signed_speed(m.linear.x, m.linear.y), m.angular.z))
        elif topic == "/tf":
            for tr in m.transforms:
                parent, child = tr.header.frame_id.lstrip("/"), tr.child_frame_id.lstrip("/")
                p, q = tr.transform.translation, tr.transform.rotation
                item = (stamp_or(tr.header.stamp, tb), p.x, p.y, yaw_of(q))
                if parent == mf and child == of:
                    d["tf_map_odom"].append(item)
                elif parent == of:
                    odom_children.setdefault(child, []).append(item)
        elif topic == STATUS_TOPIC:
            d["status"].append((tb, [(uuid_hex(s.goal_info.goal_id.uuid), int(s.status), stamp_s(s.goal_info.stamp))
                                     for s in m.status_list]))
        elif topic == FEEDBACK_TOPIC:
            f = m.feedback
            d["feedback"].append((tb, uuid_hex(m.goal_id.uuid), f.current_status, int(f.current_point_index),
                                  int(f.total_points)))
        elif topic == "/navigation_status":
            d["nav_status"].append((tb, int(m.status), m.message))
        elif topic == "/tracking_path":
            hdr = stamp_or(m.header.stamp, tb)
            poses = [(p.position.x, p.position.y, yaw_of(p.orientation)) for p in m.poses]
            d["tracking"].append({"t": hdr, "poses": poses, "target_yaw": float(m.target_yaw), "mode": int(m.mode),
                                  "backward": int(m.backward), "max_speed": float(m.max_speed)})
        elif topic == "/obstacle_status":
            hdr = getattr(m, "header", None)
            d["obstacle"].append((stamp_or(hdr.stamp, tb) if hdr else tb, int(getattr(m, "status", 0))))
        elif topic == "/imu/chassis":
            d["imu"].append((stamp_or(m.header.stamp, tb), m.linear_acceleration.x))
        elif topic == "/feedback/joint":
            d["joint"].append((stamp_or(m.header.stamp, tb), list(m.name), list(m.position)))
    for k in ("odom", "nav_status", "obstacle", "imu", "joint", "feedback", "status", "tf_map_odom"):
        d[k].sort(key=lambda x: x[0])
    for k in d["levels"]:
        d["levels"][k].sort()
    d["tracking"].sort(key=lambda x: x["t"])
    base = o["base_frame"] or (max(odom_children, key=lambda c: len(odom_children[c])) if odom_children else "")
    d["base_frame"] = base
    ob = sorted(odom_children.get(base, []))
    d["pose"] = compose_map_base(d["tf_map_odom"], ob)
    return d


def goal_windows(status):
    """status [(t_bag, [(uuid, st, accept_stamp)])] → [{uuid, t_accept, t_report, result}]，按受理时间排序。"""
    goals = {}
    for tb, items in status:
        for u, st, acc in items:
            g = goals.setdefault(u, {"uuid": u, "t_accept": acc if acc > 0 else tb, "t_report": None, "result": None})
            if st in TERMINAL and g["t_report"] is None:
                g["t_report"], g["result"] = tb, GOAL_STATUS[st]
    return sorted(goals.values(), key=lambda g: g["t_accept"])


def read_tasks_csv(path):
    if not path or not os.path.exists(path):
        return []
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def load_targets(path):
    """targets.json：{"points": {名称: {"x","y","theta"}}}；不存在返回 {}。"""
    import json
    if not path or not os.path.exists(path):
        return {}
    with open(path, encoding="utf-8") as f:
        return json.load(f).get("points", {})


def match_rows(goals, rows, tol=2.0):
    """按 goal_id 对上 tasks.csv 行；没 goal_id 的按受理时间（±tol 秒）对。返回 {uuid: row}。"""
    by_id = {r.get("goal_id", ""): r for r in rows if r.get("goal_id")}
    out, used = {}, set()
    for g in goals:
        r = by_id.get(g["uuid"])
        if r is None:
            best, best_dt = None, tol
            for i, x in enumerate(rows):
                try:
                    dt = abs(float(x.get("t_accept") or "nan") - g["t_accept"])
                except ValueError:
                    continue
                if i not in used and dt <= best_dt:
                    best, best_dt = i, dt
            if best is not None:
                used.add(best)
                r = rows[best]
        if r is not None:
            out[g["uuid"]] = r
    return out


def settle_and_pose(d, t_report, o):
    """返回 (t_settle 或 None, 停稳位姿 (x, y, yaw) 或 None)。"""
    odom = [(t, v, w) for t, v, w in d["odom"]]
    ts = find_settle(odom, t_report, o["still_v"], o["still_w"], o["still_hold"], o["max_settle"])
    if ts is None:
        return None, None
    return ts, mean_pose(window(d["pose"], ts, ts + o["avg_window"]))
