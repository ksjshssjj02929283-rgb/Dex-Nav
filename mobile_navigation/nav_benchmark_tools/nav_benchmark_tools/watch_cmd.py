"""nbt watch：盯住建图 / 定位 / 导航 / 系统 / 障碍 / 急停各状态，关键字段一变就打印一行并写 events.csv。

只在字段变化时输出（剩余距离、定位分数这类连续量默认不打印，加 --verbose 才打印）。
每行带本机接收时间、消息 header 时间和「距本组上次变化」秒数，状态码同时给出含义。

  nbt watch                         # 全部组
  nbt watch --groups nav,loc        # 只看导航和定位
  nbt watch --duration 600 -o DIR   # 看 10 min，事件写到 DIR/events.csv
"""
import argparse
import os
import signal
import time

from nav_benchmark_tools import common

GROUPS = {
    "map": ("/mapping/state", "nav_interfaces/msg/MappingState"),
    "loc": ("/localization/state", "nav_interfaces/msg/LocalizationState"),
    "nav": ("/navigation/state", "nav_interfaces/msg/NavigationState"),
    "sys": ("/command_board/nav_system_status", "nav_interfaces/msg/NavSystemStatus"),
    "obs": ("/obstacle_status", "nav_interfaces/msg/ObstacleMonitorStatus"),
    "backend": ("/command_board/navigation_backend", "std_msgs/msg/String"),
    "safety": ("/diagnostics_agg", "diagnostic_msgs/msg/DiagnosticArray"),
}
SAFETY_KEYS = ("estop_state", "bumper_strip_front_state", "bumper_strip_rear_state")
CODE_FIELDS = ("error_code", "nav_fault_code")


def _name(cls, prefix, v):
    return f"{v} {common.constant_name(cls, lambda k: k.startswith(prefix), v)}"


MAP_STATUS = {-1: "-1 ERROR", 0: "0 IDLE", 1: "1 MAPPING", 2: "2 SAVING"}
NAV_STATE = {0: "0 IDLE", 1: "1 PENDING", 2: "2 RUNNING", 3: "3 PAUSED", 4: "4 BLOCKED",
             5: "5 SUCCEEDED", 6: "6 FAILED", 7: "7 CANCELLED"}


def fields_map(m, verbose):
    return {"status": MAP_STATUS.get(m.status, str(m.status)), "map_name": m.map_name, "stage": m.stage,
            "progress": f"{m.progress:.0f}", "error_code": m.error_code}


def fields_loc(m, verbose):
    cls = type(m)
    d = {"status": _name(cls, "LOC_", m.status), "match_level": _name(cls, "MATCH_", m.match_level),
         "error_code": m.error_code}
    if verbose:
        d["score"] = f"{m.score:.2f}"
    return d


def fields_nav(m, verbose):
    cls = type(m)
    d = {"state": NAV_STATE.get(m.state, str(m.state)), "task_id": m.task_id, "error_code": m.error_code,
         "target": f"{_name(cls, 'TARGET_', m.target_type)} {m.target_name}".strip(),
         "current_index": m.current_index, "recoveries": m.recoveries,
         "speed_limit": f"{getattr(m, 'speed_limit', 0.0):g}"}
    if verbose:
        d["distance_remaining"] = f"{m.distance_remaining:.1f}"
    return d


def fields_sys(m, verbose):
    d = {"estop": m.estop_active, "bumper_front": m.bumper_front_active, "bumper_rear": m.bumper_rear_active,
         "recover_state": m.recover_state, "safety_alarm": m.safety_alarm, "nav_fault_code": m.nav_fault_code,
         "nav_fault_detail": m.nav_fault_detail, "map": m.current_map_name, "task_status": m.task_status,
         "charging": m.battery_charging, "battery": m.battery_percentage}
    st = {0: "UNKNOWN", 1: "OK", 2: "FAULT", 3: "SUPPRESSED"}
    for s in m.sensors:
        d[f"sensor:{s.name}"] = st.get(s.state, str(s.state))
    return d


def fields_obs(m, verbose):
    d = {"status": {0: "0 FREE", 1: "1 SLOWDOWN", 2: "2 STOP"}.get(m.status, str(m.status)), "zone": m.zone_name}
    if verbose:
        d["slowdown_ratio"] = f"{m.slowdown_ratio:.2f}"
    return d


def fields_backend(m, verbose):
    return {"backend": m.data}


def fields_safety(m, verbose):
    for s in m.status:
        if "PowerBoard Safety" in s.name:
            kv = {v.key: v.value for v in s.values}
            return {k: kv.get(k, "") for k in SAFETY_KEYS}
    return None            # 本条诊断里没有电源板安全项，不算变化


EXTRACT = {"map": fields_map, "loc": fields_loc, "nav": fields_nav, "sys": fields_sys, "obs": fields_obs,
           "backend": fields_backend, "safety": fields_safety}


def stamp_text(m):
    h = getattr(m, "header", None)
    if h is None:
        return ""
    t = h.stamp.sec + h.stamp.nanosec * 1e-9
    return common.now_text(t) if t > 0 else ""


def value_text(k, v):
    return common.code_text(v) if k in CODE_FIELDS else str(v)


class Watcher:
    def __init__(self, node, groups, verbose, csv_path):
        self.node, self.verbose, self.csv_path = node, verbose, csv_path
        self.last, self.last_t, self.subs, self.pending = {}, {}, [], set(groups)
        self.rows = 0
        for g in groups:
            self.try_subscribe(g)

    def try_subscribe(self, g):
        """发布者出现后再订阅，订阅 QoS 跟发布者匹配（锁存话题能拿到最新值）。"""
        topic, tstr = GROUPS[g]
        if self.node.count_publishers(topic) == 0:
            return False
        cls = common.import_msg(tstr)
        if cls is None:
            print(f"[{g}] 找不到消息类型 {tstr}，跳过（检查是否 source 了 nav_interfaces）")
            self.pending.discard(g)
            return False
        self.subs.append(self.node.create_subscription(cls, topic, lambda m, g=g: self.on_msg(g, m),
                                                       common.qos_for(self.node, topic)))
        self.pending.discard(g)
        print(f"[{g}] 已订阅 {topic}")
        return True

    def on_msg(self, g, m):
        d = EXTRACT[g](m, self.verbose)
        if d is None:
            return
        old = self.last.get(g)
        if old == d:
            return
        now = time.time()
        changed = {k: v for k, v in d.items() if old is None or old.get(k) != v}
        since = f"{now - self.last_t[g]:.2f}s" if g in self.last_t else "首条"
        self.last[g], self.last_t[g] = d, now
        text = "，".join(f"{k}={value_text(k, v)}" if old is None else
                        f"{k}: {value_text(k, old.get(k, ''))} → {value_text(k, v)}" for k, v in changed.items())
        st = stamp_text(m)
        print(f"{common.now_text(now)} [{g}] {text}（距上次变化 {since}" + (f"，stamp {st}" if st else "") + "）",
              flush=True)
        rows = [[common.now_text(now), st, g, k, "" if old is None else old.get(k, ""), v, since]
                for k, v in changed.items()]
        common.write_csv(self.csv_path, ["recv_time", "stamp", "group", "field", "old", "new", "since_last"],
                         rows, append=True)
        self.rows += len(rows)


def main(argv=None):
    ap = argparse.ArgumentParser(prog="nbt watch", description="盯状态变化（建图 / 定位 / 导航 / 系统 / 障碍 / 急停）")
    ap.add_argument("--groups", default=",".join(GROUPS), help=f"逗号分隔，可选 {','.join(GROUPS)}")
    ap.add_argument("--duration", type=float, default=0, help="秒，0 = 一直看到 Ctrl-C")
    ap.add_argument("-o", "--out", default="", help="输出目录，默认 ~/nav_eval/watch_<时间>")
    ap.add_argument("--verbose", action="store_true", help="同时打印连续量（剩余距离、定位分数、减速比例）")
    a = ap.parse_args(argv)
    groups = [g.strip() for g in a.groups.split(",") if g.strip()]
    bad = [g for g in groups if g not in GROUPS]
    if bad:
        ap.error(f"未知分组 {bad}，可选 {list(GROUPS)}")
    a.out = a.out or common.default_out("watch")
    os.makedirs(a.out, exist_ok=True)
    csv_path = os.path.join(a.out, "events.csv")

    import rclpy
    rclpy.init()
    node = rclpy.create_node("nbt_watch")
    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
    w = Watcher(node, groups, a.verbose, csv_path)
    t0, last_probe, warned = time.time(), time.time(), False
    try:
        while not stop["flag"] and (a.duration <= 0 or time.time() - t0 < a.duration):
            rclpy.spin_once(node, timeout_sec=0.1)
            if w.pending and time.time() - last_probe > 1.0:
                last_probe = time.time()
                for g in list(w.pending):
                    w.try_subscribe(g)
                if w.pending and not warned and time.time() - t0 > 3.0:
                    warned = True
                    print("暂无发布者（上线后自动订阅）：" + "，".join(f"{g}={GROUPS[g][0]}" for g in sorted(w.pending)))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(f"\n共 {w.rows} 条字段变化，已写 {csv_path}")


if __name__ == "__main__":
    main()
