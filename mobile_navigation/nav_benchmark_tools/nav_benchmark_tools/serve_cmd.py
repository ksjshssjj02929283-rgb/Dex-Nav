"""nbt serve：RViz 测试面板的后端，常驻在机器人上。

面板只负责按钮和显示，测试逻辑在这里执行（复用 nbt 各子命令的代码）；两者只通过话题通信，
面板在机器人桌面或开发机上都能用，录包和结果文件落在机器人本地。

话题（载荷是 std_msgs/String 里的 JSON，带 version 字段）：
  /nbt/cmd      面板 → 后端  {"version": 1, "id": "...", "op": "...", "args": {...}}
  /nbt/status   后端 → 面板  锁存，默认 2 Hz：状态看板、测前检查、会话、导航任务、上一条命令的应答
  /nbt/events   后端 → 面板  一条一个事件：状态字段变化、打点、命令执行
  /nbt/marker   打点文本，会话录包时一并录下

命令 op：preflight、session_start {case, label}、session_stop、mark {text}、
        nav_run {kind: point|route, targets, repeat, gap, stop_on_fail, confirm_clear}、
        nav_pause、nav_resume、nav_cancel、speed_limit {value, percent}、hard_stop

  nbt serve                     # 输出目录默认 ~/nbt_eval
  nbt serve --out /data/nbt_eval --rate 2
"""
import argparse
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time

from nav_benchmark_tools import common, record_cmd, watch_cmd

PROTO = 1
TASK_CSV_HEADERS = ["start", "end", "kind", "target", "accepted", "success", "error_code", "error_text",
                    "message", "elapsed_s", "task_id"]
EVENT_CSV_HEADERS = ["recv_time", "stamp", "group", "field", "old", "new", "since_last"]
NAV_STATE = {0: "IDLE", 1: "PENDING", 2: "RUNNING", 3: "PAUSED", 4: "BLOCKED", 5: "SUCCEEDED", 6: "FAILED",
             7: "CANCELLED"}


def pkg_version():
    try:
        from ament_index_python.packages import get_package_share_directory
        with open(os.path.join(get_package_share_directory("nav_benchmark_tools"), "package.xml")) as f:
            m = re.search(r"<version>\s*([^<\s]+)\s*</version>", f.read())
        return m.group(1) if m else ""
    except Exception:  # noqa: BLE001  拿不到版本不影响运行
        return ""


def deb_version(name):
    try:
        r = subprocess.run(["dpkg-query", "-W", "-f", "${Version}", name], capture_output=True, text=True, timeout=5)
        return r.stdout.strip()
    except (subprocess.SubprocessError, OSError):
        return ""


def nbt_argv(*args):
    """以子进程方式调用 nbt 子命令（与当前进程同一套 Python 环境）。"""
    return [sys.executable, "-m", "nav_benchmark_tools.cli", *args]


def wait_future(fut, timeout):
    """在工作线程里等 rclpy future；执行器在主线程转，回调由它触发。"""
    ev = threading.Event()
    fut.add_done_callback(lambda _f: ev.set())
    if not ev.wait(timeout):
        return None
    return fut.result()


class Serve:
    def __init__(self, node, out_root, rate):
        from rclpy.action import ActionClient
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Empty, String
        from nav_interfaces.action import Navigate
        from nav_interfaces.srv import NavCommand

        self.node, self.out_root = node, os.path.expanduser(out_root)
        self.String, self.Empty, self.Navigate, self.NavCommand = String, Empty, Navigate, NavCommand
        os.makedirs(self.out_root, exist_ok=True)
        self.lock = threading.RLock()
        self.cbg = ReentrantCallbackGroup()
        self.version = pkg_version()
        self.latest, self.last_t, self.fields, self.subs = {}, {}, {}, []
        self.pending = set(watch_cmd.GROUPS) | {"assets"}
        self.assets = {"map": "", "points": [], "routes": [], "loaded": False}
        self.preflight = None
        self.session = None
        self.ack = None
        self.task = {"running": False, "kind": "", "targets": [], "index": 0, "total": 0, "current": "",
                     "state": "", "distance": None, "rows": [], "stats": {}}
        self.stop_seq = threading.Event()
        self.goal_handle = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub_status = node.create_publisher(String, "/nbt/status", latched)
        self.pub_events = node.create_publisher(String, "/nbt/events", QoSProfile(depth=50))
        self.pub_marker = node.create_publisher(String, "/nbt/marker", QoSProfile(depth=10))
        self.pub_hard = node.create_publisher(Empty, "/cmd_vel_hard_stop", QoSProfile(depth=10))
        node.create_subscription(String, "/nbt/cmd", self.on_cmd, QoSProfile(depth=10), callback_group=self.cbg)
        self.nav = ActionClient(node, Navigate, "/navigation/navigate", callback_group=self.cbg)
        self.cmd_cli = node.create_client(NavCommand, "/navigation/nav_command", callback_group=self.cbg)
        node.create_timer(1.0, self.try_subscribe, callback_group=self.cbg)
        node.create_timer(1.0 / max(0.2, rate), self.publish_status, callback_group=self.cbg)
        node.get_logger().info(f"nbt serve up (protocol {PROTO}, output {self.out_root})")

    # ── 状态订阅 ──────────────────────────────────────────────
    def try_subscribe(self):
        for g in list(self.pending):
            if g == "assets":
                topic, tstr = "/map_asset_manager/assets", "nav_interfaces/msg/AssetSnapshot"
            else:
                topic, tstr = watch_cmd.GROUPS[g]
            if self.node.count_publishers(topic) == 0:
                continue
            cls = common.import_msg(tstr)
            if cls is None:
                self.pending.discard(g)
                continue
            cb = self.on_assets if g == "assets" else (lambda m, g=g: self.on_state(g, m))
            self.subs.append(self.node.create_subscription(cls, topic, cb, common.qos_for(self.node, topic),
                                                           callback_group=self.cbg))
            self.pending.discard(g)

    def on_assets(self, m):
        with self.lock:
            self.assets = {"map": m.map_name, "loaded": bool(m.loaded),
                           "points": sorted(p.name for p in m.points), "routes": sorted(r.name for r in m.routes)}

    def on_state(self, g, m):
        d = watch_cmd.EXTRACT[g](m, True)
        if d is None:
            return
        now = time.time()
        with self.lock:
            self.latest[g] = m
            old = self.fields.get(g)
            since = f"{now - self.last_t[g]:.2f}s" if g in self.last_t else ""
            self.last_t[g] = now
            if old == d:
                return
            self.fields[g] = d
            # 连续量（分数、剩余距离、减速比例）只更新看板，不算事件
            quiet = {"score", "distance_remaining", "slowdown_ratio", "battery"}
            changed = {k: v for k, v in d.items() if (old is None or old.get(k) != v) and k not in quiet}
            if not changed:
                return
            ev = {"t": common.now_text(now), "type": "state", "group": g,
                  "changes": {k: [None if old is None else str(old.get(k, "")), str(v)] for k, v in changed.items()}}
            self.emit(ev)
            if self.session:
                st = watch_cmd.stamp_text(m)
                rows = [[common.now_text(now), st, g, k, "" if old is None else old.get(k, ""), v, since]
                        for k, v in changed.items()]
                common.write_csv(os.path.join(self.session["dir"], "events.csv"), EVENT_CSV_HEADERS, rows, append=True)

    def emit(self, ev):
        msg = self.String()
        msg.data = json.dumps(ev, ensure_ascii=False)
        self.pub_events.publish(msg)

    # ── 状态发布 ──────────────────────────────────────────────
    def tiles(self):
        now = time.time()
        out = {}
        for g, d in self.fields.items():
            t = dict(d)
            t["_age"] = round(now - self.last_t.get(g, now), 1)
            out[g] = t
        sys_m = self.latest.get("sys")
        if sys_m is not None:
            out.setdefault("sys", {})["_sensors"] = [
                {"name": s.name, "state": int(s.state), "required": bool(s.required),
                 "rate_hz": round(float(s.rate_hz), 1), "age_sec": round(float(s.age_sec), 2)} for s in sys_m.sensors]
        return out

    def publish_status(self):
        with self.lock:
            sess = None
            if self.session:
                sess = {k: self.session[k] for k in ("case", "label", "dir", "start", "markers")}
                sess["elapsed"] = round(time.time() - self.session["t0"], 1)
                sess["recording"] = self.session["bag_proc"] is not None and self.session["bag_proc"].poll() is None
            body = {"version": PROTO, "serve_version": self.version, "time": common.now_text(),
                    "tiles": self.tiles(), "assets": self.assets, "preflight": self.preflight,
                    "session": sess, "task": self.task, "ack": self.ack,
                    "nav_ready": self.nav.server_is_ready()}
        msg = self.String()
        msg.data = json.dumps(body, ensure_ascii=False, default=str)
        self.pub_status.publish(msg)

    # ── 命令 ─────────────────────────────────────────────────
    def reply(self, cid, op, ok, message, **extra):
        with self.lock:
            self.ack = {"id": cid, "op": op, "ok": bool(ok), "message": message, "time": common.now_text(), **extra}
        self.emit({"t": common.now_text(), "type": "ack", **self.ack})

    def on_cmd(self, msg):
        try:
            req = json.loads(msg.data)
        except ValueError:
            self.reply("", "", False, "invalid json")
            return
        cid, op, args = str(req.get("id", "")), str(req.get("op", "")), req.get("args") or {}
        if req.get("version") != PROTO:
            self.reply(cid, op, False, f"protocol version mismatch: panel {req.get('version')}, serve {PROTO}")
            return
        handler = getattr(self, "op_" + op, None)
        if handler is None:
            self.reply(cid, op, False, f"unknown op: {op}")
            return
        # 命令都放到工作线程，不阻塞执行器
        threading.Thread(target=self.safe_call, args=(handler, cid, op, args), daemon=True).start()

    def safe_call(self, handler, cid, op, args):
        try:
            handler(cid, op, args)
        except Exception as e:  # noqa: BLE001  命令出错要回给面板，不能让线程静默退出
            self.node.get_logger().error(f"op {op} failed: {e}")
            self.reply(cid, op, False, f"error: {e}")

    def op_preflight(self, cid, op, args):
        self.preflight = {"time": common.now_text(), "items": self.run_preflight(args)}
        bad = [i["key"] for i in self.preflight["items"] if i["level"] == "fail"]
        self.reply(cid, op, not bad, "all checks passed" if not bad else "failed: " + ", ".join(bad))

    def run_preflight(self, args):
        items = []

        def add(key, level, detail):
            items.append({"key": key, "level": level, "detail": detail})
        with self.lock:
            f, latest, assets = dict(self.fields), dict(self.latest), dict(self.assets)
        loc = latest.get("loc")
        if loc is None:
            add("localization", "fail", "no /localization/state")
        else:
            lvl = "ok" if (loc.status == 4 and loc.match_level == 1) else ("warn" if loc.status == 4 else "fail")
            add("localization", lvl, f"status={loc.status} match_level={loc.match_level} score={loc.score:.2f}")
            t = loc.header.stamp.sec + loc.header.stamp.nanosec * 1e-9
            skew = abs(time.time() - t) if t > 0 else None
            if skew is None or t < 1.6e9:
                add("clock", "fail", "message stamp not set or before 2020")
            else:
                add("clock", "ok" if skew < 1.0 else ("warn" if skew < 5.0 else "fail"), f"skew={skew:.2f}s")
        backend = f.get("backend", {}).get("backend")
        add("backend", "ok" if backend else "fail", backend or "no /command_board/navigation_backend")
        sysm = latest.get("sys")
        if sysm is None:
            add("system", "fail", "no /command_board/nav_system_status")
        else:
            add("map", "ok" if sysm.current_map_name else "fail", sysm.current_map_name or "no map loaded")
            if sysm.estop_active or sysm.bumper_front_active or sysm.bumper_rear_active:
                add("safety", "fail", f"estop={sysm.estop_active} bumper_front={sysm.bumper_front_active} "
                                      f"bumper_rear={sysm.bumper_rear_active}")
            else:
                add("safety", "ok", "estop and bumpers released")
            bad = [s.name for s in sysm.sensors if s.required and s.state == 2]
            unk = [s.name for s in sysm.sensors if s.required and s.state == 0]
            add("sensors", "fail" if bad else ("warn" if unk else "ok"),
                ("fault: " + ",".join(bad)) if bad else (("unknown: " + ",".join(unk)) if unk else
                                                        f"{len(sysm.sensors)} sensors ok"))
            minb = float(args.get("min_battery", 30))
            b = int(sysm.battery_percentage)
            add("battery", "ok" if b >= minb else ("warn" if b >= 15 else "fail"),
                f"{b}%{' charging' if sysm.battery_charging else ''}")
            if sysm.nav_fault_code:
                add("nav_fault", "fail", common.code_text(sysm.nav_fault_code))
        obs = f.get("obs")
        if obs is not None:
            st = obs.get("status", "")
            add("obstacle", "ok" if st.startswith("0") else "warn", f"{st} {obs.get('zone', '')}".strip())
        need_routes = int(args.get("min_routes", 1))
        npts, nrt = len(assets.get("points", [])), len(assets.get("routes", []))
        add("assets", "ok" if (npts > 0 and nrt >= need_routes) else "warn", f"points={npts} routes={nrt}")
        add("nav_server", "ok" if self.nav.wait_for_server(timeout_sec=1.0) else "fail", "/navigation/navigate")
        free_gb = shutil.disk_usage(self.out_root).free / 1e9
        add("disk", "ok" if free_gb >= 5 else ("warn" if free_gb >= 1 else "fail"), f"{free_gb:.1f} GB free")
        add("versions", "ok", f"nbt {self.version or '?'} / dexe-mobile-nav {deb_version('dexe-mobile-nav') or '?'}")
        return items

    def op_session_start(self, cid, op, args):
        case = str(args.get("case", "")).strip().upper()
        label = str(args.get("label", "")).strip()
        with self.lock:
            if self.session:
                self.reply(cid, op, False, f"session already active: {self.session['case']}")
                return
        tag = case or "SESSION"
        d = os.path.join(self.out_root, "sessions", time.strftime(f"{tag}_%Y%m%d_%H%M%S"))
        os.makedirs(d, exist_ok=True)
        env_log = open(os.path.join(d, "env.log"), "w")
        subprocess.Popen(nbt_argv("env", "-o", os.path.join(d, "env"), "--label", f"{case} {label}".strip()),
                         stdout=env_log, stderr=subprocess.STDOUT)
        rec_log = open(os.path.join(d, "record.log"), "w")
        extra = ["--extra", "/nbt/marker", "/nbt/events"]
        bag = subprocess.Popen(nbt_argv("record", "--case", case, "-o", os.path.join(d, "bag"), *extra)
                               if case else nbt_argv("record", "-o", os.path.join(d, "bag"), *extra),
                               stdout=rec_log, stderr=subprocess.STDOUT, start_new_session=True)
        with self.lock:
            self.session = {"case": case, "label": label, "dir": d, "start": common.now_text(), "t0": time.time(),
                            "markers": 0, "bag_proc": bag, "logs": [env_log, rec_log]}
        with open(os.path.join(d, "session.json"), "w") as fp:
            json.dump({"case": case, "label": label, "start": self.session["start"],
                       "topics": record_cmd.topics_for(case, extra[1:])[0]}, fp, ensure_ascii=False, indent=2)
        self.node.get_logger().info(f"session started: {case or '-'} -> {d}")
        self.reply(cid, op, True, f"session started: {d}", dir=d)

    def op_session_stop(self, cid, op, args):
        with self.lock:
            s, self.session = self.session, None
        if not s:
            self.reply(cid, op, False, "no active session")
            return
        p = s["bag_proc"]
        if p and p.poll() is None:
            try:
                os.killpg(p.pid, signal.SIGINT)     # 和终端 Ctrl-C 一样，让 rosbag2 自己写完 metadata
            except ProcessLookupError:
                pass
            try:
                p.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGTERM)
        for fp in s["logs"]:
            fp.close()
        dur = time.time() - s["t0"]
        with self.lock:
            rows = [r for r in self.task["rows"] if r.get("start", "") >= s["start"]]
        lines = [f"# 测试会话：{s['case'] or '-'} {s['label']}", "",
                 common.md_table(["项", "值"], [["开始", s["start"]], ["结束", common.now_text()],
                                               ["时长", f"{dur:.0f} s"], ["打点", s["markers"]],
                                               ["导航任务", f"{len(rows)}（成功 {sum(1 for r in rows if r['success'])}）"],
                                               ["录包", os.path.join(s["dir"], "bag")],
                                               ["状态变化", os.path.join(s["dir"], "events.csv")],
                                               ["环境快照", os.path.join(s["dir"], "env", "env.md")]])]
        with open(os.path.join(s["dir"], "summary.md"), "w") as fp:
            fp.write("\n".join(lines) + "\n")
        self.node.get_logger().info(f"session stopped: {s['dir']} ({dur:.0f}s)")
        self.reply(cid, op, True, f"session stopped: {s['dir']}", dir=s["dir"])

    def op_mark(self, cid, op, args):
        text = str(args.get("text", "")).strip() or "mark"
        self.publish_marker(text)
        self.reply(cid, op, True, f"marked: {text}")

    def publish_marker(self, text):
        m = self.String()
        m.data = text
        self.pub_marker.publish(m)
        now = time.time()
        self.emit({"t": common.now_text(now), "type": "mark", "text": text})
        with self.lock:
            if self.session:
                self.session["markers"] += 1
                common.write_csv(os.path.join(self.session["dir"], "events.csv"), EVENT_CSV_HEADERS,
                                 [[common.now_text(now), "", "mark", "text", "", text, ""]], append=True)

    def call_nav_command(self, op_code, speed=0.0, percent=False):
        if not self.cmd_cli.wait_for_service(timeout_sec=2.0):
            return None, "/navigation/nav_command not available"
        req = self.NavCommand.Request(op=op_code, speed_limit=float(speed), percentage=bool(percent))
        r = wait_future(self.cmd_cli.call_async(req), 3.0)
        if r is None:
            return None, "nav_command timeout"
        return r, f"success={r.success} error_code={common.code_text(r.error_code)} {r.message}".strip()

    def op_nav_pause(self, cid, op, args):
        r, text = self.call_nav_command(1)
        self.reply(cid, op, bool(r and r.success), text)

    def op_nav_resume(self, cid, op, args):
        r, text = self.call_nav_command(2)
        self.reply(cid, op, bool(r and r.success), text)

    def op_nav_cancel(self, cid, op, args):
        self.stop_seq.set()
        r, text = self.call_nav_command(3)
        self.reply(cid, op, bool(r and r.success), text)

    def op_speed_limit(self, cid, op, args):
        r, text = self.call_nav_command(4, args.get("value", 0.0), args.get("percent", False))
        self.reply(cid, op, bool(r and r.success), text)

    def op_hard_stop(self, cid, op, args):
        # 硬停先发，保持期内再补两次；随后取消任务、停掉序列，避免保持期过后又被指令带起来
        for _ in range(3):
            self.pub_hard.publish(self.Empty())
            time.sleep(0.15)
        self.publish_marker("hard_stop")
        self.stop_seq.set()
        r, text = self.call_nav_command(3)
        self.reply(cid, op, True, f"hard stop sent; cancel: {text}")

    def op_nav_run(self, cid, op, args):
        kind = args.get("kind", "point")
        targets = [str(t) for t in args.get("targets", []) if str(t)]
        if kind not in ("point", "route") or not targets:
            self.reply(cid, op, False, "kind must be point|route and targets not empty")
            return
        if not args.get("confirm_clear"):
            self.reply(cid, op, False, "confirm_clear required: check the area is clear before moving")
            return
        with self.lock:
            if self.task["running"]:
                self.reply(cid, op, False, "a task sequence is already running")
                return
            repeat = max(1, int(args.get("repeat", 1)))
            self.task.update(running=True, kind=kind, targets=targets, index=0, total=repeat * len(targets),
                             current="", state="", distance=None)
        if not self.nav.wait_for_server(timeout_sec=3.0):
            with self.lock:
                self.task["running"] = False
            self.reply(cid, op, False, "/navigation/navigate not available")
            return
        self.stop_seq.clear()
        self.reply(cid, op, True, f"sequence started: {kind} x{len(targets)} x{repeat}")
        try:
            self.run_sequence(kind, targets, repeat, float(args.get("gap", 2.0)), bool(args.get("stop_on_fail")))
        finally:
            with self.lock:
                self.task.update(running=False, current="", state="", distance=None)
            self.emit({"t": common.now_text(), "type": "task_done", "stats": self.task["stats"]})

    def run_sequence(self, kind, targets, repeat, gap, stop_on_fail):
        n = 0
        for _ in range(repeat):
            for t in targets:
                if self.stop_seq.is_set():
                    return
                n += 1
                with self.lock:
                    self.task.update(index=n, current=t, state="PENDING", distance=None)
                row = self.run_one(kind, t)
                with self.lock:
                    self.task["rows"] = (self.task["rows"] + [row])[-50:]
                    self.task["stats"] = self.stats(self.task["rows"])
                    d = self.session["dir"] if self.session else self.out_root
                common.write_csv(os.path.join(d, "nav_tasks.csv"), TASK_CSV_HEADERS,
                                 [[row.get(h, "") for h in TASK_CSV_HEADERS]], append=True)
                self.emit({"t": common.now_text(), "type": "task", **row})
                if stop_on_fail and not row["success"]:
                    return
                end = time.time() + gap
                while time.time() < end and not self.stop_seq.is_set():
                    time.sleep(0.1)

    @staticmethod
    def stats(rows):
        ok = [r for r in rows if r["success"]]
        ts = [float(r["elapsed_s"]) for r in ok if r.get("elapsed_s") not in ("", None)]
        return {"total": len(rows), "ok": len(ok), "fail": len(rows) - len(ok),
                "min": round(min(ts), 2) if ts else None, "avg": round(sum(ts) / len(ts), 2) if ts else None,
                "max": round(max(ts), 2) if ts else None}

    def run_one(self, kind, target):
        g = self.Navigate.Goal()
        if kind == "route":
            g.target_type, g.route_name = 3, target
        else:
            g.target_type, g.point_name = 1, target
        row = {"start": common.now_text(), "end": "", "kind": kind, "target": target, "accepted": False,
               "success": False, "error_code": "", "error_text": "", "message": "", "elapsed_s": "", "task_id": ""}
        t0 = time.time()

        def on_fb(fb):
            f = fb.feedback
            with self.lock:
                self.task.update(state=NAV_STATE.get(f.state, str(f.state)), distance=round(f.distance_remaining, 2))
        gh = wait_future(self.nav.send_goal_async(g, feedback_callback=on_fb), 10.0)
        if gh is None or not gh.accepted:
            row.update(end=common.now_text(), message="goal not accepted" if gh else "no accept reply in 10 s")
            return row
        row["accepted"] = True
        self.goal_handle = gh
        rfut = gh.get_result_async()
        ev = threading.Event()
        rfut.add_done_callback(lambda _f: ev.set())
        cancelled = False
        while not ev.wait(0.1):
            if self.stop_seq.is_set() and not cancelled:
                cancelled = True
                gh.cancel_goal_async()
        res = rfut.result().result if rfut.done() and rfut.result() else None
        self.goal_handle = None
        row.update(end=common.now_text(), elapsed_s=f"{time.time() - t0:.2f}")
        if res is not None:
            row.update(success=bool(res.success), error_code=int(res.error_code),
                       error_text=common.code_text(res.error_code), message=res.message, task_id=res.task_id)
        return row


def main(argv=None):
    ap = argparse.ArgumentParser(prog="nbt serve", description="RViz 测试面板的后端（常驻在机器人上）")
    ap.add_argument("--out", default="~/nbt_eval", help="会话和结果的输出根目录")
    ap.add_argument("--rate", type=float, default=2.0, help="/nbt/status 发布频率 Hz")
    a = ap.parse_args(argv)
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init()
    node = rclpy.create_node("nbt_serve")
    serve = Serve(node, a.out, a.rate)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if serve.session:
            serve.op_session_stop("", "session_stop", {})
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
