"""nbt pathnav：经调度规划器（/navigate_through_graph_points）跑路网任务序列，判停稳，读导航反馈，录入划线结果。

  nbt pathnav plan --start 导航点1 --key 导航点2,放料点,导航点5,取盘点 -o plan.yaml --tutorial plan.html
                                                       # 从现场地图的路网生成任务计划 + 预览图 + 模拟教程
  nbt pathnav example > tray.yaml                      # 打印任务计划模板（蓝思 tray 示例，手写计划时参考）
  nbt pathnav targets --batch E075                     # 读地图资产点位 → <批次目录>/targets.json
  nbt pathnav resolve tray.yaml                        # 只解析不执行：每个任务能否解析、路线长度、是否倒车
  nbt pathnav run tray.yaml --batch E075 --laps 3 --record
  nbt pathnav cancel                                   # /schedule_planner/force_cancel，停车并取消当前任务
  nbt pathnav mode [--watch 20]                        # 当前导航模式：nav2 路网 / path_navigation（path_follower 或 controller）

run 的每个任务：下发 → 等结果 → 判停稳（里程计线速度 < 0.01 m/s 且角速度 < 0.02 rad/s 持续 0.5 s）
→ 取停稳后 1 s 的 map → 底盘平均位姿，与目标点位姿比较 → 终点是关键点位时暂停，录入划线判定
（只跑 1 圈时先提示拍照留存）→ 下一个任务。
V0.5.1 到点即上报成功、不等停稳，所以「上报成功」与「停稳」两个时刻都记。
输出在 <批次目录>/sessions/<会话号>/：tasks.csv（每任务一行）、run.json（计划、参数快照）、rosbag/（--record）。
批次目录默认 ~/nbt_eval/capability/<批次号>，-o 指定。运行号 = 批次号-日期-序号，同一批次内连续编号。
任务进行中按 Ctrl-C：取消当前任务、等停稳后退出（已写的记录保留）。
"""
import argparse
import glob
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time

import yaml

from nav_benchmark_tools import common, params, record_cmd
from nav_benchmark_tools import navlog as nl

ACTION = "/navigate_through_graph_points"
ASSETS = "/map_asset_manager/get_assets"
RESOLVE = "/map_asset_manager/resolve_nav_points"
FORCE_CANCEL = "/schedule_planner/force_cancel"
BACKEND_TOPIC = "/command_board/navigation_backend"
MODE_NODES = [("/bt_navigator", "nav2"), ("/planner_server", "nav2"), ("/controller_server", "nav2"),
              ("/schedule_planner_node", "path_navigation"), ("/path_follower_node", "path_navigation"),
              ("/velocity_smoother_path", "path_navigation")]
LIFECYCLE = {0: "unknown", 1: "unconfigured", 2: "inactive", 3: "active", 4: "finalized"}
PF_MODE = "path_navigation · path_follower"
GOAL_STATUS = {4: "succeeded", 5: "canceled", 6: "aborted"}

EXAMPLE = """\
# 能力摸底推荐速度测试的任务计划（蓝思 tray 布置示例）。点位名以 XWiz 中的实际名称为准。
# 每个关键点位单独作为一个任务的终点：路网执行器合并相邻路段，任务中间的点不会停下。
batch: E075                  # 批次号：E=空载 F=tray 搬运满载 W=满配重，后跟档位；命令行 --batch 优先
target_speed: 0.75           # 跟踪器速度上限应为此值，不一致时 run 拒绝开始
key_points: [导航点2, 放料点, 导航点5, 取盘点]   # 停在这些点时暂停录入划线判定
segment: [导航点3, 导航点4]   # 关键路段，speed analyze 统计它的速度
tasks:
  - {id: T1, points: [导航点1, 导航点2]}
  - {id: T2, points: [导航点2, 放料点]}
  - {id: T3, points: [放料点, 导航点2, 导航点3, 导航点4, 导航点5]}
  - {id: T4, points: [导航点5, 取盘点]}
  - {id: T5, points: [取盘点, 导航点5, 导航点6, 导航点1]}
"""

CSV_HEADERS = ["run_id", "batch", "session", "lap", "task", "points", "goal_id", "t_send", "t_accept", "t_report",
               "t_settle", "result", "message", "report_to_settle_s", "accept_to_settle_s", "route_len_m",
               "settled_x", "settled_y", "settled_yaw_deg", "err_lon_mm", "err_lat_mm", "err_yaw_deg",
               "key_point", "mark", "note"]
MARKS = {"y": "合格", "n": "不合格", "x": "受干扰", "": ""}


class Stop(Exception):
    pass


def batch_dir(batch, out):
    return os.path.abspath(os.path.expanduser(out or f"~/nbt_eval/capability/{batch}"))


def load_plan(path):
    with open(path, encoding="utf-8") as f:
        plan = yaml.safe_load(f) or {}
    tasks = plan.get("tasks") or []
    if not tasks:
        raise SystemExit(f"{path} 里没有 tasks")
    for i, t in enumerate(tasks):
        if not t.get("points") or len(t["points"]) < 2:
            raise SystemExit(f"{path} 第 {i + 1} 个任务点序列少于 2 个点")
        t.setdefault("id", f"T{i + 1}")
        t["points"] = [str(p) for p in t["points"]]
    plan["key_points"] = [str(p) for p in plan.get("key_points") or []]
    plan["segment"] = [str(p) for p in plan.get("segment") or []]
    return plan


def next_seq(bdir):
    """批次内已有运行号的最大序号 + 1。"""
    n = 0
    for p in glob.glob(os.path.join(bdir, "sessions", "*", "tasks.csv")):
        for r in nl.read_tasks_csv(p):
            try:
                n = max(n, int(r["run_id"].rsplit("-", 1)[-1]))
            except (KeyError, ValueError):
                continue
    return n + 1


def segment_length(seg):
    pts = [(p.x, p.y) for p in seg.control_points]
    if seg.type == 1 and len(pts) > 2:          # 贝塞尔：按 200 段折线近似弧长
        n = len(pts) - 1
        curve = []
        for k in range(201):
            u = k / 200.0
            x = y = 0.0
            for i, (px, py) in enumerate(pts):
                b = math.comb(n, i) * u ** i * (1 - u) ** (n - i)
                x, y = x + b * px, y + b * py
            curve.append((x, y))
        pts = curve
    return nl.path_length(pts)


class Robot:
    def __init__(self, node, base_frame="base_link", map_frame="map"):
        import tf2_ros
        from nav_msgs.msg import Odometry
        self.node, self.base, self.map = node, base_frame, map_frame
        self.odom = []
        self.stop = False
        node.create_subscription(Odometry, nl.ODOM, self._on_odom, 50)
        self.tf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tf, node)
        self.action = None

    def _on_odom(self, m):
        tw = m.twist.twist
        t = time.time()
        self.odom.append((t, nl.signed_speed(tw.linear.x, tw.linear.y), tw.angular.z))
        if len(self.odom) > 2000:
            del self.odom[:1000]

    def spin(self, sec, check_stop=True):
        import rclpy
        end = time.time() + sec
        while time.time() < end:
            if check_stop and self.stop:
                raise Stop()
            rclpy.spin_once(self.node, timeout_sec=min(0.05, max(0.0, end - time.time())))

    def pose(self):
        from rclpy.time import Time
        try:
            tr = self.tf.lookup_transform(self.map, self.base, Time())
        except Exception:  # noqa: BLE001  TF 还没到 / 断了
            return None
        p, q = tr.transform.translation, tr.transform.rotation
        return (time.time(), p.x, p.y, nl.yaw_of(q))

    def avg_pose(self, sec=1.0):
        samples, end = [], time.time() + sec
        while time.time() < end:
            p = self.pose()
            if p:
                samples.append(p)
            self.spin(0.02)
        return nl.mean_pose(samples), len(samples)

    def wait_still(self, still_v, still_w, hold, timeout, allow_stop=False):
        """等到停稳，返回停稳时刻；超时返回 None。allow_stop=True 时 Ctrl-C 不打断（取消后等停车用）。"""
        end, start = time.time() + timeout, None
        while time.time() < end:
            self.spin(0.05, check_stop=not allow_stop)
            recent = [o for o in self.odom if o[0] > time.time() - 0.3]
            if not recent:
                start = None
                continue
            if all(abs(v) < still_v and abs(w) < still_w for _, v, w in recent):
                start = start or recent[0][0]
                if time.time() - start >= hold:
                    return time.time()
            else:
                start = None
        return None

    def call(self, srv_type, name, req, timeout=5.0):
        cli = self.node.create_client(srv_type, name)
        try:
            if not cli.wait_for_service(timeout_sec=timeout):
                return None
            fut = cli.call_async(req)
            end = time.time() + timeout
            while not fut.done() and time.time() < end:
                import rclpy
                rclpy.spin_once(self.node, timeout_sec=0.05)
            return fut.result() if fut.done() else None
        finally:
            self.node.destroy_client(cli)

    def force_cancel(self):
        from std_srvs.srv import Trigger
        return self.call(Trigger, FORCE_CANCEL, Trigger.Request(), 3.0)


def mode_verdict(states, sp_mode, backend):
    """由各节点 lifecycle 状态、schedule_planner 的 mode、command_board 配置的后端判定当前导航模式。
    返回 (模式, 是否为本测试要求的 path_follower, [提醒])。"""
    bt, sp, pf, cs = (states.get(n) for n in ("/bt_navigator", "/schedule_planner_node", "/path_follower_node",
                                              "/controller_server"))
    notes = []
    if sp == "active" and bt == "active":
        mode, ok = "两套都激活（有人绕过 command_board 手动拉起了另一套）", False
    elif sp == "active":
        if sp_mode == "controller":
            mode, ok = "path_navigation · controller（schedule_planner 经 /follow_path 交给 controller_server）", False
            if cs != "active":
                notes.append("controller_server 没激活：controller 模式下每个路段都会失败")
        else:
            mode = PF_MODE
            ok = pf == "active"
            if not ok:
                notes.append(f"path_follower_node 状态是 {pf or '不在线'}，不是 active")
    elif bt == "active":
        mode, ok = "nav2 路网（bt_navigator → planner_server RouteGridPlanner → controller_server）", False
    else:
        mode, ok = "两套都没激活（command_board 还没完成 STARTUP，或导航没起来）", False
    active_stack = "path_navigation" if sp == "active" else ("nav2" if bt == "active" else None)
    if backend and active_stack and backend != active_stack and sp != bt:
        notes.append(f"command_board 配置的后端是 {backend}，实际激活的是 {active_stack}：有人绕过 command_board 手动切换过")
    return mode, ok, notes


def detect_mode(robot, timeout=2.0):
    """在线读：command_board 配置的后端、各节点 lifecycle 状态、schedule_planner mode、动作服务器。"""
    import rclpy
    from lifecycle_msgs.srv import GetState
    from rclpy.action import get_action_names_and_types
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    got = {}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    sub = robot.node.create_subscription(String, BACKEND_TOPIC, lambda m: got.update(v=m.data), qos)
    end = time.time() + timeout
    while time.time() < end and "v" not in got:
        rclpy.spin_once(robot.node, timeout_sec=0.05)
    robot.node.destroy_subscription(sub)
    states = {}
    for n, _ in MODE_NODES:
        r = robot.call(GetState, f"{n}/get_state", GetState.Request(), timeout)
        states[n] = None if r is None else LIFECYCLE.get(r.current_state.id, r.current_state.label)
    p = params.get_params(robot.node, "/schedule_planner_node", ["mode"], timeout)
    sp_mode = (p or {}).get("mode")
    actions = sorted(n for n, _ in get_action_names_and_types(robot.node))
    mode, ok, notes = mode_verdict(states, sp_mode, got.get("v"))
    return {"backend": got.get("v"), "states": states, "sp_mode": sp_mode, "actions": actions, "mode": mode,
            "ok": ok, "notes": notes}


def watch_motion(robot, sec):
    """sec 秒内统计各控制器的速度指令：哪一路真的在出速度。"""
    from geometry_msgs.msg import Twist
    from nav_interfaces.msg import TrackingPath
    cnt = {"/cmd_vel_nav_path": [0, 0], "/cmd_vel_nav": [0, 0], "/tracking_path": [0, 0]}

    def twist_cb(topic):
        def cb(m):
            cnt[topic][0] += 1
            cnt[topic][1] += 1 if math.hypot(m.linear.x, m.linear.y) > 0.01 or abs(m.angular.z) > 0.01 else 0
        return cb
    subs = [robot.node.create_subscription(Twist, t, twist_cb(t), 20) for t in ("/cmd_vel_nav_path", "/cmd_vel_nav")]

    def tp_cb(_m):
        cnt["/tracking_path"][0] += 1
    subs.append(robot.node.create_subscription(TrackingPath, "/tracking_path", tp_cb, 10))
    robot.spin(sec, check_stop=False)
    for sb in subs:
        robot.node.destroy_subscription(sb)
    return cnt


def print_mode(info):
    print(f"command_board 配置的后端：{info['backend'] or '（没收到 ' + BACKEND_TOPIC + '）'}")
    print(common.md_table(["节点", "所属后端", "lifecycle 状态"],
                          [[n, b, info["states"].get(n) or "不在线"] for n, b in MODE_NODES]))
    print(f"schedule_planner mode：{info['sp_mode'] or '读不到'}")
    nav_actions = [a for a in info["actions"] if a in ("/navigate_through_graph_points", "/navigate_through_poses",
                                                       "/navigate_to_pose", "/follow_path")]
    print(f"导航相关动作服务器：{'、'.join(nav_actions) or '无'}")
    print(f"\n{'✓' if info['ok'] else '✗'} 当前导航模式：{info['mode']}")
    for n in info["notes"]:
        print(f"⚠ {n}")


def cmd_mode(a):
    import rclpy
    rclpy.init()
    node = rclpy.create_node("nbt_pathnav")
    try:
        robot = Robot(node)
        info = detect_mode(robot, a.timeout)
        print_mode(info)
        if a.watch > 0:
            print(f"\n观察 {a.watch:g} s 内实际出速度的控制器（这段时间要有任务在跑）…")
            cnt = watch_motion(robot, a.watch)
            print(common.md_table(["话题", "谁发", "消息数", "其中非零"],
                                  [["/cmd_vel_nav_path", "path_follower", *cnt["/cmd_vel_nav_path"]],
                                   ["/cmd_vel_nav", "controller_server（nav2 或 controller 模式）", *cnt["/cmd_vel_nav"]],
                                   ["/tracking_path", "schedule_planner（只在 path_follower 模式下发）",
                                    cnt["/tracking_path"][0], "—"]]))
            pf, cs = cnt["/cmd_vel_nav_path"][1], cnt["/cmd_vel_nav"][1]
            if pf and not cs:
                print("✓ 实际出速度的是 path_follower")
            elif cs and not pf:
                print("✗ 实际出速度的是 controller_server（不是 path_follower）")
            elif pf and cs:
                print("⚠ 两路都有非零速度：两套导航都在出指令，先停车排查")
            else:
                print("（这段时间没有非零速度指令：没有任务在跑，判不了）")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if info["ok"] else 1


def fetch_targets(robot, timeout=5.0):
    from nav_interfaces.srv import GetAssets
    r = robot.call(GetAssets, ASSETS, GetAssets.Request(kinds=1), timeout)
    if r is None:
        return None, f"{ASSETS} 不在线或无应答"
    if not r.success:
        return None, f"error_code={common.code_text(r.error_code)} {r.message}"
    return {"map_name": r.map_name, "revision": r.revision, "fetched_at": common.now_text(),
            "points": {p.name: {"x": p.x, "y": p.y, "theta": p.theta, "type": p.type} for p in r.points}}, ""


def resolve_task(robot, points, timeout=5.0):
    """返回 (成功, 说明, 路线长度 m, 段数, 倒车段数)。"""
    from nav_interfaces.srv import ResolveNavPoints
    r = robot.call(ResolveNavPoints, RESOLVE, ResolveNavPoints.Request(nav_point_ids=points, nav_point_names=points),
                   timeout)
    if r is None:
        return False, f"{RESOLVE} 不在线或无应答", 0.0, 0, 0
    if not r.success:
        return False, r.message, 0.0, 0, 0
    return True, r.message, sum(segment_length(s) for s in r.segments), len(r.segments), \
        sum(1 for s in r.segments if s.backward)


def run_goal(robot, points, timeout):
    """下发一个路网任务并等结果，返回 dict（goal_id、各时刻、结果）。Ctrl-C 时取消并抛 Stop。"""
    from rclpy.action import ActionClient
    from nav_interfaces.action import NavigateThroughGraphPoints as A
    if robot.action is None:
        robot.action = ActionClient(robot.node, A, ACTION)
    g = A.Goal(nav_point_ids=points, nav_point_names=points)
    out = {"goal_id": "", "t_send": time.time(), "t_accept": None, "t_report": None, "result": "", "message": ""}
    last = {"s": None}

    def on_fb(fb):
        f = fb.feedback
        s = f"{f.current_status} {f.current_point_index}/{f.total_points}"
        if s != last["s"]:
            print(f"    {time.time() - out['t_send']:6.2f}s  {s}", flush=True)
            last["s"] = s

    fut = robot.action.send_goal_async(g, feedback_callback=on_fb)
    end = time.time() + 10.0
    try:
        while not fut.done():
            if time.time() > end:
                out.update(result="no_response", message="10 s 内未收到受理回复")
                return out
            robot.spin(0.05)
    except Stop:
        print("    受理前按了 Ctrl-C，调 force_cancel 确保不执行…", flush=True)
        robot.force_cancel()
        out.update(result="canceled", message="受理前中断", t_report=time.time())
        raise Stop(out)
    gh = fut.result()
    out["goal_id"] = nl.uuid_hex(gh.goal_id.uuid)
    if not gh.accepted:
        out.update(result="rejected", message="调度规划器拒绝受理")
        return out
    out["t_accept"] = time.time()
    rfut = gh.get_result_async()
    end = time.time() + timeout
    try:
        while not rfut.done():
            if time.time() > end:
                out["message"] = f"{timeout:.0f} s 内没有结果，已取消"
                raise Stop()
            robot.spin(0.05)
    except Stop:
        robot.stop = False
        print("    取消当前任务…", flush=True)
        robot.force_cancel()
        cfut = gh.cancel_goal_async()
        end = time.time() + 5.0
        while time.time() < end and not (cfut.done() and rfut.done()):
            robot.spin(0.05)
        robot.stop = True
        out.update(result=out["result"] or "canceled", t_report=time.time())
        raise Stop(out)
    res = rfut.result()
    out["t_report"] = time.time()
    out["result"] = GOAL_STATUS.get(res.status, str(res.status))
    out["message"] = res.result.message
    if res.status == 4 and not res.result.success:
        out["result"] = "aborted"
    return out


def ask_mark(end_name, fb_text, photo=False):
    """录入划线判定，返回 合格 / 不合格 / 受干扰 / 空。photo=True（单圈运行）时先提示拍照留存。
    录入期间恢复默认 SIGINT：Ctrl-C 抛 KeyboardInterrupt，能从 input() 里退出来。"""
    old = signal.signal(signal.SIGINT, signal.default_int_handler)
    try:
        print(f"  ▶ 关键点位 {end_name}：{fb_text}")
        if photo:
            print("    📷 单圈运行：拍一张俯视照片留存（标记点和划线框都要拍清）")
        while True:
            m = input("    划线判定 [y 合格 / n 不合格 / x 受干扰 / 回车跳过]：").strip().lower()
            if m in MARKS:
                return MARKS[m]
            print("    只能输入 y / n / x 或直接回车")
    finally:
        signal.signal(signal.SIGINT, old)


def start_record(session):
    topics, hidden, _ = record_cmd.topics_for("SPEED")
    out = os.path.join(session, "rosbag")
    cmd = ["ros2", "bag", "record", "-o", out] + (["--include-hidden-topics"] if hidden else []) + topics
    log = open(os.path.join(session, "rosbag_record.log"), "w")
    # 新会话：终端 Ctrl-C 不直接打到 rosbag，由本程序在车停稳后再结束录包
    p = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    return p, log, out


def stop_record(p, log):
    if p and p.poll() is None:
        p.send_signal(signal.SIGINT)
        try:
            p.wait(timeout=30)
        except subprocess.TimeoutExpired:
            p.terminate()
    if log:
        log.close()


def cmd_plan(a):
    from nav_benchmark_tools import pathplan as pp
    keys = [k.strip() for k in a.key.replace("，", ",").split(",") if k.strip()]
    seg = [x.strip() for x in a.segment.replace("，", ",").split(",")] if a.segment else None
    if seg is not None and len(seg) != 2:
        raise SystemExit("--segment 要写两个点，用逗号分隔")
    resolved, limits, node = {}, None, None
    try:
        if a.assets:
            norm = pp.load_assets_file(a.assets)
            print(f"# 离线生成：{a.assets}（不做解析校验，上机后跑 nbt pathnav resolve <计划>）")
        else:
            import rclpy
            from nav_interfaces.srv import GetAssets
            rclpy.init()
            node = rclpy.create_node("nbt_pathnav")
            robot = Robot(node)
            r = robot.call(GetAssets, ASSETS, GetAssets.Request(kinds=pp.KINDS_FOR_PLAN), a.timeout)
            if r is None:
                print(f"✗ {ASSETS} 不在线或无应答")
                return 2
            if not r.success:
                print(f"✗ 读地图资产失败：error_code={common.code_text(r.error_code)} {r.message}")
                return 2
            from rosidl_runtime_py import message_to_ordereddict
            norm = pp.normalize(json.loads(json.dumps(message_to_ordereddict(r), default=str)))
            fp = params.get_params(node, params.FOLLOWER, params.SPEED_PARAMS[params.FOLLOWER][1:])
            if fp and all(v is not None for v in fp.values()):
                limits = {"acc": fp["max_linear_acceleration"], "dec": fp["max_linear_deceleration"],
                          "jerk": fp["max_linear_jerk"]}
        n_nav = sum(1 for e in norm["edges"] if e["nav"])
        print(f"# 地图 {norm['map_name']}（revision {norm['revision']}）：点位 {len(norm['points'])} 个，路网边 "
              f"{len(norm['edges'])} 条（导航点直连 {n_nav} 条）")
        try:
            plan = pp.make_plan(norm, a.start, keys, seg, close=not a.no_return, target_speed=a.target_speed,
                                limits=limits)
        except pp.PlanError as e:
            print(f"✗ {e}")
            return 2
        print(f"起点 {plan['start']}；关键点位 {' → '.join(plan['keys'])}" + ("；回到起点" if plan["close"] else ""))
        if node is not None and not a.no_resolve:
            for t in plan["tasks"]:
                ok, msg, length, _, nback = resolve_task(robot, t["points"])
                resolved[t["id"]] = (ok, msg, length, nback)
    finally:
        if node is not None:
            node.destroy_node()
            import rclpy
            rclpy.shutdown()
    rows, bad = [], 0
    for t in plan["tasks"]:
        ok, msg, length, nback = resolved.get(t["id"], (None, "", t["len"], t["back"]))
        bad += 1 if ok is False else 0
        rows.append([t["id"], " → ".join(t["points"]), f"{length:.2f}", nback,
                     {True: "✓", False: "✗ " + msg, None: "未校验"}[ok]])
    print("\n" + common.md_table(["任务", "点序列", "路线长度 (m)", "倒车段", "解析"], rows))
    sg = plan["segment"]
    cands = sg.get("candidates", [])
    if cands:
        speed = f"{a.target_speed:g}"
        print(f"\n关键路段候选（直线前进路段，相邻共线的边合并；按预计在 {speed} m/s 保持的时长排序）：")
        print(common.md_table(
            ["#", "路段", "长度 (m)", "进 / 出", f"预计 {speed} 稳态 (s)", "能否跑到"],
            [[i + 1, " → ".join(c["via"]), f"{c['len']:.2f}",
              f"{'拐角' if c['v_in'] else '起步'} / {'拐角' if c['v_out'] else '停车'}", f"{c['steady']:.1f}",
              "✓ 能保持 2 s" if c["steady"] >= 2.0 else ("只能摸到" if c["reach"] else "✗ 跑不到")]
             for i, c in enumerate(cands)]))
    print(f"\n关键路段：{' → '.join(sg.get('via') or [sg['from'], sg['to']])}，{pp.seg_note(sg, a.target_speed)}"
          f"（从静止起步、停车到底时 2 s 稳态需要 {sg['need']:.2f} m）")
    for w in plan["warnings"]:
        print(f"⚠ {w}")
    if bad:
        print(f"\n✗ {bad} 个任务解析失败，没有写出计划；按「解析」列修路网或换关键点位")
        return 1
    out = os.path.abspath(os.path.expanduser(a.out))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        f.write(pp.plan_yaml(plan, a.batch))
    svg = os.path.abspath(os.path.expanduser(a.preview or os.path.splitext(out)[0] + ".svg"))
    with open(svg, "w", encoding="utf-8") as f:
        f.write(pp.preview_svg(norm, plan))
    print(f"\n✓ 计划已写入 {out}\n  预览图 {svg}（用浏览器打开，和现场路网图对照）")
    if a.tutorial:
        html = os.path.abspath(os.path.expanduser(a.tutorial))
        with open(html, "w", encoding="utf-8") as f:
            sc = pp.scene(norm, plan, a.batch)
            f.write(pp.render_tutorial(sc))
        course = "直线最大速度" if sc["course"] == "max" else "推荐速度"
        print(f"  模拟教程 {html}（{course}课程；用浏览器打开，按这份计划和现场地图练一遍）")
    return 0


def cmd_targets(a):
    import rclpy
    rclpy.init()
    node = rclpy.create_node("nbt_pathnav")
    try:
        robot = Robot(node)
        t, err = fetch_targets(robot, a.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if t is None:
        print(f"✗ {err}")
        return 2
    bdir = batch_dir(a.batch, a.out)
    os.makedirs(bdir, exist_ok=True)
    path = os.path.join(bdir, "targets.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(t, f, ensure_ascii=False, indent=2)
    print(f"# 地图 {t['map_name']}（revision {t['revision']}），点位 {len(t['points'])} 个\n")
    print(common.md_table(["名称", "类型", "x", "y", "θ(°)"],
                          [[n, p["type"], f"{p['x']:.3f}", f"{p['y']:.3f}", f"{math.degrees(p['theta']):.1f}"]
                           for n, p in t["points"].items()]))
    print(f"\n已保存 {path}")
    return 0


def cmd_resolve(a):
    plan = load_plan(a.plan)
    import rclpy
    rclpy.init()
    node = rclpy.create_node("nbt_pathnav")
    rows, bad = [], 0
    try:
        robot = Robot(node)
        for t in plan["tasks"]:
            ok, msg, length, nseg, nback = resolve_task(robot, t["points"], a.timeout)
            bad += 0 if ok else 1
            rows.append([t["id"], " → ".join(t["points"]), "✓" if ok else "✗", f"{length:.2f}" if ok else "",
                         nseg if ok else "", nback if ok else "", msg])
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(common.md_table(["任务", "点序列", "解析", "路线长度 (m)", "段数", "倒车段", "说明"], rows))
    return 0 if bad == 0 else 1


def summarize(rows, key_points):
    out = []
    for kp in key_points:
        rs = [r for r in rows if r["points"].split(">")[-1] == kp and r["result"] == "succeeded"]
        if not rs:
            continue

        def mx(k):
            vals = [abs(float(r[k])) for r in rs if r[k] not in ("", None)]
            return f"{max(vals):.1f}" if vals else "—"
        marks = [r["mark"] for r in rs]
        out.append([kp, len(rs), marks.count("合格"), marks.count("不合格"), marks.count("受干扰"),
                    mx("err_lon_mm"), mx("err_lat_mm"), mx("err_yaw_deg")])
    return out


def cmd_run(a):
    plan = load_plan(a.plan)
    batch = (a.batch or plan.get("batch") or "").strip()
    if not batch:
        raise SystemExit("缺批次号：--batch E075，或在计划里写 batch")
    target_speed = a.speed if a.speed is not None else plan.get("target_speed")
    bdir = batch_dir(batch, a.out)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    sid = f"{batch}-{stamp}"
    session = os.path.join(bdir, "sessions", sid)     # 前置检查全过才建目录，被拒绝的运行不留空会话
    csv_path = os.path.join(session, "tasks.csv")
    seq = next_seq(bdir)
    day = time.strftime("%Y%m%d")
    key_points = set(plan["key_points"])
    prompt = not a.no_prompt and sys.stdin.isatty()

    import rclpy
    rclpy.init()
    node = rclpy.create_node("nbt_pathnav")
    robot = Robot(node, a.base_frame)
    signal.signal(signal.SIGINT, lambda *_: setattr(robot, "stop", True))
    rec, rec_log, rows = None, None, []
    run_info = {"batch": batch, "session": sid, "plan": plan, "args": vars(a), "started": common.now_text()}
    try:
        print(f"# 批次 {batch} 会话 {sid}\n  输出 {session}（前置检查通过后创建）")
        from nav_interfaces.action import NavigateThroughGraphPoints as A
        from rclpy.action import ActionClient
        robot.action = ActionClient(node, A, ACTION)
        if not robot.action.wait_for_server(timeout_sec=5.0):
            print(f"✗ {ACTION} 不在线（导航后端不是 path_navigation，或调度规划器没起）")
            return 2
        if not a.skip_mode_check:
            info = detect_mode(robot)
            run_info["nav_mode"] = {k: info[k] for k in ("backend", "states", "sp_mode", "mode", "ok")}
            if not info["ok"]:
                print_mode(info)
                print(f"✗ 本测试要求 {PF_MODE}（schedule_planner mode=independence，path_follower 激活）；"
                      f"速度档改的是 path_follower 的参数，别的模式下测出来的数据无效。确认导航模式后再跑"
                      f"（只调试时加 --skip-mode-check）")
                return 2
            print(f"  导航模式：{info['mode']}")
        robot.spin(1.0)
        if not robot.odom:
            print(f"✗ 1 s 内没收到 {nl.ODOM}")
            return 2
        if robot.pose() is None:
            print(f"✗ 取不到 TF {robot.map} → {robot.base}（定位没起，或底盘坐标系不是 {robot.base}，用 --base-frame）")
            return 2

        snap = params.snapshot(node)
        run_info["params"] = snap
        f_now = (snap.get(params.FOLLOWER) or {}).get("max_linear_velocity")
        s_now = (snap.get(params.SMOOTHER) or {}).get("max_velocity")
        print(f"  跟踪器速度上限 {f_now}，平滑器上限 {s_now}")
        if target_speed is not None and (f_now is None or abs(float(f_now) - float(target_speed)) > 1e-6):
            print(f"✗ 跟踪器速度上限 {f_now} ≠ 计划档位 {target_speed}：先 nbt speed profile set {target_speed}"
                  f"（不按档位跑加 --speed 当前值）")
            return 2

        tpath = os.path.join(bdir, "targets.json")
        if a.refresh_targets or not os.path.exists(tpath):
            t, err = fetch_targets(robot)
            if t is None:
                print(f"✗ 读地图资产失败：{err}")
                return 2
            with open(tpath, "w", encoding="utf-8") as f:
                json.dump(t, f, ensure_ascii=False, indent=2)
        targets = nl.load_targets(tpath)
        need = {p for t in plan["tasks"] for p in t["points"]} | key_points | set(plan["segment"])
        miss = sorted(p for p in need if p not in targets)
        if miss:
            print(f"✗ 地图资产里没有这些点位：{', '.join(miss)}（点位名以 XWiz 为准；地图换过加 --refresh-targets）")
            return 2

        lengths = {}
        if not a.skip_resolve:
            for t in plan["tasks"]:
                ok, msg, length, _, nback = resolve_task(robot, t["points"])
                if not ok:
                    print(f"✗ {t['id']} 解析失败：{msg}")
                    return 2
                lengths[t["id"]] = length
                print(f"  {t['id']} 可解析，路线 {length:.2f} m" + (f"，倒车段 {nback}" if nback else ""))
        run_info["route_len_m"] = lengths

        if not 1 <= a.start_task <= len(plan["tasks"]):
            print(f"✗ --start-task 应在 1～{len(plan['tasks'])}")
            return 2
        first = plan["tasks"][a.start_task - 1]["points"][0]
        p = robot.pose()
        tg = targets[first]
        dist = math.hypot(p[1] - tg["x"], p[2] - tg["y"])
        if dist > a.start_tol:
            print(f"✗ 车离起点 {first} {dist:.2f} m（> {a.start_tol} m）：先把车开到 {first}，或加 --start-tol")
            return 2

        os.makedirs(session, exist_ok=True)
        shutil.copy(a.plan, os.path.join(session, "plan.yaml"))
        shutil.copy(tpath, os.path.join(session, "targets.json"))
        with open(os.path.join(session, "run.json"), "w", encoding="utf-8") as f:
            json.dump(run_info, f, ensure_ascii=False, indent=2, default=str)
        if a.record:
            rec, rec_log, bag = start_record(session)
            print(f"  录包 → {bag}（等 3 s 让录包订阅上话题）")
            robot.spin(3.0)
            if rec.poll() is not None:
                print(f"✗ 录包进程已退出，见 {session}/rosbag_record.log")
                return 2

        total = a.laps * len(plan["tasks"]) - (a.start_task - 1)
        n = 0
        for lap in range(a.start_lap, a.start_lap + a.laps):
            for ti, t in enumerate(plan["tasks"]):
                if lap == a.start_lap and ti < a.start_task - 1:
                    continue
                n += 1
                run_id = f"{batch}-{day}-{seq:03d}"
                seq += 1
                end_name = t["points"][-1]
                print(f"\n[{n}/{total}] 第 {lap} 圈 {t['id']} {' → '.join(t['points'])}  运行号 {run_id}")
                if robot.wait_still(a.still_v, a.still_w, a.still_hold, 10.0) is None:
                    print("  ⚠ 10 s 内没停稳，仍下发（记录备注）")
                row = {h: "" for h in CSV_HEADERS}
                row.update(run_id=run_id, batch=batch, session=sid, lap=lap, task=t["id"],
                           points=">".join(t["points"]), route_len_m=f"{lengths.get(t['id'], 0.0):.3f}"
                           if t["id"] in lengths else "", key_point="y" if end_name in key_points else "n")
                stopped = False
                try:
                    g = run_goal(robot, t["points"], a.timeout)
                except Stop as e:
                    g = e.args[0] if e.args else {}
                    stopped = True
                row.update({k: g.get(k, "") for k in ("goal_id", "result", "message")})
                for k in ("t_send", "t_accept", "t_report"):
                    row[k] = f"{g[k]:.3f}" if g.get(k) else ""
                mark = ""
                if row["result"] == "succeeded":
                    try:
                        ts = robot.wait_still(a.still_v, a.still_w, a.still_hold, a.max_settle)
                    except Stop:
                        ts, stopped = None, True
                        robot.stop = False
                    if ts is None:
                        row["note"] = f"{a.max_settle:.0f} s 内未停稳"
                        print(f"  ⚠ {row['note']}")
                    else:
                        row["t_settle"] = f"{ts:.3f}"
                        row["report_to_settle_s"] = f"{ts - g['t_report']:.2f}"
                        row["accept_to_settle_s"] = f"{ts - g['t_accept']:.2f}"
                        try:
                            pose, _ = robot.avg_pose(a.avg_window)
                        except Stop:
                            pose, stopped = None, True
                            robot.stop = False
                        if pose:
                            tg = targets[end_name]
                            e = nl.target_error(*pose, tg["x"], tg["y"], tg["theta"])
                            row.update(settled_x=f"{pose[0]:.4f}", settled_y=f"{pose[1]:.4f}",
                                       settled_yaw_deg=f"{math.degrees(pose[2]):.3f}",
                                       err_lon_mm=f"{e['longitudinal'] * 1000:.1f}",
                                       err_lat_mm=f"{e['lateral'] * 1000:.1f}",
                                       err_yaw_deg=f"{math.degrees(e['yaw']):.3f}")
                        fb = (f"导航反馈 纵 {row['err_lon_mm'] or '—'} mm  横 {row['err_lat_mm'] or '—'} mm  "
                              f"朝向 {row['err_yaw_deg'] or '—'}°（上报→停稳 {row['report_to_settle_s']} s）")
                        if end_name in key_points and prompt:
                            try:
                                mark = ask_mark(end_name, fb, photo=a.laps == 1)
                            except (KeyboardInterrupt, EOFError):
                                print("\n  已中断录入")
                                stopped = True
                        else:
                            print(f"  ✓ 停在 {end_name}：{fb}")
                    row["mark"] = mark
                else:
                    print(f"  ✗ {row['result']}  {row['message']}")
                    robot.wait_still(a.still_v, a.still_w, a.still_hold, 10.0, allow_stop=True)
                common.write_csv(csv_path, CSV_HEADERS, [[row[h] for h in CSV_HEADERS]], append=True)
                rows.append(row)
                if row["mark"] == "不合格" and a.stop_on_mark_fail:
                    print("  划线不合格，按 --stop-on-mark-fail 停止")
                    stopped = True
                if stopped or robot.stop or (row["result"] != "succeeded" and not a.continue_on_fail):
                    robot.stop = False
                    raise Stop()
                if a.dwell > 0:
                    robot.spin(a.dwell)
    except Stop:
        print("\n已停止。")
        robot.stop = False
        robot.wait_still(a.still_v, a.still_w, a.still_hold, 10.0, allow_stop=True)
    finally:
        if rec:
            print("  结束录包…")
            stop_record(rec, rec_log)
        node.destroy_node()
        rclpy.shutdown()

    ok = sum(1 for r in rows if r["result"] == "succeeded")
    print(f"\n共 {len(rows)} 个任务，成功 {ok}；记录 {csv_path}")
    s = summarize(rows, plan["key_points"])
    if s:
        print("\n" + common.md_table(["关键点位", "样本", "合格", "不合格", "受干扰", "纵 max (mm)", "横 max (mm)",
                                      "朝向 max (°)"], s))
    print(f"\n离线分析：nbt speed analyze {bdir}")
    return 0 if rows and ok == len(rows) else 1


def main(argv=None):
    ap = argparse.ArgumentParser(prog="nbt pathnav", description="经调度规划器跑路网任务（path_navigation）")
    sub = ap.add_subparsers(dest="op", required=True)
    sub.add_parser("example", help="打印任务计划模板（手写计划时参考）")
    p = sub.add_parser("plan", help="从现场地图的路网生成任务计划、预览图和模拟教程")
    p.add_argument("--start", required=True, help="起点，也是每圈回到的点")
    p.add_argument("--key", required=True, help="关键点位，按经过顺序用逗号分隔")
    p.add_argument("--segment", default="", help="关键路段「起点,终点」（按行驶方向）；默认自动取预计稳态最长的直线前进路段")
    p.add_argument("--batch", default="E075", help="写进计划的批次号")
    p.add_argument("--target-speed", type=float, default=0.75, help="写进计划的档位 m/s")
    p.add_argument("--no-return", action="store_true", help="最后一个关键点位之后不回起点")
    p.add_argument("--assets", default="", help="离线：用 nbt assets --json 导出的文件代替在线读地图")
    p.add_argument("--no-resolve", action="store_true", help="在线时也不做解析校验")
    p.add_argument("-o", "--out", default="plan.yaml", help="计划文件，默认 ./plan.yaml")
    p.add_argument("--preview", default="", help="预览图 SVG，默认与计划同名 .svg")
    p.add_argument("--tutorial", default="", help="同时生成模拟教程 HTML")
    p.add_argument("--timeout", type=float, default=5.0)
    p = sub.add_parser("targets", help="读地图资产点位，存 <批次目录>/targets.json")
    p.add_argument("--batch", required=True)
    p.add_argument("-o", "--out", default="", help="批次目录，默认 ~/nbt_eval/capability/<批次号>")
    p.add_argument("--timeout", type=float, default=5.0)
    p = sub.add_parser("resolve", help="只解析不执行，检查计划里每个任务")
    p.add_argument("plan")
    p.add_argument("--timeout", type=float, default=5.0)
    sub.add_parser("cancel", help="调 /schedule_planner/force_cancel 停车并取消当前任务")
    p = sub.add_parser("mode", help="当前导航模式：nav2 路网 / path_navigation（path_follower 或 controller）")
    p.add_argument("--watch", type=float, default=0.0, help="另外观察 N 秒内实际出速度的控制器（要有任务在跑）")
    p.add_argument("--timeout", type=float, default=2.0)
    p = sub.add_parser("run", help="按计划跑任务")
    p.add_argument("plan")
    p.add_argument("--batch", default="", help="批次号，如 E075；默认取计划里的 batch")
    p.add_argument("-o", "--out", default="", help="批次目录，默认 ~/nbt_eval/capability/<批次号>")
    p.add_argument("--laps", type=int, default=1, help="圈数")
    p.add_argument("--start-lap", type=int, default=1, help="从第几圈起编号（中断后续跑用）")
    p.add_argument("--start-task", type=int, default=1, help="第一圈从第几个任务开始（中断后续跑用）")
    p.add_argument("--record", action="store_true", help="同时录包（SPEED 预设话题），存会话目录 rosbag/")
    p.add_argument("--speed", type=float, default=None, help="期望的跟踪器速度上限，默认取计划 target_speed")
    p.add_argument("--no-prompt", action="store_true", help="关键点位不暂停录入（划线结果事后补进 tasks.csv）")
    p.add_argument("--dwell", type=float, default=0.0, help="每个任务结束后额外停留秒数")
    p.add_argument("--continue-on-fail", action="store_true", help="任务失败也继续（默认失败即停）")
    p.add_argument("--stop-on-mark-fail", action="store_true", help="划线不合格就停")
    p.add_argument("--refresh-targets", action="store_true", help="重新读地图资产（地图或点位改过时）")
    p.add_argument("--skip-resolve", action="store_true", help="开始前不做「只解析」检查")
    p.add_argument("--skip-mode-check", action="store_true", help="不检查导航模式（仅调试用）")
    p.add_argument("--start-tol", type=float, default=0.5, help="车离第一个任务起点的允许距离 m")
    p.add_argument("--timeout", type=float, default=600.0, help="单个任务等结果的上限 s，超时取消")
    p.add_argument("--base-frame", default="base_link")
    p.add_argument("--still-v", type=float, default=nl.DEFAULTS["still_v"])
    p.add_argument("--still-w", type=float, default=nl.DEFAULTS["still_w"])
    p.add_argument("--still-hold", type=float, default=nl.DEFAULTS["still_hold"])
    p.add_argument("--max-settle", type=float, default=nl.DEFAULTS["max_settle"], help="上报后等停稳的上限 s")
    p.add_argument("--avg-window", type=float, default=nl.DEFAULTS["avg_window"])
    a = ap.parse_args(argv)
    if a.op == "example":
        print(EXAMPLE, end="")
        return 0
    if a.op == "plan":
        return cmd_plan(a)
    if a.op == "mode":
        return cmd_mode(a)
    if a.op == "targets":
        return cmd_targets(a)
    if a.op == "resolve":
        return cmd_resolve(a)
    if a.op == "cancel":
        import rclpy
        rclpy.init()
        node = rclpy.create_node("nbt_pathnav")
        try:
            r = Robot(node).force_cancel()
        finally:
            node.destroy_node()
            rclpy.shutdown()
        print(f"✗ {FORCE_CANCEL} 不在线或无应答" if r is None else f"{'✓' if r.success else '✗'} {r.message}")
        return 0 if r is not None and r.success else 1
    return cmd_run(a)


if __name__ == "__main__":
    sys.exit(main())
