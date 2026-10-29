"""nbt nav：经导航门面下发任务和控制命令，打印过程与结果，每个任务追加一行到 CSV。

  nbt nav goto P1                       # 去点位 P1（/navigation/navigate，target_type=1）
  nbt nav goto P1 P2 --repeat 10        # P1、P2 轮流去，共 10 轮（重复到点、往返）
  nbt nav pose 1.0 2.0 90               # 去 map 系位姿，yaw 单位度
  nbt nav route R1 --repeat 5           # 走路线 R1 五次
  nbt nav pause | resume | cancel       # /navigation/nav_command，默认当前任务
  nbt nav speed 0.3                     # 限速 0.3 m/s；--percent 按最大速度百分比；0 = 解除
  nbt nav state                         # 打印一次 /navigation/state
任务进行中按 Ctrl-C：取消当前任务后退出。
耗时是本机「发出目标 → 收到结果」的时间，属于诊断口径；用户感知耗时以 XWiz 端计时为准。
"""
import argparse
import math
import os
import signal
import sys
import time

from nav_benchmark_tools import common

ACTION = "/navigation/navigate"
COMMAND = "/navigation/nav_command"
STATE_TOPIC = "/navigation/state"
STATE = {0: "IDLE", 1: "PENDING", 2: "RUNNING", 3: "PAUSED", 4: "BLOCKED", 5: "SUCCEEDED", 6: "FAILED", 7: "CANCELLED"}
CSV_HEADERS = ["start", "end", "target_type", "target", "accepted", "success", "error_code", "error_text",
               "message", "elapsed_s", "task_id", "states"]


class Interrupted(Exception):
    pass


def default_csv():
    return os.path.expanduser("~/nav_eval/nav_tasks.csv")


def yaw_quat(deg):
    h = math.radians(deg) / 2.0
    return 0.0, 0.0, math.sin(h), math.cos(h)


class NavClient:
    def __init__(self, node):
        from rclpy.action import ActionClient
        from nav_interfaces.action import Navigate
        self.node, self.Navigate = node, Navigate
        self.client = ActionClient(node, Navigate, ACTION)
        self.stop = False

    def spin_until(self, fut, timeout=None):
        import rclpy
        end = None if timeout is None else time.time() + timeout
        while not fut.done():
            if self.stop:
                raise Interrupted()
            if end is not None and time.time() > end:
                return False
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return True

    def run(self, target_type, target, pose=None, frame="map"):
        """下发一个目标并等结果，返回 CSV 一行。"""
        g = self.Navigate.Goal()
        g.target_type = target_type
        if target_type == 1:
            g.point_name = target
        elif target_type == 3:
            g.route_name = target
        else:
            x, y, yaw = pose
            g.pose.header.frame_id = frame
            g.pose.header.stamp = self.node.get_clock().now().to_msg()
            g.pose.pose.position.x, g.pose.pose.position.y = x, y
            q = yaw_quat(yaw)
            (g.pose.pose.orientation.x, g.pose.pose.orientation.y,
             g.pose.pose.orientation.z, g.pose.pose.orientation.w) = q
        kind = {1: "point", 2: "pose", 3: "route"}[target_type]
        row = {"start": common.now_text(), "target_type": kind, "target": target, "accepted": False,
               "success": False, "error_code": "", "error_text": "", "message": "", "elapsed_s": "",
               "task_id": "", "states": "", "end": ""}
        t0 = time.time()
        states, last = [], {"state": None, "t": 0.0}

        def on_fb(fb):
            f = fb.feedback
            now = time.time()
            if f.state != last["state"]:
                states.append(STATE.get(f.state, str(f.state)))
                print(f"  {now - t0:7.2f}s  {STATE.get(f.state, f.state):<9} 剩余 {f.distance_remaining:.1f} m"
                      + (f"，第 {f.current_index} 个点，还剩 {f.poses_remaining} 个" if target_type == 3 else ""),
                      flush=True)
                last.update(state=f.state, t=now)
            elif now - last["t"] >= 2.0:
                print(f"  {now - t0:7.2f}s  {STATE.get(f.state, f.state):<9} 剩余 {f.distance_remaining:.1f} m", flush=True)
                last["t"] = now

        print(f"→ 下发 {kind} {target}（{row['start']}）")
        fut = self.client.send_goal_async(g, feedback_callback=on_fb)
        if not self.spin_until(fut, 10.0) or fut.result() is None:
            row.update(message="10 s 内未收到受理回复", end=common.now_text())
            print("  ✗ " + row["message"])
            return row
        gh = fut.result()
        row["accepted"] = gh.accepted
        if not gh.accepted:
            row.update(message="门面拒绝受理", end=common.now_text())
            print("  ✗ " + row["message"])
            return row
        rfut = gh.get_result_async()
        try:
            self.spin_until(rfut)
        except Interrupted:
            print("  已按 Ctrl-C，取消当前任务…")
            self.stop = False
            cfut = gh.cancel_goal_async()
            self.spin_until(cfut, 3.0)
            self.spin_until(rfut, 5.0)
            self.stop = True
        res = rfut.result().result if rfut.done() and rfut.result() else None
        dt = time.time() - t0
        row.update(end=common.now_text(), elapsed_s=f"{dt:.2f}", states=">".join(states))
        if res is None:
            row["message"] = "未收到结果"
        else:
            row.update(success=res.success, error_code=res.error_code, error_text=common.code_text(res.error_code),
                       message=res.message, task_id=res.task_id)
        mark = "✓" if row["success"] else "✗"
        print(f"  {mark} {dt:.2f}s  success={row['success']}  error_code={row['error_text'] or '—'}"
              f"  {row['message']}  task_id={row['task_id']}")
        return row


def command(node, op, task_id="", speed=0.0, percent=False, timeout=3.0):
    import rclpy
    from nav_interfaces.srv import NavCommand
    cli = node.create_client(NavCommand, COMMAND)
    if not cli.wait_for_service(timeout_sec=timeout):
        print(f"✗ {COMMAND} 不在线")
        return 2
    req = NavCommand.Request(op=op, task_id=task_id, speed_limit=float(speed), percentage=percent)
    t0 = time.time()
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    if not fut.done() or fut.result() is None:
        print(f"✗ {timeout:.0f} s 内无应答")
        return 2
    r = fut.result()
    print(f"{'✓' if r.success else '✗'} {(time.time() - t0) * 1000:.0f} ms  success={r.success}  "
          f"error_code={common.code_text(r.error_code)}  {r.message}")
    print("  （服务只表示受理，结果看 nbt nav state 或 nbt watch --groups nav）")
    return 0 if r.success else 1


def show_state(node, timeout=3.0):
    import rclpy
    cls = common.import_msg("nav_interfaces/msg/NavigationState")
    got = []
    end = time.time() + 1.0
    while time.time() < end and node.count_publishers(STATE_TOPIC) == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.create_subscription(cls, STATE_TOPIC, got.append, common.qos_for(node, STATE_TOPIC, 1))
    end = time.time() + timeout
    while time.time() < end and not got:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not got:
        print(f"✗ {timeout:.0f} s 内没收到 {STATE_TOPIC}")
        return 2
    m = got[-1]
    print(common.md_table(["字段", "值"], [
        ["state", f"{m.state} {STATE.get(m.state, '')}"], ["task_id", m.task_id],
        ["error_code", common.code_text(m.error_code)],
        ["target", f"{common.constant_name(cls, lambda k: k.startswith('TARGET_'), m.target_type)} {m.target_name}"],
        ["current_index / waypoints", f"{m.current_index} / {len(m.waypoints)}"],
        ["distance_remaining", f"{m.distance_remaining:.2f} m"], ["eta_sec", f"{m.eta_sec:.1f}"],
        ["recoveries", m.recoveries], ["speed_limit", f"{getattr(m, 'speed_limit', 0.0):g}"]]))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(prog="nbt nav", description="经导航门面下发任务 / 控制命令")
    sub = ap.add_subparsers(dest="op", required=True)
    for name, hlp in (("goto", "去点位，可给多个点轮流去"), ("route", "走路线，可给多条轮流走")):
        p = sub.add_parser(name, help=hlp)
        p.add_argument("targets", nargs="+")
        p.add_argument("--repeat", type=int, default=1, help="轮数")
        p.add_argument("--gap", type=float, default=2.0, help="两个任务之间停顿秒数")
        p.add_argument("--stop-on-fail", action="store_true", help="某个任务失败就停")
        p.add_argument("--csv", default=default_csv())
    p = sub.add_parser("pose", help="去 map 系位姿：x y yaw(度)")
    p.add_argument("x", type=float)
    p.add_argument("y", type=float)
    p.add_argument("yaw", type=float)
    p.add_argument("--frame", default="map")
    p.add_argument("--csv", default=default_csv())
    for name in ("pause", "resume", "cancel"):
        p = sub.add_parser(name)
        p.add_argument("--task", default="", help="task_id，默认当前任务")
    p = sub.add_parser("speed", help="限速，0 = 解除")
    p.add_argument("value", type=float)
    p.add_argument("--percent", action="store_true", help="按最大速度百分比 0~100")
    sub.add_parser("state", help="打印一次 /navigation/state")
    a = ap.parse_args(argv)

    import rclpy
    rclpy.init()
    node = rclpy.create_node("nbt_nav")
    try:
        if a.op in ("pause", "resume", "cancel"):
            return command(node, {"pause": 1, "resume": 2, "cancel": 3}[a.op], task_id=a.task)
        if a.op == "speed":
            return command(node, 4, speed=a.value, percent=a.percent)
        if a.op == "state":
            return show_state(node)

        nc = NavClient(node)
        signal.signal(signal.SIGINT, lambda *_: setattr(nc, "stop", True))
        if not nc.client.wait_for_server(timeout_sec=5.0):
            print(f"✗ {ACTION} 不在线（导航门面没起？）")
            return 2
        if a.op == "pose":
            jobs = [(2, f"({a.x:g},{a.y:g},{a.yaw:g}°)", (a.x, a.y, a.yaw))]
            repeat, gap, stop_on_fail = 1, 0.0, False
        else:
            tt = 1 if a.op == "goto" else 3
            jobs = [(tt, t, None) for t in a.targets]
            repeat, gap, stop_on_fail = max(1, a.repeat), a.gap, a.stop_on_fail
        rows, total = [], repeat * len(jobs)
        for r in range(repeat):
            for i, (tt, target, pose) in enumerate(jobs):
                if nc.stop:
                    break
                if total > 1:
                    print(f"[{r * len(jobs) + i + 1}/{total}]")
                try:
                    row = nc.run(tt, target, pose, getattr(a, "frame", "map"))
                except Interrupted:
                    print("  已按 Ctrl-C，目标尚未受理，退出")
                    break
                rows.append(row)
                common.write_csv(a.csv, CSV_HEADERS, [[row[h] for h in CSV_HEADERS]], append=True)
                if (stop_on_fail and not row["success"]) or nc.stop:
                    nc.stop = True
                    break
                if gap > 0 and r * len(jobs) + i + 1 < total:
                    end = time.time() + gap
                    while time.time() < end and not nc.stop:
                        rclpy.spin_once(node, timeout_sec=0.1)
            if nc.stop:
                break
        ok = sum(1 for x in rows if x["success"])
        times = [float(x["elapsed_s"]) for x in rows if x["success"] and x["elapsed_s"]]
        if len(rows) > 1:
            print(f"\n共 {len(rows)} 个任务，成功 {ok}，失败 {len(rows) - ok}"
                  + (f"；成功任务耗时 最短 {min(times):.1f} s / 平均 {sum(times) / len(times):.1f} s / 最长 {max(times):.1f} s"
                     if times else ""))
        print(f"已追加到 {a.csv}")
        return 0 if rows and ok == len(rows) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
