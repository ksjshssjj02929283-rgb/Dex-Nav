"""能力摸底速度测试工具的单元测试 + 合成录包端到端测试（navlog convert / speed analyze）。

合成两段路网任务：T1 前进 A(0,0) → B(4,0) → C(10,0)，梯形速度（加速 0.5、匀速 0.75、减速 0.4），
到点前 5 mm 就上报成功、之后才停稳，最终停在 C 前方 +3 mm、横向 -4 mm、朝向 +0.002 rad；
T2 倒车 C → A，停在 A 后方 -2 mm、横向 +1 mm、朝向 -0.001 rad。map → odom 带平移和旋转，检验 TF 复合。
"""
import csv
import json
import math
import os
import uuid

import rosbag2_py
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Pose, TransformStamped, Twist
from nav_interfaces.action import NavigateThroughGraphPoints
from nav_interfaces.msg import NavigationStatus, TrackingPath
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage

from nav_benchmark_tools import navlog as nl
from nav_benchmark_tools import navlog_convert, pathnav_cmd, speed_analyze

T0 = 1_800_000_000.0
DT = 0.01
MAP_ODOM = (1.0, 2.0, 0.3)
TARGETS = {"A": {"x": 0.0, "y": 0.0, "theta": 0.0}, "B": {"x": 4.0, "y": 0.0, "theta": 0.0},
           "C": {"x": 10.0, "y": 0.0, "theta": 0.0}}
VMAX, ACC, DEC = 0.75, 0.5, 0.4


# ---------------------------------------------------------------- 单元测试

def test_target_error_decomposition():
    e = nl.target_error(1.0 + 0.003, 2.0 - 0.004, math.pi / 2 + 0.01, 1.0, 2.0, math.pi / 2)
    assert abs(e["longitudinal"] - (-0.004)) < 1e-9      # 目标朝 +y：世界 -y 是纵向负
    assert abs(e["lateral"] - (-0.003)) < 1e-9
    assert abs(e["yaw"] - 0.01) < 1e-9


def test_mean_pose_wraps_yaw():
    x, y, yaw = nl.mean_pose([(0, 0, 0, math.pi - 0.01), (1, 2, 2, -math.pi + 0.01)])
    assert (x, y) == (1.0, 1.0) and abs(abs(yaw) - math.pi) < 1e-9


def test_find_settle_needs_both_linear_and_angular():
    odom = [(i * 0.02, 0.0, 0.1 if i < 50 else 0.0) for i in range(200)]   # 前 1 s 原地转
    t = nl.find_settle(odom, 0.0, 0.01, 0.02, 0.5, 10.0)
    assert abs(t - 1.5) < 0.03
    assert nl.find_settle(odom, 0.0, 0.01, 0.02, 0.5, 0.2) is None


def test_local_slope_on_quadratic():
    ts = [i * 0.01 for i in range(300)]
    vs = [0.5 * t * t for t in ts]
    sl = nl.local_slope(ts, vs, 0.3)
    assert abs(sl[150] - ts[150]) < 1e-6 and sl[0] is not None


def test_band_time_and_limit_stage():
    series = [(i * 0.1, v, 0.0) for i, v in enumerate([0.0, 0.5, 0.72, 0.75, 0.76, 0.74, 0.6, 0.0])]
    best, total = speed_analyze.band_time(series, 0.75)
    assert abs(best - 0.3) < 1e-9 and abs(total - 0.3) < 1e-9
    peaks = {"follower": 0.75, "smoother": 0.70, "mux": 0.70, "odom": 0.69}
    assert speed_analyze.limit_stage(peaks, 0.75) == "平滑器"
    assert speed_analyze.limit_stage(dict(peaks, smoother=0.75, mux=0.75, odom=0.74), 0.75) == "无（达到目标）"
    assert speed_analyze.limit_stage(dict(peaks, follower=0.5), 0.75) == "跟踪器"


def test_point_stats_excludes_disturbed():
    stops = [{"point": "P", "mark": "合格", "lon_mm": 3.0, "lat_mm": -4.0, "yaw_deg": 0.1},
             {"point": "P", "mark": "合格", "lon_mm": 5.0, "lat_mm": -2.0, "yaw_deg": -0.1},
             {"point": "P", "mark": "受干扰", "lon_mm": 50.0, "lat_mm": 40.0, "yaw_deg": 3.0}]
    p = speed_analyze.point_stats(stops, ["P"])[0]
    assert p["n"] == 3 and p["pass"] == 2 and p["disturbed"] == 1 and p["n_stat"] == 2
    assert p["lon_mm"]["max"] == 5.0 and abs(p["lon_mm"]["mean"] - 4.0) < 1e-9
    assert abs(p["lon_mm"]["std"] - math.sqrt(2.0)) < 1e-9 and p["lat_mm"]["range"] == 2.0


def test_example_plan_parses(tmp_path):
    f = tmp_path / "p.yaml"
    f.write_text(pathnav_cmd.EXAMPLE, encoding="utf-8")
    plan = pathnav_cmd.load_plan(str(f))
    assert [t["id"] for t in plan["tasks"]] == ["T1", "T2", "T3", "T4", "T5"]
    assert plan["key_points"] == ["导航点2", "放料点", "导航点5", "取盘点"] and plan["segment"] == ["导航点3", "导航点4"]


def test_segment_length_line_and_bezier():
    class P:
        def __init__(self, x, y):
            self.x, self.y = x, y

    class S:
        def __init__(self, typ, pts):
            self.type, self.control_points = typ, [P(*p) for p in pts]
    assert abs(pathnav_cmd.segment_length(S(0, [(0, 0), (3, 4)])) - 5.0) < 1e-9
    quarter = pathnav_cmd.segment_length(S(1, [(1, 0), (1, 0.5523), (0.5523, 1), (0, 1)]))
    assert abs(quarter - math.pi / 2) < 2e-3      # 三阶贝塞尔近似四分之一圆


def test_next_seq_continues_across_sessions(tmp_path):
    for sid, ids in (("s1", ["E075-20261105-001", "E075-20261105-002"]), ("s2", ["E075-20261106-007"])):
        d = tmp_path / "sessions" / sid
        d.mkdir(parents=True)
        with open(d / "tasks.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["run_id"])
            w.writerows([[i] for i in ids])
    assert pathnav_cmd.next_seq(str(tmp_path)) == 8


def test_segment_dir_both_ways():
    assert speed_analyze.segment_dir(["放料点", "导航点3", "导航点4", "导航点5"], ["导航点3", "导航点4"]) == \
        ("导航点3", "导航点4")
    assert speed_analyze.segment_dir(["B", "A"], ["A", "B"]) == ("B", "A")
    assert speed_analyze.segment_dir(["A", "C", "B"], ["A", "B"]) == ("A", "B")    # 合并的共线路段可隔途经点
    assert speed_analyze.segment_dir(["A", "C"], ["A", "B"]) is None


def test_mode_verdict_cases():
    pf = {"/schedule_planner_node": "active", "/path_follower_node": "active", "/bt_navigator": "unconfigured",
          "/controller_server": "unconfigured"}
    assert pathnav_cmd.mode_verdict(pf, "independence", "path_navigation")[:2] == (pathnav_cmd.PF_MODE, True)
    mode, ok, notes = pathnav_cmd.mode_verdict(pf, "controller", "path_navigation")
    assert not ok and mode.startswith("path_navigation · controller") and "controller_server 没激活" in notes[0]
    nav2 = {"/schedule_planner_node": "unconfigured", "/path_follower_node": "unconfigured", "/bt_navigator": "active",
            "/controller_server": "active"}
    mode, ok, notes = pathnav_cmd.mode_verdict(nav2, "independence", "nav2")
    assert mode.startswith("nav2 路网") and not ok and notes == []
    mode, ok, notes = pathnav_cmd.mode_verdict(nav2, "independence", "path_navigation")      # 被手动切过
    assert "绕过 command_board" in notes[0]
    assert pathnav_cmd.mode_verdict({}, None, None)[0].startswith("两套都没激活")
    both = dict(pf, **{"/bt_navigator": "active"})
    assert pathnav_cmd.mode_verdict(both, "independence", "nav2")[0].startswith("两套都激活")


def test_ask_mark_only_judgement_and_photo_hint(monkeypatch, capsys):
    answers = iter(["q", "x"])
    monkeypatch.setattr("builtins.input", lambda _="": next(answers))
    assert pathnav_cmd.ask_mark("P", "fb", photo=True) == "受干扰"
    out = capsys.readouterr().out
    assert "拍一张俯视照片留存" in out and "只能输入" in out
    answers = iter([""])
    monkeypatch.setattr("builtins.input", lambda _="": next(answers))
    assert pathnav_cmd.ask_mark("P", "fb") == ""
    assert "照片" not in capsys.readouterr().out
    assert "photo" not in pathnav_cmd.CSV_HEADERS and "mark_lon_mm" not in pathnav_cmd.CSV_HEADERS


# ---------------------------------------------------------------- 合成录包

def trapezoid(L):
    """解析梯形速度：加速 ACC 到 VMAX、匀速、减速 DEC 正好停在 L；按 DT 采样，返回 [(t, s, v, a)]。"""
    ta, tc_d = VMAX / ACC, VMAX / DEC
    da, dd = VMAX * ta / 2, VMAX * tc_d / 2
    tc = (L - da - dd) / VMAX
    total = ta + tc + tc_d

    def at(t):
        if t <= ta:
            return ACC * t * t / 2, ACC * t, ACC
        if t <= ta + tc:
            return da + VMAX * (t - ta), VMAX, 0.0
        u = min(t - ta - tc, tc_d)
        return da + VMAX * tc + VMAX * u - DEC * u * u / 2, VMAX - DEC * u, -DEC
    n = int(math.ceil(total / DT))
    out = [(k * DT,) + at(k * DT) for k in range(1, n)]
    out.append((total, L, 0.0, -DEC))
    return out


def map_to_odom(x, y, yaw):
    tx, ty, tw = MAP_ODOM
    dx, dy = x - tx, y - ty
    return (math.cos(-tw) * dx - math.sin(-tw) * dy, math.sin(-tw) * dx + math.cos(-tw) * dy, yaw - tw)


def stamp(h, t):
    h.stamp.sec = int(T0 + t)
    h.stamp.nanosec = int(round(((T0 + t) % 1) * 1e9)) % 1_000_000_000


def tf_msg(parent, child, t, x, y, yaw):
    m = TransformStamped()
    stamp(m.header, t)
    m.header.frame_id, m.child_frame_id = parent, child
    m.transform.translation.x, m.transform.translation.y = x, y
    m.transform.rotation.z, m.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return TFMessage(transforms=[m])


def build_timeline():
    """返回 (采样 [(t, x, y, yaw, v, a)], 任务 [dict])。"""
    samples, tasks = [], []
    t = 0.0
    x, y, yaw = 0.0, 0.0, 0.0

    def idle(sec):
        nonlocal t
        for _ in range(int(round(sec / DT))):
            t += DT
            samples.append((t, x, y, yaw, 0.0, 0.0))

    def drive(direction, L, end_lat, end_yaw, goal_name, points):
        nonlocal t, x, y, yaw
        x0, y0, yaw0 = x, y, yaw
        prof = trapezoid(L)
        task = {"uuid": uuid.uuid4().bytes, "t_accept": t, "points": points, "goal": goal_name}
        reported = False
        for tt, s, v, a in prof:
            k = s / L
            x = x0 + direction * s
            y = y0 + (end_lat - y0) * k
            yaw = yaw0 + (end_yaw - yaw0) * k
            samples.append((t + tt, x, y, yaw, direction * v, direction * a))
            if not reported and L - s < 0.005:
                task["t_report"], reported = t + tt, True
        t += prof[-1][0]
        task["t_stop"] = t
        task["L"] = L
        tasks.append(task)

    idle(2.0)
    drive(+1, 10.003, -0.004, 0.002, "C", ["A", "B", "C"])
    idle(3.0)
    drive(-1, 10.005, 0.001, -0.001, "A", ["C", "B", "A"])
    idle(3.0)
    return samples, tasks


def write_synthetic_bag(path, samples, tasks):
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    fb_type = "nav_interfaces/action/NavigateThroughGraphPoints_FeedbackMessage"
    for n, ty in (("/feedback/odom", "nav_msgs/msg/Odometry"), ("/tf", "tf2_msgs/msg/TFMessage"),
                  ("/cmd_vel_nav_path", "geometry_msgs/msg/Twist"), ("/control/cmd_vel_nav", "geometry_msgs/msg/Twist"),
                  ("/control/cmd_vel", "geometry_msgs/msg/Twist"), ("/imu/chassis", "sensor_msgs/msg/Imu"),
                  (nl.STATUS_TOPIC, "action_msgs/msg/GoalStatusArray"), (nl.FEEDBACK_TOPIC, fb_type),
                  ("/navigation_status", "nav_interfaces/msg/NavigationStatus"),
                  ("/tracking_path", "nav_interfaces/msg/TrackingPath")):
        w.create_topic(rosbag2_py.TopicMetadata(name=n, type=ty, serialization_format="cdr"))

    def put(n, m, t, w=w):
        w.write(n, serialize_message(m), int(round((T0 + t) * 1e9)))

    events = []
    known = []
    for k, tk in enumerate(tasks):
        events.append((tk["t_accept"], "accept", k))
        events.append((tk["t_report"], "report", k))
    events.sort()
    ei = 0
    for i, (t, x, y, yaw, v, a) in enumerate(samples):
        while ei < len(events) and events[ei][0] <= t + 1e-9:
            te, kind, k = events[ei]
            tk = tasks[k]
            if kind == "accept":
                known.append([tk, GoalStatus.STATUS_EXECUTING])
                tp = TrackingPath()
                stamp(tp.header, te)
                sx, sy = (0.0, 0.0) if tk["goal"] == "C" else (10.0, 0.0)
                gx = TARGETS[tk["goal"]]["x"]
                for j in range(201):
                    p = Pose()
                    p.position.x, p.position.y = sx + (gx - sx) * j / 200, sy
                    p.orientation.w = 1.0
                    tp.poses.append(p)
                tp.target_yaw, tp.backward, tp.max_speed = 0.0, 0 if tk["goal"] == "C" else 1, 0.0
                put("/tracking_path", tp, te)
                put("/navigation_status", NavigationStatus(status=1), te)
            else:
                for item in known:
                    if item[0] is tk:
                        item[1] = GoalStatus.STATUS_SUCCEEDED
                put("/navigation_status", NavigationStatus(status=3), te)
            arr = GoalStatusArray()
            for tko, st in known:
                gs = GoalStatus(status=st)
                gs.goal_info.goal_id.uuid = list(tko["uuid"])
                stamp(gs.goal_info, tko["t_accept"])
                arr.status_list.append(gs)
            put(nl.STATUS_TOPIC, arr, te)
            ei += 1
        if i % 2 == 0:       # 50 Hz
            o = Odometry()
            stamp(o.header, t)
            o.twist.twist.linear.x = v
            put("/feedback/odom", o, t)
            ox, oy, ow = map_to_odom(x, y, yaw)
            put("/tf", tf_msg("odom", "base_link", t, ox, oy, ow), t)
        if i % 5 == 0:       # 20 Hz
            put("/tf", tf_msg("map", "odom", t, *MAP_ODOM), t)
            tw = Twist()
            tw.linear.x = v
            for n in ("/cmd_vel_nav_path", "/control/cmd_vel_nav", "/control/cmd_vel"):
                put(n, tw, t)
            for tk in tasks:
                if tk["t_accept"] <= t <= tk["t_report"]:
                    fb = NavigateThroughGraphPoints.Impl.FeedbackMessage()
                    fb.goal_id.uuid = list(tk["uuid"])
                    fb.feedback.current_status, fb.feedback.total_points = "tracking", len(tk["points"])
                    put(nl.FEEDBACK_TOPIC, fb, t)
        imu = Imu()
        stamp(imu.header, t)
        imu.linear_acceleration.x = a
        put("/imu/chassis", imu, t)
    del w


def make_session(tmp_path):
    samples, tasks = build_timeline()
    bdir = tmp_path / "E075"
    sdir = bdir / "sessions" / "E075-test"
    sdir.mkdir(parents=True)
    write_synthetic_bag(str(sdir / "rosbag"), samples, tasks)
    with open(sdir / "targets.json", "w", encoding="utf-8") as f:
        json.dump({"points": TARGETS}, f)
    (sdir / "plan.yaml").write_text("target_speed: 0.75\nkey_points: [C, A]\nsegment: [A, B]\n"
                                    "tasks:\n  - {id: T1, points: [A, B, C]}\n  - {id: T2, points: [C, B, A]}\n",
                                    encoding="utf-8")
    rows = []
    for k, tk in enumerate(tasks):
        r = {h: "" for h in pathnav_cmd.CSV_HEADERS}
        r.update(run_id=f"E075-20261105-{k + 1:03d}", batch="E075", session="E075-test", lap=1,
                 task=f"T{k + 1}", points=">".join(tk["points"]), goal_id=tk["uuid"].hex(),
                 t_accept=f"{T0 + tk['t_accept']:.3f}", result="succeeded", route_len_m="10.000", mark="合格")
        rows.append([r[h] for h in pathnav_cmd.CSV_HEADERS])
    with open(sdir / "tasks.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(pathnav_cmd.CSV_HEADERS)
        w.writerows(rows)
    return bdir, sdir, tasks


def test_convert_and_analyze_synthetic(tmp_path):
    bdir, sdir, tasks = make_session(tmp_path)

    assert navlog_convert.main([str(sdir)]) == 0
    files = sorted(os.listdir(sdir / "nav_tasks"))
    assert len(files) == 2 and all(f.startswith("nav_task_") and f.endswith("_succeeded.json") for f in files)
    j = json.load(open(sdir / "nav_tasks" / files[0], encoding="utf-8"))
    e = j["goal_reached_error"]
    assert abs(e["longitudinal"] - 0.003) < 2e-4 and abs(e["lateral"] - (-0.004)) < 2e-4
    assert abs(e["yaw"] - 0.002) < 2e-4
    assert j["goal_source"] == "assets" and j["schedule_planner"]["run_id"] == "E075-20261105-001"
    assert j["settle"]["status"] == "ok" and 0.4 < j["settle"]["delay"] < 1.0      # 上报后 0.16 s 停车 + 0.5 s 保持
    assert abs(j["report_reached_error"]["longitudinal"]) < 0.01
    assert abs(j["odom_vel"]["max_linear_speed"] - VMAX) < 1e-6 and abs(j["passing_length"] - 10.003) < 0.01
    assert j["planned_path_length_first"] > 9.9 and j["passing_path"][0][0] >= 0.0

    assert speed_analyze.main([str(bdir)]) == 0
    out = bdir / "analysis"
    stops = list(csv.DictReader(open(out / "stops.csv", encoding="utf-8")))
    assert [s["point"] for s in stops] == ["C", "A"]
    assert abs(float(stops[0]["lon_mm"]) - 3.0) < 0.2 and abs(float(stops[0]["lat_mm"]) + 4.0) < 0.2
    assert abs(float(stops[1]["lon_mm"]) + 2.0) < 0.2 and abs(float(stops[1]["lat_mm"]) - 1.0) < 0.2
    tm = list(csv.DictReader(open(out / "tasks_metrics.csv", encoding="utf-8")))
    assert abs(float(tm[0]["max_v"]) - VMAX) < 1e-3 and abs(float(tm[1]["max_v"]) - VMAX) < 1e-3   # 倒车按大小
    assert abs(float(tm[0]["acc"]) - ACC) < 0.02 and abs(float(tm[0]["dec"]) - DEC) < 0.02
    assert abs(float(tm[1]["acc"]) - ACC) < 0.02 and abs(float(tm[1]["dec"]) - DEC) < 0.02
    assert abs(float(tm[0]["imu_acc"]) - ACC) < 0.02 and abs(float(tm[0]["imu_dec"]) - DEC) < 0.02
    assert tm[0]["limit_stage"] == "无（达到目标）"
    settle_s = tasks[0]["t_stop"] - tasks[0]["t_accept"]
    assert abs(float(tm[0]["settle_s"]) - settle_s) < 0.7
    assert abs(float(tm[0]["avg_v"]) - 10.0 / float(tm[0]["settle_s"])) < 1e-3
    seg = list(csv.DictReader(open(out / "segment.csv", encoding="utf-8")))
    assert [x["direction"] for x in seg] == ["A→B", "B→A"] and all(x["found"] == "True" for x in seg)
    assert abs(float(seg[0]["max_v"]) - VMAX) < 1e-3 and abs(float(seg[1]["max_v"]) - VMAX) < 1e-3
    # A→B：速度进入 0.7125 的时刻约 1.425 s，到 s ≥ 3.8 m 约 5.82 s：带内最长连续 ≈ 4.39 s
    assert abs(float(seg[0]["steady_s"]) - 4.39) < 0.06
    # B→A（倒车）：x ≈ 4.19 进入时段已在带内，减速到 0.7125 时 x ≈ 0.63：(4.19 − 0.63) / 0.75 ≈ 4.74 s
    assert abs(float(seg[1]["steady_s"]) - 4.74) < 0.06
    summary = (out / "summary.md").read_text(encoding="utf-8")
    assert "表 1" in summary and "表 2" in summary and "关键路段" in summary and "各路段实测速度" in summary
    legs = {(g["from"], g["to"]): g for g in csv.DictReader(open(out / "legs.csv", encoding="utf-8"))}
    assert set(legs) == {("A", "B"), ("B", "C"), ("C", "B"), ("B", "A")}
    assert all(abs(float(g["max_v_mean"]) - VMAX) < 2e-3 and g["reach_laps"] == "1" for g in legs.values())
    assert float(legs[("B", "C")]["steady_mean_s"]) > float(legs[("A", "B")]["steady_mean_s"])   # B→C 不含起步段
    html = (out / "speed_curves.html").read_text(encoding="utf-8")
    data = json.loads(html.split('<script id="data" type="application/json">', 1)[1].split("</script>", 1)[0])
    assert [t["id"] for t in data["tasks"]] == ["T1", "T2"] and data["target"] == 0.75
    run = data["tasks"][0]["runs"][0]
    assert run["run_id"] == "E075-20261105-001" and abs(max(v for v in run["v"]["odom"] if v) - VMAX) < 1e-3
    assert abs(max(x for x in run["s"] if x is not None) - 10.003) < 0.05
    assert data["tasks"][0]["pts"][0]["name"] == "B" and abs(data["tasks"][0]["pts"][0]["s"] - 4.0) < 0.1
    assert [(g["from"], g["to"], len(g["runs"])) for g in data["segments"]] == [("A", "B", 1), ("B", "A", 1)]
    assert len(os.listdir(out / "nav_tasks")) == 2
