"""在线链路集成测试：起假机器人子进程，跑 speed profile set → pathnav run --record → speed analyze → profile restore。

假机器人见 fake_robot.py：每个任务停在目标点沿行进方向 +2 mm 处，且先上报成功、0.3 s 后才停稳。
用随机 ROS_DOMAIN_ID 隔离，不影响本机其他 ROS 进程。
"""
import csv
import json
import math
import os
import random
import subprocess
import sys
import time

import pytest

from nav_benchmark_tools import pathnav_cmd, speed_analyze, speed_profile

HERE = os.path.dirname(os.path.abspath(__file__))


def start_fake(**env):
    os.environ["ROS_DOMAIN_ID"] = str(random.randint(100, 200))
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    p = subprocess.Popen([sys.executable, os.path.join(HERE, "fake_robot.py")], env=dict(os.environ, **env),
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(3.0)
    assert p.poll() is None, "假机器人没起来"
    return p


@pytest.fixture()
def fake_robot():
    p = start_fake()
    yield p
    p.terminate()
    p.wait(timeout=10)


def test_mode_gate_refuses_controller_mode(tmp_path):
    p = start_fake(FAKE_SP_MODE="controller")
    try:
        assert pathnav_cmd.main(["mode"]) == 1
        plan = tmp_path / "plan.yaml"
        assert pathnav_cmd.main(["plan", "--start", "A", "--key", "B,C", "-o", str(plan)]) == 0
        assert pathnav_cmd.main(["run", str(plan), "-o", str(tmp_path / "E075"), "--no-prompt", "--speed", "0.7"]) == 2
        assert not (tmp_path / "E075" / "sessions").exists()
    finally:
        p.terminate()
        p.wait(timeout=10)


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def test_profile_run_analyze_restore(fake_robot, tmp_path):
    bdir = tmp_path / "E075"
    plan = tmp_path / "plan.yaml"
    assert pathnav_cmd.main(["mode"]) == 0                 # path_navigation + path_follower
    # 计划从假机器人的路网在线生成（含解析校验），再拿它跑
    assert pathnav_cmd.main(["plan", "--start", "A", "--key", "B,C", "-o", str(plan),
                             "--tutorial", str(tmp_path / "t.html")]) == 0
    loaded = pathnav_cmd.load_plan(str(plan))
    assert [t["points"] for t in loaded["tasks"]] == [["A", "B"], ["B", "C"], ["C", "A"]]
    assert loaded["segment"] == ["C", "A"] and (tmp_path / "plan.svg").exists() and (tmp_path / "t.html").exists()

    # 计划档位 0.75 与跟踪器当前 0.7 不一致：run 必须拒绝
    assert pathnav_cmd.main(["run", str(plan), "-o", str(bdir), "--no-prompt"]) == 2
    # 平滑器前进上限 0.8 < 0.9：不带 --with-smoother 必须拒绝
    assert speed_profile.main(["set", "0.9", "--batch", "E075", "-o", str(bdir)]) == 2
    assert speed_profile.main(["set", "0.75", "--batch", "E075", "-o", str(bdir)]) == 0
    base = json.load(open(bdir / "profile" / "original.json", encoding="utf-8"))
    assert base["params"]["/path_follower_node"]["max_linear_velocity"] == 0.7

    rc = pathnav_cmd.main(["run", str(plan), "-o", str(bdir), "--laps", "2", "--no-prompt", "--record"])
    assert rc == 0
    sessions = sorted((bdir / "sessions").iterdir())
    assert len(sessions) == 1                          # 被拒绝的那次不留会话目录
    sdir = sessions[0]
    rows = read_csv(sdir / "tasks.csv")
    assert [r["run_id"][-3:] for r in rows] == ["001", "002", "003", "004", "005", "006"]
    assert all(r["result"] == "succeeded" and r["t_settle"] for r in rows)
    assert all(float(r["report_to_settle_s"]) > 0.25 for r in rows)          # 先上报、后停稳
    by = {(r["lap"], r["task"]): r for r in rows}
    for lap in ("1", "2"):
        for task in ("T1", "T2"):
            r = by[(lap, task)]
            assert abs(float(r["err_lon_mm"]) - 2.0) < 0.3 and abs(float(r["err_lat_mm"])) < 0.3
            assert r["key_point"] == "y"
        r = by[(lap, "T3")]                            # 斜着回 A，目标朝向 0°：纵 / 横各 -√2 mm
        assert abs(float(r["err_lon_mm"]) + math.sqrt(2)) < 0.3 and abs(float(r["err_lat_mm"]) + math.sqrt(2)) < 0.3
        assert r["key_point"] == "n"
    assert abs(float(by[("1", "T1")]["route_len_m"]) - 1.0) < 1e-6
    run = json.load(open(sdir / "run.json", encoding="utf-8"))
    assert run["params"]["/path_follower_node"]["max_linear_velocity"] == 0.75
    assert run["nav_mode"]["ok"] and run["nav_mode"]["backend"] == "path_navigation"
    assert (sdir / "rosbag" / "metadata.yaml").exists()

    assert speed_analyze.main([str(bdir)]) == 0
    stops = read_csv(bdir / "analysis" / "stops.csv")
    assert len(stops) == 4 and {s["point"] for s in stops} == {"B", "C"}
    for s in stops:
        assert s["source"] == "录包"
        assert abs(float(s["lon_mm"]) - 2.0) < 0.5 and abs(float(s["lat_mm"])) < 0.5
    pts = {p["point"]: p for p in read_csv(bdir / "analysis" / "points.csv")}
    assert pts["B"]["n"] == "2" and pts["C"]["n"] == "2"
    tm = read_csv(bdir / "analysis" / "tasks_metrics.csv")
    assert len(tm) == 6 and all(abs(float(t["max_v"]) - 1.0) < 0.01 for t in tm)

    assert speed_profile.main(["restore", "--batch", "E075", "-o", str(bdir)]) == 0
    restored = sorted((bdir / "profile").glob("*_restore.json"))
    assert json.load(open(restored[-1], encoding="utf-8"))["params"]["/path_follower_node"][
        "max_linear_velocity"] == 0.7
