"""计划生成（nbt pathnav plan 的核心）与模拟教程模板的单元测试，不需要 ROS 节点。"""
import json
import math
import os
import sys

import pytest

from nav_benchmark_tools import pathnav_cmd
from nav_benchmark_tools import pathplan as pp

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "resources"))
import lansi_tray_example as ex  # noqa: E402
import line_example as lx  # noqa: E402

KEYS = ["导航点2", "放料点", "导航点5", "取盘点"]


def tiny(edges, points=None, junctions=()):
    pts = points or {"A": (0, 0), "B": (4, 0), "C": (4, 3)}
    return pp.normalize({"map_name": "t", "revision": 1,
                         "points": [{"name": n, "x": x, "y": y, "theta": 0.0} for n, (x, y) in pts.items()],
                         "junctions": [{"id": j, "x": x, "y": y} for j, x, y in junctions],
                         "edges": [dict({"id": f"e{i}", "edge_type": "LINE", "bidirectional": False, "backward": False,
                                         "control_points": []}, **e) for i, e in enumerate(edges)]})


def ep(i, t="nav"):
    return {"id": i, "type": t}


def test_example_reproduces_hand_written_tasks():
    plan = pp.make_plan(pp.normalize(ex.assets()), "导航点1", KEYS)
    assert [t["points"] for t in plan["tasks"]] == [
        ["导航点1", "导航点2"], ["导航点2", "放料点"], ["放料点", "导航点2", "导航点3", "导航点4", "导航点5"],
        ["导航点5", "取盘点"], ["取盘点", "导航点5", "导航点6", "导航点1"]]
    assert [t["back"] for t in plan["tasks"]] == [0, 0, 1, 0, 1]
    seg = plan["segment"]
    assert (seg["from"], seg["to"], seg["auto"]) == ("导航点3", "导航点4", True)
    assert abs(seg["need"] - 4.40) < 0.01 and plan["warnings"] == []


def test_ramp_distance_matches_jerk_limited_numbers():
    assert abs(pp.ramp_distance(0.75, 0.5, 0.2) - 1.452) < 1e-3     # 推荐速度方案附录 B：约 1.46 m
    assert abs(pp.ramp_distance(1.0, 0.5, 0.2) - 2.236) < 1e-3      # 直线方案附录 B：2.24 m
    assert abs(pp.ramp_distance(1.0, 0.4, 0.2) - 2.25) < 1e-3       # 2.25 m（减速度到上限）
    assert abs(pp.steady_need(1.2) - 8.34) < 0.01


def test_unreachable_and_missing_names():
    norm = pp.normalize(ex.assets())
    with pytest.raises(pp.PlanError, match="是不是"):
        pp.make_plan(norm, "导航点 1", KEYS)
    a = ex.assets()
    a["edges"] = [e for e in a["edges"] if not (e["from_ep"]["id"] == "导航点6" and e["to_ep"]["id"] == "导航点1")]
    with pytest.raises(pp.PlanError, match="没有从「取盘点」到「导航点1」"):
        pp.make_plan(pp.normalize(a), "导航点1", KEYS)


def test_bidirectional_reverse_keeps_backward_and_junction_edges_excluded():
    norm = tiny([{"from_ep": ep("A"), "to_ep": ep("B"), "bidirectional": True, "backward": True}])
    plan = pp.make_plan(norm, "B", ["A"], close=False)
    leg = plan["tasks"][0]["legs"][0]
    assert plan["tasks"][0]["points"] == ["B", "A"] and leg["reversed"] and leg["backward"]
    assert pp.task_groups(norm, plan["tasks"][0])[0]["poly"][0] == (4.0, 0.0)      # 反向行驶：折线从 B 出发
    norm = tiny([{"from_ep": ep("A"), "to_ep": ep("J", "junction")},
                 {"from_ep": ep("J", "junction"), "to_ep": ep("B")}], junctions=[("J", 2, 0)])
    with pytest.raises(pp.PlanError, match="交叉点"):
        pp.make_plan(norm, "A", ["B"], close=False)


def test_bezier_length_and_segment_choice():
    norm = tiny([{"from_ep": ep("A"), "to_ep": ep("B")},
                 {"from_ep": ep("B"), "to_ep": ep("C"), "edge_type": "BEZIER",
                  "control_points": [{"x": 4, "y": 0}, {"x": 8, "y": 1.5}, {"x": 4, "y": 3}]},
                 {"from_ep": ep("C"), "to_ep": ep("A")}])
    assert norm["edges"][1]["len"] > 3.0
    plan = pp.make_plan(norm, "A", ["B", "C"])
    assert (plan["segment"]["from"], plan["segment"]["to"]) == ("C", "A")       # 直线边里最长（5 m），弯道不选
    assert plan["warnings"] == []
    # A→B、C→A 各自就是一整个任务：起步、停车到底。A→B (4 − 2×1.452) / 0.75 ≈ 1.46 s；C→A (5 − 2×1.452) / 0.75 ≈ 2.79 s
    cands = {(c["from"], c["to"]): c for c in plan["segment"]["candidates"]}
    assert abs(cands[("A", "B")]["steady"] - 1.46) < 0.02 and abs(cands[("C", "A")]["steady"] - 2.79) < 0.02
    assert (cands[("A", "B")]["v_in"], cands[("A", "B")]["v_out"]) == (0.0, 0.0)
    plan = pp.make_plan(norm, "A", ["B", "C"], segment=["A", "B"])
    assert plan["segment"]["len"] == 4.0 and any("预计只能" in w for w in plan["warnings"])


def test_straight_runs_merge_collinear_and_stop_on_sharp_turn():
    pts = {"A": (0, 0), "B": (3, 0), "C": (8, 0), "D": (8, 4), "E": (2, 0.2)}
    norm = tiny([{"from_ep": ep("A"), "to_ep": ep("B")}, {"from_ep": ep("B"), "to_ep": ep("C")},
                 {"from_ep": ep("C"), "to_ep": ep("D")}, {"from_ep": ep("D"), "to_ep": ep("E")}], points=pts)
    plan = pp.make_plan(norm, "A", ["D", "E"], close=False)
    top = plan["segment"]
    assert (top["from"], top["to"], top["via"]) == ("A", "C", ["A", "B", "C"]) and top["len"] == 8.0   # 共线两段合并
    assert top["v_out"] == pp.V_CORNER                                   # C 处转 90°（≤ 120°）按拐角通过
    de = [c for c in top["candidates"] if (c["from"], c["to"]) == ("D", "E")][0]
    assert de["v_in"] == 0.0                                             # D 处转角约 136° > 120°：停车原地转


def test_yaml_round_trip_svg_and_tutorial(tmp_path):
    norm = pp.normalize(ex.assets())
    plan = pp.make_plan(norm, "导航点1", KEYS)
    f = tmp_path / "plan.yaml"
    f.write_text(pp.plan_yaml(plan, "F075"), encoding="utf-8")
    loaded = pathnav_cmd.load_plan(str(f))
    assert loaded["batch"] == "F075" and loaded["key_points"] == KEYS and loaded["segment"] == ["导航点3", "导航点4"]
    assert [t["points"] for t in loaded["tasks"]] == [t["points"] for t in plan["tasks"]]
    svg = pp.preview_svg(norm, plan)
    assert svg.startswith("<svg") and "导航点5" in svg and "T5" in svg and "关键路段" in svg
    html = pp.render_tutorial(pp.scene(norm, plan, "F075"))
    body = html.split('<script id="scene" type="application/json">', 1)[1].split("</script>", 1)[0]
    sc = json.loads(body)
    assert sc["batch"] == "F075" and [t["id"] for t in sc["tasks"]] == ["T1", "T2", "T3", "T4", "T5"]
    assert [g["rev"] for g in sc["tasks"][2]["groups"]] == [True, False]
    assert "const SCENE" in html


def block(html, tag):
    return json.loads(html.split(f'<script id="{tag}" type="application/json">', 1)[1].split("</script>", 1)[0])


def test_template_embeds_example_scene():
    with open(pp.template_path(), encoding="utf-8") as fh:
        html = fh.read()
    sc = block(html, "scene")
    assert sc["start"] == "导航点1" and sc["keys"] == KEYS and len(sc["tasks"]) == 5
    assert abs(sc["segment"]["len"] - 12.55) < 1e-6 and any(d["kind"] == "zone" for d in sc["decor"])
    assert math.isclose(sc["tasks"][0]["len"], 8.05, abs_tol=1e-6)
    mx = block(html, "scene_max")                      # 直线最大速度课程的示例
    assert mx["course"] == "max" and mx["keys"] == ["B", "A"] and mx["batch"] == "E100"
    assert [t["points"] for t in mx["tasks"]] == [["A", "B"], ["B", "A"]]
    assert '<meta name="nbt-course" content="recommend">' in html     # 默认打开推荐速度课程


def test_line_example_and_course_selection():
    norm = pp.normalize(lx.assets())
    plan = pp.make_plan(norm, "A", ["B", "A"], ["A", "B"], target_speed=1.0)
    assert [t["points"] for t in plan["tasks"]] == [["A", "B"], ["B", "A"]]          # 起点即最后一个关键点位，不再加回程
    assert abs(plan["segment"]["steady"] - 8.01) < 0.01 and plan["warnings"] == []     # (12.5 − 2.236 − 2.25) / 1.0
    assert abs(pp.make_plan(norm, "A", ["B", "A"], ["A", "B"], target_speed=1.2)["segment"]["steady"] - 5.47) < 0.01
    assert [c["from"] for c in plan["segment"]["candidates"]] == ["A", "B"]              # 两个方向各一段
    assert (pp.course_of(0.75), pp.course_of(0.8), pp.course_of(1.0), pp.course_of(1.2)) == \
        ("recommend", "recommend", "max", "max")
    sc = pp.scene(norm, plan, "F100")
    html = pp.render_tutorial(sc)
    assert block(html, "scene_max")["batch"] == "F100" and block(html, "scene")["batch"] == "E075"
    assert '<meta name="nbt-course" content="max">' in html                             # 生成的教程默认打开这门课
