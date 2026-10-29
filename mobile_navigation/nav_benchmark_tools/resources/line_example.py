"""模拟教程「直线最大速度」课程的示例场景：一条 12.5 m 的长直通道，测试点 A、B 之间一条双向路段。

坐标为示意，不是现场地图；现场用 nbt pathnav plan --start A --key B,A --segment A,B --target-speed 1.0 --tutorial 生成。
长度按直线最大速度方案第 2 节取 1.2 m/s 的建议布场长度（约 12.5 m）；两端各留 1.6 m 停车余量。
"""
import math

POINTS = {"A": (2.5, 4.0, 180.0), "B": (15.0, 4.0, 0.0)}      # 到点朝向沿行驶方向，掉头在下一个任务起步时做
DECOR = [("rect", (1.0, 6.0, 15.5, 1.2), "货架"), ("rect", (1.0, 0.8, 15.5, 1.2), "货架"),
         ("zone", (0.3, 2.6, 1.6, 2.8), "停车余量"), ("zone", (15.6, 2.6, 1.6, 2.8), "停车余量")]


def assets():
    pts = [{"name": n, "x": x, "y": y, "theta": math.radians(h), "type": "FREE_GOAL"}
           for n, (x, y, h) in POINTS.items()]
    edges = [{"id": "e1", "from_ep": {"id": "A", "type": "nav"}, "to_ep": {"id": "B", "type": "nav"},
              "edge_type": "LINE", "bidirectional": True, "backward": False, "control_points": []}]
    return {"map_name": "line_example", "revision": 1, "points": pts, "edges": edges, "junctions": [], "areas": []}


def scene(pathplan):
    norm = pathplan.normalize(assets())
    plan = pathplan.make_plan(norm, "A", ["B", "A"], ["A", "B"], target_speed=1.0)
    sc = pathplan.scene(norm, plan, "E100", "长直通道示例场景（坐标为示意）")
    sc["source"] = ("示例：12.5 m 长直通道，不是现场地图；现场场地定下来后用 "
                    "nbt pathnav plan --start A --key B,A --segment A,B --target-speed 1.0 --tutorial 生成")
    sc["decor"] = [{"kind": kind, "label": label, "polygon": [[x, y], [x + w, y], [x + w, y + h], [x, y + h]]}
                   for kind, (x, y, w, h), label in DECOR]
    return sc
