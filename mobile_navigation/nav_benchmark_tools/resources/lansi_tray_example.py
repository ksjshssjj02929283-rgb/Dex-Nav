"""蓝思 tray 示例场景：按路网图手绘的点位与路网（像素坐标 1 px = 1 cm），用于模拟教程的默认场景和计划生成的测试。

assets()  —— GetAssets 同结构的示例资产（导航点 + 直连边），喂给 pathplan.make_plan；
scene()   —— 用 make_plan 生成任务序列，再换成按路网图手绘的折线、加上工位框等装饰，得到教程默认场景。
"""
import math

H = 1900.0      # 图片坐标 y 向下，换成地图坐标 y 向上


def m(px, py):
    return round(px / 100.0, 3), round((H - py) / 100.0, 3)


POINTS = {  # 名称: (px, py, 朝向°（地图系，逆时针为正）, 类型)
    "导航点1": (1080, 1630, 0, "FREE_GOAL"), "导航点2": (540, 1365, 90, "FREE_GOAL"),
    "导航点3": (968, 1365, 90, "FREE_GOAL"), "导航点4": (968, 110, 90, "FREE_GOAL"),
    "导航点5": (535, 110, -90, "FREE_GOAL"), "导航点6": (1088, 1050, 0, "FREE_GOAL"),
    "放料点": (538, 1065, 90, "STATION"), "取盘点": (535, 325, -90, "STATION"),
    "放盘点": (1432, 1050, 0, "STATION"), "取料点": (1438, 1630, 0, "STATION"), "充电点1": (222, 1365, -90, "CHARGING"),
}
# (起, 终, 倒车, 按路网图画的折线（像素）)
EDGES = [
    ("导航点1", "导航点2", False, [(1080, 1630), (1080, 1663), (500, 1663), (500, 1365), (540, 1365)]),
    ("导航点2", "放料点", False, [(500, 1340), (500, 1090)]),
    ("放料点", "导航点2", True, [(568, 1090), (568, 1340)]),
    ("导航点2", "导航点3", False, [(565, 1365), (943, 1365)]),
    ("导航点3", "导航点4", False, [(940, 1340), (940, 135)]),
    ("导航点4", "导航点5", False, [(943, 110), (560, 110)]),
    ("导航点5", "取盘点", False, [(596, 135), (596, 300)]),
    ("取盘点", "导航点5", True, [(495, 300), (495, 135)]),
    ("导航点5", "导航点6", False, [(565, 62), (1040, 62), (1040, 1050), (1063, 1050)]),
    ("导航点6", "放盘点", False, [(1113, 1026), (1407, 1026)]),
    ("放盘点", "导航点6", True, [(1407, 1074), (1113, 1074)]),
    ("导航点6", "导航点1", False, [(1115, 1075), (1115, 1605)]),
    ("导航点1", "取料点", False, [(1105, 1604), (1413, 1604)]),
    ("取料点", "导航点1", True, [(1413, 1656), (1105, 1656)]),
]
# 任务里每段的行驶折线（像素），与 EDGES 同序；车从点位中心出发、停在点位中心
DRIVE = {
    ("导航点1", "导航点2"): [(1080, 1630), (540, 1630), (540, 1365)],
    ("导航点2", "放料点"): [(540, 1365), (538, 1065)],
    ("放料点", "导航点2"): [(538, 1065), (540, 1365)],
    ("导航点2", "导航点3"): [(540, 1365), (968, 1365)],
    ("导航点3", "导航点4"): [(968, 1365), (968, 110)],
    ("导航点4", "导航点5"): [(968, 110), (535, 110)],
    ("导航点5", "取盘点"): [(535, 110), (535, 325)],
    ("取盘点", "导航点5"): [(535, 325), (535, 110)],
    ("导航点5", "导航点6"): [(535, 110), (535, 60), (1040, 60), (1040, 1050), (1088, 1050)],
    ("导航点6", "导航点1"): [(1088, 1050), (1115, 1050), (1115, 1630), (1080, 1630)],
}
DECOR = [  # 路网图上的工位框、磨料机、人员工作区域（地图资产里没有，只画在示例里）
    ("rect", (420, 405, 240, 220), "取盘位"), ("rect", (420, 815, 240, 220), "放料位"),
    ("rect", (1485, 958, 240, 218), "放盘位"), ("rect", (1485, 1520, 240, 220), "取料位(满料)"),
    ("rect", (105, 1553, 240, 218), "充电桩"), ("bar", (200, 1785, 1565, 85), "磨料机"),
    ("zone", (225, 0, 1565, 350), "人员工作区域（清场）"),
]


def assets():
    pts = [{"name": n, "x": m(x, y)[0], "y": m(x, y)[1], "theta": math.radians(h), "type": t}
           for n, (x, y, h, t) in POINTS.items()]
    edges = [{"id": f"e{i + 1}", "from_ep": {"id": a, "type": "nav"}, "to_ep": {"id": b, "type": "nav"},
              "edge_type": "LINE", "bidirectional": False, "backward": back, "control_points": []}
             for i, (a, b, back, _) in enumerate(EDGES)]
    return {"map_name": "lansi_tray_example", "revision": 1, "points": pts, "edges": edges, "junctions": [],
            "areas": []}


def _poly(px):
    return [list(m(x, y)) for x, y in px]


def scene(pathplan):
    norm = pathplan.normalize(assets())
    plan = pathplan.make_plan(norm, "导航点1", ["导航点2", "放料点", "导航点5", "取盘点"], ["导航点3", "导航点4"])
    for t in plan["tasks"]:   # 长度改用按路网图走的折线
        for lg in t["legs"]:
            lg["len"] = pathplan.plen(_poly(DRIVE[(lg["from"], lg["to"])]))
        t["len"] = sum(lg["len"] for lg in t["legs"])
    plan["segment"]["len"] = pathplan.plen(_poly(DRIVE[("导航点3", "导航点4")]))
    sc = pathplan.scene(norm, plan, "E075", "蓝思 tray 示例场景（按路网图手绘，坐标为示意）")
    sc["source"] = "示例：按《基于蓝思 Tray盘搬运场景的功能需求》路网图手绘，不是现场地图；现场用 nbt pathnav plan --tutorial 生成"
    sc["edges"] = [{"poly": _poly(p), "bi": False, "nav": True, "back": back} for _, _, back, p in EDGES]
    sc["edges"].append({"poly": _poly([(393, 1248), (252, 1302)]), "bi": False, "nav": False, "free": True})
    for st, t in zip(sc["tasks"], plan["tasks"]):
        groups = []
        for lg in t["legs"]:
            poly = _poly(DRIVE[(lg["from"], lg["to"])])
            if groups and groups[-1]["rev"] == lg["backward"]:
                groups[-1]["poly"] += poly[1:]
            else:
                groups.append({"rev": lg["backward"], "poly": poly})
        st["groups"] = groups
    sc["decor"] = []
    for kind, (x, y, w, h), label in DECOR:
        sc["decor"].append({"kind": kind, "label": label,
                            "polygon": [list(m(x, y)), list(m(x + w, y)), list(m(x + w, y + h)), list(m(x, y + h))]})
    return sc
