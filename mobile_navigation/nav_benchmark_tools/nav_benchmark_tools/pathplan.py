"""从现场地图的路网生成能力摸底的任务计划（nbt pathnav plan 的核心，纯函数，不依赖 ROS）。

约束：调度规划器解析点序列时，相邻两个导航点之间必须有一条直接相连的路网边，它自己不做路径搜索；
所以计划里的点序列要逐点写全。这里在「导航点 ↔ 导航点」的边上做最短路：单向边只能顺着走，
双向边两个方向都能走（反向行驶时 backward 属性不变，与解析服务一致）；经过交叉点的边不能用于路网任务。

生成规则：起点 → 关键点 1 → … → 关键点 n →（默认）回起点，逐段最短路；每到一个关键点切出一个任务。
关键路段默认取整圈里最长的直线前进边。
"""
import difflib
import heapq
import json
import math
import os
import re
import time

KINDS_FOR_PLAN = 1 | 2 | 16 | 32          # 点位、区域、交叉点、路网边
AREA_COLOR = {1: "#495057", 2: "#c92a2a", 3: "#f08c00", 4: "#1971c2", 5: "#e8590c", 6: "#2f9e44", 7: "#2f9e44",
              8: "#7048e8", 9: "#868e96"}
AREA_NAME = {1: "虚拟墙", 2: "禁行区", 3: "限速区", 4: "绕行区", 5: "停障区", 6: "停车区", 7: "充电区", 8: "电梯区",
             9: "颠簸区"}


class PlanError(Exception):
    pass


# ---------------------------------------------------------------- 资产归一化

def _xy(p):
    return (float(p["x"]), float(p["y"])) if isinstance(p, dict) else (float(p[0]), float(p[1]))


def bezier(cps, segs=40):
    n = len(cps) - 1
    out = []
    for s in range(segs + 1):
        t = s / segs
        x = y = 0.0
        for i, (px, py) in enumerate(cps):
            b = math.comb(n, i) * (1 - t) ** (n - i) * t ** i
            x, y = x + b * px, y + b * py
        out.append((x, y))
    return out


def plen(poly):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(poly, poly[1:]))


def normalize(assets):
    """GetAssets 应答（message_to_ordereddict 后的 dict，或 nbt assets --json 的文件内容）→ 统一结构。"""
    points = {p["name"]: {"x": float(p["x"]), "y": float(p["y"]), "theta": float(p.get("theta", 0.0)),
                          "type": p.get("type", "")} for p in assets.get("points", [])}
    juncs = {j["id"]: _xy(j) for j in assets.get("junctions", [])}

    def coord(ep):
        if ep.get("type", "nav") == "junction":
            return juncs.get(ep["id"])
        p = points.get(ep["id"])
        return (p["x"], p["y"]) if p else None
    edges = []
    for e in assets.get("edges", []):
        fa, ta = e.get("from_ep", {}), e.get("to_ep", {})
        cps = [_xy(c) for c in e.get("control_points", [])]
        is_bez = e.get("edge_type", "LINE") == "BEZIER" and len(cps) >= 2
        a, b = coord(fa), coord(ta)
        if is_bez:
            poly = bezier(cps)
        elif a and b:
            poly = [a, b]
        else:
            continue
        edges.append({"id": e.get("id", ""), "from": fa.get("id", ""), "to": ta.get("id", ""),
                      "nav": fa.get("type", "nav") == "nav" and ta.get("type", "nav") == "nav",
                      "bidirectional": bool(e.get("bidirectional", False)), "backward": bool(e.get("backward", False)),
                      "line": not is_bez, "poly": poly, "len": plen(poly)})
    areas = [{"name": a.get("name", ""), "type": int(a.get("type", 0)),
              "polygon": [_xy(p) for p in a.get("polygon", [])]}
             for a in assets.get("areas", []) if len(a.get("polygon", [])) >= 3]
    return {"map_name": assets.get("map_name", ""), "revision": assets.get("revision", ""), "points": points,
            "edges": edges, "areas": areas}


def load_assets_file(path):
    with open(path, encoding="utf-8") as f:
        return normalize(json.load(f))


# ---------------------------------------------------------------- 图与最短路

def graph(norm):
    """{点: [(下一个点, 边序号, 是否反向行驶, 长度)]}，只含导航点直连边。"""
    adj = {n: [] for n in norm["points"]}
    for i, e in enumerate(norm["edges"]):
        if not e["nav"] or e["from"] not in adj or e["to"] not in adj:
            continue
        adj[e["from"]].append((e["to"], i, False, e["len"]))
        if e["bidirectional"]:
            adj[e["to"]].append((e["from"], i, True, e["len"]))
    return adj


def shortest(adj, a, b):
    """Dijkstra，返回 [(从, 到, 边序号, 反向)]；不可达返回 None；a == b 返回 []。"""
    if a == b:
        return []
    dist, prev, pq = {a: 0.0}, {}, [(0.0, a)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == b:
            break
        if d > dist.get(u, math.inf):
            continue
        for v, ei, rev, w in adj.get(u, []):
            nd = d + w
            if nd < dist.get(v, math.inf) - 1e-12:
                dist[v], prev[v] = nd, (u, ei, rev)
                heapq.heappush(pq, (nd, v))
    if b not in prev:
        return None
    legs, v = [], b
    while v != a:
        u, ei, rev = prev[v]
        legs.append((u, v, ei, rev))
        v = u
    return legs[::-1]


def ramp_distance(v, a, j):
    """jerk 受限从 0 加速到 v（或从 v 减到 0）走过的距离。"""
    if v <= 0:
        return 0.0
    if v <= a * a / j:                       # 加速度到不了上限：三角形加速度曲线
        return v * math.sqrt(v / j)
    return v / 2.0 * (v / a + a / j)


def steady_need(v, acc=0.5, dec=0.4, jerk=0.2, hold=2.0):
    """在 v 保持 hold 秒稳态所需的最短直线长度。"""
    return ramp_distance(v, acc, jerk) + v * hold + ramp_distance(v, dec, jerk)


V_CORNER = 0.35          # 合并路段在拐角处的限速近似（与模拟教程同值）
MERGE_MAX_TURN = 120.0   # 调度规划器合并相邻路段的最大转角（V0.5.1 schedule_planner.yaml merge_max_turn_angle_deg）
MAX_COURSE_ABOVE = 0.8   # 档位高于平滑器默认前进上限（要连平滑器一起改）时，模拟教程走「直线最大速度」课程
SCENE_TAGS = {"recommend": '<script id="scene" type="application/json">',
              "max": '<script id="scene_max" type="application/json">'}


def ramp_between(v0, v1, a, j):
    """从 v0 变到 v1 的距离（估算：两次从静止起步的距离之差）。"""
    lo, hi = sorted((max(v0, 0.0), max(v1, 0.0)))
    return max(0.0, ramp_distance(hi, a, j) - ramp_distance(lo, a, j))


def _travel_poly(norm, lg):
    poly = norm["edges"][lg["edge"]]["poly"]
    return poly[::-1] if lg["reversed"] else poly


def _heading(norm, lg):
    p = _travel_poly(norm, lg)
    return math.atan2(p[-1][1] - p[0][1], p[-1][0] - p[0][0])


def _turn(norm, prev, nxt):
    """前一段末端切向到后一段起点切向的转角（度）。"""
    a, b = _travel_poly(norm, prev), _travel_poly(norm, nxt)
    h1 = math.atan2(a[-1][1] - a[-2][1], a[-1][0] - a[-2][0])
    h2 = math.atan2(b[1][1] - b[0][1], b[1][0] - b[0][0])
    return abs(math.degrees(math.atan2(math.sin(h2 - h1), math.cos(h2 - h1))))


def _joint_speed(norm, prev, nxt):
    """两段衔接处的通过速度：换向（倒车切换）或转角超过合并上限 → 停车再走；否则按拐角限速通过。"""
    if prev["backward"] != nxt["backward"] or _turn(norm, prev, nxt) > MERGE_MAX_TURN:
        return 0.0
    return V_CORNER


def straight_runs(norm, tasks, v, lim, tol_deg=5.0):
    """计划里所有直线前进路段（相邻共线的直线前进边合并成一段），按「预计在 v 能保持多久」排序。

    进出速度：路段起点是任务起点、前一段要倒车或转角超过合并上限（停车原地转）→ 从 0 起步；否则按拐角限速进入；
    出口同理。预计稳态时长 = (长度 − 加速距离 − 减速距离) / v，加减速按 jerk 受限估算。"""
    runs = []
    for t in tasks:
        legs, i = t["legs"], 0
        while i < len(legs):
            lg = legs[i]
            if not lg["line"] or lg["backward"]:
                i += 1
                continue
            j, hdg = i, _heading(norm, lg)
            while j + 1 < len(legs) and legs[j + 1]["line"] and not legs[j + 1]["backward"] and \
                    abs(math.degrees(math.atan2(math.sin(_heading(norm, legs[j + 1]) - hdg),
                                                math.cos(_heading(norm, legs[j + 1]) - hdg)))) <= tol_deg:
                j += 1
            v_in = 0.0 if i == 0 else _joint_speed(norm, legs[i - 1], legs[i])
            v_out = 0.0 if j == len(legs) - 1 else _joint_speed(norm, legs[j], legs[j + 1])
            length = sum(x["len"] for x in legs[i:j + 1])
            d_up, d_dn = ramp_between(v_in, v, lim["acc"], lim["jerk"]), ramp_between(v, v_out, lim["dec"], lim["jerk"])
            runs.append({"task": t["id"], "from": legs[i]["from"], "to": legs[j]["to"],
                         "via": [legs[i]["from"]] + [x["to"] for x in legs[i:j + 1]], "len": length,
                         "v_in": v_in, "v_out": v_out, "d_up": d_up, "d_dn": d_dn,
                         "reach": length >= d_up + d_dn, "steady": max(0.0, (length - d_up - d_dn) / v)})
            i = j + 1
    best = {}
    for r in runs:
        k = (r["from"], r["to"])
        if k not in best or r["steady"] > best[k]["steady"]:
            best[k] = r
    return sorted(best.values(), key=lambda r: (-r["steady"], -r["len"]))


def suggest(names, n):
    return difflib.get_close_matches(n, list(names), n=3, cutoff=0.3)


def make_plan(norm, start, keys, segment=None, close=True, target_speed=0.75, limits=None):
    """返回计划 dict：tasks（每个含 points / legs / len / back）、segment、warnings。"""
    pts = norm["points"]
    for n in [start] + list(keys):
        if n not in pts:
            hint = suggest(pts, n)
            raise PlanError(f"地图里没有点位「{n}」" + (f"，是不是 {'、'.join(hint)}？" if hint else ""))
    if len(set(keys)) != len(keys):
        raise PlanError("关键点位有重复")
    adj = graph(norm)
    stops = list(keys) + ([start] if close else [])
    tasks, cur = [], start
    for k, target in enumerate(stops):
        legs = shortest(adj, cur, target)
        if legs is None:
            n_j = sum(1 for e in norm["edges"] if not e["nav"])
            raise PlanError(f"路网里没有从「{cur}」到「{target}」的导航点直连路线"
                            + (f"（有 {n_j} 条边经过交叉点，这类边不能用于路网任务）" if n_j else "")
                            + "；检查边的方向或在地图里补边")
        if not legs:
            cur = target
            continue
        names = [legs[0][0]] + [lg[1] for lg in legs]
        edges = [norm["edges"][lg[2]] for lg in legs]
        tasks.append({"id": f"T{len(tasks) + 1}", "points": names, "end": target, "key": target in keys,
                      "legs": [{"from": lg[0], "to": lg[1], "edge": lg[2], "reversed": lg[3],
                                "backward": norm["edges"][lg[2]]["backward"], "line": norm["edges"][lg[2]]["line"],
                                "len": norm["edges"][lg[2]]["len"]} for lg in legs],
                      "len": sum(e["len"] for e in edges), "back": sum(1 for e in edges if e["backward"])})
        cur = target
    if not tasks:
        raise PlanError("起点和关键点位相同，生成不出任务")
    warnings = []
    lim = dict({"acc": 0.5, "dec": 0.4, "jerk": 0.2}, **(limits or {}))
    need = steady_need(target_speed, lim["acc"], lim["dec"], lim["jerk"])
    runs = straight_runs(norm, tasks, target_speed, lim)
    if segment:
        a, b = segment
        seg = None
        for r in runs:
            if a in r["via"] and b in r["via"] and r["via"].index(a) < r["via"].index(b):
                if (a, b) == (r["from"], r["to"]):
                    seg = dict(r)
                else:
                    seg = {"from": a, "to": b, "len": None, "steady": None, "reach": None, "via": r["via"]}
                break
        if seg is None or seg["len"] is None:
            hit = None
            for t in tasks:
                if a in t["points"] and b in t["points"] and t["points"].index(a) < t["points"].index(b):
                    i, j = t["points"].index(a), t["points"].index(b)
                    hit = {"from": a, "to": b, "len": sum(lg["len"] for lg in t["legs"][i:j]), "steady": None,
                           "reach": None, "via": t["points"][i:j + 1]}
                    if seg is None:
                        warnings.append(f"指定的关键路段 {a} → {b} 不是一段直线前进路段（含弯道或倒车），稳态时长没有预估")
                    break
            if hit is None:
                raise PlanError(f"关键路段 {a} → {b} 不在生成的任何任务里（要按行驶方向写，起点在前）")
            seg = hit
        seg["auto"] = False
        if seg.get("steady") is not None and seg["steady"] < 2.0:
            warnings.append(f"指定的关键路段 {a} → {b}（{seg['len']:.2f} m）预计只能在 {target_speed:g} m/s 保持 "
                            f"{seg['steady']:.1f} s，判不了 2 s 稳态")
    elif runs:
        seg = dict(runs[0], auto=True)
        if runs[0]["steady"] < 2.0:
            warnings.append(f"没有能在 {target_speed:g} m/s 保持 2 s 的直线前进路段：最好的 {seg['from']} → {seg['to']}"
                            f"（{seg['len']:.2f} m）预计只能保持 {seg['steady']:.1f} s，只能记录最高速度，判不了稳态")
    else:
        lg = max((lg for t in tasks for lg in t["legs"]), key=lambda x: x["len"])
        seg = {"from": lg["from"], "to": lg["to"], "len": lg["len"], "steady": None, "reach": None,
               "via": [lg["from"], lg["to"]], "auto": True}
        warnings.append("整圈没有直线前进路段，关键路段改取最长的一段")
    seg["need"] = need
    seg["candidates"] = runs[:6]
    seg["runs"] = runs
    return {"map_name": norm["map_name"], "revision": norm["revision"], "start": start, "keys": list(keys),
            "close": close, "target_speed": target_speed, "tasks": tasks, "segment": seg, "warnings": warnings,
            "generated": time.strftime("%Y-%m-%d %H:%M:%S")}


# ---------------------------------------------------------------- 输出

def seg_note(seg, v):
    via = f"经 {'、'.join(seg['via'][1:-1])}，" if len(seg.get("via", [])) > 2 else ""
    est = f"预计 {v:g} m/s 保持 {seg['steady']:.1f} s" if seg.get("steady") is not None else "未预估稳态时长"
    head = "自动选取：预计稳态最长的直线前进路段，" if seg["auto"] else "指定，"
    return f"{head}{via}{seg['len']:.2f} m，{est}"


def plan_yaml(plan, batch):
    seg = plan["segment"]
    lines = [f"# 由 nbt pathnav plan 生成：地图 {plan['map_name']}（revision {plan['revision']}），{plan['generated']}",
             f"# 起点 {plan['start']}；关键点位按顺序 {' → '.join(plan['keys'])}"
             + ("；最后回到起点" if plan["close"] else ""),
             "# 相邻两点之间都是路网里直接相连的边；地图或路网改过之后要重新生成",
             f"batch: {batch}", f"target_speed: {plan['target_speed']:g}",
             f"key_points: [{', '.join(plan['keys'])}]",
             f"segment: [{seg['from']}, {seg['to']}]   # {seg_note(seg, plan['target_speed'])}", "tasks:"]
    for t in plan["tasks"]:
        lines.append(f"  - {{id: {t['id']}, points: [{', '.join(t['points'])}]}}   # {t['len']:.2f} m"
                     + (f"，倒车段 {t['back']}" if t["back"] else ""))
    return "\n".join(lines) + "\n"


def task_groups(norm, task):
    """把任务按「是否倒车」切成连续的组，每组给出行驶方向的折线。"""
    groups = []
    for lg in task["legs"]:
        poly = list(norm["edges"][lg["edge"]]["poly"])
        if lg["reversed"]:
            poly.reverse()
        if groups and groups[-1]["rev"] == lg["backward"]:
            groups[-1]["poly"] += poly[1:]
        else:
            groups.append({"rev": lg["backward"], "poly": poly})
    return groups


def course_of(target_speed):
    """模拟教程的课程：推荐速度（recommend）或直线最大速度（max）。"""
    return "max" if target_speed is not None and float(target_speed) > MAX_COURSE_ABOVE + 1e-9 else "recommend"


def scene(norm, plan, batch, title=""):
    """模拟教程用的场景数据（地图坐标，单位 m）。"""
    used = {p for t in plan["tasks"] for p in t["points"]}
    return {
        "course": course_of(plan["target_speed"]),
        "title": title or f"{norm['map_name']} 路网（现场地图）",
        "source": f"地图 {norm['map_name']}（revision {norm['revision']}），nbt pathnav plan 于 {plan['generated']} 生成",
        "map_name": norm["map_name"], "revision": norm["revision"],
        "batch": batch, "target_speed": plan["target_speed"], "start": plan["start"], "keys": plan["keys"],
        "points": {n: dict(p, used=n in used) for n, p in norm["points"].items()},
        "edges": [{"poly": [[round(x, 3), round(y, 3)] for x, y in e["poly"]], "bi": e["bidirectional"],
                   "nav": e["nav"]} for e in norm["edges"]],
        "areas": [{"name": a["name"], "type": a["type"], "label": AREA_NAME.get(a["type"], ""),
                   "color": AREA_COLOR.get(a["type"], "#868e96"),
                   "polygon": [[round(x, 3), round(y, 3)] for x, y in a["polygon"]]} for a in norm["areas"]],
        "decor": [],
        "tasks": [{"id": t["id"], "points": t["points"], "len": round(t["len"], 3), "back": t["back"],
                   "segs": len(t["legs"]),
                   "groups": [{"rev": g["rev"], "poly": [[round(x, 3), round(y, 3)] for x, y in g["poly"]]}
                              for g in task_groups(norm, t)]} for t in plan["tasks"]],
        "segment": {"from": plan["segment"]["from"], "to": plan["segment"]["to"],
                    "len": round(plan["segment"]["len"], 3), "steady": plan["segment"].get("steady"),
                    "via": plan["segment"].get("via", []), "auto": plan["segment"]["auto"]},
        "segment_candidates": [{k: (round(c[k], 3) if isinstance(c[k], float) else c[k])
                                for k in ("task", "from", "to", "via", "len", "v_in", "v_out", "steady", "reach")}
                               for c in plan["segment"].get("candidates", [])],
        "segment_runs": [{k: (round(c[k], 3) if isinstance(c[k], float) else c[k])
                          for k in ("task", "from", "to", "via", "len", "v_in", "v_out", "steady", "reach")}
                         for c in plan["segment"].get("runs", [])],
        "plan_yaml": plan_yaml(plan, batch),
    }


def preview_svg(norm, plan):
    """路网 + 生成的任务路线预览，给人和现场路网图对照。"""
    xs, ys = [], []
    for e in norm["edges"]:
        xs += [p[0] for p in e["poly"]]
        ys += [p[1] for p in e["poly"]]
    xs += [p["x"] for p in norm["points"].values()]
    ys += [p["y"] for p in norm["points"].values()]
    if not xs:
        raise PlanError("地图里没有点位和边，画不出预览")
    m = 1.5
    x0, x1, y0, y1 = min(xs) - m, max(xs) + m, min(ys) - m, max(ys) + m
    W, H = (x1 - x0) * 100, (y1 - y0) * 100
    k = max(0.8, max(W, H) / 2000.0)          # 小地图也保证文字看得清

    def X(x):
        return (x - x0) * 100

    def Y(y):
        return (y1 - y) * 100

    def pl(poly):
        return " ".join(f"{X(x):.1f},{Y(y):.1f}" for x, y in poly)
    colors = ["#d9480f", "#1971c2", "#2f9e44", "#ae3ec9", "#f08c00", "#0c8599", "#e03131", "#5c940d"]
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W:.0f} {H + 260 * k:.0f}" '
           f'font-family="PingFang SC,Noto Sans CJK SC,sans-serif">',
           '<defs><marker id="a" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" markerHeight="6" '
           'orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="#adb5bd"/></marker></defs>',
           '<rect width="100%" height="100%" fill="#fff"/>']
    for a in norm["areas"]:
        c = AREA_COLOR.get(a["type"], "#868e96")
        out.append(f'<polygon points="{pl(a["polygon"])}" fill="{c}" fill-opacity="0.12" stroke="{c}" '
                   f'stroke-width="{3 * k:.1f}"/>')
        cx = sum(p[0] for p in a["polygon"]) / len(a["polygon"])
        cy = sum(p[1] for p in a["polygon"]) / len(a["polygon"])
        out.append(f'<text x="{X(cx):.1f}" y="{Y(cy):.1f}" text-anchor="middle" font-size="{24 * k:.1f}" fill="{c}">'
                   f'{a["name"] or AREA_NAME.get(a["type"], "")}</text>')
    for e in norm["edges"]:
        dash = f' stroke-dasharray="{12 * k:.0f} {8 * k:.0f}"' if not e["nav"] else ""
        mk = ' marker-end="url(#a)"' + (' marker-start="url(#a)"' if e["bidirectional"] else "")
        out.append(f'<polyline points="{pl(e["poly"])}" fill="none" stroke="#ced4da" stroke-width="{6 * k:.1f}"'
                   f'{dash}{mk}/>')
    for i, t in enumerate(plan["tasks"]):
        c = colors[i % len(colors)]
        for g in task_groups(norm, t):
            dash = f' stroke-dasharray="{20 * k:.0f} {10 * k:.0f}"' if g["rev"] else ""
            out.append(f'<polyline points="{pl(g["poly"])}" fill="none" stroke="{c}" stroke-opacity="0.7" '
                       f'stroke-width="{12 * k:.1f}" stroke-linejoin="round"{dash}/>')
    seg = plan["segment"]
    pairs = set(zip(seg.get("via", [seg["from"], seg["to"]]), seg.get("via", [seg["from"], seg["to"]])[1:]))
    drawn = set()
    for t in plan["tasks"]:
        for lg in t["legs"]:
            if (lg["from"], lg["to"]) in pairs and (lg["from"], lg["to"]) not in drawn:
                drawn.add((lg["from"], lg["to"]))
                out.append(f'<polyline points="{pl(norm["edges"][lg["edge"]]["poly"])}" fill="none" stroke="#fab005" '
                           f'stroke-opacity="0.5" stroke-width="{34 * k:.1f}"/>')
    for n, p in norm["points"].items():
        cx, cy = X(p["x"]), Y(p["y"])
        if n in plan["keys"]:
            out.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{44 * k:.1f}" fill="none" stroke="#d9480f" '
                       f'stroke-width="{7 * k:.1f}" stroke-dasharray="{14 * k:.0f} {8 * k:.0f}"/>')
        out.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{22 * k:.1f}" fill="#4f9a62" stroke="#1f2428" '
                   f'stroke-width="{3 * k:.1f}"/>')
        hx, hy = math.cos(p["theta"]), -math.sin(p["theta"])
        out.append(f'<line x1="{cx:.1f}" y1="{cy:.1f}" x2="{cx + 30 * k * hx:.1f}" y2="{cy + 30 * k * hy:.1f}" '
                   f'stroke="#1f2428" stroke-width="{4 * k:.1f}"/>')
        bold = ' font-weight="700"' if n in plan["keys"] or n == plan["start"] else ""
        out.append(f'<text x="{cx:.1f}" y="{cy + 62 * k:.1f}" text-anchor="middle" font-size="{28 * k:.1f}"'
                   f'{bold}>{n}</text>')
    y = H + 40 * k
    out.append(f'<text x="{20 * k:.0f}" y="{y:.0f}" font-size="{26 * k:.1f}" fill="#495057">地图 {norm["map_name"]}'
               f'（revision {norm["revision"]}）　起点 {plan["start"]}　橙色虚线圈：关键点位　黄色粗线：关键路段 '
               f'{seg["from"]} → {seg["to"]}（{seg["len"]:.2f} m）　虚线：倒车</text>')
    for i, t in enumerate(plan["tasks"]):
        y += 34 * k
        out.append(f'<text x="{20 * k:.0f}" y="{y:.0f}" font-size="{24 * k:.1f}" fill="{colors[i % len(colors)]}">'
                   f'{t["id"]}　{" → ".join(t["points"])}　{t["len"]:.2f} m'
                   + (f"，倒车段 {t['back']}" if t["back"] else "") + '</text>')
    out.append("</svg>")
    return "\n".join(out)


def template_path():
    """模拟教程模板：安装后在 share/nav_benchmark_tools/resources/，源码树里在包目录 resources/。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory("nav_benchmark_tools"), "resources", "tutorial_template.html")
        if os.path.exists(p):
            return p
    except Exception:  # noqa: BLE001  没有 ament 环境时用源码树
        pass
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "resources",
                        "tutorial_template.html")


def render_tutorial(scene_data, template=None, html=None, set_default=True):
    """把场景数据替换进模板里对应课程的场景块（推荐速度 id="scene"，直线最大速度 id="scene_max"）；
    set_default 时打开页面默认显示这门课。html 给出时在这段文本上替换，不读模板。"""
    if html is None:
        with open(template or template_path(), encoding="utf-8") as f:
            html = f.read()
    course = scene_data.get("course") or "recommend"
    tag = SCENE_TAGS[course]
    i = html.index(tag) + len(tag)
    j = html.index("</script>", i)
    body = json.dumps(scene_data, ensure_ascii=False).replace("</", "<\\/")
    html = html[:i] + body + html[j:]
    if set_default:
        meta = f'<meta name="nbt-course" content="{course}">'
        html = re.sub(r'<meta name="nbt-course" content="[a-z]+">', meta, html, count=1)
    return html
