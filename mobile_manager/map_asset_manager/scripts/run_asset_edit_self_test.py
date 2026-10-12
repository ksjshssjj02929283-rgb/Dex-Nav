#!/usr/bin/env python3
"""地图资产按图寻址 / 几何校验 / 失效清单 / 版本回滚 / 禁行区掩膜的服务级自测（不碰真实地图）。

  ros2 run map_asset_manager run_asset_edit_self_test.py [--keep] [--domain 88]

做法：
  1. 临时 HOME，把 map_manager 包内测试图 map_standard 拷成两张：一张与安装配置的 default_map_name 同名（当前图），
     一张未加载（非当前图）；两张都放资产包的标准资产样本
  2. 直接起 map_manager_node 与 map_asset_manager_node 二进制（ROS_DOMAIN_ID 隔离，仅本机），结束时确认退出
  3. 测试点位从底图自动挑：空旷处（净空 > 0.9 m）、障碍格、未知格、贴墙处（离障碍 0.1~0.25 m）
  4. 逐条按 M5-4 §14.1 / §14.3 的验收用例调服务、核对文件与话题
  5. T 组：资产 last_edited_at 只随资产内容变化推进（加载 / 无变化保存 / 被拒写入不动），底图时间不受资产编辑影响
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np

E_NONE = 0
E_ASSET_REF_UNRESOLVED = 2203050
E_ASSET_MAP_NOT_FOUND = 2203090
E_ASSET_REVISION_MISMATCH = 2203100
E_ASSET_GEOMETRY_INVALID = 2203110
E_ASSET_VERSION_NOT_FOUND = 2203120


class Report:
    def __init__(self):
        self.rows = []

    def check(self, case, desc, ok, detail=""):
        self.rows.append((case, desc, bool(ok), detail))
        print(f"  {'✅' if ok else '❌'} [{case}] {desc}" + (f" — {detail}" if detail and not ok else ""))

    @property
    def failed(self):
        return [r for r in self.rows if not r[2]]


def yaml_value(path, key):
    for line in Path(path).read_text().splitlines():
        if line.split("#")[0].strip().startswith(key + ":"):
            return line.split(":", 1)[1].split("#")[0].strip().strip("'\"")
    return None


class Grid:
    """map.png + map.yaml → 三值栅格与到障碍 / 未知的距离（米），世界坐标换算"""

    def __init__(self, slam_dir: Path):
        y = slam_dir / "map.yaml"
        self.res = float(yaml_value(y, "resolution"))
        org = yaml_value(y, "origin").strip("[]").split(",")
        self.ox, self.oy = float(org[0]), float(org[1])
        occ_th, free_th = float(yaml_value(y, "occupied_thresh")), float(yaml_value(y, "free_thresh"))
        img = cv2.imread(str(slam_dir / "map.png"), cv2.IMREAD_GRAYSCALE)
        self.h, self.w = img.shape
        p = (255.0 - img) / 255.0
        self.cls = np.where(p > occ_th, 2, np.where(p < free_th, 0, 1))  # 行 0 = 图像顶部
        self.d_occ = cv2.distanceTransform(np.where(self.cls == 2, 0, 255).astype(np.uint8), cv2.DIST_L2, 5) * self.res
        self.d_unk = cv2.distanceTransform(np.where(self.cls == 1, 0, 255).astype(np.uint8), cv2.DIST_L2, 5) * self.res

    def world(self, row, col):
        return self.ox + (col + 0.5) * self.res, self.oy + (self.h - 1 - row + 0.5) * self.res

    def cell(self, x, y):
        col = int(math.floor((x - self.ox) / self.res))
        row = self.h - 1 - int(math.floor((y - self.oy) / self.res))
        return row, col

    def clearance(self, x, y):
        r, c = self.cell(x, y)
        return min(self.d_occ[r, c], self.d_unk[r, c])


def pick(grid: Grid):
    clr = np.minimum(grid.d_occ, grid.d_unk)
    rows, cols = np.where(clr > 1.4)
    order = np.argsort(-clr[rows, cols])
    open_pts = [grid.world(rows[i], cols[i]) for i in order[:4000:40]]
    # 一段 1.6 m 的直线，沿线净空都 > 0.9 m
    seg = None
    for (x, y) in open_pts:
        for ang in np.linspace(0, math.pi, 12, endpoint=False):
            bx, by = x + 1.6 * math.cos(ang), y + 1.6 * math.sin(ang)
            if all(grid.clearance(x + (bx - x) * t, y + (by - y) * t) > 0.9 for t in np.linspace(0, 1, 40)):
                seg = ((x, y), (bx, by))
                break
        if seg:
            break
    orow, ocol = np.argwhere(grid.cls == 2)[len(np.argwhere(grid.cls == 2)) // 2]
    unk = np.argwhere((grid.cls == 1) & (grid.d_occ > 1.0))
    near = np.argwhere((grid.cls == 0) & (grid.d_occ > 0.1) & (grid.d_occ < 0.25) & (grid.d_unk > 0.6))
    return {
        "seg": seg,
        "open": open_pts,
        "obstacle": grid.world(orow, ocol),
        "unknown": grid.world(*unk[len(unk) // 2]),
        "near_wall": grid.world(*near[len(near) // 2]),
    }


def main():
    ap = argparse.ArgumentParser(description="asset by-map / geometry / version self test")
    ap.add_argument("--domain", type=int, default=88)
    ap.add_argument("--keep", action="store_true")
    args = ap.parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"

    import rclpy
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from geometry_msgs.msg import Point, Pose2D
    from nav_interfaces.msg import AreaInfo, AssetChange, AssetSnapshot, GridStroke, NavPoint, RouteEdge, RouteEndpoint
    from nav_interfaces.srv import GetAssets, GetMaps, MapManagerCommand, RollbackAssets, SaveAssets, SetAssets, TransformAssets
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    mm_share = Path(get_package_share_directory("map_manager"))
    src_map = mm_share / "test_data/map_standard"
    sample_assets = Path(get_package_share_directory("map_asset_manager")) / "test_data/map_assets_standard.json"
    # map_manager 加载地图时会整份重写安装目录里的 config.yaml（含 default_map_name）——测试里有加载，
    # 所以先备份原文，结束时还原
    config_yaml = mm_share / "config/config.yaml"
    config_text = config_yaml.read_text()
    cur_name = yaml_value(config_yaml, "default_map_name") or "offline_map"
    other_name = "asset_edit_test_b" if cur_name != "asset_edit_test_b" else "asset_edit_test_c"

    real_maps = Path.home() / ".dexe/data/maps"
    stamp = time.time()
    tmp = Path(tempfile.mkdtemp(prefix="asset_edit_self_test_"))
    maps = tmp / "home/.dexe/data/maps"
    for n in (cur_name, other_name):
        shutil.copytree(src_map, maps / n, ignore=shutil.ignore_patterns("backups", ".*lock"))
        (maps / n / "assets").mkdir(exist_ok=True)
        shutil.copy(sample_assets, maps / n / "assets/map_assets.json")
    other_assets = maps / other_name / "assets/map_assets.json"
    grid = Grid(maps / other_name / "slam")
    sp = pick(grid)
    print(f"temp HOME: {tmp / 'home'}\ncurrent: {cur_name}  non-current: {other_name}\npicked: {json.dumps(sp, default=str)[:300]}")
    if sp["seg"] is None:
        sys.exit("no open straight segment found in the test map")

    env = dict(os.environ, HOME=str(tmp / "home"))
    mm_bin = Path(get_package_prefix("map_manager")) / "lib/map_manager/map_manager_node"
    am_bin = Path(get_package_prefix("map_asset_manager")) / "lib/map_asset_manager/map_asset_manager_node"
    procs = [subprocess.Popen([str(mm_bin), "--ros-args", "-r", "__node:=map_manager"], env=env,
                              stdout=open(tmp / "map_manager.log", "w"), stderr=subprocess.STDOUT),
             subprocess.Popen([str(am_bin)], env=env,
                              stdout=open(tmp / "map_asset_manager.log", "w"), stderr=subprocess.STDOUT)]
    rep = Report()
    rclpy.init()
    node = rclpy.create_node("asset_edit_self_test")
    snapshots, masks, changes = [], [], []
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(AssetSnapshot, "/map_asset_manager/assets", snapshots.append, latched)
    node.create_subscription(OccupancyGrid, "/keepout_filter_mask", masks.append, latched)
    node.create_subscription(AssetChange, "/map_asset_manager/asset_changes", changes.append, 10)
    try:
        ga = node.create_client(GetAssets, "/map_asset_manager/get_assets")
        sa = node.create_client(SetAssets, "/map_asset_manager/set_assets")
        sv = node.create_client(SaveAssets, "/map_asset_manager/save_assets")
        rb = node.create_client(RollbackAssets, "/map_asset_manager/rollback_assets")
        mc = node.create_client(MapManagerCommand, "/map_manager/map_command")
        gm = node.create_client(GetMaps, "/map_manager/get_maps")
        ta = node.create_client(TransformAssets, "/map_asset_manager/transform_assets")
        for c in (ga, sa, sv, rb, mc, gm, ta):
            if not c.wait_for_service(timeout_sec=20.0):
                sys.exit(f"service not available: {c.srv_name} (logs in {tmp})")

        def call(cli, req):
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
            if fut.result() is None:
                raise RuntimeError(f"call {cli.srv_name} timed out")
            return fut.result()

        def spin(sec):
            end = time.time() + sec
            while time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.05)

        def get(name="", kinds=0):
            return call(ga, GetAssets.Request(map_name=name, kinds=kinds))

        def pt(name, x, y, theta=0.0, ptype="STATION"):
            return NavPoint(name=name, type=ptype, x=float(x), y=float(y), theta=float(theta))

        def area(name, poly, atype=AreaInfo.RESTRICTED_AREA):
            return AreaInfo(name=name, type=atype, polygon=[Pose2D(x=float(x), y=float(y)) for x, y in poly])

        def edge(eid, a, b, etype="LINE", cps=(), width=0.0):
            e = RouteEdge(id=eid, edge_type=etype, from_ep=RouteEndpoint(id=a, type="nav"),
                          to_ep=RouteEndpoint(id=b, type="nav"), lane_width=float(width))
            e.control_points = [Point(x=float(x), y=float(y)) for x, y in cps]
            return e

        def setr(name, mode=SetAssets.Request.MODE_MERGE, rev="", save=False, **kw):
            req = SetAssets.Request(map_name=name, mode=mode, expected_revision=rev, save=save)
            for k, v in kw.items():
                setattr(req, k, v)
            return call(sa, req)

        OLD_STAMP = "2020-01-01T00:00:00"

        def stamp_of(path):
            return json.loads(path.read_text()).get("last_edited_at")

        def set_old_stamp(path):
            # 只改元数据不改内容：指纹不变，用来判断后续操作有没有推进时间
            j = json.loads(path.read_text())
            j["last_edited_at"] = OLD_STAMP
            path.write_text(json.dumps(j, indent=2))

        def iso_to_sec(text):
            return int(time.mktime(time.strptime(text, "%Y-%m-%dT%H:%M:%S"))) if text else 0

        def is_now(text):
            return bool(text) and abs(iso_to_sec(text) - time.time()) <= 5

        def base_edited(name):
            for m in call(gm, GetMaps.Request()).maps:
                if m.name == name:
                    return m.last_edited_at.sec
            return -1

        def file_points():
            return {p["name"]: p["pose"] for p in json.loads(other_assets.read_text())["points"]}

        # 等当前图加载完成
        for _ in range(100):
            if get().success:
                break
            time.sleep(0.2)
        spin(1.0)

        (ax, ay), (bx, by) = sp["seg"]
        mx, my = (ax + bx) / 2, (ay + by) / 2

        print("▶ A 资产按图寻址")
        r = get()
        rep.check("A-01", "get_assets without map_name reads the current map",
                  r.success and r.is_current and r.map_name == cur_name and r.map_epoch > 0 and len(r.revision) == 16,
                  r.message)
        r = get(other_name)
        rep.check("A-02", "get_assets on a non-current map", r.success and not r.is_current and r.map_epoch == 0 and
                  len(r.points) > 0 and len(r.revision) == 16, r.message)
        rev0 = r.revision
        r = get("no_such_map")
        rep.check("A-03", "missing map → E_ASSET_MAP_NOT_FOUND", r.error_code == E_ASSET_MAP_NOT_FOUND, r.message)
        r = get("../" + other_name)
        rep.check("A-03", "path-like map name rejected", r.error_code == E_ASSET_MAP_NOT_FOUND, r.message)

        base0 = base_edited(other_name)
        n_snap = len(snapshots)
        before = other_assets.read_text()
        r = setr(other_name, rev=rev0, points=[pt("T_A", ax, ay), pt("T_B", bx, by)])
        spin(0.5)
        rep.check("A-04", "MERGE two points into a non-current map", r.success and r.revision and r.revision != rev0,
                  f"{r.error_code} {r.message} {list(r.rejected)}")
        rep.check("A-04", "file written, current map topic not published",
                  other_assets.read_text() != before and "T_A" in file_points() and len(snapshots) == n_snap,
                  f"snapshots {n_snap}->{len(snapshots)}")
        rev1 = r.revision
        ev = [c for c in changes if c.map_name == other_name and c.op == "set"]
        rep.check("EV-1", "set_assets on a non-current map publishes asset_changes (map, revision, is_current=false)",
                  bool(ev) and ev[-1].revision == rev1 and not ev[-1].is_current,
                  str([(c.map_name, c.op, c.revision) for c in changes]))
        rep.check("A-04", "get_assets revision equals set_assets revision", get(other_name).revision == rev1)
        g = get(other_name)
        rep.check("T-1", "asset edit stamps map_assets.json last_edited_at = now, returned by get_assets",
                  is_now(stamp_of(other_assets)) and g.last_edited_at.sec == iso_to_sec(stamp_of(other_assets)),
                  f"{stamp_of(other_assets)} / {g.last_edited_at.sec}")
        rep.check("T-1", "asset edit leaves the base map last_edited_at (get_maps) untouched",
                  base0 > 0 and base_edited(other_name) == base0, f"{base0} -> {base_edited(other_name)}")
        set_old_stamp(other_assets)
        text1 = other_assets.read_text()
        r = setr(other_name, rev=rev0, points=[pt("T_C", ax, ay)])
        rep.check("A-05", "stale expected_revision → E_ASSET_REVISION_MISMATCH, file untouched",
                  r.error_code == E_ASSET_REVISION_MISMATCH and other_assets.read_text() == text1, r.message)

        print("▶ C 几何校验")
        def rejected(r, code):
            return r.error_code == E_ASSET_GEOMETRY_INVALID and any(code in x for x in r.rejected)
        r = setr(other_name, points=[pt("BAD", *sp["obstacle"])])
        rep.check("C-01", "point in an obstacle → in_obstacle", rejected(r, "in_obstacle"), str(list(r.rejected)))
        r = setr(other_name, points=[pt("BAD", *sp["unknown"])])
        rep.check("C-02", "point in unknown space → in_unknown", rejected(r, "in_unknown"), str(list(r.rejected)))
        r = setr(other_name, points=[pt("BAD", *sp["near_wall"])])
        rep.check("C-03", "point closer than clearance to a wall → too_close_obstacle",
                  rejected(r, "too_close_obstacle"), str(list(r.rejected)))
        rep.check("C-01~03", "rejected writes leave the file untouched", other_assets.read_text() == text1)

        r = setr(other_name, edges=[edge("E_AB", "T_A", "T_B")])
        rep.check("EDGE", "straight edge through open space is accepted", r.success, f"{r.message} {list(r.rejected)}")
        r = setr(other_name, edges=[edge("E_W", "T_A", "T_B", width=0.3)])
        rep.check("WIDTH", "lane_width below robot diameter → width_too_small", rejected(r, "width_too_small"),
                  str(list(r.rejected)))
        r = setr(other_name, edges=[edge("E_W", "T_A", "T_B", width=1.0)])
        rep.check("WIDTH", "lane_width 1.0 m in open space accepted", r.success, f"{r.message} {list(r.rejected)}")
        r = setr(other_name, edges=[edge("E_BZ", "T_A", "T_B", "BEZIER", [(ax, ay), (bx, by)])])
        rep.check("CURVE", "BEZIER without a middle control point → missing_control_point",
                  rejected(r, "missing_control_point"), str(list(r.rejected)))

        sq = [(mx - 0.1, my - 0.1), (mx + 0.1, my - 0.1), (mx + 0.1, my + 0.1), (mx - 0.1, my + 0.1)]
        r = setr(other_name, areas=[area("R_MID", sq)], edges=[edge("E_X", "T_A", "T_B")])
        rep.check("C-04", "new edge crossing a restricted area (same request) → crosses_restricted",
                  rejected(r, "edge:E_X:crosses_restricted"), str(list(r.rejected)))
        bow = [(mx - 0.3, my - 0.3), (mx + 0.3, my + 0.3), (mx + 0.3, my - 0.3), (mx - 0.3, my + 0.3)]
        r = setr(other_name, areas=[area("R_BOW", bow)])
        rep.check("C-05", "self-intersecting area → self_intersecting", rejected(r, "self_intersecting"),
                  str(list(r.rejected)))
        r = setr(other_name, areas=[area("W_LINE", [(mx - 0.3, my), (mx + 0.3, my)], AreaInfo.VIRTUAL_WALL)])
        rep.check("C-11", "two-point virtual wall is accepted (walls are polylines)", r.success,
                  f"{r.message} {list(r.rejected)}")
        r = setr(other_name, areas=[area("R_LINE", [(mx - 0.3, my), (mx + 0.3, my)])])
        rep.check("C-11", "two-point restricted area is rejected (polygons need 3 points)",
                  not r.success and any("needs >= 3" in x for x in r.rejected), str(list(r.rejected)))
        setr(other_name, mode=SetAssets.Request.MODE_REMOVE, areas=[area("W_LINE", [])])
        tr = call(ta, TransformAssets.Request(map_name="no_such_map", x=1.0, y=0.0, yaw=0.0))
        rep.check("T-0", "transform_assets on a missing map is rejected (not reported as success)",
                  not tr.success, tr.message)

        print("▶ 失效清单")
        r = setr(other_name, areas=[area("R_MID", sq)])
        issues = {(i.kind, i.key, i.code) for i in r.issues}
        rep.check("C-07", "restricted area over existing edges is saved; edges listed as crosses_restricted",
                  r.success and ("edge", "E_AB", "crosses_restricted") in issues, f"{r.message} {issues}")
        r = setr(other_name, mode=SetAssets.Request.MODE_REMOVE, areas=[area("R_MID", [])])
        rep.check("C-07", "removing the area clears the issue", r.success and not [i for i in r.issues if i.key == "E_AB"],
                  str([(i.kind, i.key, i.code) for i in r.issues]))
        info = get(other_name)
        mreq = MapManagerCommand.Request(op=MapManagerCommand.Request.OP_EDIT_GRID, map_name=other_name)
        maps_info = None
        from nav_interfaces.srv import GetMaps
        gm = node.create_client(GetMaps, "/map_manager/get_maps")
        gm.wait_for_service(timeout_sec=5.0)
        for m in call(gm, GetMaps.Request()).maps:
            if m.name == other_name:
                maps_info = m
        mreq.expected_revision = maps_info.grid_revision
        mreq.strokes = [GridStroke(tool=GridStroke.TOOL_WALL, shape=GridStroke.SHAPE_POLYLINE, width_m=0.3,
                                   points=[Point(x=ax - 0.2, y=ay), Point(x=ax + 0.2, y=ay)])]
        mr = call(mc, mreq)
        r = get(other_name)
        issues = {(i.kind, i.key, i.code) for i in r.issues}
        rep.check("C-06", "wall drawn over an existing point → point in_obstacle, edge endpoint_unavailable, asset kept",
                  mr.success and ("point", "T_A", "in_obstacle") in issues and ("edge", "E_AB", "endpoint_unavailable") in issues
                  and any(p.name == "T_A" for p in r.points), f"{mr.message} {issues}")

        print("▶ 曲线路网随动（C-09）")
        o2 = sp["open"][len(sp["open"]) // 2]
        r = setr(other_name, points=[pt("T_B2", bx, by), pt("T_C2", *o2)])
        r = setr(other_name, mode=SetAssets.Request.MODE_REMOVE, points=[pt("T_A", 0, 0)])  # 墙上的点删掉，连带 E_AB
        # 曲线 B2 → C3（C3 取 B 周围 0.8 m 内的空旷处）
        cand = [(bx + 0.8 * math.cos(t), by + 0.8 * math.sin(t)) for t in np.linspace(0, 2 * math.pi, 16, endpoint=False)]
        cand = [c for c in cand if grid.clearance(*c) > 0.6]
        c3 = cand[0]
        setr(other_name, points=[pt("T_C3", *c3)])
        mid = ((bx + c3[0]) / 2 + 0.1, (by + c3[1]) / 2 + 0.1)
        r1 = setr(other_name, edges=[edge("E_CURVE", "T_B2", "T_C3", "BEZIER", [(bx, by), mid, c3])])
        moved = (c3[0] + 0.05, c3[1] + 0.05)
        r2 = setr(other_name, points=[pt("T_C3", *moved)])
        e = [x for x in get(other_name).edges if x.id == "E_CURVE"]
        ok = False
        if r1.success and r2.success and e:
            cps = [(p.x, p.y) for p in e[0].control_points]
            # 相似变换：端点吸附到新位置，中间点与端点的相对关系不变
            a0, b0, a1, b1 = complex(bx, by), complex(*c3), complex(bx, by), complex(*moved)
            s = (b1 - a1) / (b0 - a0)
            exp_mid = a1 + s * (complex(*mid) - a0)
            ok = abs(complex(*cps[-1]) - b1) < 1e-3 and abs(complex(*cps[1]) - exp_mid) < 1e-3
        rep.check("C-09", "moving an endpoint point drags the bezier edge (shape preserved)", ok,
                  f"{r1.message} {list(r1.rejected)} / {r2.message} / {e[0].control_points if e else None}")

        print("▶ 版本与回滚")
        set_old_stamp(other_assets)
        rev_saved = get(other_name).revision
        sr = call(sv, SaveAssets.Request(map_name=other_name))
        rep.check("T-2", "save_assets without content change keeps last_edited_at",
                  stamp_of(other_assets) == OLD_STAMP, str(stamp_of(other_assets)))
        rep.check("V-1", "save_assets on a non-current map creates a version", sr.success and sr.backup_ts, sr.message)
        vs = get(other_name).versions
        rep.check("V-1", "version listed with source save", any(v.version_ts == sr.backup_ts and v.source == "save" for v in vs),
                  str([(v.version_ts, v.source) for v in vs]))
        setr(other_name, mode=SetAssets.Request.MODE_REMOVE, points=[pt("T_C2", 0, 0)])
        rev_before = get(other_name).revision
        rr = call(rb, RollbackAssets.Request(map_name=other_name, version_ts=sr.backup_ts))
        rep.check("V-2", "rollback_assets restores the version and snapshots the previous state",
                  rr.success and rr.revision == rev_saved and get(other_name).revision == rev_saved and rr.backup_ts
                  and not rr.is_current, rr.message)
        rep.check("T-3", "rollback counts as an edit: last_edited_at = now, not the version's old value",
                  is_now(stamp_of(other_assets)), str(stamp_of(other_assets)))
        spin(0.3)
        ev = [c for c in changes if c.map_name == other_name and c.op == "rollback"]
        rep.check("EV-2", "rollback_assets publishes asset_changes with the restored version and revision",
                  bool(ev) and ev[-1].version_ts == sr.backup_ts and ev[-1].revision == rr.revision)
        rep.check("V-2", "pre-rollback snapshot has source rollback",
                  any(v.version_ts == rr.backup_ts and v.source == "rollback" for v in get(other_name).versions))
        r = setr(other_name, rev=rev_before, points=[pt("T_Z", *o2)])
        rep.check("A-06", "write with a revision read before the rollback → REVISION_MISMATCH",
                  r.error_code == E_ASSET_REVISION_MISMATCH, r.message)
        rr2 = call(rb, RollbackAssets.Request(map_name=other_name, version_ts=rr.backup_ts))
        rep.check("V-3", "rolling back to the snapshot undoes the rollback",
                  rr2.success and "T_C2" not in file_points(), rr2.message)
        rr3 = call(rb, RollbackAssets.Request(map_name=other_name, version_ts="20990101-000000"))
        rep.check("V-4", "missing version → E_ASSET_VERSION_NOT_FOUND", rr3.error_code == E_ASSET_VERSION_NOT_FOUND, rr3.message)
        rr4 = call(rb, RollbackAssets.Request(map_name=other_name, version_ts="../../x"))
        rep.check("V-4", "path-like version rejected", rr4.error_code == E_ASSET_VERSION_NOT_FOUND, rr4.message)

        print("▶ A-08 删除点位级联")
        setr(other_name, points=[pt("T_D", *c3)])
        setr(other_name, edges=[edge("E_BD", "T_B2", "T_D")])
        r = setr(other_name, mode=SetAssets.Request.MODE_REMOVE, points=[pt("T_D", 0, 0)])
        rep.check("A-08", "REMOVE a point on a non-current map removes its edges",
                  r.success and not any(x.id == "E_BD" for x in get(other_name).edges), r.message)

        print("▶ 当前地图：禁行区掩膜（C-10）与回滚重载")
        spin(1.0)
        cur = get()
        masks.clear()
        r = setr("", rev=cur.revision, areas=[area("R_CUR", [(ax - 0.3, ay - 0.3), (ax + 0.3, ay - 0.3), (ax + 0.3, ay + 0.3), (ax - 0.3, ay + 0.3)])])
        spin(1.5)
        hit = False
        if masks:
            m = masks[-1]
            col = int((ax - m.info.origin.position.x) / m.info.resolution)
            row = int((ay - m.info.origin.position.y) / m.info.resolution)
            hit = m.data[row * m.info.width + col] == 100
        rep.check("C-10", "restricted area on the current map republishes /keepout_filter_mask with the area",
                  r.success and hit, f"{r.message} masks={len(masks)}")
        rep.check("P1", "current map also honours expected_revision",
                  setr("", rev=cur.revision, points=[pt("T_Q", *o2)]).error_code == E_ASSET_REVISION_MISMATCH)
        sv_cur = call(sv, SaveAssets.Request(map_name=""))
        setr("", mode=SetAssets.Request.MODE_REMOVE, areas=[area("R_CUR", [])])
        spin(0.3)
        ev = [c for c in changes if c.op == "set" and c.is_current]
        rep.check("EV-3", "writes to the current map also publish asset_changes (is_current=true)",
                  bool(ev) and ev[-1].map_name == cur_name)
        n_snap = len(snapshots)
        rr = call(rb, RollbackAssets.Request(map_name="", version_ts=sv_cur.backup_ts))
        spin(1.0)
        rep.check("V-5", "rollback on the current map reloads memory and republishes",
                  rr.success and rr.is_current and any(a.name == "R_CUR" for a in get().areas) and len(snapshots) > n_snap,
                  rr.message)

        print("▶ A-07 离线编辑后加载")
        set_old_stamp(other_assets)
        lr = call(mc, MapManagerCommand.Request(op=MapManagerCommand.Request.OP_LOAD, map_name=other_name))
        for _ in range(50):
            spin(0.2)
            g = get()
            if g.map_name == other_name:
                break
        g = get()
        rep.check("A-07", "after loading the edited map, get_assets returns the edited content",
                  lr.success and g.is_current and g.map_name == other_name and any(p.name == "T_C3" for p in g.points),
                  lr.message)
        spin(2.0)
        rep.check("T-4", "loading a map does not advance its asset last_edited_at",
                  stamp_of(other_assets) == OLD_STAMP and get().last_edited_at.sec == iso_to_sec(OLD_STAMP),
                  f"{stamp_of(other_assets)} / {get().last_edited_at.sec}")
        cur_assets = maps / cur_name / "assets/map_assets.json"
        j = json.loads(cur_assets.read_text())
        j.pop("last_edited_at", None)
        cur_assets.write_text(json.dumps(j, indent=2))
        rep.check("T-5", "legacy assets file without the key → get_assets falls back to file mtime",
                  get(cur_name).last_edited_at.sec == int(cur_assets.stat().st_mtime),
                  f"{get(cur_name).last_edited_at.sec} vs {int(cur_assets.stat().st_mtime)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for p in procs:
            p.send_signal(signal.SIGINT)
        for p in procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()

    config_changed = config_yaml.read_text() != config_text
    config_yaml.write_text(config_text)
    print("▶ isolation")
    print(f"  (install config.yaml {'rewritten by map_manager during the test, restored' if config_changed else 'untouched'})")
    rep.check("ISO", "test nodes exited", all(p.poll() is not None for p in procs))
    touched = [str(p) for p in real_maps.rglob("*") if p.exists() and p.stat().st_mtime > stamp] if real_maps.exists() else []
    rep.check("ISO", "real ~/.dexe/data/maps untouched", not touched, str(touched[:5]))
    rep.check("ISO", "install config default_map_name restored", yaml_value(config_yaml, "default_map_name") == cur_name)
    total, failed = len(rep.rows), rep.failed
    print(f"\n== {total - len(failed)}/{total} passed ==")
    if args.keep or failed:
        print(f"temp dir kept: {tmp}")
    else:
        shutil.rmtree(tmp, ignore_errors=True)
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
