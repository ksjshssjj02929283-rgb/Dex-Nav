#!/usr/bin/env python3
"""地图编辑（底图笔画编辑 / 显示朝向 / 指定地图回滚）服务级自测。

用法（先 source 好 ROS 与本工作区的 install）：
    ros2 run map_manager run_map_edit_self_test.py
    ros2 run map_manager run_map_edit_self_test.py --map-dir <某张图的目录（含 slam/）> --keep

做什么：
  1. 建一个临时 HOME，把测试地图拷成两张：一张与 install 配置里的 default_map_name 同名
     （节点启动即加载，充当「当前图」），一张 map_edit_test_b（未加载）
  2. 用临时 HOME 起 map_manager_node、command_board_node 与 map_asset_manager_node（直接起二进制，结束时确认退出）
  3. 按地图编辑方案 §14.2 的用例与版本回滚分工（map_manager 任意地图 / command_board 当前地图）逐条调用服务并核对落盘文件；底图编辑结果与本脚本里独立实现的
     栅格化规则逐像素比对（前后端一致性规则的参考实现）
  4. 核对本机真实地图目录与 install 配置文件在测试前后未被改动
  5. T 组：底图 last_edited_at（map.yaml 键）只随底图编辑推进，加载与资产操作不动它

只改临时目录，不碰 ~/.dexe/data/maps。全部通过退出码 0。
"""

from __future__ import annotations

import argparse
import hashlib
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

TOOL_WALL, TOOL_ERASE, TOOL_UNKNOWN = 1, 2, 3
SHAPE_POLYLINE, SHAPE_RECT = 1, 2
E_MAP_NOT_FOUND = 2202030
E_MAP_EDIT_INVALID = 2202100
E_MAP_GRID_REVISION_MISMATCH = 2202110
E_MAP_VERSION_NOT_FOUND = 2202120


# ── 栅格化参考实现（与 map_manager grid_edit.cpp 规则一致，独立编写用于交叉核对）──────────────

def classify(p, occ, free):
    if p > 255.0 * (1.0 - free):
        return "free"
    if p < 255.0 * (1.0 - occ):
        return "occ"
    return "unknown"


def unknown_gray(occ, free):
    lo = math.ceil(255.0 * (1.0 - occ))
    hi = math.floor(255.0 * (1.0 - free))
    return 205 if lo <= 205 <= hi else (lo + hi) // 2


def dist_to_segment(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 == 0 else min(1.0, max(0.0, ((px - ax) * dx + (py - ay) * dy) / l2))
    return math.hypot(ax + t * dx - px, ay + t * dy - py)


def rasterize(img, geo, occ, free, strokes):
    """strokes: [(tool, shape, [(x, y), ...], width_m)]；原地修改 img。"""
    res, ox, oy = geo["res"], geo["ox"], geo["oy"]
    h, w = img.shape
    ugray = unknown_gray(occ, free)

    def col_of(x):
        return int(math.floor((x - ox) / res))

    def row_of(y):
        return (h - 1) - int(math.floor((y - oy) / res))

    def apply(r, c, tool):
        cls = classify(int(img[r, c]), occ, free)
        if tool == TOOL_WALL and cls != "occ":
            img[r, c] = 0
        elif tool == TOOL_ERASE and cls == "occ":
            img[r, c] = 255
        elif tool == TOOL_UNKNOWN and cls != "unknown":
            img[r, c] = ugray

    def for_box(xmin, xmax, ymin, ymax, tool, covered):
        for r in range(max(0, row_of(ymax)), min(h - 1, row_of(ymin)) + 1):
            cy = oy + ((h - 1 - r) + 0.5) * res
            for c in range(max(0, col_of(xmin)), min(w - 1, col_of(xmax)) + 1):
                if covered(ox + (c + 0.5) * res, cy):
                    apply(r, c, tool)

    for tool, shape, pts, width in strokes:
        if shape == SHAPE_RECT:
            x0, x1 = sorted((pts[0][0], pts[1][0]))
            y0, y1 = sorted((pts[0][1], pts[1][1]))
            for_box(x0, x1, y0, y1, tool,
                    lambda cx, cy: x0 - 1e-9 <= cx <= x1 + 1e-9 and y0 - 1e-9 <= cy <= y1 + 1e-9)
            continue
        rad = width / 2.0 + res / 2.0
        segs = [(pts[0], pts[0])] if len(pts) == 1 else list(zip(pts[:-1], pts[1:]))
        for a, b in segs:
            for_box(min(a[0], b[0]) - rad, max(a[0], b[0]) + rad, min(a[1], b[1]) - rad,
                    max(a[1], b[1]) + rad, tool,
                    lambda cx, cy, a=a, b=b: dist_to_segment(cx, cy, a[0], a[1], b[0], b[1]) <= rad + 1e-9)


# ── 工具函数 ───────────────────────────────────────────────────────────

def read_yaml_lines(path):
    return Path(path).read_text().splitlines()


def strip_stamp(text):
    # 比对「底图内容是否恢复」时去掉 last_edited_at：回滚 / 编辑本身会推进它
    return [l for l in text.splitlines() if not l.startswith("last_edited_at:")]


def stamp_sec(lines):
    v = yaml_value(lines, "last_edited_at")
    return int(time.mktime(time.strptime(v, "%Y-%m-%dT%H:%M:%S"))) if v else 0


def yaml_value(lines, key):
    for line in lines:
        if line.startswith(key + ":"):
            return line.split(":", 1)[1].split("#", 1)[0].strip().strip("'\"")
    return None


def geometry(slam_dir):
    lines = read_yaml_lines(slam_dir / "map.yaml")
    ox, oy, _ = [float(v) for v in yaml_value(lines, "origin").strip("[]").split(",")]
    return ({"res": float(yaml_value(lines, "resolution")), "ox": ox, "oy": oy},
            float(yaml_value(lines, "occupied_thresh")), float(yaml_value(lines, "free_thresh")))


def diff_detail(actual, expect, before, geo, strokes):
    """不一致时给出每个差异像素的位置、前后值与到笔画的距离，便于定位规则分歧。"""
    h = actual.shape[0]
    out = []
    for r, c in np.argwhere(actual != expect)[:5]:
        cx = geo["ox"] + (c + 0.5) * geo["res"]
        cy = geo["oy"] + (h - 1 - r + 0.5) * geo["res"]
        tool, shape, pts, width = strokes[0]
        rad = width / 2.0 + geo["res"] / 2.0
        d = min(dist_to_segment(cx, cy, a[0], a[1], b[0], b[1])
                for a, b in ([(pts[0], pts[0])] if len(pts) == 1 else zip(pts[:-1], pts[1:])))
        out.append(f"(r={r},c={c}) before={before[r, c]} backend={actual[r, c]} ref={expect[r, c]} "
                   f"dist-rad={d - rad:.3e}")
    return "; ".join(out)


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def snapshot_mtimes(root):
    out = {}
    if root.exists():
        for p in root.rglob("*"):
            if p.is_file():
                out[str(p)] = p.stat().st_mtime_ns
    return out


class Report:
    def __init__(self):
        self.rows = []

    def check(self, case, desc, ok, detail=""):
        self.rows.append((case, desc, bool(ok), detail))
        print(f"  {'✅' if ok else '❌'} [{case}] {desc}" + (f" — {detail}" if detail and not ok else ""))
        return ok

    @property
    def failed(self):
        return [r for r in self.rows if not r[2]]


# ── 主流程 ─────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="map edit service-level self test")
    parser.add_argument("--map-dir", help="测试地图目录（含 slam/map.yaml|png|pbstream），默认用包内 map_standard")
    parser.add_argument("--domain", type=int, default=87, help="ROS_DOMAIN_ID（默认 87，避免与实机 / 其它测试串扰）")
    parser.add_argument("--keep", action="store_true", help="保留临时目录便于查看")
    args = parser.parse_args()

    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"

    import rclpy
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from geometry_msgs.msg import Point
    from nav_interfaces.msg import GridStroke
    from nav_interfaces.srv import GetMaps, MapManagerCommand, ResetMapOriginByXYYaw, RollbackMap

    mm_share = Path(get_package_share_directory("map_manager"))
    mm_bin = Path(get_package_prefix("map_manager")) / "lib/map_manager/map_manager_node"
    cb_bin = Path(get_package_prefix("command_board")) / "lib/command_board/command_board_node"
    config_yaml = mm_share / "config/config.yaml"
    default_map = yaml_value(read_yaml_lines(config_yaml), "default_map_name") or "map_edit_test_a"
    src_map = Path(args.map_dir) if args.map_dir else mm_share / "test_data/map_standard"
    if not (src_map / "slam/map.pbstream").exists():
        sys.exit(f"test map must contain slam/map.yaml|png|pbstream: {src_map}")

    real_maps = Path.home() / ".dexe/data/maps"
    real_before = snapshot_mtimes(real_maps)
    # map_manager 开 use_default_map 时每次启动 / 加载都会用 yaml-cpp 整份重写这个文件（去注释、去引号），
    # 这是既有行为；这里只核对语义值没被测试改掉
    config_before = {k: yaml_value(read_yaml_lines(config_yaml), k) for k in ("map_folder", "default_map_name")}

    tmp = Path(tempfile.mkdtemp(prefix="map_edit_self_test_"))
    home = tmp / "home"
    maps = home / ".dexe/data/maps"
    cur_name, other_name = default_map, "map_edit_test_b"
    for name in (cur_name, other_name):
        shutil.copytree(src_map, maps / name, ignore=shutil.ignore_patterns("backups", ".map.yaml.lock"))
    cur_slam, other_slam = maps / cur_name / "slam", maps / other_name / "slam"
    cur_yaml0 = (cur_slam / "map.yaml").read_bytes()
    print(f"temp HOME: {home}\ncurrent map: {cur_name}  non-current map: {other_name}\nsource map: {src_map}")

    env = dict(os.environ, HOME=str(home))
    procs = [subprocess.Popen([str(mm_bin), "--ros-args", "-r", "__node:=map_manager"], env=env,
                              stdout=open(tmp / "map_manager.log", "w"), stderr=subprocess.STDOUT),
             subprocess.Popen([str(cb_bin), "--ros-args", "-r", "__node:=command_board"], env=env,
                              stdout=open(tmp / "command_board.log", "w"), stderr=subprocess.STDOUT),
             # command_board 的资产回滚委托 map_asset_manager rollback_assets
             subprocess.Popen([str(Path(get_package_prefix("map_asset_manager")) /
                                   "lib/map_asset_manager/map_asset_manager_node")], env=env,
                              stdout=open(tmp / "map_asset_manager.log", "w"), stderr=subprocess.STDOUT)]
    rep = Report()
    rclpy.init()
    node = rclpy.create_node("map_edit_self_test")
    try:
        cmd = node.create_client(MapManagerCommand, "/map_manager/map_command")
        gm = node.create_client(GetMaps, "/map_manager/get_maps")
        rb = node.create_client(RollbackMap, "/command_board/rollback_map")  # 与部署 launch 的节点名一致
        for cli in (cmd, gm, rb):
            if not cli.wait_for_service(timeout_sec=20.0):
                sys.exit(f"service not available: {cli.srv_name} (see logs in {tmp})")

        def call(cli, req):
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=60.0)
            if fut.result() is None:
                raise RuntimeError(f"call {cli.srv_name} timed out")
            return fut.result()

        def info(name):
            for m in call(gm, GetMaps.Request()).maps:
                if m.name == name:
                    return m
            return None

        def gs(tool, pts, width=0.2, shape=SHAPE_POLYLINE):
            s = GridStroke()
            s.tool, s.shape, s.width_m = tool, shape, float(width)
            s.points = [Point(x=float(x), y=float(y)) for x, y in pts]
            return s

        def edit(name, strokes, rev):
            r = MapManagerCommand.Request()
            r.op, r.map_name, r.expected_revision = MapManagerCommand.Request.OP_EDIT_GRID, name, rev
            r.strokes = [gs(*s) if isinstance(s, tuple) else s for s in strokes]
            return call(cmd, r)

        def as_tuples(strokes):
            return [(s[0], s[4] if len(s) > 4 else SHAPE_POLYLINE, s[1], s[2] if len(s) > 2 else 0.2) for s in strokes]

        geo, occ, free = geometry(other_slam)
        img0 = cv2.imread(str(other_slam / "map.png"), cv2.IMREAD_UNCHANGED)
        h, w = img0.shape
        cx = geo["ox"] + w * geo["res"] / 2.0
        cy = geo["oy"] + h * geo["res"] / 2.0
        print(f"map {w}x{h} res={geo['res']} thresholds occ={occ} free={free} "
              f"(unknown gray = {unknown_gray(occ, free)})")

        # T-1 节点启动即加载当前图：加载不改写 map.yaml，也就不推进底图 last_edited_at
        rep.check("T-1", "loading the current map does not rewrite its map.yaml",
                  (cur_slam / "map.yaml").read_bytes() == cur_yaml0)

        # G-06 显示朝向：只改 map.yaml 的 display_yaw，底图不动
        print("▶ G-06 display yaw")
        rev0 = info(other_name).grid_revision
        png0 = file_hash(other_slam / "map.png")
        r = call(cmd, MapManagerCommand.Request(op=7, map_name=other_name, display_yaw=4.0))
        m = info(other_name)
        rep.check("G-06", "set display_yaw on non-current map", r.success, r.message)
        rep.check("G-06", "get_maps reads back normalized angle",
                  abs(m.display_yaw - (4.0 - 2 * math.pi)) < 1e-5, f"{m.display_yaw}")
        rep.check("G-06", "base map pixels and revision unchanged",
                  file_hash(other_slam / "map.png") == png0 and m.grid_revision == rev0)
        yl = read_yaml_lines(other_slam / "map.yaml")
        rep.check("T-2", "display_yaw change stamps map.yaml last_edited_at = now, read back by get_maps",
                  abs(stamp_sec(yl) - time.time()) <= 5 and m.last_edited_at.sec == stamp_sec(yl),
                  f"{yaml_value(yl, 'last_edited_at')} / {m.last_edited_at.sec}")
        yaml_same = (other_slam / "map.yaml").read_bytes()
        call(cmd, MapManagerCommand.Request(op=7, map_name=other_name, display_yaw=4.0))
        rep.check("T-2", "setting the same display_yaw again does not rewrite map.yaml",
                  (other_slam / "map.yaml").read_bytes() == yaml_same)

        # G-03 指纹不符：拒绝且不动文件
        print("▶ G-03 stale revision")
        wall = [(TOOL_WALL, [(cx - 3.0, cy), (cx + 3.0, cy)], 0.2)]
        yaml0 = (other_slam / "map.yaml").read_bytes()
        r = edit(other_name, wall, "deadbeefdeadbeef")
        rep.check("G-03", "rejected with E_MAP_GRID_REVISION_MISMATCH",
                  not r.success and r.error_code == E_MAP_GRID_REVISION_MISMATCH, f"{r.error_code} {r.message}")
        rep.check("G-03", "files untouched",
                  file_hash(other_slam / "map.png") == png0 and (other_slam / "map.yaml").read_bytes() == yaml0)

        # G-01 / G-07 画墙：像素与参考实现逐像素一致、三件套备份、尺寸与 origin 不变、面积重算
        print("▶ G-01 / G-07 wall on non-current map")
        before = {f: (other_slam / f).read_bytes() for f in ("map.pbstream", "map.png", "map.yaml")}
        r = edit(other_name, wall, rev0)
        rep.check("G-01", "edit succeeded, not current map",
                  r.success and not r.requires_reload and r.backup_ts, f"{r.error_code} {r.message}")
        backup1 = r.backup_ts
        bdir = other_slam / "backups" / f"backup.{backup1}"
        rep.check("G-01", "backup holds the 3 pre-edit files",
                  all((bdir / f).exists() and (bdir / f).read_bytes() == b for f, b in before.items()))
        img1 = cv2.imread(str(other_slam / "map.png"), cv2.IMREAD_UNCHANGED)
        expect = img0.copy()
        rasterize(expect, {**geo}, occ, free, as_tuples(wall))
        rep.check("G-01", "pixels equal reference rasterizer", np.array_equal(img1, expect),
                  f"{int((img1 != expect).sum())} pixels differ: " + diff_detail(img1, expect, img0, geo, as_tuples(wall)))
        rep.check("G-01", "some pixels actually changed", int((img1 != img0).sum()) > 0)
        rep.check("G-07", "image size and origin unchanged",
                  img1.shape == img0.shape and geometry(other_slam)[0] == geo)
        lines1 = read_yaml_lines(other_slam / "map.yaml")
        eff = float(yaml_value(lines1, "effective_area_m2"))
        expect_eff = int((img1 > int(255 * (1 - free))).sum()) * geo["res"] ** 2
        rep.check("G-01", "effective_area_m2 matches png", abs(eff - expect_eff) < max(1e-3, expect_eff * 1e-5),
                  f"{eff} vs {expect_eff}")
        kept = [l for l in read_yaml_lines(bdir / "map.yaml")
                if not l.startswith(("total_area_m2", "effective_area_m2", "last_edited_at"))]
        rep.check("G-01", "other map.yaml lines kept (incl. display_yaw / created_at)",
                  all(l in lines1 for l in kept))
        rep.check("T-3", "grid edit stamps last_edited_at = now",
                  abs(stamp_sec(lines1) - time.time()) <= 5, str(yaml_value(lines1, "last_edited_at")))
        rev1 = r.revision
        rep.check("G-01", "new revision returned and visible", rev1 and rev1 != rev0 and info(other_name).grid_revision == rev1)

        # G-02 擦除：只把障碍变自由，未知保持未知
        print("▶ G-02 erase keeps unknown")
        occ_px = img1 < 255 * (1 - occ)
        unk_px = (img1 >= 255 * (1 - occ)) & (img1 <= 255 * (1 - free))
        k = 9
        found = None
        for rr in range(k, h - k, 3):
            for cc in range(k, w - k, 3):
                win = (slice(rr - k // 2, rr + k // 2 + 1), slice(cc - k // 2, cc + k // 2 + 1))
                if occ_px[win].any() and unk_px[win].any():
                    found = (rr, cc)
                    break
            if found:
                break
        if rep.check("G-02", "found a window with both obstacle and unknown pixels", found is not None):
            rr, cc = found
            ex = geo["ox"] + (cc + 0.5) * geo["res"]
            ey = geo["oy"] + (h - 1 - rr + 0.5) * geo["res"]
            erase = [(TOOL_ERASE, [(ex, ey)], k * geo["res"])]
            r = edit(other_name, erase, rev1)
            img2 = cv2.imread(str(other_slam / "map.png"), cv2.IMREAD_UNCHANGED)
            expect = img1.copy()
            rasterize(expect, geo, occ, free, as_tuples(erase))
            rep.check("G-02", "erase succeeded", r.success, r.message)
            rep.check("G-02", "pixels equal reference rasterizer", np.array_equal(img2, expect),
                      f"{int((img2 != expect).sum())} pixels differ: " +
                      diff_detail(img2, expect, img1, geo, as_tuples(erase)) + f"; stroke={erase}")
            changed = img2 != img1
            rep.check("G-02", "only obstacle pixels changed, all to 255",
                      changed.any() and occ_px[changed].all() and (img2[changed] == 255).all())
            rep.check("G-02", "unknown pixels untouched", np.array_equal(img2[unk_px], img1[unk_px]))
            rev1 = r.revision or rev1

        # 无像素变化：成功但不备份
        print("▶ no-op edit")
        n_backups = len(list((other_slam / "backups").glob("backup.*")))
        far = (geo["ox"] - 100.0, geo["oy"] - 100.0)
        r = edit(other_name, [(TOOL_WALL, [far], 0.2)], rev1)
        rep.check("NOOP", "success without backup",
                  r.success and not r.backup_ts and len(list((other_slam / "backups").glob("backup.*"))) == n_backups,
                  r.message)

        # 参数非法
        print("▶ invalid parameters")
        r = edit(other_name, [(7, [(cx, cy)], 0.2)], "")
        rep.check("INVALID", "bad tool -> E_MAP_EDIT_INVALID", r.error_code == E_MAP_EDIT_INVALID, r.message)
        r = edit(other_name, [(TOOL_WALL, [(cx, cy)], 0.0)], "")
        rep.check("INVALID", "zero width -> E_MAP_EDIT_INVALID", r.error_code == E_MAP_EDIT_INVALID, r.message)
        r = edit("map_edit_test_missing", wall, "")
        rep.check("INVALID", "missing map -> E_MAP_NOT_FOUND", r.error_code == E_MAP_NOT_FOUND, r.message)

        # G-04 编辑当前图：需要重新加载
        print("▶ G-04 edit current map")
        cur_png0 = (cur_slam / "map.png").read_bytes()
        r = edit(cur_name, wall, info(cur_name).grid_revision)
        rep.check("G-04", "edit current map -> requires_reload", r.success and r.requires_reload, r.message)
        cur_backup = r.backup_ts

        # VERS 版本列表随 get_maps 的 MapInfo.versions 返回：底图编辑前的状态以 grid_edit 来源出现
        print("▶ VERS map versions (MapInfo.versions)")
        versions = info(other_name).versions
        by_ts = {v.version_ts: v for v in versions}
        rep.check("VERS", "get_maps lists the edit backup as grid_edit",
                  backup1 in by_ts and by_ts[backup1].source == "grid_edit" and by_ts[backup1].complete)
        rep.check("VERS", "saved_at parsed", backup1 in by_ts and by_ts[backup1].saved_at.sec > 0)
        rep.check("VERS", "newest first", [v.version_ts for v in versions] == sorted(by_ts, reverse=True))

        # G-08 map_manager 回滚任意地图（非当前图）：恢复到编辑前，回滚前自动快照，可撤销
        print("▶ G-08 map_manager rollback of a non-current map")
        edited = {f: (other_slam / f).read_bytes() for f in ("map.pbstream", "map.png", "map.yaml")}
        r = call(cmd, MapManagerCommand.Request(op=8, map_name=other_name, version_ts=backup1))
        rep.check("G-08", "op 8 succeeded, no reload needed for non-current map",
                  r.success and not r.requires_reload and r.backup_ts, f"{r.error_code} {r.message}")
        rep.check("G-08", "base map restored to pre-edit",
                  all((other_slam / f).read_bytes() == before[f] for f in ("map.pbstream", "map.png")) and
                  strip_stamp((other_slam / "map.yaml").read_text()) == strip_stamp(before["map.yaml"].decode()))
        set_at = stamp_sec(read_yaml_lines(other_slam / "map.yaml"))
        rep.check("T-4", "rollback counts as an edit: last_edited_at = now, not the version's value",
                  abs(set_at - time.time()) <= 5 and info(other_name).last_edited_at.sec == set_at)
        snap = r.backup_ts
        rep.check("G-08", "pre-rollback snapshot listed as rollback",
                  any(v.version_ts == snap and v.source == "rollback" for v in info(other_name).versions))
        r = call(cmd, MapManagerCommand.Request(op=8, map_name=other_name, version_ts=snap))
        rep.check("UNDO", "rolling back to the snapshot undoes the rollback",
                  r.success and all((other_slam / f).read_bytes() == edited[f] for f in ("map.pbstream", "map.png")) and
                  strip_stamp((other_slam / "map.yaml").read_text()) == strip_stamp(edited["map.yaml"].decode()),
                  r.message)
        r = call(cmd, MapManagerCommand.Request(op=8, map_name=other_name, version_ts="../x"))
        rep.check("G-08", "path-like version_ts rejected", r.error_code == E_MAP_VERSION_NOT_FOUND, r.message)
        r = call(cmd, MapManagerCommand.Request(op=8, map_name=other_name, version_ts="20990101-000000"))
        rep.check("G-08", "missing version rejected", r.error_code == E_MAP_VERSION_NOT_FOUND, r.message)

        # G-09 command_board 回滚当前地图：只回滚底图（资产 none）→ 恢复 + 重新下发，资产不动
        print("▶ G-09 command_board rollback of the current map")
        assets_live = cur_slam.parent / "assets" / "map_assets.json"
        assets_ring = cur_slam.parent / "assets" / "backups"
        assets_ring.mkdir(parents=True, exist_ok=True)

        def assets_with(point_name):
            return json.dumps({"points": [{"name": point_name, "type": "STATION", "tolerance": 0.0,
                                           "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}}]})

        def live_points():
            time.sleep(1.0)  # 资产节点按 epoch 重载后会规范化重写文件，按内容判断
            return [p["name"] for p in json.loads(assets_live.read_text()).get("points", [])]

        (assets_ring / "map_assets.20200101-000000.json").write_text(assets_with("FROM_BACKUP"))
        assets_live.write_text(assets_with("FROM_LIVE"))
        r = call(rb, RollbackMap.Request(map_version_ts=cur_backup, assets_version_ts="none"))
        rep.check("G-09", "base-only rollback of current map + redeploy",
                  r.success and "redeploy ok" in r.message and r.map_version_ts == cur_backup, r.message)
        rep.check("G-09", "current base map restored", (cur_slam / "map.png").read_bytes() == cur_png0)
        rep.check("G-09", "assets untouched when assets_version_ts = none", live_points() == ["FROM_LIVE"])
        png_now = (cur_slam / "map.png").read_bytes()
        base_t0 = info(cur_name).last_edited_at.sec
        r = call(rb, RollbackMap.Request(map_version_ts="none", assets_version_ts=""))
        rep.check("G-09", "assets-only rollback (latest assets version, delegated to map_asset_manager), base untouched",
                  r.success and live_points() == ["FROM_BACKUP"] and (cur_slam / "map.png").read_bytes() == png_now,
                  r.message)
        rep.check("T-5", "asset-only rollback leaves the base map last_edited_at untouched",
                  base_t0 > 0 and info(cur_name).last_edited_at.sec == base_t0,
                  f"{base_t0} -> {info(cur_name).last_edited_at.sec}")
        r = call(rb, RollbackMap.Request(map_version_ts="none", assets_version_ts="none"))
        rep.check("G-09", "both none -> nothing to roll back", not r.success, r.message)

        # G-10 command_board 回滚任意地图：非当前图只恢复文件、不重新下发
        print("▶ G-10 command_board rollback of a non-current map")
        loads_before = (tmp / "map_manager.log").read_text().count("map_command op=1")
        target_v = info(other_name).versions[0].version_ts
        target_dir = other_slam / "backups" / f"backup.{target_v}"
        r = call(rb, RollbackMap.Request(map_name=other_name, map_version_ts=target_v, assets_version_ts="none"))
        rep.check("G-10", "rollback_map on a non-current map succeeds without redeploy",
                  r.success and "files restored only" in r.message, r.message)
        rep.check("G-10", "base map equals the chosen version",
                  (other_slam / "map.png").read_bytes() == (target_dir / "map.png").read_bytes())
        time.sleep(0.5)
        rep.check("G-10", "no map reload was triggered",
                  (tmp / "map_manager.log").read_text().count("map_command op=1") == loads_before)
        r = call(rb, RollbackMap.Request(map_name="../x"))
        rep.check("G-10", "path-like map_name rejected", not r.success, r.message)

        # G-12 command_board 按坐标重设非当前地图原点：资产走离线工具，不需要资产节点在线
        print("▶ G-12 reset origin of a non-current map without map_asset_manager")
        if shutil.which("ros2") and subprocess.run(
                ["ros2", "pkg", "executables", "cartographer_ros"], capture_output=True, text=True
        ).stdout.find("reset_map_origin_tool") >= 0:
            xr = node.create_client(ResetMapOriginByXYYaw, "/command_board/reset_map_origin_by_xyyaw")
            assets_file = other_slam.parent / "assets" / "map_assets.json"
            # 测试地图自带的资产是空的，换成资产包的标准样本（点 / 区域 / 路网 / 房间都有）
            shutil.copy(Path(get_package_share_directory("map_asset_manager")) /
                        "test_data/map_assets_standard.json", assets_file)
            pts0 = {p["name"]: p["pose"] for p in json.loads(assets_file.read_text())["points"]}
            X, Y, T = 0.5, -0.3, 0.2
            r = call(xr, ResetMapOriginByXYYaw.Request(map_name=other_name, x=X, y=Y, yaw=T))
            rep.check("G-12", "reset_map_origin_by_xyyaw succeeds with no asset node", r.success, r.message)
            pts1 = {p["name"]: p["pose"] for p in json.loads(assets_file.read_text())["points"]}
            c, s_ = math.cos(T), math.sin(T)
            worst = 0.0
            for name, q in pts0.items():
                dx, dy = q["x"] - X, q["y"] - Y
                worst = max(worst, abs(pts1[name]["x"] - (c * dx + s_ * dy)),
                            abs(pts1[name]["y"] - (-s_ * dx + c * dy)))
            rep.check("G-12", "asset points moved to the new origin frame", bool(pts0) and worst < 1e-6,
                      f"max error {worst:.2e}")
            ring = sorted((other_slam.parent / "assets" / "backups").glob("map_assets.*.json"))
            rep.check("G-12", "asset version saved before transform",
                      bool(ring) and {p["name"]: p["pose"] for p in json.loads(ring[-1].read_text())["points"]} == pts0)
            v = info(other_name).versions[0]
            rep.check("G-12", "base map version recorded with source reset_origin", v.source == "reset_origin",
                      f"{v.version_ts} {v.source}")
            yl = read_yaml_lines(other_slam / "map.yaml")
            rep.check("T-6", "reset origin (SLAM tool, merged map.yaml write) stamps last_edited_at = now",
                      abs(stamp_sec(yl) - time.time()) <= 10, str(yaml_value(yl, "last_edited_at")))
            # G-13 资产工具失败（资产文件损坏）→ 底图回滚到重设前，两者保持一致
            print("▶ G-13 reset origin rolls the base map back when the asset transform fails")
            png_before = (other_slam / "map.png").read_bytes()
            yaml_before = (other_slam / "map.yaml").read_text()
            assets_file.write_text("{ not json")
            r = call(xr, ResetMapOriginByXYYaw.Request(map_name=other_name, x=0.3, y=0.1, yaw=0.1))
            rep.check("G-13", "reset_map_origin_by_xyyaw reports the asset failure", not r.success and
                      "transform_assets failed" in r.message and "rolled back" in r.message, r.message)
            rep.check("G-13", "base map restored to the state before the reset",
                      (other_slam / "map.png").read_bytes() == png_before and
                      strip_stamp((other_slam / "map.yaml").read_text()) == strip_stamp(yaml_before))
        else:
            print("  SKIP: cartographer_ros reset_map_origin_tool not in this environment (source the SLAM install)")

        # G-11 旧 op 行为不变、新响应字段为空
        print("▶ G-11 existing ops")
        r = call(cmd, MapManagerCommand.Request(op=3, map_name=other_name, new_name=other_name + "_renamed"))
        ok = r.success and not r.revision and not r.backup_ts and not r.requires_reload
        r2 = call(cmd, MapManagerCommand.Request(op=3, map_name=other_name + "_renamed", new_name=other_name))
        rep.check("G-11", "rename works and new response fields stay empty", ok and r2.success, r.message)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for p in procs:
            p.send_signal(signal.SIGINT)
        deadline = time.time() + 10
        for p in procs:
            try:
                p.wait(timeout=max(0.1, deadline - time.time()))
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()

    print("▶ isolation")
    rep.check("ISO", "test nodes exited", all(p.poll() is not None for p in procs))
    rep.check("ISO", "real ~/.dexe/data/maps untouched", snapshot_mtimes(real_maps) == real_before)
    config_after = {k: yaml_value(read_yaml_lines(config_yaml), k) for k in ("map_folder", "default_map_name")}
    rep.check("ISO", "install config values unchanged (map_folder / default_map_name)", config_after == config_before,
              f"{config_before} -> {config_after}")

    total, failed = len(rep.rows), rep.failed
    print(f"\n== {total - len(failed)}/{total} passed ==")
    if args.keep or failed:
        print(f"temp dir kept: {tmp}  (map_manager.log / command_board.log inside)")
    else:
        shutil.rmtree(tmp, ignore_errors=True)
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
