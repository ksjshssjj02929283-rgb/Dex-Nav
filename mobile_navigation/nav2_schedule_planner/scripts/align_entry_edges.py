#!/usr/bin/env python3
"""把目标点位入边的入射方向对齐到点位朝向，减少到点后的 phase3 原地旋转。

问题：导航到点位 P 时，入边 Q->P 的行进方向和 P 配置的 yaw 常差几度（画路网时手点的误差），
path_follower 到点后就得做 phase3 原地旋转把这几度补上。

做法：把入边起点 Q 挪到「过 P、方向为 P.yaw 的直线」上的垂足 Q'：

    u  = (cos P.yaw, sin P.yaw)
    Q' = P + ((Q - P) . u) * u

Q' -> P 的方向就与 P.yaw 一致（倒车入边则与 P.yaw + pi 一致——垂足与行进方向无关，同一个公式），
phase3 的剩余误差落进 reach_angle 就直接跳过。挪动距离 = 边长 * |sin(偏角)|。

⚠ Q 被多条边共享时，挪 Q 会连带改变 Q 上其他边的方向和长度（LINE 边几何直接取端点坐标），
   干跑输出会逐条列出这些连带影响，看完再 --apply。
⚠ Q 挪动超过 --warn-shift（默认 0.10m）打 WARNING，但照挪。
⚠ 只改 Q 的 x/y，Q 的 yaw 不动。

用法（先 source 车端环境；离线处理本地 json 时只需 python3）：

  ./align_entry_edges.py cf4 cf2 carry                 # 干跑：只打印计划，不写文件
  ./align_entry_edges.py cf4 cf2 carry --apply         # 备份 -> 原子写回 -> 调 reload_data
  ./align_entry_edges.py cf4:carry4_cf4                # 入边不唯一时用 NAME:EDGE_ID 指定
  ./align_entry_edges.py cf4 --map-assets ./map_assets.json   # 离线处理指定文件

--apply 的落盘顺序：
  1. 先把原文件拷到 <assets 目录>/backups/align_entry.<时间戳>.json
     （前缀不是 map_assets.，不会被 map_asset_manager 的 50 份 ring 轮换删掉）；备份失败整体中止；
  2. 写临时文件再 os.replace 原子替换；
  3. 调 /map_asset_manager/reload_data。**必须 reload**：map_asset_manager 以内存为准、每次编辑都
     auto-save 整个文件，不 reload 的话下一次任何编辑都会把这次改动覆盖掉。

回滚：cp <备份> <原文件> 后再调一次 reload_data（脚本会把命令打出来）。
"""

import argparse
import copy
import json
import math
import os
import shutil
import sys
import tempfile
import time
from pathlib import Path

MAP_STATE_TOPIC = '/map_manager/map_state'
RELOAD_SERVICE = '/map_asset_manager/reload_data'

ALIGNED_EPS_DEG = 0.05       # 偏角小于它视为已对齐
SAME_FOOT_EPS = 1e-3         # 两个目标共享 Q 时，垂足相差小于它视为同一个解
CP_MATCH_EPS = 1e-3          # 控制点与 Q 旧坐标的匹配容差


# --------------------------------------------------------------------------- #
# 纯计算（不依赖 ROS）
# --------------------------------------------------------------------------- #
def norm_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def heading(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def foot_on_line(q, p, yaw):
    """Q 在「过 P、方向 yaw」的直线上的垂足。"""
    ux, uy = math.cos(yaw), math.sin(yaw)
    s = (q[0] - p[0]) * ux + (q[1] - p[1]) * uy
    return (p[0] + s * ux, p[1] + s * uy)


def ep_key(ep):
    """端点键：(type, id)。nav 点和 junction 可能同名，必须带 type 区分。"""
    return (str(ep.get('type', 'nav')), str(ep.get('id')))


class Assets:
    """map_assets.json 的读写视图：按 (type,id) 取/改端点坐标。直接改 self.data 里的原字典。"""

    def __init__(self, data):
        self.data = data
        self.points = {str(p.get('name')): p for p in data.get('points') or []}
        rg = data.get('route_graph') or {}
        self.junctions = {str(j.get('id')): j for j in rg.get('junctions') or []}
        self.edges = rg.get('edges') or []

    def coord(self, key):
        typ, name = key
        if typ == 'junction':
            j = self.junctions.get(name)
            return None if j is None else (float(j.get('x', 0.0)), float(j.get('y', 0.0)))
        p = self.points.get(name)
        if p is None:
            return None
        pose = p.get('pose') or {}
        return (float(pose.get('x', 0.0)), float(pose.get('y', 0.0)))

    def yaw(self, name):
        return float((self.points[name].get('pose') or {}).get('yaw', 0.0))

    def set_coord(self, key, xy):
        typ, name = key
        if typ == 'junction':
            self.junctions[name]['x'] = xy[0]
            self.junctions[name]['y'] = xy[1]
        else:
            self.points[name].setdefault('pose', {})
            self.points[name]['pose']['x'] = xy[0]
            self.points[name]['pose']['y'] = xy[1]

    def edges_at(self, key):
        return [e for e in self.edges
                if ep_key(e.get('from') or {}) == key or ep_key(e.get('to') or {}) == key]

    def edge_dir_len(self, e):
        """边按存储方向（from->to）的几何方向与长度；BEZIER 取首尾控制点连线，仅作报告用。"""
        a = self.coord(ep_key(e['from']))
        b = self.coord(ep_key(e['to']))
        if a is None or b is None:
            return None, None
        return heading(a, b), math.dist(a, b)


def is_line(e):
    return str(e.get('edge_type', 'LINE')).upper() != 'BEZIER'


def incoming_edges(assets, name):
    """终点为 nav 点 name 的边。规则同 route_graph_manager.cpp:findNearestEdgeTo：
    to==P 正向走；或 bidirectional 且 from==P 反向走。返回 [(edge, reversed)]。"""
    key = ('nav', name)
    out = []
    for e in assets.edges:
        if ep_key(e.get('to') or {}) == key:
            out.append((e, False))
        elif e.get('bidirectional') and ep_key(e.get('from') or {}) == key:
            out.append((e, True))
    return out


def plan(assets, targets, max_angle_deg, warn_shift):
    """算出每个目标的挪点方案；不改 assets。

    返回 (moves, skipped, warnings)：
      moves:   [{target, edge, reversed, start_key, old, new, shift, delta}]
      skipped: [(target, 原因)]
    """
    moves, skipped, warnings = [], [], []
    target_names = {t for t, _ in targets}

    for name, edge_id in targets:
        if name not in assets.points:
            skipped.append((name, '点位不存在'))
            continue

        cands = incoming_edges(assets, name)
        if edge_id is not None:
            cands = [(e, r) for e, r in cands if e.get('id') == edge_id]
            if not cands:
                skipped.append((name, f'指定的入边 {edge_id} 不存在或终点不是 {name}'))
                continue
        if not cands:
            skipped.append((name, '没有入边'))
            continue
        if len(cands) > 1:
            ids = ', '.join(str(e.get('id')) for e, _ in cands)
            skipped.append((name, f'入边 {len(cands)} 条: {ids}，用 {name}:<edge_id> 指定'))
            continue

        e, rev = cands[0]
        if not is_line(e):
            skipped.append((name, f'入边 {e.get("id")} 是 BEZIER，末端切向由控制点决定，挪起点无效'))
            continue

        start_key = ep_key(e['to'] if rev else e['from'])
        if start_key == ('nav', start_key[1]) and start_key[1] in target_names:
            skipped.append((name, f'起点 {start_key[1]} 本身也在目标列表里，不能挪一个要精确停靠的点'))
            continue

        q = assets.coord(start_key)
        p = assets.coord(('nav', name))
        if q is None:
            skipped.append((name, f'起点 {start_key[1]} 坐标缺失'))
            continue
        yaw = assets.yaw(name)

        travel = heading(q, p)
        head = travel + math.pi if e.get('backward') else travel
        delta = norm_angle(head - yaw)
        if abs(math.degrees(delta)) < ALIGNED_EPS_DEG:
            skipped.append((name, f'已对齐（偏角 {math.degrees(delta):+.3f}°）'))
            continue
        if abs(math.degrees(delta)) > max_angle_deg:
            skipped.append((name, f'偏角 {math.degrees(delta):+.2f}° 超过 --max-angle {max_angle_deg}°，'
                                  f'不像画图误差，不处理'))
            continue

        new = foot_on_line(q, p, yaw)
        shift = math.dist(q, new)
        moves.append(dict(target=name, edge=e, reversed=rev, start_key=start_key,
                          old=q, new=new, shift=shift, delta=delta))

    # 两个目标共享同一个 Q：垂足一致可以合并，不一致两个都放弃
    by_start = {}
    for m in moves:
        by_start.setdefault(m['start_key'], []).append(m)
    kept = []
    for key, ms in by_start.items():
        if len(ms) > 1 and any(math.dist(ms[0]['new'], m['new']) > SAME_FOOT_EPS for m in ms[1:]):
            names = ', '.join(m['target'] for m in ms)
            for m in ms:
                skipped.append((m['target'], f'与 {names} 共享起点 {key[1]}，各自要求的垂足不一致'))
            continue
        kept.extend(ms)

    for m in kept:
        if m['shift'] > warn_shift:
            warnings.append(f"[{m['target']}] 起点 {m['start_key'][1]} 挪动 {m['shift']:.3f}m "
                            f"> --warn-shift {warn_shift:.2f}m")
    return kept, skipped, warnings


def apply_moves(assets, moves):
    """就地修改 assets：挪起点坐标，并同步 LINE 边上等于旧坐标的显式首/末控制点。"""
    for m in moves:
        key, old, new = m['start_key'], m['old'], m['new']
        assets.set_coord(key, new)
        for e in assets.edges_at(key):
            cps = e.get('control_points')
            if not cps or not is_line(e):
                continue
            for i in (0, len(cps) - 1):
                cp = cps[i]
                xy = (float(cp[0]), float(cp[1]))
                if math.dist(xy, old) < CP_MATCH_EPS:
                    cps[i] = [new[0], new[1]]


def entry_delta(assets, name, edge, rev):
    e_from = ep_key(edge['to'] if rev else edge['from'])
    q = assets.coord(e_from)
    p = assets.coord(('nav', name))
    travel = heading(q, p)
    head = travel + math.pi if edge.get('backward') else travel
    return norm_angle(head - assets.yaw(name))


def report(before, after, moves, skipped, warnings):
    """before/after 是改动前后的 Assets（after 已应用 moves）。"""
    for m in moves:
        e, key = m['edge'], m['start_key']
        a_edge = next(x for x in after.edges if x.get('id') == e.get('id'))
        d_after = entry_delta(after, m['target'], a_edge, m['reversed'])
        q_from, q_to = (e['to'], e['from']) if m['reversed'] else (e['from'], e['to'])
        mode = ('backward' if e.get('backward') else 'forward') + (', 反向走双向边' if m['reversed'] else '')
        print(f"[{m['target']}] 入边 {e.get('id')} ({q_from['id']}→{q_to['id']}, {mode})  "
              f"偏角 {math.degrees(m['delta']):+.2f}° → {math.degrees(d_after):+.3f}°")
        flag = '  WARNING' if any(w.startswith(f"[{m['target']}]") for w in warnings) else ''
        print(f"      {key[1]}: ({m['old'][0]:.3f},{m['old'][1]:.3f}) → "
              f"({m['new'][0]:.3f},{m['new'][1]:.3f})  shift {m['shift']:.3f}m{flag}")
        others = [x for x in before.edges_at(key) if x.get('id') != e.get('id')]
        if others:
            print("      连带影响（Q 上其他边，按存储方向 from→to）:")
        for x in others:
            d0, l0 = before.edge_dir_len(x)
            x_after = next(y for y in after.edges if y.get('id') == x.get('id'))
            d1, l1 = after.edge_dir_len(x_after)
            if d0 is None or d1 is None:
                continue
            tag = '' if is_line(x) else '  (BEZIER：只动了端点，曲线形状由控制点决定)'
            print(f"        {x.get('id'):<18} dir {math.degrees(d0):+8.2f}° → {math.degrees(d1):+8.2f}° "
                  f"(Δ{math.degrees(norm_angle(d1 - d0)):+.2f}°)  len {l0:.3f} → {l1:.3f}{tag}")
    for name, why in skipped:
        print(f"跳过: {name}（{why}）")
    for w in warnings:
        print(f"WARNING: {w}")


# --------------------------------------------------------------------------- #
# 文件 / ROS
# --------------------------------------------------------------------------- #
def parse_targets(items):
    out = []
    for s in items:
        name, _, edge_id = s.partition(':')
        out.append((name, edge_id or None))
    return out


def resolve_assets_path(args):
    """--map-assets 优先；否则读 /map_manager/map_state，目录版式同 graph_nav.assets_path。"""
    if args.map_assets:
        return Path(args.map_assets).expanduser()

    import rclpy
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                           QoSReliabilityPolicy)
    from nav_interfaces.msg import MapState

    rclpy.init()
    node = rclpy.create_node('align_entry_edges')
    got = []
    qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(MapState, MAP_STATE_TOPIC, got.append, qos)
    end = time.monotonic() + args.map_timeout
    while not got and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    if not got:
        raise RuntimeError(f"{args.map_timeout}s 内没收到 {MAP_STATE_TOPIC}，用 --map-assets 直接指定")

    map_dir = Path(got[-1].map_folder) / got[-1].map_name
    for cand in (map_dir / 'assets' / 'map_assets.json', map_dir / 'map_assets.json'):
        if cand.is_file():
            return cand
    raise RuntimeError(f"{map_dir} 下没有 map_assets.json")


def backup(path):
    bdir = path.parent / 'backups'
    bdir.mkdir(parents=True, exist_ok=True)
    dst = bdir / f"align_entry.{time.strftime('%Y%m%d-%H%M%S')}.json"
    shutil.copy2(path, dst)
    if dst.stat().st_size != path.stat().st_size:
        raise RuntimeError(f"备份大小不一致: {dst}")
    return dst


def atomic_write_json(path, data):
    fd, tmp = tempfile.mkstemp(prefix='.map_assets.', suffix='.tmp', dir=str(path.parent))
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
            f.write('\n')
            f.flush()
            os.fsync(f.fileno())
        shutil.copymode(path, tmp)
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise


def call_reload(timeout):
    import rclpy
    from std_srvs.srv import Trigger

    rclpy.init()
    node = rclpy.create_node('align_entry_edges_reload')
    try:
        cli = node.create_client(Trigger, RELOAD_SERVICE)
        if not cli.wait_for_service(timeout_sec=timeout):
            return False, f'{RELOAD_SERVICE} 不可用'
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        if fut.result() is None:
            return False, f'{RELOAD_SERVICE} 超时'
        return fut.result().success, fut.result().message
    finally:
        node.destroy_node()
        rclpy.shutdown()


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('targets', nargs='+', metavar='NAME[:EDGE_ID]', help='目标点位名')
    ap.add_argument('--apply', action='store_true', help='备份后写回并 reload（默认只干跑）')
    ap.add_argument('--map-assets', help='直接指定 map_assets.json，不读 map_state 话题')
    ap.add_argument('--max-angle', type=float, default=15.0,
                    help='偏角超过它不处理（度），默认 15')
    ap.add_argument('--warn-shift', type=float, default=0.10,
                    help='起点挪动超过它打 WARNING（照挪），默认 0.10m')
    ap.add_argument('--no-reload', action='store_true', help='--apply 后不调 reload_data')
    ap.add_argument('--map-timeout', type=float, default=5.0)
    ap.add_argument('--reload-timeout', type=float, default=5.0)
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    path = resolve_assets_path(args)
    with open(path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    print(f"路网文件: {path}")

    before = Assets(copy.deepcopy(data))
    after = Assets(data)
    moves, skipped, warnings = plan(before, parse_targets(args.targets),
                                    args.max_angle, args.warn_shift)
    # plan 在 before 上算，edge 引用也指向 before；apply 到 after 需要换成 after 自己的对象
    after_moves = []
    for m in moves:
        mm = dict(m)
        mm['edge'] = next(x for x in after.edges if x.get('id') == m['edge'].get('id'))
        after_moves.append(mm)
    apply_moves(after, after_moves)
    report(before, after, moves, skipped, warnings)

    if not moves:
        print("没有需要挪动的起点。")
        return 0
    if not args.apply:
        print(f"\n干跑：{len(moves)} 个起点待挪动，确认后加 --apply 写回。")
        return 0

    bak = backup(path)
    print(f"\n已备份: {bak}")
    atomic_write_json(path, data)
    print(f"已写回: {path}")
    if args.no_reload:
        print(f"未 reload（--no-reload）。⚠ map_asset_manager 下一次编辑会用内存数据覆盖本次改动，"
              f"尽快调 {RELOAD_SERVICE}。")
    else:
        ok, msg = call_reload(args.reload_timeout)
        print(f"reload_data: {'OK' if ok else '失败'} — {msg}")
        if not ok:
            print(f"⚠ 文件已改但节点未重载，手动调: ros2 service call {RELOAD_SERVICE} std_srvs/srv/Trigger")
    print(f"回滚: cp '{bak}' '{path}' && ros2 service call {RELOAD_SERVICE} std_srvs/srv/Trigger")
    return 0


if __name__ == '__main__':
    sys.exit(main())
