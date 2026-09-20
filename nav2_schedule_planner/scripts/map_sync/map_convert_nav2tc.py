#!/usr/bin/env python3
"""
将 Nav2 风格的导航地图转换为 traffic_control 项目可用的路网/地图格式。

参照 data/<zoneId>/AGV_20250827.* 的结构，把源目录下的：
  - map_assets.json  -> <name>.graph.json   路网（NetworkX node-link 格式，edges="edges"）
  - map.yaml         -> <name>.yaml.json    地图元信息（MapFile 风格：origin/resolution/width/height...）
  - map.png          -> <name>.png           底图（直接拷贝）
输出到与源目录同级的位置。

例如源目录为
  data/006962e2-c5a9-422b-a697-0a145cfe3001/20260711_xmk/{map_assets.json,map.png,map.yaml}
则产物为
  data/006962e2-c5a9-422b-a697-0a145cfe3001/20260711_xmk.{graph.json,yaml.json,png}
其中 <name> 默认取源目录名，需与 Zone.graphName / Zone.mapName 保持一致
（见 path_planning/map/map.py:get_graph 与 TcShop/layout/map_info.py 的读取路径）。

用法:
  python tools/map_convert_nav2tc.py [SRC_DIR] [-n NAME] [-z ZONE_ID] [-w WORKSPACE_ID] [-f]
  # SRC_DIR   默认 data/006962e2-c5a9-422b-a697-0a145cfe3001/20260711_xmk
  # NAME      默认 SRC_DIR 的目录名
  # ZONE_ID   默认 SRC_DIR 的父目录名
  # -f        覆盖已存在的输出文件

依赖 networkx（项目 requirements 自带）；numpy 可选（用于更精确的曲线长度，缺失时回退纯 Python）。
"""

import argparse
import hashlib
import json
import math
import shutil
import sys
import uuid
from collections import defaultdict
from pathlib import Path

try:
    import networkx as nx
except ImportError:  # pragma: no cover
    sys.exit("缺少 networkx，请在项目虚拟环境中运行：pip install networkx")

# --------------------------------------------------------------------------- #
# 与 path_planning/map/map.py 保持一致的字段定义
# --------------------------------------------------------------------------- #
No = {'M': 0, 'L': 1, 'S': 2, 'P': 3, 'C': 4, 'O': 9, 'A': 5, 'W': 6}
NODE_TYPE = {
    'MESH': 'M',
    'NORMAL': 'L',
    'STATION': 'S',
    'PARK': 'P',
    'CHARGE': 'C',
    'DOCK': 'A',
    'ELEVATOR': 'W'}
EDGE_TYPES = {'NORMAL', 'BEZIER', 'CUBIC_BEZIER'}

# Nav2 点位 type -> 交管节点类型
POINT_TYPE_MAP = {
    'NORMAL': 'NORMAL',
    'STATION': 'STATION',
    'CHARGING': 'CHARGE',
    'DOCK': 'DOCK',
    'PARKING': 'BAY',
    'BAY': 'BAY',
    'SHELF': 'SHELF',
    'RACK': 'SHELF',
    'ELEVATOR': 'ELEVATOR',
    'LIFT': 'ELEVATOR',
    'MESH': 'MESH',
}
DEFAULT_POINT_TYPE = 'STATION'

EPS = 1e-3          # 控制点端点匹配容差（米）
PI2 = 2 * math.pi


# --------------------------------------------------------------------------- #
# 小工具
# --------------------------------------------------------------------------- #
def normalize_yaw(yaw: float) -> float:
    """与 path_planning/map/point.py:convert_position 一致的角度归一化 (−π, π]，截断到 4 位。"""
    y = (yaw or 0.0) % PI2
    if y > math.pi:
        y -= PI2
    return int(y * 10000) / 10000


def parse_map_yaml(path: Path) -> dict:
    """解析 ROS map.yaml（resolution / origin / image 等）。优先用 PyYAML，缺失则手工解析。"""
    text = path.read_text(encoding='utf-8')
    try:
        import yaml
        data = yaml.safe_load(text) or {}
        origin = data.get('origin', [0.0, 0.0, 0.0]) or [0.0, 0.0, 0.0]
        if not isinstance(origin, (list, tuple)):
            origin = [0.0, 0.0, 0.0]
        origin = list(origin) + [0.0, 0.0, 0.0]
        return {
            'resolution': float(data.get('resolution', 0.05)),
            'origin': [float(v) for v in origin[:3]],
            'image': data.get('image', 'map.png'),
            'negate': int(data.get('negate', 0) or 0),
        }
    except ImportError:
        pass
    # 手工解析（key: value，origin 为 [x, y, z]）
    kv = {}
    for line in text.splitlines():
        line = line.split('#', 1)[0].strip()
        if not line or ':' not in line:
            continue
        k, _, v = line.partition(':')
        kv[k.strip()] = v.strip()
    origin = [0.0, 0.0, 0.0]
    if 'origin' in kv:
        nums = [float(x) for x in kv['origin'].strip().lstrip('[').rstrip(']').split(',') if x.strip()]
        origin = (nums + [0.0, 0.0, 0.0])[:3]
    return {
        'resolution': float(kv.get('resolution', '0.05')),
        'origin': origin,
        'image': kv.get('image', 'map.png'),
        'negate': int((kv.get('negate', '0') or '0').strip()),
    }


def png_size(path: Path):
    """读取 PNG 宽高（仅解析 IHDR，无需 Pillow）。"""
    with open(path, 'rb') as f:
        head = f.read(24)
    if head[:8] != b'\x89PNG\r\n\x1a\n':
        raise ValueError(f"{path} 不是有效的 PNG 文件")
    return int.from_bytes(head[16:20], 'big'), int.from_bytes(head[20:24], 'big')


def file_md5(path: Path) -> str:
    h = hashlib.md5()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


# --------------------------------------------------------------------------- #
# 贝塞尔曲线长度
# --------------------------------------------------------------------------- #
def _bezier_point(ctrl, t: float):
    """de Casteljau 取点，ctrl = [[x, y], ...]。"""
    pts = [list(p) for p in ctrl]
    n = len(pts) - 1
    for r in range(1, n + 1):
        for i in range(n - r + 1):
            pts[i] = [pts[i][k] + (pts[i + 1][k] - pts[i][k]) * t for k in range(2)]
    return pts[0]


def curve_length(ctrl, samples: int = 512) -> float:
    """沿控制点序列（含首尾端点）采样计算曲线长度。"""
    if len(ctrl) < 2:
        return 0.0
    try:  # 优先用 numpy 向量化
        import numpy as np
        ctrl = np.asarray(ctrl, dtype=float)
        n_ctrl = len(ctrl)
        t = np.linspace(0.0, 1.0, samples)
        from math import comb
        bern = np.stack([comb(n_ctrl - 1, i) * (t ** i) * ((1 - t) ** (n_ctrl - 1 - i))
                         for i in range(n_ctrl)], axis=1)  # (samples, n_ctrl)
        pts = bern @ ctrl  # (samples, 2)
        return float(np.sqrt(np.square(np.diff(pts, axis=0)).sum(axis=1)).sum())
    except ImportError:
        prev = _bezier_point(ctrl, 0.0)
        length = 0.0
        for i in range(1, samples + 1):
            p = _bezier_point(ctrl, i / samples)
            length += math.dist(prev, p)
            prev = p
        return length


# --------------------------------------------------------------------------- #
# 控制点解析
# --------------------------------------------------------------------------- #
def _close(a, b) -> bool:
    return abs(a[0] - b[0]) < EPS and abs(a[1] - b[1]) < EPS


def parse_control_points(cp, from_c, to_c):
    """
    Nav2 的 BEZIER control_points 通常包含首尾端点：[from, c1, c2, to]。
    返回 (anchors, endpoints_stripped)：
      - anchors: 交管路网 vertex 用的控制锚点（直线边为空）
      - 若首尾不匹配端点，则认为 control_points 全是锚点。
    """
    pts = [[float(p[0]), float(p[1])] for p in (cp or [])]
    if not pts:
        return [], False
    stripped = False
    if len(pts) >= 2 and _close(pts[0], from_c) and _close(pts[-1], to_c):
        pts = pts[1:-1]
        stripped = True
    return pts, stripped


def decide_edge_type(edge_type_src: str, anchors):
    """LINE 或无锚点 -> NORMAL；1 锚点 -> BEZIER；2 锚点 -> CUBIC_BEZIER；>2 取首尾两点。"""
    edge_type_src = (edge_type_src or 'LINE').upper()
    if edge_type_src == 'LINE' or not anchors:
        return 'NORMAL', []
    if len(anchors) == 1:
        return 'BEZIER', anchors
    if len(anchors) == 2:
        return 'CUBIC_BEZIER', anchors
    # 多于两个锚点：用首尾两个近似为三阶贝塞尔
    return 'CUBIC_BEZIER', [anchors[0], anchors[-1]]


# --------------------------------------------------------------------------- #
# 构建路网
# --------------------------------------------------------------------------- #
def build_tc_graph(map_assets: dict, source_md5: str, zone_id: str):
    points = map_assets.get('points', []) or []
    edges_in = (map_assets.get('route_graph') or {}).get('edges', []) or []

    # 1) 节点：按类型各自顺序编号 no=S001..., n = 1000*No[prefix] + idx
    type_counters = defaultdict(int)
    name2id = {}
    nodes_attr = {}  # n -> attr（不含 id，id 由 node_link_data 写为节点 key）
    for p in points:
        tc_type = POINT_TYPE_MAP.get(str(p.get('type', '')).upper(), DEFAULT_POINT_TYPE)
        type_counters[tc_type] += 1
        idx = type_counters[tc_type]
        prefix = NODE_TYPE[tc_type]
        no = f"{prefix}{idx:03d}"
        n = 1000 * No[prefix] + idx
        pose = p.get('pose') or {}
        coord = [round(float(pose.get('x', 0.0)), 3), round(float(pose.get('y', 0.0)), 3)]
        name = p.get('name') or no
        nodes_attr[n] = {
            'coord': coord,
            'parkable': bool(p.get('parkable', False)),
            'angle': normalize_yaw(float(pose.get('yaw', 0.0))),
            'narrow': bool(p.get('narrow', False)),
            'uuid': str(uuid.uuid5(uuid.NAMESPACE_URL, f"node:{zone_id}:{no}")),
            'name': name,
            'type': tc_type,
            'n': n,
            'conflicts': [],
            'no': no,
        }
        if name in name2id:
            raise ValueError(f"点位名称重复：{name}")
        name2id[name] = n

    # 2) 边：先收集 (from, to) 集合用于判断是否单向（是否有反向边）
    pair_set = set()
    for e in edges_in:
        f = (e.get('from') or {}).get('id')
        t = (e.get('to') or {}).get('id')
        if f in name2id and t in name2id:
            pair_set.add((f, t))

    def make_edge_attr(f_name, t_name, backward, src_edge):
        u = name2id[f_name]
        v = name2id[t_name]
        from_c = nodes_attr[u]['coord']
        to_c = nodes_attr[v]['coord']
        anchors, _ = parse_control_points(src_edge.get('control_points'), from_c, to_c)
        etype, vertex = decide_edge_type(src_edge.get('edge_type'), anchors)

        if etype == 'NORMAL':
            length = round(math.dist(from_c, to_c), 4)
        else:
            length = round(curve_length([from_c] + vertex + [to_c]), 4)

        attr = {
            'uuid': str(uuid.uuid5(uuid.NAMESPACE_URL,
                                   f"edge:{zone_id}:{u}->{v}:{int(backward)}")),
            'type': etype,
            'vertex': vertex,
            'length': length,
            'weight': length,
            's': v if backward else u,   # 物理路径的“正向起点”：前向=source，后向=target
            'u': u,
            'v': v,
            # name 取正向 from__to；后向边沿用正向名（与 to__from 等价）
            'name': f"{t_name}__{f_name}" if backward else f"{f_name}__{t_name}",
            'dir': math.atan2(to_c[1] - from_c[1], to_c[0] - from_c[0]),
        }
        if etype in ('BEZIER', 'CUBIC_BEZIER'):
            a0, a1 = vertex[0], vertex[-1]
            attr['dir_in'] = math.atan2(a0[1] - from_c[1], a0[0] - from_c[0])
            attr['dir_out'] = math.atan2(to_c[1] - a1[1], to_c[0] - a1[0])
        if src_edge.get('hold_direction') is not None:
            attr['holdDirection'] = normalize_yaw(float(src_edge['hold_direction']))
        return attr, u, v

    edges_out = []
    hold_direction = False
    for e in edges_in:
        f = (e.get('from') or {}).get('id')
        t = (e.get('to') or {}).get('id')
        if f not in name2id or t not in name2id:
            print(f"  [跳过] 边引用了未知点位：{e.get('id')} ({f} -> {t})")
            continue
        backward = bool(e.get('backward', False))
        attr, u, v = make_edge_attr(f, t, backward, e)
        if e.get('hold_direction') is not None:
            hold_direction = True

        has_reverse = (f != t) and ((t, f) in pair_set)
        is_one_way = (not e.get('bidirectional', False)) and (not has_reverse)
        if is_one_way:
            attr['directed'] = True
        edges_out.append((u, v, attr))

        # nav2 一般把双向拆成两条显式边(backward=false/true)；若遇到 bidirectional=true
        # 的单条边，则补一条反向边（控制点反转）。
        if e.get('bidirectional', False):
            rev_src = dict(e)
            rev_src['control_points'] = list(reversed(e.get('control_points') or []))
            attr_r, u_r, v_r = make_edge_attr(t, f, not backward, rev_src)
            edges_out.append((u_r, v_r, attr_r))

    # 3) 组装 NetworkX DiGraph
    G = nx.DiGraph()
    # graph 元信息（顺序参照 AGV_20250827.graph.json）
    G.graph['conflicts'] = {}
    G.graph['restriction'] = []
    G.graph['md5'] = source_md5
    G.graph['holdDirection'] = hold_direction
    for n, attr in nodes_attr.items():
        G.add_node(n, **attr)
    G.add_edges_from((u, v, attr) for u, v, attr in edges_out)

    # 连通分量（有向图用强连通分量，与 create_graph_using_cloud_node 一致）
    if G.is_directed():
        comps = list(nx.strongly_connected_components(G))
    else:
        comps = list(nx.connected_components(G))
    # 按最小节点 id 排序，保证输出稳定（networkx 的分量返回顺序不保证确定）
    comps = sorted((sorted(c) for c in comps), key=lambda c: c[0])
    G.graph['number_connected_components'] = len(comps)
    G.graph['connected_components'] = comps or [sorted(G.nodes)]
    G.graph['connected_components_to_idx'] = {}
    for i, cc in enumerate(G.graph['connected_components']):
        G.graph['connected_components_to_idx'].update({str(c): i for c in cc})

    return G


def graph_to_json(G) -> dict:
    """node_link_data(edges='edges')，与 path_planning/map/map.py:save_graph_to_json 一致。"""
    return nx.node_link_data(G, edges='edges')


# --------------------------------------------------------------------------- #
# 构建地图元信息 yaml.json
# --------------------------------------------------------------------------- #
def build_map_yaml_json(name, zone_id, workspace_id, origin, resolution, width, height,
                        png_size_bytes, png_md5) -> dict:
    return {
        'dataType': 'MAP_FILE',
        'id': str(uuid.uuid5(uuid.NAMESPACE_URL, f"map:{zone_id}:{name}")),
        'workspaceId': workspace_id or '',
        'name': name,
        'parentZoneIds': [],
        'zoneId': zone_id,
        'mapFileType': 'RADAR',
        'uri': f"{name}.png",
        'size': png_size_bytes,
        'md5': png_md5,
        'origin': {'x': round(float(origin[0]), 4), 'y': round(float(origin[1]), 4)},
        'resolution': float(resolution),
        'width': width,
        'height': height,
        'default': True,
        'disabled': False,
        'deleted': False,
        'isNew': False,
        'imageData': f"{name}.png",
    }


# --------------------------------------------------------------------------- #
# 主流程
# --------------------------------------------------------------------------- #
def convert(src_dir: Path, name: str, zone_id: str, workspace_id: str, force: bool):
    assets_path = src_dir / 'map_assets.json'
    png_path = src_dir / 'map.png'
    yaml_path = src_dir / 'map.yaml'
    for p, desc in [(assets_path, 'map_assets.json'), (png_path, 'map.png'), (yaml_path, 'map.yaml')]:
        if not p.exists():
            sys.exit(f"找不到 {desc}：{p}")

    out_dir = Path(f"data/{zone_id}")
    out_dir.mkdir(parents=True, exist_ok=True)
    out_graph = out_dir / f"{name}.graph.json"
    out_yaml = out_dir / f"{name}.yaml.json"
    out_png = out_dir / f"{name}.png"

    for p in (out_graph, out_yaml, out_png):
        if p.exists() and not force:
            sys.exit(f"输出已存在：{p}（使用 -f 覆盖）")

    # ---- graph.json ----
    with open(assets_path, 'r', encoding='utf-8') as f:
        map_assets = json.load(f)
    source_md5 = file_md5(assets_path)
    G = build_tc_graph(map_assets, source_md5, zone_id)
    graph_json = graph_to_json(G)
    with open(out_graph, 'w', encoding='utf-8') as f:
        json.dump(graph_json, f, ensure_ascii=False, indent=4)

    # ---- yaml.json ----
    map_yaml = parse_map_yaml(yaml_path)
    width, height = png_size(png_path)
    png_md5 = file_md5(png_path)
    map_meta = build_map_yaml_json(
        name=name, zone_id=zone_id, workspace_id=workspace_id,
        origin=map_yaml['origin'], resolution=map_yaml['resolution'],
        width=width, height=height, png_size_bytes=png_path.stat().st_size, png_md5=png_md5,
    )
    with open(out_yaml, 'w', encoding='utf-8') as f:
        json.dump(map_meta, f, ensure_ascii=False, indent=4)

    # ---- png（直接拷贝）----
    shutil.copyfile(png_path, out_png)

    # ---- 自检：能否被项目原样加载 ----
    try:
        reloaded = nx.node_link_graph(json.load(open(out_graph, 'r', encoding='utf-8')), edges='edges')
        assert reloaded.number_of_nodes() == G.number_of_nodes()
        assert reloaded.number_of_edges() == G.number_of_edges()
    except Exception as e:  # pragma: no cover
        print(f"  [警告] 输出 graph.json 自检失败：{e}")

    print("转换完成：")
    print(f"  路网 : {out_graph}  (节点 {G.number_of_nodes()}, 边 {G.number_of_edges()}, "
          f"连通块 {G.graph['number_connected_components']})")
    print(f"  地图 : {out_yaml}  ({width}x{height}, resolution={map_yaml['resolution']}, "
          f"origin={map_meta['origin']})")
    print(f"  底图 : {out_png}")
    if G.graph['number_connected_components'] > 1:
        print(f"  [提示] 路网存在 {G.graph['number_connected_components']} 个连通块，"
              f"请确认是否有孤立点位。")


def main():
    default_src = Path('data/006962e2-c5a9-422b-a697-0a145cfe3001/20260711_xmk')
    ap = argparse.ArgumentParser(description='将 Nav2 导航地图转换为交管项目可用格式')
    ap.add_argument('src_dir', nargs='?', default=str(default_src),
                    help=f'源目录（含 map_assets.json/map.png/map.yaml），默认 {default_src}')
    ap.add_argument('-n', '--name', default=None, help='输出文件名前缀，默认取源目录名')
    ap.add_argument('-z', '--zone-id', default='00ad9034-6b81-44a5-9e59-2ed5220befd1', help='区域 ID，默认取源目录的父目录名')
    ap.add_argument('-w', '--workspace-id', default='6bf9eb24-ac3f-4bce-808b-69ae4d6a5046', help='工作空间 ID（可选）')
    ap.add_argument('-f', '--force', action='store_true', help='覆盖已存在的输出文件')
    args = ap.parse_args()

    src_dir = Path(args.src_dir).resolve()
    if not src_dir.is_dir():
        sys.exit(f"源目录不存在：{src_dir}")
    name = args.name or src_dir.name
    zone_id = args.zone_id or src_dir.parent.name
    convert(src_dir, name, zone_id, args.workspace_id, args.force)


if __name__ == '__main__':
    main()
