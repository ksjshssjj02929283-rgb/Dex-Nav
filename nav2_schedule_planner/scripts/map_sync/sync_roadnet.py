#!/usr/bin/env python3
"""把车端当前使用的路网转换后同步到 dexe_scheduler 云端。

在 pc1 上运行，替代原先「scp 拉图 → 本地转换 → 前端手动上传三个文件」的手工流程。

三步，每步之间人为确认：

  1. 从 ROS 话题 /map_manager/map_folder + /map_manager/map_name 取当前使用的地图，
     列出该地图目录里的路网/元信息文件 → 确认「是否同步当前 xxx 路网」。
  2. 从 pc2 的 traffic_control.yaml 取 workspace_id / entity_id / mqtt.host，
     按 entity_id 查云端这辆车所属的园区 / 区域 / 当前生效地图并打印。
  3. 用 map_convert_nav2tc.py 转成交管格式 → 确认「是否将当前转换的路网同步到云端」
     → POST /api/maps/import 上传三件套（等价于前端「导入地图」弹框）。
     云端导入成功后会自己通知交管重新拉路网，结果一并打印。

用法:
  ./sync_roadnet.py                     # 完整交互流程
  ./sync_roadnet.py -y                  # 全部确认自动为「是」（无人值守）
  ./sync_roadnet.py --dry-run           # 只转换不上传
  ./sync_roadnet.py -m 20260902_xmk     # 指定地图名，不读 ROS 话题
  ./sync_roadnet.py --workspace-id ... --entity-id ...   # 跳过读 pc2 配置（调试用）
  ./sync_roadnet.py -c --cloud-host 192.168.15.95        # 只看云端园区/区域/机器人，不同步

依赖: python3 (标准库) + networkx + pyyaml；ros2 环境用于读话题；sshpass 用于读 pc2 配置。
"""

import argparse
import json
import mimetypes
import os
import shutil
import subprocess
import sys
import urllib.error
import urllib.request
import uuid
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent

# ── 车端 ─────────────────────────────────────────────────────────────
ROS_SETUPS = ['/opt/ros/humble/setup.bash', '/home/dexforce/w1/install/setup.bash']
ROS_DOMAIN_ID = os.environ.get('ROS_DOMAIN_ID', '20')
RMW = os.environ.get('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
TOPIC_TIMEOUT = 15
# 车端这几个话题都是 latched（transient_local + reliable），必须用同样的 QoS 订阅，
# 否则订阅上去之后没有新消息，永远收不到启动时发布的那一帧。
QOS_ARGS = '--qos-durability transient_local --qos-reliability reliable'

# ── pc2（交管配置所在的工控机）──────────────────────────────────────
PC2_HOST = os.environ.get('PC2_HOST', '192.168.20.21')
PC2_USER = os.environ.get('PC2_USER', 'dexforce')
PC2_PASS = os.environ.get('PC2_PASS')
TC_YAML = '/home/dexforce/workspace/dexe_traffic_control_service/config/traffic_control.yaml'

# ── 云端 ─────────────────────────────────────────────────────────────
CLOUD_PORT = 3001                       # 固定：dispatch-frontend，nginx 反代 /api → backend:8000
CLOUD_TOKEN = os.environ.get('CLOUD_TOKEN')
HTTP_TIMEOUT = 30
LIST_LIMIT = 500                        # 后端列表接口的 limit 上限
UPLOAD_TIMEOUT = 120


class Fail(Exception):
    """流程性失败：打印一行原因后退出，不打堆栈。"""


def required_env(name):
    value = os.environ.get(name)
    if not value:
        raise Fail(f'缺少环境变量 {name}，请先配置后重试')
    return value


def info(msg=''):
    print(msg, flush=True)


def step(n, title):
    info(f"\n\033[1m── 步骤 {n}/3 · {title} ──\033[0m")


def confirm(question, assume_yes=False):
    if assume_yes:
        info(f"{question} [y/N] y  (-y 自动确认)")
        return True
    try:
        ans = input(f"{question} [y/N] ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        info()
        return False
    return ans in ('y', 'yes', '是')


# --------------------------------------------------------------------------- #
# 步骤 1：车端当前地图
# --------------------------------------------------------------------------- #
def ros_echo_once(topic):
    """echo 一次 latched 话题，返回 CompletedProcess；超时返回 None。"""
    sourcing = ' && '.join(f'source {s}' for s in ROS_SETUPS if Path(s).exists())
    prefix = f'{sourcing} && ' if sourcing else ''
    cmd = f'{prefix}ros2 topic echo --once {QOS_ARGS} {topic}'
    env = dict(os.environ, ROS_DOMAIN_ID=ROS_DOMAIN_ID, RMW_IMPLEMENTATION=RMW)
    try:
        return subprocess.run(['bash', '-c', cmd], capture_output=True, text=True,
                              timeout=TOPIC_TIMEOUT, env=env)
    except subprocess.TimeoutExpired:
        return None


def yaml_field(text, key):
    """从 ros2 topic echo 的 yaml 输出里取一个顶层标量字段。"""
    for line in text.splitlines():
        if line.startswith(f'{key}:'):
            # 形如 `map_name: 0902`，也可能带引号
            return line.split(':', 1)[1].strip().strip("'\"") or None
    return None


def read_map_state():
    """读车端当前使用的地图，返回 (folder, name, 话题来源)。

    新版导航把 folder / name / epoch 合成一条 latched 的
    /map_manager/map_state（nav_interfaces/msg/MapState）原子下发，
    老版本是 /map_manager/map_folder + /map_manager/map_name 两个 std_msgs/String。
    先试新的，读不到再回退老的。
    """
    r = ros_echo_once('/map_manager/map_state')
    if r is not None and r.returncode == 0:
        folder, name = yaml_field(r.stdout, 'map_folder'), yaml_field(r.stdout, 'map_name')
        if folder and name:
            return folder, name, '/map_manager/map_state'

    got = []
    for topic in ('/map_manager/map_folder', '/map_manager/map_name'):
        rr = ros_echo_once(topic)
        got.append(yaml_field(rr.stdout, 'data') if rr is not None and rr.returncode == 0 else None)
    if all(got):
        return got[0], got[1], '/map_manager/map_folder + map_name'

    raise Fail("读不到车端当前地图：/map_manager/map_state（新版导航）和 "
               "/map_manager/map_folder + map_name（老版）都没拿到数据。\n"
               "  排查：ros2 topic info -v /map_manager/map_state —— Publisher count 为 0 "
               "说明 map_manager 没在发；\n"
               "  也可以用 -m <地图名> [--map-folder <目录>] 手动指定，跳过读话题。")


# 三个文件在车端地图目录里的位置：新版分 assets/ 与 slam/ 两个子目录，老版平铺。
MAP_FILES = (
    ('graph', 'map_assets.json', ('assets', '')),
    ('yaml', 'map.yaml', ('slam', '')),
    ('png', 'map.png', ('slam', '')),
)


def locate_map_files(src):
    """在地图目录里定位三件套，兼容新旧目录布局。"""
    files = {}
    for key, fname, subdirs in MAP_FILES:
        found = None
        for sub in subdirs:
            p = src / sub / fname if sub else src / fname
            if p.is_file():
                found = p
                break
        else:
            # 布局又变了的兜底：全目录找，跳过 backups/ 里的历史版本
            for p in sorted(src.rglob(fname)):
                if 'backups' not in p.parts:
                    found = p
                    break
        files[key] = found
    return files


def current_map(args):
    if args.map_name:
        folder = Path(args.map_folder or '/home/dexforce/.dexe/data/maps')
        name = args.map_name
        info(f"地图目录 : {folder}   (命令行指定)")
        info(f"地图名称 : {name}   (命令行指定)")
    else:
        folder_topic, name, source = read_map_state()
        folder = Path(args.map_folder or folder_topic)
        info(f"地图目录 : {folder}   ({source})")
        info(f"地图名称 : {name}   ({source})")

    src = folder / name
    if not src.is_dir():
        raise Fail(f"地图目录不存在：{src}")

    files = locate_map_files(src)
    for key, fname, _ in MAP_FILES:
        p = files[key]
        mark = '✓' if p else '✗ 缺失'
        where = f"  {p.relative_to(src)}  {p.stat().st_size / 1024:.1f} KB" if p else ''
        info(f"  {mark} {fname:<18}{where}")

    if any(v is None for v in files.values()):
        if files['graph'] is None:
            raise Fail(f"{src} 下没有 map_assets.json —— 这张地图还没有画路网，"
                       f"请先在车端编辑并保存路网后再同步。")
        raise Fail(f"{src} 缺少必需文件，无法转换。")
    return src, name, files


# --------------------------------------------------------------------------- #
# 步骤 2：pc2 配置 + 云端归属信息
# --------------------------------------------------------------------------- #
def read_pc2_config():
    """从 pc2 读 traffic_control.yaml，返回 (workspace_id, entity_id, mqtt_host)。"""
    ssh = ['sshpass', '-p', required_env('PC2_PASS'), 'ssh',
           '-o', 'StrictHostKeyChecking=no', '-o', 'UserKnownHostsFile=/dev/null',
           '-o', 'LogLevel=ERROR', '-o', 'ConnectTimeout=8',
           f'{PC2_USER}@{PC2_HOST}', f'cat {TC_YAML}']
    try:
        r = subprocess.run(ssh, capture_output=True, text=True, timeout=30)
    except FileNotFoundError:
        raise Fail("找不到 sshpass（sudo apt install sshpass），或改用 --workspace-id/--entity-id 手动指定")
    except subprocess.TimeoutExpired:
        raise Fail(f"连接 pc2({PC2_HOST}) 超时")
    if r.returncode != 0:
        raise Fail(f"读取 pc2 的 {TC_YAML} 失败：{r.stderr.strip()[:200]}")

    try:
        import yaml
    except ImportError:
        raise Fail("缺少 pyyaml（pip3 install pyyaml）")
    try:
        cfg = yaml.safe_load(r.stdout) or {}
    except Exception as e:
        raise Fail(f"解析 pc2 的 traffic_control.yaml 失败：{e}")

    def dig(node, key):
        """递归找第一个匹配的键——配置的嵌套层级各版本不一，不写死路径。"""
        if isinstance(node, dict):
            for k, v in node.items():
                if k == key and not isinstance(v, (dict, list)):
                    return v
            for v in node.values():
                got = dig(v, key)
                if got is not None:
                    return got
        elif isinstance(node, list):
            for v in node:
                got = dig(v, key)
                if got is not None:
                    return got
        return None

    ws = dig(cfg, 'workspace_id')
    ent = dig(cfg, 'entity_id')
    host = dig(cfg.get('mqtt', cfg), 'host') or dig(cfg, 'host')
    info(f"pc2 配置 : {PC2_USER}@{PC2_HOST}:{TC_YAML}")
    info(f"  workspace_id = {ws}")
    info(f"  entity_id    = {ent}")
    info(f"  mqtt.host    = {host}   (云端地址)")
    for label, value in (('workspace_id', ws), ('entity_id', ent), ('mqtt.host', host)):
        if not value:
            raise Fail(f"pc2 配置里没读到 {label}")
    return str(ws), str(ent), str(host)


def cloud_url(host):
    """host 允许写成 ip、ip:port 或完整 URL。"""
    if host.startswith('http'):
        return host.rstrip('/')
    return f"http://{host}" if ':' in host else f"http://{host}:{CLOUD_PORT}"


def api_get(base, path):
    req = urllib.request.Request(f"{base}{path}",
                                 headers={'Authorization': f'Bearer {required_env("CLOUD_TOKEN")}'})
    try:
        with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT) as r:
            return json.loads(r.read().decode())
    except urllib.error.HTTPError as e:
        raise Fail(f"GET {path} 返回 HTTP {e.code}：{e.read().decode()[:200]}")
    except urllib.error.URLError as e:
        raise Fail(f"连不上云端 {base}：{e.reason}\n"
                   f"  车端配置里的 mqtt.host 决定了云端地址，若它指向的不是当前调度服务，"
                   f"用 --cloud-host <ip> 覆盖。")
    except Exception as e:
        raise Fail(f"GET {path} 失败：{e}")


def api_list(base, path):
    """取一个列表接口，返回 items。后端形如 {"items": [...], "total": n}，limit 上限 500。"""
    data = api_get(base, f"{path}?limit={LIST_LIMIT}")
    if not isinstance(data, dict):
        return data or []
    items = data.get('items') or []
    total = data.get('total')
    if total is not None and len(items) < total:
        info(f"  [注意] {path} 只取回 {len(items)}/{total} 条，列表不完整")
    return items


def cloud_tree(base):
    """打印云端 园区 / 区域 / 机器人 三级结构（-c）。"""
    workspaces = api_list(base, '/api/workspaces/')
    zones = api_list(base, '/api/cbs-zones/')
    robots = api_list(base, '/api/robots/')

    ws_by_id = {str(w.get('id')): w for w in workspaces}
    zone_by_id = {str(z.get('id')): z for z in zones}

    zones_of_ws = defaultdict(list)
    for z in zones:
        zones_of_ws[str(z.get('workspace_id') or '')].append(z)

    robots_of_zone = defaultdict(list)
    homeless = defaultdict(list)          # workspace_id -> 没有有效区域的机器人
    for r in robots:
        zid = str(r.get('zone_id') or r.get('zoneId') or '')
        if zid in zone_by_id:
            robots_of_zone[zid].append(r)
        else:
            homeless[str(r.get('workspace_id') or r.get('workspace') or '')].append(r)

    def robot_line(r, prefix):
        flags = [str(r.get('status') or '?')]
        if r.get('enabled') is False:
            flags.append('禁用')
        if r.get('simulated'):
            flags.append('仿真')
        info(f"{prefix}{r.get('robot_name') or r.get('name') or '(未命名)'}  "
             f"{r.get('robot_id') or r.get('id')}  [{' / '.join(flags)}]")

    def print_ws(ws_id, ws_name):
        info(f"\n\033[1m{ws_name}\033[0m  {ws_id or '-'}")
        rows = [('zone', z) for z in zones_of_ws.get(ws_id, [])]
        if homeless.get(ws_id):
            rows.append(('loose', homeless[ws_id]))
        if not rows:
            info("  (无区域、无机器人)")
            return
        for i, (kind, item) in enumerate(rows):
            last = i == len(rows) - 1
            branch, indent = ('└─ ', '   ') if last else ('├─ ', '│  ')
            if kind == 'zone':
                zid = str(item.get('id'))
                mark = '  [禁用]' if item.get('enable') is False else ''
                rs = robots_of_zone.get(zid, [])
                empty = '' if rs else '  (无机器人)'
                info(f"  {branch}{item.get('name')}  {zid}{mark}{empty}")
                for j, r in enumerate(rs):
                    sub_branch = '└─ ' if j == len(rs) - 1 else '├─ '
                    robot_line(r, f"  {indent}{sub_branch}")
            else:
                info(f"  {branch}(未分配区域)")
                for j, r in enumerate(item):
                    sub_branch = '└─ ' if j == len(item) - 1 else '├─ '
                    robot_line(r, f"  {indent}{sub_branch}")

    info(f"云端 : {base}")
    for w in workspaces:
        print_ws(str(w.get('id')), w.get('name') or '(未命名)')

    # 归属了不存在的园区的区域 / 机器人，单独列出来，免得凭空消失
    for ws_id in sorted(set(list(zones_of_ws) + list(homeless)) - set(ws_by_id)):
        print_ws(ws_id, f"(未分配园区)" if not ws_id else f"(未知园区 {ws_id})")

    info(f"\n共 {len(workspaces)} 园区 / {len(zones)} 区域 / {len(robots)} 机器人")


def graph_summary(map_obj):
    g = map_obj.get('graph_json') or {}
    return f"{len(g.get('nodes', []))} 点 / {len(g.get('edges', []))} 边"


def cloud_context(base, workspace_id, entity_id):
    """查这辆车在云端的归属，返回 (zone_id, zone_name)。"""
    robot = api_get(base, f"/api/robots/{entity_id}")
    zone_id = robot.get('zone_id') or robot.get('zoneId')
    ws_id = robot.get('workspace_id') or workspace_id
    if not zone_id:
        raise Fail(f"云端机器人 {entity_id} 没有绑定区域(zone)，请先在调度前端为它指定区域")
    if workspace_id and ws_id and str(ws_id) != str(workspace_id):
        info(f"  [注意] 车端配置的 workspace_id({workspace_id}) 与云端记录({ws_id}) 不一致，以云端为准")

    ws = api_get(base, f"/api/workspaces/{ws_id}")
    zone = api_get(base, f"/api/cbs-zones/{zone_id}")

    info(f"云端      : {base}")
    info(f"  机器人   : {robot.get('robot_name') or robot.get('name')}  ({entity_id})")
    info(f"             状态 {robot.get('status')}，最后心跳 {robot.get('last_heartbeat')}")
    info(f"  园区     : {ws.get('name')}  ({ws_id})")
    info(f"  区域     : {zone.get('name')}  ({zone_id})")

    map_id = robot.get('map_id')
    if map_id:
        m = api_get(base, f"/api/maps/{map_id}")
        state = '生效中' if m.get('is_active') == 'enable' else m.get('is_active')
        ver = f"  v{m['version']}" if m.get('version') is not None else ''
        info(f"  当前地图 : {m.get('name')}{ver}  [{state}]  "
             f"{graph_summary(m)}  ({map_id})")
        info(f"             {m.get('description') or ''}")
    else:
        info(f"  当前地图 : 该机器人还没有关联地图")
    return str(zone_id), zone.get('name')


# --------------------------------------------------------------------------- #
# 步骤 3：转换 + 上传
# --------------------------------------------------------------------------- #
def stage_src(files, name, workdir):
    """map_convert_nav2tc.py 要求三个文件平铺在同一目录，车端新布局分散在
    assets/ 和 slam/ 下，先集中拷到 workdir/src/<name>/ 再转换。"""
    stage = workdir / 'src' / name
    stage.mkdir(parents=True, exist_ok=True)
    for key, fname, _ in MAP_FILES:
        dst = stage / fname
        if files[key].resolve() != dst.resolve():
            shutil.copy2(files[key], dst)
    return stage


def convert(src_dir, name, zone_id, workspace_id, workdir):
    """调用 map_convert_nav2tc.py，产物落在 workdir/data/<zoneId>/。"""
    converter = HERE / 'map_convert_nav2tc.py'
    if not converter.is_file():
        raise Fail(f"找不到转换脚本 {converter}")
    cmd = [sys.executable, str(converter), str(src_dir),
           '-n', name, '-z', zone_id, '-w', workspace_id, '-f']
    info(f"$ {' '.join(cmd)}")
    r = subprocess.run(cmd, cwd=str(workdir))
    if r.returncode != 0:
        raise Fail("路网转换失败，见上方输出")

    out = workdir / 'data' / zone_id
    files = {'graph': out / f'{name}.graph.json',
             'yaml': out / f'{name}.yaml.json',
             'png': out / f'{name}.png'}
    for key, p in files.items():
        if not p.is_file():
            raise Fail(f"转换后缺少产物：{p}")
    return files


def upload(base, files, map_name, zone_id):
    """等价于调度前端「导入地图」弹框：multipart 上传三件套。"""
    boundary = f'----sync-roadnet-{uuid.uuid4().hex}'
    body = bytearray()

    def add_field(name, value):
        body.extend(f'--{boundary}\r\nContent-Disposition: form-data; name="{name}"\r\n\r\n'
                    f'{value}\r\n'.encode())

    def add_file(field, path: Path):
        ctype = mimetypes.guess_type(path.name)[0] or 'application/octet-stream'
        body.extend(f'--{boundary}\r\nContent-Disposition: form-data; name="{field}"; '
                    f'filename="{path.name}"\r\nContent-Type: {ctype}\r\n\r\n'.encode())
        body.extend(path.read_bytes())
        body.extend(b'\r\n')

    add_file('graph_file', files['graph'])
    add_file('yaml_file', files['yaml'])
    add_file('png_file', files['png'])
    add_field('map_name', map_name)
    add_field('zone_id', zone_id)
    body.extend(f'--{boundary}--\r\n'.encode())

    req = urllib.request.Request(
        f"{base}/api/maps/import", data=bytes(body), method='POST',
        headers={'Authorization': f'Bearer {required_env("CLOUD_TOKEN")}',
                 'Content-Type': f'multipart/form-data; boundary={boundary}'})
    try:
        with urllib.request.urlopen(req, timeout=UPLOAD_TIMEOUT) as r:
            return json.loads(r.read().decode())
    except urllib.error.HTTPError as e:
        raise Fail(f"上传失败 HTTP {e.code}：{e.read().decode()[:300]}")
    except urllib.error.URLError as e:
        raise Fail(f"上传失败，连不上云端 {base}：{e.reason}")
    except Exception as e:
        raise Fail(f"上传失败：{e}")


# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(
        description='把车端当前使用的路网转换后同步到 dexe_scheduler 云端',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-m', '--map-name', help='指定地图名，跳过读 ROS 话题')
    ap.add_argument('--map-folder', help='指定地图根目录，默认读 /map_manager/map_folder')
    ap.add_argument('--workspace-id', help='跳过读 pc2 配置，直接指定 workspace_id')
    ap.add_argument('--entity-id', help='跳过读 pc2 配置，直接指定 entity_id')
    ap.add_argument('-c', '--cloud-list', action='store_true',
                    help='只打印云端园区/区域/机器人三级结构，不做同步')
    ap.add_argument('--cloud-host', help=f'云端地址，默认取 pc2 配置里的 mqtt.host；'
                                         f'不带端口时用 {CLOUD_PORT}')
    ap.add_argument('--cloud-map-name', help='上传到云端时使用的地图名，默认与车端地图同名')
    ap.add_argument('--workdir', default=str(HERE / 'out'), help='转换产物目录，默认 ./out')
    ap.add_argument('-y', '--yes', action='store_true', help='所有确认自动为「是」')
    ap.add_argument('--dry-run', action='store_true', help='只转换，不上传')
    args = ap.parse_args()

    # ── -c：只看云端有什么 ──
    if args.cloud_list:
        info(f"\n\033[1m── 云端 园区 / 区域 / 机器人 ──\033[0m")
        cloud_host = args.cloud_host
        if not cloud_host:
            _, _, cloud_host = read_pc2_config()
            info()
        cloud_tree(cloud_url(cloud_host))
        return

    # ── 1. 车端当前地图 ──
    step(1, '车端当前使用的路网')
    src_dir, map_name, map_files = current_map(args)
    if not confirm(f"\n是否同步当前 \033[1m{map_name}\033[0m 路网？", args.yes):
        raise Fail("已取消")

    # ── 2. 云端归属 ──
    step(2, '这辆车在云端的园区 / 区域')
    if args.workspace_id and args.entity_id:
        workspace_id, entity_id = args.workspace_id, args.entity_id
        cloud_host = args.cloud_host or '192.168.15.95'
        info(f"使用命令行指定的 workspace_id / entity_id，跳过读 pc2 配置")
        info(f"  workspace_id = {workspace_id}")
        info(f"  entity_id    = {entity_id}")
    else:
        workspace_id, entity_id, mqtt_host = read_pc2_config()
        cloud_host = args.cloud_host or mqtt_host
    base = cloud_url(cloud_host)
    info()
    zone_id, zone_name = cloud_context(base, workspace_id, entity_id)

    # ── 3. 转换 + 上传 ──
    step(3, '转换路网并同步到云端')
    workdir = Path(args.workdir).expanduser().resolve()
    workdir.mkdir(parents=True, exist_ok=True)
    cloud_map_name = args.cloud_map_name or map_name
    staged = stage_src(map_files, cloud_map_name, workdir)
    files = convert(staged, cloud_map_name, zone_id, workspace_id, workdir)

    if args.dry_run:
        info(f"\n--dry-run：产物已生成，未上传。")
        return

    info(f"\n即将上传到 \033[1m{base}\033[0m")
    info(f"  区域   : {zone_name}  ({zone_id})")
    info(f"  地图名 : {cloud_map_name}   —— 云端同区域同名地图会被覆盖，并设为生效路网")
    for p in files.values():
        info(f"  文件   : {p.name}  ({p.stat().st_size / 1024:.1f} KB)")
    if not confirm(f"\n是否将当前转换的路网同步到云端？", args.yes):
        raise Fail("已取消，产物保留在 " + str(workdir))

    resp = upload(base, files, cloud_map_name, zone_id)
    ver = f" v{resp['version']}" if resp.get('version') is not None else ''
    info(f"\n\033[32m✓ 地图已导入云端\033[0m: {resp.get('name')}{ver} "
         f"({resp.get('id')})，已设为该区域生效路网")
    tc = resp.get('tc_sync') or {}
    if tc.get('ok'):
        info(f"\033[32m✓ 交管路网已更新\033[0m: {tc.get('message')}")
    else:
        info(f"\033[33m! 交管路网更新失败\033[0m: {tc.get('message')}")
        info(f"  地图已经在云端了，但交管还在用旧路网——需要排查交管服务后重新导入。")


if __name__ == '__main__':
    try:
        main()
    except Fail as e:
        print(f"\n\033[31m错误\033[0m: {e}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n已中断", file=sys.stderr)
        sys.exit(130)
