#!/usr/bin/env python3
"""把车端当前使用的路网转换后同步到 dexe_scheduler 云端。

在 pc1 上运行，替代原先「scp 拉图 → 本地转换 → 前端手动上传三个文件」的手工流程。

三步，每步之间人为确认：

  1. 从车端 map_manager 取当前使用的地图（新旧接口自动探测，见 probe_current_map），
     列出该地图目录里的路网/元信息文件 → 确认「是否同步当前 xxx 路网」。
     地图目录的新（assets/ + slam/ 分目录）旧（平铺）两种版式都认。
  2. 从 pc2 的 traffic_control.yaml（新路径优先、旧路径兜底，见 TC_YAML_CANDIDATES）取 workspace_id / entity_id / mqtt.host，
     按 entity_id 查云端这辆车所属的园区 / 区域 / 当前生效地图并打印。
  3. 用 map_convert_nav2tc.py 转成交管格式 → 确认「是否将当前转换的路网同步到云端」
     → POST /api/maps/import 上传三件套（等价于前端「导入地图」弹框）。
     云端导入成功后会自己通知交管重新拉路网，结果一并打印。

用法:
  ./sync_roadnet.py                     # 完整交互流程
  ./sync_roadnet.py -y                  # 全部确认自动为「是」（无人值守）
  ./sync_roadnet.py --dry-run           # 只转换不上传
  ./sync_roadnet.py -m 20260902_xmk     # 指定地图名，不问 map_manager
  ./sync_roadnet.py --workspace-id ... --entity-id ...   # 跳过读 pc2 配置（调试用）
  ./sync_roadnet.py --cloud-user admin --cloud-password ...   # 换登录账号（默认超级管理员）

云端鉴权（新旧版本兼容）:
  旧版调度不校验令牌，或内置放行 debug-token；新版部署时 INTERNAL_SERVICE_TOKEN 改成了
  随机值，debug-token 会被 401。脚本先用 CLOUD_TOKEN（默认 debug-token），碰到 401 才
  POST /api/auth/login 换 JWT 并重试——旧版服务端从不触发这一步，行为与原来一致。
  账号密码来源：--cloud-user/--cloud-password > CLOUD_USER/CLOUD_PASSWORD > 默认超级管理员。

依赖: python3 (标准库) + networkx + pyyaml；ros2 环境用于问 map_manager；sshpass 用于读 pc2 配置。
"""

import argparse
import json
import mimetypes
import os
import re
import shlex
import shutil
import subprocess
import sys
import urllib.error
import urllib.request
import uuid
from pathlib import Path

HERE = Path(__file__).resolve().parent

# ── 车端 ─────────────────────────────────────────────────────────────
ROS_SETUPS = ['/opt/ros/humble/setup.bash', '/home/dexforce/w1/install/setup.bash']
ROS_DOMAIN_ID = os.environ.get('ROS_DOMAIN_ID', '20')
RMW = os.environ.get('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
TOPIC_TIMEOUT = 15
SERVICE_TIMEOUT = 20

# ── pc2（交管配置所在的工控机）──────────────────────────────────────
PC2_HOST = os.environ.get('PC2_HOST', '192.168.20.21')
PC2_USER = os.environ.get('PC2_USER', 'dexforce')
PC2_PASS = os.environ.get('PC2_PASS', '123456')
# 本地调度配置：新版挪到 dexe_application 下；按顺序找，第一个存在的生效（新路径优先，旧路径兜底）
TC_YAML_CANDIDATES = (
    '/home/dexforce/workspace/dexe_application/services/traffic_control/config/traffic_control.yaml',
    '/home/dexforce/workspace/dexe_traffic_control_service/config/traffic_control.yaml',
)

# ── 云端 ─────────────────────────────────────────────────────────────
CLOUD_PORT = 3001                       # 固定：dispatch-frontend，nginx 反代 /api → backend:8000
CLOUD_TOKEN = os.environ.get('CLOUD_TOKEN', 'debug-token')   # 旧版/内部服务约定令牌
# 新版云端登录账号：默认超级管理员（admin 受园区数据范围限制，可能查不到车）
CLOUD_USER = os.environ.get('CLOUD_USER', 'super_admin')
CLOUD_PASSWORD = os.environ.get('CLOUD_PASSWORD', 'super_admin123')
HTTP_TIMEOUT = 30
UPLOAD_TIMEOUT = 120


class Fail(Exception):
    """流程性失败：打印一行原因后退出，不打堆栈。"""


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
# 地图目录有两种版式，按「新 → 旧」顺序找：
#   新（分目录）: <map>/assets/map_assets.json + <map>/slam/{map.yaml,map.png}
#   旧（平铺）  : <map>/{map_assets.json,map.yaml,map.png}
MAP_FILE_CANDIDATES = {
    'graph': ('assets/map_assets.json', 'map_assets.json'),
    'yaml': ('slam/map.yaml', 'map.yaml'),
    'png': ('slam/map.png', 'map.png'),
}
# map_convert_nav2tc.py 只认平铺布局，转换前要按这组文件名摊平
FLAT_NAMES = {'graph': 'map_assets.json', 'yaml': 'map.yaml', 'png': 'map.png'}


def _run_ros(cmd, timeout):
    """在 source 过 ROS 环境的 bash 里跑一条命令。超时抛 TimeoutExpired，其余情况都返回。"""
    sourcing = ' && '.join(f'source {s}' for s in ROS_SETUPS if Path(s).exists())
    prefix = f'{sourcing} && ' if sourcing else ''
    env = dict(os.environ, ROS_DOMAIN_ID=ROS_DOMAIN_ID, RMW_IMPLEMENTATION=RMW)
    return subprocess.run(['bash', '-c', prefix + cmd], capture_output=True,
                          text=True, timeout=timeout, env=env)


def _strip(value):
    """去掉 ros2 输出里的引号和空白，空串一律当成没读到。"""
    v = (value or '').strip().strip("'\"").strip()
    return v or None


def read_string_topic(topic):
    """读一个 latched 的 std_msgs/String 话题，返回 data；读不到返回 None。"""
    try:
        r = _run_ros(f'ros2 topic echo --once {topic}', TOPIC_TIMEOUT)
    except subprocess.TimeoutExpired:
        return None
    for line in r.stdout.splitlines():
        if line.startswith('data:'):
            # 形如 `data: 20260902_xmk`，也可能带引号
            return _strip(line.split(':', 1)[1])
    return None


# ros2 service call 的回显形如：
#   nav_interfaces.srv.GetString_Response(success=True, data='/home/dexforce/.dexe/data/maps')
# 注意 CycloneDDS 可能往 stdout 混进 deprecated 警告，所以用正则而不是逐行切。
_SVC_OK_RE = re.compile(r'success=(True|False)')
_SVC_DATA_RE = re.compile(r"""data=(['"])(.*?)\1""", re.S)
_SVC_LIST_RE = re.compile(r'map_list=\[(.*?)\]', re.S)


def read_string_service(service):
    """新接口：nav_interfaces/srv/GetString，返回 data；不可用/失败一律返回 None。"""
    try:
        r = _run_ros(f'ros2 service call {service} nav_interfaces/srv/GetString "{{}}"',
                     SERVICE_TIMEOUT)
    except subprocess.TimeoutExpired:
        return None
    out = r.stdout + r.stderr
    ok = _SVC_OK_RE.search(out)
    if not ok or ok.group(1) != 'True':
        return None
    m = _SVC_DATA_RE.search(out)
    return _strip(m.group(2)) if m else None


def read_map_state():
    """新话题 /map_manager/map_state（nav_interfaces/msg/MapState，latched）。

    folder + name + epoch 一条消息原子下发，专门用来消除多 topic 的时序竞争。
    返回 (folder, name, epoch)；读不到返回 None。
    """
    try:
        r = _run_ros('ros2 topic echo --once /map_manager/map_state', TOPIC_TIMEOUT)
    except subprocess.TimeoutExpired:
        return None
    folder = name = epoch = None
    for line in r.stdout.splitlines():
        t = line.strip()
        if t.startswith('map_folder:'):
            folder = _strip(t.split(':', 1)[1])
        elif t.startswith('map_name:'):
            name = _strip(t.split(':', 1)[1])
        elif t.startswith('epoch:'):
            epoch = _strip(t.split(':', 1)[1])
    return (folder, name, epoch) if folder and name else None


def available_maps_hint():
    """用 /map_manager/get_map_list 列出车端可用地图名，拼成提示串；拿不到就返回空串。"""
    try:
        r = _run_ros('ros2 service call /map_manager/get_map_list '
                     'nav_interfaces/srv/GetMapList "{}"', SERVICE_TIMEOUT)
    except subprocess.TimeoutExpired:
        return ''
    m = _SVC_LIST_RE.search(r.stdout + r.stderr)
    if not m:
        return ''
    names = [n for n in (_strip(x) for x in m.group(1).split(',')) if n]
    return f"\n  车端可用地图：{', '.join(names)}" if names else ''


def probe_current_map():
    """探测车端当前使用的地图，返回 (folder, name, 来源说明)。

    新旧接口都兼容，按「新 → 旧」顺序试，任一条成功就用它：

      1. 服务 /map_manager/get_map_folder + /map_manager/get_map_name
         （nav_interfaces/srv/GetString）—— 新接口，优先。
      2. 话题 /map_manager/map_state（nav_interfaces/msg/MapState，latched）
         —— folder + name + epoch 原子下发，一次拿全，比两个服务少一次往返。
      3. 话题 /map_manager/map_folder + /map_manager/map_name（std_msgs/String，latched）
         —— 老接口。新版本 map_manager 已不再发布（Publisher count = 0），
            但老车上还在，留着兜底。

    为什么服务排在 map_state 前面：服务是本人点名的新接口，且在老版本上必定不存在，
    探测代价就是一次快速失败；map_state 只在新版本里有，放第二档同样不会误命中老车。
    """
    folder = read_string_service('/map_manager/get_map_folder')
    name = read_string_service('/map_manager/get_map_name')
    if folder and name:
        return folder, name, '服务 /map_manager/get_map_{folder,name}'

    state = read_map_state()
    if state:
        f, n, epoch = state
        return f, n, f'话题 /map_manager/map_state（epoch={epoch}）'

    f = read_string_topic('/map_manager/map_folder')
    n = read_string_topic('/map_manager/map_name')
    if f and n:
        return f, n, '话题 /map_manager/map_{folder,name}（旧接口）'

    raise Fail(
        "问不到车端当前地图，三条路都试过了：\n"
        "  1. 服务 /map_manager/get_map_folder + get_map_name（新）\n"
        "  2. 话题 /map_manager/map_state（新）\n"
        "  3. 话题 /map_manager/map_folder + map_name（旧，新版本已停发）\n"
        "  导航是否在运行？ROS_DOMAIN_ID 是否为 " + ROS_DOMAIN_ID + "？\n"
        "  也可以用 -m <地图名> [--map-folder <目录>] 直接指定，跳过探测。")


def current_map(args):
    if args.map_name:
        folder = Path(args.map_folder or '/home/dexforce/.dexe/data/maps')
        name = args.map_name
        info(f"地图目录 : {folder}   (命令行指定)")
        info(f"地图名称 : {name}   (命令行指定)")
    else:
        probed_folder, name, how = probe_current_map()
        folder = Path(args.map_folder or probed_folder)
        info(f"地图目录 : {folder}   ({'命令行指定' if args.map_folder else how})")
        info(f"地图名称 : {name}   ({how})")

    src = folder / name
    if not src.is_dir():
        raise Fail(f"地图目录不存在：{src}{available_maps_hint()}")

    files = {}
    for key, cands in MAP_FILE_CANDIDATES.items():
        hit = next((src / c for c in cands if (src / c).is_file()), None)
        files[key] = hit
        if hit:
            info(f"  ✓ {str(hit.relative_to(src)):<26}{hit.stat().st_size / 1024:.1f} KB")
        else:
            info(f"  ✗ 缺失 {cands[0]}   (也找过 {cands[-1]})")

    missing = [k for k, v in files.items() if v is None]
    if missing:
        if files['graph'] is None:
            raise Fail(f"{src} 下没有 map_assets.json —— 这张地图还没有画路网，"
                       f"请先在车端编辑并保存路网后再同步。")
        raise Fail(f"{src} 缺少必需文件，无法转换。")
    return src, name, files


def flatten_for_converter(files, name, workdir):
    """把命中的三件套摊平成 map_convert_nav2tc.py 认识的布局，返回可直接传给它的目录。

    转换脚本写死了 <src_dir>/{map_assets.json,map.yaml,map.png}，而新版地图是
    assets/ + slam/ 分目录。这里不改转换脚本，改喂给它的目录：
    已经是平铺布局就原样返回（不做多余拷贝），否则拷进 <workdir>/_flat/<name>/。
    """
    if all(p.parent == files['graph'].parent and p.name == FLAT_NAMES[k]
           for k, p in files.items()):
        return files['graph'].parent

    dest = Path(workdir) / '_flat' / name
    dest.mkdir(parents=True, exist_ok=True)
    for key, p in files.items():
        shutil.copy2(p, dest / FLAT_NAMES[key])
    info(f"地图是分目录版式，已把三件套摊平到 {dest} 供转换脚本使用")
    return dest


# --------------------------------------------------------------------------- #
# 步骤 2：pc2 配置 + 云端归属信息
# --------------------------------------------------------------------------- #
def read_pc2_config():
    """从 pc2 读 traffic_control.yaml，返回 (workspace_id, entity_id, mqtt_host)。

    按 TC_YAML_CANDIDATES 顺序取第一个存在的文件，一次 ssh 完成：
    远端先输出命中的路径（首行），再输出文件内容。
    """
    paths = ' '.join(shlex.quote(p) for p in TC_YAML_CANDIDATES)
    remote = (f'for f in {paths}; do '
              f'if [ -f "$f" ]; then echo "$f"; cat "$f"; exit 0; fi; '
              f'done; echo "都不存在" >&2; exit 3')
    ssh = ['sshpass', '-p', PC2_PASS, 'ssh',
           '-o', 'StrictHostKeyChecking=no', '-o', 'UserKnownHostsFile=/dev/null',
           '-o', 'LogLevel=ERROR', '-o', 'ConnectTimeout=8',
           f'{PC2_USER}@{PC2_HOST}', remote]
    try:
        r = subprocess.run(ssh, capture_output=True, text=True, timeout=30)
    except FileNotFoundError:
        raise Fail("找不到 sshpass（sudo apt install sshpass），或改用 --workspace-id/--entity-id 手动指定")
    except subprocess.TimeoutExpired:
        raise Fail(f"连接 pc2({PC2_HOST}) 超时")
    if r.returncode == 3:
        raise Fail("pc2 上找不到 traffic_control.yaml，已依次尝试：\n    "
                   + "\n    ".join(TC_YAML_CANDIDATES))
    if r.returncode != 0:
        raise Fail(f"读取 pc2 的 traffic_control.yaml 失败：{r.stderr.strip()[:200]}")
    tc_yaml, _, content = r.stdout.partition('\n')

    try:
        import yaml
    except ImportError:
        raise Fail("缺少 pyyaml（pip3 install pyyaml）")
    try:
        cfg = yaml.safe_load(content) or {}
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
    tag = '新路径' if tc_yaml == TC_YAML_CANDIDATES[0] else '旧路径（新路径不存在，兜底）'
    info(f"pc2 配置 : {PC2_USER}@{PC2_HOST}:{tc_yaml}  [{tag}]")
    info(f"  workspace_id = {ws}")
    info(f"  entity_id    = {ent}")
    info(f"  mqtt.host    = {host}   (云端地址)")
    for label, value in (('workspace_id', ws), ('entity_id', ent), ('mqtt.host', host)):
        if not value:
            raise Fail(f"pc2 配置里没读到 {label}")
    return str(ws), str(ent), str(host)


class CloudAuth:
    """云端令牌：先用约定令牌，401 了再登录换 JWT，新旧版本调度都能用。

    旧版：不校验，或 INTERNAL_SERVICE_TOKEN 保持默认 debug-token ⇒ 第一次请求就过，不会登录。
    新版：INTERNAL_SERVICE_TOKEN 是部署时的随机值 ⇒ 401 ⇒ /api/auth/login 换 access_token。
    """

    def __init__(self, token=CLOUD_TOKEN, user=CLOUD_USER, password=CLOUD_PASSWORD):
        self.token = token
        self.user = user
        self.password = password
        self.logged_in = False

    def header(self):
        return {'Authorization': f'Bearer {self.token}'}

    def login(self, base):
        """401 后调用：换到 JWT 返回 True；已经登录过（JWT 也被拒）返回 False。"""
        if self.logged_in:
            return False
        info(f"云端拒绝了约定令牌（新版调度需要登录），用账号 {self.user} 登录")
        req = urllib.request.Request(
            f"{base}/api/auth/login", method='POST',
            data=json.dumps({'username': self.user, 'password': self.password}).encode(),
            headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT) as r:
                token = json.loads(r.read().decode()).get('access_token')
        except urllib.error.HTTPError as e:
            raise Fail(f"云端登录失败（用户 {self.user}）HTTP {e.code}：{e.read().decode()[:200]}\n"
                       f"  用 --cloud-user/--cloud-password 或 CLOUD_USER/CLOUD_PASSWORD 换账号")
        except Exception as e:
            raise Fail(f"云端登录失败：{e}")
        if not token:
            raise Fail("云端登录返回里没有 access_token")
        self.token = token
        self.logged_in = True
        info(f"已登录云端（用户 {self.user}）")
        return True


AUTH = CloudAuth()


def unwrap(body):
    """新版后端把返回包成 {code, message, data}；旧版直接返回对象。两种都认。"""
    if isinstance(body, dict) and {'code', 'message', 'data'} <= body.keys():
        if body['code'] not in (0, 200):
            raise Fail(f"云端返回错误 code={body['code']}：{body.get('message')}")
        return body['data']
    return body


def cloud_request(base, make_req, timeout):
    """发请求，401 时登录一次后重试。make_req(headers) -> urllib.request.Request。"""
    while True:
        try:
            with urllib.request.urlopen(make_req(AUTH.header()), timeout=timeout) as r:
                return unwrap(json.loads(r.read().decode()))
        except urllib.error.HTTPError as e:
            if e.code == 401 and AUTH.login(base):
                continue
            raise


def api_get(base, path):
    try:
        return cloud_request(base, lambda h: urllib.request.Request(f"{base}{path}", headers=h),
                             HTTP_TIMEOUT)
    except Fail:
        raise
    except urllib.error.HTTPError as e:
        raise Fail(f"GET {path} 返回 HTTP {e.code}：{e.read().decode()[:200]}")
    except urllib.error.URLError as e:
        raise Fail(f"连不上云端 {base}：{e.reason}\n"
                   f"  车端配置里的 mqtt.host 决定了云端地址，若它指向的不是当前调度服务，"
                   f"用 --cloud-host <ip> 覆盖。")
    except Exception as e:
        raise Fail(f"GET {path} 失败：{e}")


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

    data = bytes(body)

    def make_req(h):
        return urllib.request.Request(
            f"{base}/api/maps/import", data=data, method='POST',
            headers={**h, 'Content-Type': f'multipart/form-data; boundary={boundary}'})
    try:
        return cloud_request(base, make_req, UPLOAD_TIMEOUT)
    except Fail:
        raise
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
    ap.add_argument('-m', '--map-name', help='指定地图名，跳过探测 map_manager')
    ap.add_argument('--map-folder',
                    help='指定地图根目录，默认问 map_manager（服务/map_state/旧话题 依次探测）')
    ap.add_argument('--workspace-id', help='跳过读 pc2 配置，直接指定 workspace_id')
    ap.add_argument('--entity-id', help='跳过读 pc2 配置，直接指定 entity_id')
    ap.add_argument('--cloud-host', help='云端地址，默认取 pc2 配置里的 mqtt.host')
    ap.add_argument('--cloud-map-name', help='上传到云端时使用的地图名，默认与车端地图同名')
    ap.add_argument('--workdir', default=str(HERE / 'out'), help='转换产物目录，默认 ./out')
    ap.add_argument('-y', '--yes', action='store_true', help='所有确认自动为「是」')
    ap.add_argument('--dry-run', action='store_true', help='只转换，不上传')
    ap.add_argument('--cloud-user', default=CLOUD_USER,
                    help='新版云端登录用户名（仅约定令牌被 401 时使用），默认 CLOUD_USER 或 super_admin')
    ap.add_argument('--cloud-password', default=CLOUD_PASSWORD,
                    help='登录密码，默认 CLOUD_PASSWORD 或超级管理员默认密码')
    args = ap.parse_args()
    AUTH.user, AUTH.password = args.cloud_user, args.cloud_password

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
    base = f"http://{cloud_host}:{CLOUD_PORT}"
    info()
    zone_id, zone_name = cloud_context(base, workspace_id, entity_id)

    # ── 3. 转换 + 上传 ──
    step(3, '转换路网并同步到云端')
    workdir = Path(args.workdir).expanduser().resolve()
    workdir.mkdir(parents=True, exist_ok=True)
    cloud_map_name = args.cloud_map_name or map_name
    convert_src = flatten_for_converter(map_files, cloud_map_name, workdir)
    files = convert(convert_src, cloud_map_name, zone_id, workspace_id, workdir)

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
