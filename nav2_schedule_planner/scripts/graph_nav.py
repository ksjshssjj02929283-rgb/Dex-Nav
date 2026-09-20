#!/usr/bin/env python3
"""按路网规划路径并下发 NavigateThroughGraphPoints 导航。

三种模式，由位置参数 TARGET 决定：

  <点位名>   规划到该点位并下发，例如 L4
  random     随机选一个目标点下发，到点后隔 --interval 秒再随机选下一个
             -d  随机到的目标点不能是当前起点
             -f  导航失败也继续下一个随机目标（默认失败即退出）
  origin     把当前起点本身当目标点下发（原地归位到最近的导航点）
  recover    脱困：不走路网、不走 action，直接给 path_follower 发一条
             「车当前坐标 -> 目标点」的直线 TrackingPath。
             不带点位名时目标取离车最近的那个导航点；
             带点位名（recover L2）时直线开向该点位。

不管哪种模式，脚本都自己完成三件事：

  1. 从 /map_manager/map_state 找到当前地图的 map_assets.json，用 networkx 建路网图
     （points 建节点，route_graph.edges 建边，bidirectional 补反向边，权重取边的几何长度）；
  2. 从 TF map->base_link 取车当前位置，按「最近点位 / 最近边」确定起点，逻辑对齐
     traffic_control 的 path_planning/map/map.py:get_nearest_node；
  3. networkx 最短路得到途经点序列，装进 NavigateThroughGraphPoints 下发，等结果。

等价于手填：

  ros2 action send_goal /navigate_through_graph_points \
      nav_interfaces/action/NavigateThroughGraphPoints \
      '{nav_point_ids: ["L2","L4"], nav_point_names: ["L5","L1","L2","L4"], ...}'

只是途经点由路网算出来，不用人填。服务端只读 nav_point_names，nav_point_ids / pose /
target_yaw 都被忽略（schedule_planner_node.cpp:executeGoal → resolveSegments），
这里 nav_point_ids 填成和 names 一样、pose/target_yaw 填 0，跟车端已有脚本保持一致。

用法（先 source 环境，或直接用同目录的 graph_nav.sh 包一层）：

  source /opt/ros/humble/setup.bash
  source ~/xumingkang/dexe_mobile_nav/install/setup.bash
  export ROS_DOMAIN_ID=20 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

  ./graph_nav.py L4                      # 规划到 L4 并下发
  ./graph_nav.py L4 --dry-run            # 只打印规划结果，不下发
  ./graph_nav.py random                  # 随机巡航：到点后隔 1s 再随机选下一个点
  ./graph_nav.py random -d -f            # 不重复当前起点，失败也继续
  ./graph_nav.py random --interval 2 --targets L1 L2 L4
  ./graph_nav.py origin                  # 把当前起点当目标点下发
  ./graph_nav.py recover                 # 直线开回离车最近的导航点
  ./graph_nav.py recover L2              # 直线开向 L2
  ./graph_nav.py --list                  # 打印路网 + 当前起点后退出

recover 是旁路：它绕过 schedule_planner 直接驱动 path_follower，路径不经过路网、
不做避障绕行，只有 collision_monitor 的停障兜底（发轨迹前会先调
collision_monitor 的 set_polygon_padding 把 PolygonStop 的膨胀临时设成 --padding，
因为 TrackingPath.padding 本身不会传到 collision_monitor）。用在车被挪到路网外、
action 因为「离起点太远」下不去的时候，别在 schedule_planner 正跑 goal 时用。

依赖：rclpy / tf2_ros / networkx / nav_interfaces（车端 overlay）。
"""

import argparse
import json
import math
import random
import sys
from pathlib import Path

import networkx as nx
import rclpy
import tf2_ros
from geometry_msgs.msg import Pose, Pose2D
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.time import Time

from std_srvs.srv import Trigger

from nav_interfaces.action import NavigateThroughGraphPoints
from nav_interfaces.msg import MapState, NavigationStatus, TrackingPath
from nav_interfaces.srv import SetPolygonPadding

ACTION_NAME = 'navigate_through_graph_points'
MODE_RANDOM = 'random'   # 保留字：随机巡航
MODE_ORIGIN = 'origin'   # 保留字：目标点取当前起点
MODE_RECOVER = 'recover'  # 保留字：直线脱困

# recover 直接顶到 path_follower 上，话题/服务/默认值都取自 schedule_planner.yaml
TRACKING_PATH_TOPIC = '/tracking_path'
NAVIGATION_STATUS_TOPIC = '/navigation_status'
STOP_SERVICE = '/stop'
# TrackingPath.padding 只是段级参数，path_follower 不会把它转给 collision_monitor：
# 真正生效的是 collision_monitor 的 ~/set_polygon_padding 服务（正常链路上由
# schedule_planner 在发每段轨迹前调）。recover 旁路了 schedule_planner，所以这里
# 自己补一次，服务名/检测器名与 schedule_planner.yaml 保持一致。
PADDING_SERVICE = '/collision_monitor/set_polygon_padding'
STOP_POLYGON_NAME = 'PolygonStop'   # 只改停障检测器，减速区保持 default_padding
DEFAULT_PADDING = 0.15              # 脱困用的停障膨胀 m（default_padding 是 0.3）
DEFAULT_RESOLUTION = 0.01        # 离散弧长间距 m（schedule_planner 的 resolution）
DEFAULT_TRACK_MODE = 1           # TrackingPath.MODE_DIFF（schedule_planner 的 default_mode）
MAP_STATE_TOPIC = '/map_manager/map_state'

# 与 route_graph_manager.cpp:edgePolyline 保持一致：BEZIER 采 40 段
BEZIER_SEGMENTS = 40

# 起点吸附阈值，取自 path_planning/map/map.py 的同名 zone 参数默认值
DISTANCE_ELEMENT_DIS = 0.10      # 离最近点位小于它 -> 直接用该点位
DIS_ALLOW_ROBOT_TO_GRAPH = 1.0   # 最近边的粗筛半径余量


# --------------------------------------------------------------------------- #
# 路网（纯计算，不依赖 ROS）
# --------------------------------------------------------------------------- #
def _xy(p):
    """control_point 兼容 [x, y] 和 {'x':, 'y':} 两种写法。"""
    if isinstance(p, dict):
        return float(p.get('x', 0.0)), float(p.get('y', 0.0))
    return float(p[0]), float(p[1])


def bezier_polyline(control_points, segments=BEZIER_SEGMENTS):
    """Bernstein 采样，control_points 含首尾端点（与车端 edgePolyline 同构）。"""
    cps = [_xy(p) for p in control_points]
    n = len(cps) - 1

    def binom(nn, k):
        v = 1.0
        for i in range(k):
            v = v * (nn - i) / (i + 1)
        return v

    poly = []
    for s in range(segments + 1):
        t = s / segments
        bx = by = 0.0
        for i, (cx, cy) in enumerate(cps):
            b = binom(n, i) * (1.0 - t) ** (n - i) * t ** i
            bx += b * cx
            by += b * cy
        poly.append((bx, by))
    return poly


def edge_polyline(edge, from_coord, to_coord):
    cps = edge.get('control_points') or []
    if str(edge.get('edge_type', 'LINE')).upper() == 'BEZIER' and len(cps) >= 2:
        return bezier_polyline(cps)
    return [from_coord, to_coord]


def polyline_length(poly):
    return sum(math.dist(poly[i], poly[i + 1]) for i in range(len(poly) - 1))


def distance_to_polyline(p, poly):
    """点到折线的最短距离（逐段投影，与 route_graph_manager.cpp 的实现一致）。"""
    if not poly:
        return math.inf
    if len(poly) == 1:
        return math.dist(p, poly[0])
    best = math.inf
    for (ax, ay), (bx, by) in zip(poly, poly[1:]):
        dx, dy = bx - ax, by - ay
        len2 = dx * dx + dy * dy
        if len2 < 1e-12:
            d = math.dist(p, (ax, ay))
        else:
            t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / len2))
            d = math.dist(p, (ax + t * dx, ay + t * dy))
        best = min(best, d)
    return best


def build_graph(assets):
    """map_assets.json -> networkx.DiGraph，节点 key 就是导航点名（下发用的也是名字）。"""
    G = nx.DiGraph()
    for p in assets.get('points') or []:
        name = p.get('name')
        if not name:
            continue
        pose = p.get('pose') or {}
        G.add_node(name,
                   coord=(float(pose.get('x', 0.0)), float(pose.get('y', 0.0))),
                   yaw=float(pose.get('yaw', 0.0)),
                   type=str(p.get('type', '')))

    for e in (assets.get('route_graph') or {}).get('edges') or []:
        u = (e.get('from') or {}).get('id')
        v = (e.get('to') or {}).get('id')
        if u not in G or v not in G:
            # 边引用了不存在的点位：跳过，与 map_convert_nav2tc.py 的处理一致
            continue
        poly = edge_polyline(e, G.nodes[u]['coord'], G.nodes[v]['coord'])
        length = polyline_length(poly)
        G.add_edge(u, v, weight=length, length=length, polyline=poly, id=e.get('id'))
        if e.get('bidirectional'):
            G.add_edge(v, u, weight=length, length=length,
                       polyline=list(reversed(poly)), id=e.get('id'))
    return G


def nearest_node(G, xy,
                 node_tol=DISTANCE_ELEMENT_DIS,
                 search_margin=DIS_ALLOW_ROBOT_TO_GRAPH):
    """车体坐标 -> 起点，逻辑对齐 path_planning/map/map.py:get_nearest_node。

    先看最近点位：距离小于 node_tol 就直接用它；否则认为车停在某条边上，
    找几何上最近的一条边（先按「边中心距 < 半长 + 余量」粗筛，再算点到折线距离），
    取这条边上离车更近的那个端点作为起点。返回 (node, dis, how)，
    dis 对齐原实现：吸到点位时是点距，吸到边上时是点到边的距离。
    """
    if not G.number_of_nodes():
        return None, math.inf, '路网为空'

    n = min(G.nodes, key=lambda k: math.dist(G.nodes[k]['coord'], xy))
    dis = math.dist(G.nodes[n]['coord'], xy)
    if dis < node_tol:
        return n, dis, f'压在点位 {n} 上'

    best = None  # (dis_to_edge, u, v)
    seen = set()
    for u, v, data in G.edges(data=True):
        key = tuple(sorted((u, v)))
        if key in seen:
            continue
        seen.add(key)
        poly = data['polyline']
        center = (sum(p[0] for p in poly) / len(poly), sum(p[1] for p in poly) / len(poly))
        if math.dist(center, xy) >= data['length'] / 2 + search_margin:
            continue
        d = distance_to_polyline(xy, poly)
        if best is None or d < best[0]:
            best = (d, u, v)

    if best is None:
        return None, math.inf, f'离最近点位 {n} {dis:.2f}m，且 {search_margin}m 内没有边'

    d, u, v = best
    near = u if math.dist(G.nodes[u]['coord'], xy) <= math.dist(G.nodes[v]['coord'], xy) else v
    return near, d, f'在边 {u}__{v} 上（离边 {d:.2f}m），吸到近端 {near}'


def nearest_point(G, xy):
    """离车最近的导航点，不做落边吸附。

    对应 get_nearest_node(ignore_limit=True)：recover 的前提就是车已经不在路网上，
    这时候「吸到最近边的端点」没有意义，要的就是几何上最近的那个点位。
    """
    n = min(G.nodes, key=lambda k: math.dist(G.nodes[k]['coord'], xy))
    return n, math.dist(G.nodes[n]['coord'], xy)


def line_poses(start_xy, goal_xy, resolution, fallback_yaw=0.0):
    """起点到终点的等距离散直线，返回 [(x, y, yaw), ...]。

    yaw 是该点的行进方向切向（直线上是常数），TrackingPath 要求发布方填好，
    否则 path_follower 得从 1cm 点距的 x,y 二次差分反推曲率，噪声放大约 4e4。
    起终点重合时切向无定义，退回 fallback_yaw（一般就是 target_yaw，只做末端旋转）。
    """
    (x0, y0), (x1, y1) = start_xy, goal_xy
    length = math.dist(start_xy, goal_xy)
    yaw = math.atan2(y1 - y0, x1 - x0) if length > 1e-9 else fallback_yaw
    if math.isnan(yaw):
        yaw = 0.0
    n = max(1, math.ceil(length / resolution))
    return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n, yaw) for i in range(n + 1)]


def plan_path(G, start, target):
    """途经点序列（含起点和终点）。起点即终点时退化成单点请求，服务端会自己处理。"""
    if start == target:
        return [target]
    return nx.shortest_path(G, start, target, weight='weight')


# --------------------------------------------------------------------------- #
# ROS 节点
# --------------------------------------------------------------------------- #
class GraphNav(Node):
    def __init__(self, args):
        super().__init__('graph_nav')
        self._args = args

        latched = QoSProfile(depth=1,
                             history=QoSHistoryPolicy.KEEP_LAST,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._map_state = None
        self.create_subscription(MapState, MAP_STATE_TOPIC, self._on_map_state, latched)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._client = ActionClient(self, NavigateThroughGraphPoints, ACTION_NAME)

        # recover 用：直接顶到 path_follower（QoS 与 schedule_planner 的发布端一致）
        self._path_pub = self.create_publisher(TrackingPath, TRACKING_PATH_TOPIC, 1)
        self._nav_status = None
        self.create_subscription(NavigationStatus, NAVIGATION_STATUS_TOPIC,
                                 self._on_nav_status, 10)
        self._stop_client = self.create_client(Trigger, STOP_SERVICE)
        self._padding_client = self.create_client(SetPolygonPadding, PADDING_SERVICE)

        self._graph = None
        self._epoch = None
        self._assets_path = None

    # ── 基础设施 ────────────────────────────────────────────────────
    def _on_map_state(self, msg):
        self._map_state = msg

    def _on_nav_status(self, msg):
        self._nav_status = msg

    def spin_for(self, seconds):
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_until(self, cond, timeout, tick=0.05):
        end = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            if cond():
                return True
            rclpy.spin_once(self, timeout_sec=tick)
        return cond()

    # ── 路网 ────────────────────────────────────────────────────────
    def assets_path(self):
        """当前地图的 map_assets.json 路径。

        目录布局取自 map_asset_manager_node.cpp：新布局 <map>/assets/map_assets.json，
        老机器扁平布局退回 <map>/map_assets.json。
        """
        if self._args.map_assets:
            return Path(self._args.map_assets).expanduser()

        if not self.wait_until(lambda: self._map_state is not None, self._args.map_timeout):
            raise RuntimeError(
                f"{self._args.map_timeout}s 内没收到 {MAP_STATE_TOPIC}——map_manager 没起来？"
                f"可以用 --map-assets 直接指定 map_assets.json")

        map_dir = Path(self._map_state.map_folder) / self._map_state.map_name
        new_layout = map_dir / 'assets' / 'map_assets.json'
        legacy = map_dir / 'map_assets.json'
        if new_layout.is_file():
            return new_layout
        if legacy.is_file():
            return legacy
        raise RuntimeError(f"{map_dir} 下没有 map_assets.json——这张图还没画路网？")

    def load_graph(self):
        """首次加载；地图 epoch 变了（重新加载/换图）就重建。"""
        epoch = self._map_state.epoch if self._map_state is not None else None
        if self._graph is not None and epoch == self._epoch:
            return self._graph

        path = self.assets_path()
        with open(path, 'r', encoding='utf-8') as f:
            assets = json.load(f)
        self._graph = build_graph(assets)
        self._epoch = self._map_state.epoch if self._map_state is not None else None
        self._assets_path = path
        self.get_logger().info(
            f"路网: {path} (点位 {self._graph.number_of_nodes()}, "
            f"有向边 {self._graph.number_of_edges()}, epoch={self._epoch})")
        return self._graph

    # ── 车体位置 / 起点 ─────────────────────────────────────────────
    def robot_xy(self):
        got = {}

        def lookup():
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._args.map_frame, self._args.base_frame, Time())
            except tf2_ros.TransformException:
                return False
            got['xy'] = (tf.transform.translation.x, tf.transform.translation.y)
            return True

        if not self.wait_until(lookup, self._args.tf_timeout):
            raise RuntimeError(
                f"{self._args.tf_timeout}s 内取不到 TF {self._args.map_frame} -> "
                f"{self._args.base_frame}——定位没起来？")
        return got['xy']

    def start_node(self, G):
        xy = self.robot_xy()
        start, dis, how = nearest_node(G, xy, self._args.node_tolerance,
                                       self._args.edge_margin)
        if start is None:
            raise RuntimeError(f"车在 ({xy[0]:.2f}, {xy[1]:.2f})，定不出起点：{how}")
        self.get_logger().info(
            f"车体位置 ({xy[0]:.2f}, {xy[1]:.2f}) -> 起点 {start}：{how}")
        return start

    # ── 下发 ────────────────────────────────────────────────────────
    def send(self, path):
        goal = NavigateThroughGraphPoints.Goal()
        # 服务端只认 nav_point_names；ids/pose/target_yaw 保持与车端已有脚本一致的填法
        goal.nav_point_ids = list(path)
        goal.nav_point_names = list(path)
        goal.pose = Pose2D(x=0.0, y=0.0, theta=0.0)
        goal.target_yaw = 0.0

        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('goal 被拒绝')
            return False

        result_future = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, result_future)
        except KeyboardInterrupt:
            self.get_logger().warn('收到 Ctrl+C，取消当前 goal')
            cancel_future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
            raise

        result = result_future.result().result
        # 成功/失败必须写成两个调用点：rclpy 按调用点缓存 severity，同一行换级别会抛
        # ValueError: Logger severity cannot be changed between calls.
        msg = f"结果: success={result.success} message='{result.message}'"
        if result.success:
            self.get_logger().info(msg)
        else:
            self.get_logger().warn(msg)
        return result.success

    def wait_server(self, timeout):
        return self._client.wait_for_server(timeout_sec=timeout)

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"[feedback] {fb.current_status} {fb.current_point_index}/{fb.total_points}")

    # ── 一次移动 ────────────────────────────────────────────────────
    def goto(self, target=None):
        """target 为 None 表示 origin 模式：目标点取车当前吸附到的那个起点。"""
        G = self.load_graph()
        if target is not None and target not in G:
            raise RuntimeError(f"路网里没有点位 {target}，已有点位：{', '.join(sorted(G.nodes))}")
        start = self.start_node(G)
        if target is None:
            target = start
            self.get_logger().info(f"origin 模式：目标点取当前起点 {start}")
        try:
            path = plan_path(G, start, target)
        except nx.NetworkXNoPath:
            self.get_logger().warn(f"{start} -> {target} 路网不连通，跳过")
            return False
        self.get_logger().info(f"规划路径: {' -> '.join(path)}")
        if self._args.dry_run:
            self.get_logger().info('--dry-run，不下发')
            return True
        return self.send(path)

    # ── 直线脱困 ────────────────────────────────────────────────────
    def target_yaw_for(self, node):
        """末端朝向：默认取点位自身的 yaw，可用 --target-yaw 覆盖、--no-final-rotation 关掉。"""
        if self._args.no_final_rotation:
            return float('nan')
        if self._args.target_yaw is not None:
            return float(self._args.target_yaw)
        return float(node['yaw'])

    def recover(self, point=None):
        G = self.load_graph()
        if point is not None and point not in G:
            raise RuntimeError(f"路网里没有点位 {point}，已有点位：{', '.join(sorted(G.nodes))}")

        xy = self.robot_xy()
        if point is None:
            point, dis = nearest_point(G, xy)
            self.get_logger().info(f"车体位置 ({xy[0]:.2f}, {xy[1]:.2f}) -> 最近点位 {point}（{dis:.2f}m）")
        else:
            self.get_logger().info(f"车体位置 ({xy[0]:.2f}, {xy[1]:.2f}) -> 指定点位 {point}")

        goal = G.nodes[point]['coord']
        target_yaw = self.target_yaw_for(G.nodes[point])
        poses = line_poses(xy, goal, self._args.resolution, target_yaw)
        yaw_str = 'nan（到点不旋转）' if math.isnan(target_yaw) else f"{target_yaw:.3f}"
        self.get_logger().info(
            f"直线轨迹: ({xy[0]:.2f}, {xy[1]:.2f}) -> ({goal[0]:.2f}, {goal[1]:.2f}) "
            f"{math.dist(xy, goal):.2f}m / {len(poses)} 点 / 间距 {self._args.resolution}m, "
            f"mode={self._args.track_mode} target_yaw={yaw_str}")

        if self._args.dry_run:
            self.get_logger().info('--dry-run，不发布')
            return True
        return self.publish_line(poses, target_yaw)

    def publish_line(self, poses, target_yaw):
        if not self.wait_until(lambda: self._path_pub.get_subscription_count() > 0,
                               self._args.server_timeout):
            self.get_logger().error(
                f"{TRACKING_PATH_TOPIC} 没有订阅者——path_follower 没起来？")
            return False

        tp = TrackingPath()
        tp.header.frame_id = self._args.map_frame
        tp.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in poses:
            p = Pose()
            p.position.x = x
            p.position.y = y
            # 只有 yaw 分量的四元数
            p.orientation.z = math.sin(yaw * 0.5)
            p.orientation.w = math.cos(yaw * 0.5)
            tp.poses.append(p)
        tp.mode = self._args.track_mode
        tp.backward = TrackingPath.FORWARD
        tp.hold_direction = float('nan')   # NaN = path_follower 用自己的默认策略
        tp.max_speed = float('nan')
        tp.padding = float(self._args.padding)   # 段级记录，实际生效靠下面的服务
        tp.target_yaw = float(target_yaw)

        # 必须在发轨迹之前设完，否则车已经起步而 padding 还是上一次的值
        self.set_stop_padding(self._args.padding)

        self._nav_status = None
        self._path_pub.publish(tp)
        self.get_logger().info(f"已发布 {TRACKING_PATH_TOPIC}（{len(tp.poses)} 点）")

        if self._args.no_wait:
            return True
        try:
            return self.wait_nav_status(self._args.recover_timeout)
        except KeyboardInterrupt:
            self.get_logger().warn(f"收到 Ctrl+C，调 {STOP_SERVICE} 停车")
            self.call_stop()
            raise

    def wait_nav_status(self, timeout):
        """等 path_follower 的跟踪结果；超时按失败处理并停车。"""
        end = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        seen = None
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            msg = self._nav_status
            if msg is None or msg.status == seen:
                continue
            seen = msg.status
            if msg.status == NavigationStatus.FINISHED:
                self.get_logger().info(f"到点: {msg.message}")
                return True
            if msg.status == NavigationStatus.FAILED:
                self.get_logger().warn(f"跟踪失败: {msg.message}")
                return False
            self.get_logger().info(f"[status] {msg.status} {msg.message}")

        self.get_logger().error(f"{timeout}s 内没等到跟踪结果，调 {STOP_SERVICE} 停车")
        self.call_stop()
        return False

    def set_stop_padding(self, padding):
        """临时把 collision_monitor 的停障膨胀改成 padding（NaN=恢复 default_padding）。

        失败只告警不中止：那样 padding 停留在上一次/默认值上，只会更保守。
        """
        if not self._padding_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                f"{PADDING_SERVICE} 不可用，停障膨胀保持原值（collision_monitor 没起来？）")
            return False

        req = SetPolygonPadding.Request()
        req.polygon_name = STOP_POLYGON_NAME
        req.padding = float(padding)
        future = self._padding_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        resp = future.result()
        if resp is None:
            self.get_logger().warn(f"{PADDING_SERVICE} 超时，停障膨胀保持原值")
            return False
        if not resp.success:
            self.get_logger().warn(f"设置停障膨胀失败: {resp.message}")
            return False

        self.get_logger().info(
            f"{STOP_POLYGON_NAME} 停障膨胀 -> {padding}m ({resp.message})")
        return True

    def call_stop(self):
        if not self._stop_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"{STOP_SERVICE} 不可用，没能停车")
            return
        future = self._stop_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

    # ── 随机巡航 ────────────────────────────────────────────────────
    def candidates(self, G, start):
        """可选的随机目标点。-d(--differ) 才把当前起点本身排掉。"""
        allowed = set(self._args.targets) if self._args.targets else set(G.nodes)
        unknown = allowed - set(G.nodes)
        if unknown:
            raise RuntimeError(f"--targets 里有路网上没有的点位：{', '.join(sorted(unknown))}")
        allowed -= set(self._args.exclude or [])
        if self._args.differ:
            allowed.discard(start)
        return [n for n in sorted(allowed) if nx.has_path(G, start, n)]

    def run_random(self):
        """返回 False 表示因导航失败退出（未加 -f）。"""
        while rclpy.ok():
            G = self.load_graph()
            start = self.start_node(G)
            picks = self.candidates(G, start)
            if not picks:
                self.get_logger().warn(f"从 {start} 出发没有可达的目标点，{self._args.interval}s 后重试")
                self.spin_for(self._args.interval)
                continue

            target = random.choice(picks)
            self.get_logger().info(f"随机目标: {target} (候选 {len(picks)} 个)")
            try:
                path = plan_path(G, start, target)
            except nx.NetworkXNoPath:
                continue
            self.get_logger().info(f"规划路径: {' -> '.join(path)}")
            if self._args.dry_run:
                self.get_logger().info('--dry-run，不下发')
            elif not self.send(path) and not self._args.keep_going:
                self.get_logger().error('导航失败，退出（加 -f 可失败后继续下一个随机目标）')
                return False

            if self._args.interval > 0:
                self.get_logger().info(f"{self._args.interval}s 后选下一个目标点")
                self.spin_for(self._args.interval)
        return True

    # ── 打印路网 ────────────────────────────────────────────────────
    def show(self):
        G = self.load_graph()
        print(f"\n点位（{G.number_of_nodes()}）:")
        for n in sorted(G.nodes):
            d = G.nodes[n]
            print(f"  {n:<10} ({d['coord'][0]:7.3f}, {d['coord'][1]:7.3f})  "
                  f"yaw={d['yaw']:6.3f}  {d['type']}")
        print(f"\n边（{G.number_of_edges()} 条有向边）:")
        for u, v, d in sorted(G.edges(data=True)):
            print(f"  {u:<10} -> {v:<10} {d['length']:.3f}m  id={d['id']}")
        try:
            start = self.start_node(G)
            print(f"\n当前起点: {start}")
            print(f"可达目标: {', '.join(self.candidates(G, start)) or '（无）'}")
        except RuntimeError as e:
            print(f"\n起点未知: {e}")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('target', nargs='?', metavar='TARGET',
                        help=f"目标点名（如 L4）；{MODE_RANDOM} = 随机巡航；"
                             f"{MODE_ORIGIN} = 把当前起点当目标点下发；"
                             f"{MODE_RECOVER} = 直线脱困")
    parser.add_argument('point', nargs='?', metavar='POINT',
                        help=f'仅 {MODE_RECOVER} 模式：直线开向该点位，不给则取离车最近的点位')
    parser.add_argument('-d', '--differ', action='store_true',
                        help=f'{MODE_RANDOM} 模式：随机到的目标点不能是当前起点')
    parser.add_argument('-f', '--keep-going', action='store_true',
                        help=f'{MODE_RANDOM} 模式：导航失败也继续下一个随机目标（默认失败即退出）')
    parser.add_argument('--interval', type=float, default=1.0,
                        help='随机巡航里两次下发之间的间隔秒数（默认 1.0）')
    parser.add_argument('--targets', nargs='+', metavar='NAME',
                        help='随机巡航的候选点位，默认路网上所有点位')
    parser.add_argument('--exclude', nargs='+', metavar='NAME', default=[],
                        help='随机巡航要排除的点位，例如 charge')
    parser.add_argument('--seed', type=int, help='随机种子，便于复现')
    parser.add_argument('--list', action='store_true', help='打印路网和当前起点后退出')
    parser.add_argument('--dry-run', action='store_true', help='只规划打印，不下发导航')
    parser.add_argument('--map-assets', help='直接指定 map_assets.json，不读 map_state 话题')
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--node-tolerance', type=float, default=DISTANCE_ELEMENT_DIS,
                        help=f'离点位多近算压在点位上（默认 {DISTANCE_ELEMENT_DIS}）')
    parser.add_argument('--edge-margin', type=float, default=DIS_ALLOW_ROBOT_TO_GRAPH,
                        help=f'找最近边时的粗筛余量（默认 {DIS_ALLOW_ROBOT_TO_GRAPH}）')
    parser.add_argument('--resolution', type=float, default=DEFAULT_RESOLUTION,
                        help=f'{MODE_RECOVER}：直线离散间距 m（默认 {DEFAULT_RESOLUTION}）')
    parser.add_argument('--track-mode', type=int, default=DEFAULT_TRACK_MODE,
                        choices=(1, 2, 3),
                        help=f'{MODE_RECOVER}：TrackingPath.mode，1=DIFF 2=OMNI_TRANSLATION '
                             f'3=OMNI_TRANS_ROTATION（默认 {DEFAULT_TRACK_MODE}）')
    parser.add_argument('--target-yaw', type=float,
                        help=f'{MODE_RECOVER}：末端朝向 rad，默认取目标点位自身的 yaw')
    parser.add_argument('--no-final-rotation', action='store_true',
                        help=f'{MODE_RECOVER}：target_yaw 置 NaN，到点即停不做末端旋转')
    parser.add_argument('--padding', type=float, default=DEFAULT_PADDING,
                        help=f'{MODE_RECOVER}：发轨迹前经 {PADDING_SERVICE} 把 '
                             f'{STOP_POLYGON_NAME} 的停障膨胀设成该值 m'
                             f'（默认 {DEFAULT_PADDING}，nan=恢复 default_padding）')
    parser.add_argument('--no-wait', action='store_true',
                        help=f'{MODE_RECOVER}：发完轨迹就退出，不等 {NAVIGATION_STATUS_TOPIC}')
    parser.add_argument('--recover-timeout', type=float, default=120.0,
                        help=f'{MODE_RECOVER}：等跟踪结果的秒数，超时会调 {STOP_SERVICE} 停车')
    parser.add_argument('--tf-timeout', type=float, default=10.0, help='等 TF 的秒数')
    parser.add_argument('--map-timeout', type=float, default=10.0, help='等 map_state 的秒数')
    parser.add_argument('--server-timeout', type=float, default=10.0, help='等 action server 的秒数')
    args = parser.parse_args(argv)

    if not args.list and not args.target:
        parser.error(f'要给一个目标点，或 {MODE_RANDOM} / {MODE_ORIGIN}，或 --list')
    if (args.differ or args.keep_going) and args.target != MODE_RANDOM:
        parser.error(f'-d / -f 只在 {MODE_RANDOM} 模式下有意义')
    if args.point and args.target != MODE_RECOVER:
        parser.error(f'第二个位置参数只在 {MODE_RECOVER} 模式下有意义'
                     f'（{MODE_RECOVER} POINT）')
    if args.target_yaw is not None and args.no_final_rotation:
        parser.error('--target-yaw 和 --no-final-rotation 只能二选一')
    return args


def main(argv=None):
    args = parse_args(argv)
    if args.seed is not None:
        random.seed(args.seed)

    rclpy.init()
    node = GraphNav(args)
    try:
        if args.list:
            node.show()
            return 0

        if args.target == MODE_RECOVER:
            return 0 if node.recover(args.point) else 1

        if not args.dry_run and not node.wait_server(args.server_timeout):
            node.get_logger().error(f"等不到 action server /{ACTION_NAME}——schedule_planner 没起来？")
            return 1

        if args.target == MODE_RANDOM:
            return 0 if node.run_random() else 1
        if args.target == MODE_ORIGIN:
            return 0 if node.goto() else 1
        return 0 if node.goto(args.target) else 1
    except KeyboardInterrupt:
        node.get_logger().info('退出')
    except RuntimeError as e:
        node.get_logger().error(str(e))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
