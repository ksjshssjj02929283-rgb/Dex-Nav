#!/usr/bin/env python3
"""折线拐角四次/五次贝塞尔平滑轨迹生成节点。

订阅：无。
发布：
  - ~/path (nav_msgs/Path)  平滑后的整段参考轨迹，供 path_follower 跟踪。

原理（锚点方案与 scripts/curvature.py 中的「四次」「五次」一致）：
  对折线 ... -> P_prev -> P_corner -> P_next -> ... 的每个内部拐角，
  在两条边上各退让 d = (bezier_degree - 2) * anchor_spacing 得到接入点 A、接出点 B：
      A = P_corner + d * unit(P_prev - P_corner)
      B = P_corner + d * unit(P_next - P_corner)
  五次：6 个控制点（锚点）取两条边上的三等分点，每边 3 个、间距 d/3：
      [A, A+(P-A)/3, A+2(P-A)/3, P+(B-P)/3, P+2(B-P)/3, B]
  四次：5 个控制点取两条边的中点并保留拐角点 P，锚点间距 d/2：
      [A, (A+P)/2, P, (P+B)/2, B]
  两种方案都满足首末各三点共线，曲线在 A、B 处与原直线段一阶且二阶相切
  （kappa(0) = kappa(1) = 0），拼接后曲率连续、无冲击。
  同样的退让距离 d 下，四次的峰值曲率明显高于五次（90° 直角、d=0.6m 实测：
  四次 kappa_max≈3.54 / R_min≈0.28m，五次 kappa_max≈2.24 / R_min≈0.45m），
  换用四次时需相应加大 anchor_spacing 才能保持相同的最小转弯半径。

  最终轨迹 = 直线段(起点->A1) + 贝塞尔(A1->B1) + 直线段(B1->A2) + ... + 直线段(Bn->终点)，
  再按弧长等距重采样到 resolution。

参数（ROS 参数，可 --ros-args -p 覆盖）：
  waypoints       : [x0,y0, x1,y1, ...] 原始折线顶点，至少 2 个点（默认 [0,0, 2,0, 2,2]）
  bezier_degree   : 拐角贝塞尔阶数，4 或 5（默认 5）
  anchor_spacing  : 贝塞尔锚点间距 m（默认 0.2），拐角退让距离 = (bezier_degree - 2) * 该值
  resolution      : 弧长采样间距 m（默认 0.005）
  frame_id        : 坐标系（默认 'map'，需与 path_follower 的 odom_frame/全局系一致）
  base_pose       : [x, y, yaw] 基准位姿，折线顶点从局部坐标变换到该位姿坐标系（默认 [0,0,0]）
  yaw_mode        : 'tangent' 每点朝向取切向角；'linear' 在 start_yaw->goal_yaw 间线性均布
                    （全向底盘可用 linear 边平移边转）
  start_yaw/goal_yaw : yaw_mode='linear' 时的起止朝向（rad）
  republish       : 是否定时重发轨迹（默认 True，便于反复测试）
  period          : 重发周期 s（默认 40.0）
  alternate_direction : 重发时正/反向交替（默认 True）
"""

import math
from typing import List, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


# ---------------------------------------------------------------------------
# 贝塞尔基础工具（与 scripts/curvature.py 同一套公式，去掉 scipy 依赖）
# ---------------------------------------------------------------------------
def _binomial(n: int, k: int) -> float:
    return float(math.comb(n, k))


def bezier_points(control_points: np.ndarray, u: np.ndarray) -> np.ndarray:
    """任意阶贝塞尔曲线批量求值。control_points: (n+1, 2); u: (m,) -> (m, 2)。"""
    cp = np.asarray(control_points, dtype=float)
    n = len(cp) - 1
    u = np.atleast_1d(np.asarray(u, dtype=float))[:, None]
    k = np.arange(n + 1)
    basis = np.array([_binomial(n, i) for i in k]) * (u ** k) * (1.0 - u) ** (n - k)
    return basis @ cp


def bezier_derivative_cp(control_points: np.ndarray, order: int = 1) -> np.ndarray:
    """逐阶前向差分降阶，返回 order 阶导数曲线的控制点。"""
    cp = np.asarray(control_points, dtype=float)
    n = len(cp) - 1
    coeff = 1.0
    for r in range(order):
        cp = np.diff(cp, axis=0)
        coeff *= (n - r)
    return coeff * cp


def bezier_max_curvature(control_points: np.ndarray, num_samples: int = 500) -> Tuple[float, float]:
    """返回 (峰值曲率, 弧长)，用于日志中确认拐角是否超出底盘转弯半径能力。"""
    d1 = bezier_derivative_cp(control_points, 1)
    d2 = bezier_derivative_cp(control_points, 2)
    u = np.linspace(0.0, 1.0, num_samples)
    r1 = bezier_points(d1, u)
    r2 = bezier_points(d2, u)
    speed = np.linalg.norm(r1, axis=1)
    cross = np.abs(r1[:, 0] * r2[:, 1] - r1[:, 1] * r2[:, 0])
    kappa = cross / np.maximum(speed ** 3, 1e-14)
    arc = float(np.sum((speed[1:] + speed[:-1]) / 2.0 * np.diff(u)))
    return float(np.max(kappa)), arc


# ---------------------------------------------------------------------------
# 拐角平滑
# ---------------------------------------------------------------------------
SUPPORTED_DEGREES = (4, 5)


def check_degree(degree: int) -> int:
    """校验贝塞尔阶数，只支持 curvature.py 里给出锚点方案的 4 次与 5 次。"""
    if int(degree) not in SUPPORTED_DEGREES:
        raise ValueError(f"bezier_degree 只支持 {SUPPORTED_DEGREES}，当前 {degree}")
    return int(degree)


def offset_factor(degree: int) -> float:
    """退让距离 / 锚点间距：每条边上的锚点间隔数（4 次 -> 2，5 次 -> 3）。"""
    return float(check_degree(degree) - 2)


def corner_control_points(p_prev: np.ndarray,
                          p_corner: np.ndarray,
                          p_next: np.ndarray,
                          offset_in: float,
                          offset_out: float,
                          degree: int = 5) -> np.ndarray:
    """按 curvature.py 的「四次」/「五次」方案构造拐角控制点（5 或 6 个）。"""
    check_degree(degree)
    dir_in = p_prev - p_corner
    dir_out = p_next - p_corner
    dir_in = dir_in / max(np.linalg.norm(dir_in), 1e-12)
    dir_out = dir_out / max(np.linalg.norm(dir_out), 1e-12)

    a = p_corner + offset_in * dir_in      # 接入点（曲线起点）
    b = p_corner + offset_out * dir_out    # 接出点（曲线终点）
    if degree == 4:
        # 中间锚点 = 两条边的中点 + 拐角点本身，两端三点共线 -> 端点曲率为 0
        return np.array([
            a,
            0.5 * (a + p_corner),
            p_corner,
            0.5 * (p_corner + b),
            b,
        ])
    return np.array([
        a,
        a + (p_corner - a) / 3.0,
        a + 2.0 * (p_corner - a) / 3.0,
        p_corner + (b - p_corner) / 3.0,
        p_corner + 2.0 * (b - p_corner) / 3.0,
        b,
    ])


def build_smoothed_polyline(waypoints: np.ndarray,
                            anchor_spacing: float,
                            dense_step: float,
                            degree: int = 5) -> Tuple[np.ndarray, List[np.ndarray]]:
    """折线 -> 拐角四次/五次贝塞尔平滑后的稠密折线（不带朝向的旧接口）。"""
    pts, _yaw, ctrl = build_smoothed_polyline_with_yaw(
        waypoints, anchor_spacing, dense_step, degree)
    return pts, ctrl


def build_smoothed_polyline_with_yaw(waypoints: np.ndarray,
                                     anchor_spacing: float,
                                     dense_step: float,
                                     degree: int = 5
                                     ) -> Tuple[np.ndarray, np.ndarray, List[np.ndarray]]:
    """折线 -> 拐角四次/五次贝塞尔平滑后的稠密折线 + 每点**解析切向角**。

    切向不是相邻点差分出来的，而是解析求导：直线段取端点方向，贝塞尔段取
    theta(u) = atan2(B'_y(u), B'_x(u))，B' 由控制点一次降阶（hodograph）得到。
    差分求切向再差分求曲率，对位置扰动的放大系数是 4/ds^2（5mm 点距下约 1.6e5），
    解析切向则完全没有这个问题——下游 path_follower 据此直接算 kappa = dtheta/ds。

    返回 (dense_points (N,2), dense_yaw (N,) 已 unwrap, 每个拐角的控制点列表)。
    退让距离 d = (degree - 2) * anchor_spacing；若相邻边过短，按「每条边最多让出
    一半长度」收缩，保证相邻两个拐角的贝塞尔段不重叠（此时该角的锚点间距同比缩小）。
    """
    wps = np.asarray(waypoints, dtype=float)
    n = len(wps)
    if n < 2:
        raise ValueError(f"waypoints 至少需要 2 个点，当前 {n} 个")

    nominal = offset_factor(degree) * anchor_spacing
    seg_len = np.linalg.norm(np.diff(wps, axis=0), axis=1)  # (n-1,)

    # 每个内部拐角在前/后两条边上的实际退让距离
    offsets: List[Tuple[float, float]] = []
    for i in range(1, n - 1):
        # 首尾边可整段使用，中间边被两个拐角共享，各取一半
        in_budget = seg_len[i - 1] * (1.0 if i == 1 else 0.5)
        out_budget = seg_len[i] * (1.0 if i == n - 2 else 0.5)
        d = min(nominal, in_budget, out_budget)
        offsets.append((d, d))

    dense: List[np.ndarray] = []
    dense_yaw: List[np.ndarray] = []
    ctrl_list: List[np.ndarray] = []
    cursor = wps[0]  # 当前直线段的起点

    for idx, i in enumerate(range(1, n - 1)):
        off_in, off_out = offsets[idx]
        cp = corner_control_points(wps[i - 1], wps[i], wps[i + 1], off_in, off_out, degree)
        ctrl_list.append(cp)
        a, b = cp[0], cp[-1]

        seg = _dense_line(cursor, a, dense_step)
        dense.append(seg)
        dense_yaw.append(np.full(len(seg), _line_yaw(cursor, a)))

        # 贝塞尔按 u 细分（后续统一按弧长重采样，这里只要足够密）
        chord = float(np.sum(np.linalg.norm(np.diff(cp, axis=0), axis=1)))
        m = max(64, int(math.ceil(chord / max(dense_step, 1e-6))) + 1)
        u = np.linspace(0.0, 1.0, m)
        dense.append(bezier_points(cp, u))
        d1 = bezier_points(bezier_derivative_cp(cp, 1), u)
        dense_yaw.append(np.arctan2(d1[:, 1], d1[:, 0]))
        cursor = b

    seg = _dense_line(cursor, wps[-1], dense_step)
    dense.append(seg)
    dense_yaw.append(np.full(len(seg), _line_yaw(cursor, wps[-1])))

    pts = np.vstack(dense)
    yaw = np.unwrap(np.concatenate(dense_yaw))
    # 去掉拼接处的重复点
    keep = np.concatenate(([True], np.linalg.norm(np.diff(pts, axis=0), axis=1) > 1e-9))
    return pts[keep], yaw[keep], ctrl_list


def _line_yaw(p0: np.ndarray, p1: np.ndarray) -> float:
    """直线段切向角；两点重合时无定义，返回 0（调用方保证不会用到）。"""
    d = p1 - p0
    return float(np.arctan2(d[1], d[0])) if np.linalg.norm(d) > 1e-12 else 0.0


def _dense_line(p0: np.ndarray, p1: np.ndarray, step: float) -> np.ndarray:
    """两点间的稠密直线段（含首尾）。长度为 0 时只返回起点。"""
    length = float(np.linalg.norm(p1 - p0))
    if length < 1e-9:
        return p0.reshape(1, 2)
    m = max(2, int(math.ceil(length / max(step, 1e-6))) + 1)
    t = np.linspace(0.0, 1.0, m)[:, None]
    return p0 + t * (p1 - p0)


def resample_by_arc_length(points: np.ndarray, resolution: float,
                           values: np.ndarray = None):
    """按弧长等距重采样（线性插值），保证相邻点间距 ≈ resolution。

    values 给出时（如已 unwrap 的切向角），在同一张弧长网格上一并插值并返回，
    这样朝向和位置严格一一对应，不需要事后再从重采样点差分反推。
    """
    pts = np.asarray(points, dtype=float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum = np.concatenate(([0.0], np.cumsum(seg)))
    total = float(cum[-1])
    if total <= 0.0:
        return pts[:1] if values is None else (pts[:1], np.asarray(values)[:1])
    m = max(2, int(math.ceil(total / max(resolution, 1e-6))) + 1)
    target = np.linspace(0.0, total, m)
    x = np.interp(target, cum, pts[:, 0])
    y = np.interp(target, cum, pts[:, 1])
    out = np.column_stack([x, y])
    if values is None:
        return out
    return out, np.interp(target, cum, np.asarray(values, dtype=float))


# ---------------------------------------------------------------------------
# 朝向
# ---------------------------------------------------------------------------
def yaw_to_quaternion(yaw: float) -> Quaternion:
    """yaw (rad) -> 四元数（仅绕 z 轴）。"""
    half = 0.5 * yaw
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def tangent_yaws(points: np.ndarray) -> np.ndarray:
    """按相邻点差分求切向角，并 unwrap 保证连续（拐角处平滑地转过 90°）。

    仅作为退化兜底保留：正常路径请用 build_smoothed_polyline_with_yaw 给出的解析切向，
    差分版对位置扰动的放大系数是 1/ds，5mm 点距下会把亚毫米抖动放大成可见的朝向噪声，
    再求一次曲率还要再放大一次。
    """
    d = np.gradient(points, axis=0)
    return np.unwrap(np.arctan2(d[:, 1], d[:, 0]))


def linear_yaws(start_yaw: float, goal_yaw: float, n: int) -> np.ndarray:
    """在 start_yaw -> goal_yaw 之间按点序线性均布（取最短角差，朝向单调）。"""
    if n <= 0:
        return np.zeros(0)
    if n == 1:
        return np.array([start_yaw])
    diff = math.atan2(math.sin(goal_yaw - start_yaw), math.cos(goal_yaw - start_yaw))
    return start_yaw + diff * np.linspace(0.0, 1.0, n)


# ---------------------------------------------------------------------------
# 节点
# ---------------------------------------------------------------------------
class CornerBezierPathGenerator(Node):
    """折线拐角四次/五次贝塞尔平滑 + 整段轨迹发布。"""

    def __init__(self) -> None:
        super().__init__('corner_bezier_path_generator')

        self.declare_parameter('waypoints', [0.0, 0.0, 2.0, 0.0, 2.0, 2.0])
        self.declare_parameter('bezier_degree', 5)
        self.declare_parameter('anchor_spacing', 0.2)
        self.declare_parameter('resolution', 0.005)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('base_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('yaw_mode', 'tangent')
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('goal_yaw', math.pi / 2.0)
        self.declare_parameter('republish', True)
        self.declare_parameter('period', 40.0)
        self.declare_parameter('alternate_direction', True)

        raw = list(self.get_parameter('waypoints').value)
        if len(raw) < 4 or len(raw) % 2 != 0:
            raise ValueError(
                f"waypoints 必须为偶数个且至少 2 个点 [x0,y0, x1,y1, ...]，当前 {len(raw)} 个数")
        wps = np.array(raw, dtype=float).reshape(-1, 2)

        bp = list(self.get_parameter('base_pose').value)
        if len(bp) != 3:
            raise ValueError(f"base_pose 必须为 3 个数 [x, y, yaw]，当前 {len(bp)} 个")
        bx, by, byaw = float(bp[0]), float(bp[1]), float(bp[2])
        cos_b, sin_b = math.cos(byaw), math.sin(byaw)
        rot = np.array([[cos_b, -sin_b], [sin_b, cos_b]])
        self.waypoints = wps @ rot.T + np.array([bx, by])

        self.bezier_degree = check_degree(self.get_parameter('bezier_degree').value)
        self.anchor_spacing = float(self.get_parameter('anchor_spacing').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.yaw_mode = str(self.get_parameter('yaw_mode').value).lower()
        if self.yaw_mode not in ('tangent', 'linear'):
            raise ValueError(f"yaw_mode 只能是 'tangent' 或 'linear'，当前 {self.yaw_mode!r}")
        self.start_yaw = float(self.get_parameter('start_yaw').value) + byaw
        self.goal_yaw = float(self.get_parameter('goal_yaw').value) + byaw
        self.republish = bool(self.get_parameter('republish').value)
        self.period = float(self.get_parameter('period').value)
        self.alternate = bool(self.get_parameter('alternate_direction').value)

        # ---- 生成轨迹 ----
        # 贝塞尔离散步长要明显细于重采样间距，否则折线化的曲线会给切向留下台阶。
        dense, dense_yaw, ctrl_list = build_smoothed_polyline_with_yaw(
            self.waypoints, self.anchor_spacing, min(self.resolution / 5.0, 1e-3),
            self.bezier_degree)
        self.positions, self.tangent_yaw = resample_by_arc_length(
            dense, self.resolution, dense_yaw)

        # 发布者：latched，迟到的订阅者也能拿到轨迹
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(Path, '~/path', qos)

        self.path_msg_forward = self._build_path(reverse=False)
        self.path_msg_reverse = self._build_path(reverse=True)
        self._publish_forward_next = True

        total_len = float(np.sum(np.linalg.norm(np.diff(self.positions, axis=0), axis=1)))
        self.get_logger().info(
            f"Corner-bezier trajectory: {len(self.positions)} points, "
            f"length={total_len:.3f}m, resolution={self.resolution}m, "
            f"degree={self.bezier_degree}, "
            f"anchor_spacing={self.anchor_spacing}m, yaw_mode={self.yaw_mode}, "
            f"frame={self.frame_id}, waypoints={self.waypoints.tolist()}")
        for i, cp in enumerate(ctrl_list):
            kappa, arc = bezier_max_curvature(cp)
            radius = 1.0 / kappa if kappa > 1e-9 else float('inf')
            self.get_logger().info(
                f"  corner[{i}] offset={np.linalg.norm(cp[-1] - self.waypoints[i + 1]):.3f}m "
                f"arc={arc:.3f}m kappa_max={kappa:.3f} (R_min={radius:.3f}m) "
                f"anchors={np.round(cp, 4).tolist()}")

        self._publish_alternating()
        if self.republish:
            self._republish_timer = self.create_timer(
                max(self.period, 1e-6), self._publish_alternating)

    def _build_path(self, reverse: bool = False) -> Path:
        positions = self.positions[::-1] if reverse else self.positions
        n = len(positions)

        if self.yaw_mode == 'tangent':
            # 解析切向（随位置一起按弧长重采样，逐点对应）。反向发布时轨迹倒序，
            # 行进方向随之反向，切向加 pi。
            yaws = self.tangent_yaw[::-1] + math.pi if reverse else self.tangent_yaw
        elif reverse:
            yaws = linear_yaws(self.goal_yaw, self.start_yaw, n)
        else:
            yaws = linear_yaws(self.start_yaw, self.goal_yaw, n)

        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.poses = []
        for pos, yaw in zip(positions, yaws):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = Pose()
            ps.pose.position = Point(x=float(pos[0]), y=float(pos[1]), z=0.0)
            ps.pose.orientation = yaw_to_quaternion(float(yaw))
            msg.poses.append(ps)
        return msg

    def _publish_alternating(self) -> None:
        if self.alternate:
            msg = self.path_msg_forward if self._publish_forward_next else self.path_msg_reverse
            direction = "forward" if self._publish_forward_next else "reverse"
            self._publish_forward_next = not self._publish_forward_next
        else:
            msg, direction = self.path_msg_forward, "forward"

        stamp = self.get_clock().now().to_msg()
        msg.header.stamp = stamp
        for ps in msg.poses:
            ps.header.stamp = stamp
        self.path_pub.publish(msg)
        self.get_logger().info(f"Trajectory published ({direction}).")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CornerBezierPathGenerator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
