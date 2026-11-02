#!/usr/bin/env python3
"""三次贝塞尔曲线轨迹生成节点。

订阅：无。
发布：
  - ~/path (nav_msgs/Path)  带朝向的参考轨迹，供 path_follower 跟踪。

每个采样点的朝向不再取切向角，而是在「起始朝向 → 目标朝向」之间
按点序线性均匀分布，并按最短角差 unwrap（保证朝向单调变化），
转成四元数填入 pose.orientation —— 与 path_follower 用
yawFromQuaternion 解析每个点朝向的方式一致。
这样全向底盘可一边沿曲线平移、一边匀速旋转朝向。

三次贝塞尔：
  B(t)  = (1-t)^3 P0 + 3(1-t)^2 t P1 + 3(1-t) t^2 P2 + t^3 P3
  B'(t) = 3(1-t)^2 (P1-P0) + 6(1-t) t (P2-P1) + 3 t^2 (P3-P2)

参数（ROS 参数，可 --ros-args -p 覆盖）：
  control_points : [x0,y0, x1,y1, x2,y2, x3,y3, start_yaw, goal_yaw]，长度必须为 10
                   前 8 个为贝塞尔控制点，最后 2 个为起始/目标朝向角（rad）
  resolution     : 弧长采样间距 m（默认 0.005）
  frame_id       : 坐标系（默认 'odom'，需与 path_follower 的 odom_frame 一致）
  republish      : 是否定时重发轨迹（默认 True，便于反复测试）
  period        : 重发周期 s（默认 20.0，即每 20 秒）
  base_pose      : [x, y, yaw] 基准位姿，控制点将从局部坐标变换到该位姿坐标系（默认 [0,0,0]）
"""

import math
from typing import List, Tuple

import numpy as np
import matplotlib.pyplot as plt
import rclpy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


def bezier_point(t: float, p0, p1, p2, p3) -> np.ndarray:
    """三次贝塞尔曲线上的点。"""
    u = 1.0 - t
    return (u ** 3) * p0 + 3.0 * (u ** 2) * t * p1 + 3.0 * u * (t ** 2) * p2 + (t ** 3) * p3


def bezier_tangent(t: float, p0, p1, p2, p3) -> np.ndarray:
    """三次贝塞尔曲线的切向量 B'(t)。"""
    u = 1.0 - t
    return (3.0 * (u ** 2) * (p1 - p0)
            + 6.0 * u * t * (p2 - p1)
            + 3.0 * (t ** 2) * (p3 - p2))


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """yaw (rad) -> 四元数 (仅绕 z 轴)。"""
    half = 0.5 * yaw
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def sample_bezier_arc_length(
    p0, p1, p2, p3, resolution: float, samples_per_segment: int = 200
) -> List[np.ndarray]:
    """按弧长等距采样三次贝塞尔曲线。

    返回 [position_xy, ...]，相邻点弧长间距约为 resolution。
    用细分 t 描出整条曲线，再按累积弧长重采样，保证等距。
    朝向不在此处计算，由调用方按点序均匀分布填入。
    """
    # 1) 细分 t，求每段弧长
    ts = np.linspace(0.0, 1.0, samples_per_segment + 1)
    pts = np.array([bezier_point(t, p0, p1, p2, p3) for t in ts])
    seg = np.diff(pts, axis=0)
    seg_len = np.hypot(seg[:, 0], seg[:, 1])
    cum = np.concatenate(([0.0], np.cumsum(seg_len)))
    total = cum[-1]

    # 2) 按弧长间距取目标 s，反插值求对应 t
    if total <= 0.0:
        # 退化（控制点重合）：返回端点
        return [bezier_point(0.0, p0, p1, p2, p3)]

    n = max(2, int(math.ceil(total / max(resolution, 1e-6))))
    target_s = np.linspace(0.0, total, n)
    # cum 单调（严格递增段可能因重合点持平），用插值
    sampled_t = np.interp(target_s, cum, ts)
    result = [bezier_point(t, p0, p1, p2, p3) for t in sampled_t]
    return result


def angle_unwrap_diff(start_yaw: float, goal_yaw: float) -> float:
    """返回从 start_yaw 到 goal_yaw 的最短有向角差（|diff| <= pi），保证朝向单调。"""
    return math.atan2(math.sin(goal_yaw - start_yaw), math.cos(goal_yaw - start_yaw))


def linear_yaw_distribution(start_yaw: float, goal_yaw: float, n: int) -> List[float]:
    """在 [start_yaw, start_yaw+diff] 上按点序线性均匀分布 n 个朝向角。

    diff 取最短角差（unwrap），使朝向沿点序单调变化，
    与 path_follower 的 planHeadingTrapezoidal（朝向单调假设）一致。
    """
    if n <= 0:
        return []
    diff = angle_unwrap_diff(start_yaw, goal_yaw)
    return [start_yaw + diff * (i / max(n - 1, 1)) for i in range(n)]


class BezierPathGenerator(Node):
    """生成并发布三次贝塞尔曲线轨迹的节点。"""

    def __init__(self) -> None:
        super().__init__('bezier_path_generator')

        # 声明参数
        self.declare_parameter(
            'control_points',
            [0.0, 0.0, 1.0, 0.5, 2.0, 0.5, 3.0, 0.0, 0.0, math.pi / 2.0])
        self.declare_parameter('resolution', 0.005)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('republish', True)
        self.declare_parameter('period', 20.0)
        self.declare_parameter('base_pose', [0.0, 0.0, 0.0])  # [x, y, yaw] 基准位姿

        cp = self.get_parameter('control_points').value
        if len(cp) != 10:
            raise ValueError(
                f"control_points 必须为 10 个数 [x0,y0,...,x3,y3, start_yaw, goal_yaw]，"
                f"当前 {len(cp)} 个")
        self.p0 = np.array([cp[0], cp[1]], dtype=float)
        self.p1 = np.array([cp[2], cp[3]], dtype=float)
        self.p2 = np.array([cp[4], cp[5]], dtype=float)
        self.p3 = np.array([cp[6], cp[7]], dtype=float)
        self.start_yaw = float(cp[8])
        self.goal_yaw = float(cp[9])

        # 将控制点从局部坐标系变换到 base_pose 坐标系
        bp = self.get_parameter('base_pose').value
        if len(bp) != 3:
            raise ValueError(
                f"base_pose 必须为 3 个数 [x, y, yaw]，当前 {len(bp)} 个")
        bx, by, byaw = float(bp[0]), float(bp[1]), float(bp[2])
        cos_b, sin_b = math.cos(byaw), math.sin(byaw)

        def transform_pt(pt: np.ndarray) -> np.ndarray:
            """将局部坐标点变换到 base_pose 坐标系。"""
            return np.array([
                bx + cos_b * pt[0] - sin_b * pt[1],
                by + sin_b * pt[0] + cos_b * pt[1],
            ])

        self.p0 = transform_pt(self.p0)
        self.p1 = transform_pt(self.p1)
        self.p2 = transform_pt(self.p2)
        self.p3 = transform_pt(self.p3)
        self.start_yaw += byaw
        self.goal_yaw += byaw
        self.resolution = float(self.get_parameter('resolution').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.republish = bool(self.get_parameter('republish').value)
        self.period = float(self.get_parameter('period').value)

        # 发布者：latched，使迟到的订阅者也能拿到轨迹
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(Path, '~/path', qos)
        # remap 到 /path 后与 path_follower 对接

        # 预生成正向/反向轨迹，按周期交替发布
        self.path_msg_forward = self._build_path(reverse=False)
        self.path_msg_reverse = self._build_path(reverse=True)
        self._publish_forward_next = True
        self.get_logger().info(
            f"Bezier trajectory: {len(self.path_msg_forward.poses)} points, "
            f"resolution={self.resolution}m, frame={self.frame_id}, "
            f"control P0={self.p0.tolist()} P1={self.p1.tolist()} "
            f"P2={self.p2.tolist()} P3={self.p3.tolist()}, "
            f"start_yaw={self.start_yaw:.4f} goal_yaw={self.goal_yaw:.4f}")

        # 首发立即发布 + （可选）按 period 周期重发。
        # 启动延迟由 launch.TimerAction 控制节点启动时机，脚本不再自持 start_delay 计时器。
        self._publish_alternating()
        if self.republish:
            self._republish_timer = self.create_timer(
                max(self.period, 1e-6), self._republish)

    def _build_path(self, reverse: bool = False) -> Path:
        positions = sample_bezier_arc_length(
            self.p0, self.p1, self.p2, self.p3, self.resolution)

        # 仿真取货任务测试
        # self.start_yaw, self.goal_yaw = 1.57, 3.14
        # ps = [
        #     [np.array([-1.5, 0.8], dtype=float),
        #      np.array([-1.5, -2.0], dtype=float),
        #      np.array([1.0, -2.0], dtype=float),
        #      np.array([1.0, -5.0], dtype=float)],
        #     [np.array([1.0, -5.0], dtype=float),
        #      np.array([1.0, -6.5], dtype=float),
        #      np.array([1.0, -6.5], dtype=float),
        #      np.array([-0.7, -6.5], dtype=float)],
        # ]
        # positions = sum([sample_bezier_arc_length(*pp, 0.005) for pp in ps], [])
        # positions_ = np.array(positions)
        # plt.plot(positions_[:, 0], positions_[:, 1])
        # plt.axis('equal')

        n = len(positions)

        # 朝向按点序线性均匀分布。正向：start_yaw -> goal_yaw；
        # 反向：点序倒过来，朝向随之倒序（反向起点为 goal_yaw，终点为 start_yaw）。
        if reverse:
            positions = list(reversed(positions))
            yaws = linear_yaw_distribution(self.goal_yaw, self.start_yaw, n)
        else:
            yaws = linear_yaw_distribution(self.start_yaw, self.goal_yaw, n)

        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.poses = []
        for pos, yaw in zip(positions, yaws):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = Pose()
            ps.pose.position = Point(x=float(pos[0]), y=float(pos[1]), z=0.0)
            ps.pose.orientation = yaw_to_quaternion(yaw)
            msg.poses.append(ps)
        return msg

    def _publish_alternating(self) -> None:
        msg = self.path_msg_forward if self._publish_forward_next else self.path_msg_reverse
        direction = "forward" if self._publish_forward_next else "reverse"
        self._publish_forward_next = not self._publish_forward_next

        stamp = self.get_clock().now().to_msg()
        msg.header.stamp = stamp
        for ps in msg.poses:
            ps.header.stamp = stamp
        self.path_pub.publish(msg)
        self.get_logger().info(f"Trajectory published ({direction}).")

    def _republish(self) -> None:
        self._publish_alternating()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BezierPathGenerator()
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
