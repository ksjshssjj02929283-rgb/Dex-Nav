#!/usr/bin/env python3
"""TrackingPath 轨迹生成节点（测试用）。

发布 nav_interfaces/TrackingPath 到 ~/tracking_path，供 path_follower 的
onTrackingPath 回调测试段级跟踪参数（mode/backward/target_yaw/
hold_direction/max_speed/padding）。

几何可选三次贝塞尔或直线，按弧长等距采样（沿用 bezier_path_generator 的采样逻辑）。
交替发布正向 / 反向几何（路线反向）：每周期翻转点序，P0->P3 与 P3->P0 轮流；
backward 与所有段级参数在两方向间保持不变（由参数统一指定）。
poses 朝向置单位四元数（新消息约定朝向无意义，目标朝向走顶层 target_yaw）。

参数（ROS 参数，可 --ros-args -p 覆盖）：
  # 几何
  path_type     : 'bezier' / 'line'（默认 'bezier'）
  control_points: bezier 8 个 [x0,y0,x1,y1,x2,y2,x3,y3]；
                  line 4 个 [x0,y0,x1,y1]（默认贝塞尔控制点）
  resolution    : 弧长采样间距 m（默认 0.005）
  frame_id      : 坐标系（默认 'odom'，需与 path_follower 一致）
  base_pose     : [x, y, yaw] 基准位姿变换（默认 [0,0,0]）
  # 段级跟踪参数
  mode          : 1=DIFF / 2=OMNI_TRANSLATION / 3=OMNI_TRANS_ROTATION（默认 1）
  backward      : 0=FORWARD / 1=REVERSE（默认 0）
  target_yaw    : 到点目标朝向 rad；NaN=到点不旋转（默认 NaN）
  hold_direction: mode2 保持朝向 rad；NaN=保持当前（默认 NaN）
  max_speed     : 段限速 m/s；NaN=不限（默认 NaN）
  padding       : 车体膨胀 m；NaN=默认（默认 NaN）
  # 发布
  tracking_path_topic: 发布话题（默认 '/tracking_path'，与 path_follower ~/tracking_path 对接）
  alternate     : 是否交替正反几何（默认 True）
  republish     : 是否定时重发（默认 True）
  period        : 重发周期 s（默认 20.0，按路径时长调小）

target_yaw 在正向/反向两腿间取同一值：NaN 则两腿都跳过末段旋转，
设值则两腿到点都旋转到该朝向。

示例：
  ros2 run path_follower tracking_path_generator.py \\
    --ros-args -p path_type:=line -p control_points:="[0,0,2,0]" \\
    -p mode:=1 -p backward:=1
"""

import math
from typing import List

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_interfaces.msg import TrackingPath
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


def _dyn() -> ParameterDescriptor:
    """允许 CLI 用整数字面量覆盖浮点参数（如 period:=2 / control_points:=[0,0,2,0]），
    读取时统一 cast 为 float。"""
    return ParameterDescriptor(dynamic_typing=True)


# ---------------------------------------------------------------------------
# 几何采样（沿用 bezier_path_generator）
# ---------------------------------------------------------------------------
def bezier_point(t: float, p0, p1, p2, p3) -> np.ndarray:
    """三次贝塞尔曲线上的点。"""
    u = 1.0 - t
    return (u ** 3) * p0 + 3.0 * (u ** 2) * t * p1 + 3.0 * u * (t ** 2) * p2 + (t ** 3) * p3


def sample_bezier_arc_length(p0, p1, p2, p3, resolution: float,
                             samples_per_segment: int = 200) -> List[np.ndarray]:
    """按弧长等距采样三次贝塞尔曲线，返回 [xy, ...]，相邻点弧长约 resolution。"""
    ts = np.linspace(0.0, 1.0, samples_per_segment + 1)
    pts = np.array([bezier_point(t, p0, p1, p2, p3) for t in ts])
    seg = np.diff(pts, axis=0)
    seg_len = np.hypot(seg[:, 0], seg[:, 1])
    cum = np.concatenate(([0.0], np.cumsum(seg_len)))
    total = cum[-1]
    if total <= 0.0:
        # 退化（控制点重合）：返回起点
        return [bezier_point(0.0, p0, p1, p2, p3)]
    n = max(2, int(math.ceil(total / max(resolution, 1e-6))))
    target_s = np.linspace(0.0, total, n)
    sampled_t = np.interp(target_s, cum, ts)
    return [bezier_point(t, p0, p1, p2, p3) for t in sampled_t]


def sample_line(p0, p1, resolution: float) -> List[np.ndarray]:
    """等距采样直线段 p0->p1，返回 [xy, ...]。"""
    diff = p1 - p0
    dist = float(np.hypot(diff[0], diff[1]))
    if dist <= 0.0:
        return [p0.copy(), p1.copy()]
    n = max(2, int(math.ceil(dist / max(resolution, 1e-6))))
    ts = np.linspace(0.0, 1.0, n)
    return [p0 + t * diff for t in ts]


def identity_quaternion() -> Quaternion:
    """单位四元数（poses 朝向无意义，统一置此）。"""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = 0.0
    q.w = 1.0
    return q


class TrackingPathGenerator(Node):
    """生成并发布 TrackingPath 的节点，交替正向/反向几何。"""

    def __init__(self) -> None:
        super().__init__('tracking_path_generator')

        # ---------------- 声明参数 ----------------
        # 几何
        self.declare_parameter('path_type', 'bezier')
        self.declare_parameter(
            'control_points',
            [0.0, 0.0, 1.0, 0.5, 2.0, 0.5, 3.0, 0.0],
            descriptor=_dyn())
        self.declare_parameter('resolution', 0.005, descriptor=_dyn())
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('base_pose', [0.0, 0.0, 0.0], descriptor=_dyn())
        # 段级跟踪参数
        self.declare_parameter('mode', 1, descriptor=_dyn())
        self.declare_parameter('backward', 0, descriptor=_dyn())
        self.declare_parameter('target_yaw', float('nan'), descriptor=_dyn())
        self.declare_parameter('hold_direction', float('nan'), descriptor=_dyn())
        self.declare_parameter('max_speed', float('nan'), descriptor=_dyn())
        self.declare_parameter('padding', float('nan'), descriptor=_dyn())
        # 发布
        self.declare_parameter('tracking_path_topic', '/tracking_path')
        self.declare_parameter('alternate', True)
        self.declare_parameter('republish', True)
        self.declare_parameter('period', 20.0, descriptor=_dyn())

        # ---------------- 读取 + 校验 ----------------
        self.path_type = str(self.get_parameter('path_type').value)
        cp = list(self.get_parameter('control_points').value)
        if self.path_type == 'line':
            if len(cp) != 4:
                raise ValueError(
                    f"path_type=line 需要 4 个控制点 [x0,y0,x1,y1]，当前 {len(cp)} 个")
        else:  # bezier
            if len(cp) != 8:
                raise ValueError(
                    f"path_type=bezier 需要 8 个控制点 [x0,y0,...,x3,y3]，当前 {len(cp)} 个")
        pts = [np.array([float(cp[i]), float(cp[i + 1])], dtype=float)
               for i in range(0, len(cp), 2)]

        # base_pose 局部 -> 全局变换
        bp = list(self.get_parameter('base_pose').value)
        if len(bp) != 3:
            raise ValueError(f"base_pose 需要 3 个数 [x,y,yaw]，当前 {len(bp)} 个")
        bx, by, byaw = float(bp[0]), float(bp[1]), float(bp[2])
        cos_b, sin_b = math.cos(byaw), math.sin(byaw)

        def transform(pt: np.ndarray) -> np.ndarray:
            return np.array([bx + cos_b * pt[0] - sin_b * pt[1],
                             by + sin_b * pt[0] + cos_b * pt[1]])

        pts = [transform(p) for p in pts]

        self.resolution = float(self.get_parameter('resolution').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        # 段级参数
        self.mode = int(self.get_parameter('mode').value)
        if self.mode not in (TrackingPath.MODE_DIFF,
                             TrackingPath.MODE_OMNI_TRANSLATION,
                             TrackingPath.MODE_OMNI_TRANS_ROTATION):
            raise ValueError(f"mode 必须为 1/2/3，当前 {self.mode}")
        self.backward = int(self.get_parameter('backward').value)
        if self.backward not in (TrackingPath.FORWARD, TrackingPath.REVERSE):
            raise ValueError(f"backward 必须为 0/1，当前 {self.backward}")
        if self.backward == TrackingPath.REVERSE and self.mode != TrackingPath.MODE_DIFF:
            self.get_logger().warn(
                f"backward=REVERSE 在 mode={self.mode}（非 DIFF）下将被 path_follower 忽略")
        self.target_yaw = float(self.get_parameter('target_yaw').value)
        self.hold_direction = float(self.get_parameter('hold_direction').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.padding = float(self.get_parameter('padding').value)

        # 发布参数
        self.tracking_path_topic = str(self.get_parameter('tracking_path_topic').value)
        self.alternate = bool(self.get_parameter('alternate').value)
        self.republish = bool(self.get_parameter('republish').value)
        self.period = float(self.get_parameter('period').value)

        # ---------------- 采样 ----------------
        self._positions = self._sample(pts)

        # ---------------- 发布者（latched，迟到订阅者也能拿到）----------------
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(TrackingPath, self.tracking_path_topic, qos)

        # ---------------- 预生成正/反消息 ----------------
        self.msg_forward = self._build(reverse=False)
        self.msg_reverse = self._build(reverse=True)
        self._publish_forward_next = True

        self.get_logger().info(
            f"TrackingPath ready: type={self.path_type} poses={len(self._positions)} "
            f"mode={self.mode} backward={self.backward} "
            f"target_yaw={'nan' if math.isnan(self.target_yaw) else f'{self.target_yaw:.4f}'} "
            f"hold_dir={'nan' if math.isnan(self.hold_direction) else 'set'} "
            f"max_speed={'nan' if math.isnan(self.max_speed) else 'set'} "
            f"padding={'nan' if math.isnan(self.padding) else 'set'} "
            f"alternate={self.alternate}")

        # 首发立即发布 + （可选）按 period 周期重发。
        # 启动延迟由 launch.TimerAction 控制节点启动时机，脚本不再自持 start_delay 计时器。
        self._publish_alternating()
        if self.republish:
            self._republish_timer = self.create_timer(
                max(self.period, 1e-6), self._republish)

    # ------------------------------------------------------------------
    def _sample(self, pts: List[np.ndarray]) -> List[np.ndarray]:
        if self.path_type == 'line':
            return sample_line(pts[0], pts[1], self.resolution)
        return sample_bezier_arc_length(pts[0], pts[1], pts[2], pts[3], self.resolution)

    def _build(self, reverse: bool) -> TrackingPath:
        positions = list(self._positions)
        if reverse:
            positions = list(reversed(positions))  # 路线反向：点序倒置

        msg = TrackingPath()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        for pos in positions:
            p = Pose()
            p.position = Point(x=float(pos[0]), y=float(pos[1]), z=0.0)
            p.orientation = identity_quaternion()
            msg.poses.append(p)
        # 段级参数两腿一致（backward 不动、target_yaw 同值）
        msg.mode = self.mode
        msg.backward = self.backward
        msg.hold_direction = self.hold_direction
        msg.max_speed = self.max_speed
        msg.padding = self.padding
        msg.target_yaw = self.target_yaw
        return msg

    def _publish_alternating(self) -> None:
        if self.alternate:
            msg = self.msg_forward if self._publish_forward_next else self.msg_reverse
            direction = 'forward' if self._publish_forward_next else 'reverse'
            self._publish_forward_next = not self._publish_forward_next
        else:
            msg = self.msg_forward
            direction = 'forward'
        # 仅刷新 header.stamp（poses 无逐点 header）
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        self.get_logger().info(f"TrackingPath published ({direction}, {len(msg.poses)} pts).")

    def _republish(self) -> None:
        self._publish_alternating()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackingPathGenerator()
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
