#!/usr/bin/env python3
"""Randomly send NavigateThroughGraphPoints goals alternating between two sequences.

Usage:
    ros2 run nav2_schedule_planner random_navigate_through_graph_points.py
    # or directly:
    python3 random_navigate_through_graph_points.py [--interval SEC] [--seed N]

Randomly chooses between ["L5","L6"] and ["L6","L5"] each iteration,
sends the action goal, waits for the result, then loops until Ctrl+C.

每次移动结束后记录一行到当前目录的 move_record.csv（到点精度评估）：
  - 目标点来自 /map_asset_manager/resolve_nav_points（末段末控制点 + 段 target_yaw），
    与 schedule_planner 实际跟踪的终点是同一个；action goal 里的 pose/target_yaw 被服务端忽略。
  - 到点位姿采自 TF map -> base_link（与 path_follower 的 map 闭环同源），
    result 返回后立即采样 --settle 秒（默认 1s）。
  - 该窗口内记录标准差 std_* 和极差 range_*（max-min，yaw 走圆残差），
    用来看静止时定位的抖动幅度与最大跳变。
用 plot_move_record.py 读取分析 / 绘制散点。
"""

import argparse
import csv
import math
import os
import random
import sys
from datetime import datetime

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros

from geometry_msgs.msg import Pose2D
from nav_interfaces.action import NavigateThroughGraphPoints
from nav_interfaces.srv import ResolveNavPoints


SEQUENCES = [
    ["L1"],
    ["L2"],
]

RESOLVE_SERVICE = "/map_asset_manager/resolve_nav_points"

CSV_FIELDS = [
    "timestamp",
    "path",
    "success",
    "message",
    "duration_s",
    "target_x",
    "target_y",
    "target_yaw",
    "mean_x",
    "mean_y",
    "mean_yaw",
    "std_x",
    "std_y",
    "std_yaw",
    "range_x",
    "range_y",
    "range_yaw",
    "sample_count",
    "err_x",
    "err_y",
    "err_yaw",
    "dist",
]


def yaw_from_quaternion(q) -> float:
    """四元数 -> yaw，手算避免依赖 tf_transformations（机器上不一定装）。"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a: float) -> float:
    """归一化到 (-pi, pi]。"""
    return math.atan2(math.sin(a), math.cos(a))


def mean_std(values):
    """样本均值 + 样本标准差（n<2 时标准差为 0）。"""
    n = len(values)
    if n == 0:
        return float("nan"), float("nan")
    mean = sum(values) / n
    if n < 2:
        return mean, 0.0
    var = sum((v - mean) ** 2 for v in values) / (n - 1)
    return mean, math.sqrt(var)


def value_range(values):
    """极差 max-min：标准差看的是整体离散度，极差抓的是窗口内的最大跳变。"""
    if not values:
        return float("nan")
    return max(values) - min(values)


def circular_range(angles, mean):
    """角度极差：先取对圆均值的 wrap 残差再算 max-min，避开 ±pi 跨越。"""
    if not angles:
        return float("nan")
    residuals = [wrap_angle(a - mean) for a in angles]
    return max(residuals) - min(residuals)


def circular_mean_std(angles):
    """角度圆统计：均值 atan2(sum sin, sum cos)，标准差取对圆均值的 wrap 残差 RMS。

    直接算术平均在 ±pi 附近会给出完全错误的均值和虚高的标准差。
    """
    n = len(angles)
    if n == 0:
        return float("nan"), float("nan")
    mean = math.atan2(sum(math.sin(a) for a in angles),
                      sum(math.cos(a) for a in angles))
    if n < 2:
        return mean, 0.0
    var = sum(wrap_angle(a - mean) ** 2 for a in angles) / (n - 1)
    return mean, math.sqrt(var)


class RandomNavClient(Node):
    def __init__(self, interval: float, settle: float, sample_hz: float,
                 record_path: str, map_frame: str, base_frame: str):
        super().__init__("random_navigate_through_graph_points")
        self._client = ActionClient(
            self, NavigateThroughGraphPoints, "navigate_through_graph_points"
        )
        self._interval = interval
        self._settle = settle
        self._sample_period = 1.0 / max(sample_hz, 1.0)
        self._map_frame = map_frame
        self._base_frame = base_frame

        # 到点位姿来源：TF map -> base_link，与 path_follower 的 map 闭环判定同源
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # 目标点解析：与 schedule_planner 用的是同一个服务
        self._resolve_client = self.create_client(ResolveNavPoints, RESOLVE_SERVICE)

        self._record_path = record_path
        self._csv_file = open(record_path, "a", newline="")
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=CSV_FIELDS)
        if self._csv_file.tell() == 0:
            self._csv_writer.writeheader()
            self._csv_file.flush()
        self.get_logger().info(f"Recording to {record_path}")

    def close(self):
        if self._csv_file and not self._csv_file.closed:
            self._csv_file.close()

    def wait_server(self, timeout: float = 10.0) -> bool:
        self.get_logger().info("Waiting for action server /navigate_through_graph_points...")
        return self._client.wait_for_server(timeout_sec=timeout)

    # ── 目标点解析 ──────────────────────────────────────────────────

    def resolve_target(self, ids):
        """(x, y, yaw) of the final waypoint, or None.

        末段的最后一个 control_point 即终点；朝向取该段 target_yaw
        （PathSegment: path_follower 只采信末点朝向）。
        """
        if not self._resolve_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"{RESOLVE_SERVICE} unavailable, target columns will be empty")
            return None

        req = ResolveNavPoints.Request()
        req.nav_point_ids = list(ids)
        req.nav_point_names = list(ids)
        future = self._resolve_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        resp = future.result()
        if resp is None or not resp.success or not resp.segments:
            msg = "no response" if resp is None else resp.message
            self.get_logger().warn(f"resolve_nav_points failed: {msg}")
            return None

        seg = resp.segments[-1]
        if not seg.control_points:
            self.get_logger().warn("resolve_nav_points returned a segment without control points")
            return None
        end = seg.control_points[-1]
        return (end.x, end.y, seg.target_yaw)

    # ── 到点位姿采样 ────────────────────────────────────────────────

    def _lookup_pose(self):
        """(stamp_ns, x, y, yaw) from TF, or None."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, Time())
        except tf2_ros.TransformException:
            return None
        return (
            Time.from_msg(tf.header.stamp).nanoseconds,
            tf.transform.translation.x,
            tf.transform.translation.y,
            yaw_from_quaternion(tf.transform.rotation),
        )

    def sample_pose(self, duration: float):
        """采样 duration 秒内的 map 位姿，返回 [(x, y, yaw), ...]。

        按 TF header.stamp 去重：同一帧被重复采到会把标准差压向 0，
        去重后标准差才反映真实的定位抖动。
        """
        samples = []
        last_stamp = None
        end_ns = self.get_clock().now().nanoseconds + int(duration * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end_ns:
            rclpy.spin_once(self, timeout_sec=self._sample_period)
            got = self._lookup_pose()
            if got is None:
                continue
            stamp, x, y, yaw = got
            if stamp == last_stamp:
                continue
            last_stamp = stamp
            samples.append((x, y, yaw))
        if not samples:
            self.get_logger().warn(
                f"No TF {self._map_frame} -> {self._base_frame} during settle window")
        return samples

    # ── 一次移动 ────────────────────────────────────────────────────

    def send_once(self, ids):
        path = "->".join(ids)
        # 先解析目标点：避免服务调用挤占 result 之后的采样窗口
        target = self.resolve_target(ids)

        goal = NavigateThroughGraphPoints.Goal()
        goal.nav_point_ids = list(ids)
        goal.nav_point_names = list(ids)
        goal.pose = Pose2D(x=0.0, y=0.0, theta=0.0)
        goal.target_yaw = 0.0

        self.get_logger().info(f"Sending goal: {ids}")
        t0 = self.get_clock().now().nanoseconds
        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn("Goal rejected.")
            duration = (self.get_clock().now().nanoseconds - t0) / 1e9
            self._record(path, False, "goal rejected", duration, target, [])
            return

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        duration = (self.get_clock().now().nanoseconds - t0) / 1e9
        result = result_future.result().result
        # 失败结果按 warn 打：随机巡航跑几百次，失败那几条必须能从 info 洪流里挑出来。
        # 必须写成两个独立的调用点，不能用 `log = info if ... else warn` 再统一调用一次：
        # rclpy 的 logger 按调用点（文件/行/函数）缓存 severity，同一行换级别会抛
        # ValueError: Logger severity cannot be changed between calls.
        msg = f"Result: success={result.success}, message='{result.message}'"
        if result.success:
            self.get_logger().info(msg)
        else:
            self.get_logger().warn(msg)

        # result 返回即开始采样，不要先 sleep interval
        samples = self.sample_pose(self._settle)
        self._record(path, result.success, result.message, duration, target, samples)

    def _record(self, path, success, message, duration, target, samples):
        row = {f: "" for f in CSV_FIELDS}
        row["timestamp"] = datetime.now().isoformat(timespec="milliseconds")
        row["path"] = path
        row["success"] = success
        row["message"] = (message or "").replace("\n", " ")
        row["duration_s"] = f"{duration:.3f}"
        row["sample_count"] = len(samples)

        if target is not None:
            tx, ty, tyaw = target
            row["target_x"] = f"{tx:.6f}"
            row["target_y"] = f"{ty:.6f}"
            row["target_yaw"] = f"{tyaw:.6f}"

        if samples:
            mean_x, std_x = mean_std([s[0] for s in samples])
            mean_y, std_y = mean_std([s[1] for s in samples])
            mean_yaw, std_yaw = circular_mean_std([s[2] for s in samples])
            row["mean_x"] = f"{mean_x:.6f}"
            row["mean_y"] = f"{mean_y:.6f}"
            row["mean_yaw"] = f"{mean_yaw:.6f}"
            row["std_x"] = f"{std_x:.6f}"
            row["std_y"] = f"{std_y:.6f}"
            row["std_yaw"] = f"{std_yaw:.6f}"
            rng_x = value_range([s[0] for s in samples])
            rng_y = value_range([s[1] for s in samples])
            rng_yaw = circular_range([s[2] for s in samples], mean_yaw)
            row["range_x"] = f"{rng_x:.6f}"
            row["range_y"] = f"{rng_y:.6f}"
            row["range_yaw"] = f"{rng_yaw:.6f}"

            if target is not None and not math.isnan(target[2]):
                err_x = mean_x - target[0]
                err_y = mean_y - target[1]
                err_yaw = wrap_angle(mean_yaw - target[2])
                row["err_x"] = f"{err_x:.6f}"
                row["err_y"] = f"{err_y:.6f}"
                row["err_yaw"] = f"{err_yaw:.6f}"
                row["dist"] = f"{math.hypot(err_x, err_y):.6f}"
                self.get_logger().info(
                    f"[record] {path} dist={math.hypot(err_x, err_y) * 1000:.1f}mm "
                    f"err_yaw={math.degrees(err_yaw):.2f}deg "
                    f"std=({std_x * 1000:.1f},{std_y * 1000:.1f})mm "
                    f"rng=({rng_x * 1000:.1f},{rng_y * 1000:.1f})mm/"
                    f"{math.degrees(rng_yaw):.2f}deg n={len(samples)}")

        self._csv_writer.writerow(row)
        self._csv_file.flush()

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"[feedback] status={fb.current_status} "
            f"index={fb.current_point_index}/{fb.total_points}"
        )

    def run(self):
        try:
            while rclpy.ok():
                seq = random.choice(SEQUENCES)
                self.send_once(seq)
                if self._interval > 0.0:
                    self.get_logger().info(
                        f"Sleeping {self._interval:.2f}s before next goal..."
                    )
                    # simple sleep via spin_once loop to allow shutdown
                    end = self.get_clock().now().nanoseconds + int(
                        self._interval * 1e9
                    )
                    while (
                        rclpy.ok()
                        and self.get_clock().now().nanoseconds < end
                    ):
                        rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Seconds to wait between goals after a result is received (default: 1.0).",
    )
    parser.add_argument(
        "--seed", type=int, default=None, help="Optional RNG seed for reproducibility."
    )
    parser.add_argument(
        "--settle",
        type=float,
        default=1.0,
        help="到点后采样窗口秒数，均值/标准差都在这个窗口内统计 (default: 1.0).",
    )
    parser.add_argument(
        "--sample-hz",
        type=float,
        default=50.0,
        help="采样循环频率上限，实际样本数受 TF 发布频率限制 (default: 50).",
    )
    parser.add_argument(
        "--record",
        default="move_record.csv",
        help="CSV 输出路径，相对路径基于当前目录 (default: move_record.csv).",
    )
    parser.add_argument("--map-frame", default="map", help="Global frame (default: map).")
    parser.add_argument(
        "--base-frame", default="base_link", help="Robot base frame (default: base_link)."
    )
    args = parser.parse_args(argv)

    if args.seed is not None:
        random.seed(args.seed)

    record_path = os.path.abspath(args.record)

    rclpy.init()
    node = RandomNavClient(
        interval=args.interval,
        settle=args.settle,
        sample_hz=args.sample_hz,
        record_path=record_path,
        map_frame=args.map_frame,
        base_frame=args.base_frame,
    )
    try:
        if not node.wait_server():
            node.get_logger().error("Action server not available. Exiting.")
            return 1
        node.run()
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
    # # 构建
    # source install/setup.zsh
    #
    # # 运行（两种方式均可）
    # ros2 run nav2_schedule_planner random_navigate_through_graph_points.py
    # # 或
    # ros2 run nav2_schedule_planner random_navigate_through_graph_points.py --interval 2.0 --seed 42
    #
    # # 分析 / 绘图（在 move_record.csv 所在目录）
    # python3 plot_move_record.py --show
