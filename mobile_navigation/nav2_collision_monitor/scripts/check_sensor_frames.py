#!/usr/bin/env python3
"""Observe sensor headers without modifying messages or robot configuration."""

import argparse
import time
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan, PointCloud2


DEFAULT_SCANS = [
    '/laser_scan/front', '/laser_scan/rear',
    '/orbbec/front/laser_scan', '/orbbec/rear/laser_scan',
]


class SensorFrameCheck(Node):
    def __init__(self, scans, clouds, interval):
        super().__init__('sensor_frame_check')
        self.interval = interval
        self.started = time.monotonic()
        self.stats = {}
        self.subs = []
        for msg_type, topics in ((LaserScan, scans), (PointCloud2, clouds)):
            for topic in dict.fromkeys(topics):
                resolved = self.resolve_topic_name(topic)
                if resolved in self.stats:
                    raise ValueError('Topic specified more than once: ' + resolved)
                self.stats[resolved] = {
                    'total': 0, 'empty': 0, 'blank': 0, 'frames': set(),
                    'last_rx': None, 'last_error': None,
                }
                self.subs.append(self.create_subscription(
                    msg_type, topic, partial(self.receive, resolved),
                    qos_profile_sensor_data))
                self.get_logger().info(f'Listening: {resolved} ({msg_type.__name__})')
        self.timer = self.create_timer(interval, self.report)

    def publishers(self, topic):
        # Graph discovery lists candidates; it does not attribute a sample.
        endpoints = self.get_publishers_info_by_topic(topic)
        return ', '.join(
            f'{ep.node_namespace.rstrip("/")}/{ep.node_name} [{ep.topic_type}]'
            for ep in endpoints
        ) or '(no publisher discovered)'

    def receive(self, topic, msg):
        now = time.monotonic()
        state = self.stats[topic]
        state['total'] += 1
        state['last_rx'] = now
        frame = msg.header.frame_id
        if not frame or not frame.strip():
            state['empty' if not frame else 'blank'] += 1
            if state['last_error'] is None or now - state['last_error'] >= self.interval:
                state['last_error'] = now
                stamp = msg.header.stamp
                size = (f'ranges={len(msg.ranges)}' if isinstance(msg, LaserScan)
                        else f'points={msg.width * msg.height}, bytes={len(msg.data)}')
                self.get_logger().error(
                    f'INVALID FRAME topic={topic} frame_id={frame!r} '
                    f'stamp={stamp.sec}.{stamp.nanosec:09d} {size} '
                    f'empty={state["empty"]} whitespace={state["blank"]} '
                    f'total={state["total"]}; '
                    f'publisher candidates: {self.publishers(topic)}')
            return
        if frame not in state['frames']:
            state['frames'].add(frame)
            self.get_logger().info(f'Frame observed: topic={topic} frame_id={frame!r}')

    def report(self):
        now = time.monotonic()
        for topic, state in self.stats.items():
            if state['last_rx'] is None:
                self.get_logger().warning(
                    f'NO DATA topic={topic} waiting={now - self.started:.1f}s; '
                    f'publisher candidates: {self.publishers(topic)}')
                continue
            age = now - state['last_rx']
            message = (
                f'topic={topic} total={state["total"]} empty={state["empty"]} '
                f'whitespace={state["blank"]} frames={sorted(state["frames"])} '
                f'last_received={age:.1f}s ago')
            if age >= self.interval:
                self.get_logger().warning('STALE ' + message)
            else:
                self.get_logger().info(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scan-topics', nargs='*', default=DEFAULT_SCANS,
                        help='LaserScan topics; defaults to the four collision monitor inputs')
    parser.add_argument('--pointcloud-topics', nargs='*', default=[],
                        help='Optional PointCloud2 topics')
    parser.add_argument('--interval', type=float, default=5.0,
                        help='Summary and per-topic error logging interval in seconds')
    args = parser.parse_args(remove_ros_args()[1:])
    if not 0 < args.interval < float('inf'):
        parser.error('--interval must be finite and positive')
    if not args.scan_topics and not args.pointcloud_topics:
        parser.error('At least one topic is required')
    if set(args.scan_topics) & set(args.pointcloud_topics):
        parser.error('A topic cannot be both LaserScan and PointCloud2')
    rclpy.init()
    node = None
    try:
        node = SensorFrameCheck(args.scan_topics, args.pointcloud_topics, args.interval)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.report()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
