#!/usr/bin/env python3
"""Unit tests for the Python pub_current_position node and C++ log contract.

The Python tests use small ROS 2 test doubles, so they can run without a
running ROS graph. The C++ checks are source-level contract checks; runtime
verification of the C++ node should be done after building it in a ROS 2
environment.
"""

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PYTHON_SOURCE = PACKAGE_ROOT / 'scripts' / 'pub_current_position.py'
CPP_SOURCE = PACKAGE_ROOT / 'src' / 'pub_current_position.cpp'


class FakeLogger:
    def __init__(self):
        self.records = []

    def info(self, message):
        self.records.append(('INFO', str(message)))

    def warning(self, message):
        self.records.append(('WARN', str(message)))


class FakeClock:
    def now(self):
        return SimpleNamespace(to_msg=lambda: 'test-time')


class FakePublisher:
    def __init__(self, topic, qos):
        self.topic = topic
        self.qos = qos
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    def __init__(self, name):
        self.name = name
        self.logger = FakeLogger()
        self.clock = FakeClock()
        self.publishers = []
        self.subscriptions = []
        self.timers = []

    def create_publisher(self, _msg_type, topic, qos):
        publisher = FakePublisher(topic, qos)
        self.publishers.append(publisher)
        return publisher

    def create_subscription(self, _msg_type, topic, callback, qos):
        subscription = SimpleNamespace(
            topic=topic,
            callback=callback,
            qos=qos,
        )
        self.subscriptions.append(subscription)
        return subscription

    def create_timer(self, period, callback):
        timer = SimpleNamespace(period=period, callback=callback)
        self.timers.append(timer)
        return timer

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return self.clock

    def destroy_node(self):
        pass


class QoSProfile:
    def __init__(self, history, depth, reliability, durability):
        self.history = history
        self.depth = depth
        self.reliability = reliability
        self.durability = durability


def install_ros_test_doubles():
    """Install only the ROS modules imported by pub_current_position.py."""
    rclpy = types.ModuleType('rclpy')
    rclpy.init = lambda *args, **kwargs: None
    rclpy.ok = lambda: True
    rclpy.shutdown = lambda: None
    rclpy.spin = lambda *_args, **_kwargs: None

    geometry_msgs = types.ModuleType('geometry_msgs')
    geometry_msgs_msg = types.ModuleType('geometry_msgs.msg')
    geometry_msgs_msg.PoseStamped = type('PoseStamped', (), {
        '__init__': lambda self: setattr(
            self,
            '__dict__',
            {
                'header': SimpleNamespace(frame_id='', stamp=None),
                'pose': SimpleNamespace(
                    position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
            },
        ),
    })

    nav_msgs = types.ModuleType('nav_msgs')
    nav_msgs_msg = types.ModuleType('nav_msgs.msg')
    nav_msgs_msg.Odometry = type('Odometry', (), {})

    executors = types.ModuleType('rclpy.executors')
    executors.ExternalShutdownException = type(
        'ExternalShutdownException', (Exception,), {}
    )
    node_module = types.ModuleType('rclpy.node')
    node_module.Node = FakeNode
    qos_module = types.ModuleType('rclpy.qos')
    qos_module.QoSProfile = QoSProfile
    qos_module.HistoryPolicy = SimpleNamespace(KEEP_LAST='KEEP_LAST')
    qos_module.ReliabilityPolicy = SimpleNamespace(RELIABLE='RELIABLE')
    qos_module.DurabilityPolicy = SimpleNamespace(VOLATILE='VOLATILE')

    geometry_msgs.msg = geometry_msgs_msg
    nav_msgs.msg = nav_msgs_msg
    rclpy.executors = executors
    rclpy.node = node_module
    rclpy.qos = qos_module

    sys.modules.update({
        'rclpy': rclpy,
        'rclpy.executors': executors,
        'rclpy.node': node_module,
        'rclpy.qos': qos_module,
        'geometry_msgs': geometry_msgs,
        'geometry_msgs.msg': geometry_msgs_msg,
        'nav_msgs': nav_msgs,
        'nav_msgs.msg': nav_msgs_msg,
    })


def load_python_node_module():
    install_ros_test_doubles()
    spec = importlib.util.spec_from_file_location(
        'pub_current_position_under_test', PYTHON_SOURCE
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def make_odom(frame_id='map', child_frame_id='base_link'):
    msg = SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id),
        child_frame_id=child_frame_id,
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1.2, y=-0.4, z=3.5),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.3, w=0.95),
            )
        ),
    )
    return msg


class PythonNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_python_node_module()

    def setUp(self):
        self.now = 0.0
        self.monotonic_patch = mock.patch.object(
            self.module.time,
            'monotonic',
            side_effect=lambda: self.now,
        )
        self.monotonic_patch.start()
        self.node = self.module.BaseLinkPosition()

    def tearDown(self):
        self.monotonic_patch.stop()

    @property
    def publisher(self):
        return self.node.publisher

    def warnings(self):
        return [message for level, message in self.node.logger.records
                if level == 'WARN']

    def test_valid_odom_is_converted_and_published(self):
        self.node._odom_callback(make_odom())

        self.assertEqual(len(self.publisher.messages), 1)
        pose = self.publisher.messages[0]
        self.assertEqual(pose.header.frame_id, 'map')
        self.assertEqual(pose.header.stamp, 'test-time')
        self.assertEqual(pose.pose.position.x, 1.2)
        self.assertEqual(pose.pose.position.y, -0.4)
        self.assertEqual(pose.pose.position.z, 0.0)
        self.assertEqual(pose.pose.orientation.z, 0.3)
        self.assertIn('Received valid odometry.',
                      [message for _, message in self.node.logger.records])

    def test_invalid_frame_is_rejected_and_logged(self):
        self.node._odom_callback(make_odom('odom', 'base_link'))

        self.assertEqual(len(self.publisher.messages), 0)
        self.assertEqual(len(self.warnings()), 1)
        self.assertIn('Rejected odometry', self.warnings()[0])

    def test_problem_logs_are_limited_to_once_per_five_seconds(self):
        self.node._odom_callback(make_odom('odom', 'base_link'))
        self.now = 4.9
        self.node._odom_callback(make_odom('odom', 'base_link'))
        self.assertEqual(len(self.warnings()), 1)

        self.now = 5.0
        self.node._odom_callback(make_odom('odom', 'base_link'))
        self.assertEqual(len(self.warnings()), 2)

    def test_missing_odom_is_reported_and_recovery_is_logged(self):
        self.node._odom_callback(make_odom())
        self.now = 2.1
        self.node._monitor_odom()
        self.assertEqual(len(self.warnings()), 1)
        self.assertIn('No valid odometry received', self.warnings()[0])

        self.now = 4.9
        self.node._monitor_odom()
        self.assertEqual(len(self.warnings()), 1)

        self.now = 7.2
        self.node._monitor_odom()
        self.assertEqual(len(self.warnings()), 2)

        self.now = 7.3
        self.node._odom_callback(make_odom())
        infos = [message for level, message in self.node.logger.records
                 if level == 'INFO']
        self.assertIn('Odometry recovered.', infos)

    def test_output_and_input_qos_are_preserved(self):
        self.assertEqual(self.publisher.topic, '/feedback/chassis_pose')
        self.assertEqual(self.publisher.qos.depth, 10)
        self.assertEqual(self.publisher.qos.reliability, 'RELIABLE')
        self.assertEqual(self.node.odom_subscription.topic,
                         '/localization/odometry/filtered')
        self.assertEqual(self.node.odom_subscription.qos.reliability,
                         'RELIABLE')
        self.assertEqual(self.node.timers[0].period, 1.0)


class CppSourceContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = CPP_SOURCE.read_text(encoding='utf-8')

    def test_cpp_keeps_pose_contract(self):
        for text in (
            '"/localization/odometry/filtered"',
            '"/feedback/chassis_pose"',
            'rclcpp::KeepLast(10)',
            'output_qos.reliable()',
            'child_frame_id != "base_link"',
            'child_frame_id != "base_footprint"',
            'pose_msg.pose.position.z = 0.0',
        ):
            self.assertIn(text, self.source)

    def test_cpp_has_timeout_and_throttled_problem_logging(self):
        for text in (
            'std::chrono::seconds(5)',
            'std::chrono::seconds(2)',
            'create_wall_timer(',
            'RCLCPP_WARN(get_logger(), "%s", message.c_str())',
            'Rejected odometry',
            'No valid odometry received',
            'Odometry recovered.',
        ):
            self.assertIn(text, self.source)


if __name__ == '__main__':
    unittest.main(verbosity=2)
