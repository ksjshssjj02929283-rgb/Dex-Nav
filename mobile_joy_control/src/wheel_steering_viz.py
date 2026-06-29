#!/usr/bin/env python3
# Copyright 2026 DexForce
#
# 独立脚本：实时订阅底盘四轮舵角/轮速并 matplotlib 可视化。
# 不依赖本包编译安装，source ROS 工作空间后直接运行。
#
# 依赖: rclpy, chassis_interfaces, matplotlib, numpy
#
# 用法:
#   source /opt/ros/humble/setup.bash
#   source ~/your_ws/install/setup.bash   # 含 chassis_interfaces
#   python3 mobile_joy_control/src/wheel_steering_viz.py
#
# 可选参数:
#   python3 wheel_steering_viz.py --ros-args \
#       -p feedback_topic:=/feedback/omni_chassis_motor \
#       -p cmd_topic:=/control/omni_chassis_motors \
#       -p show_cmd_overlay:=true

from __future__ import annotations

import math
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, List, Optional

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import rclpy
from chassis_interfaces.msg import ChassisMotorControl
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# 与 mobile_cmd_omni_region.cpp 一致
CHASSIS_L_M = 0.460
CHASSIS_W_M = 0.345
STEER_LIMIT_RAD = math.pi / 2.0
ANGLE_EPS_RAD = 1.0

WHEEL_NAMES = ("FL", "FR", "RL", "RR")
# 俯视图: x 向右, y 向前 (base_link)
WHEEL_XY = (
    (-CHASSIS_W_M / 2.0, CHASSIS_L_M / 2.0),   # FL
    (CHASSIS_W_M / 2.0, CHASSIS_L_M / 2.0),    # FR
    (-CHASSIS_W_M / 2.0, -CHASSIS_L_M / 2.0),  # RL
    (CHASSIS_W_M / 2.0, -CHASSIS_L_M / 2.0),   # RR
)


def rad2deg(r: float) -> float:
    return math.degrees(r)


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass
class WheelSnapshot:
    angles_rad: List[float] = field(default_factory=lambda: [0.0] * 4)
    speeds: List[float] = field(default_factory=lambda: [0.0] * 4)
    stamp_sec: float = 0.0
    received: bool = False


class WheelSteeringVizNode(Node):
    def __init__(self) -> None:
        super().__init__('wheel_steering_viz')

        self.declare_parameter('feedback_topic', '/feedback/omni_chassis_motor')
        self.declare_parameter('cmd_topic', '/control/omni_chassis_motors')
        self.declare_parameter('show_cmd_overlay', True)
        self.declare_parameter('history_length', 300)

        feedback_topic = self.get_parameter('feedback_topic').get_parameter_value().string_value
        cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.show_cmd_overlay = self.get_parameter('show_cmd_overlay').get_parameter_value().bool_value
        self.history_len = self.get_parameter('history_length').get_parameter_value().integer_value

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.feedback = WheelSnapshot()
        self.command = WheelSnapshot()
        self.lock = threading.Lock()

        self.angle_history: List[Deque[float]] = [
            deque(maxlen=self.history_len) for _ in range(4)
        ]
        self.time_history: Deque[float] = deque(maxlen=self.history_len)
        self._t0: Optional[float] = None

        self.msg_count = 0
        self.last_wall_time: Optional[float] = None

        self.create_subscription(ChassisMotorControl, feedback_topic, self._on_feedback, qos)
        if self.show_cmd_overlay:
            self.create_subscription(ChassisMotorControl, cmd_topic, self._on_command, qos)

        self.get_logger().info(
            f'监听反馈: {feedback_topic}' +
            (f', 指令: {cmd_topic}' if self.show_cmd_overlay else ''))

    def _parse_motor_msg(self, msg: ChassisMotorControl) -> WheelSnapshot:
        snap = WheelSnapshot()
        snap.stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        for i in range(4):
            if i < len(msg.position):
                snap.angles_rad[i] = float(msg.position[i])
            if i < len(msg.speed):
                snap.speeds[i] = float(msg.speed[i])
        snap.received = True
        return snap

    def _on_feedback(self, msg: ChassisMotorControl) -> None:
        snap = self._parse_motor_msg(msg)
        with self.lock:
            self.feedback = snap
            self.msg_count += 1
            self.last_wall_time = self.get_clock().now().nanoseconds * 1e-9
            if self._t0 is None:
                self._t0 = (
                    snap.stamp_sec if snap.stamp_sec > 0.0
                    else self.last_wall_time
                )
            t = (
                snap.stamp_sec - self._t0
                if snap.stamp_sec > 0.0
                else self.last_wall_time - self._t0
            )
            self.time_history.append(t)
            for i in range(4):
                self.angle_history[i].append(snap.angles_rad[i])

    def _on_command(self, msg: ChassisMotorControl) -> None:
        with self.lock:
            self.command = self._parse_motor_msg(msg)

    def get_state(self):
        with self.lock:
            return (
                WheelSnapshot(
                    list(self.feedback.angles_rad),
                    list(self.feedback.speeds),
                    self.feedback.stamp_sec,
                    self.feedback.received,
                ),
                WheelSnapshot(
                    list(self.command.angles_rad),
                    list(self.command.speeds),
                    self.command.stamp_sec,
                    self.command.received,
                ),
                list(self.time_history),
                [list(h) for h in self.angle_history],
                self.msg_count,
            )


class WheelSteeringVisualizer:
    def __init__(self, node: WheelSteeringVizNode) -> None:
        self.node = node

        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(12, 7))
        self.fig.canvas.manager.set_window_title('Omni Wheel Steering Monitor')

        gs = self.fig.add_gridspec(2, 2, width_ratios=[1.2, 1.0], height_ratios=[1.0, 1.0])

        self.ax_chassis = self.fig.add_subplot(gs[:, 0])
        self.ax_bars = self.fig.add_subplot(gs[0, 1])
        self.ax_hist = self.fig.add_subplot(gs[1, 1])

        self._setup_chassis_axes()
        self._setup_bar_axes()
        self._setup_history_axes()

        self.fig.tight_layout(pad=2.0)

    def _setup_chassis_axes(self) -> None:
        ax = self.ax_chassis
        ax.set_title('俯视图 (base_link: x右 y前)')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.25)

        half_w = CHASSIS_W_M / 2.0
        half_l = CHASSIS_L_M / 2.0
        margin = 0.15
        ax.set_xlim(-half_w - margin, half_w + margin)
        ax.set_ylim(-half_l - margin, half_l + margin)

        # 底盘轮廓
        body = patches.Rectangle(
            (-half_w, -half_l), CHASSIS_W_M, CHASSIS_L_M,
            linewidth=1.5, edgecolor='#88aaff', facecolor='#223355', alpha=0.5)
        ax.add_patch(body)
        ax.arrow(0, 0, 0, half_l * 0.55, head_width=0.05, head_length=0.05,
                 fc='#66ff66', ec='#66ff66', length_includes_head=True)
        ax.text(0.02, half_l * 0.35, 'front', color='#66ff66', fontsize=9)

        # ±90° 限位参考（舵角合法区间）
        ax.axhline(0, color='gray', lw=0.5, alpha=0.4)
        ax.axvline(0, color='gray', lw=0.5, alpha=0.4)
        ax.text(-half_w, -half_l - 0.08, f'舵角限位 ±{rad2deg(STEER_LIMIT_RAD):.0f}°',
                color='#ffaa44', fontsize=9)

        self.wheel_lines_fb = []
        self.wheel_lines_cmd = []
        self.wheel_labels = []
        arrow_len = 0.14

        for i, (wx, wy) in enumerate(WHEEL_XY):
            # 用 line 画舵角方向，比 quiver.set_UVC 在各后端上更可靠
            line_fb, = ax.plot(
                [wx, wx + arrow_len], [wy, wy],
                color='#00e5ff', lw=3, solid_capstyle='round',
                label='反馈' if i == 0 else None)
            line_cmd, = ax.plot(
                [wx, wx + arrow_len * 0.85], [wy, wy],
                color='#ff6b6b', lw=2, alpha=0.85, ls='--',
                label='指令' if i == 0 else None)
            self.wheel_lines_fb.append(line_fb)
            self.wheel_lines_cmd.append(line_cmd)
            self.wheel_labels.append(
                ax.text(wx, wy - 0.11, WHEEL_NAMES[i], ha='center', fontsize=10, color='white'))

        ax.legend(loc='upper right', fontsize=8)

        self.wait_text = ax.text(
            0.5, 0.5, '等待 /feedback/omni_chassis_motor ...',
            transform=ax.transAxes,
            ha='center', va='center', fontsize=12, color='#ffaa44')

    @staticmethod
    def _set_steer_line(line, wx: float, wy: float, angle_rad: float, length: float) -> None:
        ux = length * math.cos(angle_rad)
        uy = length * math.sin(angle_rad)
        line.set_data([wx, wx + ux], [wy, wy + uy])

    def _setup_bar_axes(self) -> None:
        ax = self.ax_bars
        ax.set_title('当前舵角 (°)')
        x = np.arange(4)
        self.bar_fb = ax.bar(x - 0.15, [0] * 4, width=0.3, label='反馈', color='#00e5ff')
        self.bar_cmd = ax.bar(x + 0.15, [0] * 4, width=0.3, label='指令', color='#ff6b6b', alpha=0.85)
        ax.set_xticks(x, WHEEL_NAMES)
        ax.set_ylim(-95, 95)
        ax.axhline(rad2deg(STEER_LIMIT_RAD), color='#ffaa44', ls='--', lw=1, alpha=0.8)
        ax.axhline(-rad2deg(STEER_LIMIT_RAD), color='#ffaa44', ls='--', lw=1, alpha=0.8)
        ax.axhline(0, color='gray', lw=0.5)
        ax.legend(fontsize=8)
        ax.grid(True, axis='y', alpha=0.25)

        self.status_text = ax.text(
            0.02, 0.02, '', transform=ax.transAxes, fontsize=8,
            verticalalignment='bottom', family='monospace', color='#cccccc')

    def _setup_history_axes(self) -> None:
        ax = self.ax_hist
        ax.set_title('舵角历史 (°)')
        ax.set_xlabel('t (s)')
        ax.set_ylabel('angle (deg)')
        colors = ['#00e5ff', '#4dd0e1', '#80deea', '#b2ebf2']
        self.hist_lines = []
        for i in range(4):
            line, = ax.plot([], [], label=WHEEL_NAMES[i], color=colors[i], lw=1.2)
            self.hist_lines.append(line)
        ax.axhline(rad2deg(STEER_LIMIT_RAD), color='#ffaa44', ls='--', lw=0.8, alpha=0.6)
        ax.axhline(-rad2deg(STEER_LIMIT_RAD), color='#ffaa44', ls='--', lw=0.8, alpha=0.6)
        ax.legend(fontsize=7, ncol=4, loc='upper right')
        ax.grid(True, alpha=0.25)

    def update(self) -> bool:
        fb, cmd, times, histories, msg_count = self.node.get_state()
        if not fb.received:
            self.wait_text.set_visible(True)
            self.status_text.set_text(
                '未收到反馈 (count=0)\n'
                '请确认: ros2 topic echo /feedback/omni_chassis_motor\n'
                '且底盘驱动 / mobile_cmd_omni 在运行')
            return False

        self.wait_text.set_visible(False)
        arrow_len = 0.14
        for i, (wx, wy) in enumerate(WHEEL_XY):
            self._set_steer_line(
                self.wheel_lines_fb[i], wx, wy, fb.angles_rad[i], arrow_len)

            if cmd.received and self.node.show_cmd_overlay:
                self._set_steer_line(
                    self.wheel_lines_cmd[i], wx, wy, cmd.angles_rad[i], arrow_len * 0.85)
                self.wheel_lines_cmd[i].set_visible(True)
            else:
                self.wheel_lines_cmd[i].set_visible(False)

            deg = rad2deg(fb.angles_rad[i])
            near_limit = abs(fb.angles_rad[i]) > STEER_LIMIT_RAD * 0.85
            color = '#ff4444' if near_limit else '#ffffff'
            self.wheel_labels[i].set_text(
                f'{WHEEL_NAMES[i]}\n{deg:+.1f}°\nspd {fb.speeds[i]:+.0f}')
            self.wheel_labels[i].set_color(color)

        for i, bar in enumerate(self.bar_fb):
            bar.set_height(rad2deg(fb.angles_rad[i]))
        for i, bar in enumerate(self.bar_cmd):
            if cmd.received:
                bar.set_height(rad2deg(cmd.angles_rad[i]))
            else:
                bar.set_height(0.0)

        # 相邻帧舵角跳变（对应 ANGLE_EPS 停轮逻辑）
        jumps = []
        if len(times) >= 2 and all(len(h) >= 2 for h in histories):
            for i in range(4):
                d = abs(wrap_pi(histories[i][-1] - histories[i][-2]))
                jumps.append(d)

        status_lines = [f'反馈帧数: {msg_count}']
        if jumps:
            max_jump = max(jumps)
            status_lines.append(f'max |Δδ|/frame: {rad2deg(max_jump):+.1f}°  (ANGLE_EPS={rad2deg(ANGLE_EPS_RAD):.0f}°)')
            if max_jump > ANGLE_EPS_RAD:
                status_lines.append('⚠ 超过 ANGLE_EPS → 底盘可能 Two-Phase 停轮')

        for i in range(4):
            if abs(fb.angles_rad[i]) > STEER_LIMIT_RAD:
                status_lines.append(f'⚠ {WHEEL_NAMES[i]} 超出 ±90° 限位')

        self.status_text.set_text('\n'.join(status_lines) if status_lines else 'OK')

        if times:
            for i, line in enumerate(self.hist_lines):
                if len(histories[i]) == len(times):
                    line.set_data(times, [rad2deg(a) for a in histories[i]])
            self.ax_hist.relim()
            self.ax_hist.autoscale_view(scaling='y')

        return True


def main() -> None:
    rclpy.init()
    node = WheelSteeringVizNode()

    viz = WheelSteeringVisualizer(node)

    # interactive 模式 + 主循环：比 fig.canvas.new_timer 在各 GUI 后端上更可靠
    plt.ion()
    viz.fig.show()

    try:
        while rclpy.ok() and plt.fignum_exists(viz.fig.number):
            rclpy.spin_once(node, timeout_sec=0.01)
            viz.update()
            viz.fig.canvas.draw()
            viz.fig.canvas.flush_events()
            plt.pause(0.03)   # ~30 Hz 刷新
    except KeyboardInterrupt:
        pass
    finally:
        plt.ioff()
        plt.close(viz.fig)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
