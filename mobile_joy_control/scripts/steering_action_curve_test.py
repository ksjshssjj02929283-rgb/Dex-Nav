#!/usr/bin/env python3
"""发送一次静止对舵 Action，并绘制四轮理论角度与实际反馈曲线。

使用前只修改下方 TEST_CONFIG，然后直接运行：

  python3 steering_action_curve_test.py

安全要求：运行前停止导航、手柄和其他速度来源。Action 执行期间 region 会强制
四轮驱动速度为零，但 Action 结束后会释放执行权，残留非零 Twist 可能让车辆运动。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from chassis_interfaces.msg import ChassisMotorControl
from nav_interfaces.action import SteeringAngleControl
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


# ============================ 用户配置区 ============================
TEST_CONFIG = {
    # 四轮顺序：左前、右前、左后、右后；单位均为弧度。
    "target_angles_rad": [0.0, 0.0, 0.0, 0.0],
    "steering_action_name": "/steering_angle_control",
    "motor_feedback_topic": "/feedback/omni_chassis_motor",
    "action_timeout_sec": 5.0,
    "settle_tolerance_rad": 0.02,
    "steering_limit_rad": 0.8 * math.pi,
    "feedback_timeout_sec": 0.2,
    # Action 前记录初始稳态，Action 结束后继续记录，以观察释放后的角度回弹。
    "record_before_action_sec": 0.5,
    "record_after_result_sec": 2.0,
    "output_dir": "./steering_action_results",
    # 留空时自动搜索同时支持中文、数字和字母的字体。
    "chinese_font_path": "",
}
# ==================================================================


WHEEL_NAMES = ("左前轮", "右前轮", "左后轮", "右后轮")
WHEEL_COUNT = len(WHEEL_NAMES)


@dataclass(frozen=True)
class MotorSample:
    receive_sec: float
    angles_rad: List[float]


def normalize_angle(angle: float) -> float:
    """把角度归一化到 [-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_error(target: float, actual: float) -> float:
    """返回考虑角度周期性的绝对角度误差。"""
    return abs(normalize_angle(target - actual))


class SteeringActionCurveTest(Node):
    """执行一次静止对舵并保存四轮角度响应图。"""

    def __init__(self) -> None:
        super().__init__("steering_action_curve_test")

        for name, default_value in TEST_CONFIG.items():
            self.declare_parameter(name, default_value)

        self.target_angles = [
            float(value)
            for value in self.get_parameter("target_angles_rad").value
        ]
        self.action_name = str(
            self.get_parameter("steering_action_name").value)
        self.feedback_topic = str(
            self.get_parameter("motor_feedback_topic").value)
        self.action_timeout_sec = float(
            self.get_parameter("action_timeout_sec").value)
        self.settle_tolerance_rad = float(
            self.get_parameter("settle_tolerance_rad").value)
        self.steering_limit_rad = float(
            self.get_parameter("steering_limit_rad").value)
        self.feedback_timeout_sec = float(
            self.get_parameter("feedback_timeout_sec").value)
        self.record_before_action_sec = float(
            self.get_parameter("record_before_action_sec").value)
        self.record_after_result_sec = float(
            self.get_parameter("record_after_result_sec").value)
        self.chinese_font_path = str(
            self.get_parameter("chinese_font_path").value).strip()

        self._validate_config()

        output_root = Path(
            str(self.get_parameter("output_dir").value)
        ).expanduser().resolve()
        output_root.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.output_dir = output_root / f"驻车对舵测试_{timestamp}"
        self.output_dir.mkdir(parents=True, exist_ok=False)
        self.plot_path = self.output_dir / "四轮驻车对舵角度曲线.png"

        self.action_client = ActionClient(
            self, SteeringAngleControl, self.action_name)
        self.feedback_sub = self.create_subscription(
            ChassisMotorControl,
            self.feedback_topic,
            self._motor_feedback_callback,
            50,
        )

        self.phase = "waiting"
        self.finished = False
        self.success = False
        self.latest_feedback: Optional[MotorSample] = None
        self.samples: List[MotorSample] = []
        self.initial_angles: List[float] = []
        self.baseline_deadline_sec = 0.0
        self.action_sent_sec = math.nan
        self.action_completed_sec = math.nan
        self.client_deadline_sec = math.inf
        self.post_record_deadline_sec = math.inf
        self.goal_handle = None
        self.action_status = GoalStatus.STATUS_UNKNOWN
        self.action_result_success = False
        self.action_result_message = "尚未返回结果"
        self.latest_action_max_error = math.nan
        self.last_wait_log_sec = -math.inf

        self.timer = self.create_timer(0.02, self._timer_callback)
        self.get_logger().info(
            "驻车对舵曲线测试启动："
            f"目标=[{', '.join(f'{value:.4f}' for value in self.target_angles)}]弧度，"
            f"容差={self.settle_tolerance_rad:.4f}弧度，"
            f"结果目录={self.output_dir}"
        )

    def _validate_config(self) -> None:
        if len(self.target_angles) != WHEEL_COUNT:
            raise ValueError("target_angles_rad 必须恰好包含四个角度")
        for wheel_name, target in zip(WHEEL_NAMES, self.target_angles):
            if not math.isfinite(target):
                raise ValueError(f"{wheel_name}目标角度不是有限值")
            if abs(target) > self.steering_limit_rad:
                raise ValueError(
                    f"{wheel_name}目标角度 {target:.4f} 超过限位 "
                    f"±{self.steering_limit_rad:.4f} 弧度")
        if self.action_timeout_sec <= 0.0:
            raise ValueError("action_timeout_sec 必须大于零")
        if self.settle_tolerance_rad <= 0.0:
            raise ValueError("settle_tolerance_rad 必须大于零")
        if self.feedback_timeout_sec <= 0.0:
            raise ValueError("feedback_timeout_sec 必须大于零")
        if self.record_before_action_sec < 0.0:
            raise ValueError("record_before_action_sec 不能小于零")
        if self.record_after_result_sec < 0.0:
            raise ValueError("record_after_result_sec 不能小于零")

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _throttled_wait_log(self, message: str) -> None:
        now_sec = self._now_sec()
        if now_sec - self.last_wait_log_sec >= 2.0:
            self.last_wait_log_sec = now_sec
            self.get_logger().info(message)

    def _motor_feedback_callback(
        self, message: ChassisMotorControl
    ) -> None:
        angles = [float(value) for value in message.position[:WHEEL_COUNT]]
        if len(angles) != WHEEL_COUNT or not all(
            math.isfinite(value) for value in angles
        ):
            return
        sample = MotorSample(self._now_sec(), angles)
        self.latest_feedback = sample
        if self.phase in (
            "baseline", "sending_goal", "executing", "post_record"
        ):
            self.samples.append(sample)

    def _timer_callback(self) -> None:
        if self.finished:
            return

        now_sec = self._now_sec()
        if self.phase == "waiting":
            if not self.action_client.server_is_ready():
                self._throttled_wait_log(
                    f"等待驻车对舵动作服务器 {self.action_name}")
                return
            feedback_fresh = (
                self.latest_feedback is not None
                and now_sec - self.latest_feedback.receive_sec
                <= self.feedback_timeout_sec
            )
            if not feedback_fresh:
                self._throttled_wait_log(
                    f"等待新鲜四轮角度反馈 {self.feedback_topic}")
                return
            assert self.latest_feedback is not None
            self.initial_angles = list(self.latest_feedback.angles_rad)
            self.samples = [self.latest_feedback]
            self.phase = "baseline"
            self.baseline_deadline_sec = (
                now_sec + self.record_before_action_sec)
            self.get_logger().info(
                f"开始记录动作前角度，持续 {self.record_before_action_sec:.2f} 秒")
            return

        if self.phase == "baseline":
            if now_sec >= self.baseline_deadline_sec:
                self._send_goal()
            return

        if self.phase in ("sending_goal", "executing"):
            if now_sec >= self.client_deadline_sec:
                if self.goal_handle is not None:
                    try:
                        self.goal_handle.cancel_goal_async()
                    except Exception as error:
                        self.get_logger().error(f"取消超时动作失败：{error}")
                self.action_status = GoalStatus.STATUS_ABORTED
                self.action_result_success = False
                self.action_result_message = "客户端等待动作结果超时"
                self._begin_post_record()
            return

        if (
            self.phase == "post_record"
            and now_sec >= self.post_record_deadline_sec
        ):
            self._finish_and_plot()

    def _send_goal(self) -> None:
        assert self.latest_feedback is not None
        self.initial_angles = list(self.latest_feedback.angles_rad)

        goal = SteeringAngleControl.Goal()
        goal.target_steer_angles_rad = list(self.target_angles)
        goal.timeout_sec = self.action_timeout_sec
        goal.settle_tolerance_rad = self.settle_tolerance_rad

        self.action_sent_sec = self._now_sec()
        self.client_deadline_sec = (
            self.action_sent_sec + self.action_timeout_sec + 1.0)
        self.phase = "sending_goal"
        try:
            future = self.action_client.send_goal_async(
                goal, feedback_callback=self._action_feedback_callback)
        except Exception as error:
            self.action_status = GoalStatus.STATUS_ABORTED
            self.action_result_message = f"发送动作目标失败：{error}"
            self._begin_post_record()
            return
        future.add_done_callback(self._goal_response_callback)
        self.get_logger().info(
            "已发送静止对舵目标，四轮驱动速度应保持为零")

    def _action_feedback_callback(self, feedback_message) -> None:
        self.latest_action_max_error = float(
            feedback_message.feedback.max_steer_error)

    def _goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            self.action_status = GoalStatus.STATUS_ABORTED
            self.action_result_message = f"读取目标响应失败：{error}"
            self._begin_post_record()
            return
        if self.phase == "post_record" or self.finished:
            if goal_handle is not None and goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if goal_handle is None or not goal_handle.accepted:
            self.action_status = GoalStatus.STATUS_ABORTED
            self.action_result_message = "驻车对舵动作目标被拒绝"
            self._begin_post_record()
            return

        self.goal_handle = goal_handle
        self.phase = "executing"
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future) -> None:
        if self.phase == "post_record" or self.finished:
            return
        self.goal_handle = None
        try:
            wrapped_result = future.result()
            self.action_status = int(wrapped_result.status)
            result = wrapped_result.result
            self.action_result_success = (
                result is not None and bool(result.success))
            self.action_result_message = (
                str(result.message) if result is not None else "动作没有返回结果")
        except Exception as error:
            self.action_status = GoalStatus.STATUS_ABORTED
            self.action_result_success = False
            self.action_result_message = f"读取动作结果失败：{error}"
        self._begin_post_record()

    def _begin_post_record(self) -> None:
        if self.phase == "post_record" or self.finished:
            return
        self.action_completed_sec = self._now_sec()
        self.phase = "post_record"
        self.post_record_deadline_sec = (
            self.action_completed_sec + self.record_after_result_sec)
        self.get_logger().info(
            "动作已结束，继续记录释放后的角度变化："
            f"状态={self._status_text(self.action_status)}，"
            f"结果={'成功' if self.action_result_success else '失败'}，"
            f"继续记录={self.record_after_result_sec:.2f}秒"
        )
        if self.record_after_result_sec <= 0.0:
            self._finish_and_plot()

    @staticmethod
    def _status_text(status: int) -> str:
        names = {
            GoalStatus.STATUS_UNKNOWN: "未知",
            GoalStatus.STATUS_ACCEPTED: "已接受",
            GoalStatus.STATUS_EXECUTING: "执行中",
            GoalStatus.STATUS_CANCELING: "正在取消",
            GoalStatus.STATUS_SUCCEEDED: "成功",
            GoalStatus.STATUS_CANCELED: "已取消",
            GoalStatus.STATUS_ABORTED: "已中止",
        }
        return names.get(status, f"状态码{status}")

    def _find_chinese_font(self, font_manager, ft2font):
        required_chars = (
            "四轮驻车对舵理论实际角度曲线时间秒弧度左前右后目标最终"
            "误差最小时刻动作结束结果成功失败"
            "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
            "（）()[]:,.+-_ "
        )
        candidates: List[str] = []
        if self.chinese_font_path:
            candidates.append(str(
                Path(self.chinese_font_path).expanduser().resolve()))
        system_fonts = font_manager.findSystemFonts()
        system_fonts.sort(key=lambda path: (
            0 if "notosanscjk" in path.lower() else
            1 if "wenquanyi" in path.lower() else
            2 if "simhei" in path.lower() else
            3 if "ukai" in path.lower() else 4,
            path,
        ))
        candidates.extend(system_fonts)

        checked = set()
        for candidate in candidates:
            if candidate in checked:
                continue
            checked.add(candidate)
            try:
                font = ft2font.FT2Font(candidate)
                if all(
                    font.get_char_index(ord(character)) != 0
                    for character in required_chars
                ):
                    font_manager.fontManager.addfont(candidate)
                    return font_manager.FontProperties(fname=candidate)
            except (OSError, RuntimeError):
                continue
        self.get_logger().error(
            "没有找到同时支持中文、数字和字母的字体，图片文字可能显示为方框")
        return None

    def _finish_and_plot(self) -> None:
        if self.finished:
            return
        if not self.samples or not math.isfinite(self.action_sent_sec):
            self.get_logger().error("没有足够的角度反馈，无法生成曲线图")
            self.finished = True
            return

        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            from matplotlib import font_manager, ft2font
        except Exception as error:
            self.get_logger().error(f"无法加载绘图库：{error}")
            self.finished = True
            return

        chinese_font = self._find_chinese_font(font_manager, ft2font)
        matplotlib.rcParams["axes.unicode_minus"] = False

        relative_times = [
            sample.receive_sec - self.action_sent_sec
            for sample in self.samples
        ]
        plot_start = min(relative_times)
        plot_end = max(relative_times)
        if plot_end <= plot_start:
            plot_end = plot_start + 1.0e-3
        action_end_relative = (
            self.action_completed_sec - self.action_sent_sec
            if math.isfinite(self.action_completed_sec) else math.nan
        )

        figure, axes_grid = plt.subplots(2, 2, figsize=(14, 10), sharex=True)
        axes_list = list(axes_grid.flat)
        summary_lines = []

        for wheel, axes in enumerate(axes_list):
            actual_angles = [
                sample.angles_rad[wheel] for sample in self.samples
            ]
            target = self.target_angles[wheel]
            initial = self.initial_angles[wheel]

            post_goal_indices = [
                index for index, elapsed in enumerate(relative_times)
                if elapsed >= 0.0
            ]
            if not post_goal_indices:
                post_goal_indices = list(range(len(self.samples)))
            minimum_index = min(
                post_goal_indices,
                key=lambda index: angle_error(target, actual_angles[index]),
            )
            final_index = len(self.samples) - 1
            minimum_error = angle_error(
                target, actual_angles[minimum_index])
            final_error = angle_error(target, actual_angles[final_index])
            minimum_time = relative_times[minimum_index]

            axes.step(
                [plot_start, 0.0, plot_end],
                [initial, target, target],
                where="post",
                color="tab:red",
                linestyle="--",
                linewidth=1.8,
                label="理论角度",
            )
            axes.plot(
                relative_times,
                actual_angles,
                color="tab:blue",
                linewidth=1.6,
                label="实际角度",
            )
            axes.scatter(
                [minimum_time],
                [actual_angles[minimum_index]],
                color="tab:green",
                s=45,
                zorder=5,
                label="最小误差点",
            )
            axes.scatter(
                [relative_times[final_index]],
                [actual_angles[final_index]],
                color="tab:orange",
                s=40,
                zorder=5,
                label="最终记录点",
            )
            axes.axvline(
                0.0,
                color="gray",
                linewidth=0.9,
                alpha=0.7,
                label="动作发送",
            )
            if math.isfinite(action_end_relative):
                axes.axvline(
                    action_end_relative,
                    color="purple",
                    linewidth=0.9,
                    linestyle=":",
                    alpha=0.8,
                    label="动作结束",
                )

            annotation = (
                f"目标角度：{target:.5f} 弧度\n"
                f"最终实际角度：{actual_angles[final_index]:.5f} 弧度\n"
                f"最终角度差：{final_error:.5f} 弧度\n"
                f"最小角度差：{minimum_error:.5f} 弧度\n"
                f"最小点实际角度：{actual_angles[minimum_index]:.5f} 弧度\n"
                f"最小时刻：{minimum_time:.3f} 秒"
            )
            axes.text(
                0.02,
                0.98,
                annotation,
                transform=axes.transAxes,
                va="top",
                ha="left",
                fontproperties=chinese_font,
                fontsize=10,
                bbox={
                    "boxstyle": "round",
                    "facecolor": "white",
                    "alpha": 0.82,
                },
            )
            axes.set_title(WHEEL_NAMES[wheel], fontproperties=chinese_font)
            axes.set_ylabel("舵角（弧度）", fontproperties=chinese_font)
            axes.grid(True, linestyle="--", alpha=0.35)
            axes.legend(prop=chinese_font, loc="best")
            summary_lines.append(
                f"{WHEEL_NAMES[wheel]}：最终差={final_error:.5f}弧度，"
                f"最小差={minimum_error:.5f}弧度，"
                f"最小时刻={minimum_time:.3f}秒")

        for axes in axes_list[2:]:
            axes.set_xlabel("相对动作发送时间（秒）", fontproperties=chinese_font)

        status_summary = (
            f"动作状态：{self._status_text(self.action_status)}；"
            f"动作结果：{'成功' if self.action_result_success else '失败'}；"
            f"结果信息：{self.action_result_message}"
        )
        figure.suptitle(
            "四轮驻车对舵理论角度与实际角度曲线",
            fontproperties=chinese_font,
            fontsize=16,
        )
        figure.text(
            0.5,
            0.012,
            status_summary,
            ha="center",
            fontproperties=chinese_font,
            fontsize=10,
        )
        figure.tight_layout(rect=(0.0, 0.04, 1.0, 0.96))
        try:
            figure.savefig(self.plot_path, dpi=170)
        except Exception as error:
            self.get_logger().error(f"保存曲线图失败：{error}")
            self.finished = True
            plt.close(figure)
            return
        plt.close(figure)

        self.success = (
            self.action_status == GoalStatus.STATUS_SUCCEEDED
            and self.action_result_success
        )
        self.finished = True
        self.get_logger().info(f"曲线图已保存：{self.plot_path}")
        for line in summary_lines:
            self.get_logger().info(line)

    def request_stop(self) -> None:
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception as error:
                self.get_logger().error(f"取消驻车对舵动作失败：{error}")
        self.finished = True
        self.success = False


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node: Optional[SteeringActionCurveTest] = None
    try:
        node = SteeringActionCurveTest()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        if node is not None:
            node.request_stop()
            for _ in range(10):
                if not rclpy.ok():
                    break
                rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
