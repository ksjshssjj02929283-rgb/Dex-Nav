"""基于 Matplotlib 的轻量级交互调参程序。"""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import Dict

import matplotlib.pyplot as plt
from matplotlib.widgets import Button, RadioButtons, Slider

from .config import PIDConfig, ToolConfig, load_tool_config, validate_config
from .pid_lab import simulate_pid_step
from .plotting import draw_pid_response, draw_rpp_result
from .scenarios import SCENARIOS
from .simulator import simulate_rpp


SLIDERS = (
    ("min_lookahead_dist", "最小前瞻距离 [m]", 0.1, 1.0),
    ("max_lookahead_dist", "最大前瞻距离 [m]", 0.2, 2.0),
    ("lookahead_time", "前瞻时间 [s]", 0.1, 3.0),
    ("angular_gain", "角速度增益", 0.2, 2.5),
    ("regulated_linear_scaling_min_radius", "曲率半径阈值 [m]", 0.05, 3.0),
    ("rotate_to_path_entry_angle", "原地转向进入角 [rad]", 0.05, 1.6),
    ("rotate_to_path_exit_angle", "原地转向退出角 [rad]", 0.02, 1.2),
    ("heading_error_scaling_min_angle", "朝向降速起始角 [rad]", 0.01, 1.2),
    ("heading_error_scaling_max_angle", "朝向降速结束角 [rad]", 0.02, 1.5),
    ("lateral_error_threshold", "横向降速起始距离 [m]", 0.01, 0.8),
    ("lateral_error_max_dist", "横向降速结束距离 [m]", 0.02, 1.2),
    ("max_angular_vel", "RPP 最大角速度 [rad/s]", 0.1, 2.0),
)

PID_SLIDERS = (
    ("kp", "Kp", 0.0, 8.0),
    ("ki", "Ki", 0.0, 3.0),
    ("kd", "Kd", 0.0, 2.0),
    ("integrator_saturation", "积分限幅", 0.0, 1.5),
    ("max_accel", "最大加速度", 0.05, 2.0),
)


class PIDDebugWindow:
    """用于实时调整 PID 阶跃响应的两个小窗口。"""

    def __init__(self, base_config: PIDConfig):
        self.base_config = base_config
        self.control_figure = plt.figure(figsize=(6.2, 4.2))
        self.result_figure = plt.figure(figsize=(9, 6))
        self.sliders: Dict[str, Slider] = {}
        for index, (name, label, minimum, maximum) in enumerate(PID_SLIDERS):
            axis = self.control_figure.add_axes((0.31, 0.78 - index * 0.14, 0.60, 0.06))
            slider = Slider(
                axis, label, minimum, maximum, valinit=getattr(base_config, name)
            )
            slider.on_changed(lambda _value: self.rerun())
            self.sliders[name] = slider
        reset_axis = self.control_figure.add_axes((0.35, 0.03, 0.30, 0.09))
        self.reset_button = Button(reset_axis, "恢复 YAML 参数")
        self.reset_button.on_clicked(self._reset)
        self.control_figure.canvas.manager.set_window_title("DEXE 速度 PID 参数")
        self.result_figure.canvas.manager.set_window_title("DEXE 速度 PID 阶跃响应")
        self.rerun()

    def current_config(self) -> PIDConfig:
        values = {name: slider.val for name, slider in self.sliders.items()}
        return replace(self.base_config, **values)

    def rerun(self) -> None:
        draw_pid_response(self.result_figure, simulate_pid_step(self.current_config()))
        self.result_figure.canvas.draw_idle()

    def _reset(self, _event) -> None:
        for name, slider in self.sliders.items():
            slider.set_val(getattr(self.base_config, name))
        self.rerun()


class DebugApp:
    def __init__(self, config: ToolConfig | Path | str):
        self.base_config = config if isinstance(config, ToolConfig) else load_tool_config(config)
        self.config = self.base_config
        self.scenario_key = "curve_1m"
        self.figure = plt.figure(figsize=(6.8, 9))
        self.result_figure = plt.figure(figsize=(12, 8))
        self.sliders: Dict[str, Slider] = {}
        self.pid_windows: list[PIDDebugWindow] = []
        self._create_controls()
        self.rerun()

    def _create_controls(self) -> None:
        radio_axis = self.figure.add_axes((0.05, 0.73, 0.90, 0.22))
        self.radio = RadioButtons(
            radio_axis,
            [scenario.name for scenario in SCENARIOS.values()],
            active=list(SCENARIOS).index(self.scenario_key),
        )
        self.radio.on_clicked(self._scenario_changed)
        slider_height = 0.025
        start = 0.68
        for index, (name, label, minimum, maximum) in enumerate(SLIDERS):
            axis = self.figure.add_axes((0.39, start - index * 0.045, 0.53, slider_height))
            value = getattr(self.config.rpp, name)
            slider = Slider(axis, label, minimum, maximum, valinit=value)
            self.sliders[name] = slider
        run_axis = self.figure.add_axes((0.08, 0.06, 0.35, 0.045))
        reset_axis = self.figure.add_axes((0.57, 0.06, 0.35, 0.045))
        pid_axis = self.figure.add_axes((0.25, 0.01, 0.50, 0.04))
        self.run_button = Button(run_axis, "运行 RPP")
        self.reset_button = Button(reset_axis, "恢复 YAML 参数")
        self.pid_button = Button(pid_axis, "打开速度 PID 阶跃响应")
        self.run_button.on_clicked(lambda _event: self.rerun())
        self.reset_button.on_clicked(self._reset)
        self.pid_button.on_clicked(self._open_pid)

    def _scenario_changed(self, label: str) -> None:
        self.scenario_key = next(key for key, value in SCENARIOS.items() if value.name == label)

    def _collect_config(self) -> ToolConfig:
        updates = {name: slider.val for name, slider in self.sliders.items()}
        return self.base_config.with_rpp(**updates)

    def rerun(self) -> None:
        self.config = self._collect_config()
        result = simulate_rpp(SCENARIOS[self.scenario_key], self.config)
        issues = validate_config(self.config)
        draw_rpp_result(self.result_figure, result, issues)
        severity_names = {"error": "错误", "warning": "警告"}
        issue_lines = [f"[{severity_names.get(i.severity, i.severity)}] {i.message}" for i in issues]
        # 每次运行复用同一个状态文本对象，避免文字不断叠加。
        if hasattr(self, "status_text"):
            self.status_text.remove()
        self.status_text = self.figure.text(
            0.01, 0.115, "\n".join(issue_lines[:5]) or "配置检查：正常",
            fontsize=7.5, color="darkred" if issues else "darkgreen", va="bottom",
        )
        self.figure.canvas.draw_idle()
        self.result_figure.canvas.draw_idle()

    def _reset(self, _event) -> None:
        for name, slider in self.sliders.items():
            slider.set_val(getattr(self.base_config.rpp, name))
        self.rerun()

    def _open_pid(self, _event) -> None:
        window = PIDDebugWindow(self.config.pid)
        self.pid_windows.append(window)
        window.control_figure.show()
        window.result_figure.show()


def launch(config: ToolConfig | Path | str) -> None:
    app = DebugApp(config)
    app.figure.canvas.manager.set_window_title("DEXE RPP/PID 参数")
    app.result_figure.canvas.manager.set_window_title("DEXE RPP/PID 仿真结果")
    plt.show()
