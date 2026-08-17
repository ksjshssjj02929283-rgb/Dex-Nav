"""交互界面与无界面报告共用的 Matplotlib 绘图函数。"""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager, rcParams

from .pid_lab import PIDResponse
from .simulator import SimulationResult


def _configure_cjk_font() -> None:
    """优先使用已安装的中文字体，保证场景名称能够正常显示。"""
    installed = {font.name for font in font_manager.fontManager.ttflist}
    preferred = (
        "Noto Sans CJK SC",
        "Noto Sans CJK JP",
        "Droid Sans Fallback",
        "AR PL UMing CN",
    )
    selected = next((name for name in preferred if name in installed), None)
    if selected:
        rcParams["font.sans-serif"] = [selected, "DejaVu Sans"]
    rcParams["axes.unicode_minus"] = False


_configure_cjk_font()


def draw_rpp_result(figure, result: SimulationResult, issues=None) -> None:
    figure.clear()
    grid = figure.add_gridspec(3, 2, height_ratios=(1.2, 1.0, 1.0))
    path_axis = figure.add_subplot(grid[:, 0])
    speed_axis = figure.add_subplot(grid[0, 1])
    geometry_axis = figure.add_subplot(grid[1, 1])
    angular_axis = figure.add_subplot(grid[2, 1])

    path = result.scenario.path
    path_axis.plot(path[:, 0], path[:, 1], "k--", linewidth=1.5, label="规划路径")
    if len(result.pose):
        path_axis.plot(result.pose[:, 0], result.pose[:, 1], color="tab:blue", label="机器人轨迹")
        stride = max(1, len(result.pose) // 14)
        path_axis.scatter(
            result.main_carrot[::stride, 0], result.main_carrot[::stride, 1],
            s=14, color="tab:orange", alpha=0.65, label="主前瞻点",
        )
        path_axis.scatter(
            result.near_carrot[::stride, 0], result.near_carrot[::stride, 1],
            s=10, color="tab:green", alpha=0.55, label="近场前瞻点",
        )
    if result.scenario.obstacle_xy is not None:
        obstacle = plt.Circle(
            result.scenario.obstacle_xy,
            result.scenario.obstacle_radius,
            color="tab:red",
            alpha=0.35,
            label="动态障碍物",
        )
        path_axis.add_patch(obstacle)
    path_axis.set_aspect("equal", adjustable="datalim")
    path_axis.grid(True, alpha=0.25)
    path_axis.set_title(f"{result.scenario.name}：路径与闭环轨迹")
    path_axis.set_xlabel("x [m]")
    path_axis.set_ylabel("y [m]")
    path_axis.legend(loc="best", fontsize=8)

    time = result.time
    channels = result.channels
    for key, label, style in (
        ("planner", "规划速度", "-"),
        ("curvature_cap", "曲率约束后", "--"),
        ("obstacle_cap", "障碍约束后", "--"),
        ("lateral_cap", "横向误差约束后", "--"),
        ("heading_cap", "朝向误差约束后", ":"),
        ("angular_capacity_cap", "最终约束", "-"),
        ("controller_v", "PID 输出", ":"),
        ("smoother_v", "平滑器输出", "-."),
        ("measured_v", "实测速度", "-"),
    ):
        speed_axis.plot(time, channels[key], style, linewidth=1.2, label=label)
    speed_axis.set_ylabel("线速度 [m/s]")
    speed_axis.grid(True, alpha=0.25)
    speed_axis.legend(ncol=3, fontsize=7)

    geometry_axis.plot(time, channels["curvature"], label="主前瞻曲率")
    geometry_axis.plot(time, channels["near_curvature"], label="近场曲率", alpha=0.8)
    geometry_axis.plot(time, channels["xte"], label="横向误差")
    geometry_axis.plot(time, np.abs(channels["heading_error"]), label="|朝向误差| [rad]")
    geometry_axis.grid(True, alpha=0.25)
    geometry_axis.legend(ncol=2, fontsize=7)

    angular_axis.plot(time, channels["controller_w"], label="控制器 w")
    angular_axis.plot(time, channels["smoother_w"], label="平滑器 w")
    angular_axis.plot(time, channels["measured_w"], label="实测 w", alpha=0.8)
    rotate_mask = np.asarray([state == "ROTATE" for state in result.states], dtype=bool)
    if len(time) and np.any(rotate_mask):
        angular_axis.fill_between(
            time, 0.0, 1.0, where=rotate_mask,
            color="tab:orange", alpha=0.12, transform=angular_axis.get_xaxis_transform(),
            label="原地转向状态",
        )
    angular_axis.set_xlabel("时间 [s]")
    angular_axis.set_ylabel("角速度 [rad/s]")
    angular_axis.grid(True, alpha=0.25)
    angular_axis.legend(ncol=2, fontsize=7)

    metrics = result.metrics
    status = "已完成" if result.completed else "未完成"
    text = (
        f"{status}；最大 XTE={metrics['max_cross_track_error_m']:.3f}m；"
        f"平均速度={metrics['mean_speed_mps']:.3f}m/s；"
        f"进入原地转向次数={int(metrics['rotate_entries'])}"
    )
    if issues:
        text += f"；配置问题数={len(issues)}"
    figure.suptitle(text, fontsize=10)
    figure.tight_layout(rect=(0, 0, 1, 0.965))


def draw_pid_response(figure, response: PIDResponse) -> None:
    figure.clear()
    speed_axis, error_axis = figure.subplots(2, 1, sharex=True)
    speed_axis.plot(response.time, response.target, "k--", label="目标速度")
    speed_axis.plot(response.time, response.command, label="指令速度")
    speed_axis.plot(response.time, response.measured, label="实测速度")
    speed_axis.set_ylabel("速度 [m/s]")
    speed_axis.grid(True, alpha=0.25)
    speed_axis.legend()
    error_axis.plot(response.time, response.error, color="tab:red")
    error_axis.axhline(0.0, color="black", linewidth=0.7)
    error_axis.set_ylabel("误差 [m/s]")
    error_axis.set_xlabel("时间 [s]")
    error_axis.grid(True, alpha=0.25)
    figure.tight_layout()
