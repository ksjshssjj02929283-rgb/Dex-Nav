#!/usr/bin/env python3
"""RPP/PID 交互调试与无界面调试的命令行入口。"""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
from typing import Iterable


class ChineseArgumentParser(argparse.ArgumentParser):
    """将 argparse 自动生成的标题改为中文。"""

    def format_usage(self) -> str:
        return super().format_usage().replace("usage:", "用法：", 1)

    def format_help(self) -> str:
        return super().format_help().replace("usage:", "用法：", 1)


def _configure_matplotlib(headless: bool, output_dir: Path | None = None) -> None:
    cache = (output_dir or Path("/tmp")) / "dexe_rpp_debug_mpl"
    cache.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(cache))
    if headless:
        import matplotlib
        matplotlib.use("Agg")


def _parse_args() -> argparse.Namespace:
    from debug_tool.config import DEFAULT_CONFIG

    parser = ChineseArgumentParser(
        description="本仓库 RPP/PID 参数的离线可视化与批量检查工具。",
        add_help=False,
        usage="%(prog)s [选项]",
    )
    parser._optionals.title = "选项"
    parser.add_argument("-h", "--help", action="help", help="显示帮助信息并退出")
    parser.add_argument(
        "--config", type=Path, default=DEFAULT_CONFIG, metavar="YAML路径",
        help="要读取的 Nav2 YAML",
    )
    parser.add_argument(
        "--scenario", default="curve_1m", metavar="场景代号",
        help="仿真模式使用的场景代号",
    )
    parser.add_argument("--list-scenarios", action="store_true", help="列出所有标准场景")
    parser.add_argument("--check", action="store_true", help="仅输出配置检查结果")
    parser.add_argument("--batch", action="store_true", help="运行全部标准场景")
    parser.add_argument(
        "--output", type=Path, default=Path("debug_tool/output"),
        metavar="输出目录", help="输出目录",
    )
    parser.add_argument("--no-gui", action="store_true", help="不打开窗口，直接保存 PNG")
    parser.add_argument("--set", action="append", default=[], metavar="参数名=数值",
                        help="临时覆盖 RPP 参数，不写入 YAML")
    return parser.parse_args()


def _overrides(items: Iterable[str]) -> dict[str, float]:
    result = {}
    for item in items:
        if "=" not in item:
            raise ValueError(f"无效的临时参数 {item!r}；格式应为 参数名=数值")
        name, value = item.split("=", 1)
        result[name.strip()] = float(value)
    return result


def _print_issues(config) -> int:
    from debug_tool.config import validate_config

    issues = validate_config(config)
    if not issues:
        print("配置检查：正常")
        return 0
    parameter_names = {
        "lookahead": "前瞻距离",
        "heading_error_scaling": "朝向误差缩放",
        "heading/rotate": "朝向降速与原地转向",
        "rotate_to_path": "路径原地转向",
        "lateral_error": "横向误差",
        "inflation_cost_scaling_factor": "障碍距离反推系数",
        "max_angular_vel": "最大角速度",
        "max_linear_accel": "最大线加速度",
    }
    for issue in issues:
        severity = {"error": "错误", "warning": "警告"}.get(issue.severity, issue.severity)
        parameter = parameter_names.get(issue.parameter, issue.parameter)
        print(f"[{severity}] {parameter}：{issue.message}")
    return 2 if any(issue.severity == "error" for issue in issues) else 1


def _batch(config, output: Path) -> None:
    from debug_tool.plotting import draw_rpp_result
    from debug_tool.scenarios import SCENARIOS
    from debug_tool.simulator import simulate_rpp
    import matplotlib.pyplot as plt

    output.mkdir(parents=True, exist_ok=True)
    rows = []
    for key, scenario in SCENARIOS.items():
        result = simulate_rpp(scenario, config)
        figure = plt.figure(figsize=(13, 8))
        draw_rpp_result(figure, result)
        figure.savefig(output / f"{key}.png", dpi=140)
        plt.close(figure)
        metric_headers = {
            "completed": "是否完成",
            "collision": "是否触发碰撞停止",
            "duration_s": "持续时间_s",
            "max_cross_track_error_m": "最大横向误差_m",
            "mean_abs_cross_track_error_m": "平均绝对横向误差_m",
            "mean_speed_mps": "平均速度_mps",
            "max_abs_angular_speed_rps": "最大绝对角速度_rps",
            "rotate_entries": "进入原地转向次数",
            "minimum_obstacle_clearance_m": "最小障碍物余量_m",
        }
        row = {"场景": scenario.name}
        for metric_key, header in metric_headers.items():
            value = result.metrics[metric_key]
            row[header] = ("是" if value else "否") if metric_key in {"completed", "collision"} else value
        rows.append(row)
        print(
            f"{scenario.name:18} 完成={'是' if result.completed else '否'} "
            f"最大XTE={result.metrics['max_cross_track_error_m']:.3f}m "
            f"平均v={result.metrics['mean_speed_mps']:.3f}m/s "
            f"原地转向次数={int(result.metrics['rotate_entries'])}"
        )
    with (output / "summary.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"批量报告：{output.resolve()}")


def main() -> int:
    args = _parse_args()
    _configure_matplotlib(args.no_gui or args.batch, args.output)
    from debug_tool.config import load_tool_config
    from debug_tool.scenarios import SCENARIOS

    if args.list_scenarios:
        for key, scenario in SCENARIOS.items():
            print(f"{key:18} {scenario.name}")
        return 0

    config = load_tool_config(args.config)
    overrides = _overrides(args.set)
    unknown = sorted(set(overrides) - set(config.rpp.__dataclass_fields__))
    if unknown:
        raise ValueError(f"未知的 RPP 参数：{', '.join(unknown)}")
    if overrides:
        config = config.with_rpp(**overrides)

    if args.check:
        return _print_issues(config)
    if args.batch:
        _batch(config, args.output)
        return 0
    if args.scenario not in SCENARIOS:
        raise ValueError(f"未知场景 {args.scenario!r}；请使用 --list-scenarios 查看场景")

    if not args.no_gui:
        from debug_tool.app import launch
        launch(config)
        return 0

    from debug_tool.plotting import draw_rpp_result
    from debug_tool.simulator import simulate_rpp
    import matplotlib.pyplot as plt

    args.output.mkdir(parents=True, exist_ok=True)
    result = simulate_rpp(SCENARIOS[args.scenario], config)
    figure = plt.figure(figsize=(13, 8))
    draw_rpp_result(figure, result)
    target = args.output / f"{args.scenario}.png"
    figure.savefig(target, dpi=150)
    print(f"已保存：{target.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
