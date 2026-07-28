# Copyright 2026 DEXE Robot Engineer
# SPDX-License-Identifier: Apache-2.0
"""
Nav monitor 分析脚本公共工具：数据目录解析.

解析顺序（谁先命中用谁）：
  1. 命令行显式传入的目录/文件
  2. 运行中 nav2_nav_monitor 节点的 output_dir 参数（ros2 param get）
  3. 节点上次启动写下的 marker 文件 ~/.dexe/navigation/.nav_monitor_last_output_dir
  4. 内置默认 ~/.dexe/navigation/nav_monitor
这样脚本默认永远跟随节点实际配置的 output_dir，YAML 改目录后脚本无需改参数。
"""
import os
import subprocess

DEFAULT_DIR = "~/.dexe/navigation/nav_monitor"
MARKER_FILE = "~/.dexe/navigation/.nav_monitor_last_output_dir"


def _from_running_node(timeout=3.0):
    try:
        out = subprocess.run(
            ["ros2", "param", "get", "/nav2_nav_monitor", "output_dir"],
            capture_output=True, text=True, timeout=timeout)
        if out.returncode == 0 and "String value is:" in out.stdout:
            val = out.stdout.split("String value is:", 1)[1].strip()
            if val:
                return val
    except (subprocess.TimeoutExpired, FileNotFoundError, OSError):
        pass
    return None


def _from_marker():
    try:
        with open(os.path.expanduser(MARKER_FILE)) as f:
            val = f.read().strip()
            if val:
                return val
    except OSError:
        pass
    return None


def resolve_data_dir(cli_arg=None, quiet=False):
    """按解析顺序返回数据目录（已展开 ~）；quiet=False 时打印来源."""
    if cli_arg:
        return os.path.expanduser(cli_arg)
    val = _from_running_node()
    if val:
        if not quiet:
            print(f"数据目录（来自运行中节点的 output_dir 参数）：{val}")
        return os.path.expanduser(val)
    val = _from_marker()
    if val:
        if not quiet:
            print(f"数据目录（来自节点上次启动的 marker 记录）：{val}")
        return os.path.expanduser(val)
    if not quiet:
        print(f"数据目录（内置默认）：{DEFAULT_DIR}")
    return os.path.expanduser(DEFAULT_DIR)
