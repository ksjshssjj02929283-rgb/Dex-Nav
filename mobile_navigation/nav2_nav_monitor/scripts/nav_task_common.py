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


# ---------------------------------------------------------------------------
# 采样断档处理
#
# 采样话题在某些阶段会整段停发（典型：hard stop 后 BT 切到 recovery 行为，
# controller_server 的 FollowPath 结束，规划管线随之停摆，desired_vel /
# controller_target 可能几秒钟一个样本都没有）。matplotlib 的折线会把断档
# 两端直接连成直线，看上去像一条平滑的衰减曲线——实测出现过 5.3 秒只有
# 1 个样本、却被画成"从 0.600 线性降到 0"的情况，直接把排查引向错误的因果。
#
# 断档是**真实信息**（那段没有规划输出），必须让它在图上表现为断开而不是插值。
# ---------------------------------------------------------------------------

DEFAULT_GAP_FACTOR = 5.0
DEFAULT_MIN_GAP = 0.30


def detect_sample_gaps(times, gap_factor=DEFAULT_GAP_FACTOR,
                       min_gap=DEFAULT_MIN_GAP):
    """返回 [(gap_start, gap_end), ...]：相邻样本间隔异常大的区间.

    判据取「gap_factor × 本序列间隔中位数」与 min_gap 的较大者，
    这样既适应不同话题的固有频率（100Hz odom 与 10Hz 诊断阈值不同），
    又不会因为个别抖动就把线打断。
    """
    if len(times) < 3:
        return []
    deltas = sorted(times[i] - times[i - 1] for i in range(1, len(times)))
    median_dt = deltas[len(deltas) // 2]
    threshold = max(gap_factor * median_dt, min_gap)
    return [(times[i - 1], times[i])
            for i in range(1, len(times))
            if times[i] - times[i - 1] > threshold]


def split_on_gaps(times, *series, gap_factor=DEFAULT_GAP_FACTOR,
                  min_gap=DEFAULT_MIN_GAP):
    """在断档处插入 NaN，使 matplotlib 断线而不是跨断档插值.

    返回 (times, *series, gaps)；times/series 已插入断点，gaps 为断档区间列表，
    供调用方额外标注（只断线不标注的话，读图人仍不知道那里是"没数据"）。
    """
    gaps = detect_sample_gaps(times, gap_factor, min_gap)
    if not gaps:
        return (list(times), *[list(s) for s in series], gaps)
    gap_starts = {g[0] for g in gaps}
    out_t, out_s = [], [[] for _ in series]
    for index, t in enumerate(times):
        out_t.append(t)
        for slot, s in enumerate(series):
            out_s[slot].append(s[index])
        if t in gap_starts and index + 1 < len(times):
            out_t.append(float("nan"))
            for slot in range(len(series)):
                out_s[slot].append(float("nan"))
    return (out_t, *out_s, gaps)
