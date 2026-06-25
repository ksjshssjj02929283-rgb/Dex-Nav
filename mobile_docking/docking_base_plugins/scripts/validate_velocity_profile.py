#!/usr/bin/env python3
"""StraightLinePlanner 梯形速度剖面数值验证（落 C++ 前的 gate）。

复刻 planner 沿直线段生成的逐点纵向速度剖面：0 → 巡航 → 0（末端停在目标），
支持倒车（负速）+ 段过短退化为三角形。喂边界态实测 accel/jerk 在约束内、末端速度=0。
通过后才把同一算法落到 C++ StraightLinePlanner。
"""
import math


def trapezoidal_speed(s, length, v_max, a_max):
    """返回沿弧长 s∈[0,length] 处的速度幅值（>=0）。梯形/三角形自动选择。"""
    if length <= 1e-9:
        return 0.0
    s = max(0.0, min(length, s))
    d_acc = v_max * v_max / (2.0 * a_max)          # 0→v_max 加速距离
    d_dec = d_acc                                   # v_max→0 减速距离
    if d_acc + d_dec <= length:                     # 梯形：够长能到 v_max
        if s < d_acc:
            return math.sqrt(2.0 * a_max * s)
        if s <= length - d_dec:
            return v_max
        return math.sqrt(max(0.0, 2.0 * a_max * (length - s)))
    # 三角形：太短，对称加减速，峰值 < v_max
    s_peak = length / 2.0
    if s < s_peak:
        return math.sqrt(2.0 * a_max * s)
    return math.sqrt(max(0.0, 2.0 * a_max * (length - s)))


def sample_profile(length, v_max, a_max, resolution, reverse=False):
    """按分辨率重采样段路径，返回 [(s, v_signed)]。v 末端=0。"""
    n = max(1, int(math.ceil(length / resolution)))
    pts = []
    for i in range(n + 1):
        s = min(length, i * resolution)
        v = trapezoidal_speed(s, length, v_max, a_max)
        pts.append((s, -v if reverse else v))
    return pts


def validate(name, length, v_max, a_max, resolution, reverse=False,
             a_tol=1e-3, jerk_limit=None):
    pts = sample_profile(length, v_max, a_max, resolution, reverse)
    max_abs_v = max(abs(v) for _, v in pts)
    end_v = pts[-1][1]
    # 逐段反推 accel（dt=ds/v_avg）+ jerk
    accels, dts = [], []
    for i in range(len(pts) - 1):
        ds = pts[i + 1][0] - pts[i][0]
        v0, v1 = abs(pts[i][1]), abs(pts[i + 1][1])
        v_avg = (v0 + v1) / 2.0
        if ds <= 1e-9 or v_avg <= 1e-6:
            continue
        dt = ds / v_avg
        accels.append((v1 - v0) / dt)
        dts.append(dt)
    max_abs_a = max((abs(a) for a in accels), default=0.0)
    jerks = []
    for i in range(len(accels) - 1):
        dt = (dts[i] + dts[i + 1]) / 2.0
        if dt > 1e-9:
            jerks.append((accels[i + 1] - accels[i]) / dt)
    max_abs_j = max((abs(j) for j in jerks), default=0.0)

    ok_v = max_abs_v <= v_max + 1e-6
    ok_end = abs(end_v) <= 1e-6
    ok_a = max_abs_a <= a_max + a_tol
    ok_jerk = (jerk_limit is None) or (max_abs_j <= jerk_limit)
    ok = ok_v and ok_end and ok_a and ok_jerk

    print(f"[{'PASS' if ok else 'FAIL'}] {name}")
    print(f"    L={length:.3f} v_max={v_max:.3f} a_max={a_max:.3f} res={resolution:.3f} "
          f"reverse={reverse} pts={len(pts)}")
    print(f"    max|v|={max_abs_v:.4f} (<= {v_max})  end_v={end_v:+.5f} (~0)  "
          f"max|a|={max_abs_a:.4f} (<= {a_max})  max|jerk|={max_abs_j:.3f}")
    assert ok_v, f"{name}: 速度超限 {max_abs_v} > {v_max}"
    assert ok_end, f"{name}: 末端速度未归零 {end_v}"
    assert ok_a, f"{name}: 加速度超限 {max_abs_a} > {a_max}"
    if jerk_limit is not None:
        assert ok_jerk, f"{name}: jerk 超限 {max_abs_j} > {jerk_limit}"
    return ok


if __name__ == "__main__":
    print("=== StraightLinePlanner 梯形速度剖面数值验证 ===\n")
    # 1 正常充电倒车贴桩段（DRAWBACK：drawback_point→docking_stop_point ≈0.23m，慢速倒车）
    validate("充电倒车贴桩(梯形/倒车)", length=0.23, v_max=0.03, a_max=0.10,
             resolution=0.05, reverse=True)
    # 2 出桩前进段（DockingExit exit_distance=0.4）
    validate("出桩前进(梯形)", length=0.40, v_max=0.05, a_max=0.20, resolution=0.05)
    # 3 段过短退化三角形（峰值 < v_max）
    validate("短段退化三角形", length=0.06, v_max=0.10, a_max=0.20, resolution=0.02)
    # 4 极短段（1-2 点）
    validate("极短段", length=0.01, v_max=0.05, a_max=0.20, resolution=0.05)
    # 5 长段巡航为主
    validate("长段巡航", length=1.50, v_max=0.15, a_max=0.20, resolution=0.05)
    # 6 高分辨率充电段（更细采样下 accel 仍达上限不超）
    validate("高分辨率充电段", length=0.23, v_max=0.03, a_max=0.10, resolution=0.01)
    print("\n=== 全部边界态通过：梯形速度剖面可落 C++ ===")
