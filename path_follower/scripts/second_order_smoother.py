"""
二阶临界阻尼滤波器 —— AGV 速度平滑仿真
========================================
用途：对目标速度折线做平滑，实现 jerk 受控的加减速过渡，
      抑制障碍物触发的速度突变带来的车身晃动。

依赖：numpy, matplotlib
    pip install numpy matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# 1. 二阶临界阻尼滤波器
# ---------------------------------------------------------------------------
class SecondOrderSmoother:
    """
    二阶临界阻尼滤波器，天然实现 jerk 受控的平滑效果。

    连续时间模型：
        v'' + 2*zeta*wn*v' + wn^2*v = wn^2*v_target

    参数：
        omega_n: 自然频率，越大跟踪目标越快（越接近直接跟随），越小越迟缓/平滑
        zeta:    阻尼比，1.0 = 临界阻尼（无超调，最接近 S 型曲线），
                 <1 会有超调震荡，>1 响应更慢但更保守
    """

    def __init__(self, omega_n: float = 2.0, zeta: float = 1.0):
        self.zeta = zeta
        self.omega_n = omega_n
        self.v = 0.0   # 当前平滑后速度
        self.a = 0.0   # 当前加速度（连续变化，无跳变）

    def reset(self, v0: float = 0.0, a0: float = 0.0):
        self.v = v0
        self.a = a0

    def update(self, v_target, a_target, dt, omega_n=None):
        """
        a_target: 目标速度的变化率（前馈项）,斜坡段 = 斜坡斜率,平台段 = 0
        """
        wn = omega_n if omega_n is not None else self.omega_n
        # 误差动力学: 同时收敛速度误差和加速度误差
        jerk = wn**2 * (v_target - self.v) + 2*self.zeta*wn * (a_target - self.a)
        jerk = np.clip(jerk, -2, 2)   # ← 核心就这一行
        self.a += jerk * dt
        self.v += self.a * dt
        return self.v, self.a, jerk


# ---------------------------------------------------------------------------
# 2. 自适应 omega 选择：单调变化快跟踪，方向反转慢跟踪
# ---------------------------------------------------------------------------
def direction_reversed(v_tgt_prev2: float, v_tgt_prev1: float, v_tgt_now: float,
                       eps: float = 1e-3) -> bool:
    """
    判断目标速度是否发生"方向反转"（如障碍物突现导致目标速度掉头）。
    用离散差分比较符号变化，比数值梯度更抗噪声，适合实车逐周期调用。
    """
    d1 = v_tgt_prev1 - v_tgt_prev2
    d2 = v_tgt_now - v_tgt_prev1
    if abs(d1) < eps or abs(d2) < eps:
        return False
    return (d1 > 0) != (d2 > 0)


def adaptive_omega(reversed_flag: bool, omega_fast: float = 3.0, omega_slow: float = 0.8) -> float:
    """方向反转 -> 迟缓跟踪(omega_slow)；单调变化 -> 快速跟踪(omega_fast)"""
    return omega_slow if reversed_flag else omega_fast


# ---------------------------------------------------------------------------
# 3. 目标速度折线定义 (t, v) 分段线性插值
# ---------------------------------------------------------------------------
WAYPOINTS = [
    (0, 0.0), (2, 1.0), (5, 1.0), (6, 0.5), (7, 1.0),
    (8, 0.5), (9, 0.5), (10, 1.0), (12, 0.0),
]


def v_target_func(t: float, waypoints=WAYPOINTS) -> float:
    ts = np.array([p[0] for p in waypoints])
    vs = np.array([p[1] for p in waypoints])
    return float(np.interp(t, ts, vs))


# ---------------------------------------------------------------------------
# 4. 仿真主流程
# ---------------------------------------------------------------------------
def run_simulation(dt: float = 0.02, omega_fast: float = 3.0, omega_slow: float = 0.8,
                   zeta: float = 1.0):
    t_end = WAYPOINTS[-1][0]
    t_arr = np.arange(0, t_end + dt, dt)

    smoother = SecondOrderSmoother(omega_n=omega_fast, zeta=zeta)

    v_tgt_hist = np.zeros_like(t_arr)
    v_out = np.zeros_like(t_arr)
    a_out = np.zeros_like(t_arr)
    jerk_out = np.zeros_like(t_arr)
    omega_used = np.zeros_like(t_arr)

    # 维护最近两个目标速度用于方向反转判断
    v_prev2 = v_target_func(0.0)
    v_prev1 = v_target_func(0.0)

    for i, t in enumerate(t_arr):
        v_tgt = v_target_func(t)
        v_tgt_hist[i] = v_tgt

        rev = direction_reversed(v_prev2, v_prev1, v_tgt)
        if 0:
            wn = omega_slow
            a_ff = 0.0          # 反转时把前馈掐掉,让系统按自身惯性柔和转向
        else:
            wn = omega_fast
            a_ff = (v_tgt - v_prev1) / dt   # 单调段全前馈,零滞后跟踪

        v, a, j = smoother.update(v_tgt, a_ff, dt, omega_n=wn)
        v_out[i] = v
        a_out[i] = a
        jerk_out[i] = j
        omega_used[i] = wn

        v_prev2, v_prev1 = v_prev1, v_tgt

    return {
        "t": t_arr,
        "v_target": v_tgt_hist,
        "v_smooth": v_out,
        "a_smooth": a_out,
        "jerk": jerk_out,
        "omega_used": omega_used,
    }


# ---------------------------------------------------------------------------
# 5. 绘图
# ---------------------------------------------------------------------------
def plot_result(result: dict):
    t = result["t"]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    # ---- 速度图 ----
    ax1.plot(t, result["v_target"], color="0.6", lw=1.3, ls="--", label="v_target (目标折线)")
    ax1.plot(t, result["v_smooth"], color="tab:blue", lw=2.2, label="v_smooth (滤波后)")
    ax1.set_ylabel("velocity (m/s)")
    ax1.set_title("二阶临界阻尼滤波：速度平滑效果")
    ax1.legend(loc="upper right")
    ax1.grid(alpha=0.3)

    # ---- 加速度 / omega 图 ----
    ax2.plot(t, result["a_smooth"], color="tab:orange", lw=1.8, label="a_smooth (加速度)")
    ax2.set_ylabel("acceleration (m/s^2)")
    ax2.set_xlabel("time (s)")
    ax2.grid(alpha=0.3)

    # 用背景色区分 fast / slow 跟踪区段（阴影标注 omega 切换）
    ax2b = ax2.twinx()
    ax2b.plot(t, result["omega_used"], color="tab:green", lw=1.0, alpha=0.5,
              ls=":", label="omega_n (右轴)")
    ax2b.set_ylabel("omega_n", color="tab:green")
    ax2b.tick_params(axis="y", labelcolor="tab:green")

    lines1, labels1 = ax2.get_legend_handles_labels()
    lines2, labels2 = ax2b.get_legend_handles_labels()
    ax2.legend(lines1 + lines2, labels1 + labels2, loc="upper right")

    plt.tight_layout()
    # plt.savefig("second_order_smoother_result.png", dpi=130)
    # print("图片已保存: second_order_smoother_result.png")
    plt.show()


# ---------------------------------------------------------------------------
# 6. 入口
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    result = run_simulation(
        dt=0.02,
        omega_fast=3.0,   # 单调段跟踪速度，越大越贴近目标
        omega_slow=0.8,   # 反转段跟踪速度，越小越平滑但滞后越大
        zeta=1.0,         # 临界阻尼，无超调
    )

    print(f"最大加速度绝对值: {np.max(np.abs(result['a_smooth'])):.4f} m/s^2")
    print(f"最大jerk绝对值:   {np.max(np.abs(result['jerk'])):.4f} m/s^3")

    plot_result(result)