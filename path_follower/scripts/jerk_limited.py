import numpy as np
import matplotlib.pyplot as plt

def two_phase_decel(v0, a_max, jerk=None, v_mid=None, dt=1e-3):
    """
    两段式减速剖面:
      阶段1: v0 -> v_mid, 恒减速 a_max
      阶段2: v_mid -> 0,  减速度从 a_max 线性撤到 0 (jerk受控)
    约束: v_mid = a_max^2 / (2*jerk), 给定 jerk 或 v_mid 其一, 另一个自动求出
    返回: t, v, a, s (时间/速度/加速度/累计距离数组)
    """
    if jerk is None and v_mid is None:
        raise ValueError("jerk 和 v_mid 至少给一个")
    if v_mid is None:
        v_mid = a_max**2 / (2 * jerk)
    else:
        jerk = a_max**2 / (2 * v_mid)
    assert v_mid < v0, f"v_mid({v_mid:.3f}) 必须小于 v0({v0})"

    t1 = (v0 - v_mid) / a_max      # 阶段1时长
    t2 = a_max / jerk              # 阶段2时长
    T = t1 + t2

    t = np.arange(0, T + dt, dt)
    v = np.empty_like(t)
    a = np.empty_like(t)

    ph1 = t <= t1
    v[ph1] = v0 - a_max * t[ph1]
    a[ph1] = -a_max

    tau = t[~ph1] - t1             # 阶段2局部时间
    a[~ph1] = -a_max + jerk * tau
    v[~ph1] = v_mid - a_max * tau + 0.5 * jerk * tau**2

    v = np.clip(v, 0, None)
    s = np.cumsum(v) * dt
    print(f"jerk={jerk:.3f} m/s^3, v_mid={v_mid:.3f} m/s, "
          f"阶段1: {t1:.2f}s, 阶段2: {t2:.2f}s, 总时长 {T:.2f}s, 制动距离 {s[-1]:.3f} m")
    return t, v, a, s

def export_lookup_table(t, v, s, ds=0.005, fname="decel_table.csv"):
    """按剩余距离 5mm 间隔导出查找表: dist_to_go -> v"""
    s_total = s[-1]
    d_togo = np.arange(0, s_total, ds)[::-1]
    v_tab = np.interp(s_total - d_togo, s, v)
    np.savetxt(fname, np.column_stack([d_togo, v_tab]),
               header="dist_to_go_m,v_mps", delimiter=",", fmt="%.5f")
    print(f"查找表已导出 {fname}, 共 {len(d_togo)} 点")

# ===== 两种参数模式 =====
# 模式A: 固定 jerk=0.1, v_mid 自动 => 0.45
tA, vA, aA, sA = two_phase_decel(v0=0.7, a_max=0.3, jerk=0.1)
# 模式B: 固定 v_mid=0.3, jerk 自动 => 0.15
tB, vB, aB, sB = two_phase_decel(v0=0.7, a_max=0.3, v_mid=0.3)

# 对比: 纯梯形
tT = np.arange(0, 0.7/0.3 + 1e-3, 1e-3)
vT = np.clip(0.7 - 0.3*tT, 0, None)

# ===== 绘图 =====
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)

ax1.plot(tA, vA, 'b-',  lw=2, label='两段式 A: jerk=0.1, v_mid=0.45')
ax1.plot(tB, vB, 'g-',  lw=2, label='两段式 B: v_mid=0.3, jerk=0.15')
ax1.plot(tT, vT, 'r--', lw=1.5, label='纯梯形 (0.3 m/s$^2$)')
ax1.axhline(0.45, color='b', ls=':', lw=0.8, alpha=0.5)
ax1.axhline(0.30, color='g', ls=':', lw=0.8, alpha=0.5)
ax1.set_ylabel('v (m/s)'); ax1.legend(); ax1.grid(alpha=0.3)
ax1.set_title('两段式减速: 恒减速段 + jerk收敛段')

ax2.plot(tA, aA, 'b-',  lw=2)
ax2.plot(tB, aB, 'g-',  lw=2)
ax2.plot(tT, np.where(vT > 0, -0.3, 0), 'r--', lw=1.5)
ax2.set_xlabel('t (s)'); ax2.set_ylabel('a (m/s$^2$)')
ax2.grid(alpha=0.3)
ax2.set_title('减速度: 梯形在终点跳变(红), 两段式线性归零(蓝/绿)')

plt.tight_layout()
plt.savefig('two_phase_decel.png', dpi=120)
plt.show()

export_lookup_table(tB, vB, sB)   # 导出模式B的距离查找表