"""
Retime a corner-smoothed path for a differential-drive base
============================================================

折线 [0,0] -> [2,0] -> [2,2]，拐角按 ``scripts/corner_bezier_path_generator.py``
的四次贝塞尔锚点方案倒角，整体形态 直线 -> 四次贝塞尔 -> 直线，
再用 TOPPRA 做时间参数化，直接给出差速底盘要下发的 (v, omega)。

一、为什么自由度是 (弧长, 航向) 而不是 (x, y)
----------------------------------------------
差速底盘只有 v、omega 两个控制量，车头始终沿轨迹切线。把位形取成

    q(s) = [ sigma(s), theta(s) ] = [ s, theta(s) ]      # 用弧长本身做路径参数

弧长参数化下 |dp/ds| = 1，于是 dtheta/ds 正好是曲率 kappa：

    q'(s)  = [ 1, kappa(s)  ]
    q''(s) = [ 0, kappa'(s) ]

代进 TOPPRA 的链式法则，两个自由度的"关节速度/加速度"就是我们要的量：

    qd  = q'(s) * sd               ->  v = sd            omega = kappa(s) * sd
    qdd = q'(s)*sdd + q''(s)*sd^2  ->  a = sdd           alpha = kappa*sdd + kappa'*sd^2

于是 JointVelocityConstraint 的两行就是 [v_max, omega_max]，
JointAccelerationConstraint 的两行就是 [a_max, alpha_max]，不需要任何额外包装。

二、v 和 omega 相关，这个耦合怎么处理
--------------------------------------
不需要单独处理 —— 耦合已经写在 q'(s) = [1, kappa] 里了。TOPPRA 的做法就是把所有
自由度的约束统统映射到同一个标量 sd 上，各条约束天然取交集：

    |v|     <= v_max      ->  sd <= v_max
    |omega| <= omega_max  ->  sd <= omega_max / |kappa(s)|          # 曲率越大越慢
    合起来                ->  sd <= min(v_max, omega_max/|kappa|)

三、kappa'(s) 必须解析给出，不能靠样条二阶导
--------------------------------------------
alpha 里带 kappa'。直线段 kappa' = 0，贝塞尔起点 kappa'(A+) = 4.17（本例），
即 kappa' 在接入点 A、接出点 B 上是**跳变**的。三次样条的二阶导连续，表示不了
跳变，会在 A/B 附近振铃过冲约 13%，TOPPRA 拿到的 kappa' 偏大又偏小，最终输出的
alpha 会超限约 17%，且加密网格无法收敛。因此这里不拟合样条，直接用贝塞尔的解析
一阶/二阶/三阶导数造一张 (s, theta, kappa, kappa') 表，做成路径对象喂给 TOPPRA。
四、和 path_follower 现有流水线的对照
--------------------------------------
脚本末尾附一组对照曲线，复刻 path_follower 当前的三步做法：
    Ruckig 1D（沿弧长）-> 曲率软钳 -> 加减速包络（U=v^2 双扫描）
其中软钳 ratio = max(ratio_min, gain / (w^2 + gain))，w = v * kappa，
包络就是 velocity_planner.cpp::applyAccelerationEnvelope 那两遍扫描。
对照组的曲率直接取解析值（不走三点外接圆），这样比较的是**算法本身**，
而不是曲率来源的差异。
"""
################################################################################
# Import necessary libraries.
import os
import sys

import numpy as np
import matplotlib.pyplot as plt

import ruckig
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
from toppra.interpolator import AbstractGeometricPath

# 直接复用 ROS 节点里的倒角实现，保证离线分析与上车轨迹完全一致
# sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
os.chdir('/home/xmk/project/dex/mobile/dexe_mobile_nav/path_follower/scripts/')
from corner_bezier_path_generator import (  # noqa: E402
    bezier_derivative_cp,
    bezier_points,
    build_smoothed_polyline,
    resample_by_arc_length,
)

ta.setup_logging("INFO")

################################################################################
# 折线顶点与倒角参数（与 corner_bezier_path_generator 的 ROS 参数同名同义）
WAYPTS = np.array([[0.0, 0.0],
                   [2.0, 0.0],
                   [2.0, 2.0]])
BEZIER_DEGREE = 4        # 四次
ANCHOR_SPACING = 0.3     # 退让距离 d = (degree - 2) * anchor_spacing = 0.6 m
RESOLUTION = 0.005       # xy 轨迹弧长重采样间距（画图/下发用）
DENSE_STEP = 0.001       # 贝塞尔离散步长，必须明显细于 RESOLUTION

# 差速底盘限幅
V_MAX = 0.7              # 线速度 m/s
W_MAX = 1.0              # 角速度 rad/s
A_MAX = 0.5              # 线加速度 m/s^2
ALPHA_MAX = 0.7          # 角加速度 rad/s^2

# 对照组（path_follower 现有流水线）专用参数，取自 path_follower.yaml 的同名项
JERK_MAX = 1.0           # 线 jerk 上限 m/s^3（max_linear_jerk）
MIN_V = 0.02             # 起终点速度种子 = 底盘死区（min_linear_velocity）
CURV_DEC_GAIN = 0.3      # curvature_dec_gain
CURV_DEC_RATIO_MIN = 0.2 # curvature_dec_ratio_min
CTRL_DT = 0.02           # ruckig_control_dt


################################################################################
# 解析的 (弧长, 航向) 路径对象
################################################################################
class HeadingPath(AbstractGeometricPath):
    """q(s) = [s, theta(s)]，一/二阶导给出 [1, kappa] 和 [0, kappa']。

    直线段 kappa = kappa' = 0；贝塞尔段用控制点降阶求导，解析算出
    kappa 与 dkappa/ds。表格按弧长存，取值用线性插值 —— 关键是在接入/接出点
    两侧各放一个样本，让 kappa' 的跳变被如实保留（样条做不到这点）。
    """

    def __init__(self, waypoints, ctrl_list, n_bezier=4001, eps=1e-9):
        s_tab, th_tab, k_tab, kd_tab = [], [], [], []
        cursor = np.asarray(waypoints[0], dtype=float)
        s0 = 0.0

        def add_line(p0, p1, s_start):
            length = float(np.linalg.norm(p1 - p0))
            ang = float(np.arctan2(p1[1] - p0[1], p1[0] - p0[0]))
            s_tab.extend([s_start, s_start + length - eps])
            th_tab.extend([ang, ang])
            k_tab.extend([0.0, 0.0])
            kd_tab.extend([0.0, 0.0])
            return s_start + length, ang

        for cp in ctrl_list:
            a_pt, b_pt = cp[0], cp[-1]
            s0, _ = add_line(cursor, a_pt, s0)

            d1 = bezier_derivative_cp(cp, 1)
            d2 = bezier_derivative_cp(cp, 2)
            d3 = bezier_derivative_cp(cp, 3)
            u = np.linspace(0.0, 1.0, n_bezier)
            r1, r2, r3 = bezier_points(d1, u), bezier_points(d2, u), bezier_points(d3, u)
            spd = np.linalg.norm(r1, axis=1)
            cross12 = r1[:, 0] * r2[:, 1] - r1[:, 1] * r2[:, 0]
            kappa = cross12 / spd ** 3
            # dkappa/du = (r1 x r3)/|r1|^3 - 3 (r1.r2)(r1 x r2)/|r1|^5，再除以 |r1| 换到 ds
            cross13 = r1[:, 0] * r3[:, 1] - r1[:, 1] * r3[:, 0]
            dot12 = np.einsum('ij,ij->i', r1, r2)
            dkappa_du = cross13 / spd ** 3 - 3.0 * dot12 * cross12 / spd ** 5
            kappa_s = dkappa_du / spd
            arc = np.concatenate([[0.0], np.cumsum(0.5 * (spd[1:] + spd[:-1]) * np.diff(u))])
            theta = np.unwrap(np.arctan2(r1[:, 1], r1[:, 0]))

            s_tab.extend(s0 + arc)
            th_tab.extend(theta)
            k_tab.extend(kappa)
            kd_tab.extend(kappa_s)
            s0 += float(arc[-1])
            cursor = b_pt

        s0, _ = add_line(cursor, np.asarray(waypoints[-1], dtype=float), s0)

        self._s = np.asarray(s_tab)
        self._theta = np.asarray(th_tab)
        self._kappa = np.asarray(k_tab)
        self._kappa_s = np.asarray(kd_tab)
        self._length = float(self._s[-1])

    @property
    def dof(self):
        return 2

    @property
    def path_interval(self):
        return np.array([0.0, self._length])

    def _lookup(self, s, table):
        return np.interp(s, self._s, table)

    def __call__(self, s, order=0):
        scalar = np.ndim(s) == 0
        sa = np.atleast_1d(np.asarray(s, dtype=float))
        if order == 0:
            out = np.column_stack([sa, self._lookup(sa, self._theta)])
        elif order == 1:
            out = np.column_stack([np.ones_like(sa), self._lookup(sa, self._kappa)])
        elif order == 2:
            out = np.column_stack([np.zeros_like(sa), self._lookup(sa, self._kappa_s)])
        else:
            raise ValueError("order must be 0, 1 or 2")
        return out[0] if scalar else out

    def eval(self, s):
        return self(s, 0)

    def evald(self, s):
        return self(s, 1)

    def evaldd(self, s):
        return self(s, 2)


################################################################################
# 第一步：折线 -> 直线 + 四次贝塞尔 + 直线
dense, ctrl_list = build_smoothed_polyline(
    WAYPTS, ANCHOR_SPACING, DENSE_STEP, BEZIER_DEGREE)
positions = resample_by_arc_length(dense, RESOLUTION)      # xy 稠密轨迹，画图/下发用
xy_s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(positions, axis=0), axis=1))])

path = HeadingPath(WAYPTS, ctrl_list)
s_grid = np.linspace(*path.path_interval, 4000)
kappa = path(s_grid, 1)[:, 1]
kappa_d = path(s_grid, 2)[:, 1]
cp = ctrl_list[0]
print("倒角: degree={} d={:.3f}m  kappa_max={:.3f} (R_min={:.3f}m)  kappa'(A+)={:.3f}"
      .format(BEZIER_DEGREE, np.linalg.norm(cp[0] - WAYPTS[1]),
              np.abs(kappa).max(), 1.0 / np.abs(kappa).max(),
              kappa_d[np.argmax(np.abs(kappa_d) > 1e-9)]))
print("路径: 总弧长 {:.3f} m, 累计转角 {:.1f} deg"
      .format(path.path_interval[1],
              np.degrees(path(path.path_interval[1])[1] - path(0.0)[1])))

################################################################################
# Setup the velocity and acceleration.
#   dof0 = 弧长 -> qd0 = v,     qdd0 = a
#   dof1 = 航向 -> qd1 = omega, qdd1 = alpha
pc_vel = constraint.JointVelocityConstraint(
    np.array([[-V_MAX, V_MAX],
              [-W_MAX, W_MAX]]))
pc_acc = constraint.JointAccelerationConstraint(
    np.array([[-A_MAX, A_MAX],
              [-ALPHA_MAX, ALPHA_MAX]]),
    discretization_scheme=constraint.DiscretizationType.Interpolation)

################################################################################
# Setup the problem instance and solve it.
# parametrizer 用 ConstAccel：默认的 ParametrizeSpline 会把结果在时间域重新拟合
# 成三次样条，输出可能轻微越界。
gridpoints = np.linspace(*path.path_interval, 2000)
instance = algo.TOPPRA([pc_vel, pc_acc], path, gridpoints=gridpoints,
                       solver_wrapper='seidel', parametrizer="ParametrizeConstAccel")
traj = instance.compute_trajectory(0, 0)
duration = traj.duration
print("Found optimal trajectory with duration {:f} sec".format(duration))

################################################################################
# 采样结果：q = [s, theta]，qd = [v, omega]，qdd = [a, alpha]
ts = np.linspace(0.0, duration, 4000)
q, qd, qdd = traj.eval(ts), traj.evald(ts), traj.evaldd(ts)
s_t, theta_t = q[:, 0], q[:, 1]
v_t, w_t = qd[:, 0], qd[:, 1]
a_t, alpha_t = qdd[:, 0], qdd[:, 1]

# 两条速度约束各自给出的线速度包络（画图用）
env_w = W_MAX / np.maximum(np.abs(kappa), 1e-9)
env = np.minimum(np.full_like(s_grid, V_MAX), env_w)

print("上限     : v<={:.2f}  omega<={:.2f}  a<={:.2f}  alpha<={:.2f}"
      .format(V_MAX, W_MAX, A_MAX, ALPHA_MAX))
print("实际峰值 : v={:.3f}  omega={:.3f}  a={:.3f}  alpha={:.3f}"
      .format(np.abs(v_t).max(), np.abs(w_t).max(),
              np.abs(a_t).max(), np.abs(alpha_t).max()))
in_curve = np.abs(np.interp(s_t, s_grid, kappa)) > 1e-6
print("拐角     : 合成 v 包络最低 {:.3f} m/s，曲线段实际最低 {:.3f} m/s"
      .format(env.min(), v_t[in_curve].min()))
ds_grid = (path.path_interval[1] - path.path_interval[0]) / len(s_grid)
print("绑定约束 : v_max {:.2f}m / omega_max {:.2f}m"
      .format(np.sum(env >= V_MAX - 1e-9) * ds_grid,
              np.sum(env < V_MAX - 1e-9) * ds_grid))

################################################################################
# 对照组：复刻 path_follower 现有流水线（Ruckig 1D -> 曲率软钳 -> 加减速包络）
################################################################################
def ruckig_profile(dist, v0, a0, v_max, v_target, a_max, d_max, jerk, dt):
    """1-DOF Ruckig：把弧长本身当 position 规划，再按 dist 网格插值回填 v/a。

    与 path_follower/src/ruckig_velocity_planner.cpp 的做法一致。
    """
    otg = ruckig.Ruckig(1, dt)
    inp = ruckig.InputParameter(1)
    out = ruckig.OutputParameter(1)
    inp.current_position = [0.0]
    inp.current_velocity = [v0]
    inp.current_acceleration = [a0]
    inp.target_position = [float(dist[-1] - dist[0])]
    inp.target_velocity = [v_target]
    inp.target_acceleration = [0.0]
    inp.max_velocity = [v_max]
    inp.max_acceleration = [a_max]
    inp.min_acceleration = [-d_max]
    inp.max_jerk = [jerk]

    ss, vs, accs = [0.0], [v0], [a0]
    res = ruckig.Result.Working
    guard = 0
    while res == ruckig.Result.Working and guard < 200000:
        res = otg.update(inp, out)
        ss.append(out.new_position[0])
        vs.append(out.new_velocity[0])
        accs.append(out.new_acceleration[0])
        out.pass_to_input(inp)
        guard += 1
    ss = np.array(ss)
    keep = np.concatenate([[True], np.diff(ss) > 1e-12])   # np.interp 要求 x 严格递增
    rel = dist - dist[0]
    return (np.interp(rel, ss[keep], np.array(vs)[keep]),
            np.interp(rel, ss[keep], np.array(accs)[keep]))


def curvature_clamp(v, kappa):
    """velocity_planner.cpp::applyCurvatureDeceleration"""
    w = v * kappa
    ratio = np.maximum(CURV_DEC_RATIO_MIN, CURV_DEC_GAIN / (w * w + CURV_DEC_GAIN))
    return np.maximum(v * ratio, MIN_V)


def accel_envelope(v, s, a_max, d_max):
    """velocity_planner.cpp::applyAccelerationEnvelope：U = v^2 上的前后向双扫描"""
    u = v ** 2
    ds = np.diff(s)
    for i in range(len(u) - 1, 0, -1):          # 反向：保证刹得住
        u[i - 1] = min(u[i - 1], u[i] + 2.0 * d_max * ds[i - 1])
    for i in range(1, len(u)):                  # 正向：保证加得起
        u[i] = min(u[i], u[i - 1] + 2.0 * a_max * ds[i - 1])
    return np.sqrt(np.maximum(u, 0.0))


def profile_duration(v, s):
    """由 v(s) 反算总耗时 T = ∫ ds / v（梯形积分，1/v 用两端平均）。

    速度统一下钳到 MIN_V：TOPPRA 的首末点速度是严格的 0，积分会发散；
    Ruckig 那条本来就是从 MIN_V 起步的，钳到同一个下限才可比。
    """
    vv = np.maximum(v, MIN_V)
    return float(np.sum(np.diff(s) * 0.5 * (1.0 / vv[:-1] + 1.0 / vv[1:])))


# path_follower 看到的是 5mm 弧长网格；曲率直接取解析值（隔离"曲率来源"这个变量）
s_pf = xy_s.copy()
kappa_pf = path(s_pf, 1)[:, 1]

v_ruckig, _ = ruckig_profile(s_pf, MIN_V, 0.0, V_MAX, MIN_V, A_MAX, A_MAX, JERK_MAX, CTRL_DT)
v_clamped = curvature_clamp(v_ruckig, kappa_pf)
v_pf = accel_envelope(v_clamped.copy(), s_pf, A_MAX, A_MAX)

# TOPPRA 的 v(s)（把 v(t) 按 s(t) 重排到同一张 5mm 网格上，便于逐点比较）
v_toppra_on_s = np.interp(s_pf, s_t, v_t)

print("\n—— 线速度规划对照（同一条几何、同一组限幅）——")
print("%-34s %-9s %-11s %-11s %-11s"
      % ("方案", "T [s]", "max|omega|", "max|alpha|", "过弯最低 v"))
in_curve_pf = np.abs(kappa_pf) > 1e-6
kappa_d_pf = path(s_pf, 2)[:, 1]
for name, vv in (("Ruckig 1D（不看曲率）", v_ruckig),
                 ("  + 曲率软钳", v_clamped),
                 ("  + 加减速包络 = path_follower 现状", v_pf),
                 ("TOPPRA", v_toppra_on_s)):
    w = np.abs(vv * kappa_pf).max()
    # alpha = kappa * a + kappa' * v^2，a 由 U=v^2 的差分给出
    acc = np.gradient(vv ** 2, s_pf) * 0.5
    al = np.abs(kappa_pf * acc + kappa_d_pf * vv ** 2).max()
    flag = ""
    if w > W_MAX * 1.005:
        flag += " <-- 超 omega_max"
    if al > ALPHA_MAX * 1.05:
        flag += " <-- 超 alpha_max"
    print("%-34s %-9.3f %-11s %-11s %-11.3f%s"
          % (name, profile_duration(vv, s_pf), "%.3f" % w, "%.3f" % al,
             vv[in_curve_pf].min(), flag))
print("TOPPRA 相对 path_follower 现状：耗时 %.1f%%，过弯速度 %.2f 倍"
      % (100.0 * profile_duration(v_toppra_on_s, s_pf) / profile_duration(v_pf, s_pf),
         v_toppra_on_s[in_curve_pf].min() / v_pf[in_curve_pf].min()))

################################################################################
# We can now visualize the result
fig = plt.figure(figsize=(12.5, 8.0))
gs = fig.add_gridspec(3, 2, width_ratios=[1.1, 1.0], height_ratios=[1.5, 1.0, 1.0],
                      hspace=.5, wspace=.26)

# 左上：xy 平面里的几何路径 + 等弧长的车头朝向
ax = fig.add_subplot(gs[0, 0])
ax.plot(WAYPTS[:, 0], WAYPTS[:, 1], '--', color='0.55', lw=1.2, label='原折线')
ax.plot(positions[:, 0], positions[:, 1], '-', lw=2.2, color='C0', label='倒角后路径')
ax.plot([cp[0, 0], cp[-1, 0]], [cp[0, 1], cp[-1, 1]], 's', color='C3', ms=7,
        label='接入/接出点 A,B')
s_mark = np.linspace(0.0, path.path_interval[1], 22)
xy_mark = np.column_stack([np.interp(s_mark, xy_s, positions[:, 0]),
                           np.interp(s_mark, xy_s, positions[:, 1])])
th_mark = np.interp(s_mark, s_t, theta_t)
ax.quiver(xy_mark[:, 0], xy_mark[:, 1], np.cos(th_mark), np.sin(th_mark),
          angles='xy', scale_units='xy', scale=5.5, width=.006,
          color='C1', label='车头朝向 (等弧长)')
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_title('line - bezier(deg {}) - line'.format(BEZIER_DEGREE))
ax.set_aspect('equal')
ax.margins(.14)
ax.grid(alpha=.3)
ax.legend(loc='upper left', fontsize=8)

# 左中：曲率 + 速度包络 + 实际 v(s)
ax = fig.add_subplot(gs[1, 0])
ax.plot(s_grid, kappa, color='0.35', lw=1, label='kappa')
ax.set_xlabel('s [m]')
ax.set_ylabel('kappa [1/m]')
ax.set_ylim(0, np.abs(kappa).max() * 1.9)
ax.grid(alpha=.3)
ax2 = ax.twinx()
ax2.plot(s_grid, env_w, ':', color='C2', lw=1.1, label='omega_max/|kappa|')
ax2.plot(s_grid, env, '-', color='C3', lw=1.6, label='合成包络')
ax2.plot(s_t, v_t, '-', color='k', lw=1.3, label='TOPPRA v(s)')
ax2.set_ylim(0, V_MAX * 1.55)
ax2.set_ylabel('v [m/s]')
h1, l1 = ax.get_legend_handles_labels()
h2, l2 = ax2.get_legend_handles_labels()
ax.legend(h1 + h2, l1 + l2, fontsize=7, ncol=2, loc='upper center',
          framealpha=.9, borderpad=.3, columnspacing=1.0)
ax.set_title('curvature & velocity envelope', fontsize=9, pad=2)

# 右侧：v/omega 与 a/alpha
ax_v = fig.add_subplot(gs[0, 1])
ax_v.plot(ts, v_t, color='C0', label='v [m/s]')
ax_v.plot(ts, w_t, color='C1', label='omega [rad/s]')
ax_v.axhline(V_MAX, color='C0', ls=':', lw=1)
ax_v.axhline(W_MAX, color='C1', ls=':', lw=1)
ax_v.set_title('cmd_vel, T={:.3f}s'.format(duration), fontsize=9)

ax_a = fig.add_subplot(gs[1, 1], sharex=ax_v)
ax_a.plot(ts, a_t, color='C0', label='a [m/s^2]')
ax_a.plot(ts, alpha_t, color='C1', label='alpha [rad/s^2]')
for lim, c in ((A_MAX, 'C0'), (ALPHA_MAX, 'C1')):
    ax_a.axhline(lim, color=c, ls=':', lw=1)
    ax_a.axhline(-lim, color=c, ls=':', lw=1)
ax_a.set_xlabel('t [s]')

for ax_ in (ax_v, ax_a):
    ax_.grid(alpha=.3)
    ax_.legend(ncol=2, fontsize=8, loc='lower left')

# 左下 + 右下：Ruckig 流水线 vs TOPPRA
ax_c = fig.add_subplot(gs[2, 0])
ax_c.plot(s_pf, v_ruckig, '--', color='0.55', lw=1.2, label='Ruckig 1D（不看曲率）')
ax_c.plot(s_pf, v_clamped, ':', color='C1', lw=1.4, label='+ 曲率软钳')
ax_c.plot(s_pf, v_pf, '-', color='C1', lw=2.0, label='+ 加减速包络（现状）')
ax_c.plot(s_pf, v_toppra_on_s, '-', color='C0', lw=2.0, label='TOPPRA')
ax_c.plot(s_grid, env, ':', color='C3', lw=1.2, label='ω 包络 ω_max/|κ|')
ax_c.axhline(V_MAX, color='r', ls=':', lw=1)
ax_c.set_xlabel('s [m]')
ax_c.set_ylabel('v [m/s]')
ax_c.set_ylim(0, V_MAX * 1.55)
ax_c.grid(alpha=.3)
ax_c.legend(fontsize=7, ncol=2, loc='upper center', framealpha=.92,
            borderpad=.3, columnspacing=1.0)
ax_c.set_title('linear velocity: ruckig pipeline vs toppra', fontsize=9, pad=2)

ax_o = fig.add_subplot(gs[2, 1], sharex=ax_c)
ax_o.plot(s_pf, np.abs(v_ruckig * kappa_pf), '--', color='0.55', lw=1.2, label='Ruckig 1D')
ax_o.plot(s_pf, np.abs(v_pf * kappa_pf), '-', color='C1', lw=2.0, label='现状')
ax_o.plot(s_pf, np.abs(v_toppra_on_s * kappa_pf), '-', color='C0', lw=2.0, label='TOPPRA')
ax_o.axhline(W_MAX, color='r', ls=':', lw=1)
ax_o.set_xlabel('s [m]')
ax_o.set_ylabel('|omega| [rad/s]')
ax_o.grid(alpha=.3)
ax_o.legend(fontsize=8, ncol=3, loc='upper right')
ax_o.set_title('angular velocity along path (limit {:.2f})'.format(W_MAX), fontsize=9, pad=2)

plt.show()
