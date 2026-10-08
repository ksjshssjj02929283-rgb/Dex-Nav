"""
二次/三次/四次/五次贝塞尔曲线在直角拐弯处的曲率对比
==================================================
锚点方案（按用户指定）：
  二次: 中间锚点 = 直角点 P1
  三次: 中间两锚点 = 两条直角边各自的中点
  四次: 中间锚点 = 直角点 P1；另两个中间锚点 = 两条直角边各自的中点
  五次: 中间四锚点 = 两条直角边各自的三等分点 (1/3, 2/3 处)

依赖：pip install numpy matplotlib scipy
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.special import comb


# ---------------------------------------------------------------------------
# 1. 通用贝塞尔曲线工具：位置、各阶导数（对参数 u，非弧长）
# ---------------------------------------------------------------------------
def bezier_point(control_points: np.ndarray, u) -> np.ndarray:
    """control_points: (n+1, dim)；u: 标量或数组 -> (dim,) 或 (m, dim)"""
    n = len(control_points) - 1
    u = np.atleast_1d(u)[:, None]
    k = np.arange(n + 1)
    basis = comb(n, k) * (u ** k) * (1 - u) ** (n - k)
    return basis @ control_points


def bezier_derivative(control_points: np.ndarray, order: int = 1) -> np.ndarray:
    """前向差分逐阶降阶，得到 order 阶导数对应的新控制点 (n-order+1, dim)
       导数曲线本身仍是贝塞尔曲线，其控制点 = n!/(n-order)! 倍的前向差分
    """
    n = len(control_points) - 1
    cp = control_points.copy()
    coeff = 1.0
    for r in range(order):
        cp = np.diff(cp, axis=0)
        coeff *= (n - r)
    return coeff * cp


# ---------------------------------------------------------------------------
# 2. 曲率剖面计算（沿弧长重新参数化后输出 s(u), kappa(u)）
# ---------------------------------------------------------------------------
def curvature_profile(control_points: np.ndarray, num_samples: int = 500):
    """
    返回:
        s: (N,) 沿曲线的累积弧长（数值积分近似）
        kappa: (N,) 对应弧长处的曲率
    通用曲率公式（任意维度）:
        kappa = sqrt(||r'||^2 ||r''||^2 - (r'.r'')^2) / ||r'||^3
    2D 情形下根号内等价于 |x'y''-y'x''|^2，这里直接用通用公式实现，
    方便同一套代码未来扩展到 n 维关节空间。
    """
    control_points = np.asarray(control_points, dtype=float)
    d1_cp = bezier_derivative(control_points, order=1)
    d2_cp = bezier_derivative(control_points, order=2)

    u = np.linspace(0, 1, num_samples)
    r1 = bezier_point(d1_cp, u)   # (N, dim)
    r2 = bezier_point(d2_cp, u)   # (N, dim)

    speed = np.linalg.norm(r1, axis=1)
    r1r1 = np.sum(r1 * r1, axis=1)
    r2r2 = np.sum(r2 * r2, axis=1)
    r1r2 = np.sum(r1 * r2, axis=1)

    numerator = np.sqrt(np.maximum(r1r1 * r2r2 - r1r2 ** 2, 0.0))
    kappa = numerator / np.maximum(speed ** 3, 1e-14)

    # 数值积分弧长 (梯形法)
    s = np.concatenate([[0.0],
                        np.cumsum((speed[1:] + speed[:-1]) / 2 * np.diff(u))])
    return s, kappa


# ---------------------------------------------------------------------------
# 3. 按指定方案构造四条曲线的控制点
# ---------------------------------------------------------------------------
def build_control_points(A: np.ndarray, P1: np.ndarray, B: np.ndarray) -> dict:
    """
    A, P1, B: 直角拐弯的三个几何点 (入边起点、拐角、出边终点)
    返回 {degree: control_points_array}
    """
    curves = {}

    # 二次: 中间锚点 = 拐角本身
    curves[2] = np.array([A, P1, B])

    # 三次: 中间两锚点 = 两条边的中点
    curves[3] = np.array([A, (A + P1) / 2, (P1 + B) / 2, B])

    # 四次: 中间锚点 = 拐角; 另两个中间锚点 = 两边中点
    # 注意: 中间锚点恰好是两条切线的交点(=P1)，
    #       这让 [A,mid1,P1] 与 [P1,mid2,B] 共享同一个点，
    #       使得两端同时满足三点共线 -> 两端曲率均可为零(特例构造)
    curves[4] = np.array([A, (A + P1) / 2, P1, (P1 + B) / 2, B])

    # 五次: 中间四锚点 = 两条边各自的三等分点 (1/3, 2/3 处)
    curves[5] = np.array([
        A,
        A + (P1 - A) / 3,
        A + 2 * (P1 - A) / 3,
        P1 + (B - P1) / 3,
        P1 + 2 * (B - P1) / 3,
        B,
    ])
    return curves


# ---------------------------------------------------------------------------
# 4. 主流程：计算 + 打印 + 绘图
# ---------------------------------------------------------------------------
def main():
    # 直角拐弯几何: A -> P1 -> B, 入边沿 x 轴, 出边沿 y 轴
    P1 = np.array([4.0, 0.0])
    A = np.array([3.2, 0.0])    # 入边起点 (距拐角 0.8)
    B = np.array([4.0, 0.8])    # 出边终点 (距拐角 0.8)

    curves = build_control_points(A, P1, B)

    results = {}
    print(f"{'次数':<6}{'kappa(0)':<12}{'kappa(1)':<12}{'峰值kappa':<12}{'弧长':<10}")
    print("-" * 52)
    for degree, cp in curves.items():
        s, kappa = curvature_profile(cp)
        results[degree] = (s, kappa, cp)
        print(f"{degree:<6}{kappa[0]:<12.4f}{kappa[-1]:<12.4f}"
              f"{kappa.max():<12.4f}{s[-1]:<10.4f}")

    # ---- 绘图 ----
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.5))

    colors = {2: "tab:orange", 3: "tab:brown", 4: "tab:red", 5: "tab:blue"}
    labels = {
        2: "2次: 中锚=拐角点",
        3: "3次: 中锚=两边中点",
        4: "4次: 中锚=拐角点+两边中点",
        5: "5次: 四锚点均匀分布于三等分点",
    }

    # 左图: 曲线形状对比
    ax1.plot([A[0], P1[0], B[0]], [A[1], P1[1], B[1]],
             "--", color="0.6", lw=1.2, label="原始折线 A-P1-B")
    for degree, (s, kappa, cp) in results.items():
        u = np.linspace(0, 1, 200)
        pts = bezier_point(cp, u)
        ax1.plot(pts[:, 0], pts[:, 1], color=colors[degree],
                  lw=2, label=labels[degree])
        ax1.plot(cp[:, 0], cp[:, 1], "o", color=colors[degree],
                  ms=4, alpha=0.5)
    ax1.set_xlabel("x")
    ax1.set_ylabel("y")
    ax1.set_title("曲线形状对比")
    ax1.legend(fontsize=8, loc="upper left")
    ax1.grid(alpha=0.3)
    ax1.axis("equal")

    # 右图: 曲率 vs 弧长
    for degree, (s, kappa, cp) in results.items():
        ax2.plot(s, kappa, color=colors[degree], lw=2, label=labels[degree])
    ax2.set_xlabel("沿曲线弧长 s")
    ax2.set_ylabel("曲率 kappa (1/m)")
    ax2.set_title("曲率剖面对比")
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig("bezier_curvature_comparison.png", dpi=130)
    print("\n图片已保存: bezier_curvature_comparison.png")
    plt.show()


if __name__ == "__main__":
    main()