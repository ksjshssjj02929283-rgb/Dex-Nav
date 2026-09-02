#include "path_follower/velocity_envelope.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace path_follower
{
    namespace
    {
        constexpr double kEpsilon = 1e-9;
        /// |kappa| 低于此值视为直线：角加速度项 alpha = kappa·sdd + kappa'·x 里
        /// kappa·sdd 这一项对 sdd 不再构成约束（否则除以 kappa 会炸）。
        constexpr double kStraightCurvature = 1e-6;
        /// 前后向扫描的最大轮数。角加速度约束会让正向扫描把某点压到"上一点用尽减速度
        /// 也刹不进来"的位置，破坏反向扫描刚建立的不变式，所以要反复扫到收敛。
        /// 每轮只会把 u 调低且有下界 x_floor，故单调收敛；实测 2~3 轮即稳定。
        constexpr int kMaxEnvelopePasses = 8;
        /// 收敛判据：某轮内 u 的最大变化量小于它就停。
        constexpr double kEnvelopeTol = 1e-10;

        /// 弧长间隔，兜底非单调输入（累加 hypot 得到的 s 理应单调不减）。
        inline double span(const std::vector<double>& s, std::size_t i)
        {
            return std::max(0.0, s[i] - s[i - 1]);
        }
    } // namespace

    std::vector<double> computeCurvatureRate(
        const std::vector<double>& s, const std::vector<double>& kappa)
    {
        const std::size_t n = kappa.size();
        std::vector<double> rate(n, 0.0);
        if (n < 3 || s.size() != n)
        {
            return rate;
        }
        // 取**两侧单边差分中绝对值更大**的那个，而不是中心差分。
        //
        // kappa' 在贝塞尔接入/接出点是跳变的（直线侧为 0，曲线侧为有限值）。中心差分
        // 会跨在阶跃上、把两侧平均掉：实测 d=0.3m、1cm 采样时，接出点前最后一个点的
        // kappa' 真值 16.34，中心差分只给出 12.92（低估 21%），而角加速度峰值恰好就
        // 落在这一点上——这是 alpha 超限最主要的来源。单边差分不跨越阶跃，取更大的
        // 一侧则把偏差压向**保守**方向：约束宁可算紧，不可算松。
        // 代价是平滑段上从 O(ds^2) 退化为 O(ds) 精度并略偏大，换来的是不会低估。
        for (std::size_t i = 1; i + 1 < n; ++i)
        {
            const double ds_b = s[i] - s[i - 1];
            const double ds_f = s[i + 1] - s[i];
            const double back = (ds_b > kEpsilon) ? (kappa[i] - kappa[i - 1]) / ds_b : 0.0;
            const double fwd = (ds_f > kEpsilon) ? (kappa[i + 1] - kappa[i]) / ds_f : 0.0;
            rate[i] = (std::fabs(back) >= std::fabs(fwd)) ? back : fwd;
        }
        rate.front() = rate[1];
        rate.back() = rate[n - 2];
        return rate;
    }

    void legacyCurvatureClamp(
        const std::vector<double>& kappa,
        std::vector<double>& v, std::vector<double>& a,
        double gain, double ratio_min, double min_v)
    {
        // gain 必须为正：它同时是分子和分母的加数，取 0 会让 ratio 恒为 0（速度全被
        // 压到 min_v 兜底值），取负则分母可能过零、ratio 发散变号。
        const double g = std::max(gain, kEpsilon);
        // ratio_min 夹在 (0, 1]：> 1 会把速度放大到 Ruckig 规划值之上，越过全局速度上限。
        const double r_min = std::clamp(ratio_min, kEpsilon, 1.0);

        for (std::size_t i = 0; i < v.size(); ++i)
        {
            const double raw_w = v[i] * kappa[i];
            const double ratio = std::max(r_min, g / (raw_w * raw_w + g));
            a[i] *= ratio; // 与被打折的速度保持数值自洽
            v[i] = std::max(v[i] * ratio, min_v);
        }
    }

    void legacyAccelEnvelope(
        const std::vector<double>& s,
        std::vector<double>& v, std::vector<double>& a,
        double a_max, double d_max)
    {
        const std::size_t n = v.size();
        if (n < 2 || s.size() != n)
        {
            return; // 少于两个点无从谈"沿弧长加减速"
        }
        const double acc = std::max(a_max, kEpsilon);
        const double dec = std::max(d_max, kEpsilon);

        std::vector<double> u(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            u[i] = v[i] * v[i];
        }

        // 反向扫描：保证从每一点都能减速到它后面的所有限速点。低谷的影响会沿路径
        // 向前传播 (v_before^2 - v_valley^2)/(2·d_max) 米，"提前减速"由此自动产生，
        // 不需要任何前视/低谷识别逻辑。
        for (std::size_t i = n - 1; i > 0; --i)
        {
            u[i - 1] = std::min(u[i - 1], u[i] + 2.0 * dec * span(s, i));
        }
        // 正向扫描：保证每一点都能从它前面的速度加速上来（含起点速度这个边界条件）。
        for (std::size_t i = 1; i < n; ++i)
        {
            u[i] = std::min(u[i], u[i - 1] + 2.0 * acc * span(s, i));
        }

        for (std::size_t i = 0; i < n; ++i)
        {
            v[i] = std::sqrt(std::max(u[i], 0.0));
        }

        // 参考加速度由 U 的前向差分直接给出：a = ½·dU/ds，与回写的 v 严格自洽。
        // 两条扫描不等式合起来正是 a ∈ [-d_max, +a_max]，clamp 只为削浮点毛刺。
        double last_a = 0.0;
        for (std::size_t i = 0; i + 1 < n; ++i)
        {
            const double ds = span(s, i + 1);
            if (ds > kEpsilon)
            {
                last_a = std::clamp((u[i + 1] - u[i]) / (2.0 * ds), -dec, acc);
            }
            a[i] = last_a;
        }
        a[n - 1] = last_a;
    }

    void toppraEnvelope(
        const std::vector<double>& s,
        const std::vector<double>& kappa,
        const std::vector<double>& kappa_rate,
        std::vector<double>& v, std::vector<double>& a,
        const EnvelopeLimits& lim)
    {
        const std::size_t n = v.size();
        if (n < 2 || s.size() != n || kappa.size() != n)
        {
            return;
        }
        const double acc = std::max(lim.a_max, kEpsilon);
        const double dec = std::max(lim.d_max, kEpsilon);
        const double w_max = std::max(lim.w_max, kEpsilon);
        const bool use_alpha = (lim.alpha_max > 0.0) && (kappa_rate.size() == n);
        const double al_max = use_alpha ? lim.alpha_max : 0.0;
        const double x_floor = lim.min_v * lim.min_v;

        // ---- 速度层：逐点上界 x = v^2 ----
        std::vector<double> u(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            double x = v[i] * v[i];                       // Ruckig 预规划速度，包络不得超过
            const double k_abs = std::fabs(kappa[i]);
            if (k_abs > kStraightCurvature)
            {
                const double v_w = w_max / k_abs;         // |omega| = |kappa|·v <= w_max
                x = std::min(x, v_w * v_w);
            }
            if (use_alpha)
            {
                const double kd = kappa_rate[i];
                if (std::fabs(kd) > kEpsilon)
                {
                    // 让 sdd 的可行区间**恒不为空**：存在 sdd ∈ [-d_max, a_max]
                    // 使 |kappa·sdd + kappa'·x| <= alpha_max。
                    // kappa·sdd 的取值范围是 [m, M]，与 [-alpha-kappa'x, alpha-kappa'x]
                    // 有交集当且仅当 m <= alpha - kappa'x 且 M >= -alpha - kappa'x。
                    // kappa≈0 时 m=M=0，退化成精确的 x <= alpha_max/|kappa'|——
                    // 贝塞尔接入点正是这种情形（曲率为 0 但曲率导数跳变），
                    // 那里 alpha = kappa'·v^2 与 sdd 无关，只能靠降速。
                    const double m = std::min(-kappa[i] * dec, kappa[i] * acc);
                    const double M = std::max(-kappa[i] * dec, kappa[i] * acc);
                    const double x_cap = (kd > 0.0) ? (al_max - m) / kd : (-al_max - M) / kd;
                    x = std::min(x, x_cap);
                }
            }
            u[i] = std::max(x, x_floor);
        }

        // ---- 加速度层：前后向扫描 ----
        //
        // 关键点：两个方向的界都从**已知**的那一端算起，未知量只出现在不等式的一侧。
        // 反向扫描求"从哪些 x 出发还刹得住"（可控集），正向求"从已知 x 能加到多快"
        // （可达集）——这正是 TOPPRA 的做法。若反过来把约束建在未知量那一侧，
        // 可行集为空时解出的上界会变成负数，并沿扫描方向一路传染，把整段压到速度下限。
        //
        // 角加速度约束 |kappa·sdd + kappa'·x| <= alpha_max 在 kappa 与 kappa' 同时变号时
        // 不变，故统一翻成 kappa >= 0 处理，省掉一半分支。记 A = kappa/(2·ds)：
        //   sdd 的下界（减速能力）：max(-d_max, (-alpha_max - kappa'·x)/kappa)
        //   sdd 的上界（加速能力）：min( a_max, ( alpha_max - kappa'·x)/kappa)
        //
        // sdd 在一个区间内是常数，但 kappa/kappa' 不是：拐角紧、采样疏时，kappa' 在相邻
        // 两点之间能变几十个百分点（实测 d=0.3m、1cm 采样下相邻点 |Δkappa'| 达 13，
        // 而 kappa' 峰值只有 46）。只用区间左端点施加约束，右半个区间就会越界——
        // 实测左端点配对刚好顶在上限 0.560，右端点配对却到了 0.731。
        // 因此每个区间用**两端各算一次、取更紧的那个**。
        auto flipped = [&](std::size_t idx, double& k, double& kd)
        {
            k = kappa[idx];
            kd = use_alpha ? kappa_rate[idx] : 0.0;
            if (k < 0.0)
            {
                k = -k;
                kd = -kd;
            }
        };

        // 反复扫到收敛：单纯"反向一遍 + 正向一遍"在只有 a_max/d_max 时就够了
        // （正向压低 u[i] 时反向不等式自动仍成立），但加入角加速度约束后不再成立——
        // 正向的 alpha 界可以把 u[i] 压到"u[i-1] 用尽 d_max 也刹不进来"的位置。
        // 实测 d=0.6m、1cm 采样下会在接入点出现 -1.29 m/s^2 的等效减速度（上限 0.4）。
        for (int pass = 0; pass < kMaxEnvelopePasses; ++pass)
        {
        double max_delta = 0.0;

        // 反向扫描：u[i-1] 的上界 = 从它出发、用尽减速能力后仍能落进 u[i] 的最大值。
        for (std::size_t i = n - 1; i > 0; --i)
        {
            const double ds = span(s, i);
            if (ds <= kEpsilon)
            {
                u[i - 1] = std::min(u[i - 1], u[i]);
                continue;
            }
            double bound = u[i] + 2.0 * dec * ds; // 线减速度 sdd >= -d_max

            if (use_alpha)
            {
                for (std::size_t idx : {i - 1, i}) // 区间两端各施加一次，取更紧者
                {
                    double k = 0.0, kd = 0.0;
                    flipped(idx, k, kd);
                    if (k <= kStraightCurvature)
                    {
                        continue;
                    }
                    // 角加速度限制的最大减速： sdd >= (-alpha_max - kappa'·x)/kappa
                    //   x + 2ds·(-alpha_max - kappa'·x)/kappa <= u[i]
                    //   x·(1 - kappa'/A) <= u[i] + alpha_max/A
                    const double A = k / (2.0 * ds);
                    const double G = 1.0 - kd / A;
                    if (G > kEpsilon)
                    {
                        bound = std::min(bound, (u[i] + al_max / A) / G);
                    }
                    // G <= 0 时该式对 x 不构成上界（曲率导数已经在替减速出力），不处理。
                }
            }
            const double next = std::min(u[i - 1], std::max(bound, x_floor));
            max_delta = std::max(max_delta, u[i - 1] - next);
            u[i - 1] = next;
        }

        // 正向扫描：u[i] 的上界 = 从 u[i-1] 出发、用尽加速能力后能到达的最大值。
        for (std::size_t i = 1; i < n; ++i)
        {
            const double ds = span(s, i);
            if (ds <= kEpsilon)
            {
                u[i] = std::min(u[i], u[i - 1]);
                continue;
            }
            double bound = u[i - 1] + 2.0 * acc * ds; // 线加速度 sdd <= a_max

            if (use_alpha)
            {
                for (std::size_t idx : {i - 1, i}) // 区间两端各施加一次，取更紧者
                {
                    double k = 0.0, kd = 0.0;
                    flipped(idx, k, kd);
                    if (k <= kStraightCurvature)
                    {
                        continue;
                    }
                    // 角加速度限制的最大加速： sdd <= (alpha_max - kappa'·x)/kappa
                    //   x_next <= x + 2ds·(alpha_max - kappa'·x)/kappa
                    //           = x·(1 - kappa'/A) + alpha_max/A
                    const double A = k / (2.0 * ds);
                    bound = std::min(bound, u[i - 1] * (1.0 - kd / A) + al_max / A);
                }
            }
            const double next = std::min(u[i], std::max(bound, x_floor));
            max_delta = std::max(max_delta, u[i] - next);
            u[i] = next;
        }

        if (max_delta <= kEnvelopeTol)
        {
            break;
        }
        } // pass

        // 速度层已把每点兜底到 min_v^2，两遍扫描都是 min 操作，理论上不会掉到它以下；
        // 但角加速度那几条上界是从不等式解出来的，浮点上可能给出略小甚至负的值，
        // 这里统一兜一次底，避免把参考速度压进底盘死区。
        for (std::size_t i = 0; i < n; ++i)
        {
            u[i] = std::max(u[i], x_floor);
            v[i] = std::sqrt(u[i]);
        }

        double last_a = 0.0;
        for (std::size_t i = 0; i + 1 < n; ++i)
        {
            const double ds = span(s, i + 1);
            if (ds > kEpsilon)
            {
                last_a = std::clamp((u[i + 1] - u[i]) / (2.0 * ds), -dec, acc);
            }
            a[i] = last_a;
        }
        a[n - 1] = last_a;
    }
} // namespace path_follower
