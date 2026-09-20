#ifndef PATH_FOLLOWER__VELOCITY_ENVELOPE_HPP_
#define PATH_FOLLOWER__VELOCITY_ENVELOPE_HPP_

#include <vector>

namespace path_follower
{
    /**
     * @file
     * @brief 沿弧长的速度包络算法（纯数组进出，不依赖 Route / rclcpp / 节点状态）。
     *
     * 这里放两套互不干扰的实现：
     *   legacy*  —— 历史的"曲率软钳 + 加减速双扫描"，逐字保持原行为，便于回滚与对拍；
     *   toppra*  —— 物理包络 + 含角加速度约束的前后向扫描。
     *
     * 两套都只回答同一个问题：**每个弧长位置上线速度最大能是多少**，
     * 上游 Ruckig 给出的速度剖面作为逐点上限传进来，包络只会往下压、不会超过它。
     */

    /// 曲率导数 kappa' = dkappa/ds（中心差分，端点用单侧）。
    ///
    /// 只在 kappa 本身可信时才有意义：kappa 若来自 x,y 三点外接圆，再差分一次
    /// 噪声会被继续放大 1/ds，结果不可用（1cm 点距下放大 1e2）。调用方需保证
    /// kappa 是由解析切向差分而来。
    ///
    /// @param s      单调不减的弧长序列
    /// @param kappa  逐点曲率
    /// @return       与输入等长的 kappa'；输入少于 3 点时返回全 0
    std::vector<double> computeCurvatureRate(
        const std::vector<double>& s, const std::vector<double>& kappa);

    /// legacy 曲率软钳：ratio = max(ratio_min, gain / (w^2 + gain))，w = v * kappa。
    ///
    /// 这是一条启发式打折曲线，不是物理约束：小曲率处角速度峰值被压到 sqrt(gain)/2
    /// （远低于 w_max），而 w > sqrt(gain/ratio_min - gain) 之后 ratio 触底、
    /// 打折退化成固定比例，角速度反而随曲率线性回升并可能越过 w_max。
    /// ratio_min 与 min_v 兜底存在的根本原因是三点外接圆算出的曲率噪声太大。
    ///
    /// @param kappa  逐点曲率
    /// @param v      原地修改：入参为 Ruckig 剖面，出参为打折后的速度
    /// @param a      原地修改：与 v 同比例缩放，保持数值自洽
    /// @param gain, ratio_min, min_v  软钳参数与速度下限
    void legacyCurvatureClamp(
        const std::vector<double>& kappa,
        std::vector<double>& v, std::vector<double>& a,
        double gain, double ratio_min, double min_v);

    /// legacy 加减速包络：U = v^2 空间的前后向双扫描。
    ///
    /// a = v·dv/ds = ½·dU/ds，于是加减速约束在 U 空间是纯斜率约束：
    ///   反向扫描（保证刹得住）：U[i]   <= U[i+1] + 2·d_max·ds
    ///   正向扫描（保证加得起）：U[i]   <= U[i-1] + 2·a_max·ds
    /// 先反后正一遍即为最终解，无需迭代（正向只会继续调低 U，不会破坏反向约束）。
    ///
    /// @param s      弧长
    /// @param v      原地修改：入参为逐点限速，出参为可达剖面
    /// @param a      输出：由 U 的差分给出，天然落在 [-d_max, a_max]
    void legacyAccelEnvelope(
        const std::vector<double>& s,
        std::vector<double>& v, std::vector<double>& a,
        double a_max, double d_max);

    /// toppraEnvelope 的限幅集合。
    struct EnvelopeLimits
    {
        double w_max{1.0};      ///< 角速度上限 rad/s
        double a_max{0.5};      ///< 线加速度上限 m/s^2（正值）
        double d_max{0.5};      ///< 线减速度上限 m/s^2（正值，内部取负）
        double alpha_max{0.7};  ///< 角加速度上限 rad/s^2；<=0 表示不施加该约束
        double min_v{0.02};     ///< 速度下限（底盘死区），包络不会压到它以下
    };

    /**
     * @brief TOPPRA 物理包络：把 (v, omega, a, alpha) 四条约束折叠成一条 v(s)。
     *
     * 位形取 q(s) = [s, theta(s)]，弧长参数化下 q'(s) = [1, kappa]、q''(s) = [0, kappa']，
     * 于是 v = sd、omega = kappa·sd、a = sdd、alpha = kappa·sdd + kappa'·sd^2。
     * 所有约束都落到同一个标量 x = sd^2 上，自然取交集：
     *
     *   速度层（逐点上界，与 sdd 无关）
     *     x <= v_in(s)^2                      上游 Ruckig 的预规划速度，包络不得超过它
     *     x <= (w_max / |kappa|)^2            曲率越大越慢
     *     x <= alpha_max / |kappa'|           保证 sdd=0 时角加速度也不越界。
     *                                         在 kappa≈0 而 kappa'≠0 的地方（贝塞尔接入点）
     *                                         这一条是精确的：此时 alpha = kappa'·v^2，
     *                                         与 sdd 无关，只能靠降速。其余位置略保守
     *                                         （放弃了用 kappa·sdd 抵消 kappa'·x 的可能），
     *                                         换来的是可行域**恒不为空**（sdd=0 始终可选），
     *                                         控制器不会出现"无解"分支。
     *
     *   加速度层（前后向扫描，sdd 的可行区间随 x 变化）
     *     sdd ∈ [-d_max, a_max] ∩ [(-alpha_max - kappa'·x)/kappa, (alpha_max - kappa'·x)/kappa]
     *     把 sdd = (x[i+1] - x[i]) / (2·ds) 代入，每一步化成关于待求 x 的一元线性不等式，
     *     取交集后求上界。闭式，不需要 LP。kappa≈0（直线段）时角加速度项对 sdd 无约束，
     *     退化回 legacyAccelEnvelope 的常数斜率。
     *
     * @param s            弧长（单调不减）
     * @param kappa        逐点曲率
     * @param kappa_rate   逐点曲率导数；**留空表示不施加角加速度约束**
     *                     （kappa 来自三点外接圆时必须留空，否则是在噪声上做优化）
     * @param v            原地修改：入参为 Ruckig 预规划速度剖面（逐点上限），
     *                     出参为包络后的可达速度剖面
     * @param a            输出：由 U 的差分给出的参考加速度
     * @param lim          限幅集合
     */
    void toppraEnvelope(
        const std::vector<double>& s,
        const std::vector<double>& kappa,
        const std::vector<double>& kappa_rate,
        std::vector<double>& v, std::vector<double>& a,
        const EnvelopeLimits& lim);
} // namespace path_follower

#endif  // PATH_FOLLOWER__VELOCITY_ENVELOPE_HPP_
