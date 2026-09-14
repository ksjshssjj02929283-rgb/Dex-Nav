// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 可行性结果：把「物理做不到」与「求解器没算出来」分开表达。
#ifndef NAV2_SPEED_PLANNER__CORE__FEASIBILITY_HPP_
#define NAV2_SPEED_PLANNER__CORE__FEASIBILITY_HPP_

namespace nav2_speed_planner
{

/**
 * @enum ConstraintFailurePolicy
 * @brief 约束**无法评估**时的处置策略（调研 P0-3）。
 *
 * 注意区分"无法评估"与"评估结果为无约束"：
 *   - 无法评估 = TF 变换失败 / costmap 缺失 → 我们**不知道**前方有没有障碍
 *   - 无约束   = 查过了，确实没有障碍
 * 此前两者都以 `return false` 表达，调用方无法区分，等价于隐式 FAIL_OPEN——
 * 即 TF 一失败，机器人就在"没有任何障碍约束"的状态下继续按巡航速度行驶。
 */
/// 速度规划的可行性结果。Phase A（包络）与 Phase B（剖面后端）共用。
///
/// 三个维度**刻意分开**，因为受众不同：
///
/// - `Kind`：谁该响应。`kPhysical` = 机器人在动力学限制下真的做不到（消费侧
///   须响应：降级、告警、上报）；`kNumerical` = 求解器/数值出问题（开发者该
///   看，消费侧按保守处理即可）。二者混在一个 bool 或一个扁平枚举里，消费侧
///   就无法区分"物理停不住"和"我们的求解器今天不太行"——这两者应触发完全
///   不同的响应。
/// - `Reason`：具体哪一步失败，供开发者定位。
/// - `Provenance`：结果**是怎么来的**（正常求解 / 走了制动兜底）。对应 Apollo
///   「记录 fallback/relax 是否发生，避免把 fallback 轨迹当正常优化结果分析」。
///
/// ⚠️ 当前它只是**通报**，没有任何消费方据此改变行为。Apollo 的做法是
///   「约束无解 → 进入 fallback task，**而不是发布一条未约束轨迹**」，并有
///   三级降级（初值加速度归零 → 放宽边界重试 → 上层 fallback）。把本结构接成
///   契约是重建步 4 的内容，见项目文档 8 §三。
struct FeasibilityResult
{
  enum class Kind
  {
    kFeasible = 0,
    kPhysical,    ///< 动力学限制下够不着（消费侧须响应）
    kNumerical,   ///< 求解器 / 数值失败（开发者定位；消费侧保守处理）
  };

  enum class Reason
  {
    kNone = 0,
    kStableSegmentUnreachable,   ///< 稳定末段生成失败                  [Physical]
    kFallbackUnreachable,        ///< 制动兜底后仍未达稳定目标          [Physical]
    kDurationBudgetInvalid,      ///< duration budget 非有限或 <= 0     [Numerical]
    kSolverError,                ///< 求解器返回非 Working/Finished     [Numerical]
    kNonFiniteOutput,            ///< 求解器输出非有限                  [Numerical]
    kIterationExhausted,         ///< 段未在步数预算内完成              [Numerical]
  };

  enum class Provenance
  {
    kNormal = 0,        ///< 正常求解，未降级
    kBrakingFallback,   ///< 走了制动兜底路径
  };

  Kind kind{Kind::kFeasible};
  Reason reason{Reason::kNone};
  Provenance provenance{Provenance::kNormal};
  /// **进入**兜底路径时的原始原因。`resolve()` 会覆写 `reason`（那是最终判定），
  /// 若不单独保留，"当初为什么进兜底"就丢了——而这恰恰是区分
  /// "求解器解不出" 与 "物理上就做不到" 的唯一依据。
  Reason entry_reason{Reason::kNone};
  /// 求解器原始返回码与当时的输入快照——用于区分"输入不一致"与"真的无解"。
  int solver_code{0};
  double solver_v0{0.0}, solver_a0{0.0}, solver_vmax{0.0};
  double solver_target_v{0.0}, solver_distance{0.0};

  bool infeasible() const {return kind != Kind::kFeasible;}

  static Kind kindOf(Reason r)
  {
    switch (r) {
      case Reason::kStableSegmentUnreachable:
      case Reason::kFallbackUnreachable:
        return Kind::kPhysical;
      case Reason::kDurationBudgetInvalid:
      case Reason::kSolverError:
      case Reason::kNonFiniteOutput:
      case Reason::kIterationExhausted:
        return Kind::kNumerical;
      case Reason::kNone:
        break;
    }
    return Kind::kFeasible;
  }

  /// 赋值语义 —— 归一前的 5 处「进入」站点（各自随后 break）
  void mark(Reason r, bool cond)
  {
    kind = cond ? kindOf(r) : Kind::kFeasible;
    reason = cond ? r : Reason::kNone;
    if (entry_reason == Reason::kNone) {entry_reason = r;}   // 首因，不被覆写
  }
  /// 或入语义 —— 归一前 L762「段未完成」
  void markOr(Reason r, bool cond)
  {
    const bool next = infeasible() || cond;
    if (!next) {kind = Kind::kFeasible; reason = Reason::kNone; return;}
    if (reason == Reason::kNone) {reason = r; kind = kindOf(r);}
    if (entry_reason == Reason::kNone) {entry_reason = r;}
  }
  /// 赋值语义 —— 归一前 L807，**当前唯一能把不可行清回可行的路径**。
  /// 它承担着退出判据的职责，但语义与"制动兜底成没成功"耦合，没有独立的
  /// 「已回到可行集」判断（重建步 4 补）。
  void resolve(bool cond, Reason r)
  {
    kind = cond ? kindOf(r) : Kind::kFeasible;
    reason = cond ? r : Reason::kNone;
  }
};

}  // namespace nav2_speed_planner

#endif  // NAV2_SPEED_PLANNER__CORE__FEASIBILITY_HPP_
