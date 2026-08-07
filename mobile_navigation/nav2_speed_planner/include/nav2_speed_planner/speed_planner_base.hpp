// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 速度规划器抽象基类（nav2_core::SpeedPlanner 实现）。
//
// 架构（两阶段，base 只负责 Phase A，Phase B 完全交给派生类）：
//   Phase A 约束包络（base，逐点 min 叠加，安全置零绝对优先）：
//     A1 applyObstacleConstraint（障碍 + 缓冲带置零）
//     A2 computeCruiseLimit + setPlanSpeedLimit（理想巡航求解 + 回填）
//     A3 applyCurvatureConstraint（曲率限速）
//     A4 applyEndSpeedConstraint（末端蠕行区：距终点 stop_distance 内压到 end_speed）
//   Phase B 运动学可行化（派生类纯虚 assignVelocityProfile）：
//     T 型 = 恒加速度前向/后向剖面；S 型 = Ruckig jerk 限制剖面。
//
// base 不含任何具体速度剖面算法；computeSpeedPlan 为模板方法：
//   path → 工作结构（仅合并几何与朝向均等价的冗余点）→ Phase A → hook
//   → 组装 RobotTrajectory + 可观测日志。
//
// 减速语义为 receding-horizon：派生类从**当前实际速度**出发规划，
// 前提是高频重规划（每控制周期重算，减速区随速度增长逐周期外扩）。
#ifndef NAV2_SPEED_PLANNER__SPEED_PLANNER_BASE_HPP_
#define NAV2_SPEED_PLANNER__SPEED_PLANNER_BASE_HPP_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_interfaces/msg/robot_trajectory.hpp"

#include "nav2_core/speed_planner.hpp"

namespace nav2_speed_planner
{

/**
 * @struct TrajPointWork
 * @brief 内部工作结构：{pose, speed_limit, curvature} + S 型扩展字段 + 语义透传字段。
 */
struct TrajPointWork
{
  geometry_msgs::msg::PoseStamped pose;
  double speed_limit{0.0};
  double curvature{0.0};

  // —— 时空剖面扩展字段（恒加速度剖面默认 0，jerk 限制剖面回填）——
  double accel{0.0};
  double rel_time{0.0};

  // —— 透传字段（来自带语义的 path；nav_msgs/Path 无此字段时置默认）——
  float speed_hint{0.0f};
  std::string region_type;
  std::string segment_id;
};

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
enum class ConstraintFailurePolicy
{
  /// 本周期速度规划失败（输出空轨迹），由 ControllerServer 发布零速。生产默认。
  FAIL_CLOSED,
  /// 跳过该约束继续规划。仅允许仿真/明确灰度，且必须有 diagnostic。
  FAIL_OPEN
};

/**
 * @enum PointObstacleStatus
 * @brief 单点障碍判定的三态。
 *
 * ⚠️ `OUT_OF_RANGE` 不是失败：local costmap 是 ±4m 的 rolling window，
 * 路径上超出覆盖范围的点属正常情况（footprint 半对角 0.495m → 约 3.5m 之后即出图）。
 * 若把它当作"不可评估"并触发 FAIL_CLOSED，任何长于约 3.5m 的路径都会让机器人原地不动。
 * 它只计数、只上报，不参与策略判定——但计数本身很重要：R47 的幻影障碍
 * （出图被 nav2 判成 LETHAL）当初若有这个计数，一眼就能看出"3.5m 之后全是出图"。
 */
enum class PointObstacleStatus
{
  FREE,          ///< 查过，无障碍
  OBSTACLE,      ///< 查过，有障碍
  OUT_OF_RANGE   ///< 超出 costmap 覆盖，无信息（正常，不判障、不触发策略）
};

/**
 * @struct ObstacleConstraintOutcome
 * @brief A1 障碍约束的结果——把"有没有障碍"与"能不能评估"分开表达。
 */
struct ObstacleConstraintOutcome
{
  bool has_obstacle{false};        ///< 是否检测到障碍（仅在 evaluable 时有意义）
  bool evaluable{true};            ///< false = 本周期无法评估（TF/costmap 不可用）
  std::string unavailable_reason;  ///< evaluable=false 时的机器可读原因
  size_t scanned_points{0};        ///< 实际完成查询的点数
  size_t out_of_range_points{0};   ///< 超出 costmap 覆盖的点数（观测用）
};

/**
 * @class SpeedPlannerBase
 * @brief 速度规划抽象基类：实现 nav2_core::SpeedPlanner 全部接口与 Phase A 约束包络；
 *        Phase B（速度剖面生成）为纯虚 hook，由派生类实现。
 */
class SpeedPlannerBase : public nav2_core::SpeedPlanner
{
public:
  SpeedPlannerBase() = default;
  ~SpeedPlannerBase() override = default;

  // —— nav2_core::SpeedPlanner 接口实现（签名不得修改）——
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief 模板方法：path + 当前速度 → 带速度的时空轨迹。
   *        固定流程：构建工作结构（相邻重复点去重）→ 初始限速取严（外部限速/速度提示）→
   *        Phase A 约束包络（A1 障碍 → A2 巡航 → A3 曲率 → A4 末端蠕行）→
   *        Phase B hook assignVelocityProfile → 组装 RobotTrajectory + 剖面可观测日志。
   */
  nav_interfaces::msg::RobotTrajectory computeSpeedPlan(
    const nav_msgs::msg::Path & path,
    const nav2_core::SpeedPlanningBoundary & boundary) override;

  nav2_core::SpeedPlanningDiagnostics getLastPlanningDiagnostics() const override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  struct ObstacleConstraintTiming
  {
    double total_ms{0.0};
    double tf_ms{0.0};
    double scan_ms{0.0};
    double apply_ms{0.0};
    bool transform_required{false};
  };

  // ============================================================
  // Phase B hook（派生类必须实现）：在 Phase A 包络之上生成运动学可行的速度剖面。
  // @param work 已完成 Phase A 的工作结构（speed_limit = 包络上界；含障碍零速点）
  // @param curr_vel 当前实际速度（receding-horizon 起点）
  // @param cruise_limit Phase A 求得的巡航上界（可作为剖面上限参考）
  // ============================================================
  virtual void assignVelocityProfile(
    std::vector<TrajPointWork> & work, double curr_vel, double curr_accel,
    double cruise_limit, bool terminal_brake_latched) = 0;

  // ============================================================
  // Phase A 约束包络（共享，逐点 min 叠加；安全置零绝对优先，任何后续写入不得抬升）
  // ============================================================

  /// A1 障碍约束：首个障碍点及其后全部置零，障碍前 collision_stop_distance_ 缓冲带置零
  /// （index 0 为机器人当前位置，不动）。
  /// @return 见 ObstacleConstraintOutcome——**"没有障碍"与"没法查"是两件事**
  ObstacleConstraintOutcome applyObstacleConstraint(std::vector<TrajPointWork> & path);

  /// A2 理想巡航上界（纯查询）：非零速点限速的最小值，clamp 到 [min_speed_, max_speed_]；
  /// 全部点被安全置零时返回 0。
  double computeCruiseLimit(const std::vector<TrajPointWork> & path) const;

  /// A2 回填：巡航上界与各点现有限速**取严**后写入非零速点（零速点跳过，安全零不回填）。
  /// 取严而非覆写，才符合 Phase A 的"逐点 min 叠加"契约——见 .cpp 内说明。
  void setPlanSpeedLimit(std::vector<TrajPointWork> & path, const double speed_limit);

  /// A3 曲率约束：小半径处限速（只降不升）+ 曲率值写入 + 转弯保持匀速（可选）。
  void applyCurvatureConstraint(std::vector<TrajPointWork> & path, const double ex_speed_limit);

  /// 在 goal 前 stop_distance_ 处插入精确 stop-zone 入口，避免入口依赖路径采样点距。
  void insertEndSpeedBoundary(std::vector<TrajPointWork> & path) const;

  /// A4 末端蠕行区：未来点距终点 stop_distance_ 内包络压到 end_speed_；
  /// index 0 是当前实测边界，不被未来包络覆盖。
  void applyEndSpeedConstraint(std::vector<TrajPointWork> & path);

  // ============================================================
  // 共享工具（T/S 均可用）
  // ============================================================

  /// 路径几何总长。
  static double computePathLength(const std::vector<TrajPointWork> & path);

  /// 累计弧长序列（arc_s[0]=0）。距离一律用弧长——到端点弦长在弯曲路径上
  /// 偏短且可能非单调。
  static std::vector<double> computeCumulativeArcLengths(const std::vector<TrajPointWork> & path);

  /// 三点圆拟合曲率序列（Menger 曲率）。
  std::vector<double> calcTrajectoryCurvatureFrom3Points(
    const std::vector<TrajPointWork> & path, size_t idx_dist);

  /// 三点 Menger 曲率（denominator 过小抛 std::runtime_error）。
  static double calcCurvature(
    const geometry_msgs::msg::Pose & p1,
    const geometry_msgs::msg::Pose & p2,
    const geometry_msgs::msg::Pose & p3);

  /// footprint 代价查询不可用（costmap 缺失 / 位姿或 footprint 越出地图）时的返回值。
  static constexpr double kFootprintCostUnavailable = -1.0;

  /// 机器人在 (x,y,theta) 姿态下的 footprint 覆盖格子最大代价；
  /// 不可用时返回 kFootprintCostUnavailable。
  ///
  /// ⚠️ nav2 的 `FootprintCollisionChecker::footprintCost` 在**任一 footprint 顶点
  /// `worldToMap` 失败时直接返回 LETHAL_OBSTACLE**。local costmap 是 8×8 rolling
  /// （±4m），footprint 半对角约 0.495m，于是路径上超过约 3.5m 的点会因顶点出图被
  /// 判成"障碍"——R47 改成 footprint 查询后引入的幻影障碍。这里先逐顶点校验是否全部
  /// 在图内，出图一律按"无信息、不判障"处理（与 R47 之前单点查询 worldToMap 失败
  /// 即返回 false 的语义一致）。
  double footprintCostAt(double x, double y, double theta) const;

  /// 障碍判定（三态，virtual 供测试注入）：机器人在 (x,y,theta) 姿态下的 footprint
  /// 代价 >= obstacle_cost_threshold_ 即 OBSTACLE；查询超出 costmap 覆盖返回
  /// OUT_OF_RANGE（无信息，不判障）。
  ///
  /// R47：原实现只查路径中心线单点，贴中心线两侧、不压中心线本身的障碍会被完全漏检——
  /// RPP 的 isCollisionImminent 是 footprint 查询，两套检测口径不一致，SpeedPlanner
  /// 因此看不到 RPP 独立捕获到的障碍。现改为与 RPP 同源的 footprint 查询
  /// （阈值、更新频率不变，仅检查几何从单点改为整车）。
  virtual PointObstacleStatus checkObstacleAt(double x, double y, double theta) const;

  /// 障碍约束是否**具备评估条件**（costmap 就绪）。virtual 供测试注入。
  /// 与 checkObstacleAt 的 OUT_OF_RANGE 是两件事：这里为 false 表示整轮没法查
  /// （触发 ConstraintFailurePolicy），OUT_OF_RANGE 只表示某点在覆盖范围外。
  virtual bool isObstacleQueryAvailable() const;


  // —— 成员（protected：派生类可访问）——
  std::string plugin_name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  // footprint 障碍查询（与 RegulatedPurePursuitController::collision_checker_ 同类型，
  // 保证两套检测口径一致）；costmap 指针在 footprintCostAt 每次调用时刷新，因为规划期与
  // configure 期的实际 Costmap2D* 可能不同。
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  collision_checker_;
  rclcpp::Logger logger_{rclcpp::get_logger("SpeedPlannerBase")};
  rclcpp::Clock::SharedPtr clock_;

  // 缓存路径（来自 setPlan）。
  nav_msgs::msg::Path current_plan_;

  // —— 共享参数（包络与运动学上限；T/S 共用）——
  double max_speed_{0.6};
  double min_speed_{0.03};
  double accel_{0.35};
  double decel_{-0.15};
  double curve_compute_dist_{0.1};
  double regulated_linear_scaling_min_radius_{1.2};
  double min_curvature_speed_{0.1};
  double constant_velocity_dist_threshold_{0.1};
  // 曲率约束提前量（O-6）：进弯前该弧长距离即压到该弯的限速（而非物理刚好
  // 够减速的最晚时刻才开始）；出弯后延续该弧长距离才允许恢复。给可能的
  // 跟踪滞后（R22）留安全边际，也让转弯前减速更从容。<=0 关闭对应方向。
  double curve_lead_in_distance_{0.3};
  double curve_lead_out_distance_{0.15};
  bool enable_constant_velocity_while_turning_{false};
  // 路径终点目标速度。>=1e-3：末端 stop_distance_ 内蠕行该速度；
  // <1e-3：无蠕行区，剖面在终点收敛到 0。
  double end_speed_{0.03};
  double stop_distance_{0.2};
  double collision_stop_distance_{0.2};
  // 障碍判定 cost 阈值（cost >= 即视为障碍；NO_INFORMATION 恒不算）。
  // 默认 254=LETHAL：只有 path 点真正落在障碍格内才触发停障——
  // 253 内切区是"机器人中心到此会碰撞"的膨胀带，会把贴障碍的合法路径误判停障。
  int obstacle_cost_threshold_{254};
  // 起点豁免距离：距起点该弧长内的障碍点不触发置零——机器人贴墙停放时自身位置
  // 在障碍内切区（cost>=253），无豁免会把整条剖面置零形成起步死锁；
  // 近身避障由 prepare/控制器障碍代价负责，速度规划只管起步不被自己锁死。
  double obstacle_start_exempt_distance_{0.25};
  // 障碍约束**无法评估**（TF 变换失败 / costmap 缺失）时的处置策略（调研 P0-3）。
  // 生产默认 FAIL_CLOSED：本周期输出空轨迹 → ControllerServer 发零速。
  // ⚠️ 不含"路径点出图"——那是 OUT_OF_RANGE，属正常情况，见 PointObstacleStatus。
  ConstraintFailurePolicy obstacle_unavailable_policy_{ConstraintFailurePolicy::FAIL_CLOSED};

  // 外部限速（setSpeedLimit 保存；computeSpeedPlan 与 path speed_hint 取严）。
  double speed_limit_{0.0};
  bool speed_limit_is_percentage_{false};

  // 运行时缓存。
  double curr_path_length_{0.0};
  double points_interval_{0.1};
  // 当前路径坐标系（computeSpeedPlan 捕获；A1 查询 costmap 前用于对齐到
  // costmap 系——两者通常分别为 map 与 odom，直接查询会差一个定位修正量）。
  std::string path_frame_;
  ObstacleConstraintTiming last_obstacle_timing_;
  nav2_core::SpeedPlanningDiagnostics last_planning_diagnostics_;
};

}  // namespace nav2_speed_planner

#endif  // NAV2_SPEED_PLANNER__SPEED_PLANNER_BASE_HPP_
