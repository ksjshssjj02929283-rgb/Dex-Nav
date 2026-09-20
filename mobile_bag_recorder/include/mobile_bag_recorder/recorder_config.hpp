// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#ifndef MOBILE_BAG_RECORDER__RECORDER_CONFIG_HPP_
#define MOBILE_BAG_RECORDER__RECORDER_CONFIG_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace mobile_bag_recorder
{

/// 常态录制录满 max_segments 之后的收尾策略
enum class ContinuousMode
{
  kRolling,  ///< 滚动覆盖：删最旧的段，永久运行
  kLimited,  ///< 录满即停：达到段数上限后停止录制
};

struct ContinuousConfig
{
  bool enabled{true};
  /// 每段时长（秒），交给 rosbag2 的 max_bagfile_duration 分段
  int segment_duration{300};
  ContinuousMode mode{ContinuousMode::kRolling};
  /// rolling：最多保留的段数；limited：录满即停的段数
  int max_segments{24};
  /// 总容量上限（MB）。段大小随场景浮动，只靠段数控不住盘，故两者同时生效
  int max_total_size_mb{20480};
  std::vector<std::string> topics;
};

struct EventConfig
{
  bool enabled{true};
  /// 触发点之前保留多久（秒）。缓冲按时间戳裁剪，不按字节
  int pre_seconds{60};
  /// 触发之后继续录多久（秒）
  int post_seconds{30};
  /// 冷却期（秒），避免连续失败刷出大量重叠 bag
  int cooldown_seconds{120};
  /// 事件 bag 保留个数，超限删最旧
  int max_events{50};
  /// 事件目录总容量上限（MB），与 max_events 同时生效
  int max_total_size_mb{5120};
  /// 环形缓冲内存硬上限（MB）。时间窗口先到先裁，此项是防 OOM 的兜底：
  /// 数据率突增时宁可让 pre 窗口短于 pre_seconds，也不能把内存吃穿
  int max_buffer_mb{512};
  bool inherit_continuous_topics{true};
  /// 从继承清单里排除的大流量话题——控制环形缓冲内存的主要旋钮
  std::vector<std::string> exclude_topics;
  std::vector<std::string> extra_topics;
};

/// 事件触发时一并留存的系统日志快照（systemd journal）
struct LogCaptureConfig
{
  bool enabled{true};
  /// 触发点之前取多久的日志（秒）。与 event.pre_seconds 独立：日志比 bag 便宜得多
  /// （pc1 实测 5 min ≈ 300~540 KB），窗口可以开得比数据窗口长
  int pre_seconds{300};
  /// journalctl --lines 上限，0 = 不限。日志跟着事件 bag 一起被保留策略回收，
  /// 正常无需设，只在某个节点疯狂刷屏时用来兜底
  int max_lines{0};
  /// 单次采集超时（秒），超时杀掉 journalctl，已写入的部分保留
  int timeout_seconds{30};
  /// journalctl -t 过滤的 syslog identifier（auto_mode.sh 里 systemd-cat -t 的那个名字）。
  /// 留空 = 全量，连 kernel / systemd 一起收——掉电、OOM、网卡抖动在 ROS 侧看不到
  std::vector<std::string> identifiers;
};

struct TriggerConfig
{
  bool on_nav_aborted{true};
  bool on_nav_canceled{false};
  bool on_recovery_behavior{false};
  bool on_diagnostics_error{true};
  /// 终点误差触发阈值（米），<=0 表示不判定
  double goal_error_threshold_m{0.0};

  /// 导航卡住检测：任务进行中（action EXECUTING）且持续无实质推进
  bool on_nav_stall{true};
  /// 判定卡住所需的持续时长（秒）
  double stall_timeout_s{20.0};
  /// 该时长内推进量低于此值（米）即视为卡住
  double stall_min_progress_m{0.2};
  /// 推进量来源：auto / feedback（action 的 distance_remaining）/ odom（机器人位移）。
  /// auto = 有 nav2_msgs 时用 feedback（能识别"在动但没接近目标"），否则回退 odom
  std::string stall_progress_source{"auto"};

  /// 被视为"恢复行为"的行为树节点名，进入 RUNNING 即触发（仅 on_recovery_behavior 生效时）
  std::vector<std::string> recovery_node_names{
    "Spin", "BackUp", "Wait", "ClearEntireCostmap", "ClearCostmapExceptRegion"};

  std::string nav_status_topic{"/navigate_to_pose/_action/status"};
  std::string diagnostics_topic{"/navigation/speed_plan_diagnostics"};
  std::string goal_error_topic{"/navigation_goal_error"};
  std::string behavior_tree_log_topic{"/behavior_tree_log"};
  std::string nav_feedback_topic{"/navigate_to_pose/_action/feedback"};
  std::string odom_topic{"/feedback/odom"};
};

/// yaml → 强类型配置。声明、读取、校验都收在这里，其余各层只读结构体
class RecorderConfig
{
public:
  /// 在 node 上声明全部参数并读取。校验失败的项回落到默认值并告警，不抛异常
  static RecorderConfig declareAndLoad(rclcpp::Node * node);

  /// 事件流最终生效的话题清单（继承常态 - 排除 + 补录，去重）
  std::vector<std::string> resolveEventTopics() const;

  /// 把 "话题:最大Hz" 规则解析成 topic -> 最小间隔（纳秒）。
  /// 格式非法或频率 <=0 的条目跳过并告警，不让一条笔误废掉整份配置。
  static std::unordered_map<std::string, int64_t> parseThrottleRules(
    const std::vector<std::string> & rules, const rclcpp::Logger & logger);

  /// 限速规则原文（"话题:最大Hz"）。**只作用于常态流**——事件流照收全量，
  /// 因为触发即意味着要排查故障，取证需要原始分辨率
  std::vector<std::string> throttle;

  std::string output_dir;
  /// 落盘缓存上限（字节）。>0 时 rosbag2 走 cache + consumer 线程，
  /// 写盘不阻塞订阅回调；设 0 会退化成回调里同步写盘
  int64_t writer_cache_size{100 * 1024 * 1024};
  std::string storage_id{"sqlite3"};
  /// storage_id 不可用时的回退后端。留空表示不回退（启动即失败）。
  /// 回退不是静默降级——会以 ERROR 级别告警说明换了后端及原因
  std::string storage_fallback_id{"sqlite3"};

  ContinuousConfig continuous;
  EventConfig event;
  LogCaptureConfig log_capture;
  TriggerConfig trigger;

private:
  static ContinuousMode parseMode(const std::string & text, const rclcpp::Logger & logger);
};

/// 展开 output_dir 里的 "~"，并保证目录存在。失败返回空串
std::string prepareOutputDir(const std::string & raw_dir, const rclcpp::Logger & logger);

/// 启动自检：在 root 下试开一个临时 bag，确认存储后端可用且目录可写。
///
/// 存在的理由：storage 插件缺失时 rosbag2 只在每次开段时抛异常，节点却能继续
/// "正常"运行——订阅、限速、切段定时器全都在跑，唯独一个字节都没落盘。这种静默
/// 空转比启动失败危险得多，因为等到需要 bag 时才会发现。
///
/// 返回实际可用的 storage_id：优先 preferred，不行则 fallback；都不行返回空串。
std::string resolveStorageId(
  const std::string & preferred, const std::string & fallback,
  const std::string & root, const rclcpp::Logger & logger);

}  // namespace mobile_bag_recorder

#endif  // MOBILE_BAG_RECORDER__RECORDER_CONFIG_HPP_
