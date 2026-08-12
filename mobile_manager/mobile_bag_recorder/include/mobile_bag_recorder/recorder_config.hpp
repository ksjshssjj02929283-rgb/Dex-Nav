// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#ifndef MOBILE_BAG_RECORDER__RECORDER_CONFIG_HPP_
#define MOBILE_BAG_RECORDER__RECORDER_CONFIG_HPP_

#include <string>
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
  bool inherit_continuous_topics{true};
  /// 从继承清单里排除的大流量话题——控制环形缓冲内存的主要旋钮
  std::vector<std::string> exclude_topics;
  std::vector<std::string> extra_topics;
};

struct TriggerConfig
{
  bool on_nav_aborted{true};
  bool on_nav_canceled{false};
  bool on_recovery_behavior{false};
  bool on_diagnostics_error{true};
  /// 终点误差触发阈值（米），<=0 表示不判定
  double goal_error_threshold_m{0.0};
};

/// yaml → 强类型配置。声明、读取、校验都收在这里，其余各层只读结构体
class RecorderConfig
{
public:
  /// 在 node 上声明全部参数并读取。校验失败的项回落到默认值并告警，不抛异常
  static RecorderConfig declareAndLoad(rclcpp::Node * node);

  /// 事件流最终生效的话题清单（继承常态 - 排除 + 补录，去重）
  std::vector<std::string> resolveEventTopics() const;

  std::string output_dir;
  /// 落盘缓存上限（字节）。>0 时 rosbag2 走 cache + consumer 线程，
  /// 写盘不阻塞订阅回调；设 0 会退化成回调里同步写盘
  int64_t writer_cache_size{100 * 1024 * 1024};
  std::string storage_id{"sqlite3"};

  ContinuousConfig continuous;
  EventConfig event;
  TriggerConfig trigger;

private:
  static ContinuousMode parseMode(const std::string & text, const rclcpp::Logger & logger);
};

/// 展开 output_dir 里的 "~"，并保证目录存在。失败返回空串
std::string prepareOutputDir(const std::string & raw_dir, const rclcpp::Logger & logger);

}  // namespace mobile_bag_recorder

#endif  // MOBILE_BAG_RECORDER__RECORDER_CONFIG_HPP_
