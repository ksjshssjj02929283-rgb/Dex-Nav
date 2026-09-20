// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#ifndef MOBILE_BAG_RECORDER__TRIGGER_SOURCE_HPP_
#define MOBILE_BAG_RECORDER__TRIGGER_SOURCE_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "mobile_bag_recorder/recorder_config.hpp"

namespace mobile_bag_recorder
{

/// 触发源向录制器上报的回调。reason 会写进事件 bag 的目录名
using TriggerCallback = std::function<void (const std::string & reason)>;

/// 触发源的运行上下文
struct TriggerContext
{
  rclcpp::Node * node;
  TriggerConfig config;
  TriggerCallback fire;
};

/// 一个自动触发源。
///
/// 新增触发源 = 加一个实现文件 + 在 createBuiltinTriggerSources() 里加一行，
/// 不改调度器、录制器和节点装配。
///
/// 约定：判定一律取**边沿**（异常出现的那一刻），持续异常不重复上报——
/// 冷却与并发抑制虽然在录制器侧兜底，但那是最后一道，不是让触发源偷懒的理由。
class TriggerSource
{
public:
  virtual ~TriggerSource() = default;

  /// 触发源名称，用于日志
  virtual std::string name() const = 0;

  /// 该源在当前配置下是否启用
  virtual bool isEnabled(const TriggerConfig & config) const = 0;

  /// 建立订阅。仅在 isEnabled 为真时调用
  virtual void setup(const TriggerContext & context) = 0;

  /// 启用但因缺少可选依赖等原因无法工作时返回原因，否则返回空串
  virtual std::string unavailableReason() const {return {};}
};

/// 内建触发源清单。新增源在此登记
std::vector<std::unique_ptr<TriggerSource>> createBuiltinTriggerSources();

}  // namespace mobile_bag_recorder

#endif  // MOBILE_BAG_RECORDER__TRIGGER_SOURCE_HPP_
