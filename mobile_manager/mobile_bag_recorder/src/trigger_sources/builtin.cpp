// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include <memory>
#include <utility>
#include <vector>

#include "mobile_bag_recorder/trigger_sources.hpp"

namespace mobile_bag_recorder
{

/// 内建触发源登记处。
///
/// 新增一个触发源只需三步，核心代码（调度器 / 录制器 / 节点装配）一行不动：
///   1. 加 src/trigger_sources/<name>_source.cpp，实现 TriggerSource
///   2. 在 trigger_sources.hpp 声明 make<Name>Source()
///   3. 在下面的列表里加一行，并把源文件加进 CMakeLists
std::vector<std::unique_ptr<TriggerSource>> createBuiltinTriggerSources()
{
  std::vector<std::unique_ptr<TriggerSource>> sources;
  sources.push_back(makeNavStatusSource());
  sources.push_back(makeDiagnosticsSource());
  sources.push_back(makeGoalErrorSource());
  sources.push_back(makeNavStallSource());
  sources.push_back(makeRecoveryBehaviorSource());
  return sources;
}

}  // namespace mobile_bag_recorder
