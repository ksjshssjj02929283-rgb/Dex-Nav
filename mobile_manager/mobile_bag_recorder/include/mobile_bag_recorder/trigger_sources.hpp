// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#ifndef MOBILE_BAG_RECORDER__TRIGGER_SOURCES_HPP_
#define MOBILE_BAG_RECORDER__TRIGGER_SOURCES_HPP_

#include <memory>

#include "mobile_bag_recorder/trigger_source.hpp"

namespace mobile_bag_recorder
{

// 内建触发源工厂。新增一个源：加一个实现文件 + 在此声明 + 在 builtin.cpp 登记一行
std::unique_ptr<TriggerSource> makeNavStatusSource();
std::unique_ptr<TriggerSource> makeDiagnosticsSource();
std::unique_ptr<TriggerSource> makeGoalErrorSource();
std::unique_ptr<TriggerSource> makeNavStallSource();
std::unique_ptr<TriggerSource> makeRecoveryBehaviorSource();

}  // namespace mobile_bag_recorder

#endif  // MOBILE_BAG_RECORDER__TRIGGER_SOURCES_HPP_
