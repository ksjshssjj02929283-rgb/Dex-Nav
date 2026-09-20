// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <chrono>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "mobile_bag_recorder/log_collector.hpp"

using mobile_bag_recorder::LogCaptureConfig;
using mobile_bag_recorder::LogCollector;

namespace
{

bool contains(const std::vector<std::string> & args, const std::string & value)
{
  return std::find(args.begin(), args.end(), value) != args.end();
}

/// 返回 flag 后面紧跟的那个值，找不到返回空串
std::string valueAfter(const std::vector<std::string> & args, const std::string & flag)
{
  const auto it = std::find(args.begin(), args.end(), flag);
  if (it == args.end() || it + 1 == args.end()) {
    return {};
  }
  return *(it + 1);
}

}  // namespace

TEST(LogCollectorTest, DefaultArgsAskForPreciseTimestampsAndNoPager)
{
  LogCaptureConfig config;
  const auto args = LogCollector::buildJournalctlArgs(config, std::chrono::system_clock::now());

  ASSERT_FALSE(args.empty());
  EXPECT_EQ(args.front(), "journalctl");
  EXPECT_TRUE(contains(args, "--no-pager"));
  // 默认的 short 只到秒，对不上 bag 里的消息时间戳
  EXPECT_EQ(valueAfter(args, "-o"), "short-precise");
  // 不带 --until：不带它就是"截至当前"，右端天然盖住 post 段
  EXPECT_FALSE(contains(args, "--until"));
}

TEST(LogCollectorTest, SinceIsFormattedAsLocalWallClock)
{
  LogCaptureConfig config;
  const auto now = std::chrono::system_clock::now();
  const auto args = LogCollector::buildJournalctlArgs(config, now);

  const auto since = valueAfter(args, "--since");
  EXPECT_EQ(since, LogCollector::formatTime(now));
  // journalctl 认的字面量形如 "2026-09-12 14:57:21"
  EXPECT_EQ(since.size(), 19u);
  EXPECT_EQ(since[4], '-');
  EXPECT_EQ(since[10], ' ');
  EXPECT_EQ(since[13], ':');
}

TEST(LogCollectorTest, OptionalFiltersAreOmittedWhenUnset)
{
  LogCaptureConfig config;
  config.max_lines = 0;
  config.identifiers.clear();
  const auto args = LogCollector::buildJournalctlArgs(config, std::chrono::system_clock::now());

  EXPECT_FALSE(contains(args, "--lines"));
  EXPECT_FALSE(contains(args, "-t"));
}

TEST(LogCollectorTest, IdentifiersBecomeRepeatedDashT)
{
  LogCaptureConfig config;
  config.max_lines = 5000;
  config.identifiers = {"dexe_path_follower", "dexe_schedule_planner"};
  const auto args = LogCollector::buildJournalctlArgs(config, std::chrono::system_clock::now());

  EXPECT_EQ(valueAfter(args, "--lines"), "5000");
  EXPECT_EQ(std::count(args.begin(), args.end(), std::string("-t")), 2);
  EXPECT_TRUE(contains(args, "dexe_path_follower"));
  EXPECT_TRUE(contains(args, "dexe_schedule_planner"));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
