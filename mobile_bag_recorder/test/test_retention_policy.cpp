// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "mobile_bag_recorder/retention_policy.hpp"

namespace fs = std::filesystem;
using mobile_bag_recorder::RetentionPolicy;

namespace
{

constexpr int64_t kMb = 1024 * 1024;

class RetentionPolicyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    root_ = fs::temp_directory_path() /
      ("mobile_bag_recorder_test_" +
      std::to_string(::testing::UnitTest::GetInstance()->random_seed()) + "_" +
      ::testing::UnitTest::GetInstance()->current_test_info()->name());
    fs::remove_all(root_);
    fs::create_directories(root_);
  }

  void TearDown() override
  {
    fs::remove_all(root_);
  }

  /// 造一个假段目录，age_index 越大表示越新
  void makeSegment(const std::string & name, int age_index, size_t bytes = 1024)
  {
    const auto dir = root_ / name;
    fs::create_directories(dir);
    std::ofstream out(dir / "data.db3", std::ios::binary);
    out << std::string(bytes, 'x');
    out.close();

    // mtime 显式设置，避免依赖创建顺序与文件系统时间精度
    const auto base = fs::file_time_type::clock::now() - std::chrono::hours(24);
    fs::last_write_time(dir, base + std::chrono::minutes(age_index));
  }

  std::vector<std::string> remainingNames() const
  {
    std::vector<std::string> names;
    for (const auto & item : fs::directory_iterator(root_)) {
      if (item.is_directory()) {
        names.push_back(item.path().filename().string());
      }
    }
    std::sort(names.begin(), names.end());
    return names;
  }

  RetentionPolicy makePolicy(int max_entries, int64_t max_bytes)
  {
    return RetentionPolicy(
      rclcpp::get_logger("retention_test"), root_.string(), max_entries, max_bytes);
  }

  fs::path root_;
};

TEST_F(RetentionPolicyTest, EmptyRootRemovesNothing)
{
  auto policy = makePolicy(2, 100 * kMb);
  EXPECT_EQ(policy.enforce(), 0);
  EXPECT_EQ(policy.countEntries(), 0);
}

TEST_F(RetentionPolicyTest, KeepsExactlyMaxEntriesAndDropsOldest)
{
  makeSegment("seg-a", 1);
  makeSegment("seg-b", 2);
  makeSegment("seg-c", 3);
  makeSegment("seg-d", 4);

  auto policy = makePolicy(2, 100 * kMb);
  EXPECT_EQ(policy.enforce(), 2);

  // 最旧的两个被回收，语义是"磁盘上最多保留 max_entries 个段"
  const auto remaining = remainingNames();
  ASSERT_EQ(remaining.size(), 2u);
  EXPECT_EQ(remaining[0], "seg-c");
  EXPECT_EQ(remaining[1], "seg-d");
}

TEST_F(RetentionPolicyTest, NeverRemovesNewestEvenWhenLimitIsOne)
{
  makeSegment("seg-only", 1);

  // 上限比现有段数还小，仍必须保住最新的那个——它可能正在被写入
  auto policy = makePolicy(0, 1);
  EXPECT_EQ(policy.enforce(), 0);
  EXPECT_EQ(policy.countEntries(), 1);
}

TEST_F(RetentionPolicyTest, EnforcesTotalSizeLimitWhenCountIsWithinBudget)
{
  // 段数没超（3 <= 10），但总容量超了，仍应回收
  makeSegment("seg-a", 1, 3 * 1024 * 1024);
  makeSegment("seg-b", 2, 3 * 1024 * 1024);
  makeSegment("seg-c", 3, 3 * 1024 * 1024);

  auto policy = makePolicy(10, 7 * kMb);
  EXPECT_EQ(policy.enforce(), 1);

  const auto remaining = remainingNames();
  ASSERT_EQ(remaining.size(), 2u);
  EXPECT_EQ(remaining[0], "seg-b");
  EXPECT_EQ(remaining[1], "seg-c");
}

TEST_F(RetentionPolicyTest, ReportsTotalBytesAcrossSegments)
{
  makeSegment("seg-a", 1, 2 * 1024 * 1024);
  makeSegment("seg-b", 2, 2 * 1024 * 1024);

  auto policy = makePolicy(10, 100 * kMb);
  EXPECT_GE(policy.totalBytes(), 4 * kMb);
  EXPECT_EQ(policy.enforce(), 0);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
