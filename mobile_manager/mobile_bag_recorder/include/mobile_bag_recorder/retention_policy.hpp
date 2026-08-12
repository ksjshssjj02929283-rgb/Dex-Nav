// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#ifndef MOBILE_BAG_RECORDER__RETENTION_POLICY_HPP_
#define MOBILE_BAG_RECORDER__RETENTION_POLICY_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace mobile_bag_recorder
{

/// 目录级保留策略：把某个根目录下的 bag 段数与总容量约束在上限内。
///
/// 判定依据是**磁盘上实际存在的段目录**，不是内存计数器——节点重启后旧段仍受管理，
/// 否则每次重启都从零计数，磁盘照样涨满。
class RetentionPolicy
{
public:
  RetentionPolicy(
    rclcpp::Logger logger, std::string root_dir, int max_entries, int64_t max_total_bytes);

  /// 按 mtime 从旧到新删除，直到段数与总容量都不超上限。返回被删除的段数
  int enforce();

  /// 当前根目录下的段数
  int countEntries() const;

  /// 当前根目录总占用（字节）
  int64_t totalBytes() const;

private:
  struct Entry
  {
    std::string path;
    int64_t bytes;
    int64_t mtime;
  };

  std::vector<Entry> scan() const;
  static int64_t directorySize(const std::string & path);

  rclcpp::Logger logger_;
  std::string root_dir_;
  int max_entries_;
  int64_t max_total_bytes_;
};

}  // namespace mobile_bag_recorder

#endif  // MOBILE_BAG_RECORDER__RETENTION_POLICY_HPP_
