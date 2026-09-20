// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include "mobile_bag_recorder/retention_policy.hpp"

#include <algorithm>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

namespace mobile_bag_recorder
{

RetentionPolicy::RetentionPolicy(
  rclcpp::Logger logger, std::string root_dir, int max_entries, int64_t max_total_bytes)
: logger_(std::move(logger)),
  root_dir_(std::move(root_dir)),
  max_entries_(max_entries),
  max_total_bytes_(max_total_bytes)
{
}

int64_t RetentionPolicy::directorySize(const std::string & path)
{
  std::error_code ec;
  int64_t total = 0;
  for (fs::recursive_directory_iterator it(path, ec), end; it != end; it.increment(ec)) {
    if (ec) {
      break;
    }
    if (it->is_regular_file(ec)) {
      const auto size = it->file_size(ec);
      if (!ec) {
        total += static_cast<int64_t>(size);
      }
    }
  }
  return total;
}

std::vector<RetentionPolicy::Entry> RetentionPolicy::scan() const
{
  std::vector<Entry> entries;
  std::error_code ec;
  if (!fs::exists(root_dir_, ec)) {
    return entries;
  }

  for (const auto & item : fs::directory_iterator(root_dir_, ec)) {
    if (ec) {
      break;
    }
    if (!item.is_directory(ec)) {
      continue;
    }
    const auto write_time = fs::last_write_time(item.path(), ec);
    if (ec) {
      continue;
    }
    entries.push_back(
      Entry{
        item.path().string(),
        directorySize(item.path().string()),
        write_time.time_since_epoch().count()});
  }

  std::sort(
    entries.begin(), entries.end(),
    [](const Entry & a, const Entry & b) {return a.mtime < b.mtime;});
  return entries;
}

int RetentionPolicy::countEntries() const
{
  return static_cast<int>(scan().size());
}

int64_t RetentionPolicy::totalBytes() const
{
  int64_t total = 0;
  for (const auto & entry : scan()) {
    total += entry.bytes;
  }
  return total;
}

int RetentionPolicy::enforce()
{
  auto entries = scan();
  int64_t total = 0;
  for (const auto & entry : entries) {
    total += entry.bytes;
  }

  int removed = 0;
  size_t index = 0;
  // 最后一个是当前正在写的段，永远不删——只回收已经关闭的历史段
  while (index + 1 < entries.size()) {
    const bool over_count = static_cast<int>(entries.size() - index) > max_entries_;
    const bool over_size = total > max_total_bytes_;
    if (!over_count && !over_size) {
      break;
    }

    const auto & victim = entries[index];
    std::error_code ec;
    const auto count = fs::remove_all(victim.path, ec);
    if (ec) {
      RCLCPP_WARN(
        logger_, "Failed to remove old segment: %s (%s)",
        victim.path.c_str(), ec.message().c_str());
      ++index;
      continue;
    }

    total -= victim.bytes;
    ++removed;
    RCLCPP_INFO(
      logger_, "Reclaimed old segment %s (%ld files, %.1f MB, reason: %s)",
      victim.path.c_str(), static_cast<long>(count),
      static_cast<double>(victim.bytes) / (1024.0 * 1024.0),
      over_count ? "segment count limit" : "total size limit");
    ++index;
  }

  return removed;
}

}  // namespace mobile_bag_recorder
