// Copyright (c) 2026 Dexforce
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_collision_monitor/range_filter.hpp"

namespace nav2_collision_monitor
{

RangeFilter::RangeFilter(int window_size, double max_jump_distance)
: window_size_(std::max(1, window_size)),
  max_jump_distance_(max_jump_distance)
{
}

float RangeFilter::update(float raw_range, bool & is_outlier)
{
  std::lock_guard<std::mutex> lock(mutex_);
  is_outlier = false;

  history_.push_back(raw_range);
  while (static_cast<int>(history_.size()) > window_size_) {
    history_.pop_front();
  }

  // 计算滑动窗口中值，有效滤除单帧或短突发孤立离群值
  std::vector<float> sorted_vals(history_.begin(), history_.end());
  std::nth_element(
    sorted_vals.begin(),
    sorted_vals.begin() + sorted_vals.size() / 2,
    sorted_vals.end());
  const float median_range = sorted_vals[sorted_vals.size() / 2];

  // 突变跳变门限判定
  if (max_jump_distance_ > 0.0 && std::abs(raw_range - median_range) > max_jump_distance_) {
    is_outlier = true;
  }

  return median_range;
}

void RangeFilter::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  history_.clear();
}

void RangeFilter::setWindowSize(int window_size)
{
  std::lock_guard<std::mutex> lock(mutex_);
  window_size_ = std::max(1, window_size);
  while (static_cast<int>(history_.size()) > window_size_) {
    history_.pop_front();
  }
}

int RangeFilter::getWindowSize() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return window_size_;
}

void RangeFilter::setMaxJumpDistance(double max_jump_distance)
{
  std::lock_guard<std::mutex> lock(mutex_);
  max_jump_distance_ = max_jump_distance;
}

double RangeFilter::getMaxJumpDistance() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return max_jump_distance_;
}

}  // namespace nav2_collision_monitor
