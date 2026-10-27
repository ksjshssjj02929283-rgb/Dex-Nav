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

RangeFilter::RangeFilter(
  const int window_size,
  const double max_jump_distance,
  const double low_pass_alpha)
: window_size_(std::max(1, window_size)),
  max_jump_distance_(max_jump_distance),
  low_pass_alpha_(std::clamp(low_pass_alpha, 0.0, 1.0))
{
}

float RangeFilter::update(const float raw_range, bool & is_outlier)
{
  std::lock_guard<std::mutex> lock(mutex_);
  is_outlier = false;

  history_.push_back(raw_range);
  while (static_cast<int>(history_.size()) > window_size_) {
    history_.pop_front();
  }

  // 1. 计算滑动窗口中值，有效滤除单帧或短突发孤立离群值
  std::vector<float> sorted_vals(history_.begin(), history_.end());
  std::nth_element(
    sorted_vals.begin(),
    sorted_vals.begin() + sorted_vals.size() / 2,
    sorted_vals.end());
  const float median_range = sorted_vals[sorted_vals.size() / 2];

  // 2. 突变跳变门限判定
  if (max_jump_distance_ > 0.0 && std::abs(raw_range - median_range) > max_jump_distance_) {
    is_outlier = true;
  }

  // 3. 一阶低通滤波（中值滤波之后平滑高频抖动，延时 1-2 周期）
  //    y[k] = alpha * median_range + (1 - alpha) * y[k-1]
  //    当 alpha = 0.5 时，等效群时延 D = (1 - alpha) / alpha = 1.0 周期（阶跃响应 1-2 周期达到主导平滑）
  float filtered_range = median_range;
  if (low_pass_alpha_ > 0.0 && low_pass_alpha_ < 1.0) {
    if (!has_lpf_state_) {
      last_lpf_range_ = median_range;
      has_lpf_state_ = true;
      filtered_range = median_range;
    } else {
      filtered_range = static_cast<float>(
        low_pass_alpha_ * median_range + (1.0 - low_pass_alpha_) * last_lpf_range_);
      last_lpf_range_ = filtered_range;
    }
  } else {
    last_lpf_range_ = median_range;
    has_lpf_state_ = true;
    filtered_range = median_range;
  }

  return filtered_range;
}

void RangeFilter::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  history_.clear();
  has_lpf_state_ = false;
  last_lpf_range_ = 0.0f;
}

void RangeFilter::setWindowSize(const int window_size)
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

void RangeFilter::setMaxJumpDistance(const double max_jump_distance)
{
  std::lock_guard<std::mutex> lock(mutex_);
  max_jump_distance_ = max_jump_distance;
}

double RangeFilter::getMaxJumpDistance() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return max_jump_distance_;
}

void RangeFilter::setLowPassAlpha(const double low_pass_alpha)
{
  std::lock_guard<std::mutex> lock(mutex_);
  low_pass_alpha_ = std::clamp(low_pass_alpha, 0.0, 1.0);
}

double RangeFilter::getLowPassAlpha() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return low_pass_alpha_;
}

}  // namespace nav2_collision_monitor
