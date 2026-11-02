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

#ifndef NAV2_COLLISION_MONITOR__RANGE_FILTER_HPP_
#define NAV2_COLLISION_MONITOR__RANGE_FILTER_HPP_

#include <deque>
#include <mutex>
#include <vector>
#include <algorithm>
#include <cmath>

namespace nav2_collision_monitor
{

/**
 * @brief 离群点与平滑滤波器：基于滑动窗口中值滤波、突变跳变抑制与一阶低通滤波，
 * 用于过滤超声波探头偶发的假回波、离群噪点与脉冲尖峰跳变，并平滑高频毛刺（延时 1-2 周期）。
 */
class RangeFilter
{
public:
  /**
   * @brief 构造函数
   * @param window_size 滑动窗口大小（默认 5）
   * @param max_jump_distance 相邻帧突变跳变门限（米，默认 0.30m，0.0 表示不限制）
   * @param low_pass_alpha 一阶低通滤波系数（0.0 < alpha <= 1.0，默认 0.5，对应 1-2 周期群延迟）
   */
  explicit RangeFilter(
    const int window_size = 5,
    const double max_jump_distance = 0.30,
    const double low_pass_alpha = 0.5);

  ~RangeFilter() = default;

  /**
   * @brief 更新并滤波当前测距值
   * @param raw_range 传感器原始测距值
   * @param is_outlier 输出参数，当前帧是否被识别为突变离群点
   * @return 中值滤波与低通滤波后的距离值
   */
  float update(const float raw_range, bool & is_outlier);

  /**
   * @brief 重置滤波器状态（如当障碍物脱离视野或传感器超时时）
   */
  void reset();

  void setWindowSize(const int window_size);
  int getWindowSize() const;

  void setMaxJumpDistance(const double max_jump_distance);
  double getMaxJumpDistance() const;

  void setLowPassAlpha(const double low_pass_alpha);
  double getLowPassAlpha() const;

private:
  mutable std::mutex mutex_;
  int window_size_{5};
  double max_jump_distance_{0.30};
  double low_pass_alpha_{0.5};
  std::deque<float> history_;
  bool has_lpf_state_{false};
  float last_lpf_range_{0.0f};
};

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__RANGE_FILTER_HPP_
