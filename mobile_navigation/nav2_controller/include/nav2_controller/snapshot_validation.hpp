// Copyright 2026 DEXE
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_CONTROLLER__SNAPSHOT_VALIDATION_HPP_
#define NAV2_CONTROLLER__SNAPSHOT_VALIDATION_HPP_
#include <cmath>
#include <cstdint>
#include <vector>
namespace nav2_controller
{
namespace snapshot
{
inline bool validScan(
  const std::vector<float> & ranges, double minimum, double maximum,
  double angle_increment, bool has_frame, int64_t stamp, int64_t previous_stamp,
  double age, double timeout)
{
  if (!has_frame || ranges.empty() || !std::isfinite(minimum) || !std::isfinite(maximum) ||
    minimum < 0 || maximum <= minimum || !std::isfinite(angle_increment) ||
    angle_increment == 0 || stamp <= 0 || stamp <= previous_stamp ||
    !std::isfinite(age) || age < -0.05 || age > timeout) {return false;}
  for (float range : ranges) {
    if ((std::isinf(range) && range > 0) ||
      (std::isfinite(range) && range >= minimum && range <= maximum)) {return true;}
  }
  return false;
}
inline bool usableResult(
  uint64_t session, uint64_t revision, uint64_t expected_session,
  uint64_t expected_revision, bool success, double age, double max_age)
{
  return session == expected_session && revision == expected_revision && success &&
    std::isfinite(age) && age >= 0 && age <= max_age;
}
}  // namespace snapshot
}  // namespace nav2_controller
#endif
