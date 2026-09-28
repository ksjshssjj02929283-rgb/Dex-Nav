// Copyright 2026 DEXE
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_CONTROLLER__SNAPSHOT_VALIDATION_HPP_
#define NAV2_CONTROLLER__SNAPSHOT_VALIDATION_HPP_
#include <cmath>
#include <cstdint>
namespace nav2_controller
{
namespace snapshot
{
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
