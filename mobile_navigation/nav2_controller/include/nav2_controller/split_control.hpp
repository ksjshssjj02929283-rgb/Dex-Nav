// Copyright (c) 2026 nav2_controller contributors
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

#ifndef NAV2_CONTROLLER__SPLIT_CONTROL_HPP_
#define NAV2_CONTROLLER__SPLIT_CONTROL_HPP_

#include "nav2_msgs/action/follow_path.hpp"

namespace nav2_controller
{
namespace detail
{

// Frequencies are resolved once per FollowPath goal. A zero field inherits its
// fallback, so clients that do not specify split rates remain well-defined.
struct SplitControlRates
{
  double control_hz;
  double planning_hz;
  double max_plan_age;

  static SplitControlRates fromGoal(
    const nav2_msgs::action::FollowPath::Goal & goal, double default_control_hz);
};

}  // namespace detail
}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__SPLIT_CONTROL_HPP_
