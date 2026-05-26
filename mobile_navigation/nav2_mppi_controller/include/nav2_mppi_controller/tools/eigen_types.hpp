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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__EIGEN_TYPES_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__EIGEN_TYPES_HPP_

#include <Eigen/Core>

namespace mppi::models
{

using Vector = Eigen::ArrayXf;
using Matrix = Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__EIGEN_TYPES_HPP_
