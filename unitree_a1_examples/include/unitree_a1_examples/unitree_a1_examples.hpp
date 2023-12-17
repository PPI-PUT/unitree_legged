// Copyright 2023 Maciej Krupka
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef UNITREE_A1_EXAMPLES__UNITREE_A1_EXAMPLES_HPP_
#define UNITREE_A1_EXAMPLES__UNITREE_A1_EXAMPLES_HPP_

#include <cstdint>

#include "unitree_a1_examples/visibility_control.hpp"
#include "unitree_a1_legged_msgs/msg/low_cmd.hpp"
#include "unitree_a1_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/quadruped.h"
#include "unitree_legged_sdk/a1_const.h"
#include "unitree_legged_sdk/comm.h"

#include "std_msgs/msg/float64.hpp"

namespace unitree_a1_examples
{

class UNITREE_A1_EXAMPLES_PUBLIC UnitreeA1Examples
{
public:
  UnitreeA1Examples();
  double jointLinearInterpolation(double initPos, double targetPos, double rate);
};

}  // namespace unitree_a1_examples

#endif  // UNITREE_A1_EXAMPLES__UNITREE_A1_EXAMPLES_HPP_
