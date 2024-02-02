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

#ifndef UNITREE_A1_STATE_MACHINE__UNITREE_A1_STATE_MACHINE_HPP_
#define UNITREE_A1_STATE_MACHINE__UNITREE_A1_STATE_MACHINE_HPP_

#include <cstdint>

#include "unitree_a1_state_machine/visibility_control.hpp"
#include <unitree_a1_legged_msgs/srv/gait.hpp>
#include <unitree_a1_legged_msgs/action/fixed_stand.hpp>
#include <unitree_a1_legged_msgs/msg/low_cmd.hpp>
namespace unitree_a1_state_machine
{
enum class State
{
  UNKNOWN,
  STOP,
  STAND,
  HOLD,
  WALK
};

class UNITREE_A1_STATE_MACHINE_PUBLIC UnitreeStateMachine
{
public:
  UnitreeStateMachine();
  void nextState();
  State getState() const;
  void setState(State state);

private:
  State state_;
};

}  // namespace unitree_a1_state_machine

#endif  // UNITREE_A1_STATE_MACHINE__UNITREE_A1_STATE_MACHINE_HPP_
