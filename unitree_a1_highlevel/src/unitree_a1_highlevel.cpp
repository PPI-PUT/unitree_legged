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

#include "unitree_a1_highlevel/unitree_a1_highlevel.hpp"

#include <iostream>

namespace unitree_a1_highlevel
{

UnitreeStateMachine::UnitreeStateMachine()
{
  state_ = State::UNKNOWN;
}

void UnitreeStateMachine::nextState()
{
  state_ = static_cast<State>(static_cast<int>(state_) + 1);
}
State UnitreeStateMachine::getState() const
{
  return state_;
}
void UnitreeStateMachine::setState(State state)
{
  state_ = state;
}
uint8_t UnitreeStateMachine::getControllerType() const
{
  switch (state_)
  {
  case State::UNKNOWN:
    return unitree_a1_legged_msgs::msg::ControllerType::UNKNOWN;
  case State::STOP:
    return unitree_a1_legged_msgs::msg::ControllerType::STOP;
  case State::STAND:
    return unitree_a1_legged_msgs::msg::ControllerType::FIXED_STAND;
  case State::HOLD:
    return unitree_a1_legged_msgs::msg::ControllerType::HOLD;
  case State::WALK:
    return unitree_a1_legged_msgs::msg::ControllerType::NEURAL;
  default:
    return unitree_a1_legged_msgs::msg::ControllerType::ZERO;
  }
}

}  // namespace unitree_a1_highlevel
