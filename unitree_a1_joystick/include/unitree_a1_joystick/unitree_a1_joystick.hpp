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

#ifndef UNITREE_A1_JOYSTICK__UNITREE_A1_JOYSTICK_HPP_
#define UNITREE_A1_JOYSTICK__UNITREE_A1_JOYSTICK_HPP_

#include <cstdint>

#include "unitree_a1_joystick/visibility_control.hpp"

#include <sensor_msgs/msg/joy.hpp>
#include <algorithm>
namespace unitree_a1_joystick
{

class UNITREE_A1_JOYSTICK_PUBLIC UnitreeA1Joystick
{
public:
  UnitreeA1Joystick(const sensor_msgs::msg::Joy & j) : j_(j) {}
  float linear_x() {return ly();}
  float linear_y() {return lx();}
  float angular_z() {return rx();}

private:
  const sensor_msgs::msg::Joy j_;

  float lx() {return j_.axes.at(0);}
  float rx() {return j_.axes.at(1);}
  float ry() {return j_.axes.at(2);}
  float ly() {return j_.axes.at(3);}
  float L2_axes() {return j_.axes.at(4);}

  bool R1() {return j_.buttons.at(0);}
  bool L1() {return j_.buttons.at(1);}
  bool start() {return j_.buttons.at(2);}
  bool select() {return j_.buttons.at(3);}
  bool R2() {return j_.buttons.at(4);}
  bool L2() {return j_.buttons.at(5);}
  bool F1() {return j_.buttons.at(6);}
  bool F2() {return j_.buttons.at(7);}
  bool A() {return j_.buttons.at(8);}
  bool B() {return j_.buttons.at(9);}
  bool X() {return j_.buttons.at(10);}
  bool Y() {return j_.buttons.at(11);}
  bool up() {return j_.buttons.at(12);}
  bool right() {return j_.buttons.at(13);}
  bool down() {return j_.buttons.at(14);}
  bool left() {return j_.buttons.at(15);}
};

}  // namespace unitree_a1_joystick

#endif  // UNITREE_A1_JOYSTICK__UNITREE_A1_JOYSTICK_HPP_
