// Copyright 2023 Maciej Krupka
// Perception for Physical Interaction Laboratory at Poznan University of Technology
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

#include "unitree_a1_joystick/unitree_a1_joystick.hpp"
#include "rclcpp/rclcpp.hpp"

namespace unitree_a1_legged
{
using Time = rclcpp::Time;
using Duration = rclcpp::Duration;
class UNITREE_A1_JOYSTICK_PUBLIC Button
{
public:
  Button();
  Button(int32_t deadzone_sec, uint32_t deadzone_nsec);
  Button(
    int32_t deadzone_sec, uint32_t deadzone_nsec, double increment, double precision,
    double min, double max);
  bool getAction(const bool button_pressed, const Time & time_pressed, const double alpha);
  double getValue() const;

private:
  double value_{0.0};
  double increment_;
  double precision_;
  double max_value_;
  double min_value_;
  Duration hold_duration_ = Duration(0, 0);
  Duration deadzone_ = Duration(0, 0);
  Time last_time_pressed_;
};

}  // namespace unitree_a1_legged