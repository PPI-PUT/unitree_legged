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

#include "unitree_a1_joystick/unitree_a1_button.hpp"

namespace unitree_a1_legged
{

Button::Button()
{
}
Button::Button(int32_t deadzone_sec, uint32_t deadzone_nsec)
: increment_(0.0), precision_(0.0), max_value_{0.0}, min_value_{0.0}
{
  deadzone_ = Duration(deadzone_sec, deadzone_nsec);
}
Button::Button(
  int32_t deadzone_sec, uint32_t deadzone_nsec, double increment, double precision,
  double min, double max)
: increment_(increment), precision_(precision), max_value_{max}, min_value_{min}
{
  deadzone_ = Duration(deadzone_sec, deadzone_nsec);
}

double Button::getValue() const
{
  return value_;
}

bool Button::getAction(const bool button_pressed, const Time & time_pressed, const double alpha)
{
  if (button_pressed) {
    hold_duration_ = hold_duration_ + (time_pressed - last_time_pressed_);
  } else {
    hold_duration_ = Duration(0, 0);
  }
  last_time_pressed_ = time_pressed;
  if (hold_duration_ > deadzone_) {
    value_ = value_ + alpha * increment_;
    value_ = std::clamp(value_, min_value_, max_value_);
    value_ = std::round(value_ / precision_) * precision_;
    hold_duration_ = Duration(0, 0);
    return true;
  }
  return false;
}


} // namespace unitree_a1_legged